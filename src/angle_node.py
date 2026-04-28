#!/usr/bin/env python3
"""
巡线角度检测节点。

功能概述：
1. 订阅相机图像话题
2. 可选应用相机标定结果，对图像做去畸变
3. 将图像转换成适合滑窗搜索的二值候选图
   - 默认使用 CLAHE 局部增强 + 自适应阈值 + 形态学
   - 也可以通过参数切回“边缘 + 线结构”或旧的灰度阈值法
4. 使用滑动窗口在图像底部到顶部逐段搜索线条中心
5. 对搜索到的中心点做直线拟合
6. 计算线条相对车体“竖直向前方向”的偏转角
7. 计算线条底部交点相对图像中心的横向偏移
8. 发布检测角度 `/line_follow/line_angle_deg`
9. 发布横向偏移 `/line_follow/line_offset_px` 和 `/line_follow/line_offset_norm`
10. 发布调试图像话题，便于远程查看去畸变图、二值图、检测图和角度曲线
"""

import math
import os
import json
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32

try:
    # RDK X5 图像链路常用共享内存消息。开发电脑或普通 ROS 环境可能没有该包，
    # 所以这里做成可选导入，只有 image_msg_type=hbmem 时才强制要求它存在。
    from hbm_img_msgs.msg import HbmMsg1080P
except ImportError:
    HbmMsg1080P = None


def filter_components(
    gray,
    bin_img,
    line_is_white=True,
    min_component_area=0,
    max_component_area=0,
    max_component_width_px=0,
    max_component_width_ratio=0.0,
    component_intensity_limit=-1,
):
    """
    按连通域尺寸、宽度和灰度强度过滤候选对象。

    该函数用于二值图的后处理：先把二值图里的白色区域分成一个个连通域，
    再按照面积、宽度、亮度等规则丢弃明显不像线的区域。
    对 threshold 方法而言，亮度过滤可以区分黑线/白线；对 edge_line 方法而言，
    通常只使用面积和宽度过滤。

    `component_intensity_limit` 为负数时关闭强度过滤。
    黑线模式下保留平均灰度更低的对象，白线模式下保留平均灰度更高的对象。
    """
    # connectedComponentsWithStats 会返回：
    # - labels：每个像素属于哪个连通域
    # - stats：每个连通域的外接框、面积等统计量
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(bin_img, connectivity=8)
    if num_labels <= 1:
        return bin_img

    _, w = bin_img.shape[:2]
    max_width_limit = 0
    # 宽度限制既支持绝对像素，也支持占整幅图宽度的比例；两者同时设置时取更严格的值。
    if float(max_component_width_ratio) > 0.0:
        max_width_limit = int(round(w * float(max_component_width_ratio)))
    if int(max_component_width_px) > 0:
        max_width_limit = (
            min(max_width_limit, int(max_component_width_px))
            if max_width_limit > 0
            else int(max_component_width_px)
        )

    filtered = np.zeros_like(bin_img)
    min_component_area = max(0, int(min_component_area))
    max_component_area = max(0, int(max_component_area))
    intensity_limit = float(component_intensity_limit)

    # label=0 是背景，真正的候选区域从 label=1 开始。
    for label in range(1, num_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        width_px = int(stats[label, cv2.CC_STAT_WIDTH])
        if min_component_area > 0 and area < min_component_area:
            continue
        if max_component_area > 0 and area > max_component_area:
            continue
        if max_width_limit > 0 and width_px > max_width_limit:
            continue

        mask = labels == label
        if intensity_limit >= 0.0:
            # 用原始灰度图的均值判断该连通域是否足够黑/足够白。
            # 这比只看二值图更可靠，因为二值图已经丢掉了亮度信息。
            mean_gray = float(np.mean(gray[mask]))
            if line_is_white:
                if mean_gray < intensity_limit:
                    continue
            elif mean_gray > intensity_limit:
                continue

        filtered[mask] = 255

    return filtered


def _odd_kernel_size(value):
    """返回 OpenCV 滤波可用的正奇数核大小，避免用户传入偶数导致 OpenCV 报错。"""
    k = max(1, int(value))
    return k + 1 if k % 2 == 0 else k


def make_threshold_binary(
    bgr,
    line_is_white=True,
    blur_ksize=5,
    thresh=-1,
    morph_ksize=3,
    min_component_area=0,
    max_component_area=0,
    max_component_width_px=0,
    max_component_width_ratio=0.0,
    component_intensity_limit=-1,
):
    """
    使用灰度阈值将输入 BGR 图像转换为适合巡线检测的二值图像。

    这是旧版检测路径，适合光照比较稳定、线条和地面灰度差异明显的场景。
    输出图中白色像素代表“可能属于线”的区域，黑色像素代表背景。

    参数说明：
    - `line_is_white=True` 表示目标线是白线，背景较暗
    - `line_is_white=False` 表示目标线是黑线，背景较亮
    - `thresh=-1` 时使用 Otsu 自动阈值
    - `morph_ksize` 用于开闭运算，尽量去掉小噪点并填补小孔洞
    """
    # 先转灰度，后续阈值分割只需要亮度信息。
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # 模糊用于降低椒盐噪声和纹理细节对阈值分割的影响。
    k = _odd_kernel_size(blur_ksize)
    gray = cv2.GaussianBlur(gray, (k, k), 0)

    # 二值化阶段将“线”和“非线区域”分开。
    if thresh >= 0:
        # 固定阈值适合光照固定的场地，现场可通过 ros2 param 快速调。
        ttype = cv2.THRESH_BINARY if line_is_white else cv2.THRESH_BINARY_INV
        _, bin_img = cv2.threshold(gray, int(thresh), 255, ttype)
    else:
        # Otsu 自动阈值会根据当前帧直方图自动找分割点，光照变化时更省调参，
        # 但画面里干扰较多时可能被大面积背景或阴影带偏。
        ttype = (
            cv2.THRESH_BINARY | cv2.THRESH_OTSU
            if line_is_white
            else cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
        )
        _, bin_img = cv2.threshold(gray, 0, 255, ttype)

    # 开运算去除散点，闭运算填补线条内部的小断裂。
    mk = max(1, int(morph_ksize))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (mk, mk))
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel, iterations=1)
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel, iterations=1)
    bin_img = filter_components(
        gray,
        bin_img,
        line_is_white=line_is_white,
        min_component_area=min_component_area,
        max_component_area=max_component_area,
        max_component_width_px=max_component_width_px,
        max_component_width_ratio=max_component_width_ratio,
        component_intensity_limit=component_intensity_limit,
    )
    return bin_img


def make_edge_line_binary(
    bgr,
    blur_ksize=5,
    canny_low=40,
    canny_high=120,
    edge_morph_ksize=3,
    edge_dilate_ksize=5,
    hough_threshold=40,
    hough_min_line_length_ratio=0.12,
    hough_max_line_gap=35,
    max_line_angle_from_vertical_deg=65.0,
    line_draw_width=9,
    fallback_to_edge_band=True,
    min_component_area=0,
    max_component_area=0,
    max_component_width_px=0,
    max_component_width_ratio=0.0,
):
    """
    使用“边缘 + 线结构”生成候选线条二值图。

    这是当前默认检测路径。它不直接依赖“线比背景黑/白多少”，而是先找边缘，
    再判断这些边缘是否组成了连续的线段结构，因此对地面亮度变化更稳一些。

    Canny 提供线条边界，HoughLinesP 只保留具有连续线段结构的候选边缘。
    当 Hough 没有稳定线段时，可回退到连接后的边缘带，避免现场光照波动导致完全丢线。

    输出仍然是一张二值图，后面的滑动窗口逻辑不需要知道它来自边缘法还是阈值法。
    """
    # 1. 灰度化和模糊：Canny 只需要单通道亮度图；模糊可以降低地面纹理带来的细碎边缘。
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    k = _odd_kernel_size(blur_ksize)
    gray = cv2.GaussianBlur(gray, (k, k), 0)

    # 2. Canny 边缘检测：
    #    low/high 越低，越容易保留弱边缘，同时噪声也更多；
    #    low/high 越高，噪声更少，但线条边缘弱时可能断掉。
    low = max(0, int(canny_low))
    high = max(low + 1, int(canny_high))
    edges = cv2.Canny(gray, low, high)

    # 3. 形态学闭运算：把短距离断裂的边缘接起来，便于后续滑窗统计。
    morph_k = max(1, int(edge_morph_ksize))
    morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_k, morph_k))
    edge_band = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, morph_kernel, iterations=1)

    # 4. 膨胀：Canny 只输出一像素宽边缘，滑窗直接统计会太稀疏；
    #    膨胀后得到“边缘带”，让后面的直方图和窗口命中更稳定。
    dilate_k = max(1, int(edge_dilate_ksize))
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (dilate_k, dilate_k))
    edge_band = cv2.dilate(edge_band, dilate_kernel, iterations=1)

    # 5. HoughLinesP 查找线段结构：
    #    这里用图像高度的比例作为最短线段长度，分辨率变化时参数不用完全重调。
    h, _ = edge_band.shape[:2]
    min_len = max(10, int(round(h * float(hough_min_line_length_ratio))))
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=max(1, int(hough_threshold)),
        minLineLength=min_len,
        maxLineGap=max(1, int(hough_max_line_gap)),
    )

    line_mask = np.zeros_like(edge_band)
    accepted_count = 0
    max_angle = max(1.0, min(89.0, float(max_line_angle_from_vertical_deg)))
    draw_width = max(1, int(line_draw_width))

    if lines is not None:
        for line in lines.reshape(-1, 4):
            x1, y1, x2, y2 = [int(v) for v in line]
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = math.hypot(dx, dy)
            if length < min_len:
                continue

            # 线条在画面中主要沿“上下方向”延伸。这里计算线段相对竖直方向的夹角，
            # 排除过于水平的边缘，例如地面接缝、阴影边缘、画面底部横向纹理。
            angle_from_vertical = math.degrees(math.atan2(abs(dx), max(abs(dy), 1e-6)))
            if angle_from_vertical > max_angle:
                continue

            # 把通过筛选的线段画成较粗的 mask。后面用它与 edge_band 相交，
            # 只留下“既是边缘，又靠近有效线段结构”的像素。
            cv2.line(line_mask, (x1, y1), (x2, y2), 255, draw_width)
            accepted_count += 1

    if accepted_count > 0:
        # Hough 找到结构线段时，只保留线段附近的边缘带，减少无关边缘干扰。
        bin_img = cv2.bitwise_and(edge_band, cv2.dilate(line_mask, dilate_kernel, iterations=1))
    elif bool(fallback_to_edge_band):
        # 回退策略：如果 Hough 太严格导致没线段，仍然使用连接后的边缘带，
        # 避免整帧直接丢线。现场如果噪声太多，可以把该参数设为 false。
        bin_img = edge_band
    else:
        bin_img = line_mask

    # 最后再做一次闭运算和连通域过滤，清理孤立边缘和过宽干扰块。
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, morph_kernel, iterations=1)
    bin_img = filter_components(
        gray,
        bin_img,
        line_is_white=True,
        min_component_area=min_component_area,
        max_component_area=max_component_area,
        max_component_width_px=max_component_width_px,
        max_component_width_ratio=max_component_width_ratio,
        component_intensity_limit=-1,
    )
    return bin_img


def make_clahe_adaptive_binary(
    bgr,
    line_is_white=True,
    blur_ksize=5,
    morph_ksize=3,
    clahe_clip_limit=3.0,
    clahe_tile_grid_size=8,
    adaptive_block_size=51,
    adaptive_c=5.0,
    min_component_area=0,
    max_component_area=0,
    max_component_width_px=0,
    max_component_width_ratio=0.0,
    component_intensity_limit=-1,
):
    """
    使用 CLAHE + 自适应阈值 + 形态学生成候选线条二值图。

    该方法针对强光、阴影和地面亮度不均的场景：
    1. CLAHE 先做局部对比度增强，让线和附近地面的差异更明显；
    2. adaptiveThreshold 按局部窗口计算阈值，避免一处强光影响整幅图；
    3. 形态学开/闭运算去除散点并连接断裂线条。
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    tile = max(1, int(clahe_tile_grid_size))
    clip_limit = max(0.1, float(clahe_clip_limit))
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tile, tile))
    enhanced = clahe.apply(gray)

    k = _odd_kernel_size(blur_ksize)
    enhanced = cv2.GaussianBlur(enhanced, (k, k), 0)

    block_size = _odd_kernel_size(adaptive_block_size)
    block_size = max(3, block_size)
    threshold_type = cv2.THRESH_BINARY if line_is_white else cv2.THRESH_BINARY_INV
    bin_img = cv2.adaptiveThreshold(
        enhanced,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        threshold_type,
        block_size,
        float(adaptive_c),
    )

    mk = max(1, int(morph_ksize))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (mk, mk))
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel, iterations=1)
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel, iterations=2)
    bin_img = filter_components(
        enhanced,
        bin_img,
        line_is_white=line_is_white,
        min_component_area=min_component_area,
        max_component_area=max_component_area,
        max_component_width_px=max_component_width_px,
        max_component_width_ratio=max_component_width_ratio,
        component_intensity_limit=component_intensity_limit,
    )
    return bin_img


def apply_detection_roi_mask(
    bin_img,
    roi_bottom_offset_ratio=0.0,
    roi_height_ratio=0.5,
    roi_side_margin_ratio=0.05,
    roi_left_margin_ratio=None,
    roi_right_margin_ratio=None,
):
    """
    只保留参与识别的 ROI 区域，ROI 外全部清零。

    这样后续滑窗不会被 ROI 外的强光、地面纹理或远处背景干扰；
    Web 页面里的 Binary/Detect 调试图也只显示真正参与识别的区域。
    """
    h, w = bin_img.shape[:2]
    roi_bottom_offset_ratio = min(0.95, max(0.0, float(roi_bottom_offset_ratio)))
    roi_height_ratio = min(1.0, max(0.05, float(roi_height_ratio)))
    roi_side_margin_ratio = min(0.45, max(0.0, float(roi_side_margin_ratio)))
    left_margin_ratio = (
        roi_side_margin_ratio
        if roi_left_margin_ratio is None
        else min(0.45, max(0.0, float(roi_left_margin_ratio)))
    )
    right_margin_ratio = (
        roi_side_margin_ratio
        if roi_right_margin_ratio is None
        else min(0.45, max(0.0, float(roi_right_margin_ratio)))
    )
    roi_bottom = int(round(h * (1.0 - roi_bottom_offset_ratio)))
    roi_height = max(1, int(round(h * roi_height_ratio)))
    roi_top = max(0, roi_bottom - roi_height)
    roi_bottom = min(h, max(roi_top + 1, roi_bottom))
    roi_left = int(round(w * left_margin_ratio))
    roi_right = max(roi_left + 1, int(round(w * (1.0 - right_margin_ratio))))
    roi_right = min(w, roi_right)

    cropped = np.zeros_like(bin_img)
    cropped[roi_top:roi_bottom, roi_left:roi_right] = bin_img[roi_top:roi_bottom, roi_left:roi_right]
    return cropped


def make_binary(
    bgr,
    detection_method="clahe_adaptive",
    line_is_white=True,
    blur_ksize=5,
    thresh=-1,
    morph_ksize=3,
    min_component_area=0,
    max_component_area=0,
    max_component_width_px=0,
    max_component_width_ratio=0.0,
    component_intensity_limit=-1,
    canny_low=40,
    canny_high=120,
    edge_morph_ksize=3,
    edge_dilate_ksize=5,
    hough_threshold=40,
    hough_min_line_length_ratio=0.12,
    hough_max_line_gap=35,
    max_line_angle_from_vertical_deg=65.0,
    line_draw_width=9,
    fallback_to_edge_band=True,
    clahe_clip_limit=3.0,
    clahe_tile_grid_size=8,
    adaptive_block_size=51,
    adaptive_c=5.0,
):
    """
    根据检测方法生成滑窗输入二值图。

    这个函数是算法入口的“分流器”：
    - clahe_adaptive / adaptive：使用 CLAHE + 自适应阈值 + 形态学
    - edge_line / edge / edges / line_structure：使用边缘 + 线结构检测
    - 其他值：使用旧版灰度阈值检测

    这样可以在现场通过 ROS 参数切换算法，而不需要改代码或重编译。
    """
    method = str(detection_method).strip().lower()
    if method in ("clahe", "adaptive", "clahe_adaptive", "adaptive_threshold"):
        return make_clahe_adaptive_binary(
            bgr,
            line_is_white=line_is_white,
            blur_ksize=blur_ksize,
            morph_ksize=morph_ksize,
            clahe_clip_limit=clahe_clip_limit,
            clahe_tile_grid_size=clahe_tile_grid_size,
            adaptive_block_size=adaptive_block_size,
            adaptive_c=adaptive_c,
            min_component_area=min_component_area,
            max_component_area=max_component_area,
            max_component_width_px=max_component_width_px,
            max_component_width_ratio=max_component_width_ratio,
            component_intensity_limit=component_intensity_limit,
        )

    if method in ("edge", "edge_line", "edges", "line_structure"):
        return make_edge_line_binary(
            bgr,
            blur_ksize=blur_ksize,
            canny_low=canny_low,
            canny_high=canny_high,
            edge_morph_ksize=edge_morph_ksize,
            edge_dilate_ksize=edge_dilate_ksize,
            hough_threshold=hough_threshold,
            hough_min_line_length_ratio=hough_min_line_length_ratio,
            hough_max_line_gap=hough_max_line_gap,
            max_line_angle_from_vertical_deg=max_line_angle_from_vertical_deg,
            line_draw_width=line_draw_width,
            fallback_to_edge_band=fallback_to_edge_band,
            min_component_area=min_component_area,
            max_component_area=max_component_area,
            max_component_width_px=max_component_width_px,
            max_component_width_ratio=max_component_width_ratio,
        )

    return make_threshold_binary(
        bgr,
        line_is_white=line_is_white,
        blur_ksize=blur_ksize,
        thresh=thresh,
        morph_ksize=morph_ksize,
        min_component_area=min_component_area,
        max_component_area=max_component_area,
        max_component_width_px=max_component_width_px,
        max_component_width_ratio=max_component_width_ratio,
        component_intensity_limit=component_intensity_limit,
    )


def detect_line_and_angle(
    bin_img,
    n_windows=9,
    margin=60,
    minpix=50,
    kp=1.0,
    roi_bottom_offset_ratio=0.0,
    roi_height_ratio=0.5,
    roi_side_margin_ratio=0.05,
    roi_left_margin_ratio=None,
    roi_right_margin_ratio=None,
    base_search_half_width_ratio=0.25,
    draw=False,
):
    """
    用滑动窗口跟踪线条并估计其方向角。

    返回值：
    - `ok`：本帧是否成功检测到足够多的线条点
    - `angle_deg`：线条相对车体前向的夹角，右偏为正
    - `steer_deg`：一个基于比例控制的建议转向量
    - `offset_px`：拟合线在图像底边交点相对图像中心的偏移，右偏为正
    - `offset_norm`：归一化横向偏移，范围约为 [-1, 1]
    - `vis`：调试图，用于窗口和拟合线显示
    """
    h, w = bin_img.shape[:2]

    # 默认 ROI 为图像中线往下的四分之一：0.50H 到 0.75H。
    roi_bottom_offset_ratio = min(0.95, max(0.0, float(roi_bottom_offset_ratio)))
    roi_height_ratio = min(1.0, max(0.05, float(roi_height_ratio)))
    roi_side_margin_ratio = min(0.45, max(0.0, float(roi_side_margin_ratio)))
    left_margin_ratio = (
        roi_side_margin_ratio
        if roi_left_margin_ratio is None
        else min(0.45, max(0.0, float(roi_left_margin_ratio)))
    )
    right_margin_ratio = (
        roi_side_margin_ratio
        if roi_right_margin_ratio is None
        else min(0.45, max(0.0, float(roi_right_margin_ratio)))
    )
    roi_bottom = int(round(h * (1.0 - roi_bottom_offset_ratio)))
    roi_height = max(1, int(round(h * roi_height_ratio)))
    roi_top = max(0, roi_bottom - roi_height)
    roi_bottom = min(h, max(roi_top + 1, roi_bottom))
    roi_left = int(round(w * left_margin_ratio))
    roi_right = max(roi_left + 1, int(round(w * (1.0 - right_margin_ratio))))
    roi_right = min(w, roi_right)

    bottom = bin_img[roi_top:roi_bottom, roi_left:roi_right]
    hist_roi = np.sum(bottom > 0, axis=0).astype(np.int32)
    hist = np.zeros(w, dtype=np.int32)
    hist[roi_left:roi_right] = hist_roi
    if hist.size == 0 or int(np.max(hist)) <= 0:
        vis = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR) if draw else None
        if vis is not None:
            cv2.rectangle(vis, (roi_left, roi_top), (roi_right - 1, roi_bottom - 1), (255, 128, 0), 2)
            cv2.putText(
                vis,
                "line not found in detect ROI",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
        return False, 0.0, 0.0, 0.0, 0.0, vis
    image_center_x = 0.5 * float(w - 1)
    base_search_half_width_ratio = min(0.49, max(0.05, float(base_search_half_width_ratio)))
    half_width = max(1, int(round(w * base_search_half_width_ratio)))
    center_idx = int(round(image_center_x))
    center_left = max(0, center_idx - half_width)
    center_right = min(w, center_idx + half_width + 1)
    center_hist = hist[center_left:center_right]
    if center_hist.size > 0 and int(np.max(center_hist)) > 0:
        base_x = int(np.argmax(center_hist)) + center_left
    else:
        base_x = int(np.argmax(hist))

    # 滑动窗口仅在检测 ROI 内从下往上逐窗跟踪线条中心。
    nw = max(3, int(n_windows))
    roi_search_height = max(1, roi_bottom - roi_top)
    win_h = max(1, int(math.ceil(roi_search_height / float(nw))))
    margin = max(10, int(margin))
    minpix = max(10, int(minpix))

    current_x = base_x
    centers = []
    vis = None
    if draw:
        vis = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(vis, (roi_left, roi_top), (roi_right - 1, roi_bottom - 1), (255, 128, 0), 2)

    for i in range(nw):
        # 逐层向上构建窗口区域。
        win_y_high = max(roi_top, roi_bottom - i * win_h)
        win_y_low = max(roi_top, roi_bottom - (i + 1) * win_h)
        win_y_high = min(roi_bottom, win_y_high)
        win_x_low = max(roi_left, current_x - margin)
        win_x_high = min(roi_right, current_x + margin)
        if win_x_high <= win_x_low or win_y_high <= win_y_low:
            continue

        # 在窗口内部寻找非零像素，非零像素就是候选线条区域。
        roi = bin_img[win_y_low:win_y_high, win_x_low:win_x_high]
        nz = cv2.findNonZero(roi)
        if nz is not None and len(nz) >= minpix:
            # 用窗口内像素均值作为该层线条中心，并把它作为下一层窗口的搜索中心。
            pts = nz.reshape(-1, 2)
            cx = int(np.mean(pts[:, 0])) + win_x_low
            cy = int(np.mean(pts[:, 1])) + win_y_low
            centers.append((cx, cy))
            current_x = cx
            if draw:
                cv2.rectangle(vis, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)
                cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)
        elif draw:
            cv2.rectangle(vis, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)

    # 拟合直线至少需要多个点；少于 3 个点时本帧视为“检测失败”。
    if len(centers) < 3:
        return False, 0.0, 0.0, 0.0, 0.0, vis

    # 用 OpenCV 的 fitLine 做最小二乘拟合，得到方向向量 (vx, vy)。
    centers_np = np.array(centers, dtype=np.float32).reshape(-1, 1, 2)
    vx, vy, x0f, y0f = cv2.fitLine(centers_np, cv2.DIST_L2, 0, 0.01, 0.01).flatten()

    # fitLine 返回的方向向量没有朝向约束；若直接算角度，可能同一条线在
    # “向上”和“向下”两个等价方向之间跳变，从而出现接近 180 度的假象。
    # 这里统一约束为“指向画面上方”，再相对竖直向上计算夹角。
    if float(vy) > 0.0:
        vx = -vx
        vy = -vy

    # 相对“竖直向上”计算夹角，并约束为 [-90, 90]。
    # 在本程序中约定：线条向画面右侧倾斜为正角度。
    angle = math.atan2(float(vx), float(-vy))
    angle_deg = angle * 180.0 / math.pi

    # 一个最简单的比例控制建议值，仅用于调试和观察。
    steer_deg = -float(kp) * angle_deg

    # 用拟合线与图像底边的交点估计横向偏移，右偏为正。
    eps = 1e-6
    k = 1e6 if abs(vy) < eps else (vx / vy)
    y_bottom = h - 1
    x_bottom = float(x0f + (y_bottom - y0f) * k)
    offset_px = x_bottom - image_center_x
    offset_norm = 0.0 if image_center_x <= 0.0 else max(-1.0, min(1.0, offset_px / image_center_x))

    if draw and vis is not None:
        y1, y2 = h - 1, 0
        x1 = int(round(x0f + (y1 - y0f) * k))
        x2 = int(round(x0f + (y2 - y0f) * k))
        x_bottom_draw = int(round(max(0.0, min(float(w - 1), x_bottom))))
        center_x_draw = int(round(image_center_x))
        cv2.line(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.line(vis, (center_x_draw, 0), (center_x_draw, h - 1), (255, 0, 255), 1)
        cv2.circle(vis, (x_bottom_draw, y_bottom), 6, (0, 165, 255), -1)
        txt = (
            f"angle={angle_deg:.2f} deg  steer={steer_deg:.2f} deg  "
            f"offset={offset_px:.1f}px ({offset_norm:+.2f})"
        )
        cv2.putText(vis, txt, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    return True, angle_deg, steer_deg, offset_px, offset_norm, vis


def _first_available_attr(msg, names):
    """返回消息中第一个存在的字段值。"""
    for name in names:
        if hasattr(msg, name):
            value = getattr(msg, name)
            if value is not None:
                return value
    return None


def hbmem_msg_to_bgr(msg):
    """
    将 Horizon 共享内存 NV12 图像消息转换成 OpenCV BGR 图像。

    这里对字段名做了兼容处理，优先适配 `width/height/data`，
    也兼容一些平台实现里常见的别名。
    """
    width = _first_available_attr(msg, ("width", "image_width", "w"))
    height = _first_available_attr(msg, ("height", "image_height", "h"))
    stride = _first_available_attr(msg, ("stride", "step", "width_step", "aligned_width"))
    data = _first_available_attr(msg, ("data", "img", "image", "image_data"))

    if width is None or height is None or data is None:
        available = list(getattr(msg, "__slots__", []))
        raise ValueError(
            "HbmMsg1080P missing required fields, expected width/height/data compatible attributes, "
            f"available={available}"
        )

    width = int(width)
    height = int(height)
    stride = int(stride) if stride is not None else width
    if width <= 0 or height <= 0 or stride < width:
        raise ValueError(f"invalid hbmem image geometry: width={width}, height={height}, stride={stride}")

    buf = np.frombuffer(data, dtype=np.uint8)
    expected = stride * height * 3 // 2
    if buf.size < expected:
        raise ValueError(f"hbmem image buffer too small: got {buf.size} bytes, need {expected}")

    nv12 = buf[:expected].reshape((height * 3 // 2, stride))
    if stride != width:
        nv12 = nv12[:, :width]

    return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)


def resolve_calibration_path(path_text):
    """
    把标定文件路径解析为源码目录或 ROS 安装目录中的实际文件。

    现场运行时可能有两种路径来源：
    - 开发环境：直接从源码目录运行，`config/camera_calibration.json` 是相对当前工作目录的文件
    - 安装环境：通过 `ros2 launch` 从 install 目录运行，需要在 ROS 包 share 目录下查找

    该函数按“绝对路径 -> 当前目录相对路径 -> 包安装目录”顺序查找。
    """
    path_text = str(path_text).strip()
    if not path_text:
        return None

    path = Path(path_text).expanduser()
    # 用户传入绝对路径时优先使用，便于临时指定外部标定文件。
    if path.is_absolute() and path.exists():
        return path
    # 源码目录调试时，相对路径通常直接能找到。
    if not path.is_absolute() and path.exists():
        return path

    try:
        # 安装后的 ROS 包资源目录，例如 install/line_follow/share/line_follow。
        package_share = Path(get_package_share_directory("line_follow"))
    except Exception:
        package_share = None

    if package_share is not None:
        # 支持两种写法：
        # - config/camera_calibration.json
        # - camera_calibration.json
        candidates = [package_share / path_text]
        if not path_text.startswith("config/"):
            candidates.append(package_share / "config" / path_text)
        for candidate in candidates:
            if candidate.exists():
                return candidate

    return path


class CameraUndistorter:
    """
    根据 OpenCV 标定结果缓存去畸变映射表。

    去畸变每帧直接调用 cv2.undistort 会重复计算映射关系，开销较大。
    这里在第一次看到某个图像尺寸时，用 initUndistortRectifyMap 生成 map1/map2；
    后续每帧只调用 cv2.remap，性能更稳定。

    标定文件基于某个分辨率生成，例如 1920x1080。实际运行时如果输入图像尺寸不同，
    会按宽高比例缩放相机内参，保证同一份标定可以用于等比例缩放后的图像。
    """

    def __init__(self, calibration_path, logger):
        self.logger = logger
        self.calibration_path = resolve_calibration_path(calibration_path)
        self.base_size = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.map_size = None
        self.map1 = None
        self.map2 = None
        self._load()

    def _load(self):
        """读取 JSON 标定文件，并转换成 OpenCV 使用的 numpy 矩阵。"""
        if self.calibration_path is None:
            raise ValueError("empty camera calibration file path")
        with self.calibration_path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        # base_size 是标定图片的原始尺寸，后面用于判断运行时图像是否需要缩放内参。
        image_size = data.get("image_size", {})
        self.base_size = (int(image_size["width"]), int(image_size["height"]))
        self.camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
        self.dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64).reshape(-1, 1)
        optimal = data.get("optimal_new_camera_matrix")
        # optimal_new_camera_matrix 来自 getOptimalNewCameraMatrix，
        # 相比原始内参会在保留视野和去除黑边之间做一个折中。
        self.new_camera_matrix = (
            np.array(optimal, dtype=np.float64)
            if optimal is not None
            else self.camera_matrix.copy()
        )

    def _scaled_matrices(self, size):
        """根据当前输入图像尺寸缩放内参矩阵。"""
        width, height = size
        base_w, base_h = self.base_size
        scale_x = float(width) / float(base_w)
        scale_y = float(height) / float(base_h)

        camera_matrix = self.camera_matrix.copy()
        camera_matrix[0, 0] *= scale_x
        camera_matrix[0, 2] *= scale_x
        camera_matrix[1, 1] *= scale_y
        camera_matrix[1, 2] *= scale_y

        new_camera_matrix = self.new_camera_matrix.copy()
        new_camera_matrix[0, 0] *= scale_x
        new_camera_matrix[0, 2] *= scale_x
        new_camera_matrix[1, 1] *= scale_y
        new_camera_matrix[1, 2] *= scale_y
        return camera_matrix, new_camera_matrix

    def undistort(self, frame):
        """对单帧 BGR 图像执行去畸变，返回校正后的 BGR 图像。"""
        height, width = frame.shape[:2]
        size = (width, height)
        if self.map_size != size:
            # 图像尺寸变化时才重新建表；常规运行中这里只会在第一帧执行一次。
            camera_matrix, new_camera_matrix = self._scaled_matrices(size)
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                camera_matrix,
                self.dist_coeffs,
                None,
                new_camera_matrix,
                size,
                cv2.CV_16SC2,
            )
            self.map_size = size
            self.logger.info(
                "camera undistort map ready: "
                f"calibration={self.calibration_path}, base_size={self.base_size[0]}x{self.base_size[1]}, "
                f"frame_size={width}x{height}"
            )

        return cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)


class LineFollowAngleNode(Node):
    """
    ROS2 巡线角度检测节点。

    节点负责把相机图像转换成车辆控制需要的低维信息：
    - line_angle_deg：线条相对车体前向的角度
    - line_offset_px / line_offset_norm：线条底部相对图像中心的横向偏移
    - line_detected：当前帧是否可靠检测到线

    同时发布多路调试图像，供 Web 页面观察每一步中间结果。
    """

    def __init__(self):
        super().__init__("line_follow_angle_node")

        # 图像输入与算法参数都做成 ROS 参数，方便现场不改代码直接调参。
        # launch 文件和 config/line_follow_params.yaml 会覆盖这些默认值。
        self.declare_parameter("image_topic", "/hbmem_img")
        self.declare_parameter("image_msg_type", "hbmem")

        # 相机标定/去畸变参数。启用后，本节点后续检测和 debug_undistorted 话题
        # 都使用校正后的图像。
        self.declare_parameter("enable_camera_undistort", True)
        self.declare_parameter("camera_calibration_file", "config/camera_calibration.json")

        # 视觉检测方法。clahe_adaptive 是当前默认方法；edge_line/threshold 保留为回退方案。
        self.declare_parameter("detection_method", "clahe_adaptive")

        # threshold 方法参数：用于旧版灰度阈值检测。
        self.declare_parameter("line_is_white", False)
        self.declare_parameter("blur_ksize", 5)
        self.declare_parameter("thresh", -1)
        self.declare_parameter("morph_ksize", 3)
        self.declare_parameter("min_component_area", 20)
        self.declare_parameter("max_component_area", 0)
        self.declare_parameter("max_component_width_px", 0)
        self.declare_parameter("max_component_width_ratio", 0.0)
        self.declare_parameter("component_intensity_limit", -1.0)

        # clahe_adaptive 方法参数：强光/阴影下优先增强局部对比度，再做局部阈值分割。
        self.declare_parameter("clahe_clip_limit", 3.0)
        self.declare_parameter("clahe_tile_grid_size", 8)
        self.declare_parameter("adaptive_block_size", 51)
        self.declare_parameter("adaptive_c", 5.0)

        # edge_line 方法参数：先找边缘，再用 Hough 筛选线段结构。
        self.declare_parameter("canny_low", 40)
        self.declare_parameter("canny_high", 120)
        self.declare_parameter("edge_morph_ksize", 3)
        self.declare_parameter("edge_dilate_ksize", 5)
        self.declare_parameter("hough_threshold", 40)
        self.declare_parameter("hough_min_line_length_ratio", 0.12)
        self.declare_parameter("hough_max_line_gap", 35)
        self.declare_parameter("max_line_angle_from_vertical_deg", 65.0)
        self.declare_parameter("line_draw_width", 9)
        self.declare_parameter("fallback_to_edge_band", True)

        # 滑动窗口和几何输出参数。无论使用哪种二值图生成方法，后续都走同一套滑窗拟合。
        self.declare_parameter("n_windows", 9)
        self.declare_parameter("margin", 180)
        self.declare_parameter("minpix", 50)
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("angle_bias_deg", 3.36)
        self.declare_parameter("target_offset_px", 67.7)
        self.declare_parameter("roi_bottom_offset_ratio", 0.10)
        self.declare_parameter("roi_height_ratio", 0.5)
        self.declare_parameter("roi_side_margin_ratio", 0.05)
        self.declare_parameter("roi_left_margin_ratio", 0.15)
        self.declare_parameter("roi_right_margin_ratio", 0.05)
        self.declare_parameter("base_search_half_width_ratio", 0.25)

        # 调试输出参数。本地窗口依赖 DISPLAY；远程调试主要看 ROS 图像话题和 Web 页面。
        self.declare_parameter("show_debug", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_publish_every_n", 3)
        self.declare_parameter("debug_undistorted_topic", "/line_follow/debug_undistorted")
        self.declare_parameter("debug_binary_topic", "/line_follow/debug_binary")
        self.declare_parameter("debug_detect_topic", "/line_follow/debug_detect")
        self.declare_parameter("debug_angle_curve_topic", "/line_follow/debug_angle_curve")
        self.declare_parameter("offset_px_topic", "/line_follow/line_offset_px")
        self.declare_parameter("offset_norm_topic", "/line_follow/line_offset_norm")
        self.declare_parameter("visual_heading_error_topic", "/line_follow/visual_heading_error_deg")
        self.declare_parameter("visual_lateral_error_px_topic", "/line_follow/visual_lateral_error_px")
        self.declare_parameter("visual_lateral_error_norm_topic", "/line_follow/visual_lateral_error_norm")
        self.declare_parameter("line_detected_topic", "/line_follow/line_detected")
        self.declare_parameter("show_angle_curve", True)
        self.declare_parameter("angle_curve_history_size", 240)
        self.declare_parameter("angle_curve_limit_deg", 45.0)

        # CvBridge 用于在 ROS Image 和 OpenCV Mat 之间转换。
        self.bridge = CvBridge()
        self._load_fixed_endpoints()

        # 角度历史用于生成曲线调试图。deque 自动限制长度，避免长时间运行内存增长。
        history_size = max(30, int(self.get_parameter("angle_curve_history_size").value))
        self.angle_history = deque(maxlen=history_size)

        # 板端服务通常没有桌面环境，此时不能调用 imshow 创建本地窗口。
        # 但 debug 图像话题仍然可以正常发布到 Web 页面。
        self.display_available = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.warned_no_display = False

        # 去畸变对象按需创建。关闭参数后会释放引用，后续帧直接使用原图。
        self.undistorter = None
        self.undistort_enabled = bool(self.get_parameter("enable_camera_undistort").value)
        self.camera_calibration_file = str(self.get_parameter("camera_calibration_file").value)
        if self.undistort_enabled:
            self._load_camera_undistorter()

        # 检测成功/失败计数用于日志节流和现场观察稳定性。
        self.detect_fail_streak = 0
        self.detect_success_count = 0
        self.detect_fail_count = 0
        self.frame_count = 0

        # 调试发布器支持运行时开关和换话题，因此先保存 topic，再由 _refresh_debug_runtime 创建。
        self.publish_debug_image_enabled = False
        self.debug_undistorted_topic = str(self.get_parameter("debug_undistorted_topic").value)
        self.debug_binary_topic = str(self.get_parameter("debug_binary_topic").value)
        self.debug_detect_topic = str(self.get_parameter("debug_detect_topic").value)
        self.debug_angle_curve_topic = str(self.get_parameter("debug_angle_curve_topic").value)
        self.pub_undistorted = None
        self.pub_binary = None
        self.pub_detect = None
        self.pub_angle_curve = None

        # 订阅相机图像。RDK 板端默认消费共享内存 hbmem；电脑或普通 ROS 图像可用 raw。
        if self.image_msg_type == "hbmem":
            if HbmMsg1080P is None:
                raise RuntimeError(
                    "image_msg_type=hbmem but hbm_img_msgs is unavailable. "
                    "Please source the tros environment and install the hbm_img_msgs package."
                )
            self.sub = self.create_subscription(HbmMsg1080P, self.image_topic, self.on_image, qos_profile_sensor_data)
            subscribed_type = "hbm_img_msgs/msg/HbmMsg1080P"
        elif self.image_msg_type == "raw":
            self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
            subscribed_type = "sensor_msgs/msg/Image"
        else:
            raise RuntimeError(f"unsupported image_msg_type: {self.image_msg_type}, expected 'hbmem' or 'raw'")

        # 发布检测角度、建议转向量、横向偏移和检测有效标志。
        # 下游运动模型主要订阅 angle 和 line_detected，也可以进一步使用 offset 做横向纠偏。
        self.pub_steer = self.create_publisher(Float32, "/line_follow/steer_angle_deg", 10)
        self.pub_angle = self.create_publisher(Float32, "/line_follow/line_angle_deg", 10)
        self.pub_offset_px = self.create_publisher(Float32, self.offset_px_topic, 10)
        self.pub_offset_norm = self.create_publisher(Float32, self.offset_norm_topic, 10)
        self.pub_visual_heading_error = self.create_publisher(
            Float32,
            str(self.get_parameter("visual_heading_error_topic").value),
            10,
        )
        self.pub_visual_lateral_error_px = self.create_publisher(
            Float32,
            str(self.get_parameter("visual_lateral_error_px_topic").value),
            10,
        )
        self.pub_visual_lateral_error_norm = self.create_publisher(
            Float32,
            str(self.get_parameter("visual_lateral_error_norm_topic").value),
            10,
        )
        self.pub_line_detected = self.create_publisher(
            Bool,
            str(self.get_parameter("line_detected_topic").value),
            10,
        )
        self._refresh_debug_runtime()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            "Subscribed to "
            f"{self.image_topic} ({subscribed_type}), publishing /line_follow/line_angle_deg, "
            f"{self.offset_norm_topic}, {self.offset_px_topic} and /line_follow/steer_angle_deg"
        )

    def _load_fixed_endpoints(self):
        """
        加载需要重启节点后才可改变的输入输出端点。

        订阅器和发布器创建以后，话题名和消息类型不能简单通过参数热切换；
        因此这些参数在 _on_set_parameters 中被标记为 static，需要重启节点生效。
        """
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.image_msg_type = str(self.get_parameter("image_msg_type").value).strip().lower()
        self.offset_px_topic = str(self.get_parameter("offset_px_topic").value)
        self.offset_norm_topic = str(self.get_parameter("offset_norm_topic").value)

    def _load_camera_undistorter(self):
        """加载相机标定参数，后续按实际图像尺寸缓存 remap 表。"""
        self.camera_calibration_file = str(self.get_parameter("camera_calibration_file").value)
        self.undistorter = CameraUndistorter(self.camera_calibration_file, self.get_logger())
        self.get_logger().info(f"camera undistort enabled: {self.undistorter.calibration_path}")

    def _destroy_debug_publishers(self):
        """销毁当前调试图像发布器，用于关闭调试输出或切换调试话题名。"""
        for name in ("pub_undistorted", "pub_binary", "pub_detect", "pub_angle_curve"):
            pub = getattr(self, name)
            if pub is not None:
                try:
                    self.destroy_publisher(pub)
                except Exception:
                    pass
                setattr(self, name, None)

    def _create_debug_publishers(self):
        """
        按当前参数创建调试图像发布器。

        发布的四路图像分别用于 Web 页面四宫格：
        - debug_undistorted：去畸变后的原始相机图
        - debug_binary：算法实际使用的二值候选图
        - debug_detect：滑窗、拟合线、角度和偏移可视化
        - debug_angle_curve：最近一段时间角度变化曲线
        """
        self.pub_undistorted = self.create_publisher(Image, self.debug_undistorted_topic, 10)
        self.pub_binary = self.create_publisher(Image, self.debug_binary_topic, 10)
        self.pub_detect = self.create_publisher(Image, self.debug_detect_topic, 10)
        self.pub_angle_curve = self.create_publisher(Image, self.debug_angle_curve_topic, 10)

    def _refresh_debug_runtime(self, overrides=None):
        """
        刷新调试发布器和曲线历史配置，支持运行时切换。

        该函数既在节点初始化时调用，也在 ros2 param set 修改参数时调用。
        如果只是调试开关变化，就创建或销毁发布器；如果曲线历史长度变化，
        就保留最近的历史数据并调整 deque 容量。
        """
        values = {
            "publish_debug_image": bool(self.get_parameter("publish_debug_image").value),
            "debug_undistorted_topic": str(self.get_parameter("debug_undistorted_topic").value),
            "debug_binary_topic": str(self.get_parameter("debug_binary_topic").value),
            "debug_detect_topic": str(self.get_parameter("debug_detect_topic").value),
            "debug_angle_curve_topic": str(self.get_parameter("debug_angle_curve_topic").value),
            "angle_curve_history_size": int(self.get_parameter("angle_curve_history_size").value),
        }
        if overrides:
            values.update(overrides)

        publish_debug_image = bool(values["publish_debug_image"])
        debug_undistorted_topic = str(values["debug_undistorted_topic"])
        debug_binary_topic = str(values["debug_binary_topic"])
        debug_detect_topic = str(values["debug_detect_topic"])
        debug_angle_curve_topic = str(values["debug_angle_curve_topic"])
        history_size = max(30, int(values["angle_curve_history_size"]))

        current_history = list(getattr(self, "angle_history", []))
        if not hasattr(self, "angle_history") or self.angle_history.maxlen != history_size:
            self.angle_history = deque(current_history[-history_size:], maxlen=history_size)

        topics_changed = (
            debug_undistorted_topic != getattr(self, "debug_undistorted_topic", debug_undistorted_topic)
            or debug_binary_topic != getattr(self, "debug_binary_topic", debug_binary_topic)
            or debug_detect_topic != getattr(self, "debug_detect_topic", debug_detect_topic)
            or debug_angle_curve_topic != getattr(self, "debug_angle_curve_topic", debug_angle_curve_topic)
        )
        self.debug_undistorted_topic = debug_undistorted_topic
        self.debug_binary_topic = debug_binary_topic
        self.debug_detect_topic = debug_detect_topic
        self.debug_angle_curve_topic = debug_angle_curve_topic
        self.publish_debug_image_enabled = publish_debug_image

        if publish_debug_image:
            if (
                topics_changed
                or self.pub_undistorted is None
                or self.pub_binary is None
                or self.pub_detect is None
                or self.pub_angle_curve is None
            ):
                self._destroy_debug_publishers()
                self._create_debug_publishers()
        elif (
            self.pub_undistorted is not None
            or self.pub_binary is not None
            or self.pub_detect is not None
            or self.pub_angle_curve is not None
        ):
            self._destroy_debug_publishers()

    def _on_set_parameters(self, params):
        """
        ROS 参数运行时更新回调。

        大多数视觉算法参数都允许热更新，便于现场调参；
        但输入话题、消息类型、输出话题和标定文件路径涉及订阅器/发布器或缓存对象，
        为避免运行中状态不一致，要求重启节点后生效。
        """
        static_params = {
            "image_topic",
            "image_msg_type",
            "offset_px_topic",
            "offset_norm_topic",
            "visual_heading_error_topic",
            "visual_lateral_error_px_topic",
            "visual_lateral_error_norm_topic",
            "camera_calibration_file",
        }
        overrides = {}
        for param in params:
            if param.name in static_params:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} requires node restart because subscriptions/publishers are already created",
                )
            overrides[param.name] = param.value

        try:
            self._refresh_debug_runtime(overrides)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if overrides:
            changed = ", ".join(sorted(overrides.keys()))
            self.get_logger().info(f"runtime parameters updated: {changed}")
        return SetParametersResult(successful=True)

    def draw_angle_curve(self, current_angle_deg, ok):
        """绘制最近一段时间的角度变化曲线。"""
        self.angle_history.append(float(current_angle_deg) if ok else None)

        canvas_h = 240
        canvas_w = 640
        pad = 40
        canvas = np.full((canvas_h, canvas_w, 3), 24, dtype=np.uint8)
        limit_deg = max(5.0, float(self.get_parameter("angle_curve_limit_deg").value))
        center_y = canvas_h // 2
        plot_w = canvas_w - 2 * pad
        plot_h = canvas_h - 2 * pad

        cv2.rectangle(canvas, (pad, pad), (canvas_w - pad, canvas_h - pad), (70, 70, 70), 1)
        cv2.line(canvas, (pad, center_y), (canvas_w - pad, center_y), (90, 90, 90), 1)

        for tick_deg in (-limit_deg, 0.0, limit_deg):
            y = int(round(center_y - (tick_deg / limit_deg) * (plot_h / 2.0)))
            cv2.line(canvas, (pad - 5, y), (pad, y), (160, 160, 160), 1)
            cv2.putText(
                canvas,
                f"{tick_deg:.0f}",
                (5, max(15, y + 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (180, 180, 180),
                1,
            )

        points = []
        history = list(self.angle_history)
        denom = max(1, len(history) - 1)
        for idx, angle in enumerate(history):
            if angle is None:
                points.append(None)
                continue
            x = pad + int(round(idx * plot_w / denom))
            clipped = max(-limit_deg, min(limit_deg, float(angle)))
            y = int(round(center_y - (clipped / limit_deg) * (plot_h / 2.0)))
            points.append((x, y))

        prev = None
        for pt in points:
            if pt is None:
                prev = None
                continue
            if prev is not None:
                cv2.line(canvas, prev, pt, (0, 220, 255), 2)
            prev = pt

        status_text = f"angle={current_angle_deg:.2f} deg" if ok else "angle=lost"
        cv2.putText(canvas, status_text, (pad, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)
        cv2.putText(
            canvas,
            f"history={len(history)}  limit=+/-{limit_deg:.0f} deg",
            (pad, canvas_h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (180, 180, 180),
            1,
        )
        return canvas

    def on_image(self, msg):
        """
        图像回调：每来一帧图像就执行一次完整巡线检测。

        单帧处理流程：
        1. 把 ROS 图像消息转换成 OpenCV BGR 图像
        2. 按需做相机去畸变
        3. 读取当前 ROS 参数，支持运行中调参
        4. 生成二值候选图
        5. 滑窗拟合线条，计算角度和横向偏移
        6. 发布控制量、检测状态和调试图像
        """
        try:
            if isinstance(msg, Image):
                # raw 模式：sensor_msgs/Image 直接通过 CvBridge 转 BGR。
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                # hbmem 模式：RDK 共享内存消息通常是 NV12，需要手动转 BGR。
                frame = hbmem_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")
            return

        # 去畸变放在二值化之前，这样检测、偏移计算、Web 原图显示都基于同一张校正图。
        # 如果标定文件加载失败或 remap 失败，本帧会回退到原图，避免节点直接退出。
        enable_camera_undistort = bool(self.get_parameter("enable_camera_undistort").value)
        if enable_camera_undistort:
            if self.undistorter is None:
                try:
                    self._load_camera_undistorter()
                except Exception as e:
                    self.get_logger().warn(f"camera undistort disabled because calibration load failed: {e}")
                    self.undistorter = None
            if self.undistorter is not None:
                try:
                    frame = self.undistorter.undistort(frame)
                except Exception as e:
                    self.get_logger().warn(f"camera undistort failed, using raw frame: {e}")
        else:
            self.undistorter = None

        # 每帧都读取当前参数，便于现场通过 ros2 param 动态调试。
        # 这些 get_parameter 调用开销相对图像处理很小，换来的是无需重启即可调参。
        detection_method = str(self.get_parameter("detection_method").value)
        line_is_white = bool(self.get_parameter("line_is_white").value)
        blur_ksize = int(self.get_parameter("blur_ksize").value)
        thresh = int(self.get_parameter("thresh").value)
        morph_ksize = int(self.get_parameter("morph_ksize").value)
        min_component_area = int(self.get_parameter("min_component_area").value)
        max_component_area = int(self.get_parameter("max_component_area").value)
        max_component_width_px = int(self.get_parameter("max_component_width_px").value)
        max_component_width_ratio = float(self.get_parameter("max_component_width_ratio").value)
        component_intensity_limit = float(self.get_parameter("component_intensity_limit").value)
        clahe_clip_limit = float(self.get_parameter("clahe_clip_limit").value)
        clahe_tile_grid_size = int(self.get_parameter("clahe_tile_grid_size").value)
        adaptive_block_size = int(self.get_parameter("adaptive_block_size").value)
        adaptive_c = float(self.get_parameter("adaptive_c").value)
        canny_low = int(self.get_parameter("canny_low").value)
        canny_high = int(self.get_parameter("canny_high").value)
        edge_morph_ksize = int(self.get_parameter("edge_morph_ksize").value)
        edge_dilate_ksize = int(self.get_parameter("edge_dilate_ksize").value)
        hough_threshold = int(self.get_parameter("hough_threshold").value)
        hough_min_line_length_ratio = float(self.get_parameter("hough_min_line_length_ratio").value)
        hough_max_line_gap = int(self.get_parameter("hough_max_line_gap").value)
        max_line_angle_from_vertical_deg = float(
            self.get_parameter("max_line_angle_from_vertical_deg").value
        )
        line_draw_width = int(self.get_parameter("line_draw_width").value)
        fallback_to_edge_band = bool(self.get_parameter("fallback_to_edge_band").value)
        n_windows = int(self.get_parameter("n_windows").value)
        margin = int(self.get_parameter("margin").value)
        minpix = int(self.get_parameter("minpix").value)
        kp = float(self.get_parameter("kp").value)
        angle_bias_deg = float(self.get_parameter("angle_bias_deg").value)
        target_offset_px = float(self.get_parameter("target_offset_px").value)
        roi_bottom_offset_ratio = float(self.get_parameter("roi_bottom_offset_ratio").value)
        roi_height_ratio = float(self.get_parameter("roi_height_ratio").value)
        roi_side_margin_ratio = float(self.get_parameter("roi_side_margin_ratio").value)
        roi_left_margin_ratio = float(self.get_parameter("roi_left_margin_ratio").value)
        roi_right_margin_ratio = float(self.get_parameter("roi_right_margin_ratio").value)
        base_search_half_width_ratio = float(
            self.get_parameter("base_search_half_width_ratio").value
        )
        show = bool(self.get_parameter("show_debug").value)
        publish_debug_image = self.publish_debug_image_enabled
        debug_publish_every_n = max(1, int(self.get_parameter("debug_publish_every_n").value))
        self.frame_count += 1
        debug_publish_due = publish_debug_image and (self.frame_count % debug_publish_every_n == 0)
        show_angle_curve = bool(self.get_parameter("show_angle_curve").value)
        enable_local_windows = show and self.display_available
        # 只有确实要显示 detect 图时才让 detect_line_and_angle 绘制可视化内容，
        # 否则可减少每帧画矩形、画线和颜色转换的开销。
        need_detect_vis = enable_local_windows or (debug_publish_due and self.pub_detect is not None)

        # 生成滑窗输入二值图。clahe_adaptive 模式下这里输出的是局部增强和自适应阈值后的线条区域；
        # edge_line 模式输出边缘/线结构候选区域；threshold 模式输出灰度阈值分割区域。
        bin_img = make_binary(
            frame,
            detection_method,
            line_is_white,
            blur_ksize,
            thresh,
            morph_ksize,
            min_component_area,
            max_component_area,
            max_component_width_px,
            max_component_width_ratio,
            component_intensity_limit,
            canny_low,
            canny_high,
            edge_morph_ksize,
            edge_dilate_ksize,
            hough_threshold,
            hough_min_line_length_ratio,
            hough_max_line_gap,
            max_line_angle_from_vertical_deg,
            line_draw_width,
            fallback_to_edge_band,
            clahe_clip_limit,
            clahe_tile_grid_size,
            adaptive_block_size,
            adaptive_c,
        )
        bin_img = apply_detection_roi_mask(
            bin_img,
            roi_bottom_offset_ratio=roi_bottom_offset_ratio,
            roi_height_ratio=roi_height_ratio,
            roi_side_margin_ratio=roi_side_margin_ratio,
            roi_left_margin_ratio=roi_left_margin_ratio,
            roi_right_margin_ratio=roi_right_margin_ratio,
        )
        # 在二值图上做滑动窗口搜索，并拟合出线条方向。
        # 返回的 offset_px/offset_norm 目前以图像中心为参考，后续可扩展为目标偏右/偏左参考线。
        ok, angle_deg, steer_deg, offset_px, offset_norm, vis = detect_line_and_angle(
            bin_img,
            n_windows=n_windows,
            margin=margin,
            minpix=minpix,
            kp=kp,
            roi_bottom_offset_ratio=roi_bottom_offset_ratio,
            roi_height_ratio=roi_height_ratio,
            roi_side_margin_ratio=roi_side_margin_ratio,
            roi_left_margin_ratio=roi_left_margin_ratio,
            roi_right_margin_ratio=roi_right_margin_ratio,
            base_search_half_width_ratio=base_search_half_width_ratio,
            draw=need_detect_vis,
        )
        # angle_bias_deg 用来补偿相机安装角度不正、车体机械偏差等固定误差。
        angle_deg += angle_bias_deg
        steer_deg = -float(kp) * angle_deg
        lateral_error_px = offset_px - target_offset_px
        image_center_x = 0.5 * float(max(1, bin_img.shape[1] - 1))
        lateral_error_norm = (
            0.0
            if image_center_x <= 0.0
            else max(-1.0, min(1.0, lateral_error_px / image_center_x))
        )
        if need_detect_vis and vis is not None:
            cv2.rectangle(vis, (0, 0), (vis.shape[1] - 1, 42), (0, 0, 0), -1)
            cv2.putText(
                vis,
                (
                    f"angle={angle_deg:.2f} deg  steer={steer_deg:.2f} deg  "
                    f"lat={lateral_error_px:.1f}px ({lateral_error_norm:+.2f})"
                ),
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

        if ok:
            # 检测成功时发布真实角度、建议转向和横向偏移。
            self.detect_success_count += 1
            if self.detect_fail_streak > 0:
                self.get_logger().info(
                    f"line detection recovered after {self.detect_fail_streak} failed frames, "
                    f"angle={angle_deg:.2f} deg"
                )
                self.detect_fail_streak = 0

            m1 = Float32()
            m1.data = float(angle_deg)
            self.pub_angle.publish(m1)

            m2 = Float32()
            m2.data = float(steer_deg)
            self.pub_steer.publish(m2)

            m3 = Float32()
            m3.data = float(offset_px)
            self.pub_offset_px.publish(m3)

            m4 = Float32()
            m4.data = float(offset_norm)
            self.pub_offset_norm.publish(m4)

            m5 = Float32()
            m5.data = float(angle_deg)
            self.pub_visual_heading_error.publish(m5)

            m6 = Float32()
            m6.data = float(lateral_error_px)
            self.pub_visual_lateral_error_px.publish(m6)

            m7 = Float32()
            m7.data = float(lateral_error_norm)
            self.pub_visual_lateral_error_norm.publish(m7)
        else:
            # 检测失败时记录连续失败次数，并按节流策略打印关键参数。
            # white_ratio 是二值图白色像素比例：太小通常表示没提取到线，太大通常表示噪声过多。
            self.detect_fail_count += 1
            self.detect_fail_streak += 1
            if self.detect_fail_streak == 1 or self.detect_fail_streak % 30 == 0:
                white_ratio = float(np.count_nonzero(bin_img)) / float(bin_img.size) if bin_img.size else 0.0
                self.get_logger().warn(
                    "line detection failed: "
                    f"streak={self.detect_fail_streak}, white_ratio={white_ratio:.3f}, "
                    f"detection_method={detection_method}, line_is_white={line_is_white}, "
                    f"thresh={thresh}, adaptive_block_size={adaptive_block_size}, "
                    f"adaptive_c={adaptive_c:.1f}, clahe_clip_limit={clahe_clip_limit:.1f}, "
                    f"canny={canny_low}/{canny_high}, "
                    f"hough_threshold={hough_threshold}, margin={margin}, "
                    f"minpix={minpix}, roi_bottom_offset_ratio={roi_bottom_offset_ratio:.2f}, "
                    f"roi_height_ratio={roi_height_ratio:.2f}, "
                    f"roi_side_margin_ratio={roi_side_margin_ratio:.2f}, "
                    f"roi_left_margin_ratio={roi_left_margin_ratio:.2f}, "
                    f"roi_right_margin_ratio={roi_right_margin_ratio:.2f}, "
                    f"base_search_half_width_ratio={base_search_half_width_ratio:.2f}"
                )

            # 丢线时不再发布新的 angle；下游运动模型会通过 line_detected=false
            # 或角度超时进入停车，避免把无效视觉结果解释成“直行”。
            m2 = Float32()
            m2.data = 0.0
            self.pub_steer.publish(m2)

            m3 = Float32()
            m3.data = 0.0
            self.pub_offset_px.publish(m3)

            m4 = Float32()
            m4.data = 0.0
            self.pub_offset_norm.publish(m4)

            m5 = Float32()
            m5.data = 0.0
            self.pub_visual_heading_error.publish(m5)

            m6 = Float32()
            m6.data = 0.0
            self.pub_visual_lateral_error_px.publish(m6)

            m7 = Float32()
            m7.data = 0.0
            self.pub_visual_lateral_error_norm.publish(m7)

        detected_msg = Bool()
        detected_msg.data = bool(ok)
        self.pub_line_detected.publish(detected_msg)

        # 尽量沿用输入图像 header，方便后续用时间戳对齐其他传感器或录包回放。
        header = getattr(msg, "header", None)

        if debug_publish_due and self.pub_undistorted is not None:
            try:
                # 第一宫格：去畸变后的相机原图。Web 页面默认显示这一路作为 Camera。
                undistorted_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                if header is not None:
                    undistorted_msg.header = header
                self.pub_undistorted.publish(undistorted_msg)
            except Exception as e:
                self.get_logger().warn(f"publish undistorted debug image failed: {e}")

        if debug_publish_due and self.pub_binary is not None:
            try:
                # 第二宫格：二值候选图。调 CLAHE、自适应阈值、ROI 和形态学参数时主要看这一路。
                binary_msg = self.bridge.cv2_to_imgmsg(bin_img, encoding="mono8")
                if header is not None:
                    binary_msg.header = header
                self.pub_binary.publish(binary_msg)
            except Exception as e:
                self.get_logger().warn(f"publish binary debug image failed: {e}")

        if debug_publish_due and self.pub_detect is not None:
            try:
                # 第三宫格：检测结果图。包含 ROI、滑窗、拟合线、图像中心线和底部交点。
                detect_vis = vis if vis is not None else cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
                detect_msg = self.bridge.cv2_to_imgmsg(detect_vis, encoding="bgr8")
                if header is not None:
                    detect_msg.header = header
                self.pub_detect.publish(detect_msg)
            except Exception as e:
                self.get_logger().warn(f"publish detect debug image failed: {e}")

        curve_vis = None
        if show_angle_curve:
            # 曲线图不参与控制，只用于观察角度是否抖动、是否频繁丢线。
            curve_vis = self.draw_angle_curve(angle_deg, ok)

        if debug_publish_due and self.pub_angle_curve is not None and curve_vis is not None:
            try:
                # 第四宫格：角度历史曲线。
                curve_msg = self.bridge.cv2_to_imgmsg(curve_vis, encoding="bgr8")
                if header is not None:
                    curve_msg.header = header
                self.pub_angle_curve.publish(curve_msg)
            except Exception as e:
                self.get_logger().warn(f"publish angle curve debug image failed: {e}")

        if show and not enable_local_windows and not self.warned_no_display:
            self.warned_no_display = True
            self.get_logger().warn(
                "show_debug=true but no DISPLAY/WAYLAND_DISPLAY found; local OpenCV windows disabled, "
                "debug image topics remain available"
            )

        if enable_local_windows:
            # 本地桌面调试窗口。板端服务通常没有显示环境，此分支一般不会执行。
            h, w = frame.shape[:2]
            size_text = f"size: {w}x{h}"

            binary_vis = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
            cv2.putText(binary_vis, size_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("binary", binary_vis)

            if vis is not None:
                cv2.putText(vis, size_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("detect", vis)

            if curve_vis is not None:
                cv2.imshow("angle_curve", curve_vis)

            cv2.waitKey(1)


def main():
    """节点入口：初始化 ROS，创建节点，循环运行直到退出。"""
    rclpy.init()
    node = LineFollowAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
