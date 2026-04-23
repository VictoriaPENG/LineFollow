#!/usr/bin/env python3
"""
巡线角度检测节点。

功能概述：
1. 订阅相机图像话题
2. 将图像做灰度化、模糊、二值化和形态学去噪
3. 使用滑动窗口在图像底部到顶部逐段搜索线条中心
4. 对搜索到的中心点做直线拟合
5. 计算线条相对车体“竖直向前方向”的偏转角
6. 计算线条底部交点相对图像中心的横向偏移
7. 发布检测角度 `/line_follow/line_angle_deg`
8. 发布横向偏移 `/line_follow/line_offset_px` 和 `/line_follow/line_offset_norm`
9. 发布一个简单的建议转向角 `/line_follow/steer_angle_deg`
10. 发布调试图像话题，便于远程查看检测结果
"""

import math
import os
from collections import deque

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

try:
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

    `component_intensity_limit` 为负数时关闭强度过滤。
    黑线模式下保留平均灰度更低的对象，白线模式下保留平均灰度更高的对象。
    """
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(bin_img, connectivity=8)
    if num_labels <= 1:
        return bin_img

    _, w = bin_img.shape[:2]
    max_width_limit = 0
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
            mean_gray = float(np.mean(gray[mask]))
            if line_is_white:
                if mean_gray < intensity_limit:
                    continue
            elif mean_gray > intensity_limit:
                continue

        filtered[mask] = 255

    return filtered


def make_binary(
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
    将输入 BGR 图像转换为适合巡线检测的二值图像。

    参数说明：
    - `line_is_white=True` 表示目标线是白线，背景较暗
    - `line_is_white=False` 表示目标线是黑线，背景较亮
    - `thresh=-1` 时使用 Otsu 自动阈值
    - `morph_ksize` 用于开闭运算，尽量去掉小噪点并填补小孔洞
    """
    # 先转灰度，后续阈值分割只需要亮度信息。
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # 模糊用于降低椒盐噪声和纹理细节对阈值分割的影响。
    k = max(1, int(blur_ksize))
    if k % 2 == 0:
        k += 1
    gray = cv2.GaussianBlur(gray, (k, k), 0)

    # 二值化阶段将“线”和“非线区域”分开。
    if thresh >= 0:
        ttype = cv2.THRESH_BINARY if line_is_white else cv2.THRESH_BINARY_INV
        _, bin_img = cv2.threshold(gray, int(thresh), 255, ttype)
    else:
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


def detect_line_and_angle(
    bin_img,
    n_windows=9,
    margin=60,
    minpix=50,
    kp=1.0,
    roi_bottom_offset_ratio=0.25,
    roi_height_ratio=0.25,
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
    roi_height_ratio = min(0.95, max(0.05, float(roi_height_ratio)))
    roi_bottom = int(round(h * (1.0 - roi_bottom_offset_ratio)))
    roi_height = max(1, int(round(h * roi_height_ratio)))
    roi_top = max(0, roi_bottom - roi_height)
    roi_bottom = min(h, max(roi_top + 1, roi_bottom))

    bottom = bin_img[roi_top:roi_bottom, :]
    hist = np.sum(bottom > 0, axis=0).astype(np.int32)
    if hist.size == 0 or int(np.max(hist)) <= 0:
        vis = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR) if draw else None
        if vis is not None:
            cv2.rectangle(vis, (0, roi_top), (w - 1, roi_bottom - 1), (255, 128, 0), 2)
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
        cv2.rectangle(vis, (0, roi_top), (w - 1, roi_bottom - 1), (255, 128, 0), 2)

    for i in range(nw):
        # 逐层向上构建窗口区域。
        win_y_high = max(roi_top, roi_bottom - i * win_h)
        win_y_low = max(roi_top, roi_bottom - (i + 1) * win_h)
        win_y_high = min(roi_bottom, win_y_high)
        win_x_low = max(0, current_x - margin)
        win_x_high = min(w, current_x + margin)
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
    image_center_x = 0.5 * float(w - 1)
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


class LineFollowAngleNode(Node):
    """ROS2 巡线角度检测节点。"""

    def __init__(self):
        super().__init__("line_follow_angle_node")

        # 图像输入与算法参数都做成 ROS 参数，方便现场标定。
        self.declare_parameter("image_topic", "/hbmem_img")
        self.declare_parameter("image_msg_type", "hbmem")
        self.declare_parameter("line_is_white", False)
        self.declare_parameter("blur_ksize", 5)
        self.declare_parameter("thresh", -1)
        self.declare_parameter("morph_ksize", 3)
        self.declare_parameter("min_component_area", 20)
        self.declare_parameter("max_component_area", 0)
        self.declare_parameter("max_component_width_px", 0)
        self.declare_parameter("max_component_width_ratio", 0.0)
        self.declare_parameter("component_intensity_limit", -1.0)
        self.declare_parameter("n_windows", 9)
        self.declare_parameter("margin", 180)
        self.declare_parameter("minpix", 50)
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("angle_bias_deg", -10.0)
        self.declare_parameter("roi_bottom_offset_ratio", 0.25)
        self.declare_parameter("roi_height_ratio", 0.25)
        self.declare_parameter("show_debug", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_binary_topic", "/line_follow/debug_binary")
        self.declare_parameter("debug_detect_topic", "/line_follow/debug_detect")
        self.declare_parameter("debug_angle_curve_topic", "/line_follow/debug_angle_curve")
        self.declare_parameter("offset_px_topic", "/line_follow/line_offset_px")
        self.declare_parameter("offset_norm_topic", "/line_follow/line_offset_norm")
        self.declare_parameter("show_angle_curve", True)
        self.declare_parameter("angle_curve_history_size", 240)
        self.declare_parameter("angle_curve_limit_deg", 45.0)

        # CvBridge 用于在 ROS Image 和 OpenCV Mat 之间转换。
        self.bridge = CvBridge()
        self._load_fixed_endpoints()
        history_size = max(30, int(self.get_parameter("angle_curve_history_size").value))
        self.angle_history = deque(maxlen=history_size)
        self.display_available = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.warned_no_display = False
        self.detect_fail_streak = 0
        self.detect_success_count = 0
        self.detect_fail_count = 0
        self.publish_debug_image_enabled = False
        self.debug_binary_topic = str(self.get_parameter("debug_binary_topic").value)
        self.debug_detect_topic = str(self.get_parameter("debug_detect_topic").value)
        self.debug_angle_curve_topic = str(self.get_parameter("debug_angle_curve_topic").value)
        self.pub_binary = None
        self.pub_detect = None
        self.pub_angle_curve = None

        # 订阅相机图像。
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

        # 发布检测角度和建议转向量。
        self.pub_steer = self.create_publisher(Float32, "/line_follow/steer_angle_deg", 10)
        self.pub_angle = self.create_publisher(Float32, "/line_follow/line_angle_deg", 10)
        self.pub_offset_px = self.create_publisher(Float32, self.offset_px_topic, 10)
        self.pub_offset_norm = self.create_publisher(Float32, self.offset_norm_topic, 10)
        self._refresh_debug_runtime()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            "Subscribed to "
            f"{self.image_topic} ({subscribed_type}), publishing /line_follow/line_angle_deg, "
            f"{self.offset_norm_topic}, {self.offset_px_topic} and /line_follow/steer_angle_deg"
        )

    def _load_fixed_endpoints(self):
        """加载需要重启节点后才可改变的输入输出端点。"""
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.image_msg_type = str(self.get_parameter("image_msg_type").value).strip().lower()
        self.offset_px_topic = str(self.get_parameter("offset_px_topic").value)
        self.offset_norm_topic = str(self.get_parameter("offset_norm_topic").value)

    def _destroy_debug_publishers(self):
        """销毁当前调试图像发布器。"""
        for name in ("pub_binary", "pub_detect", "pub_angle_curve"):
            pub = getattr(self, name)
            if pub is not None:
                try:
                    self.destroy_publisher(pub)
                except Exception:
                    pass
                setattr(self, name, None)

    def _create_debug_publishers(self):
        """按当前参数创建调试图像发布器。"""
        self.pub_binary = self.create_publisher(Image, self.debug_binary_topic, 10)
        self.pub_detect = self.create_publisher(Image, self.debug_detect_topic, 10)
        self.pub_angle_curve = self.create_publisher(Image, self.debug_angle_curve_topic, 10)

    def _refresh_debug_runtime(self, overrides=None):
        """刷新调试发布器和曲线历史配置，支持运行时切换。"""
        values = {
            "publish_debug_image": bool(self.get_parameter("publish_debug_image").value),
            "debug_binary_topic": str(self.get_parameter("debug_binary_topic").value),
            "debug_detect_topic": str(self.get_parameter("debug_detect_topic").value),
            "debug_angle_curve_topic": str(self.get_parameter("debug_angle_curve_topic").value),
            "angle_curve_history_size": int(self.get_parameter("angle_curve_history_size").value),
        }
        if overrides:
            values.update(overrides)

        publish_debug_image = bool(values["publish_debug_image"])
        debug_binary_topic = str(values["debug_binary_topic"])
        debug_detect_topic = str(values["debug_detect_topic"])
        debug_angle_curve_topic = str(values["debug_angle_curve_topic"])
        history_size = max(30, int(values["angle_curve_history_size"]))

        current_history = list(getattr(self, "angle_history", []))
        if not hasattr(self, "angle_history") or self.angle_history.maxlen != history_size:
            self.angle_history = deque(current_history[-history_size:], maxlen=history_size)

        topics_changed = (
            debug_binary_topic != getattr(self, "debug_binary_topic", debug_binary_topic)
            or debug_detect_topic != getattr(self, "debug_detect_topic", debug_detect_topic)
            or debug_angle_curve_topic != getattr(self, "debug_angle_curve_topic", debug_angle_curve_topic)
        )
        self.debug_binary_topic = debug_binary_topic
        self.debug_detect_topic = debug_detect_topic
        self.debug_angle_curve_topic = debug_angle_curve_topic
        self.publish_debug_image_enabled = publish_debug_image

        if publish_debug_image:
            if topics_changed or self.pub_binary is None or self.pub_detect is None or self.pub_angle_curve is None:
                self._destroy_debug_publishers()
                self._create_debug_publishers()
        elif self.pub_binary is not None or self.pub_detect is not None or self.pub_angle_curve is not None:
            self._destroy_debug_publishers()

    def _on_set_parameters(self, params):
        """允许运行时切换调试发布器和曲线缓存。"""
        static_params = {"image_topic", "image_msg_type", "offset_px_topic", "offset_norm_topic"}
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
        """图像回调：每来一帧图像就执行一次巡线检测。"""
        try:
            if isinstance(msg, Image):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                frame = hbmem_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")
            return

        # 每帧都读取当前参数，便于现场通过 ros2 param 动态调试。
        line_is_white = bool(self.get_parameter("line_is_white").value)
        blur_ksize = int(self.get_parameter("blur_ksize").value)
        thresh = int(self.get_parameter("thresh").value)
        morph_ksize = int(self.get_parameter("morph_ksize").value)
        min_component_area = int(self.get_parameter("min_component_area").value)
        max_component_area = int(self.get_parameter("max_component_area").value)
        max_component_width_px = int(self.get_parameter("max_component_width_px").value)
        max_component_width_ratio = float(self.get_parameter("max_component_width_ratio").value)
        component_intensity_limit = float(self.get_parameter("component_intensity_limit").value)
        n_windows = int(self.get_parameter("n_windows").value)
        margin = int(self.get_parameter("margin").value)
        minpix = int(self.get_parameter("minpix").value)
        kp = float(self.get_parameter("kp").value)
        angle_bias_deg = float(self.get_parameter("angle_bias_deg").value)
        roi_bottom_offset_ratio = float(self.get_parameter("roi_bottom_offset_ratio").value)
        roi_height_ratio = float(self.get_parameter("roi_height_ratio").value)
        show = bool(self.get_parameter("show_debug").value)
        publish_debug_image = self.publish_debug_image_enabled
        show_angle_curve = bool(self.get_parameter("show_angle_curve").value)
        enable_local_windows = show and self.display_available
        need_detect_vis = enable_local_windows or (publish_debug_image and self.pub_detect is not None)

        bin_img = make_binary(
            frame,
            line_is_white,
            blur_ksize,
            thresh,
            morph_ksize,
            min_component_area,
            max_component_area,
            max_component_width_px,
            max_component_width_ratio,
            component_intensity_limit,
        )
        ok, angle_deg, steer_deg, offset_px, offset_norm, vis = detect_line_and_angle(
            bin_img,
            n_windows=n_windows,
            margin=margin,
            minpix=minpix,
            kp=kp,
            roi_bottom_offset_ratio=roi_bottom_offset_ratio,
            roi_height_ratio=roi_height_ratio,
            draw=need_detect_vis,
        )
        angle_deg += angle_bias_deg
        steer_deg = -float(kp) * angle_deg

        if ok:
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
        else:
            self.detect_fail_count += 1
            self.detect_fail_streak += 1
            if self.detect_fail_streak == 1 or self.detect_fail_streak % 30 == 0:
                white_ratio = float(np.count_nonzero(bin_img)) / float(bin_img.size) if bin_img.size else 0.0
                self.get_logger().warn(
                    "line detection failed: "
                    f"streak={self.detect_fail_streak}, white_ratio={white_ratio:.3f}, "
                    f"line_is_white={line_is_white}, thresh={thresh}, margin={margin}, "
                    f"minpix={minpix}, roi_bottom_offset_ratio={roi_bottom_offset_ratio:.2f}, "
                    f"roi_height_ratio={roi_height_ratio:.2f}"
                )

            # 丢线时持续发布“直行”保底命令，避免下游因无角度输入而停住。
            m1 = Float32()
            m1.data = 0.0
            self.pub_angle.publish(m1)

            m2 = Float32()
            m2.data = 0.0
            self.pub_steer.publish(m2)

            m3 = Float32()
            m3.data = 0.0
            self.pub_offset_px.publish(m3)

            m4 = Float32()
            m4.data = 0.0
            self.pub_offset_norm.publish(m4)

        header = getattr(msg, "header", None)

        if publish_debug_image and self.pub_binary is not None:
            try:
                binary_msg = self.bridge.cv2_to_imgmsg(bin_img, encoding="mono8")
                if header is not None:
                    binary_msg.header = header
                self.pub_binary.publish(binary_msg)
            except Exception as e:
                self.get_logger().warn(f"publish binary debug image failed: {e}")

        if publish_debug_image and self.pub_detect is not None:
            try:
                detect_vis = vis if vis is not None else cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
                detect_msg = self.bridge.cv2_to_imgmsg(detect_vis, encoding="bgr8")
                if header is not None:
                    detect_msg.header = header
                self.pub_detect.publish(detect_msg)
            except Exception as e:
                self.get_logger().warn(f"publish detect debug image failed: {e}")

        curve_vis = None
        if show_angle_curve:
            curve_vis = self.draw_angle_curve(angle_deg, ok)

        if publish_debug_image and self.pub_angle_curve is not None and curve_vis is not None:
            try:
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
