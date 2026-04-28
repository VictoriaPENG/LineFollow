#!/usr/bin/env python3
"""OpenCV-only line-follow verifier.

Run the same core image-processing logic without ROS so the detector can be
checked on a laptop using a camera, a video file, or a single image.
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


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
    """按面积、宽度和灰度约束过滤二值连通域。"""
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
    line_is_white=False,
    blur_ksize=5,
    thresh=-1,
    morph_ksize=3,
    min_component_area=10,
    max_component_area=40000,
    max_component_width_px=80,
    max_component_width_ratio=0.0,
    component_intensity_limit=-1,
):
    """完成灰度、模糊、阈值和形态学处理，生成巡线二值图。"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    k = max(1, int(blur_ksize))
    if k % 2 == 0:
        k += 1
    gray = cv2.GaussianBlur(gray, (k, k), 0)

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
    roi_y_start_ratio=0.5,
    draw=False,
):
    """在 ROI 中搜索线条中心并估计角度与横向偏移。"""
    h, w = bin_img.shape[:2]
    roi_y_start_ratio = min(0.95, max(0.0, float(roi_y_start_ratio)))
    y0 = int(round(h * roi_y_start_ratio))
    bottom = bin_img[y0:h, :]
    hist = np.sum(bottom > 0, axis=0).astype(np.int32)
    if hist.size == 0 or int(np.max(hist)) <= 0:
        vis = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR) if draw else None
        if vis is not None:
            cv2.line(vis, (0, y0), (w - 1, y0), (255, 128, 0), 2)
            cv2.putText(
                vis,
                "line not found in bottom ROI",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
        return False, 0.0, 0.0, vis

    base_x = int(np.argmax(hist))
    nw = max(3, int(n_windows))
    win_h = h // nw
    margin = max(10, int(margin))
    minpix = max(10, int(minpix))

    current_x = base_x
    centers = []
    vis = None
    if draw:
        vis = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
        cv2.line(vis, (0, y0), (w - 1, y0), (255, 128, 0), 2)

    for i in range(nw):
        win_y_low = max(0, h - (i + 1) * win_h)
        win_y_high = min(h, h - i * win_h)
        win_x_low = max(0, current_x - margin)
        win_x_high = min(w, current_x + margin)
        if win_x_high <= win_x_low or win_y_high <= win_y_low:
            continue

        roi = bin_img[win_y_low:win_y_high, win_x_low:win_x_high]
        nz = cv2.findNonZero(roi)
        if nz is not None and len(nz) >= minpix:
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

    if len(centers) < 3:
        return False, 0.0, 0.0, vis

    centers_np = np.array(centers, dtype=np.float32).reshape(-1, 1, 2)
    vx, vy, x0f, y0f = cv2.fitLine(centers_np, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    angle = math.atan2(float(vx), float(-vy))
    angle_deg = angle * 180.0 / math.pi
    steer_deg = -float(kp) * angle_deg

    if draw and vis is not None:
        eps = 1e-6
        k = 1e6 if abs(vy) < eps else (vx / vy)
        y1, y2 = h - 1, 0
        x1 = int(round(x0f + (y1 - y0f) * k))
        x2 = int(round(x0f + (y2 - y0f) * k))
        cv2.line(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
        txt = f"angle={angle_deg:.2f} deg  steer={steer_deg:.2f} deg"
        cv2.putText(vis, txt, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    return True, angle_deg, steer_deg, vis


def parse_args():
    """解析离线验证工具参数。"""
    parser = argparse.ArgumentParser(description="Verify line-follow angle detection with OpenCV only.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", help="Path to a single image")
    source.add_argument("--video", help="Path to a video file")
    source.add_argument("--camera", type=int, help="Camera index, for example 0")
    parser.add_argument("--line-is-white", action="store_false", help="Target line is white on dark background")
    parser.add_argument("--blur-ksize", type=int, default=5)
    parser.add_argument("--thresh", type=int, default=-1, help="Threshold; -1 means Otsu")
    parser.add_argument("--morph-ksize", type=int, default=3)
    parser.add_argument("--min-component-area", type=int, default=10)
    parser.add_argument("--max-component-area", type=int, default=10000)
    parser.add_argument("--max-component-width-px", type=int, default=100)
    parser.add_argument("--max-component-width-ratio", type=float, default=0.0)
    parser.add_argument(
        "--component-intensity-limit",
        type=float,
        default=-1.0,
        help="For black-line mode keep mean gray <= this limit; for white-line mode keep mean gray >= this limit",
    )
    parser.add_argument("--n-windows", type=int, default=20)
    parser.add_argument("--margin", type=int, default=600)
    parser.add_argument("--minpix", type=int, default=20)
    parser.add_argument("--kp", type=float, default=1.0)
    parser.add_argument("--roi-y-start-ratio", type=float, default=0.35)
    parser.add_argument("--save-dir", help="Optional directory to save binary/detect outputs")
    return parser.parse_args()


def annotate_frame(frame, ok, angle_deg, steer_deg):
    """在图像上叠加检测状态和角度文本。"""
    text = f"OK angle={angle_deg:.2f} steer={steer_deg:.2f}" if ok else "LOST"
    color = (0, 255, 0) if ok else (0, 0, 255)
    vis = frame.copy()
    cv2.putText(vis, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    return vis


def ensure_save_dir(save_dir):
    """按需创建输出目录。"""
    if not save_dir:
        return None
    path = Path(save_dir)
    path.mkdir(parents=True, exist_ok=True)
    return path


def process_frame(frame, args):
    """对单帧执行完整检测，并返回调试中间结果。"""
    bin_img = make_binary(
        frame,
        line_is_white=args.line_is_white,
        blur_ksize=args.blur_ksize,
        thresh=args.thresh,
        morph_ksize=args.morph_ksize,
        min_component_area=args.min_component_area,
        max_component_area=args.max_component_area,
        max_component_width_px=args.max_component_width_px,
        max_component_width_ratio=args.max_component_width_ratio,
        component_intensity_limit=args.component_intensity_limit,
    )
    ok, angle_deg, steer_deg, detect_vis = detect_line_and_angle(
        bin_img,
        n_windows=args.n_windows,
        margin=args.margin,
        minpix=args.minpix,
        kp=args.kp,
        roi_y_start_ratio=args.roi_y_start_ratio,
        draw=True,
    )
    overlay = annotate_frame(frame, ok, angle_deg, steer_deg)
    return bin_img, detect_vis, overlay, ok, angle_deg, steer_deg


def run_single_image(frame, args, save_dir):
    """处理单张图片，并在需要时把结果保存到磁盘。"""
    bin_img, detect_vis, overlay, ok, angle_deg, steer_deg = process_frame(frame, args)
    print(f"ok={ok} angle_deg={angle_deg:.2f} steer_deg={steer_deg:.2f}")

    cv2.imshow("source", overlay)
    cv2.imshow("binary", bin_img)
    if detect_vis is not None:
        cv2.imshow("detect", detect_vis)

    if save_dir is not None:
        cv2.imwrite(str(save_dir / "source_overlay.png"), overlay)
        cv2.imwrite(str(save_dir / "binary.png"), bin_img)
        if detect_vis is not None:
            cv2.imwrite(str(save_dir / "detect.png"), detect_vis)

    cv2.waitKey(0)


def run_stream(cap, args, save_dir):
    """处理摄像头或视频流，实时显示巡线检测结果。"""
    frame_idx = 0
    while True:
        ok_read, frame = cap.read()
        if not ok_read:
            break

        bin_img, detect_vis, overlay, ok, angle_deg, steer_deg = process_frame(frame, args)
        print(f"frame={frame_idx} ok={ok} angle_deg={angle_deg:.2f} steer_deg={steer_deg:.2f}")

        cv2.imshow("source", overlay)
        cv2.imshow("binary", bin_img)
        if detect_vis is not None:
            cv2.imshow("detect", detect_vis)

        if save_dir is not None and frame_idx == 0:
            cv2.imwrite(str(save_dir / "source_overlay.png"), overlay)
            cv2.imwrite(str(save_dir / "binary.png"), bin_img)
            if detect_vis is not None:
                cv2.imwrite(str(save_dir / "detect.png"), detect_vis)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        frame_idx += 1


def main():
    """离线 OpenCV 验证工具入口。"""
    args = parse_args()
    save_dir = ensure_save_dir(args.save_dir)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"failed to read image: {args.image}")
        run_single_image(frame, args, save_dir)
    else:
        source = args.video if args.video else args.camera
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise SystemExit(f"failed to open source: {source}")
        try:
            run_stream(cap, args, save_dir)
        finally:
            cap.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
