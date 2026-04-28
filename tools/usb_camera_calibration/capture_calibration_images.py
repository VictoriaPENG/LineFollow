#!/usr/bin/env python3
"""从 USB 相机采集有效的棋盘格标定图片。"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

DEFAULT_CAMERA_CANDIDATES = (2, 3)


def detect_corners(gray, pattern_size):
    """检测棋盘格角点，供实时预览和样本筛选使用。"""
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, None)
        if found:
            return found, corners

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        40,
        0.001,
    )
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def parse_args() -> argparse.Namespace:
    """解析采集脚本参数。"""
    parser = argparse.ArgumentParser(description="Capture chessboard images for camera calibration.")
    parser.add_argument(
        "--camera",
        type=int,
        default=None,
        help="USB camera index. If omitted, only video2 and video3 are probed.",
    )
    parser.add_argument(
        "--allow-video0",
        action="store_true",
        help="Allow opening video0 explicitly. Disabled by default to avoid touching the board camera.",
    )
    parser.add_argument("--cols", type=int, default=9, help="Inner corner columns.")
    parser.add_argument("--rows", type=int, default=6, help="Inner corner rows.")
    parser.add_argument("--width", type=int, default=1920, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=1080, help="Requested capture height.")
    parser.add_argument(
        "--preview-scale",
        type=float,
        default=0.5,
        help="Preview window scale. Saved images keep the original capture size.",
    )
    parser.add_argument("--save-prefix", default="calib", help="Saved image file prefix.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("tools/usb_camera_calibration/output/images"),
        help="Directory to store calibration images.",
    )
    parser.add_argument(
        "--min-delay-sec",
        type=float,
        default=0.5,
        help="Minimum time between saved frames.",
    )
    return parser.parse_args()


def open_camera(index: int, width: int, height: int):
    """用 V4L2/MJPG 打开指定相机，并确认能读到第一帧。"""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None, None

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    ok, frame = cap.read()
    if not ok:
        cap.release()
        return None, None

    return cap, frame


def main() -> None:
    """打开相机预览，按空格保存已检测到角点的画面。"""
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    pattern_size = (args.cols, args.rows)
    if args.camera == 0 and not args.allow_video0:
        raise RuntimeError("refuse to open video0; pass --allow-video0 only if this is intentional")

    camera_candidates = (args.camera,) if args.camera is not None else DEFAULT_CAMERA_CANDIDATES
    cap = None
    frame = None
    camera_index = None
    for candidate in camera_candidates:
        cap, frame = open_camera(candidate, args.width, args.height)
        if cap is not None:
            camera_index = candidate
            break

    if cap is None or frame is None or camera_index is None:
        probed = ", ".join(f"video{candidate}" for candidate in camera_candidates)
        raise RuntimeError(f"failed to open a readable camera from: {probed}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera opened: index={camera_index}, actual_size={actual_w}x{actual_h}, backend=V4L2, fourcc=MJPG")
    print(f"Preview scale: {args.preview_scale}")
    print("Controls: press SPACE to save current frame when corners are detected, press Q to quit.")

    saved_count = 0
    last_save_time = 0.0

    while True:
        ok, current_frame = cap.read()
        if not ok:
            raise RuntimeError("failed to read frame from camera")

        frame = current_frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = detect_corners(gray, pattern_size)

        preview = frame.copy()
        if found:
            cv2.drawChessboardCorners(preview, pattern_size, corners, found)
            status = f"corners found | saved={saved_count}"
            color = (0, 220, 0)
        else:
            status = f"corners not found | saved={saved_count}"
            color = (0, 0, 255)

        cv2.putText(
            preview,
            status,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            "SPACE: save frame   Q: quit",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if args.preview_scale != 1.0:
            preview_w = max(1, int(preview.shape[1] * args.preview_scale))
            preview_h = max(1, int(preview.shape[0] * args.preview_scale))
            preview = cv2.resize(preview, (preview_w, preview_h), interpolation=cv2.INTER_AREA)

        cv2.imshow("usb_camera_calibration_capture", preview)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            break
        # 只在当前帧检测到完整角点时保存，避免采进无效样本。
        if key == ord(" ") and found:
            now = time.time()
            if now - last_save_time < args.min_delay_sec:
                continue
            path = args.output_dir / f"{args.save_prefix}_{saved_count:03d}.jpg"
            if not cv2.imwrite(str(path), frame):
                raise RuntimeError(f"failed to save frame to {path}")
            saved_count += 1
            last_save_time = now
            print(f"Saved: {path}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"Capture finished, total saved images: {saved_count}")


if __name__ == "__main__":
    main()
