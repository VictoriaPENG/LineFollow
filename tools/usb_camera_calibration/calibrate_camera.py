#!/usr/bin/env python3
"""根据棋盘格图片执行相机标定。"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np


def detect_corners(gray, pattern_size):
    """优先使用更稳健的新接口检测角点，失败时回退到传统接口。"""
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


def build_object_points(cols: int, rows: int, square_size_mm: float) -> np.ndarray:
    """构造棋盘格在真实世界坐标系下的角点坐标。"""
    objp = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] = grid * square_size_mm
    return objp


def compute_mean_reprojection_error(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs):
    """计算平均重投影误差，便于快速判断标定质量。"""
    total_error = 0.0
    for objp, imgp, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
        error = cv2.norm(imgp, projected, cv2.NORM_L2) / len(projected)
        total_error += error
    return total_error / max(len(objpoints), 1)


def parse_args() -> argparse.Namespace:
    """解析标定参数以及输入输出路径。"""
    parser = argparse.ArgumentParser(description="Calibrate a camera from chessboard images.")
    parser.add_argument("--images", required=True, help="Glob pattern for calibration images.")
    parser.add_argument("--cols", type=int, default=9, help="Inner corner columns.")
    parser.add_argument("--rows", type=int, default=6, help="Inner corner rows.")
    parser.add_argument(
        "--square-size-mm",
        type=float,
        default=25.0,
        help="Physical size of one chessboard square in millimeters.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("tools/usb_camera_calibration/output/camera_calibration.json"),
        help="Path to the JSON calibration output.",
    )
    parser.add_argument(
        "--preview-dir",
        type=Path,
        default=Path("tools/usb_camera_calibration/output/preview"),
        help="Directory for debug overlays and undistorted preview.",
    )
    return parser.parse_args()


def main() -> None:
    """读取图片、筛选有效样本并输出标定结果。"""
    args = parse_args()
    image_paths = sorted(glob.glob(args.images))
    if not image_paths:
        raise FileNotFoundError(f"no images matched: {args.images}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.preview_dir.mkdir(parents=True, exist_ok=True)

    pattern_size = (args.cols, args.rows)
    base_object_points = build_object_points(args.cols, args.rows, args.square_size_mm)

    objpoints = []
    imgpoints = []
    accepted_images = []
    image_size = None

    for image_path in image_paths:
        image = cv2.imread(image_path)
        if image is None:
            print(f"Skip unreadable image: {image_path}")
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])

        found, corners = detect_corners(gray, pattern_size)
        overlay = image.copy()
        if found:
            objpoints.append(base_object_points.copy())
            imgpoints.append(corners)
            accepted_images.append(image_path)
            cv2.drawChessboardCorners(overlay, pattern_size, corners, found)
            overlay_path = args.preview_dir / (Path(image_path).stem + "_corners.jpg")
            cv2.imwrite(str(overlay_path), overlay)
            print(f"Accepted: {image_path}")
        else:
            print(f"Rejected: {image_path}")

    if len(objpoints) < 8:
        raise RuntimeError(
            f"only {len(objpoints)} valid images found, at least 8 are recommended for calibration"
        )
    if image_size is None:
        raise RuntimeError("unable to determine image size from inputs")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )
    mean_error = compute_mean_reprojection_error(
        objpoints,
        imgpoints,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    first_image = cv2.imread(accepted_images[0])
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        image_size,
        1.0,
        image_size,
    )
    undistorted = cv2.undistort(first_image, camera_matrix, dist_coeffs, None, new_camera_matrix)
    undistorted_path = args.preview_dir / "undistorted_preview.jpg"
    cv2.imwrite(str(undistorted_path), undistorted)

    result = {
        "image_size": {"width": image_size[0], "height": image_size[1]},
        "pattern": {
            "inner_corners_cols": args.cols,
            "inner_corners_rows": args.rows,
            "square_size_mm": args.square_size_mm,
        },
        "valid_image_count": len(accepted_images),
        "accepted_images": accepted_images,
        "rms_reprojection_error": float(rms),
        "mean_reprojection_error_px": float(mean_error),
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "optimal_new_camera_matrix": new_camera_matrix.tolist(),
        "roi": {
            "x": int(roi[0]),
            "y": int(roi[1]),
            "width": int(roi[2]),
            "height": int(roi[3]),
        },
        "undistorted_preview": str(undistorted_path),
    }

    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Calibration result written: {args.output}")
    print(f"Valid images: {len(accepted_images)}")
    print(f"RMS reprojection error: {rms:.6f}")
    print(f"Mean reprojection error: {mean_error:.6f} px")
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(dist_coeffs.reshape(-1))


if __name__ == "__main__":
    main()
