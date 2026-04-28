# USB Camera Calibration Tools

This directory contains a minimal end-to-end workflow for calibrating a USB camera:

1. Generate a printable chessboard target.
2. Capture calibration images from the USB camera.
3. Solve intrinsic parameters and lens distortion.
4. Save calibration results for later use.

## Files

- `generate_chessboard.py`: generate printable chessboard files in SVG, 300 DPI RGB PNG, and A4 PDF.
- `capture_calibration_images.py`: preview the USB camera and save valid calibration frames.
- `calibrate_camera.py`: run OpenCV calibration and export the results.

## Quick Start

```bash
python3 tools/usb_camera_calibration/generate_chessboard.py \
  --cols 9 \
  --rows 6 \
  --square-size-mm 25 \
  --output-dir tools/usb_camera_calibration/output/pattern

python3 tools/usb_camera_calibration/capture_calibration_images.py \
  --cols 9 \
  --rows 6 \
  --width 1920 \
  --height 1080 \
  --output-dir tools/usb_camera_calibration/output/images

python3 tools/usb_camera_calibration/calibrate_camera.py \
  --images "tools/usb_camera_calibration/output/images/*.jpg" \
  --cols 9 \
  --rows 6 \
  --square-size-mm 25 \
  --output tools/usb_camera_calibration/output/camera_calibration.json
```

The capture script probes only `video2` and `video3` by default, so it does
not touch a board-side camera on `video0`. If needed, pass `--camera 2` or
`--camera 3` to choose a specific laptop camera device.

On Windows 11, print the generated `*_a4_landscape.pdf` first. In the print
dialog, use 100% / actual size and disable fit-to-page scaling.

For the full step-by-step procedure, see `Markdown/USB相机标定指南.md`.
