# USB相机标定指南

本文提供一套可以直接落地的 USB 相机标定流程，从生成棋盘格、打印、采集图像，到求解内参和畸变系数，全部基于仓库里的脚本完成。

相关工具目录：

- `tools/usb_camera_calibration/`

目录内容：

- `generate_chessboard.py`：生成可打印棋盘格。
- `capture_calibration_images.py`：打开 USB 相机，实时检测棋盘格，按空格保存有效图片。
- `calibrate_camera.py`：读取采集图片，计算相机内参和畸变参数，导出 JSON 结果。

## 1. 标定前你需要知道的事

相机标定的目标，是求出下面这些参数：

- 焦距 `fx`、`fy`
- 主点 `cx`、`cy`
- 畸变参数 `k1`、`k2`、`p1`、`p2`、`k3` 等

这些参数主要用于：

- 消除桶形畸变、枕形畸变
- 让像素坐标与实际几何关系更稳定
- 给后续测距、定位、视觉控制提供可靠输入

如果你只是做简单巡线，标定不是绝对必需；但如果你后续要做更稳定的视觉几何计算，标定值得做。

## 2. 棋盘格规格怎么选

推荐先用下面这组参数：

- 内角点列数：`9`
- 内角点行数：`6`
- 方格边长：`25 mm`

说明：

- `9 x 6` 指的是“内角点”数量，不是黑白格数量。
- 实际打印出来的棋盘格格子数会比内角点多一圈，也就是 `10 x 7` 个格子。
- `25 mm` 是单个小格子的物理边长，打印之后要用尺子量一下，确认没有被打印机自动缩放。

## 3. 生成棋盘格

执行：

```bash
python3 tools/usb_camera_calibration/generate_chessboard.py \
  --cols 9 \
  --rows 6 \
  --square-size-mm 25 \
  --output-dir tools/usb_camera_calibration/output/pattern
```

生成结果：

- `tools/usb_camera_calibration/output/pattern/chessboard_9x6_25mm.svg`
- `tools/usb_camera_calibration/output/pattern/chessboard_9x6_25mm.png`
- `tools/usb_camera_calibration/output/pattern/chessboard_9x6_25mm_a4_landscape.pdf`

建议：

- 复制到 Windows 11 后，优先打印 `PDF`
- 在打印窗口里选择“实际大小”或 `100%`
- 关闭“适应页面”“自动缩放”“边距压缩”之类选项
- 打印完成后，用尺子测量任意一格的边长，确认接近 `25 mm`

如果实测不是 `25 mm`，不要继续拍。要么重新打印，要么在后续标定时把 `--square-size-mm` 改成真实测量值。

## 4. 打印和安装棋盘格

建议做法：

- 把棋盘格贴在硬纸板、亚克力板或木板上
- 保证棋盘格表面尽量平整
- 不要卷边、起泡、弯曲
- 棋盘格尽量占满相机视野的较大区域，但不要大到经常出画

错误做法：

- 打印在软纸上直接拿在手里晃
- 棋盘格表面反光严重
- 棋盘格有折痕
- 棋盘格边缘被裁掉

## 5. 采集标定图片

执行：

```bash
python3 tools/usb_camera_calibration/capture_calibration_images.py \
  --cols 9 \
  --rows 6 \
  --width 1920 \
  --height 1080 \
  --output-dir tools/usb_camera_calibration/output/images
```

默认只探测 `video2` 和 `video3`，不会打开板端常用的 `video0`。
如果已经确认笔记本相机编号，也可以显式指定 `--camera 2` 或 `--camera 3`。

运行后：

- 画面中检测到棋盘格时，会显示角点覆盖结果
- 按空格保存当前帧
- 按 `Q` 退出

### 5.1 怎么拍才是有效数据

建议至少保存 `15` 到 `25` 张图，最低不要少于 `8` 张。

这批图片要覆盖下面这些变化：

- 棋盘格位于画面中央
- 棋盘格位于左上、右上、左下、右下
- 棋盘格有轻微旋转
- 棋盘格有前后远近变化
- 棋盘格有一定倾斜角，但不要大到角点缺失

一句话原则：

- 要让棋盘格“遍布视野”
- 不要只在正中央拍一堆几乎一样的图

### 5.2 拍摄时的硬要求

- 每张图里，整个棋盘格尽量完整出现
- 图像不要糊
- 不要严重过曝或过暗
- 避免运动模糊
- 避免强反光

### 5.3 常见错误

- 相机和棋盘格距离始终不变
- 每张图姿态几乎一样
- 只拍中心区域
- 自动曝光一直在大幅抖动
- 图像边缘严重发虚

如果你的 USB 相机会自动对焦或自动曝光乱跳，先尽量把这些参数固定住，再采集。

## 6. 运行标定

执行：

```bash
python3 tools/usb_camera_calibration/calibrate_camera.py \
  --images "tools/usb_camera_calibration/output/images/*.jpg" \
  --cols 9 \
  --rows 6 \
  --square-size-mm 25 \
  --output tools/usb_camera_calibration/output/camera_calibration.json
```

输出内容：

- `tools/usb_camera_calibration/output/camera_calibration.json`
- `tools/usb_camera_calibration/output/preview/*_corners.jpg`
- `tools/usb_camera_calibration/output/preview/undistorted_preview.jpg`

其中：

- `*_corners.jpg` 用来检查每张图的角点检测是否正常
- `undistorted_preview.jpg` 用来粗看去畸变效果

## 7. 如何判断标定结果好不好

重点看两个指标：

- `rms_reprojection_error`
- `mean_reprojection_error_px`

经验上：

- 小于 `0.3 px`：很好
- `0.3 px` 到 `0.8 px`：通常可用
- 大于 `1.0 px`：要怀疑采集质量或棋盘格参数

这个标准不是绝对值，但足够指导你是否要重拍。

同时还要做人工检查：

- 去畸变后直线是否更直
- 图像边缘是否明显拉扯异常
- 标定矩阵数值是否离谱

如果你看到下面这些现象，通常说明结果不可信：

- 畸变系数大得异常
- 重投影误差很大
- 去畸变后画面反而更弯
- 只用了极少数有效图片

## 8. 输出结果里每个字段是什么意思

`camera_calibration.json` 主要字段说明：

- `image_size`：参与标定的图片分辨率
- `pattern`：棋盘格内角点数量和方格物理尺寸
- `valid_image_count`：真正用于求解的有效图片数
- `rms_reprojection_error`：OpenCV 返回的总体 RMS 重投影误差
- `mean_reprojection_error_px`：脚本额外计算的平均像素误差
- `camera_matrix`：相机内参矩阵
- `dist_coeffs`：畸变参数
- `optimal_new_camera_matrix`：去畸变优化后的新内参
- `roi`：去畸变后建议保留的有效图像区域

## 9. 结果怎么在后续程序里使用

如果你后续要在 Python/OpenCV 里做去畸变，典型用法如下：

```python
import json
import cv2
import numpy as np

cfg = json.load(open("tools/usb_camera_calibration/output/camera_calibration.json", "r", encoding="utf-8"))
camera_matrix = np.array(cfg["camera_matrix"], dtype=np.float64)
dist_coeffs = np.array(cfg["dist_coeffs"], dtype=np.float64)

frame = cv2.imread("test.jpg")
undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
```

注意：

- 标定参数只对“同一台相机、同一镜头状态、同一分辨率”有效
- 如果你把分辨率从 `1920x1080` 改成 `640x480`，最好重新标定
- 如果镜头焦距、对焦位置、安装结构发生明显变化，也建议重新标定

## 10. 推荐的实操流程

建议按这个顺序做：

1. 生成棋盘格
2. 打印并测量实际方格尺寸
3. 固定相机分辨率
4. 采集 `15` 到 `25` 张高质量图片
5. 运行标定
6. 检查误差和去畸变预览
7. 如果效果不好，删掉图片重拍，不要在坏数据上硬调

## 11. 最常见的问题排查

### 11.1 检测不到棋盘格

先检查：

- `--cols` 和 `--rows` 是否与打印棋盘格一致
- 棋盘格是否完整入镜
- 是否反光、过曝、模糊
- 是否距离过近导致局部裁切

### 11.2 能检测到，但标定误差很大

常见原因：

- 图片太少
- 大部分图片姿态重复
- 只有中心视角，没有边缘视角
- 方格物理尺寸填错
- 打印时被缩放了

### 11.3 去畸变效果看起来不对

先确认：

- 标定分辨率和运行分辨率是否一致
- 标定时是否使用了同一台相机
- 标定图是否包含足够的边缘覆盖

## 12. 命令汇总

生成棋盘格：

```bash
python3 tools/usb_camera_calibration/generate_chessboard.py \
  --cols 9 \
  --rows 6 \
  --square-size-mm 25 \
  --output-dir tools/usb_camera_calibration/output/pattern
```

采集图片：

```bash
python3 tools/usb_camera_calibration/capture_calibration_images.py \
  --cols 9 \
  --rows 6 \
  --width 1920 \
  --height 1080 \
  --output-dir tools/usb_camera_calibration/output/images
```

运行标定：

```bash
python3 tools/usb_camera_calibration/calibrate_camera.py \
  --images "tools/usb_camera_calibration/output/images/*.jpg" \
  --cols 9 \
  --rows 6 \
  --square-size-mm 25 \
  --output tools/usb_camera_calibration/output/camera_calibration.json
```

如果你后面要把这套标定结果接到当前巡线程序里，我可以继续帮你把相机去畸变预处理也接进现有图像链路。 
