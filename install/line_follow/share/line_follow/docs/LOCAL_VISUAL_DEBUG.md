# 本地 OpenCV 视觉调试说明

本文档用于在开发机上脱离 ROS2 和开发板，直接验证巡线图像算法。

适用场景：

- 先在本地摄像头/图片上验证二值化和角度计算
- 不想每次都同步到板端再调参数
- 想快速确认是“算法问题”还是“ROS/驱动链路问题”

对应脚本：

- `tools/opencv_line_verify.py`

---

## 1. 目标

本地调试脚本会复用项目里的核心视觉处理逻辑：

- 灰度化
- 模糊
- 二值化
- 形态学去噪
- 滑窗找线
- 拟合角度

这样你在开发机上调好的参数，后续更容易迁移到 ROS2 节点。

---

## 2. 环境准备

开发机需要：

- Python 3
- OpenCV
- NumPy

建议在开发机先确认：

```bash
python3 -c "import cv2, numpy; print('ok')"
```

如果报错，再先安装依赖。

---

## 3. 支持的输入方式

脚本支持两种输入：

- USB 摄像头实时画面
- 单张本地图像

---

## 4. 最简单的实时调试

如果开发机接了 USB 摄像头：

```bash
cd /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow
python3 tools/opencv_line_verify.py --camera 0
```

如果不是 `0` 号摄像头，可以改成：

```bash
python3 tools/opencv_line_verify.py --camera 1
```

运行后会看到调试窗口，通常包括：

- 原图
- 二值图
- 检测结果图

---

## 5. 用图片调试

如果你已经拍了一张现场图像：

```bash
cd /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow
python3 tools/opencv_line_verify.py --image /path/to/test.png
```

这样适合反复观察同一帧效果。

---

## 6. 常用调参方式

### 6.1 黑线 / 白线

黑线场景：

```bash
python3 tools/opencv_line_verify.py \
  --camera 0 \
  --line-is-white false
```

白线场景：

```bash
python3 tools/opencv_line_verify.py \
  --camera 0 \
  --line-is-white true
```

### 6.2 固定阈值

```bash
python3 tools/opencv_line_verify.py \
  --camera 0 \
  --thresh 120
```

如果不写 `--thresh`，默认通常使用自动阈值策略。

### 6.3 调整模糊核

```bash
python3 tools/opencv_line_verify.py \
  --camera 0 \
  --blur-ksize 7
```

### 6.4 调整形态学核

```bash
python3 tools/opencv_line_verify.py \
  --camera 0 \
  --morph-ksize 5
```

---

## 7. 连通域过滤

如果地面噪声很多，可以调这些参数：

```bash
python3 tools/opencv_line_verify.py \
  --camera 0 \
  --min-component-area 150 \
  --max-component-width-ratio 0.6
```

典型用途：

- `min-component-area`：去掉太小的噪点
- `max-component-width-ratio`：去掉横向铺满的大块误检

---

## 8. ROI 调试

当前默认检测 ROI 是图像最下面三分之一高度：

- 从图像底边开始
- 到图像 `1/3` 高度处结束

如果你想进一步验证 ROI 是否合适，可以直接观察检测窗口中的 ROI 框和滑窗位置。

---

## 9. 常见问题

### 9.1 能看到线，但检测失败

优先检查：

- `line_is_white` 是否设置正确
- 阈值是否太高或太低
- 形态学是否把线条腐蚀掉了
- 连通域过滤是否过严

### 9.2 检测结果乱跳

优先尝试：

- 增大 `blur_ksize`
- 增大 `morph_ksize`
- 提高 `min-component-area`

### 9.3 摄像头打不开

先试：

```bash
python3 tools/opencv_line_verify.py --camera 1
python3 tools/opencv_line_verify.py --camera 2
```

---

## 10. 建议流程

推荐顺序：

1. 先用本地图片调到能稳定检线
2. 再用本地摄像头看实时效果
3. 最后再同步到板端跑 ROS2 全链路

这样能明显减少“算法问题”和“系统集成问题”混在一起的排查成本。
