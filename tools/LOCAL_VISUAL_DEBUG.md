# 本地调试视觉部分程序

本文档用于在开发机本地单独调试视觉巡线算法，不依赖 ROS2、相机 launch、底盘驱动和电机控制。核心目标是先把图像二值化、滑动窗口搜索和角度检测调通，再部署到 RDK X5 做整机联调。

当前仓库已经提供了一个纯 OpenCV 调试脚本：

- `tools/opencv_line_verify.py`

这个脚本复用了视觉节点中的核心图像处理逻辑，支持三种输入源：

- 单张图片
- 本地视频文件
- 本机摄像头

## 1. 适用场景

建议在以下情况下优先使用本地调试：

- 还没有把程序部署到 RDK X5
- 视觉参数明显没调好，整机联调成本太高
- 需要快速验证黑线/白线、阈值和 ROI 是否合理
- 需要录制样本视频后反复回放调参

本地调试只验证视觉检测链路，不验证：

- ROS2 图像话题订阅
- 串口驱动和 Modbus 通信
- 电机差速控制

## 2. 环境准备

开发机需要具备：

- Python3
- OpenCV Python 模块 `cv2`
- `numpy`
- 可用的桌面显示环境

建议先做一次最小检查：

```bash
python3 -c "import cv2, numpy; print('opencv local debug env ok')"
```

如果你准备直接调用本机摄像头，再检查摄像头设备：

```bash
ls /dev/video*
```

## 3. 输入数据准备

推荐优先用录制好的图片或视频调参，而不是一开始就接实时摄像头。原因很简单：同一份样本可重复回放，便于稳定比较参数效果。

建议准备三类样本：

- 正常光照下的直线
- 弯道或大角度偏转
- 阴影、反光、地面纹理干扰较强的场景

如果现场是黑线白底，就准备黑线样本；如果是白线黑底，就准备白线样本。

## 4. 基本用法

进入仓库目录：

```bash
cd /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow
```

### 4.1 调试单张图片

```bash
python3 tools/opencv_line_verify.py \
  --image /home/yunbo/Downloads/1.jpeg
```

适合做静态参数初调。程序会弹出三个窗口：

- `source`：原图叠加检测结果
- `binary`：二值化结果
- `detect`：滑动窗口和拟合线可视化

终端会打印：

```text
ok=True angle_deg=xx.xx steer_deg=xx.xx
```

其中：

- `ok=True` 表示当前图像检测成功
- `angle_deg` 是线相对车体前向的方向角
- `steer_deg` 是按 `kp` 换算后的建议转向角

### 4.2 调试视频文件

```bash
python3 tools/opencv_line_verify.py \
  --video /path/to/test.mp4
```

适合观察连续多帧稳定性。终端会逐帧打印：

```text
frame=0 ok=True angle_deg=... steer_deg=...
```

退出方式：

- 按 `q`
- 按 `Esc`

### 4.3 调试本机摄像头

```bash
python3 tools/opencv_line_verify.py \
  --camera 2
```

这里的 `0` 是 OpenCV 摄像头索引，不一定等同于 ROS 或系统里的 `/dev/video0` 编号。如果打不开，可以依次试：

```bash
python3 tools/opencv_line_verify.py --camera 1
python3 tools/opencv_line_verify.py --camera 2
```

## 5. 常用参数

脚本参数与 ROS 视觉节点的核心参数基本保持一致，所以本地调好的值可以直接迁移到 `line_follow_angle_node`。

### 5.1 黑线或白线

默认不加 `--line-is-white` 时，按黑线白底处理。

黑线白底：

```bash
python3 tools/opencv_line_verify.py --image /path/to/test.png
```

白线黑底：

```bash
python3 tools/opencv_line_verify.py \
  --image /path/to/test.png \
  --line-is-white
```

如果线和背景极性选反，`binary` 窗口通常会明显不对，要么整幅图接近全黑，要么大量背景被误检成线。

### 5.2 阈值 `--thresh`

- 默认值 `-1`，表示使用 Otsu 自动阈值
- 光照变化不大时，可尝试固定阈值提升稳定性

示例：

```bash
python3 tools/opencv_line_verify.py \
  --image /path/to/test.png \
  --thresh 120
```

判断依据：

- 线条区域应尽量连贯
- 背景噪点应尽量少

### 5.3 模糊核 `--blur-ksize`

用于抑制纹理和散点噪声：

- 值太小：噪点多
- 值太大：线边界会变钝，细线可能被抹掉

示例：

```bash
python3 tools/opencv_line_verify.py \
  --video /path/to/test.mp4 \
  --blur-ksize 7
```

### 5.4 形态学核 `--morph-ksize`

用于开闭运算去噪和补洞：

- 值太小：碎点保留太多
- 值太大：细线可能被破坏或连成一片

示例：

```bash
python3 tools/opencv_line_verify.py \
  --image /path/to/test.png \
  --morph-ksize 5
```

### 5.5 ROI 起点 `--roi-y-start-ratio`

默认值是 `0.5`，表示只在图像下半部分做底部直方图统计。

示例：

```bash
python3 tools/opencv_line_verify.py \
  --video /path/to/test.mp4 \
  --roi-y-start-ratio 0.35
```

经验：

- `0.5`：抗干扰更强，适合线条在画面下方比较明显的情况
- `0.35`：更容易提早看到远处线条
- `0.2`：ROI 更大，但阴影和纹理干扰也会更明显

如果经常提示 `line not found in bottom ROI`，优先尝试减小这个值。

### 5.6 滑动窗口参数

常用参数如下：

- `--n-windows`：窗口层数
- `--margin`：每层窗口向左右搜索的宽度
- `--minpix`：认为该窗口找到线所需的最小像素数

示例：

```bash
python3 tools/opencv_line_verify.py \
  --video /path/to/test.mp4 \
  --margin 300 \
  --minpix 50 \
  --n-windows 9
```

调参建议：

- 弯道大、线条横向变化快时，适当增大 `--margin`
- 噪声多时，适当增大 `--minpix`
- 图像分辨率较高且线条跨越范围大时，可适当增加 `--n-windows`

### 5.7 比例系数 `--kp`

这个参数不会影响线是否检测成功，只影响 `steer_deg` 的数值大小。

示例：

```bash
python3 tools/opencv_line_verify.py \
  --image /path/to/test.png \
  --kp 1.2
```

如果当前阶段只关心视觉角度本身，这个参数可以先保持默认。

## 6. 保存调试结果

如果希望把首帧结果保存下来，便于对比不同参数，可增加 `--save-dir`：

```bash
python3 tools/opencv_line_verify.py \
  --camera 2
  --save-dir /tmp/line_follow_debug
```

输出目录中会生成：

- `source_overlay.png`
- `binary.png`
- `detect.png`

这对比参数前后效果很有用，尤其适合整理现场标定记录。

## 7. 推荐调试顺序

建议按下面顺序进行，不要一开始同时改很多参数。

### 7.1 先确认极性

先只判断一件事：

- 是黑线白底
- 还是白线黑底

如果这一步错了，后面其他参数基本都没有意义。

### 7.2 再看二值化

优先观察 `binary` 窗口：

- 目标线是否连续
- 背景是否干净

这一步主要调：

- `--thresh`
- `--blur-ksize`
- `--morph-ksize`

### 7.3 再看滑动窗口是否跟在线上

观察 `detect` 窗口：

- 绿色框是否基本沿着目标线往上走
- 红点是否落在目标线中心附近
- 黄色拟合线是否和实际线方向一致

这一步主要调：

- `--margin`
- `--minpix`
- `--n-windows`
- `--roi-y-start-ratio`

### 7.4 最后再看角度稳定性

当检测已经基本可靠后，再回放视频观察终端中的 `angle_deg` 是否平稳，是否会频繁跳变。

## 8. 从本地参数迁移到 ROS 节点

本地调好以后，可把参数直接映射到 ROS2 节点。例如本地验证命令：

```bash
python3 tools/opencv_line_verify.py \
  --video /path/to/test.mp4 \
  --thresh 110 \
  --blur-ksize 5 \
  --morph-ksize 3 \
  --margin 300 \
  --minpix 50 \
  --roi-y-start-ratio 0.35
```

对应到 ROS 节点参数：

- `--thresh` -> `thresh`
- `--blur-ksize` -> `blur_ksize`
- `--morph-ksize` -> `morph_ksize`
- `--margin` -> `margin`
- `--minpix` -> `minpix`
- `--roi-y-start-ratio` -> `roi_y_start_ratio`
- `--line-is-white` -> `line_is_white`
- `--kp` -> `kp`

后续在 RDK X5 上可直接这样带参启动：

```bash
ros2 run line_follow line_follow_angle_node \
  --ros-args \
  -p image_topic:=/hbmem_img \
  -p image_msg_type:=hbmem \
  -p line_is_white:=false \
  -p thresh:=110 \
  -p blur_ksize:=5 \
  -p morph_ksize:=3 \
  -p margin:=300 \
  -p minpix:=50 \
  -p roi_y_start_ratio:=0.35
```

## 9. 常见问题

### 9.1 程序能跑，但窗口不显示

通常是因为当前环境没有桌面显示能力，比如纯 SSH 终端或服务器环境。这个脚本依赖 OpenCV GUI 窗口，本地调试建议在有图形界面的电脑上执行。

### 9.2 视频能打开，但角度一直是 0

优先检查：

- 线条极性是否选反
- 二值图是否基本正确
- `roi_y_start_ratio` 是否过大，导致线没有进入底部统计区
- `margin` 是否太小，窗口跟丢线

### 9.3 `ok=False` 偶尔出现，是否一定有问题

不一定。单帧丢失在阴影、反光、快速抖动场景里是正常现象。更重要的是看连续视频里是否频繁丢失，以及恢复速度是否足够快。

### 9.4 为什么建议先用视频调，而不是直接接摄像头

因为视频样本可重复。你改一组参数后，可以对同一段画面做前后对比，这比实时摄像头更容易定位问题。

## 10. 与整机联调文档的关系

本文档只负责本地视觉调试。

如果你已经把视觉部分调通，下一步请看：

- `RDK_X5_DEPLOY_AND_VERIFY.md`

建议流程是：

1. 先用本文档把视觉参数调到基本可用
2. 再部署到 RDK X5 验证 ROS 图像链路
3. 最后再接入运动模型和电机驱动做全链路联调
