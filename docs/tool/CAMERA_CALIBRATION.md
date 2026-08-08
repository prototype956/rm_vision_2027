# MindVision 相机内参标定

## 标定前准备

工具固定使用 `src/config/hal/camera/mindvision.yaml` 中的相机参数，并要求实际输出为
1280x720 BGR8。标定完成前不要改变镜头、焦距、对焦、硬件 ROI、分辨率或曝光模式。

默认标定板为 9x6 内角点、25 mm 方格。这里的 9x6 是内角点数量，不是黑白格数量；
实物尺寸不同时，先修改 `src/config/tool/camera_calibration.yaml`。

## 标定板打印

仓库提供可直接交给打印店的 A4 横向矢量文件：

[`assets/calibration_board_9x6_25mm_A4.pdf`](assets/calibration_board_9x6_25mm_A4.pdf)

打印时必须选择“实际大小”或“100%”，禁止“适合页面”“缩小超大页面”等自动缩放。
打印后先测量页面底部的校验线，长度应为 100 mm，再抽查棋盘方格边长应为 25 mm；
尺寸不正确时不能用于标定。建议使用哑光纸，并将成品平整粘贴到硬质板材上，避免反光、
翘曲、气泡和拉伸。

需要重新生成 PDF 时运行：

```bash
./scripts/generate_calibration_board.sh
```

## 构建与启动

在连接 MindVision 相机的实机上构建：

```bash
cmake -S . -B build-camera \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MAIN=OFF \
  -DUSE_OPENVINO=OFF
cmake --build build-camera --parallel 4 --target mv-camera-calibration
```

启动默认 `build-camera` 中的程序：

```bash
./scripts/calibrate_camera.sh
```

使用其他构建目录：

```bash
./scripts/calibrate_camera.sh build-openvino
```

脚本不会自动构建，也不会调用 `sudo`。

## 采集操作

预览中的绿色角点表示完整棋盘已经被识别。依次采集棋盘位于画面中心、四周、不同距离、
水平倾斜和垂直倾斜的图像，避免反光、运动模糊以及大量重复姿态。

| 按键 | 操作 |
|---|---|
| `Space` | 接纳并立即保存当前帧 |
| `U` | 排除最近一个有效样本，原图仍保留 |
| `C` | 使用当前有效样本求解并执行质量检查 |
| `Q` / `Esc` | 保存会话状态并退出 |

质量检查要求至少 20 个有效样本、画面 3x3 区域完整覆盖、远近面积倍率不低于 2、
横纵方向各至少 4 张明显倾斜视图、全局 RMS 不超过 0.5 px、任一视图 RMS 不超过
1.0 px。不通过时查看控制台中的失败原因和最差样本编号，继续采样或用 `U` 排除样本后
再次按 `C`。

## 输出

每次启动都会创建 `artifacts/camera_calibration/<时间戳>/`：

- `images/` 保存所有接纳过的无损原图；
- `session.yaml` 保存样本状态、角点、覆盖指标和逐视图误差；
- `intrinsics.yaml` 仅在全部质量条件通过后生成。

后续由人工选择通过验收的 `intrinsics.yaml` 并移动到 PnP 配置位置。内参严格绑定文件中
记录的输出分辨率和硬件 ROI offset，不能用于不同的相机输出设置。
