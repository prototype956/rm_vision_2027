# MindVision 相机内参标定

## 标定前准备

工具固定使用 `src/config/hal/camera/mindvision.yaml` 中的相机参数，并要求实际输出为
1280x720 BGR8。标定完成前不要改变镜头、焦距、对焦、硬件 ROI、分辨率或曝光模式。

默认标定板为 11x8 内角点，格长暂设 25 mm，使用前必须实测确认。11x8 是内角点数量，
对应 12x9 个黑白格；
实物尺寸不同时，先修改 `src/config/tool/camera_calibration.yaml`。

## 标定板打印

仓库仍提供旧版 9x6 内角点的 A4 横向矢量文件（与当前 11x8 默认配置不同，
若使用此文件，必须将配置改回 9x6、25 mm）：

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

## 先采集，再离线解算

启动命令默认进入采集模式，也可显式指定：

```bash
./scripts/calibrate_camera.sh build-openvino capture
```

采集阶段只预览画面和保存原图，不运行棋盘检测、清晰度检查或内参求解。
因此没有绿色角点提示，未包含完整棋盘或模糊的图像也可以保存，稍后离线检测。
依次拍摄棋盘位于中心、四周、不同距离、水平倾斜和垂直倾斜的图像，避免反光和模糊。

| 按键 | 操作 |
|---|---|
| `Space` / `S` | 保存当前原始 PNG 图像 |
| `U` | 将最近保存的图像移到 `excluded/`，保留原图但不参与解算 |
| `Q` / `Esc` | 结束采集并退出 |

每次启动创建 `artifacts/camera_calibration/<时间戳>/`，其中 `images/` 保存原图，
`session.yaml` 保存采集时的相机实际参数和棋盘尺寸。采集文件的样本列表为空；
已采图片以 `images/` 中的 PNG 为准。此阶段不产生内参，按 `C` 不再触发求解。

退出采集后，将终端打印的采集目录传给离线解算命令：

```bash
./scripts/calibrate_camera.sh build-openvino solve artifacts/camera_calibration/<时间戳>
```

也可直接运行：

```bash
./build-openvino/bin/mv-camera-calibration solve artifacts/camera_calibration/<时间戳>
```

离线解算不打开相机，但需要图形桌面。程序按文件名排序扫描 `images/` 下的 PNG，
不要求编号连续。每张图检测后弹窗显示检测点和彩色连线、文件名及检测状态，
按任意键（包括 Q/Esc）继续下一张。检测失败的图片也会显示，按键后跳过。
关闭窗口或按 Ctrl+C 取消整个流程，写入 aborted 报告并返回退出码 6，不求解内参。
全部图片确认后关闭窗口，才开始求解内参和重投影误差。
终端输出内参矩阵、畸变系数、全局与最大单图 RMS、最差样本编号及逐图 RMS。
彩色连线表示检测点的排列顺序，不是重投影点；叠加画面不保存到磁盘。
它使用采集目录记录的相机、ROI 和棋盘参数，以及当前 `tool/camera_calibration.yaml`
中的质量阈值。不能把其他分辨率、ROI、相机或棋盘的图像混入同一目录。
无法读取或尺寸不匹配的图片会导致报错；未识别完整棋盘的图片会跳过并输出日志。
清晰度（拉普拉斯方差）和透视倾斜比仍写入报告用于诊断，但不作为筛选或验收条件；
平均灰度没有验收门限。

质量检查仍要求至少 20 个有效样本、角点覆盖全部 3x3 区域、远近面积倍率不低于 2、
全局 RMS 不超过 0.5 px、任一视图 RMS 不超过
1.0 px。失败时查看日志和解算报告，排除坏图后重新解算；采样不足则重新采集完整的一组。
需要排除任意图片时将它移到采集目录的 `excluded/`，不要删除原图。

## 解算输出

每次解算单独创建 `<采集目录>/solutions/<解算时间戳>/`：

- `session.yaml`：有效样本对应的原图路径、角点、覆盖统计、逐视图误差和失败原因。
- `intrinsics.yaml`：仅在全部质量条件通过后生成。

每次解算保留独立结果，质量失败返回退出码 5；读取或配置错误返回 1。
旧结果不会自动删除，使用时必须确认选取的是本次通过验收的结果。
后续由人工选择通过验收的 `intrinsics.yaml` 并接入 PnP。
内参绑定输出分辨率、硬件 ROI、相机和镜头状态；标定后可改变曝光，但不要改变几何设置。

已移除配置字段 `capture.min_sharpness`、`quality.min_tilted_views`、
`quality.min_tilt_ratio`，旧配置需要删除这些字段。历史解算报告保持原样；
重新解算使用当前质量配置及采集目录中的棋盘尺寸。
