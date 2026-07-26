# 1280×720 MindVision 实机验收

## 配置与启动

生产入口是无 HUD、无性能统计的简单原始画面预览，固定读取
`src/config/core/logger.yaml` 和 `src/config/hal/camera/mindvision.yaml`：

```bash
sudo ./build/bin/mv-vision-main
```

按 `Q`、`Esc` 或关闭窗口退出。该程序不写 CSV、JSON 或样本帧。

完整实机测试固定读取 `src/config/test/camera_test.yaml`，运行命令为：

```bash
sudo ./build/bin/mv-camera-test
```

完整测试按 `Q`、`Esc` 或关闭窗口可提前退出。结果写入 `artifacts/camera_test/`：

- `metrics_*.csv`：每个报告周期的 FPS、帧间隔、CPU、RSS 等；
- `events_*.jsonl`：取帧失败事件及最后成功帧号；
- `summary_*.json`：验收指标与最终 PASS/FAIL；
- `sample_*.jpg`：定时样本帧。

## 测试顺序

### 五分钟画面检查

临时将 `duration_sec` 改为 `300`、`warmup_sec` 改为 `30`。手持装甲板经过中心、
四角和上下边缘。确认画面为 1280×720 BGR、中心 ROI、无比例拉伸、红蓝正确且没有
明显重复帧。退出后立即再次启动，设备应能重新打开。

### 三十分钟基线

恢复 `duration_sec: 1800`、`warmup_sec: 60`。程序以最初 60 秒建立 `F0`，并按以下
条件判定：

- 有效帧比例不低于 99.9%；
- 分辨率和图像类型错误均为 0；
- 任一滚动一分钟 FPS 不低于 `0.95 × F0`；
- 帧间隔 p99 不高于 `2/F0` 秒；
- 最大无有效帧时间小于 500 ms；
- RSS 增长不超过 20 MiB；
- 无断开和致命 SDK 错误。

### 50 次重复启停

将 `restart_cycles` 改为 `50`。此模式不执行长时间基线，而是执行 50 次
`Open → Grab 100 frames → Close`，输出 `restart_summary_*.json`。完成后将
`restart_cycles` 恢复为 `0`。

### 拔线

基线运行时拔出相机。程序应在取帧超时后报告 SDK 错误码、Grab 状态和最后成功帧号，
不切换 OpenCV、不自动恢复，随后正常关闭。重新插入后由人工再次启动。

## 常见定位

| 现象 | 优先检查 |
|---|---|
| 找不到配置 | 构建时记录的 `CONFIG_FILE_PATH` 和配置文件路径 |
| 不是 1280×720 | SDK ROI 设置返回码、能力范围和实际帧头尺寸 |
| 图像变形 | HAL 中不得存在软件 `resize` |
| ROI 不居中 | 传感器最大尺寸和偶数对齐后的 offset |
| 颜色异常 | ISP 输出必须是 `CAMERA_MEDIA_TYPE_BGR8` |
| 周期性超时 | 曝光、USB 带宽、DMA buffer 释放、100 ms timeout |
| 重启打不开 | 所有退出路径是否执行 `CameraUnInit` |
