# YOLO 0526 检测验收

检测模块只保留实机长时验收和离线视频验收两个手动入口，不注册 CTest。模型文件由
使用者手工管理，正式运行仍会检查 OpenVINO 输入输出契约。

## 前置检查与构建

重新登录以刷新补充组，然后确认当前进程和 OpenVINO 均能访问 GPU：

```bash
id
source /opt/intel/openvino_2024.0.0/setupvars.sh
python3 - <<'PY'
import openvino as ov
print(ov.Core().available_devices)
PY
```

`id` 必须包含 `render`、`video`，OpenVINO 设备列表必须包含 `GPU` 或
`GPU.<index>`。单核显环境通常显示为 `GPU`；编号形式用于区分多个 GPU。检测器
明确指定 GPU，不使用 `AUTO`、`MULTI` 或 CPU 回退，也不要使用 `sudo` 运行。

```bash
source /opt/intel/openvino_2024.0.0/setupvars.sh
cmake -S . -B build-openvino -DCMAKE_BUILD_TYPE=Release -DUSE_OPENVINO=ON
cmake --build build-openvino --parallel 4
```

## MindVision 实机长时验收

程序固定读取：

- `src/config/core/logger.yaml`
- `src/config/modules/armor_detector.yaml`
- `src/config/hal/camera/mindvision.yaml`
- `src/config/test/armor_detector_test.yaml`

默认配置执行 30 分钟测试，前 60 秒用于 GPU 预热和循环帧率基线，不纳入检测延迟
统计。运行命令：

```bash
./build-openvino/bin/mv-armor-detector-test
```

预览窗口显示帧号、循环 FPS、结果/候选数量、预处理/推理/后处理/总耗时以及检测
四边形、颜色、标签和 objectness。按 `Q`、`Esc` 或关闭窗口会提前结束，并在最终
报告中记为未完成。

结果写入 `artifacts/armor_detector_test/`：

- `metrics_*.csv`：周期抓帧、检测数量、循环 FPS、各阶段延迟、CPU 和 RSS。
- `events_*.jsonl`：相机抓帧失败、检测异常和用户提前退出事件。
- `summary_*.json`：最终 PASS/FAIL 及完整汇总。
- `sample_*.jpg`：按配置周期保存的检测叠加图。

自动 PASS 条件：

- 完成配置时长且有效相机帧比例不低于 99.9%。
- 输出始终为 `1280×720 CV_8UC3`，没有检测运行错误。
- 检测器初始化日志中的 `EXECUTION_DEVICES` 只包含 GPU。
- 预热后完整检测链路 P95 小于 `16.7 ms`。
- 滚动一分钟最低循环 FPS 不低于预热基线的 95%。
- 最大无有效帧时间小于 500 ms，RSS 增长不超过 20 MiB。
- 没有相机断开或致命错误。

检测数量不参与自动 PASS/FAIL。测试时应让敌方颜色装甲板经过中心、四角和画面边缘，
通过实时 HUD 和保存样本人工检查颜色、标签、漏检、误检和角点位置。

## 离线视频

默认视频由 `src/config/test/armor_detector_video_test.yaml` 指定。当前配置读取本地
`vedio/demo.avi` 并默认打开预览窗口，因此可以直接运行：

```bash
./build-openvino/bin/mv-armor-detector-video-test
```

预览窗口按视频原始 FPS 播放带识别框的画面。HUD 显示视频帧号、实际循环 FPS、
检测数和候选数、预处理/推理/后处理耗时、完整检测耗时及累计运行时间。按 `Q`、
`Esc` 或关闭窗口可提前结束。无桌面环境可使用 `--no-preview`；`--benchmark`
会自动关闭预览，避免窗口绘制和播放节流干扰性能验收。

需要临时测试其他视频时，`--video` 会覆盖默认配置：

```bash
./build-openvino/bin/mv-armor-detector-video-test \
  --video /path/to/battle.mp4 \
  --config src/config/modules/armor_detector.yaml \
  --start-frame 0 \
  --end-frame 2000 \
  --sample-stride 100 \
  --output-dir artifacts/detector_test/battle
```

帧区间为左闭右开 `[start-frame, end-frame)`；省略 `--end-frame` 时读取到视频
结束。输出包括：

- `detections.jsonl`：逐帧颜色、标签、objectness、包围盒和四角点。
- `timings.csv`：预处理、推理、后处理、总耗时和候选数量。
- `summary.json`：平均值及 P50/P95/P99 汇总。
- `frame_*.jpg`：按 `--sample-stride` 保存的可视化样本。

本期不读取人工标注，也不计算 precision、recall 或角点误差。

## 离线性能验收

视频在起始帧后至少需要 1100 帧：

```bash
./build-openvino/bin/mv-armor-detector-video-test \
  --benchmark \
  --output-dir artifacts/detector_test/benchmark
```

程序先预热 100 帧，再统计 1000 帧。计时包含 Letterbox、OpenVINO 推理、解码和
NMS，不包含视频解码和绘制。完整检测链路 P95 必须小于 `16.7 ms`；不满足门槛时
根据 `summary.json` 中各阶段 P95 定位，本期不引入异步队列规避单帧延迟。
