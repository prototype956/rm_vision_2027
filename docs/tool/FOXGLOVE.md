# Foxglove 调试输出

Foxglove 模块将相机原图、二维装甲标注、空间坐标和调试指标异步发布到 WebSocket，并可选写入
MCAP。`mv-vision-main` 和 `mv-armor-detector-test` 共用
`src/config/tool/foxglove.yaml`；发布失败不会反压检测主链路，也不参与装甲检测测试的
PASS/FAIL 判定。

## 配置与话题

`foxglove.yaml` 包含以下开关：

- `enabled`：控制整个 Foxglove 调试输出。
- `server.host` 和 `server.port`：控制 WebSocket 监听地址，默认是 `0.0.0.0:8765`。
- `image.max_fps` 和 `image.jpeg_quality`：控制图像限流与 JPEG 质量。
- `recording.enabled` 和 `recording.output_dir`：控制可选 MCAP 录制及输出目录。

实时与录制使用相同的话题：

- `/vision/camera/image`：JPEG `foxglove.CompressedImage`。
- `/vision/armor/annotations`：四角框和颜色、类别、置信度文字。
- `/vision/armor/stats`：检测耗时、候选数和最终检测数。
- `/vision/debug/stats`：采集时间、空间元数据状态、JPEG 耗时、发布延迟及调试丢帧统计。
- `/vision/transforms`：`world -> gimbal -> camera_optical` 两级 TF。
- `/vision/camera/calibration`：与当前图像同帧的针孔内参和畸变参数。
- `/vision/camera/frustum`：位于 `camera_optical` 下、深度 1 米的相机视锥。
- `/simulation/ground_truth`：机器人中心、朝向和装甲中心投影探针的三维真值。
- `/simulation/ground_truth/annotations`：装甲中心真值在相机图像上的重投影点。

将 Foxglove 连接到 `ws://<NUC-IP>:8765`，在 Image 面板选择图像话题，再将
annotations 话题加入 Image annotations。Plot 面板可直接选择 stats 中的数值字段。
Talos 仿真验收时在 3D 面板将固定坐标系设为 `world`，同时启用 transforms、frustum 和
ground truth；在 Image 面板额外启用 ground truth annotations。
`0.0.0.0` 不包含认证和 TLS，只应用于可信机器人局域网。

内部实现按 `image`、`armor_detector`、`spatial` 和 `simulation` 分离消息
编码，由 `pipeline` 统一管理限流、后台线程和频道。各领域仍属于同一调试帧，
不会因为拆分模块而产生话题间时间差。

## 构建与运行

```bash
source /opt/intel/openvino_2024.0.0/setupvars.sh
cmake -S . -B build-openvino -DCMAKE_BUILD_TYPE=Release -DUSE_OPENVINO=ON
cmake --build build-openvino --parallel 4

./build-openvino/bin/mv-vision-main
# 或在装甲检测实机验收期间观察同一组话题
./build-openvino/bin/mv-armor-detector-test
```

构建会将 vendored `libfoxglove.so` 放入 `build-openvino/lib/`，可执行程序通过
`$ORIGIN/../lib` 查找，不需要手工设置 `LD_LIBRARY_PATH`。正式程序可将
`debug_window.yaml` 的 `enabled` 设为 `false` 后无窗口运行，并使用 `Ctrl+C` 或
`SIGTERM` 正常退出。装甲检测测试按自身配置时长结束。

Foxglove 配置无法解析、端口被占用或某个 sink 初始化失败时，程序记录诊断日志并继续
检测。开启 `recording.enabled` 后，正常退出会排空最后一帧并关闭 MCAP；录制文件写入
配置的 `recording.output_dir`。WebSocket 订阅、重连和 MCAP 完整性由使用者按需观察，
不设置独立测试目标或自动验收门槛。
