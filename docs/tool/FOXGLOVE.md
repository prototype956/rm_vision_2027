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
- `/simulation/ground_truth`：机器人中心、朝向及装甲姿态、四角和局部坐标轴真值。
- `/simulation/ground_truth/annotations`：黄色装甲灯条端点真值及 `GT:TL/TR/BR/BL` 标签。
- `/vision/pnp/estimate`：正式检测单链 PnP 的相机系/世界系三维估计。
- `/vision/pnp/corners`：青色原始角点和有效的洋红色精修角点。
- `/vision/pnp/reprojection`：绿色正式 PnP 重投影。
- `/vision/pnp/error_vectors`：网络原角点到真值的灰线，以及成功精修角点到真值的洋红线。
- `/vision/corner_refiner/axes`：左右灯条浅蓝 PCA 中心轴与质心。
- `/vision/corner_refiner/candidates`：橙色搜索区间、黄色扫描线候选、绿色已提交或红色回退端点。
- `/vision/pnp/stats`：逐次求解状态、候选、重投影和真值误差 JSON。

将 Foxglove 连接到 `ws://<NUC-IP>:8765`，在 Image 面板选择图像话题，再将
annotations 话题加入 Image annotations。Plot 面板可直接选择 stats 中的数值字段。
Talos 仿真验收时在 3D 面板将固定坐标系设为 `world`，同时启用 transforms、frustum 和
ground truth；在 Image 面板额外启用 ground truth annotations。
`0.0.0.0` 不包含认证和 TLS，只应用于可信机器人局域网。

Plot 的 Y 值必须指向数值叶子，不能直接选择整个 stats 对象。例如检测耗时使用
`/vision/armor/stats.total_ms`，PnP 成功数使用 `/vision/pnp/stats.successful`，累计检测链
最终检测位置误差使用 `/vision/pnp/stats.summary.detection.position_error_m.p50`；精修前后
二维角点误差分别使用 `refinement.raw_mean_corner_error_px.p50` 和
`refinement.final_mean_corner_error_px.p50`。精修失败时四角整体回退，不会把部分候选与原始
端点混合进入 PnP。逐次 PnP 指标位于
数组中，可用 `/vision/pnp/stats.attempts[0].reprojection_rmse_px` 选择指定元素；求解失败时
该类可空指标为 `null`，Plot 会留下空点。

推荐布局：一个 Image 面板选择 `/vision/camera/image`，叠加 ground truth、corners、
reprojection、error vectors、axes 和 candidates；一个 Plot 面板观察精修成功/回退计数、
raw/final 角点误差及最终深度误差；一个 3D 面板以 `world` 为固定坐标系叠加 transforms、
frustum、ground truth 和 `/vision/pnp/estimate`。

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
