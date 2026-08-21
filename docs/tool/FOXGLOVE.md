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
- `/vision/lightbars/annotations`：独立灯条原始/去重/拒绝/接受状态及预测灯条。
- `/vision/lightbars/stats`：实际阈值、轮廓筛选、检测耗时、融合计数和安全回退状态。
- `/vision/debug/stats`：采集时间、空间元数据状态、JPEG 耗时、发布延迟及调试丢帧统计。
- `/vision/transforms`：`world -> gimbal -> camera_optical` 两级 TF。
- `/vision/camera/calibration`：与当前图像同帧的针孔内参和畸变参数。
- `/vision/camera/frustum`：位于 `camera_optical` 下、深度 1 米的相机视锥。
- `/simulation/ground_truth`：机器人中心、朝向及装甲姿态、四角和局部坐标轴真值。
- `/simulation/projectiles/stats`：17 mm 发射、装甲/能量机关命中、飞镖发射累计值，以及
  装甲命中率和尚未命中数。
- `/simulation/ground_truth/annotations`：黄色装甲灯条端点真值及 `GT:TL/TR/BR/BL` 标签。
- `/vision/pnp/estimate`：正式检测单链 PnP 的相机系/世界系三维估计。
- `/vision/pnp/corners`：青色原始角点和有效的洋红色精修角点。
- `/vision/pnp/reprojection`：绿色正式 PnP 重投影。
- `/vision/pnp/error_vectors`：网络原角点到真值的灰线，以及成功精修角点到真值的洋红线。
- `/vision/corner_refiner/axes`：左右灯条浅蓝 PCA 中心轴与质心。
- `/vision/corner_refiner/candidates`：橙色搜索区间、黄色扫描线候选、绿色已提交或红色回退端点。
- `/vision/pnp/stats`：逐次求解状态、候选、重投影和真值误差 JSON。
- `/vision/prediction/scene`：13维 ESEKF 车体完整姿态、双半径和当前/未来四装甲。
- `/vision/prediction/state`：固定顺序状态、协方差、创新、NIS及每自由度NIS、机动模式、关联
  门限与接受/拒绝计数、逐灯条关联、灯条-only/联合融合/装甲回退标志、累计重置计数、耗时及
  Talos 中心/yaw/yaw角速度误差。
- `/vision/prediction/truth_overlay`：车辆中心估计到同标签 Talos 真值的世界系误差线。
- `/vision/prediction/current_annotations`：当前重投影及橙色实测轮廓；正式提交的关联预测为
  青色，被像素或NIS门限拒绝的最近候选预测为洋红色。
- `/vision/prediction/future_annotations`：100 ms后的四装甲图像重投影。

将 Foxglove 连接到 `ws://<NUC-IP>:8765`，在 Image 面板选择图像话题，再将
annotations 话题加入 Image annotations。Plot 面板可直接选择 stats 中的数值字段。
Talos 仿真验收时在 3D 面板将固定坐标系设为 `world`，同时启用 transforms、frustum 和
ground truth；在 Image 面板额外启用 ground truth annotations。
`0.0.0.0` 不包含认证和 TLS，只应用于可信机器人局域网。

Talos 自动开火基线仅在 `camera.backend=talos` 时存在命令输出。F5 关闭、跟踪未确认、
TEMP_LOST、数据过期、MPC 回退或任一火控门控失败都会保持 `fire=false`。测试期间不要使用
`Space` 手动发弹或 `G` 发射飞镖。`not_yet_hit_count` 包含仍在飞行的弹丸，只有关闭 F5 并
等待至少 6 秒后才可将其解释为未命中。以每累计 10 次 `armor_hit_count` 作为一次代理击杀，
基线应同时记录 TTK10、达到第 10 次命中时的发射数、命中频率、代理击杀/分钟和平均耗弹。

Plot 的 Y 值必须指向数值叶子，不能直接选择整个 stats 对象。例如检测耗时使用
`/vision/armor/stats.total_ms`，PnP 成功数使用 `/vision/pnp/stats.successful`，累计检测链
最终检测位置误差使用 `/vision/pnp/stats.summary.detection.position_error_m.p50`；精修前后
二维角点误差分别使用 `refinement.raw_mean_corner_error_px.p50` 和
`refinement.final_mean_corner_error_px.p50`。精修失败时四角整体回退，不会把部分候选与原始
端点混合进入 PnP。逐次 PnP 指标位于
数组中，可用 `/vision/pnp/stats.attempts[0].reprojection_rmse_px` 选择指定元素；求解失败时
该类可空指标为 `null`，Plot 会留下空点。

独立灯条联调可绘制 `/vision/lightbars/stats.elapsed_ms`、`accepted_count`、
`deduplicated_count` 和 `rejected_count`，并同时观察
`/vision/prediction/state.light_fusion_used`、`light_only_pair_count`、`light_only_update`、
`light_only_update_blocked`、`light_only_rejection_reason` 和 `armor_fallback_used`。
Image 面板叠加 `/vision/lightbars/annotations` 后，绿色为接受、黄色为完整装甲去重、红色为
拒绝、青色为对应预测灯条。联合更新被 NIS 门控拒绝而装甲基准通过时，独立关联原因为
`combined_nis_gate`，完整装甲更新仍会提交；无完整装甲且没有同槽左右灯条对时，已匹配单灯条
记录为 `insufficient_light_only_geometry`，不会提交 ESEKF 后验。

ESEKF 机动联调时建议同时绘制
`/vision/prediction/state.truth_yaw_equivalent_error_rad`、`truth_yaw_velocity_error_rad_s`、
`maneuver_active`、`maneuver_phase`、连续证据、确认/活动剩余时间、实际 yaw 过程噪声、
`trial_yaw_velocity_update_rad_s`、`association_gate_used`、先验 `nis_per_dof` 和
`reset_count`。原始
`truth_yaw_error_rad` 保留任意槽位0带来的 `pi/2` 相位差，不应用作四装甲姿态精度门槛；
`reset_reason` 表示最近原因，只有 `reset_count` 增加才代表发生了新的安全重置。

推荐布局：一个 Image 面板选择 `/vision/camera/image`，叠加 ground truth、lightbars、corners、
reprojection、error vectors、axes 和 candidates；一个 Plot 面板观察灯条检测/接受数、精修成功/回退计数、
raw/final 角点误差及最终深度误差；一个 3D 面板以 `world` 为固定坐标系叠加 transforms、
frustum、ground truth 和 `/vision/pnp/estimate`。

内部实现按 `image`、`armor_detector`、`armor_light_detector`、`spatial` 和 `simulation` 分离消息
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

## 使用 Codex 分析 MCAP

仓库级 `rm-vision-mcap` skill 位于
`.agents/skills/rm-vision-mcap/`，只在本仓库及其子目录中向 Codex 提供只读分析流程。
分析器要求 `PATH` 中已有兼容的 `mcap` CLI；skill 不负责安装软件或修改系统环境。
分析器保留在 skill 内，不安装全局包装命令：

```bash
MCAP_ANALYZE=.agents/skills/rm-vision-mcap/scripts/mcap_analyze.py
python3 "$MCAP_ANALYZE" inspect artifacts/foxglove/example.mcap
python3 "$MCAP_ANALYZE" preset artifacts/foxglove/example.mcap pnp
python3 "$MCAP_ANALYZE" stats artifacts/foxglove/example.mcap \
  --topic /vision/armor/stats --field total_ms
python3 "$MCAP_ANALYZE" frames artifacts/foxglove/example.mcap \
  --start 1786977451.0 --end 1786977453.0 --count 6 --layers pnp
```

`inspect` 会把视觉与火控后台写入交错产生的大量时间顺序警告按话题汇总；其他结构错误仍
单独报告。`query` 默认最多输出 20 条消息，`stats` 使用有界样本计算大文件分位数，
`frames` 默认抽取 6 帧且最多 50 帧。抽帧、叠加图和 manifest 默认写入
`/tmp/mcap-analysis/`。所有命令只读源 MCAP；禁止用该工作流执行 `add`、`filter`、
`compress`、`recover` 等改写操作。
