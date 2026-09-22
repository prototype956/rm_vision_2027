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

- `/referee/self`：Talos v7 的最近 10 Hz 自身裁判采样，包含血量、热量、弹量及禁射原因。
- `/simulation/combat/evaluation`：当前物理状态、按机器人统计和有界事件，敌方血量仅在评估通道。
  时间与去重约定见 [战斗遥测验收](../test/talos_combat.md)。
- `/vision/camera/image`：JPEG `foxglove.CompressedImage`。
- `/vision/armor/annotations`：四角框和颜色、类别、置信度文字。
- `/vision/armor/stats`：检测耗时、候选数、最终检测数及装甲检测阶段状态。
- `/vision/lightbars/annotations`：独立灯条原始/去重/拒绝/接受状态及预测灯条。
- `/vision/lightbars/stats`：实际阈值、轮廓筛选、检测耗时、融合计数和安全回退状态。
- `/vision/debug/stats`：采集时间、空间元数据状态、JPEG 耗时、发布延迟及调试丢帧统计。
- `/vision/camera/transforms`：实机图像采集时刻的 `foxglove.FrameTransforms`，发布
  `world -> gimbal -> camera_optical`；只在该帧具有有效平台运动学时发送。
- `/vision/transforms`：控制时刻的 `world -> gimbal -> camera_optical` 两级短时外推 TF。
- `/vision/camera/calibration`：与当前图像同帧的针孔内参和畸变参数。
- `/vision/camera/frustum`：位于 `camera_optical` 下、深度 1 米的相机视锥。
- `/simulation/ground_truth`：机器人中心、朝向及装甲姿态、四角和局部坐标轴真值。
- `/simulation/projectiles/stats`：17 mm 发射、装甲/能量机关命中、飞镖发射累计值，以及
  装甲命中率和尚未命中数。
- `/simulation/ground_truth/annotations`：黄色装甲灯条端点真值及 `GT:TL/TR/BR/BL` 标签。
- `/vision/pnp/estimate`：正式检测单链 PnP 的相机系/世界系三维估计。
- `/vision/pnp/raw_corners`：网络输出的青色 1.5 px 实线原始 PnP 输入角点。
- `/vision/pnp/final_corners`：实际提交给正式 PnP 的角点；洋红色 2.5 px 实线
  `REFINED` 表示精修成功，黄色 2.5 px 实线 `FALLBACK:<status>` 表示精修失败并整体回退
  原始角点。
- `/vision/pnp/reprojection`：绿色正式 PnP 重投影。
- `/vision/pnp/error_vectors`：网络原角点到真值的灰线，以及成功精修角点到真值的洋红线。
- `/vision/corner_refiner/axes`：左右灯条浅蓝 PCA 中心轴与质心。
- `/vision/corner_refiner/candidates`：橙色搜索区间、黄色扫描线候选、绿色已提交或红色回退端点。
- `/vision/pnp/stats`：正式 PnP 阶段状态、逐次求解、候选、重投影和真值误差 JSON。
- `/vision/prediction/scene`：13维 ESEKF 当前车体中心、速度、姿态轴、双半径和当前四装甲。
- `/vision/control/impact_scene`：弹道命中时域的紫色四装甲；最终选中槽位使用不透明粗线，
  其余槽位使用半透明细线。
- `/vision/control/selection_scene`：装甲选择时域的四槽位候选；锁定槽位为绿色粗线，待切换
  槽位为黄色粗线，可进入候选为青色，不可进入候选为半透明灰色，并保留 `slot/view` 标签。
- `/vision/control/aim_scene`：白色枪口球、弹道目标球、白色当前反馈射线和绿色已发布指令
  射线；目标球红/黄/紫分别表示已开火、满足开火资格和其余有效弹道状态，不附加状态文字。
- `/vision/control/trajectory_scene`：最近 1 秒白色估计反馈、洋红色实测反馈、青色参考轨迹，
  以及有效时黄色、无效时红色的 MPC 计划轨迹，不附加文字。
- `/vision/control/state`：100 Hz 火控状态、MPC 阶段状态、弹道、命令和开火门控 JSON。
- `/vision/prediction/state`：固定顺序状态、协方差、创新、NIS及每自由度NIS、机动模式、关联
  门限与接受/拒绝计数、逐灯条关联、灯条-only/联合融合/装甲回退标志、累计重置计数、耗时及
  Talos 中心/yaw/yaw角速度误差。
- `/vision/prediction/truth_overlay`：车辆中心估计到同标签 Talos 真值的世界系误差线。
- `/vision/prediction/current_annotations`：当前四槽位预测中，正面使用深绿色 2 px 实线，
  背面使用 35% 透明的绿色 1.5 px 实线；亮绿色 3 px 已接受关联框位于最上层。
- `/vision/control/impact_annotations`：火控最终选中槽位在弹道命中时域的紫色 4 px 实线预测框，
  不附加文字。
- `/vision/control/selected_armor_annotations`：当前时域的绿色锁定槽位和黄色待切换槽位。

将 Foxglove 连接到 `ws://<NUC-IP>:8765`，在 Image 面板选择图像话题，再将
annotations 话题加入 Image annotations。Plot 面板可直接选择 stats 中的数值字段。
Talos 仿真验收时在 3D 面板将固定坐标系设为 `world`，同时启用 transforms、frustum 和
ground truth；在 Image 面板额外启用 ground truth annotations。
`0.0.0.0` 不包含认证和 TLS，只应用于可信机器人局域网。

## 处理链状态转移

在 Foxglove 的 State Transitions 面板中分别添加以下四个消息路径，即可按行观察
`装甲检测 -> PnP -> EKF -> MPC` 的处理状态：

- `/vision/armor/stats.detection_state`：`not_detected` 表示当前帧没有正式检测结果，
  `detected` 表示至少检测到一块装甲板。
- `/vision/pnp/stats.pnp_state`：`not_attempted` 表示没有装甲输入，`unavailable` 表示检测到
  装甲但缺少空间或标定数据，`failed` 表示正式 PnP 已运行但没有有效位姿，`solved` 表示
  至少得到一个正式位姿。该字段不使用仿真真值求解结果。
- `/vision/prediction/state.tracker_state`：ESEKF 跟踪器的 `lost`、`detecting`、`tracking` 或
  `temp_lost`。
- `/vision/control/state.mpc.state`：`inactive` 表示控制前置条件不满足、MPC 未运行，
  `solved` 表示当前周期规划成功，`failed` 表示规划失败，`fallback` 表示规划失败后仍在发布
  最近有效轨迹的 100 ms 短时回退。

四行都是周期快照而不是仅在转换时发布的事件。检测、PnP 和 EKF 行使用图像采集时间并受
`image.max_fps` 限流；MPC 行使用控制命令时间并保持 100 Hz。实时连接与 MCAP 回放使用相同
字段和 Schema，因此同一 State Transitions 布局可以直接复用。各行独立表达当前阶段，遮挡时
可能同时出现 `not_detected / not_attempted / temp_lost / solved`，这表示预测与控制正在短时
延续历史目标，并不代表数据跨阶段串错。

TF 不再跟随 `image.max_fps` 的图像调试限流，而是由 100 Hz 控制诊断管线发布。
其平移使用与采集帧同步的底盘体系线速度做最长 100 ms 恒速外推，姿态使用新鲜
云台反馈及角速度外推。底盘速度不可用时保持采集位置；采集时间或云台反馈超过
100 ms 时停止发布该周期 TF，不伪造低延迟位姿。图像、标注、PnP、视锥和仿真真值仍保留
原始采集时间。

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

推荐布局：一个 Image 面板选择 `/vision/camera/image`，按调试阶段叠加 ground truth、lightbars、
raw/final corners、reprojection、current association、selected armor 或 impact；一个 Plot 面板观察
灯条检测/接受数、精修成功/回退计数、raw/final 角点误差及最终深度误差；3D 面板以 `world`
为固定坐标系，空间定位时叠加 transforms、frustum、ground truth 和 `/vision/pnp/estimate`，
火控联调时按需独立启用 selection、aim、trajectory 或 impact scene，避免无关几何相互遮挡。

内部实现按 `image`、`armor_detector`、`armor_light_detector`、`spatial` 和 `simulation` 分离消息
编码。图像调试领域由 `pipeline` 统一限流并保持同帧采集时间；`transforms` 复用
`spatial` 编码器，但由控制诊断线程按控制时间发布。

运行时不再依赖 Foxglove 类型，而是向传输无关的 `IRuntimeDiagnosticsSink` 分别提交
`VisionFrameOutput + VisionFrameDiagnostics` 和
`ControlCycleOutput + ControlCycleDiagnostics`。应用层适配器再将两套数据交给 Foxglove，
编码边界按既有 Schema 重新组合。当前完整创建 33 个频道；诊断发布失败不改变
正式视觉结果、控制命令或退出状态。
`impact_annotations` 或 `impact_scene` 被订阅/录制时，图像调试线程等待最多 20 ms，并只接受
完全相同 `source_sequence` 的控制快照；两者复用一次等待结果，超时分别清除旧标注和旧实体。
控制话题的 `runtime` 分组和扁平 `tracking` 中提供 `feedback_projection_dt_s`、
`pose_projection_dt_s`、`chassis_motion_valid` 和 `pose_projection_status`，用于区分完整运动外推、
缺少底盘速度时的位置保持，以及过期或时间非法降级。
`/vision/control/state.ballistics.prediction_horizon_s` 记录生成最终 `target_world` 时实际使用的
总预测时域，供 impact 标注和控制状态交叉核对。
三个控制 SceneUpdate 话题和 `/vision/control/trajectory` 共用 `image.max_fps` 降采样节奏；
实时输出只编码已订阅的场景，MCAP 录制启用时记录全部三个场景。旧
`/vision/control/scene` 不再创建，也不提供兼容别名。

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

实时 WebSocket 与 MCAP sink 独立维护健康状态，一个失效不会关闭另一个。两者全部不可用时，
应用层把诊断组件标记为降级，但不改变视觉、预测、控制命令或进程退出原因。该健康快照只在
C++ 控制面和状态转换日志中使用，其自身不增加频道或额外诊断字段。

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


## 实机 3D 坐标系

串口配置正确且主程序显示 `IMU:ok profile=... sync=true` 后，重启使用新编译的主程序，
连接 `ws://<NUC-IP>:8765`（本机使用 `ws://127.0.0.1:8765`）。

1. 新增 **3D** 面板，将 **Fixed frame** 和 **Display frame** 都设为 `world`，
   以固定观察方向查看云台旋转，避免视角跟随相机一起旋转。
2. 在 **Transforms** 中启用 `world`、`gimbal`、`camera_optical` 的显示，打开 **Labels**；
   **Axis scale** 可从 0.2 开始调整。红、绿、蓝轴分别表示 X、Y、Z。
3. 在 **Topics** 中启用 `/vision/camera/frustum`，观察相机朝向和视场。
4. 可另加 **Transform Tree** 检查父子关系，或用 **Raw Messages** 查看
   `/vision/camera/transforms` 的时间、平移和四元数。

实机 TF、视锥、图像等同帧消息共用采集时间；没有采集时间的源才回退接收时间。
采集单调时间通过固定时钟锚点转换为消息 epoch 时间。实机相机时间仍为软件估计。
新 TF 跟随视觉调试队列的 `image.max_fps` 限流（默认 20 Hz），不是原始 200 Hz IMU 流。
实时订阅与已启用的 MCAP 录制均包含该话题；无需额外开启录制。
Talos 帧不在新话题发布 TF，继续使用控制链 `/vision/transforms`，避免两个时间基准
同时更新相同 child。无需 ROS 或 robot_state_publisher。

当前车辆 IMU 名义安装旋转已固定为 `diag(-1,-1,1)`；平移仍为占位值，三个原点重合
属于预期。重叠时可分别切换坐标轴显示，其他车辆应先选择对应部署 profile。
这里只显示最终刚体关系，不显示大 yaw、小 yaw、pitch 的独立机械关节，也不增加未经测量的
IMU/枪口坐标轴。world 是惯性方向、随相机移动原点的近似，不代表导航位置。
IMU 失效时停止发布新 TF；Foxglove 可能保留最后一帧坐标轴，因此应同时查看 TF 时间是否
持续更新以及主程序 IMU 状态，不能仅凭画面存在判断链路在线。

面板选项参考 [Foxglove 3D 官方说明](https://docs.foxglove.dev/docs/visualization/panels/3d)。
