# 云台 PID / MPC 控制

控制算法位于 `src/modules/gimbal_control/pid/`、`mpc/`，共用参考类型和
`GimbalReferencePlanner`。HAL 只提供硬件反馈和传输，算法模块不依赖串口实现。

## 数据流与模式

火控选择装甲并解算弹道，生成同一参考时域。MindVision 默认使用 PID，Talos 保持 MPC：

- PID：参考序列 → 下一参考点（index=1）→ 发送阶段双轴 PID → 限幅角速度 → type 7。
- 实机 MPC：参考序列 → MPC 约束轨迹 → 前视命令 → 独立跟随环 → 限幅角速度 → type 7。
- Talos MPC：参考序列 → MPC 约束轨迹 → 前视角度命令 → 现有 Talos 共享内存协议。

PID 不创建、不调用 MPC 求解器，也不自动回退到 MPC。历史 yaw MPC + pitch 开关控制已移除。
PID 的直接参考序列与 MPC 轨迹均可参与原有的短时目标丢失回退，回退时不延长开火脉冲。

`SerialGimbalControlAdapter` 在控制线程发送阶段重新取得直接 IMU 反馈，与重构前一致；
不能用火控估计器外推或限幅后的反馈替代。PID 使用实际控制周期；超过 100 ms 时清除状态。
角度为 rad、角速度为 rad/s、时间为 s。yaw 误差包到 ±π，pitch 不包角。
D 项为测量角速度低通后的负反馈，不对目标阶跃求导；积分权重随误差增大下降。
进入死区清积分并输出零速度，输出饱和时禁止向饱和方向继续积分。

## 配置迁移

所有配置路径相对于当前配置根目录，支持 `RM_VISION_CONFIG_ROOT`。
本次一次性迁移，不支持旧 YAML 格式或旧实验开关：

| 文件（相对 `src/config/`） | 字段与职责 |
| --- | --- |
| `modules/gimbal_control.yaml` | `modes.mindvision/talos`、`reference`、`feedback_limits`、`rate_limits` |
| `modules/gimbal_control/pid.yaml` | `yaw`、`pitch` 的 Kp/Ki/Kd、积分、滤波和死区 |
| `modules/gimbal_control/mpc.yaml` | `planner` 的前视/归一化/约束/权重/求解参数；`follower` 的实机跟随参数 |
| `hal/serial/gimbal.yaml` | `enabled`、`fire_output_enabled`、`max_feedback_age_ms` |

在公共配置中设置 `modes.mindvision: pid` 或 `mpc`，正常停止程序后重启；
`modes.talos` 只允许 `mpc`。未知模式、旧开关或不完整配置均报错，不静默回退。
PID 只读取 PID 文件；MPC 只读取 MPC 文件，另一文件缺失或损坏不影响所选模式。
串口设备、波特率和时钟同步仍由当前车辆 profile 的 IMU 配置管理。

公共参考保持 `dt_s: 0.01`、`horizon_steps: 50`；PID 前视 10 ms，MPC 前视 50 ms。
当前 PID yaw Kp/Ki/Kd 为 10.0/0.2/0.5，pitch 为 8.5/0.5/0.8。
反馈估计两轴限速均为 3 rad/s；实机输出限速分别为 3/4 rad/s，二者独立。
MPC 规划两轴速度约束仍为 3 rad/s、加速度约束 50/100 rad/s²。
发射输出、裁判测试旁路、摩擦轮 1 秒续转保留当前部署值。

旧 `hal/gimbal/serial.yaml`、`modules/gimbal_trajectory_planner.yaml` 已删除；
部署时应同步复制整套新配置并重新构建主程序，不要只更新一个 YAML。

## 实机热更新

运行时后台每 250 ms 读取公共配置、当前算法配置和串口输出配置；
连续两次内容相同后整体校验，再由控制线程整体应用。磁盘操作不在 100 Hz 控制线程执行。

- 可热更新：PID 两轴参数、公共 `rate_limits`、MPC `follower` 参数。
- 需重启：模式、公共参考时域、反馈限速、MPC `planner` 参数、串口输出使能及反馈时效。

一次稳定快照混入需重启字段时整组拒绝，保持上一组有效参数；修正保存后可以继续加载。
PID 参数或限速只重置变化轴的 I/D 状态；仅改注释或相同值不重置。
后台出现 `validated ... pending` 表示等待控制周期，`applied ... tuning` 表示已经应用。
切换模式前应停止程序，再修改配置并重启。

## 传输与诊断

`hal/serial` 同时存放基础链路和云台速率适配，但分别构建；依赖方向为
云台速率适配 → IMU → 基础串口。只有 ControllerLink 线程读写串口，重连不重放旧命令。
运行时适配器实现原角度命令接口；速率传输接口 `SendRate(target, rate)` 保留目标、
时间戳和摩擦轮/拨弹元数据。无效目标时停止运动及拨弹，允许原有的零速摩擦轮续转。
反馈过期、发送失败或停止会清除 PID 状态，现有恢复策略和 MCU 看门狗不变。

Foxglove `/vision/control/tracking` 新增 `control_mode: pid/mpc`。
PID 项继续使用 `serial_pid_active`、`pid_yaw_*`、`pid_pitch_*`；串口 queued 字段表示
上位机入队状态，不是 MCU 回执。历史 `mpc_valid`、`raw_mpc_*` 字段保留兼容，
PID 模式下代表直接参考输出；两个 MPC 求解迭代数为零，不表示求解失败。
命令来源为 `pid`、`mpc`、`trajectory_fallback` 或 `stop`。

硬件不具备时只报告软件验收；实机跟随、热更新与断线恢复效果待用户实机验证。
