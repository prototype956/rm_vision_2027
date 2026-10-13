# 实机姿态链路 v1

## 2026-10-12 运行中通信恢复

`src/config/runtime/error_policy.yaml` 的
`control.command_sink_wait_for_recovery: true` 使控制通道故障持续进入 safe-stop，
不再因 1 秒不健康而终止整个程序。省略或设为 false 时仍使用原来的终止超时。
这只改变控制通道故障的退出策略；相机故障、控制线程异常等仍按原策略处理。

- 姿态有效期仍不超过 100 ms，负数据年龄仍不允许控制，不外推、不把负值当成新鲜值。
- 姿态过期后清空 PID 状态并尝试发送零速率、禁用、无发射请求的命令。
  物理链路断开时无法保证停机包送达，由 MCU 原有 200 ms 命令超时兜底。
- 普通控制会话在同步后，姿态持续无效/过期/来自未来 500 ms，关闭并重新打开串口；
  1 秒后重试，使用新会话重新 HELLO/SYNC，清空姿态历史及旧待发命令。
  链路层收包/同步超时 3 秒也走重开流程，失败后继续重试。
  手眼标定不因机构未就绪触发上述 500 ms 姿态恢复。
- 恢复须重新满足同步、姿态新鲜度和目标有效性；是否执行视觉命令仍由 MCU 遥控器模式仲裁。
  未更改手动/自动拨杆逻辑，也未更改正常的摩擦轮与拨弹轮关系。

日志新增 `rx_age_ms`（最后有效会话帧）、`attitude_rx_age_ms`（最后姿态包）、
`progress_age_ms`（最后新有效姿态样本）、`packet`、`sample`、`duplicates`、`generation`。
收包年龄增长指向断流；收包年龄小但样本进展年龄增长指向重复/无效姿态。
若包和样本都更新而映射年龄异常，则继续检查时钟同步。未知年龄为 -1；
超过 1 秒未收到姿态包时频率清零，不继续展示断流前的 200 Hz。

此修改修复恢复路径，不声称已经查实拨杆切换附近断流的硬件或固件根因。
编译/离线验收不接触电机；断流、设备插拔、切换手动后持续上报以及恢复控制需实机验收。

上位机 `hal/serial/attitude_wire.h` 与云台板 `app_attitude_link` 使用同一份 `attitude_wire.h`。
USART1：460800、8N1、无流控。上位机只发送会话和同步消息，不发送运动/开火命令。
新链路不改变底盘固件；同一端口仍支持旧视觉帧和 PC 调参协议。

## 字节布局

所有整数小端，float 为 IEEE754 binary32。不得发送原生结构体内存。
CRC 为 CRC16-CCITT-FALSE：poly=0x1021、init=0xffff、refin=false、refout=false、xorout=0。
覆盖从帧头到 payload 最后一个字节，CRC 自身以小端存储；`123456789` 的 CRC 为 0x29b1。

| 偏移 | 字节 | 内容 |
|---|---:|---|
| 0 | 2 | A6 5A |
| 2 | 1 | 版本，固定 1 |
| 3 | 1 | 消息类型 |
| 4 | 2 | payload 长度 N |
| 6 | 4 | 序号 |
| 10 | 8 | 上位机会话标识，非零 |
| 18 | N | payload |
| 18+N | 2 | CRC |

总长为 20+N，当前最大完整帧为 116 字节，接收缓冲区至少为 128 字节。
不同会话的回复和姿态不能混用。

| 类型 | N | payload | 序号含义 |
|---|---:|---|---|
| 1 HELLO | 0 | 无 | 请求序号 |
| 2 ACK | 0 | 无 | 回显 HELLO 序号 |
| 3 SYNC | 8 | t1:uint64，上位机单调微秒 | 请求序号 |
| 4 SYNC_REPLY | 24 | t1,t2,t3，各 uint64 | 回显 SYNC 序号 |
| 5 ATTITUDE | 42 | 见下表 | 会话内每个 5 ms 发送机会递增，允许跳号 |
| 6 CALIBRATION_SAMPLE | 96 | 前 42 字节同 ATTITUDE，后续为关节标定数据 | 同 ATTITUDE |

ATTITUDE 的 payload 相对偏移：

| 偏移 | 字节 | 内容 |
|---|---:|---|
| 0 | 8 | IMU 软件读取开始时刻，MCU 启动后微秒 |
| 8 | 4 | INS 更新序号，与传输序号不同 |
| 12 | 2 | 状态 bit0=姿态有效，其余保留为 0 |
| 14 | 16 | qw,qx,qy,qz，解算机体系到惯性参考系 |
| 30 | 12 | wx,wy,wz，解算机体系角速度，rad/s |

姿态完整帧为 62 字节，200 Hz 时占单方向线路约 27%。
INS 序号约按解算频率变化，不能用它的间隔计算串口丢包；诊断丢包使用传输序号。
无效帧保留旧样本时间/序号并清除有效位，禁止将旧姿态标为新采样。

`SerialImu` 同时接受 type 5 和 type 6。对 type 6，除通用帧校验外还要求 payload 偏移
71 的 `payload_version == 2`、偏移 70 的 `gyro_calibration_status == 2`、标定机构状态位
满足采集门槛、保留字段为 0、扩展浮点数和方向字段有效，且姿态状态同时具有 READY 与
FINITE；手眼求解使用四元数，角速度仅参与有效性检查和诊断，不做积分。

## 时间与恢复

`t1` 在上位机开始写请求前记录；`t2` 是下位机接收完整有效请求时刻；
`t3` 是下位机开始响应 DMA 前的时刻；`t4` 是上位机读到完整响应时刻。

```
host_minus_mcu = ((t1 - t2) + (t4 - t3)) / 2
rtt = (t4 - t1) - (t3 - t2)
host_sample_time = mcu_sample_time + host_minus_mcu
```

采用最近 10 秒、最多 32 个同步样本中 RTT 最低者；RTT 超过 20 ms 拒绝。
至少 3 个有效同步样本后启用姿态插值。初始每 100 ms 同步，稳定后每秒一次。
双向串口序列化、USB 缓冲和任务调度存在不对称延迟，因此这是软件时间估计。

3 秒无有效链路消息或同步更新、串口断开、MCU 时间/序号回退时重建会话，清空姿态缓存。
下位机 3 秒无有效会话请求恢复旧视觉发送。新会话不接受旧会话回复。
DWT 的 64 位扩展依赖至少每个硬件计数回绕周期调用一次；正常 1 ms 任务满足此要求，
不能保证调试器暂停 CPU 超过一个回绕周期时仍恢复真实时间。

## 发送所有权

`AttitudeLink_Service` 在云台任务执行：同步/握手优先，其次最新姿态，再允许旧协议发送。
`AttitudeLink_SendLegacy` 统一复制到独占 DMA 缓冲区；忙时返回 0，由原调用方保留原来的重试语义。
姿态只有一个待发送槽，拥堵时新样本替换旧样本。大于 96 字节的既有 CAN trace 仍可发送，
它使用独立旧格式，统一 TX 缓冲区容量为 192 字节。
RX 中断只记录有界请求，时间同步应答在任务中发送。

## 坐标和占位外参

`INS.q` 直接发送，不应用下位机 Euler yaw 的 -90° 修正。
IMU 轴是经过固件安装/参数修正后实际参与 EKF 的轴，不能只看传感器丝印。
最终 pitch 刚体系定义 X 前、Y 左、Z 上，相机光学系 X 右、Y 下、Z 前。

```
R_WG = R_WI * transpose(R_GI)
p_G = R_GC * p_C + t_GC
p_W = R_WG * p_G  # 本阶段忽略平台平移
```

当前车辆 IMU 名义旋转已按 2026-09-22 单轴实验固定为 `diag(-1,-1,1)`，即 X 后、Y 右、Z 上；
实验仅大致水平，未采用细小拟合偏角。相机外参已使用 2026-10-10 的 30 组固定板手眼数据求解并完成实机检查：

```text
R_GC = [-0.0006241308,  0.0291797530,  0.9995739855;
        -0.9999844861, -0.0055510177, -0.0004623408;
         0.0055351619, -0.9995587668,  0.0291827649]
t_GC = [0.1453868590, 0.0007650707, 0.1585809998] m
```

枪口外参仍为单位变换占位值，平台平移仍按零处理，因此配置继续保留 `placeholder: true`。
该标志不再表示相机外参为占位值。world 在本阶段只有惯性方向，原点近似随云台移动，
不是导航固定世界系。大 yaw、小 yaw、pitch 的共同转动已经体现在四元数中，不重复叠加。


## 上位机职责边界

- `hal/serial/controller_link`：设备打开与读写、收发线程、分帧与 CRC 校验、会话握手、
  时钟映射和断线重连；不依赖 Eigen、IMU 或 FramePacket。
- `hal/serial/attitude_wire.h`：共用线协议及基础字节编解码。
- `hal/imu/serial_imu`：解释姿态载荷、校验并归一化四元数、检查样本顺序、缓存和插值，
  通过串口模块提供的时钟偏差匹配图像时间。角速度用于诊断及实机控制反馈，不做积分。
- 串口模块通过状态/消息回调向 IMU 传递数据，IMU 互斥保护自己的样本和健康快照；
  新会话代次清空缓存，销毁时先结束串口回调，再释放 IMU 状态。
- 保留 `SerialImu` 以及 `hal/imu/serial.yaml` 作为现有运行时的配置兼容入口。
  `device`、波特率和同步周期转交串口模块，缓存及插值参数由 IMU 模块消费。
  基础信封及 CRC 保持兼容；使用 type 6 标定采集或 type 7 云台控制需对应固件。
- `geometry/gimbal_geometry.hpp`：固定安装外参模型、旋转矩阵合法性检查及
  `R_WG = R_WI * R_GI^T` 的纯数学计算，仅依赖 Eigen 和基础刚体变换。
- `runtime/real_frame_geometry`：读取配置、核对相机尺寸/ROI、按图像时间查询姿态、
  检查时钟代次、装配 `FramePacket` 并输出诊断。YAML 和硬件依赖不进入 geometry。

部署入口为 `runtime/real_geometry.yaml`，通过 `active_profile` 选择串口、相机内参和安装外参。
所有引用路径相对配置根目录，支持现有 `RM_VISION_CONFIG_ROOT`。详情见
[实机部署配置](../test/real_imu_pnp.md#车辆部署选择)。

HAL 的四元数与角速度保留 IMU 解算机体系语义；运行时调用几何层统一入口：
姿态为 `R_WG = R_WI * R_GI^T`，角速度为 `omega_G = R_GI * omega_I`。
角速度日志分别标注 `gyro_imu_rad_s` 与 `gyro_gimbal_rad_s`，
不是相机帧时刻插值后的角速度，不用于积分或预测；实机控制使用最近有效样本。原始单轴采集 CSV 不应用安装旋转。
此变换只执行一次，不再叠加电控 Euler yaw 的 -90° 或关节角；惯性 yaw 零点不属于安装外参。

## Phase 2A 实机云台命令（type 7）

Phase 2A 在同一 session 中增加 `GIMBAL_COMMAND = 7`，不创建第二个串口线程。控制线程只更新 latest-wins 命令槽，`ControllerLink` 收发线程仍是文件描述符的唯一读写者。

payload 固定为 12 字节：

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 4 | `yaw_rate_rad_s : float32` |
| 4 | 4 | `pitch_rate_rad_s : float32` |
| 8 | 2 | `flags : uint16`，bit0 ENABLE，bit1 FIRE_REQUEST（拨弹），bit2 FRICTION_REQUEST（摩擦轮） |
| 10 | 2 | `reserved = 0` |

完整帧为 32 字节。下位机要求两轴速率有限且绝对值不超过 `4 rad/s`，sequence 在 session 内前进；命令超过 200 ms、新 HELLO、重连或 session 超时后失效。FIRE_REQUEST 会映射到原有 `SHOOTER + FIRE` 控制标志。

2026-10-12：新增独立摩擦轮请求 bit2；仅该位不触发拨弹。两种请求均需要 ENABLE；
FIRE_REQUEST 兼容旧 host，仍隐含开启摩擦轮。flags=5 为只开摩擦轮，flags=7 为同时拨弹。
摩擦轮续转期间若运动目标无效，发送两轴零速率、ENABLE+FRICTION，禁止 FIRE。
上位机自最后有效可见瞄准目标起续转 1 秒；通信超时不续转。
必须先更新云台固件再使用新 host：旧固件会拒绝含 bit2 的整条命令；帧版本和长度不变。

MindVision 运行时通过 `runtime/serial_gimbal_control_adapter` 调用独立 PID 或 MPC 跟随环，
将绝对 yaw/pitch 目标转换为限幅速率命令；`hal/serial/serial_gimbal_command_sink` 负责传输。实际角度来自同一 SerialImu 姿态与手眼安装旋转，实际角速度来自 `omega_G = R_GI * omega_I`；yaw 使用 `omega_G.z`，pitch 使用 `-omega_G.y`。

Phase 2A 临时禁火层已经解除：串口 sink 按 `fire_output_enabled` 编码发射请求，固件使用 `PHASE2A_FIRE_OUTPUT_INHIBIT=0`。原有 `FireControl` 的裁判数据与瞄准条件仍决定上层是否产生 fire。当前 `modules/gimbal_control.yaml` 的 yaw/pitch 输出限速为 `3.0/4.0 rad/s`；启动时会一直等待串口同步成功，并由底层周期重连。详见 [Phase 2A 实机接入](competition_phase2a.md)。


## 统一 IMU 诊断入口

`./scripts/calibrate_camera.sh` 主项 `4` 提供安装轴向采集、静止漂移采集及漂移分析。
轴向程序也支持无参数启动，默认读取当前车辆 profile；历史 `DEVICE [NEW_OUTPUT_DIRECTORY]`
命令保留。原始轴向 CSV 不应用安装旋转，新增 YAML 配置快照供实验追溯。

轴向和漂移接受 type 5/6，复用基础载荷解码及结构检查。type 6 要求 payload v2、
合法保留/方向/浮点字段、READY/FINITE 和有效姿态；陀螺标定及机构状态只记录到协议诊断
CSV，不阻断诊断实验。`SerialImu`/手眼仍保留前述严格门槛，手眼强制 type 6。
详细命令、依赖与数据说明见 [统一入口](../tool/CALIBRATION.md)。

算法、模式选择和配置迁移见 [云台 PID / MPC 控制](../modules/GIMBAL_CONTROL.md)。
