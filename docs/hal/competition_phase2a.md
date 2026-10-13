# Phase 2A MindVision 实机云台接入

## 2026-10-12 摩擦轮预转与拨弹分离（需要更新云台固件）

| 条件 | 摩擦轮 | 拨弹盘 |
| --- | --- | --- |
| 未见有效自瞄目标，且不在续转期 | 停 | 停 |
| 出现有效目标，尚未满足开火条件 | 转 | 停 |
| 满足原有发射条件，fire 脉冲有效 | 转 | 转 |
| 目标短暂丢失，距最后有效目标未满 1 秒 | 续转 | 结束连续拨弹，补齐当前格 |
| 续转到期、外部控制关闭或反馈/通信异常 | 停 | 停 |

“有效目标”沿用现有自瞄判定：DETECTING/TRACKING 状态、数据新鲜、能够形成有效瞄准命令；
不是任意原始检测框。已有目标时不必等待瞄准进入发射窗口才开始摩擦轮预转。
TEMP_LOST、旧帧或轨迹回退不刷新最后见到目标的时间。续转只针对摩擦轮，不延长任何 fire 脉冲。
运动外推到期后继续续转时，上位机发送零 yaw/pitch 速率，不继续追赶旧目标。

修改 `src/config/modules/fire_control.yaml` 的 `timing.friction_hold_s: 1.0` 可调整续转时长，
单位秒，允许 0～10，0 表示关闭延时；此文件需要重启生效，不属于 PID YAML 热更新。
`hal/serial/gimbal.yaml` 的 `fire_output_enabled` 仍统一控制摩擦轮和拨弹输出。
fire_control 的 `enabled/auto_fire` 控制拨弹建议，不阻止有效目标下的摩擦轮预转。
现有瞄准、发射脉冲、裁判配置保持原值；本次不增加“摩擦轮已达转速”判定，因此预转不等于
已验证轮速就绪，需要现场确认启动加速时间。双轴 PID 参数不改。

协议 type 7 的 flags 新增 bit2 `FRICTION_REQUEST`。bit1 `FIRE_REQUEST` 为拨弹请求，
兼容旧上位机时仍隐含摩擦轮开启；bit0 ENABLE 必须有效。payload 仍为 12 字节，整帧 32 字节。
只开摩擦轮时 flags=5；同时拨弹时 flags=7；停止两者时摩擦轮/拨弹位均为 0。
新固件接受旧 host 的 flags=3；旧固件不认识 bit2，会拒绝包含该位的整条命令。
**先停止上位机，Keil 编译并烧录新版云台固件，再启动已更新的 NUC 程序。**

按分格送球机构要求，固件保留 Feeder_FINISH：摩擦轮仍开启时撤销连续拨弹，补齐当前格并
保持对齐，不连续送下一格。仅预转、尚未拨弹时保持 Feeder_NULL，不追赶旧角度。
补齐期间仍运行位置/速度闭环，不清积分；只有 Feeder_NULL 强停状态才清除拨弹盘速度 PID
并输出零力矩。摩擦轮 PID、转向、转速不改；摩擦轮关闭、失能等强停仍优先于补齐。
手动/自动接管、200 ms 命令超时、大 yaw 禁用等现有规则保持不变，断线不使用 1 秒续转。

Foxglove `/vision/control/tracking` 新增：

- `friction_requested`：本周期控制会话的摩擦轮请求。
- `friction_queued`：上一次成功放入串口队列的摩擦轮请求。
- `feeder_queued`：上一次成功放入串口队列的拨弹请求。

queued 不是 MCU 回执或实测转速，通常比本周期请求晚一拍。
先观察“出现目标→预转”“遮挡目标→结束连拨并补齐当前格、轮续转→1 秒后轮停”
“重新出现→恢复预转”。补齐当前格可能将该格内的小球送出，这是分格机构的预期行为。

验证：NUC 主程序编译通过，既有控制验收 `PASS 268 checks`；云台既有静态回归通过（含 8 种
ENABLE/FIRE/FRICTION 组合）。未启动实机、仿真或自动烧录；本机没有 Keil，完整固件编译与
烧录由用户完成。NUC 无 clang-format/clang-tidy/clangd，未执行这些检查。
NUC 修改前备份：`artifacts/backups/shooter-split-before-20261012.tar.gz`。

## 当前控制结构与调参入口

MindVision 默认双轴 PID，Talos 继续 MPC。模式、PID 参数、MPC 参数和串口输出配置
已分离，历史开关控制实验模式已移除。当前操作以
[云台 PID / MPC 控制](../modules/GIMBAL_CONTROL.md) 为准。

PID 调参文件为 `src/config/modules/gimbal_control/pid.yaml`；实机输出限速在
`src/config/modules/gimbal_control.yaml`，发射使能在 `src/config/hal/serial/gimbal.yaml`。
PID 数值与速率上限保存后支持热更新，模式切换仍需重启。默认数值保持 yaw
10.0/0.2/0.5、pitch 8.5/0.5/0.8，实机输出限速 3/4 rad/s。

<details>
<summary>重构前的阶段与调参记录（仅供历史参考）</summary>

以下配置路径、实验开关和操作命令属于重构前版本，不能用于当前程序。
特别是 pitch 开关控制实验已删除；历史验收数字不代表当前重构已重新执行。

## 2026-10-12 默认采用双轴 PID（当前部署）

MindVision 实车默认使用现有可变积分 PID，不再使用 MPC。下文的 MPC 对比及旧调参记录
仅作历史参考；旧代码保留，但 PID 运行期间不初始化、不求解 MPC，也不会自动回退到 MPC。

控制链：视觉目标/预测与弹道角度参考 → yaw/pitch PID → 限幅角速度 → 串口 → 下位机控制环。
现有参考时域保持 10 ms × 50 点，PID 取下一参考点（index=1）；主程序不再读取
`modules/gimbal_trajectory_planner.yaml` 的 MPC 配置。诊断中的命令前视时间修正为实际 10 ms。
保留 `test_pid_bypass: true` 这个历史字段名以兼容配置及热更新；省略时也默认 PID。
必须提供 `pid.yaw` / `pid.pitch` 参数，缺失报错，不会悄悄切回 MPC。
`test_pitch_bang_bang` 保持 false。旧模式只有显式关闭 PID 才能进入，正常使用不要修改模式开关。

唯一实车 PID 调参文件：`src/config/hal/gimbal/serial.yaml`。
本次保留现场数值：yaw Kp/Ki/Kd = 10.0/0.2/0.5，pitch = 8.5/0.5/0.8；
yaw/pitch 最大速率 = 3.0/4.0 rad/s。积分分离、可变积分、限幅、D 滤波和 YAML 热更新均保留。
旧 `control` 节仍为兼容保留并进行配置校验，但其增益不参与 PID 输出，不要用它调 PID。
Foxglove 继续查看 `serial_pid_active`、`pid_yaw_*`、`pid_pitch_*` 和实际姿态反馈；
历史 `mpc_valid` 等字段名仍存在，不表示 MPC 求解器正在运行。

更新主程序后首次需要重新启动，之后 PID 参数保存即可热更新。无需重新烧录下位机。
本次不改变开火、裁判测试开关、底盘或大 yaw 配置，不自动启动程序或操作电机。
修改前备份：`artifacts/backups/pid-default-before-20261012.tar.gz`。

验证：主程序及现有控制验收编译通过，`PASS 229 checks`，`git diff --check` 通过。
新增检查覆盖默认 PID、无效 MPC 权重不影响 PID、无 MPC warm start、目标切换及非法参考拒绝。
NUC 未安装 clang-format/clang-tidy/clangd，未执行这三项检查；未启动实车或仿真，实车待用户验证。

## 2026-10-12 连续调参 / YAML 热更新

安装此版本后启动一次主程序即可。在另一个 NUC 终端编辑：

```bash
nano ~/Workspace/rm_vision_2027/src/config/hal/gimbal/serial.yaml
```

修改后按 `Ctrl+O`、回车保存，不必退出编辑器或视觉程序。后台每 250 ms 读取
一次，内容连续两次相同后校验，通常保存后约 0.25～0.5 s 进入控制线程。
日志 `GimbalTuning APPLIED` 才表示整组参数已应用；`validated ... pending`
仅表示已等待下一个控制周期。YAML 语法或范围错误显示 `rejected tuning`，
继续使用上一组有效参数，修正并再次保存会自动恢复加载。

可热更新：`pid.yaw` / `pid.pitch` 全部字段、`control` 下的串口增益/前馈/阻尼、
`limits.max_yaw_rate_rad_s` / `limits.max_pitch_rate_rad_s`、开关实验的死区。
修改了哪个轴的 PID 参数或速度上限，就清空哪个轴的 I 与 D 滤波状态；
仅修改注释、保存相同值不会清空状态。两轴配置在同一控制周期整体应用，
文件读取和 YAML 解析均不在 100 Hz 控制线程执行。

仍须重启：PID/MPC/开关控制的模式开关、`enabled`、`fire_output_enabled`、
`max_feedback_age_ms`，以及其他文件（例如 MPC 权重配置）的改动。
若一次保存混入这些需重启字段，整次热更新拒绝，不会只应用部分参数。
读取失败或短暂文件不存在不会停止原控制；恢复有效文件后继续调参。

本次不改现场 PID 数值或速度上限；部署时已有 pitch Kp=40、最大速率=4 rad/s。
保存即会改变控制输出，不是预览。建议每次只动一个参数，记录超调与稳定时间；
热更新会清空该轴已有积分，因此保存后的第一段过渡不要和稳定段混在一起比较。

验证：主程序编译通过；现有火控验收 `PASS 224 checks`，包含真实后台文件读取、
原子保存、相同值不更新、非法 YAML/范围拒绝及恢复、重启专用字段拒绝；
`git diff --check` 通过。未自动启动视觉程序或驱动电机，实车热调效果待用户验证。
NUC 缺少 clang-format/clang-tidy/clangd，本次未执行这些静态检查。
修改前备份：`artifacts/backups/pid-hot-reload-before-20261012.tar.gz`。

## 2026-10-12 双轴可变积分 PID / MPC 对比

配置文件：`src/config/hal/gimbal/serial.yaml`。PID 数值支持上述保存即生效；
切换控制模式仍须正常退出并重启 `./build-openvino/bin/mv-vision-main`。
无需重新编译或烧录。

| 模式 | test_pid_bypass | test_pitch_bang_bang |
| --- | --- | --- |
| 双轴可变积分 PID（当前实验配置） | true | false |
| 原有双轴 MPC + 串口跟随环 | false | false |
| 原有 pitch 开关控制、yaw MPC | false | true |

两个实验开关不能同时为 true，否则配置校验报错。PID 模式只用于 MindVision 实机
入口；不执行 yaw/pitch MPC 求解，直接取参考时域 index=1 的角度，输出受串口
`limits.max_yaw_rate_rad_s` / `max_pitch_rate_rad_s` 限幅的角速度。
原 `control` 中的 kp、近目标增益、前馈、阻尼在 PID 模式下不参与输出。
MPC 的权重、前视时间、速度/加速度限制不作用于 PID 输出；下位机控制环、机械
限位和命令超时仍保留。不是电机力矩 PID，也不是 MCU 内环的替代。
开火配置及裁判测试开关按用户要求保持现值，不因 PID 模式切换自动改变。

### PID 定义

两轴分别配置 `pid.yaw` 和 `pid.pitch`，单位统一使用 rad 和秒：

```text
e = reference_angle - actual_angle    # yaw 包角到 [-pi, pi]
P = kp * e
D = -kd * lowpass(measured_angular_rate)
I += ki * weight(abs(e)) * e * actual_dt
rate_command = clamp(P + I + D, -max_rate, +max_rate)
```

`weight` 在 `|e| <= integral_full_error_rad` 时为 1；在 full 与 stop 之间
线性降为 0；到达 `integral_stop_error_rad` 后为 0，并清除已有 I。
因此此处“可变积分”是误差越大积分越弱，不是自动调 Ki。
I 项本身限幅到 `±integral_limit_rad_s`；输出饱和时禁止继续同向累积，但允许
反向消除积分。`ki=0` 清除 I；进入 `deadband_rad`（含边界）时清除 I、输出零速度。
D 使用实测速率，目标阶跃不会产生误差差分冲击；`derivative_filter_tau_s` 是
一阶滤波时间常数，0 为不滤波。增大它会更平滑但也增加制动滞后。
启动、无效命令、发送失败、连接不健康、控制周期间隔超过 100 ms 时重置 PID。
只保证主机输出限幅，不能消除下位机已累积的目标角。

参数须有限且非负，满足 `deadband_rad <= integral_full_error_rad < integral_stop_error_rad`。
当前起始值（仅为手工调参基线，未实车验证）：yaw Kp=1、pitch Kp=2，Ki/Kd 均为 0；
full=0.02 rad（1.15°）、stop=0.10 rad（5.73°）、I 上限=0.20 rad/s、D 滤波=0.03 s、
死区=0.002 rad（0.115°）。这意味着最初实际是 P 控制，可变积分待用户设置 Ki 后启用。

### 手工调参顺序

1. 一次只调一个轴，固定目标与测试幅度；另一轴保持参数不变。
2. 先令 Ki=Kd=0。Kp 从起始值每次增加约 20%；变快但明显超调/往复时退回上一档。
   若 `pid_*_rate_command` 已长期达到限速，继续加 Kp 不会提高峰值速度。
3. 再小步加入 Kd（可从 0.05 起，每次增加 0.05），观察是否减少超调。
   过大可能迟钝或放大速率噪声；不要在此阶段同时改 Kp 和滤波时间常数。
4. 瞬态稳定后，只有静止目标仍存在持续偏差时才加 Ki（可从 0.1 起，每次增加 0.1）。
   这些是试调步长，不是已验证的最优值。误差已在死区内时不会积分；
   若误差大于 stop，先检查跟随能力，不要通过不断增大 Ki 处理。
5. 出现慢周期来回摆动时优先减小 Ki，观察 I 曲线是否长时间顶到上限。
6. 对比 MPC 时将两个开关都设为 false，保持固件、机械、目标、距离及动作幅度不变。
   比较上升时间、超调、稳定时间、静止偏差；不要只凭最终是否对准判断优劣。

### Foxglove 观察

原 `/vision/control/tracking` 新增以下字段（yaw/pitch 各一组）：

```text
serial_pid_active
pid_pitch_error_rad
pid_pitch_p
pid_pitch_i
pid_pitch_d
pid_pitch_integral_weight
pid_pitch_rate_command
```

把 `pitch` 换成 `yaw` 可观察另一轴。误差单位 rad；P/I/D/输出单位 rad/s；
积分权重为 0..1。建议角度一张图（reference_next/published/actuator_actual），
P/I/D/输出另一张图，避免不同单位混在一起。此诊断来自最近成功排入串口队列的命令，
不是下位机收到或执行的确认；可能比本周期图表角度滞后一个控制周期。非 PID 模式
这些数值为 null。启动日志应显示 `TEST dual-axis variable-integral PID`，
两个 MPC 求解迭代数为 0；旧 `mpc_valid` 字段此时表示规划输出有效，不代表运行了 MPC。

手动遥控期间下位机可能不执行主机指令，这时不能用跟随曲线评估 PID；自动模式
测试前重启程序可清空此前的积分状态。首次验证不自动启动电机。

本次验证：主程序编译通过且新增初始化警告已消除；现有火控验收通过
`PASS 215 checks`，覆盖 PID 各项方向/单位、可变积分、饱和、死区、超时重置、
参数校验和两轴 MPC 旁路；`git diff --check` 通过。未启动实车、未运行闭环仿真。
NUC 未安装 clang-format/clang-tidy/clangd，未执行这些静态检查。
修改前备份：`artifacts/backups/pid-bypass-before-20261012.tar.gz`，新增文件
`src/hal/gimbal/serial/variable_integral_rate_pid.hpp` 不在修改前备份中。

## 2026-10-12 Pitch MPC 排除实验

在 `src/config/hal/gimbal/serial.yaml` 中设置：

```yaml
test_pitch_bang_bang: true
pitch_bang_bang_deadband_rad: 0.01
```

该开关仅作用于 MindVision 实机入口：不执行 pitch MPC 求解，直接使用
`pitch_reference_next` 对应的参考角；yaw 的 MPC 和串口控制参数保持不变。
串口 pitch 控制不使用 Kp、前馈或阻尼：以实时实际角计算误差，误差绝对值
大于 0.01 rad（约 0.573°）时，发送正/负 `limits.max_pitch_rate_rad_s`，
当前为 3 rad/s；误差在死区内（含边界）发送零速度。不直接输出最大力矩。
pitch 的 MPC 速度/加速度约束在此模式下不生效；协议速率范围和下位机限位仍生效。
本实验保留当前 `fire_output_enabled` 和裁判测试开关设置。

启动日志显示 `TEST pitch bang-bang`；pitch 求解迭代数应为 0，不能将此时
`pitch_solver.solved=false` 理解成求解失败。正常有效周期内，Foxglove 的
`pitch_published` 应与 `pitch_reference_next` 重合，yaw 仍取原有前视点。
测试目标丢失、反馈过期、yaw 求解失败等原有控制状态仍可能使输出停止或进入原有回退。

注意：零速度表示停止累加下位机目标角，不保证电机立即停在当前实际角。
本轮不修改固件目标角积分、MIT 环或补偿，以便隔离上位机 MPC 影响；bang-bang
本身也可能因延迟、参考角累积和离散采样出现往复超调，不能仅凭超调断定电机故障。

配置在启动时读取。停止旧程序后按原命令重启，无需烧录；恢复时将
`test_pitch_bang_bang` 改为 `false` 并重启，原 pitch MPC 和串口增益重新生效。

验证：主程序和现有火控验收编译通过，`mv-control-acceptance` 为
`PASS 196 checks`，`git diff --check` 通过。未运行闭环仿真或自动启动实车；
NUC 无 clang-format/clang-tidy/clangd，未执行这些工具检查。
修改前备份：`artifacts/backups/pitch-bypass-before-20261012.tar.gz`。

## 2026-10-12 临时无裁判测试模式

`src/config/modules/fire_control.yaml` 新增 `test_bypass_referee`，代码默认值为
`false`，当前 NUC 临时测试配置为 `true`。该开关同时跳过开火输出和策略动作筛选中的
裁判数据有效性、时效性、存活、供电/开火许可及弹量判定，不伪造裁判观测数据。
目标跟踪、反馈时效、弹道/MPC 有效性、原有瞄准窗口、开火周期和串口输出使能仍生效。

启动日志会显示 `TEST MODE` 和 `test_bypass_referee=true`。沿用原来的
`./build-openvino/bin/mv-vision-main` 启动方式；配置在启动时读取。
恢复比赛判定时将该字段设为 `false` 并重启程序，不需要重新烧录下位机。

注意：当前固件把摩擦轮启动和拨弹绑定在 FIRE 请求上；启用此模式后，满足其余
开火条件便可能同时驱动这些电机，并非只启用跟踪。无裁判模式不代表硬件供电/CAN
已验证，也不解决摩擦轮预热与拨弹时序问题。不要把测试开关用于正式比赛配置。

本次验证：`mv-vision-main` 与现有 `mv-control-acceptance` 编译通过；执行
`./build-openvino/test/control/mv-control-acceptance src/config/modules`，结果
`PASS 187 checks`，覆盖默认裁判门控、显式旁路、策略筛选及非裁判条件保留。
`git diff --check` 通过。NUC 未安装 clang-format、clang-tidy、clangd，相关静态检查
未执行；未运行闭环仿真、未自动启动实车，电机动作待用户验证。
修改前文件备份：`artifacts/backups/referee-test-before-20261012.tar.gz`。

以下为原 2026-10-10 阶段记录：

## 已实现

- MindVision 后端会创建并启动 100 Hz `ControlRuntime`。
- `RealFrameGeometry` 持有唯一 `SerialImu`/`ControllerLink`；姿态接收、同步和云台命令复用同一个 `/dev/serial/by-id/...` 设备。
- `SerialGimbalCommandSink` 使用实时姿态反馈，把绝对 yaw/pitch 轨迹闭环转换为 type 7 双轴速率命令。
- 姿态/命令不健康或样本年龄超过 100 ms 时停止外部控制。
- 启动时不再设置 5 秒上限；串口未就绪时持续等待，底层每秒周期重连，支持现场设备插拔，`Ctrl+C` 可终止等待。
- Phase 2A 临时禁火层已解除：上位机允许 type 7 携带 FIRE，固件 `PHASE2A_FIRE_OUTPUT_INHIBIT=0`。原有 `FireControl` 的裁判数据、瞄准条件等比赛判定仍保留。
- Release `mv-vision-main` 编译通过；现有 `mv-runtime-policy-test` 通过。

## 配置

串口：`src/config/hal/imu/serial.yaml`

```yaml
device: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
baud: 460800
```

云台闭环：`src/config/hal/gimbal/serial.yaml`

```yaml
fire_output_enabled: true
control:
  yaw_kp: 6.0
  pitch_kp: 6.0
limits:
  max_yaw_rate_rad_s: 3.0
  max_pitch_rate_rad_s: 3.0
  max_feedback_age_ms: 100.0
```

当前部署值为 `3.0 rad/s`；协议与固件仍拒绝绝对值超过 `4 rad/s` 的异常命令。固件命令看门狗为 `200 ms`。

## 尚未验证

- 新的正式云台固件尚未 Keil 编译、烧录。
- type 7 尚未做 `200 ms` 超时、运行中断线重连及 FIRE 位传输的实机测试。
- 未接入真实裁判数据、枪口外参和弹速来源。
- 未做底盘运动补偿或比赛动态测试。

当前物理发射机构已拆除，现场急停与机器人保护由操作人员负责；部署时应确认固件 `PHASE2A_FIRE_OUTPUT_INHIBIT=0` 与上位机 `fire_output_enabled=true` 和本次配置一致。

</details>
