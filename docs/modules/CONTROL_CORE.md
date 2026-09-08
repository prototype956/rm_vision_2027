# 同步控制核心与策略接入

阶段一保留现有规则、弹道及 TinyMPC，将控制拆为 `RulePolicy`、`FireControl`、
`ControlSession` 与负责线程/通信的 `ControlRuntime`。正式主程序默认仍使用规则策略。
本接口不是模型张量、Gym 环境或 Talos 协议；Talos 保持 v7。

## 同步调用

```cpp
mv::modules::ControlSession session(fire_config, planner_config);
auto result = session.Step(input, actuator, healthy, now, transport_timestamp_ns);
const bool sent = sink.Send(result.output.command);
session.AcknowledgePublication(result, sent, now);
```

每会话由单一调用序列拥有。`Step` 和确认必须交替；不得混用不同会话、旧周期或重复确认。
确认时间不得早于当前 Step。调用方负责在异常时停止输出；主程序已有异常安全停止路径。
`now` 是调用方的单调控制时间，同回合必须递增，零时间合法。
传输时间戳只用于命令关联与遥测年龄，必须与输入传输时间戳同域；训练可使用人为纪元。
核心不读取真实时钟推进控制；求解性能计时仅用于诊断。

预先确定的脚本动作可通过 `Step` 最后一个可选参数传入。需要使用当周期反馈和候选观测时：

```cpp
mv::modules::ControlPolicy policy =
    [](const mv::modules::ControlInputSnapshot& projected,
       const mv::modules::PolicyObservation& observation) {
      // projected 供 FireOnlyPolicyAdapter 复用规则选择；模型只使用 observation 白名单。
      return mv::modules::PolicyDecision{observation.action_mask[1] ? 1 : 0};
    };
auto result = session.StepWithPolicy(input, actuator, healthy, now, transport_timestamp_ns, policy);
session.AcknowledgePublication(result, sink.Send(result.output.command), now);
```

会话先融合反馈、投影位姿并处理回合/目标/模式复位，然后构建观测、同步调用一次策略，
再对选定槽位求解 MPC。输入无效时直接产生停止命令，不调用策略。回调引用不得保存，
也不得重入会话。回调异常取消本地脉冲并向调用方传播；模型故障停用锁存由后续部署层实现。
策略自有历史需根据回合、`track_generation` 与模式变化复位。

## 动作与约束

动作版本 1：0 为 WAIT；`1+2*i` 跟踪槽位 i；`2+2*i` 跟踪该槽位并请求单发，i 为 0–3。
WAIT 不换板、不创建请求，有效旧槽位继续跟踪。外部动作立即换板，允许同周期请求射击。
非法动作停止本周期控制。没有新请求不等于立即拉低正在输出的已接受脉冲。

规则策略保留观察角滞回、切换确认、误差窗口、稳定周期与不确定度判断。
外部策略不套用这些经验阈值。公共层仍执行数据有效性、跟踪状态、使能、裁判、弹道、
MPC 和脉冲间隔约束；动作掩码只使用估计和公开信息，最终执行仍可能被拒绝。
`FireOnlyPolicyAdapter` 每周期按规则选板，不能通过 WAIT 保持旧槽位；无候选才允许 WAIT。

`shot_requested` 是本地请求，`shot_accepted` 是本地脉冲接纳，`published_valid` 是发布成功。
这些字段均不是实际出膛确认。脉冲忙或间隔不足直接拒绝，不排队；失效、禁发、求解失败、
发布失败会取消本地脉冲，但不能撤回已传输请求。

## 裁判与复位

`RefereeObservation` 只保留自身存活/允许发射、禁发标志、热量/冷却和弹量信息。
`AdaptRefereeObservation` 检查回合、采样时间、枚举及数值，排除伤害/命中/出膛等评估计数。
缺失或过期时继续有效跟踪并禁发；`timing.max_referee_age_s` 默认 0.30 s，可配置。
年龄由同帧仿真采样差与本机单调经过时间相加，不能直接相减不同纪元的时间戳。

新回合或显式 `Reset` 清空全部控制状态并允许新时间原点。目标成功重建时预测器增加
`track_generation`；清空旧目标槽位、稳定性和可回退轨迹，但保留己方反馈与机械间隔。
TEMP_LOST 恢复不增加代次；短时可跟踪但禁发。MPC 失败仅可用已成功发布、同目标同槽位的
有限轨迹，最长 100 ms，始终禁发。

## 构建与验收

```bash
cmake -S . -B build-control-core -DBUILD_CONTROL_CORE_ONLY=ON \
  -DBUILD_MAIN=OFF -DUSE_OPENVINO=OFF -DUSE_MINDVISION_SDK=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build-control-core --parallel 4
./build-control-core/test/control/mv-control-acceptance src/config/modules
./build-control-core/test/control/mv-control-reference src/config/modules
./build-control-core/test/control/mv-control-session-reference src/config/modules
```

独立核心依赖 Eigen、OpenCV core、Ceres Jet 头文件、yaml-cpp、fmt、spdlog、Threads；
不链接相机 SDK、检测器、Talos、Foxglove 或窗口。静态库使用 PIC，可供后续语言绑定使用。
完整构建命令沿用仓库指南。ARM64 编译、同步仿真和策略训练属于后续阶段。

Foxglove `/vision/control/state` 包含本地请求/接受、`runtime.referee_age_s` 及
`runtime.control_compute_time_us`。后者计入同步计算、快照复制与发布结果处理，排除 Send；
线程调度迟到、周期长度和计算耗时分开统计，完整闭环验收必须确认外部控制确实开启。
