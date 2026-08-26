# 运行时错误与降级策略

`RuntimeSupervisor` 是相机、正式视觉流水线、100 Hz 控制、Talos 命令通道、仿真评估、
调试窗口和诊断后端共享的线程安全健康控制面。正式数据仍按
`相机 → 视觉 → 预测 → 控制 → HAL` 流动；监督器只接收故障和恢复事件，不参与算法判定。

策略由 `src/config/runtime/error_policy.yaml` 配置。Schema 1 的全部字段均为必填正数，
未知字段、错误 Schema 或非法门限会在启动时返回退出码 1：

```yaml
schema_version: 1
camera:
  no_valid_frame_timeout_ms: 2000
control:
  command_sink_unhealthy_timeout_ms: 1000
optional_components:
  evaluation_disable_after_consecutive_errors: 1
  debug_window_disable_after_consecutive_errors: 1
```

## 固定处置

| 组件或故障 | 运行时处置 | 终止结果 |
|---|---|---|
| 相机 `TIMEOUT` / `INVALID_FRAME` | 跳过当前帧并共用无合法帧计时；合法帧立即恢复 | 持续达到 2 秒后安全停止，退出 4 |
| 相机 `DISCONNECTED` / `FATAL` | 立即安全停止 | 退出 4 |
| 正式视觉流水线异常 | 不重试且不生成伪空检测，立即安全停止 | 退出 5 |
| 控制快照或控制线程异常 | 发送无效停止命令并锁存首个故障 | 退出 7 |
| Talos 健康检查或命令发送失败 | 首次失败清除投影、禁止开火并发送无效停止；健康检查与发送同时恢复后清除故障 | 持续达到 1 秒后退出 7 |
| 仿真评估异常 | 达到配置次数后禁用本进程内评估 | 不终止 |
| 调试窗口异常 | 关闭并禁用窗口；主动关窗仍为正常退出 | 不终止 |
| Foxglove / MCAP 故障 | 各 sink 独立停用；全部不可用时标记诊断降级 | 不终止 |

无检测、跟踪器 `DETECTING` / `LOST`、预测过期、外部控制未开启、MPC 求解失败和既有
100 ms 轨迹回退均属于正常正式状态，不会上报运行时故障。系统不自动重连相机或 Talos，
不切换后端，也不执行 OpenVINO CPU 回退。

## 健康状态与退出

每个组件具有 `HEALTHY`、`DEGRADED` 或 `FAILED` 状态。日志只记录进入降级、恢复、禁用和
终止等状态转换。首个终止故障一旦锁存，后续恢复或其他故障不会覆盖它。进程退出码保持：
正常 0、通用启动/配置 1、检测器初始化 2、相机打开 3、相机运行 4、正式视觉流水线 5、
Talos 命令通道打开 6、控制运行 7。

确定性策略与安全停止验收：

```bash
./build-openvino/bin/mv-runtime-policy-test
```
