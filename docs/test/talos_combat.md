# Talos v7 战斗遥测与允许发弹量验收

两端必须同时升级。Rust `crates/talos-ipc/src/layout.rs` 与视觉
`src/hal/camera/talos/talos_ipc_layout.hpp` 通过编译期断言固定 ABI；
`src/simulation/combat_frame_data.hpp` 保存经 HAL 校验的仿真附件。
战斗状态不参与目标选择、预测、瞄准或火控决策。

## 配置和启动

在模拟器 `config.toml` 的 `[combat]` 中配置：

```toml
controlled_allowance = { mode = "unlimited" }
target_allowance = { mode = "unlimited" }
# 有限训练场景示例：controlled_allowance = { mode = "limited", initial = 50 }
```

默认无限，只建模允许发弹量，不建模实际弹仓。有限额度只能是非负 `u32`；
仅在实际生弹后扣减一发。无额度、供弹等待、机械间隔拒绝、缺枪口均不扣减；
无限模式仍受生命、机械间隔与热量限制。配置在启动或 `R` 重置时统一应用，
不会在回合中热更新战斗属性。英雄假人为无发射机构受击目标。

正常使用 `./scripts/run_simulation_vision.sh`。Foxglove 连接 `ws://localhost:8765`：

- `/referee/self`：`self.hp`、`self.heat`、`self.heat_limit`、
  `self.cooling_per_second`、允许量、生命及发射许可；`valid=false` 时 `self=null`。
  可用 Plot 展示热量和上限，用 Raw Messages 查看多个禁射原因。
- `/simulation/combat/evaluation`：所有机器人当前状态、实际发射、拒绝请求、
  装甲接触、伤害命中、造成/承受的有效伤害、击毁及热量锁定次数/时长；事件用于审计。
  敌方 HP、归因和计数只放在这个评估通道，自身裁判通道也不夹带伤害归因。

话题按订阅需求编码。录制仍完全服从原有 `recording.enabled`，本功能不自动开启录制。
短时验收应先关闭录制，结束后恢复原配置。

## 时间与回合契约

- `sequence` 与话题的 `timestamp` 对应原始图像的采集序号和 Unix epoch 时间。
- `sim_time_ns` 为本次快照的 Fixed 仿真时间，进程内单调递增；
  `round_started_ns` 为本回合起点；`round_time_s` 为二者相减。
- 裁判 `sample_time_ns`、`sample_sequence` 指向最近一次真实采样。
  每回合按 100 ms 截止点，在 FixedLast（本物理步伤害结算后）采样；
  首次物理步及重置后的第一步立即采样。因物理步离散，采样可晚一个物理步，
  不倒填时间或伪造中间采样。图像可重复携带同一份样本。
- 样本不随画面每帧更新，`sample_sequence` 相同时，样本时间及自身状态必须相同。
  评估的当前状态可以比裁判样本新。样本序号在重置后继续递增。
- `round_id` 从 1 开始，`R` 增加；本回合计数归零。回合围栏阻止旧 GPU 回调发布。
  每帧有明确回合归属，HAL 拒绝跨回合的采样时间。
- 命令原样携带预测来源图像的回合、序号和采集时间。模拟器对有效目标要求回合匹配、
  来源采集时间非零，并保留 F5 启用时的生成/接收时间门限。旧回合图像产生的新命令
  也不能重新开火；停止命令仍允许安全停止。
- 本阶段不重置视觉跟踪器内部估计，不撤回已经送入视觉线程的旧图像；消费方按回合
  区分统计，旧命令由模拟器拒绝。不提供全链路仿真时钟或可加速 `reset/step` 接口。

## 编码与边界

机器人 `robot_id` 是稳定场景 ID：受控机器人 1，步兵目标 2，英雄目标 3。
图像中的机器人位置真值也用这些 ID；装甲真值新增 `owner_robot_id`（偏移 24），
非战斗装甲为 0。非战斗目标位置真值使用高位为 1 的实体 ID 命名空间。
外观分类标签与身份仍独立。

| 原始字段 | 编码 |
| --- | --- |
| team | 0 红、1 蓝 |
| role | 0 步兵、1 哨兵、2 英雄假人 |
| life | 0 存活、1 战亡 |
| shooter | 0 无、1 17 mm |
| allowance_mode | 0 无限、1 有限；无限时原始余额 0，JSON 余额 null |
| fire_blocks bit 0–7 | 战亡、无机构、普通热锁、整局锁、无额度、机械间隔、供弹等待、枪口缺失或无效 |

`fire_permitted` 等价于 `fire_blocks == 0`，表达物理发射条件，不包含 F5 外部控制开关；
它不是“下一请求一定发射”的承诺，也不主动预留下一发热量。请求拒绝事件保留实际的
`ExternalControlDisabled`、`QueueFull` 等原因；多个物理禁射位可以同时存在。
血量单位 HP，热量采用裁判热量单位，冷却为每秒热量，锁定时长使用仿真秒。

每回合模拟器保留最近 1024 条事件，IPC 每帧携带最近 64 条，按递增 ID 排列。
事件 ID 在进程内跨回合递增；每条携带回合、回合内仿真纳秒和含原因/参与者的 `detail`。
描述为最多 231 字节 UTF-8，按字符边界截断并以 NUL 结束。
事件是结构内的诊断描述，接收端无需解析控制日志；请求、实际出膛、装甲接触、扣血、
战亡与锁定各自记录。`Fired` 含弹丸 ID，拒绝不计入实际发射。

客户端按事件 ID 去重，不可每帧累加重复事件；ID 跳号表示可能遗漏。
`events_dropped` 只统计模拟器 1024 条缓存的淘汰数，不能代表传输完整性。
慢客户端也可能错过 64 条传输窗口，精确累计值应使用机器人的当前计数；
需要更长事件明细可临时启用 `combat.event_details`，不自行长期录制。

| v7 ABI 项 | 字节数／偏移 |
| --- | ---: |
| magic / version | 0x54414C07 / 7 |
| RobotCombatMeta / CombatEventMeta | 128 / 256 |
| CombatFrameMeta | 18624 |
| CapturedFrameMeta.combat 偏移 | 6144 |
| CapturedFrameMeta / FrameTripleBuffer | 24768 / 74368 |
| GimbalCmd / GimbalTripleBuffer | 64 / 256 |
| ShmMetaRegion.gimbal_cmd / runtime_state 偏移 | 74432 / 74688 |
| ShmMetaRegion | 74752 |

命令 `source_round_id`、`source_frame_sequence`、`source_capture_timestamp_ns`
偏移依次为 24、32、40。运行状态区仍为 64 字节。

## 可重复验收

```bash
./build-openvino/bin/mv-talos-protocol-test
# 不启动相机或占用运行中的 IPC：独占临时映射验证 v6 明确拒绝、v7 解码及来源标识。

# 运行已有闭环后，使用只读订阅检查两个通道，默认 15 秒（不录 MCAP）：
# 需要 Node.js 和 ws；系统已装 node-ws 时不需要以下临时安装。
npm install --prefix /tmp/talos-protocol-check --ignore-scripts --no-audit --no-fund ws@8.18.3
NODE_PATH=/tmp/talos-protocol-check/node_modules node scripts/check_talos_protocol.mjs 15
```

工具检查实际元数据大小和版本、两套 Schema、同采样序号内容一致、10 Hz 仿真采样率、
有限/无限余额编码、回合内累计值单调、事件 ID 内容一致和回合时间范围。
它通过 Foxglove 读取已解码的数据，不争用 IPC 图像三缓冲消费槽。

闭环场景：有限额度 3，F5 自动出膛至额度归零，R 后手动验证重新获得 3 发；
三个规则预设分别使用无限额度观察热锁、伤害、战亡和重置后的新回合统计。

## 本次实际检查（2026-09-06）

- Rust 格式检查、Talos release 构建、Clippy 完成；Clippy 保留 26 项既有警告。
  28 项 combat 测试通过，另有来源回合拒绝测试通过。全量为 83 通过、3 项既有失败：
  `debug_sequence_and_indexes_keep_legacy_order`、`sticker_slot_tables_keep_asset_suffixes`、
  `robot_configs_preserve_legacy_armor_values`，均为旧装甲标签断言。
- C++ OpenVINO 全量构建、clang-format、全部受影响源文件 clang-tidy 完成，
  本次暴露的项目命名及无效 move 诊断已修正。clangd 14 的全功能 `--check` 在部分
  文件的重构功能自检中报错/崩溃；改用 `--check-lines=0` 完成全文件解析、语法和头文件
  诊断，全部 0 errors，未修改仓库忽略配置。
- `mv-talos-protocol-test` 与 `mv-runtime-policy-test` 通过。
  协议测试使用独占临时映射，覆盖完整/较小 v6 文件拒绝、v7 图像解码、命令来源回合/
  帧号/时间和旧回合裁判样本拒绝；运行策略验收的故障日志为主动注入。
- 使用原启动脚本完成有限额度及三个无限额度预设，实时订阅 Foxglove 核验。
  有限额度 3：Talos 实际出膛 3 发，R 后手动实际出膛 3 发，余额归零；
  请求拒绝不增加发射或热量。自身裁判通道没有敌方状态和伤害归因。

| 无限额度预设 | 实际发射 | 有效伤害 HP | 击毁 | 热量锁定次数 |
| --- | ---: | ---: | ---: | ---: |
| 步兵血量/冷却 | 40 | 700 | 2 | 3 |
| 步兵血量/爆发 | 39 | 700 | 2 | 1 |
| 哨兵 | 41 | 700 | 2 | 1 |

三个预设分别收到每个战斗话题 800 帧；裁判样本约 10 Hz，高帧率重复样本内容一致。
R 后下一回合的发射、拒绝、伤害、击毁和热锁计数归零，没有残留事件或自动出膛。
这些结果是各次短时运行的观察值，不是算法性能指标或可复现训练种子的承诺。
本次验收临时关闭录制，结束恢复原配置；没有新增 MCAP。

Foxglove 使用当前 SDK 的 `foxglove.sdk.v1` 子协议；验收客户端同时声明该协议与旧的
`foxglove.websocket.v1`。已核验实时消息和 Schema，未做 Foxglove 桌面端人工面板布局验收。
ROS2 战斗遥测、完整仿真步进、强化学习训练及长时间性能测试不在本次交付范围。

最后一次有限额度复验中，两个战斗话题各收到 320 帧，320 对同帧序号、回合、
仿真时间与采集时间全部匹配；重置前后仍各实际出膛 3 发。
