# 仿真外参采集

`mv-extrinsic-calibration` 从 Talos 共享内存读取仿真器的原始相机图像，并将图像与同一
`FramePacket` 中的相机内参、帧序号、采集时间戳、`world_t_gimbal` 和
`gimbal_t_camera_optical` 一起保存。预览文字不会写入 PNG。

采集阶段不执行手眼解算；离线 `solve` 使用 `sp_vision_25` 中的
`calibrateHandEye` 流程，不需要预先提供棋盘的 world 位姿。求解使用图像、内参和
`world_t_gimbal`；`gimbal_t_camera_optical` 只用于解算后的真值比较，不能作为待估外参输入。

当前采集程序已经提供对应的离线解算入口。它复刻固定棋盘的 Tsai 手眼标定流程，使用
`findChessboardCornersSB` 检测当前仿真器中的黑白棋盘格，并逐张显示检测结果，按任意键
确认当前图像：

```bash
./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics solve \
  artifacts/extrinsic_calibration/<timestamp>
```

`Q`、`Esc`、关闭窗口或 `Ctrl+C` 会取消解算。检测失败的图像会记录原因并跳过；有效样本
不足 3 张或 OpenCV 手眼求解失败时不会生成外参结果。

## 构建

```bash
cmake -S . -B build-openvino -DCMAKE_BUILD_TYPE=Release -DUSE_OPENVINO=ON
cmake --build build-openvino --parallel 4 --target mv-extrinsic-calibration
```

工具也可以在 `BUILD_MAIN=OFF`、未启用检测器的相机版本中构建。

## 启动

推荐使用统一标定入口：

```bash
./scripts/calibrate_camera.sh
```

进入主项 `3`「仿真相机—云台外参」，子项 `1` 采集、`2` 解算、`3` Foxglove 三维验收。
会话列表读取配置中的 `output_dir`，按修改时间倒序显示；验收仅列出具有成功报告和
外参文件的结果。入口与数据类型筛选详见 [统一标定与 IMU 入口](CALIBRATION.md)。
所有模式支持 `RM_VISION_CONFIG_ROOT`，未设置或为空使用默认配置；相对配置/数据路径
按项目根目录解释。离线解算仍按现有规则用配置核对会话棋盘参数。

也可以使用下面的显式命令，适合自动化或不使用菜单的场景。

统一入口会启动仿真器和采集程序，把 `VISION_BIN` 设置为
`mv-extrinsic-calibration`，并跳过只针对主视觉程序的 `main.yaml` backend 检查；Talos
共享内存、动态库和仿真器就绪检查仍然执行，因此不需要临时修改配置文件。
采集程序读取 `hal/camera/talos.yaml` 和 `tool/extrinsic_calibration.yaml`：

```bash
./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics capture
```

仿真器默认自动选择仓库同级的 `bevy_robomaster_simulator`，也可以显式指定路径：

```bash
SIMULATOR_ROOT="$(pwd)/../bevy_robomaster_simulator" \
  ./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics capture
```

## 采集操作

- `Space` 或 `S`：保存当前原始图像和同帧真值；
- `U`：将最近保存的样本移到 `excluded/` 并标记为 inactive；
- `Q` / `Esc`：正常结束，状态为 `captured`；
- 关闭窗口、Ctrl+C、Talos 断开、无效帧或写盘失败：中止，状态为 `aborted` 并保存原因。

按键需作用于采集预览窗口；仿真器中的 Space 是开火。Talos 帧三缓冲为单消费者，
采集时不要同时运行视觉主程序或其他 Talos 相机工具。F2 仅为仿真窗口截图，不能代替本工具。

为了后续手眼解算，使用机器人第一人称视角，固定相机安装关系及同一块棋盘；不要混用自由
相机、第三人称或两块棋盘。采集不同 yaw/pitch 的静止姿态，不能只有单轴旋转。按 R 重置
后重新创建会话；工具检测到回合变化会中止，避免混用世界基准。参考项目将云台平移置零，
直接复用该假设时不能移动云台原点；本工具仍保存完整 `world_t_gimbal` 供后续解算使用。

每次运行会创建：

```text
artifacts/extrinsic_calibration/<timestamp>/
  images/sample_0001.png
  excluded/
  session.yaml
```

解算不会修改原始采集目录。每次解算会在其下创建独立结果目录：

```text
solutions/<timestamp>/
  extrinsics.yaml
  report.yaml
```

`extrinsics.yaml` 保存估计的 `gimbal_t_camera_optical`，平移单位为米、四元数顺序为
`xyzw`，并同时保存行优先旋转矩阵。`report.yaml` 保存每帧角点检测状态、PnP 重投影误差、
估计外参、仿真器同帧真值以及旋转角误差和平移误差统计。误差报告不设置固定通过阈值，
也不会自动写入 `src/config/hal/imu/extrinsics.yaml`。

## Foxglove 离线三维验收

可以直接回放某次解算结果，不需要启动 Talos。查看器从 `extrinsics.yaml` 读取估计外参，
从 `report.yaml` 读取每帧 `world_t_gimbal` 和仿真真值外参，并发布两个独立话题：

```bash
./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics visualize \
  artifacts/extrinsic_calibration/20261006_160142/solutions/20261006_172133
```

启动 Foxglove 后连接 `ws://127.0.0.1:8765`，在 3D 面板中选择固定坐标系 `world`，添加：

离线查看器需要独占 Foxglove 配置中的监听端口；如果视觉主程序已经占用 `8765`，先停止
主程序，或在 `src/config/tool/foxglove.yaml` 中临时改用其他未占用端口。

- `/calibration/extrinsic/transforms`：`world`、真值/估计云台和相机的 TF；
- `/calibration/extrinsic/scene`：世界、云台、真值相机、估计相机的彩色坐标轴，以及相机原点误差线。

估计和真值使用 `gimbal_estimated`、`camera_optical_estimated` 与
`gimbal_truth`、`camera_optical_truth`，不会在 Foxglove 中互相覆盖。查看器默认每 750 ms
循环回放一个 accepted 样本，按 `Ctrl+C` 退出；增加 `--once` 只发布一轮。场景标签同时显示
当前帧误差和报告中的平均误差。该可视化用于发现坐标系方向、安装位置和帧间异常，数值验收仍
以 `report.yaml` 中的旋转和平移统计为准。

`session.yaml` 中的平移单位为米，四元数顺序为 `xyzw`，变换均表示子坐标系到父坐标系。
每条记录含 `id`、`active`、相对 `image` 路径、源 `frame_sequence`、Unix epoch 纳秒
`capture_timestamp_ns`、`simulation_round_id`、同帧 `camera` 内参与五项畸变、两组变换。
内参键为 `width/height/fx/fy/cx/cy/distortion`，变换键为 `translation/rotation_xyzw`。
每次保存和排除都会原子替换清单；失败不删除上一份清单。编号不复用，排除的记录和图像均
保留，后续解算以清单中的 `active: true` 为准。进程被强制杀死时可能留下未登记 PNG，
不得仅扫描 images 目录作为完整样本集。磁盘本身不可写时无法保证落盘 aborted 状态，需
查看终端报错。

默认棋盘为 10×7 内角点，15 mm 仅是 CALIB.glb 纹理标注值；实际解算前应复核 UV 映射和
模型缩放后的物理间距。采集阶段不检测棋盘和求解外参；离线 `solve` 阶段执行角点检测和
手眼求解。

采集配置中的 `solve.min_samples` 默认值为 3，仅用于拒绝无法进行手眼求解的样本过少会话；
它不是外参误差验收阈值。

退出码：0 正常结束，1 命令行/配置/初始化写盘异常，2 Talos 连接失败，3 输出规格不符，
4 采集异常或窗口关闭，5 解算输入校验失败、有效观测不足或 OpenCV 求解失败，6 信号中止。
没有发布端时默认等待 5 秒后返回 2。
