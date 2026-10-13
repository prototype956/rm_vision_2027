# 标定与 IMU 工具入口

从任意目录运行 `scripts/calibrate_camera.sh`（脚本路径按实际位置填写），无参数进入菜单：

```text
1. 相机内参
   1. 采集
   2. 解算已有会话
2. 实机相机—云台外参
   1. 采集
   2. 审核并解算已有会话
3. 仿真相机—云台外参
   1. 采集
   2. 解算已有会话
   3. Foxglove 三维验收
4. IMU 检查
   1. 安装轴向采集
   2. 静止漂移采集
   3. 漂移数据离线分析
```

`q` 返回上一级；任务退出后留在所属子菜单。菜单仅组织入口，分别调用独立程序，
不会修改部署参数。菜单需要 Python 3 和 PyYAML（Ubuntu 包 `python3-yaml`）。
非交互终端使用下面的显式命令；`CALIBRATION_MENU=1` 可用于脚本菜单验收。

## 程序与命令

| 功能 | 独立程序 | 数据默认目录 |
|---|---|---|
| 内参 | `mv-camera-calibration capture/solve` | `artifacts/camera_calibration/` |
| 实机手眼 | `mv-camera-calibration extrinsics-capture/extrinsics-solve` | 上述目录的 `extrinsics/` |
| 仿真外参 | `mv-extrinsic-calibration capture/solve/visualize` | `artifacts/extrinsic_calibration/` |
| IMU 轴向 | `mv-imu-axis-capture` | `artifacts/imu_axis/` |
| IMU 漂移 | `mv-imu-drift capture/analyze` | `artifacts/imu_drift/` |

统一入口的非交互形式为：

```text
./scripts/calibrate_camera.sh [--build-dir DIR] <功能组> <动作> [参数]

intrinsic             capture | solve SESSION
real-extrinsics       capture | solve SESSION
simulation-extrinsics capture | solve SESSION | visualize SOLUTION [--once]
imu-axis              capture [DEVICE [NEW_OUTPUT_DIRECTORY]]
imu-drift             capture [采集选项] | analyze SESSION
```

示例：

```bash
./scripts/calibrate_camera.sh --build-dir build-openvino intrinsic capture
./scripts/calibrate_camera.sh --build-dir build-openvino intrinsic solve artifacts/camera_calibration/<会话>
./scripts/calibrate_camera.sh --build-dir build-openvino real-extrinsics capture
./scripts/calibrate_camera.sh --build-dir build-openvino real-extrinsics solve artifacts/camera_calibration/extrinsics/<会话>
./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics capture
./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics solve artifacts/extrinsic_calibration/<会话>
./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics visualize artifacts/extrinsic_calibration/<会话>/solutions/<结果>
./scripts/calibrate_camera.sh imu-axis capture
./scripts/calibrate_camera.sh imu-axis capture /dev/ttyUSB0 artifacts/imu_axis/<新目录>
./scripts/calibrate_camera.sh imu-drift capture --duration 900 --thermal-state cold
./scripts/calibrate_camera.sh imu-drift analyze artifacts/imu_drift/<会话>
```

`--help` 或 `<功能组> --help` 仅显示帮助，不构建、不采集。
漂移采集支持 `--device`、`--duration`、`--note`、`--motor-state` 和 `--thermal-state`，
默认 900 秒，状态为 `unspecified`。轴向第二个可选参数为新输出目录，设备留空可沿用 profile。

无参数进入菜单；`--build-dir DIR` 也可用于指定菜单使用的构建目录。
非交互终端必须明确提供功能组和动作。旧的“构建目录 + 动作”、`simulation` 等参数形式
已移除，两个独立 IMU 启动脚本也已移除；原有命令应改为上述分组形式。
内部 C++ 工具及其 CMake 目标保留，由统一入口调用。

`--build-dir` 优先于环境变量，指定目录缺少对应程序时直接报错，不回退。
未指定时按 `BUILD_DIR`、仿真工具的 `VISION_BUILD_DIR` 和可用构建目录搜索。
内参、实机外参和 IMU 默认搜索顺序为 `build-camera`、`build-openvino`、`build-tensorrt`；
仿真外参为 `build-openvino`、`build-tensorrt`、`build-camera`。
未找到 IMU 程序时，统一入口独立编译到 `build-imu-axis` 或 `build-imu-drift`；
编译失败不会启动工具。独立构建需要 C++20、Eigen3、yaml-cpp、spdlog、fmt 和 pthread，
不需要 OpenCV、相机 SDK 或 OpenVINO。相机/仿真程序仍需按各自文档提前构建。

仿真 `capture` 自动启动 Talos，并将 `capture` 转发给采集程序；
`solve`、`visualize` 不启动仿真器。非交互命令保留工具输出及退出码；
菜单报告状态码并返回所属子菜单。会话筛选由 `scripts/internal/calibration_menu_support.py`
提供，属于只读内部辅助实现，不是公开操作入口。

## 配置和会话选择

所有入口支持 `RM_VISION_CONFIG_ROOT`；未设置或为空使用项目默认配置。
脚本和上述独立程序的相对配置、数据路径按项目根目录解释。
轴向和漂移默认读取 `runtime/real_geometry.yaml` 的 `active_profile`，
使用该 profile 的串口参数。启动输出 profile、设备、波特率及输出目录。
轴向的 `DEVICE`、漂移的 `--device` 仅覆盖本次设备；不写回配置。
空设备或 IMU 被禁用时采集报错，不创建有效实验。

内参/外参菜单从配置的 `output_dir` 发现会话，IMU 漂移目录沿用上述默认路径。
列表按目录修改时间倒序显示，过滤尚未结束、损坏、类型不符或缺少必要文件的会话，
并显示跳过原因。实机手眼识别 `kind: single_yaw_handeye`，漂移识别
`kind: imu_static_drift`；历史内参和仿真会话按原有清单结构识别，无需迁移。
Foxglove 菜单只提供具有 `extrinsics.yaml` 和成功 `report.yaml` 的仿真结果。
实机手眼与漂移离线处理使用采集快照；仿真离线处理仍使用现有配置检查棋盘参数。

## IMU 诊断与手眼门槛

轴向和漂移同时接收普通 type 5 与标定 type 6。type 6 要求 payload v2、长度正确、
保留字段为零、扩展浮点数及方向字段合法，姿态 READY/FINITE、四元数和角速度有效。
所有采集仍要求时间同步与样本顺序正确。

IMU 诊断不要求陀螺标定完成、关节在线或大 yaw 禁用，机构状态仅用于实验解释。
运行时 `SerialImu` 和实机手眼保持原有严格门槛；手眼还强制使用 type 6。
诊断工具只发送 HELLO/SYNC，不发送 type 7，不控制云台、不生成补偿参数。

两个 IMU 工具保留原来的 `samples.csv`，增加 `protocol_diagnostics.csv`：
记录会话代次、主机接收时间、MCU 时间、传输/样本序号、消息类型、payload 版本、
陀螺标定状态、机构/pitch 状态位和接纳/拒收原因。type 5 无扩展字段的列留空。
长度错误无法解码时 MCU/样本字段留空；CRC 错帧不生成记录。
`session.yaml` 汇总这些状态计数，漂移的离线报告也复制该摘要。
历史漂移会话没有这些新字段仍可分析。

轴向会话新增 `kind: imu_axis_capture` 的 `session.yaml`，冻结有效串口配置和安装外参，
保留原有阶段标签及 `README.txt`。轴向原始值不应用安装旋转；漂移分析使用快照旋转。

## 详细操作

- [相机内参及实机手眼](CAMERA_CALIBRATION.md)
- [仿真外参与 Foxglove 验收](EXTRINSIC_CALIBRATION.md)
- [IMU 轴向约定与串口协议](../hal/attitude_protocol.md)
- [IMU 静止漂移](../test/imu_drift.md)
- [实机 IMU/PnP 验收](../test/real_imu_pnp.md)

标定、IMU 诊断与视觉主程序应顺序运行，避免竞争相机、串口或仿真共享内存。
工具生成候选与报告后仍由操作者审核，原始数据和旧结果不覆盖。
