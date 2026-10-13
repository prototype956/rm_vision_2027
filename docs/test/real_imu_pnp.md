# 实机 IMU 与 PnP 第一阶段

本文覆盖实机姿态链路、安装配置和 PnP 验收。下位机由用户在 Keil 中编译、手动烧录。
编译与配置检查不能代替实机验收；主程序不自动启用录制，也不发送实机瞄准/开火命令。
协议及坐标约定见 [姿态协议](../hal/attitude_protocol.md)。

## 配置与启动

1. 烧录云台工程，确认 USART1 与 USB 转串口相连，参数为 460800、8N1。
2. 在 `src/config/hal/imu/serial.yaml` 中设置实际 `device`，推荐稳定的 `/dev/serial/by-id/...`。
   空路径或 `enabled: false` 保留图像检测，不提供空间数据。不要同时启动其他串口读取程序。
3. `hal/camera/intrinsics.yaml` 已复制最新 `20260926_161918` 标定值，要求 1280×720、ROI `(0,152)`。
4. `hal/imu/extrinsics.yaml` 中 IMU 名义安装旋转已确认为 `diag(-1,-1,1)`。
   相机外参已由 2026-10-10 手眼标定结果替换；枪口外参仍未标定，因此 `placeholder: true` 保留。
5. `hal/camera/mindvision.yaml` 的 `capture.time_offset_ms` 默认 0，是加到估计采集时刻的修正，
   正数表示更晚、负数更早。SDK `uiTimeStamp` 单位为 0.1 ms，初始 8 帧后建立软件映射。
   映射取最近 10 秒（最多 2048 帧）的最小接收延迟，尚未补偿 USB 最小传输延迟或曝光相位。
   不自动假定时间戳等于曝光中点。

```bash
cmake --build build-openvino --parallel 4
./build-openvino/bin/mv-vision-main
```

使用当前已配置的 OpenVINO 构建环境。设备访问由用户配置权限；GPU 检测按仓库说明使用普通用户。

## 用户实机观察

日志 `Geometry` 每秒显示：原因、同步状态、接收 Hz、最新样本年龄、外参占位、CRC 错误、
传输丢包、host-minus-MCU 偏差、同步 RTT 和三轴角速度。
`gyro_imu_rad_s` 为原始 IMU 系，`gyro_gimbal_rad_s` 为前、左、上云台系；
当前安装应满足云台角速度 `[-wx, -wy, wz]`。状态摘要还包含所选 profile。窗口和 Web 预览显示状态摘要。
角速度仅作为诊断值，不做积分或外推。日志记录受现有 logger 配置控制。

- 正常连接后出现 `sync=true`、接近 200 Hz，空间状态为 `ok`。
- `serial_device_not_configured` / `serial_open_failed`：检查路径、占用和访问权限。
- `camera_clock_not_ready`：相机时钟初始收敛或重置。
- `intrinsics_roi_mismatch`：实际尺寸或 ROI 与标定不一致，禁止静默缩放内参。
- `image_outside_imu_history`：图像时刻不在缓存覆盖范围内，检查两端时间和相机时间修正。
- `imu_sample_gap`：用于插值的两个样本间隔超过 20 ms；不外推。
- `imu_not_ready` / `invalid_attitude`：下位机尚未有效或姿态非法。
- `clock_generation_changed`：新会话/相机时间回退，当前帧主动跳过空间解算以清空预测历史。

断连、MCU 重启和无效数据时，应保留图像检测并失去空间结果，恢复后重新建轨。
同端口 PC 调参仍由同一个串口拥有者发送；不要让两个进程竞争读取串口。
当前车辆的 IMU 轴向和相机外参已经确认，可用固定目标检查小 yaw/pitch 转动时的世界系稳定性。
大 yaw/底盘平移没有进入本阶段模型；涉及这些自由度时不能要求目标惯性系位置完全不动。


## Foxglove 实机坐标轴

新主程序在 `/vision/camera/transforms` 发布同帧实机 TF，3D 面板以 `world` 为参考，
启用 Transforms 坐标轴与 `/vision/camera/frustum` 视锥。
具体设置及占位外参限制见 [Foxglove 实机 3D 坐标系](../tool/FOXGLOVE.md#实机-3d-坐标系)。
2026-10-10 用户实机确认 TF 持续更新，云台转动时相机视锥同步转动；固定目标在小 yaw/pitch
转动时保持在同一惯性位置且无明显抖动。

## 2026-10-10 当前车辆验收记录

- 下位机由用户完成 Keil 编译与手动烧录；主程序和标定工具在本仓库重新构建。
- 姿态接收约 200 Hz，`IMU:ok`、`sync=true`；观测到样本年龄约 4–11 ms、同步 RTT 约 2.0–2.6 ms、CRC 错误 0、传输丢帧 0。
- 外参采集会话：`artifacts/camera_calibration/extrinsics/20261010_154217`；求解：`solutions/20261010_154858`。
- 30/30 样本接纳；PARK 求解的 PnP 重投影 RMS 为 0.109290 px、固定板位置一致性 RMS 为 0.012819 m、旋转一致性 RMS 为 0.497539°。
- 最大位置残差为 0.026761 m，最大旋转残差为 1.104592°，平移约束条件数为 2.249189。
- 已部署候选中的 `gimbal_r_camera` 与 `gimbal_t_camera_m`；部署前配置保存在求解目录的 `deployment_before.yaml`。
- 验收记录：`logs/2026-10-10_16-08-27.log`；MCAP：`artifacts/foxglove/miraclevision_20261010_160830_59419.mcap`，程序正常停止并完成文件封口。

本记录表示相机—云台外参和第一阶段姿态/PnP 链路通过当前实机检查；不表示枪口外参、弹道、
底盘平移或导航世界坐标已完成。


## 源码位置

串口设备、分帧、握手、同步和重连位于 `src/hal/serial/controller_link.cpp`，
共用协议头位于 `src/hal/serial/attitude_wire.h`。
IMU 姿态解析及插值位于 `src/hal/imu/serial_imu.cpp`；
部署选择和安装矩阵校验位于 `src/runtime/real_geometry_config.cpp`。


## 车辆部署选择

`src/config/runtime/real_geometry.yaml` 的 `active_profile` 默认是 `current_vehicle`。
每个 profile 指定 `imu_config`、`camera_intrinsics`、`extrinsics` 三个文件，路径相对
配置根目录，而不是相对 YAML 所在目录。安装旋转按行排列，方向均为 child -> gimbal。
只读取选中的 profile；未知 profile、缺失字段、非法矩阵会报错，不静默选择其他车辆。

例如为另一台步兵新增配置时，复制并填写它自己的串口、内参和完整外参文件，再增加：

```yaml
schema_version: 1
active_profile: infantry_2
profiles:
  current_vehicle:
    imu_config: hal/imu/serial.yaml
    camera_intrinsics: hal/camera/intrinsics.yaml
    extrinsics: hal/imu/extrinsics.yaml
  infantry_2:
    imu_config: vehicles/infantry_2/serial.yaml
    camera_intrinsics: vehicles/infantry_2/intrinsics.yaml
    extrinsics: vehicles/infantry_2/extrinsics.yaml
```

示例中的 `vehicles/infantry_2/` 文件需部署者准备；不是当前车辆的已标定模板。
同一兵种的不同安装也可以选择不同 profile，不在算法中按兵种分支。
profile 不选择相机驱动；设备后端、分辨率和 ROI 仍由 `app/main.yaml` 及相机配置设置，
必须匹配所选内参。需要整套配置隔离时使用既有入口：

```bash
RM_VISION_CONFIG_ROOT=/path/to/vehicle/config ./build-openvino/bin/mv-vision-main
```

部署目录应包含完整配置树以及新增的 `runtime/real_geometry.yaml`。启动日志打印实际
profile、内外参路径和 IMU 安装四元数；更换 profile 后重启程序，不支持运行中热切换。
Talos 分支使用仿真提供的同帧几何，不读取此实机 profile。

当前固定的 `diag(-1,-1,1)` 来自实验 1790080270511235 与 1790080827626598。
相机外参已完成标定和实机检查；枪口仍未标定，因此不能清除整体占位标记。
配置回归后需用户实机确认：水平左转时 `gyro_gimbal_rad_s.z > 0`，抬头时
`gyro_gimbal_rad_s.y < 0`，Foxglove 云台及相机坐标轴随动作正确变化。


## 相机外参工具

现在可在固定大 yaw 的前提下，使用现有标定工具的 `extrinsics-capture` /
`extrinsics-solve` 模式标定相机到小 yaw/pitch 云台的安装变换。
采集与人工审核步骤、结果含义见 [单 yaw 外参标定](../tool/CAMERA_CALIBRATION.md#单-yaw-相机外参标定)。
工具生成的候选文件不自动应用。当前车辆的 2026-10-10 候选已人工部署并完成实机验收；
其他车辆仍须单独完成审核和部署，所有未标定外参继续保留占位标记。


标定及 IMU 工具现统一由 `scripts/calibrate_camera.sh` 分组菜单启动；实机手眼在主项 `2`，
IMU 轴向/漂移在主项 `4`。完整入口与配置说明见
[标定与 IMU 工具](../tool/CALIBRATION.md)。
