# 实机 IMU 与 PnP 第一阶段

本轮只验收上位机编译。下位机由用户在 Keil 中编译、手动烧录和实测。
没有新增测试程序，不运行仿真，不自动启用录制，也不发送实机瞄准/开火命令。
协议及坐标约定见 [姿态协议](../hal/attitude_protocol.md)。

## 配置与启动

1. 烧录云台工程，确认 USART1 与 USB 转串口相连，参数为 460800、8N1。
2. 在 `src/config/hal/imu/serial.yaml` 中设置实际 `device`，推荐稳定的 `/dev/serial/by-id/...`。
   空路径或 `enabled: false` 保留图像检测，不提供空间数据。不要同时启动其他串口读取程序。
3. `hal/camera/intrinsics.yaml` 已复制 20260919_214820 标定值，要求 1280×720、ROI `(0,152)`。
4. `hal/imu/extrinsics.yaml` 明确为占位值。IMU 单位旋转尚未验证，不能据此评价定位精度。
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
传输丢包、host-minus-MCU 偏差、同步 RTT 和三轴角速度。窗口和 Web 预览显示状态摘要。
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
完成 IMU 轴向确认及外参测量后，才能验证转动云台时固定目标的方向稳定性。
相机绕双 yaw 的位置变化本轮没有补偿，不能要求目标惯性系位置完全不动。


## Foxglove 实机坐标轴

新主程序在 `/vision/camera/transforms` 发布同帧实机 TF，3D 面板以 `world` 为参考，
启用 Transforms 坐标轴与 `/vision/camera/frustum` 视锥。
具体设置及占位外参限制见 [Foxglove 实机 3D 坐标系](../tool/FOXGLOVE.md#实机-3d-坐标系)。
本轮仅验证编译通过；用户实机确认 TF 持续更新，云台转动时相机视锥同步转动。


## 源码位置

串口设备、分帧、握手、同步和重连位于 `src/hal/serial/controller_link.cpp`，
共用协议头位于 `src/hal/serial/attitude_wire.h`。
IMU 姿态解析及插值位于 `src/hal/imu/serial_imu.cpp`；现有配置路径和启动命令不变。
