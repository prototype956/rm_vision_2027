# EX_MiracleVision

北京林业大学 RoboMaster 视觉系统。

项目当前处于架构精简和算法模块重设计阶段。现阶段包含相机 HAL、配置、日志、
MindVision 实机验收程序，以及基于 OpenVINO 的深圳大学 RobotDetectionModel
0526 装甲板检测模块和 Foxglove 调试输出。

## 当前可执行程序

- `mv-vision-main`：使用 MindVision 相机同步执行 YOLO 0526 检测并显示叠加结果。
- `mv-camera-test`：MindVision 长时间稳定性、重复启停和拔线验收程序。
- `mv-armor-detector-test`：MindVision、GPU 检测器与可选 Foxglove 输出的长时实机
  验收程序。
- `mv-armor-detector-video-test`：离线视频检测、可视化、逐帧耗时和性能验收程序。

模型权重是本地文件，不提交到 Git，由使用者手工管理。来源、固定提交和放置方法见
[`src/modules/armor_detector/models/README.md`](src/modules/armor_detector/models/README.md)。

## 环境要求

- Linux（推荐 Ubuntu 22.04）
- GCC 11 或兼容的 C++20 编译器
- CMake 3.15+
- OpenCV
- fmt
- spdlog
- yaml-cpp
- OpenVINO Runtime 2024.0（构建主程序和检测模块时需要）
- 仓库内 `3rdparty/mindvision` SDK（实机运行需要）
- 仓库内 `3rdparty/foxglove` SDK（预编译 x86-64 Linux 库与 C++ 封装）

Ubuntu 系统依赖示例：

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake \
    libopencv-dev libfmt-dev libspdlog-dev libyaml-cpp-dev
```

## 构建

```bash
source /opt/intel/openvino_2024.0.0/setupvars.sh
cmake -S /home/nuc/Workspace/rm_vision_2027 \
  -B /home/nuc/Workspace/rm_vision_2027/build-openvino \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_OPENVINO=ON
cmake --build /home/nuc/Workspace/rm_vision_2027/build-openvino --parallel 4
```

可用选项：

- `BUILD_MAIN=ON|OFF`：是否构建 `mv-vision-main`，默认开启。
- `USE_OPENVINO=ON|OFF`：是否构建检测模块，默认开启。`BUILD_MAIN=ON` 时不能关闭。
- `USE_MINDVISION_SDK=ON|OFF`：是否链接 MindVision SDK，默认开启。关闭后仍可
  编译 Camera HAL，但 MindVision `Open()` 会返回失败。

不依赖 OpenVINO 的纯相机构建：

```bash
cmake -S . -B build-camera \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MAIN=OFF \
  -DUSE_OPENVINO=OFF
cmake --build build-camera --parallel 4
```

## 运行

```bash
./build-openvino/bin/mv-vision-main
./build-openvino/bin/mv-camera-test
./build-openvino/bin/mv-armor-detector-test
./build-openvino/bin/mv-armor-detector-video-test
```

当前登录会话必须具有 `render`、`video` 组，且 OpenVINO 必须能看到 `GPU` 或
`GPU.<index>`；检测器不会回退 CPU，也不应使用 `sudo` 运行。相机预览按 `Q`、
`Esc` 或关闭窗口退出。完整实机验收流程见
[docs/test/CAMERA_TEST.md](docs/test/CAMERA_TEST.md)。

离线检测和性能验收见
[`docs/test/ARMOR_DETECTOR_TEST.md`](docs/test/ARMOR_DETECTOR_TEST.md)。
Foxglove 连接、话题与 MCAP 使用方法见
[`docs/tool/FOXGLOVE.md`](docs/tool/FOXGLOVE.md)。
检测模块的接口、模型协议、二维坐标系和后处理契约见
[`docs/modules/ARMOR_DETECTOR.md`](docs/modules/ARMOR_DETECTOR.md)。

## 配置

当前有效配置：

```text
src/config/
├── core/logger.yaml
├── hal/camera/
│   ├── mindvision.yaml
│   └── opencv.yaml
├── modules/
│   └── armor_detector.yaml
├── tool/
│   ├── debug_window.yaml
│   └── foxglove.yaml
└── test/
    ├── camera_test.yaml
    ├── armor_detector_test.yaml
    └── armor_detector_video_test.yaml
```

主程序固定读取日志、装甲检测器、MindVision 相机与调试工具配置。相机测试额外读取
`test/camera_test.yaml`；装甲检测实机测试额外读取 `test/armor_detector_test.yaml` 和
共享的 `tool/foxglove.yaml`；离线视频测试默认从
`test/armor_detector_video_test.yaml` 读取本地视频目录、文件名和窗口预览开关。
模型路径相对于项目根目录解析。

## 项目结构

```text
.
├── cmake/                 # 编译器、系统依赖和 MindVision SDK 配置
├── docs/
│   └── test/              # 硬件测试与验收文档
├── src/
│   ├── app/               # MindVision + YOLO 检测入口
│   ├── core/              # YAML 配置工具和日志
│   ├── hal/
│   │   ├── camera/        # ICamera 及按后端分组的 MindVision/OpenCV 驱动
│   │   └── serial/        # 空目录，等待重新设计
│   ├── modules/
│   │   └── armor_detector/# OpenVINO YOLO 0526 检测模块
│   └── tool/
│       ├── debug/         # OpenCV 调试窗口与装甲可视化
│       └── foxglove/      # WebSocket、MCAP 与异步装甲调试发布
├── test/
│   ├── camera/            # MindVision 实机验收程序
│   └── armor_detector/    # 装甲检测实机与离线视频验收程序
└── 3rdparty/              # MindVision 与 Foxglove vendored SDK
```

## 当前边界

- 首期只支持 `0526.onnx`；`0708.onnx` 不在兼容和测试范围内。
- 检测器为同步、非线程安全实现，明确使用配置的 `GPU` 或 `GPU.<index>`，不接受
  `AUTO`、`MULTI` 或 CPU 回退。
- 暂不包含标注精度评估。
- 没有串口或通信实现。
- 没有 PnP、目标选择、连续帧锁定、预测或异步推理。
- 不做相机后端自动切换或故障自动恢复。

## License

本项目代码采用 [Apache License 2.0](LICENSE)。本地模型权重不受本项目许可证
授权，使用和分发前需单独确认上游权利。
