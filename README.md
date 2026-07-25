# EX_MiracleVision

北京林业大学 RoboMaster 视觉系统。

项目当前处于架构精简和算法模块重设计阶段。现阶段保留相机 HAL、配置、日志，
以及 MindVision 实机预览和验收程序；算法和串口通信实现暂时为空。

## 当前可执行程序

- `mv-vision-main`：固定使用 MindVision 相机，显示 1280×720 BGR 原始画面。
- `mv-camera-test`：MindVision 长时间稳定性、重复启停和拔线验收程序。

当前所有可执行程序都直接使用 Camera HAL，尚未接入算法模块。

## 环境要求

- Linux（推荐 Ubuntu 22.04）
- GCC 11 或兼容的 C++20 编译器
- CMake 3.15+
- OpenCV
- fmt
- spdlog
- yaml-cpp
- 仓库内 `3rdparty/mindvision` SDK（实机运行需要）

Ubuntu 系统依赖示例：

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake \
    libopencv-dev libfmt-dev libspdlog-dev libyaml-cpp-dev
```

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 4
```

可用选项：

- `BUILD_MAIN=ON|OFF`：是否构建 `mv-vision-main`，默认开启。
- `USE_MINDVISION_SDK=ON|OFF`：是否链接 MindVision SDK，默认开启。关闭后仍可
  编译 Camera HAL，但 MindVision `Open()` 会返回失败。

## 运行

```bash
sudo ./build/bin/mv-vision-main
sudo ./build/bin/mv-camera-test
```

相机预览按 `Q`、`Esc` 或关闭窗口退出。完整实机验收流程见
[docs/CAMERA_TEST.md](docs/CAMERA_TEST.md)。

## 配置

当前有效配置：

```text
src/config/
├── apps/camera_test.yaml
├── core/logger.yaml
└── hal/camera/
    ├── mindvision.yaml
    └── opencv.yaml
```

主程序固定读取 `core/logger.yaml` 和 `hal/camera/mindvision.yaml`。相机测试额外
读取 `apps/camera_test.yaml` 中的测试参数。

## 项目结构

```text
.
├── cmake/                 # 编译器、系统依赖和 MindVision SDK 配置
├── docs/                  # 当前有效文档
├── src/
│   ├── app/               # MindVision 预览入口
│   ├── core/              # YAML 配置工具和日志
│   ├── hal/
│   │   ├── camera/        # ICamera、MindVisionCamera、OpenCvCamera
│   │   └── serial/        # 空目录，等待重新设计
│   └── modules/           # 空算法目录，等待重新设计
├── test/                  # MindVision 实机验收程序
└── 3rdparty/mindvision/   # MindVision SDK
```

## 当前边界

- 没有算法实现。
- 没有串口或通信实现。
- 没有 OpenVINO 或 ONNX Runtime 构建接入。
- 不做相机后端自动切换或故障自动恢复。

## License

本项目采用 [MIT License](LICENSE)。
