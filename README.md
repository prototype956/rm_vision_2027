# EX_MiracleVision# EX_MiracleVision

bjfu rm 2026 vision system

**BJFU RoboMaster 2026 视觉系统**

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Ubuntu](https://img.shields.io/badge/ubuntu-22.04-orange.svg)]()
[![GCC](https://img.shields.io/badge/gcc-11.4.0-green.svg)]()

---

## 📖 目录

- [项目简介](#项目简介)
- [主要特性](#主要特性)
- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [项目结构](#项目结构)
- [文档导航](#文档导航)
- [贡献指南](#贡献指南)
- [许可证](#许可证)

---

## 📌 项目简介

EX_MiracleVision 是北京林业大学 RoboMaster 2026 赛季的自动瞄准视觉系统。该项目实现了完整的装甲板识别、追踪、预测和自动射击功能，支持多种传感器和算法模块。

### 核心功能
- 🎯 **装甲板检测**: 基于传统视觉和深度学习的双重检测方案
- 🔄 **目标追踪**: 卡尔曼滤波器实现稳定追踪
- 📐 **姿态解算**: PnP 算法精确计算目标位置和姿态
- 🎪 **能量机关识别**: 大小能量机关的检测和击打(未实现)
- 🚀 **弹道预测**: 考虑重力、空气阻力的弹道补偿
- 📡 **数据可视化**: Foxglove Studio 实时调试和分析
- 🔌 **串口通信**: 与下位机的高速数据交互

---

## ✨ 主要特性

### 架构设计
- **模块化设计**: 各功能模块独立，易于维护和扩展
- **高性能**: 基于 C++17 和现代 CMake 构建，优化的多线程处理
- **跨平台**: 支持 x86_64 架构的 Linux 系统
- **可配置**: YAML 配置文件管理所有参数

### 环境依赖
- **操作系统**: Ubuntu 22.04 LTS
- **语言**: C++17/20
- **构建系统**: CMake 3.22+
- **编译器**: GCC 11.4.0 或更高版本
- **计算机视觉**: OpenCV 4.5+
- **深度学习**: ONNX Runtime + OpenVino
- **通信协议**: Foxglove 
- **数学库**: Eigen3
- **线程管理**: TBB
- **日志系统**: spdlog, fmt

---

## 🖥️ 硬件配置
- **内存**: 16GB DDR4
- **硬盘**: PCIe4.0 NVMe 512G
- **相机**: MindVision 工业相机（推荐）
- **串口**: USB-TTL 
- **处理器**: i7-1260P/U5-125H

---

## 🚀 快速开始

### 1. 克隆项目

```bash
git clone https://github.com/prototype956/EX_MiracleVision.git
cd EX_MiracleVision
```

### 2. 安装依赖

**自动安装（推荐）**：
```bash
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

**手动安装**：
```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git \
    libopencv-dev libfmt-dev libspdlog-dev \
    libeigen3-dev libyaml-cpp-dev \
    nlohmann-json3-dev libtbb-dev
```

详细说明请查看 [环境配置文档](docs/ENVIRONMENT_SETUP.md)

### 3. 编译项目

```bash
# 创建构建目录
mkdir -p build && cd build

# 配置 CMake
cmake -DCMAKE_BUILD_TYPE=Release ..

# 编译（使用 4 个并行任务）
make -j4

# 可选：编译测试程序
cmake -DBUILD_TESTS=ON ..
make -j4
```

### 4. 运行程序

```bash
# 运行主程序
cd build
./bin/MiracleVision

# 运行测试程序
./bin/minimum_vision
```

---

## 📁 项目结构

```
EX_MiracleVision/
├── base/                   # 主程序入口
│   ├── MiracleVision.cpp  # 主函数
│   └── MiracleVision.hpp  # 主类定义
├── module/                 # 功能模块
│   ├── armor/             # 装甲板检测
│   ├── buff/              # 能量机关识别
│   ├── angle_solve/       # 姿态解算
│   ├── predictor/         # 弹道预测
│   ├── filter/            # 滤波器
│   ├── roi/               # 感兴趣区域
│   ├── ml/                # 深度学习推理
│   ├── camera/            # 相机标定
│   └── foxglove_publisher/ # 数据发布
├── devices/               # 设备层
│   ├── camera/            # 相机驱动
│   └── serial/            # 串口通信
├── utils/                 # 工具库
│   ├── logger.hpp         # 日志系统
│   ├── math_tools.hpp     # 数学工具
│   ├── img_tools.hpp      # 图像处理工具
│   └── fps.hpp            # 帧率统计
├── 3rdparty/              # 第三方库
│   ├── mindvision/        # MindVision SDK
│   ├── foxglove/          # Foxglove SDK
│   └── fmt/               # fmt 库（源码）
├── configs/               # 配置文件
│   ├── auto_aim.yaml      # 自瞄配置
│   ├── armor/             # 装甲板参数
│   ├── buff/              # 能量机关参数
│   ├── camera/            # 相机参数
│   └── serial/            # 串口配置
├── src/test/              # 测试程序
├── docs/                  # 文档
├── cmake/                 # CMake 模块
├── scripts/               # 工具脚本
└── video/                 # 测试视频
```

详细的架构说明请查看 [架构文档](docs/ARCHITECTURE.md)

---

## 📚 文档导航

### 入门文档
- [环境配置指南](docs/ENVIRONMENT_SETUP.md) - 系统环境、依赖安装、常见问题
- [构建指南](docs/BUILD_GUIDE.md) - CMake 配置、编译步骤、故障排除

### 开发文档
- [架构文档](docs/ARCHITECTURE.md) - 项目结构、模块说明、设计思路
- [开发指南](docs/DEVELOPMENT.md) - 开发环境、代码规范、Git 工作流

### 历史文档
- [GCC 升级指南](docs/gcc/GCC_UPGRADE_GUIDE.md) - 编译器问题解决
- [CMake 重构总结](docs/cmake/CMAKE_REFACTOR_SUMMARY.md) - 构建系统改进

---

## 🤝 贡献指南

我们欢迎所有形式的贡献！

### 如何贡献

1. **Fork 项目**
2. **创建特性分支**: `git checkout -b feature/AmazingFeature`
3. **提交更改**: `git commit -m 'Add some AmazingFeature'`
4. **推送到分支**: `git push origin feature/AmazingFeature`
5. **提交 Pull Request**

### 开发规范
- 遵循现有的代码风格
- 为新功能添加注释和文档
- 测试你的更改
- 更新相关文档

详细规范请查看 [开发指南](docs/DEVELOPMENT.md)

---

## 📝 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 👥 团队

**北京林业大学 RoboMaster 战队**

- 项目负责人: [@prototype956](https://github.com/prototype956)
- 视觉组成员: [贡献者列表](https://github.com/prototype956/EX_MiracleVision/graphs/contributors)

---

## 📞 联系我们

- **Issue**: [GitHub Issues](https://github.com/prototype956/EX_MiracleVision/issues)
- **讨论**: [GitHub Discussions](https://github.com/prototype956/EX_MiracleVision/discussions)
- **邮箱**: [待添加]

---

## 🙏 致谢

感谢以下开源项目和工具：

- [OpenCV](https://opencv.org/) - 计算机视觉库
- [Eigen](https://eigen.tuxfamily.org/) - 线性代数库
- [spdlog](https://github.com/gabime/spdlog) - 高性能日志库
- [fmt](https://github.com/fmtlib/fmt) - 格式化库
- [Foxglove Studio](https://foxglove.dev/) - 机器人可视化工具

---

## ⭐ Star History

如果这个项目对你有帮助，请给我们一个 Star ⭐！

[![Star History Chart](https://api.star-history.com/svg?repos=prototype956/EX_MiracleVision&type=Date)](https://star-history.com/#prototype956/EX_MiracleVision&Date)
