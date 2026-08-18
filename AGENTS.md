# Repository Guidelines（仓库贡献指南）

## 项目结构与模块组织

本项目是 Linux 平台的 C++20 RoboMaster 视觉系统。生产代码位于 `src/`：`app/` 是主程序入口，`core/` 提供配置与日志，`hal/` 封装相机和云台，`geometry/` 存放几何类型，`modules/` 按处理链组织装甲板检测、角点精修、PnP、预测、火控和云台轨迹规划，`tool/` 提供标定、调试窗口及 Foxglove 发布功能。运行时 YAML 位于 `src/config/`，其层级应与代码职责保持一致。验收程序在 `test/`，操作说明在 `docs/test/`；第三方代码集中在 `3rdparty/`，除升级依赖外不要修改。

## 构建、测试与本地运行

完整构建依赖 OpenVINO 2024.0：

```bash
source /opt/intel/openvino_2024.0.0/setupvars.sh
cmake -S . -B build-openvino -DCMAKE_BUILD_TYPE=Release -DUSE_OPENVINO=ON
cmake --build build-openvino --parallel 4
```

不需要检测器时，可增加 `-DBUILD_MAIN=OFF -DUSE_OPENVINO=OFF` 构建纯相机版本。使用 `./scripts/run_simulation_vision.sh` 联动启动 Talos 仿真和视觉主程序。当前没有注册 CTest；按改动范围运行验收程序：

```bash
./build-openvino/bin/mv-camera-test
./build-openvino/bin/mv-armor-detector-video-test --benchmark
```

实机测试前阅读 `docs/test/`。GPU 检测程序不得使用 `sudo`，登录用户需具备 `render` 和 `video` 组权限。

## 编码风格与命名规范

遵循仓库根目录的 `.clang-format`：Google 风格、2 空格缩进、禁止 Tab、同行大括号、每行不超过 100 字符。修改完成后，使用 `build-openvino/compile_commands.json` 检查所有受影响的项目源文件：

```bash
clang-format -i src/path/file.cpp src/path/file.hpp
clang-tidy -p build-openvino src/path/file.cpp
clangd --check=src/path/file.cpp --compile-commands-dir=build-openvino
```

修复本次修改新增或暴露的项目代码诊断，不要通过增加忽略规则掩盖真实问题；`.clangd` 中针对 clangd 14 已知误报的现有配置应保留。检查范围不包含 `3rdparty/`，也不要对其批量格式化。

类、结构体和函数使用 `CamelCase`，变量和命名空间使用 `lower_case`，私有/受保护成员以 `_` 结尾，常量、枚举值和宏使用 `UPPER_CASE`。头文件使用 `#pragma once`，项目头文件优先于标准库和第三方头文件。

修改代码时同步补充必要注释，重点解释公共接口、数据流、坐标系、单位、线程约束和非显然算法意图。沿用项目的 Doxygen 风格，包括 `/** @brief ... */`、`@param`、`@return` 和成员尾注释 `///< ...`；不要为显而易见的赋值、循环或语法行为添加冗余注释。

## 测试与配置要求

当前处于仿真验证算法阶段。算法代码修改后，应按影响范围运行 `./scripts/run_simulation_vision.sh` 完成 Talos 闭环测试，并结合程序日志和 Foxglove 话题检查行为。不要为了形式覆盖率编写脱离实际运行环境、没有验证价值的主机单元测试。

需要回放或深入定位时，由用户按需手动录制并提供 MCAP；Codex 不自行启用长期录制。MCAP 分析应保持只读，临时结果写入 `/tmp` 或 `artifacts/`。相机、标定等硬件功能仍按 `docs/test/` 执行验收；硬件不可用时，明确标记为“待用户实机验证”。

配置字段、公共接口、坐标系、单位、消息协议、运行命令或算法外部行为发生变化时，同步更新 `src/config/` 示例以及对应的 `docs/modules/`、`docs/tool/`、`docs/test/` 或 `README.md`。纯内部重构且外部行为不变时，不要求制造无意义的文档改动。

## 变更交付与提交规范

Codex 完成修改后只汇报代码和配置变化、实际执行的检查/仿真结果以及尚未验证的项目，不得自动执行 `git commit`、`git push` 或创建 Pull Request。由用户完成测试并确认功能稳定后手动提交。

手动提交时沿用仓库的 Conventional Commits 风格，例如 `feat:实现...`。使用 `feat:`、`fix:`、`docs:`、`refactor:` 等短前缀，主题应简洁且只描述一个逻辑改动。涉及可视化输出时可附截图或 Foxglove 录制；依赖相机、GPU、本地模型或仿真器时，应注明复现条件。

## 本地文件与安全注意事项

模型权重、视频、日志、构建目录和 `artifacts/` 均由 `.gitignore` 排除，不得强制提交。模型路径相对项目根目录解析；修改默认配置前确认不会写入机器专属绝对路径、设备编号或凭据。生成文件应保留在构建目录或 `artifacts/`，不要混入 `src/`。
