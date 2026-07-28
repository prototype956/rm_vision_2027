# RobotDetectionModel 0526 权重

本目录用于存放深圳大学 RobotPilots 开源项目的本地模型权重。权重文件不会提交或
随本仓库分发。

- 上游仓库：`https://github.com/broalantaps/RobotDetectionModel`
- 固定提交：`babaebd6c8b3aeceda0ca924772b2d70d51801b4`
- 本期模型：`0526.onnx`
- 输入：`images [1,3,640,640] FP16`
- 输出：`output [1,25200,22] FP32`
- 输出颜色通道 `9..12`：`blue、red、gray、purple`

颜色通道顺序按 `0526.onnx` 在 BGR 输入经 RGB 转换后的实际推理结果记录。上游文档
将前两个通道描述为红、蓝，但实测语义相反；本项目以模型实际输出为准。

上游仓库在上述提交中没有提供许可证文件，因此不能假定本项目的 Apache-2.0
许可证覆盖该权重。使用者需要自行确认模型的使用与分发权限。

本地缺失模型时，从固定提交的兄弟仓库手工复制：

```bash
cp /home/nuc/Workspace/RobotDetectionModel/Model/0526.onnx \
  /home/nuc/Workspace/rm_vision_2027/src/modules/armor_detector/models/0526.onnx
```

`0708.onnx` 仅可作为未承诺兼容性的本地备用文件；本期配置和测试都不使用它。

部署中的左上对齐 Letterbox 行为参考了本地同济视觉仓库
`/home/nuc/Workspace/sp_vision_25` 的固定提交
`58c627846b0344a62e780c60fffece4433d0fe53`。两个参考仓库都只用于核对模型协议
和部署行为，没有加入本项目 CMake、Git 子模块，也没有复制其 C++ 实现。
