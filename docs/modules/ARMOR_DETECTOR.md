# 装甲检测模块

`mv-modules-armor-detector` 使用 OpenVINO 同步执行 RobotDetectionModel `0526.onnx`，
输出敌方装甲板的二维四角点、颜色、类别、objectness 和轴对齐外接框。本篇描述
当前实现的公共契约；模型来源与本地放置方式见
[模型说明](../../src/modules/armor_detector/models/README.md)，实机和离线验收流程见
[装甲检测验收](../test/ARMOR_DETECTOR_TEST.md)。

## 职责与边界

检测器负责：

- 校验配置、模型输入输出契约和 GPU 执行设备；
- 将任意正尺寸的 `CV_8UC3` BGR 图像缩放到左上对齐的 `640×640` 画布；
- 同步执行 OpenVINO 推理；
- 完成 objectness、颜色、有效区域、几何和 NMS 筛选；
- 将模型坐标映射并裁剪回调用方输入图像坐标；
- 提供最近一次成功检测的分阶段耗时。

当前模块不包含相机抓帧、畸变校正、PnP、装甲物理尺寸推断、目标选择、连续帧跟踪、
预测、串口通信或异步推理。检测器不会修改传入图像，实例不可拷贝、不可移动且
**非线程安全**；一个实例只允许成功调用一次 `Init()`。

## 模型和运行环境

本期只支持 `0526.onnx`，初始化时严格检查：

| 项目 | 固定契约 |
| --- | --- |
| 输入名称 | `images` |
| 输入形状 | `[1, 3, 640, 640]` |
| 输入类型 | FP16 |
| 输出名称 | `output` |
| 输出形状 | `[1, 25200, 22]` |
| 输出类型 | FP32 |

调用方输入为 BGR U8 NHWC 图像。左上 Letterbox 完成后，OpenVINO 预处理执行：

```text
BGR U8 NHWC → RGB → FP16 → /255 → NCHW
```

模型使用 `LATENCY` 性能模式和一个复用的 `InferRequest`。固定 `640×640` BGR
缓冲区与输入 Tensor 在初始化时建立并跨帧复用，初始化完成前使用黑图预热 10 次。

设备只接受 `GPU` 或 `GPU.<非负整数>`。初始化前要求配置设备出现在 OpenVINO
可用设备列表中；编译后要求 `EXECUTION_DEVICES` 全部为 `GPU` 或 `GPU.<index>`。
模块不接受 `CPU`、`AUTO`、`MULTI`，也不会自动回退 CPU。`GPU` 是 OpenVINO
通用设备名，单 GPU 主机编译后的实际执行设备可能显示为 `GPU.0`。

## 二维坐标系契约

### 输入图像坐标

所有公开坐标都属于本次传给 `Detect()` 的原始 `cv::Mat`，而不是相机传感器的完整
像面，也不是 `640×640` 模型画布。若调用方在检测前裁剪、缩放或旋转图像，返回
坐标只对应变换后的输入图像。

```text
                    +x
        (0, 0) o─────────────►
               │
               │   输入图像
               │
            +y ▼
```

- 原点 `(0,0)` 是左上像素中心；
- `+x` 向右，`+y` 向下；
- 单位为像素，使用 `float` 保存亚像素坐标；
- 宽为 `W`、高为 `H` 时，返回范围为
  `x ∈ [0, W-1]`、`y ∈ [0, H-1]`；
- 检测器不执行相机畸变校正，坐标是否处于去畸变图像中由调用方输入决定。

### 装甲四角点顺序

公开结果 `ArmorDetection::corners` 固定为：

```text
corners[0] TL ─────────────── TR corners[1]
              装甲板正面
corners[3] BL ─────────────── BR corners[2]
```

即 `TL → TR → BR → BL`，在当前 `+x` 向右、`+y` 向下的图像坐标系中沿装甲板
顺时针排列。这里的角点身份来自模型输出语义，不会在后处理中按 x/y 大小重新排序。
透视、倾斜或部分越界时，不应使用“最小 x/y”重新推断角点身份。

模型每行的原始角点顺序是 `LT → LB → RB → RT`。公开接口按下表重排：

| 公开索引 | 公开语义 | 模型原始角点 |
| --- | --- | --- |
| `corners[0]` | `TL` | `LT` |
| `corners[1]` | `TR` | `RT` |
| `corners[2]` | `BR` | `RB` |
| `corners[3]` | `BL` | `LB` |

`bounding_box` 是四个公开角点的轴对齐 `cv::Rect2f` 外接框：

```text
x      = min(corner.x)
y      = min(corner.y)
width  = max(corner.x) - min(corner.x)
height = max(corner.y) - min(corner.y)
```

它只用于快速定位和 NMS，不表达装甲板的旋转方向。

### 三维坐标边界

当前模块**没有定义装甲板局部三维坐标系**。装甲板物理原点、XYZ 轴方向、大小装甲
尺寸、三维角点顺序、相机坐标系和位姿正方向，必须由后续 PnP 模块统一定义，不能
从本篇二维 `TL/TR/BR/BL` 约定自行推导为公共三维契约。

## Letterbox 与坐标映射

设输入图像宽高为 `W、H`，模型画布宽高均为 640：

```text
scale = min(640 / W, 640 / H)
content_width  = clamp(round(W × scale), 1, 640)
content_height = clamp(round(H × scale), 1, 640)
```

缩放图像放在画布左上角，右侧和底部未使用区域填黑，没有居中偏移。原图到模型
画布的理想正向映射为：

```text
x_model = x_image × scale
y_model = y_image × scale
```

模型角点返回原图时使用：

```text
x_image = clamp(x_model / scale, 0, W - 1)
y_image = clamp(y_model / scale, 0, H - 1)
```

逆映射不减 padding，因为 padding 只位于右侧或底部。实现使用原始 `scale` 逆映射，
而不是使用舍入后 `content_width/W` 或 `content_height/H` 重新计算比例。

例如输入 `1280×720`：

```text
scale          = 0.5
有效图像区域   = 640×360
模型画布       = 640×640
右侧 padding   = 0
底部 padding   = 280
```

候选四角点的平均中心必须位于 `[0, content_width) × [0, content_height)`，否则
视为填充区候选并丢弃。坐标映射回原图后会裁剪到有效像素范围；裁剪后外接框宽高
近似为零或四边形面积近似为零的候选也会被拒绝。

## 输出 Tensor 和后处理

每个候选行包含 22 个 FP32 值：

| 索引 | 内容 |
| --- | --- |
| `0,1` | `LT.x, LT.y` |
| `2,3` | `LB.x, LB.y` |
| `4,5` | `RB.x, RB.y` |
| `6,7` | `RT.x, RT.y` |
| `8` | objectness logit |
| `9..12` | 颜色 logits |
| `13..21` | 9 类装甲标签 logits |

`0526.onnx` 在本项目部署链路中的实测颜色顺序为：

| 颜色索引 | 语义 |
| --- | --- |
| `0` | `BLUE` |
| `1` | `RED` |
| `2` | gray，过滤 |
| `3` | purple，过滤 |

上游文字资料将前两个颜色通道描述为红、蓝，但实际模型推理结果相反。本项目以
`0526.onnx` 实测语义为准。配置项 `enemy_color` 表示**需要保留的敌方装甲颜色**，
不是己方颜色。

标签通道与 `ArmorLabel` 枚举同序：

| 标签索引 | 枚举 |
| --- | --- |
| `0` | `SENTRY` |
| `1` | `ONE` |
| `2` | `TWO` |
| `3` | `THREE` |
| `4` | `FOUR` |
| `5` | `FIVE` |
| `6` | `OUTPOST` |
| `7` | `BASE_SMALL` |
| `8` | `BASE_BIG` |

后处理顺序为：

1. 任意一个输出值为 NaN 或 Inf 时丢弃整行。
2. 对索引 8 使用数值稳定 sigmoid，只用 objectness 做阈值过滤。
3. 对颜色和标签 logits 分别取 argmax；过滤灰、紫及非配置敌方颜色。
4. 过滤中心位于 Letterbox 填充区的候选。
5. 重排角点，映射并裁剪回输入图像，过滤退化几何。
6. 基于轴对齐外接框执行类别无关 IoU NMS。
7. 结果按 objectness 稳定降序返回。

颜色或标签 logit 不与 objectness 相乘。NMS 也只按 objectness 排序，不使用标签
分数，并且不区分颜色或装甲标签。

## 公开接口

公共头文件为
[`armor_detector.hpp`](../../src/modules/armor_detector/armor_detector.hpp)。

### 数据类型

- `ArmorColor`：公开红、蓝两种可返回颜色。
- `ArmorLabel`：模型支持的 9 类装甲标签。
- `ArmorDetection`：颜色、标签、sigmoid 后 objectness、轴对齐外接框和四角点。
- `DetectorStats`：最近一次成功检测的分阶段耗时与候选数量。
- `ArmorDetectorConfig`：已解析的模型、设备、敌方颜色和阈值配置。

### 生命周期与最小用法

```cpp
#include "core/config.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"

const std::filesystem::path project_root = "/home/nuc/Workspace/rm_vision_2027";
const auto yaml =
    mv::ConfigLoader::LoadFile(project_root / "src/config/modules/armor_detector.yaml");

mv::modules::YoloArmorDetector detector;
detector.Init(mv::modules::ParseArmorDetectorConfig(yaml, project_root));

// frame 必须是非空 CV_8UC3 BGR 图像。
const auto detections = detector.Detect(frame);
// 立即复制统计值，避免下一次 Detect() 覆盖。
const auto stats = detector.LastStats();
```

建议初始化顺序是“日志 → 检测配置和检测器 → 相机 → 调试窗口”。检测器初始化失败
时不要打开相机。无目标是正常情况，`Detect()` 返回空 `std::vector`，不应记录为
错误。

## YAML 配置

默认配置为
[`src/config/modules/armor_detector.yaml`](../../src/config/modules/armor_detector.yaml)：

```yaml
schema_version: 1
model_path: src/modules/armor_detector/models/0526.onnx
device: GPU
enemy_color: blue
confidence_threshold: 0.65
nms_iou_threshold: 0.45
```

| 字段 | 合法值与语义 |
| --- | --- |
| `schema_version` | 当前必须为 `1` |
| `model_path` | 非空路径；相对路径按项目根目录解析 |
| `device` | `GPU` 或 `GPU.<非负整数>` |
| `enemy_color` | 小写 `red` 或 `blue`，表示需要检测的敌方颜色 |
| `confidence_threshold` | objectness 阈值，开区间 `(0,1)` |
| `nms_iou_threshold` | 类别无关 NMS 阈值，闭区间 `[0,1]` |

所有字段都是必填项，未知键会被拒绝，避免字段拼写错误被静默忽略。

## 错误语义

| 阶段 | 异常 | 典型原因 |
| --- | --- | --- |
| 配置解析 | `ConfigError` | 缺少/未知字段、类型错误、值域错误、非法设备名 |
| 初始化前置状态 | `std::logic_error` | 同一实例重复调用 `Init()` |
| 初始化 | `ArmorDetectorInitError` | 模型缺失或不可读取、I/O 契约不匹配、GPU 不可见、编译失败、执行设备包含非 GPU |
| 检测参数 | `std::invalid_argument` | 输入图像为空或不是 `CV_8UC3` |
| 检测前置状态 | `std::logic_error` | 尚未成功初始化就调用 `Detect()` |
| 检测运行 | `ArmorDetectorRuntimeError` | OpenVINO 推理失败、输出 Tensor 异常或后处理失败 |

推理异常不会触发 CPU 回退。一次运行错误也不等价于“无目标”；只有成功返回的空集合
表示当前帧没有通过筛选的装甲板。

## 性能统计语义

`LastStats()` 返回检测器内部最近一次成功结果的只读引用，下一次 `Detect()` 会覆盖。
调用方需要在同一同步调用链中立即读取或复制。

| 字段 | 计时范围 |
| --- | --- |
| `preprocess_ms` | Letterbox 参数计算、复用画布清零和 `cv::resize` |
| `inference_ms` | `InferRequest::infer()`；包含编译进 OpenVINO 图中的 BGR→RGB、FP16 转换和 `/255` |
| `postprocess_ms` | 输出 Tensor 校验、解码、筛选、坐标映射和 NMS |
| `total_ms` | 上述三个阶段的完整检测链路 |
| `threshold_candidates` | 通过 objectness 阈值的原始行数，尚未经过颜色和几何筛选 |
| `kept_detections` | 完成颜色、几何筛选和 NMS 后的结果数 |

这些统计不包含相机抓帧、视频解码、调用方图像克隆、识别框/HUD 绘制、窗口刷新和
报告写盘。

## 接入与调试

公共识别框绘制接口为
[`DrawArmorDetections()`](../../src/tool/debug/armor_detection_overlay.hpp)，它原地绘制
`TL/TR/BR/BL` 四边形及颜色、标签和置信度，不负责检测或坐标变换。

实机长测、离线视频预览、性能基准和输出产物的使用方法统一维护在
[装甲检测验收](../test/ARMOR_DETECTOR_TEST.md)，本篇不重复测试命令和验收门槛。

常见问题：

- **模型缺失**：按模型说明将 `0526.onnx` 手工放到配置路径；运行时不会联网下载。
- **配置 `GPU.0` 不可见，但设备列表有 `GPU`**：单 GPU 环境优先配置 `GPU`；
  编译日志中的实际执行设备仍应是 `GPU.0`。
- **CPU、AUTO 或 MULTI 初始化失败**：这是禁止回退的预期行为。
- **输入类型错误**：确认图像非空且 `frame.type() == CV_8UC3`，通道语义为 BGR。
- **红蓝检测相反**：确认 `enemy_color` 表示目标敌方颜色；不要按上游错误的红蓝
  通道文字说明自行交换配置。
- **坐标整体偏移**：确认没有按居中 Letterbox 减 padding；当前画布左上对齐。
- **坐标无法对应原始相机画面**：检查调用方是否在检测前裁剪、缩放、旋转或
  去畸变；返回坐标只属于传入 `Detect()` 的那张图像。

