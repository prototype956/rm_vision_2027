# MindVision 相机内参与实机手眼标定

## 标定前准备

工具固定使用 `src/config/hal/camera/mindvision.yaml` 中的相机参数，并要求实际输出为
1280x720 BGR8。标定完成前不要改变镜头、焦距、对焦、硬件 ROI、分辨率或曝光模式。

默认标定板为 11x8 内角点，格长暂设 25 mm，使用前必须实测确认。11x8 是内角点数量，
对应 12x9 个黑白格；
实物尺寸不同时，先修改 `src/config/tool/camera_calibration.yaml`。

标定板需要由使用者自行准备，并在使用前实测方格尺寸。打印时选择“实际大小”或“100%”，
禁止“适合页面”“缩小超大页面”等自动缩放；建议使用哑光纸，并将棋盘平整粘贴到硬质板材上，
避免反光、翘曲、气泡和拉伸。仓库不再提供标定板生成脚本或预生成 PDF。

## 构建与启动

在连接 MindVision 相机的实机上构建：

```bash
cmake -S . -B build-camera \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MAIN=OFF \
  -DUSE_OPENVINO=OFF
cmake --build build-camera --parallel 4 --target mv-camera-calibration
```

推荐从统一标定入口启动：

```bash
./scripts/calibrate_camera.sh
```

菜单主项 `1` 为相机内参，子项 `1` 采集、`2` 解算；主项 `2` 为实机相机—云台外参，
子项 `1` 采集、`2` 审核解算。菜单读取配置的输出目录并筛选对应会话，按修改时间倒序
显示。`q` 返回上级，任务结束回到子菜单。仿真外参与 IMU 的入口、依赖及配置说明见
[统一标定与 IMU 入口](CALIBRATION.md)。

使用其他构建目录：

```bash
./scripts/calibrate_camera.sh --build-dir build-openvino intrinsic capture
```

相机标定工具需要提前构建；入口不会调用 `sudo`。

同一个脚本也提供仿真外参采集入口。它会启动 Talos 仿真器，并将采集程序连接到
共享内存；不会修改 `src/config/app/main.yaml`：

```bash
./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics capture
```

如果仿真器不在仓库同级的 `../bevy_robomaster_simulator`，设置其路径：

```bash
SIMULATOR_ROOT=/path/to/bevy_robomaster_simulator \
  ./scripts/calibrate_camera.sh --build-dir build-openvino simulation-extrinsics capture
```

该模式保存的是仿真外参采集会话，详见
[`EXTRINSIC_CALIBRATION.md`](EXTRINSIC_CALIBRATION.md)。

## 先采集，再离线解算

也可以使用显式命令进入采集模式：

```bash
./scripts/calibrate_camera.sh --build-dir build-openvino intrinsic capture
```

采集阶段只预览画面和保存原图，不运行棋盘检测、清晰度检查或内参求解。
因此没有绿色角点提示，未包含完整棋盘或模糊的图像也可以保存，稍后离线检测。
依次拍摄棋盘位于中心、四周、不同距离、水平倾斜和垂直倾斜的图像，避免反光和模糊。

| 按键 | 操作 |
|---|---|
| `Space` / `S` | 保存当前原始 PNG 图像 |
| `U` | 将最近保存的图像移到 `excluded/`，保留原图但不参与解算 |
| `Q` / `Esc` | 结束采集并退出 |

每次启动创建 `artifacts/camera_calibration/<时间戳>/`，其中 `images/` 保存原图，
`session.yaml` 保存采集时的相机实际参数和棋盘尺寸。采集文件的样本列表为空；
已采图片以 `images/` 中的 PNG 为准。此阶段不产生内参，按 `C` 不再触发求解。

退出采集后，将终端打印的采集目录传给离线解算命令：

```bash
./scripts/calibrate_camera.sh --build-dir build-openvino intrinsic solve artifacts/camera_calibration/<时间戳>
```

离线解算不打开相机，但需要图形桌面。程序按文件名排序扫描 `images/` 下的 PNG，
不要求编号连续。每张图检测后弹窗显示检测点和彩色连线、文件名及检测状态，
按任意键（包括 Q/Esc）继续下一张。检测失败的图片也会显示，按键后跳过。
关闭窗口或按 Ctrl+C 取消整个流程，写入 aborted 报告并返回退出码 6，不求解内参。
全部图片确认后关闭窗口，才开始求解内参和重投影误差。
终端输出内参矩阵、畸变系数、全局与最大单图 RMS、最差样本编号及逐图 RMS。
彩色连线表示检测点的排列顺序，不是重投影点；叠加画面不保存到磁盘。
它使用采集目录记录的相机、ROI 和棋盘参数，以及当前 `tool/camera_calibration.yaml`
中的质量阈值。不能把其他分辨率、ROI、相机或棋盘的图像混入同一目录。
无法读取或尺寸不匹配的图片会导致报错；未识别完整棋盘的图片会跳过并输出日志。
清晰度（拉普拉斯方差）和透视倾斜比仍写入报告用于诊断，但不作为筛选或验收条件；
平均灰度没有验收门限。

棋盘投影面积最大/最小倍率仅写入报告用于诊断，不作为验收条件。
质量检查仍要求至少 20 个有效样本、角点覆盖全部 3x3 区域、
全局 RMS 不超过 0.5 px、任一视图 RMS 不超过
1.0 px。失败时查看日志和解算报告，排除坏图后重新解算；采样不足则重新采集完整的一组。
需要排除任意图片时将它移到采集目录的 `excluded/`，不要删除原图。

## 解算输出

每次解算单独创建 `<采集目录>/solutions/<解算时间戳>/`：

- `session.yaml`：有效样本对应的原图路径、角点、覆盖统计、逐视图误差和失败原因。
- `intrinsics.yaml`：仅在全部质量条件通过后生成。

每次解算保留独立结果，质量失败返回退出码 5；读取或配置错误返回 1。
旧结果不会自动删除，使用时必须确认选取的是本次通过验收的结果。
后续由人工选择通过验收的 `intrinsics.yaml` 并接入 PnP。
内参绑定输出分辨率、硬件 ROI、相机和镜头状态；标定后可改变曝光，但不要改变几何设置。

已移除配置字段 `capture.min_sharpness`、`quality.min_tilted_views`、
`quality.min_tilt_ratio`、`quality.min_area_ratio`，旧配置需要删除这些字段。历史解算报告保持原样；
重新解算使用当前质量配置及采集目录中的棋盘尺寸。

## 单 yaw 相机外参标定

现有 `capture`、`solve` 命令仍用于内参标定；新增外参流程不改变历史内参会话格式。
两种流程共用棋盘参数、图像保存和检测实现。所有模式支持 `RM_VISION_CONFIG_ROOT`（空值使用默认配置），
相对配置/数据路径按项目根目录解释，
默认配置根目录为 `src/config`；输出根目录仍由 `tool/camera_calibration.yaml` 决定。

### 第一轮机械约定与准备

- 固定大 yaw、底盘及标定板，只转动小 yaw 和 pitch。相机与 IMU 刚性连接在最终 pitch 刚体上。
- 云台原点定义为小 yaw 与 pitch 转轴的近似交点；云台坐标轴为随该刚体运动的前、左、上。
- 标定结果为相机光学系到云台系：`p_G = R_GC * p_C + t_GC`，平移以米输出。
- 沿用已确认的 IMU 安装旋转，当前车辆为 `diag(-1,-1,1)`。不联合标定 IMU 安装误差、
  时间延迟或枪口外参。不是双 yaw 联动标定，不能用两根不相交的 yaw 轴一起采样。
- 当前内参、分辨率、ROI 和镜头状态须一致。外参模式要求相机的实际分辨率、ROI 匹配
  部署内参，并使用 BGR8；不会自动缩放内参。
- 内参采集允许移动棋盘；**外参采集过程中棋盘必须完全固定**。棋盘为当前配置的
  11×8 内角点、25 mm。可在棋盘外边框标记一个物理角，以便离线审核时统一编号方向。

外参采集读取 `runtime/real_geometry.yaml` 中当前 `active_profile` 对应的串口、内参、
完整安装外参；相机采集参数继续使用配置根目录下的 `hal/camera/mindvision.yaml`。
启动前停止占用相机或 IMU 串口的主程序/调试工具。底层 IMU 运行链路兼容普通固件的
`ATTITUDE`（type 5 / 42 字节）和标定固件的 `CALIBRATION_SAMPLE`
（type 6 / payload v2 / 96 字节），但手眼外参采集入口强制要求 type 6，并要求
`gyro_calibration_status == 2`、机构状态位及扩展字段全部满足采集门槛。
工具不发送运动或开火指令。

### 外参采集

```bash
cmake --build build-openvino --parallel 4 --target mv-camera-calibration
bash scripts/calibrate_camera.sh --build-dir build-openvino real-extrinsics capture
```

也可以使用前文 `BUILD_MAIN=OFF`、`USE_OPENVINO=OFF` 的独立相机构建。

等待预览显示 IMU 已同步。每次转到一个新 yaw/pitch 组合后停稳，再保存；建议至少保存
20–30 组完整清晰棋盘，让两根转轴都有充分变化，避免仅左右转动或反复保存同一个姿态。
保持棋盘完整出现在画面中，尽可能覆盖不同姿态。固定板手眼标定不沿用内参的远近面积
倍率和 3×3 图像覆盖门限，不要为了满足内参采样习惯移动棋盘。

| 按键（预览窗口获得焦点） | 操作 |
|---|---|
| `S` / 空格 | 保存显示图像及其采集时刻插值得到的 IMU 四元数 |
| `U` | 将最近有效图像—姿态对标为排除，保留原图 |
| `Q` / `Esc` / 关闭窗口 / Ctrl+C | 结束采集、保存清单 |

没有有效图像时间、IMU 未同步、插值失败或查询期间会话改变时，该次保存被拒绝，窗口显示原因。
不会使用“当前最新四元数”代替图像时刻的姿态。第一组保存后发生 IMU 会话重建，采集终止，
需重新启动建立新目录；已经保存的配对仍保留，不混入新的惯性参考。

每次创建 `artifacts/camera_calibration/extrinsics/<时间戳>/`（若修改 output_dir，则跟随它）：

- `images/`：无损原始 PNG，包括被排除的样本。
- `session.yaml`：`kind: single_yaw_handeye`、`schema_version: 1`，冻结车辆 profile、
  相机实际元数据及配置、IMU 配置、内参、完整安装外参、棋盘参数、最少样本数。
- `samples` 清单保存 ID、相对图像路径、active、图像单调时间（微秒）、归一化 `wxyz`
  四元数（IMU 解算机体系到惯性系）、IMU 会话代次及同步诊断。每次保存/排除立即更新清单。

清单是唯一配对依据。不要单独向 `images/` 拷贝其他照片，普通内参会话没有 IMU 姿态，
不能直接用于外参求解。写入中断产生的未列入清单的图片不会参与求解。

### 离线审核和求解

```bash
bash scripts/calibrate_camera.sh --build-dir build-openvino real-extrinsics solve \
  artifacts/camera_calibration/extrinsics/<时间戳>
```

离线只读取会话快照，不打开相机/串口，也不加载当前车辆配置。需要图形桌面。
逐张检测棋盘后显示 0 号角点、红色 X 方向及绿色 Y 方向：**各图必须指向同一个物理棋盘角及方向**。
不要把“图像左上角”误当成永久棋盘原点。对称棋盘存在 180° 编号歧义，需人工审核。

| 按键 | 操作 |
|---|---|
| `S` / 空格 | 接纳；执行 IPPE PnP，失败时留在该张并显示原因 |
| `F` | 反转全部角点顺序（等价于棋盘编号旋转 180°） |
| `X` | 排除该张；检测失败时也使用此键继续 |
| `Q` / `Esc` / 关闭窗口 / Ctrl+C | 取消本次审核，不生成候选外参 |

审核结束使用 OpenCV PARK 手眼算法；棋盘点按米构造，各帧云台到世界平移假设为零。
旋转使用运行时同一个入口：`R_WG = R_WI * R_GI^T`。

默认最少 20 个有效样本（与 `capture.min_samples` 一起冻结）。除了非法值和非法旋转，
还拒绝激励不足的运动：相对旋转小于 0.5° 不计入激励统计；相对旋转向量矩阵第二/第一
奇异值之比小于 0.05，或平移约束矩阵最小/最大奇异值之比小于 0.001，判为退化。
这些是可求解性检查，不是外参精度验收门限。失败时应增加不同 yaw/pitch 组合，不能只补相同姿态。
不会按误差自动剔除样本，也不将内参的 RMS 门限直接套到手眼结果。

### 外参结果与人工验收

每次求解创建独立的 `<采集目录>/solutions/<时间戳>/`：

- `review.yaml`：采集快照、每张图的接纳/排除及翻转决定、采用的角点、审核状态。
- `review/sample_*.png`：已审核图像的角点和编号方向诊断图；不是重投影叠加图。
- `report.yaml`：样本数、PARK 方法、每张图 PnP 重投影 RMS、相对转轴/转角分布、
  运动约束奇异值及条件数。求解成功时还包含每帧世界系棋盘位姿、相对于平均位姿的
  位置/旋转误差、总体 RMS、最大误差及最差样本 ID。
- `extrinsics_candidate.yaml`：仅数值求解成功时生成，兼容现有部署外参格式。
  只替换 `gimbal_r_camera`、`gimbal_t_camera_m`，保留采集时的 IMU、枪口字段和 placeholder。

候选矩阵按行展开；平移表示相机光学中心相对小 yaw/pitch 交点的位置。
`solved: true` 仅代表数值求解成功；`manual_validation_required: true` 表示精度待用户确认。
低 PnP 重投影误差不保证手眼外参正确，仍须检查固定棋盘位姿是否稳定、平移量是否符合机械尺寸。
不同转轴不完全相交、IMU 安装近似和时间误差都可能反映在一致性误差中。

退出码：0 数值求解成功或正常采集结束；1 配置/文件/其他异常；4 采集会话中止；
5 样本不足/运动退化/求解失败；6 用户取消审核。旧结果和原始采集数据不覆盖。

先把整个采集目录及最新 solutions 反馈分析，确认后再手动将候选文件用于所选车辆。
程序不自动修改运行配置，也不因相机外参求解成功就清除枪口等未标定外参的占位状态。
应用后重启视觉程序，用户实机检查固定目标随小 yaw/pitch 转动时的位置和方向稳定性。
本次开发只编译标定工具与主程序，不自动运行实机、仿真或新增测试套件。

### 2026-10-10 当前车辆结果

本次使用 `20260926_161918` 内参（1280×720、ROI `(0,152)`），采集目录为
`artifacts/camera_calibration/extrinsics/20261010_154217`，求解目录为
`solutions/20261010_154858`。30/30 样本接纳，OpenCV PARK 求解结果为：

```yaml
gimbal_r_camera:
  [-0.0006241308, 0.0291797530, 0.9995739855,
   -0.9999844861, -0.0055510177, -0.0004623408,
   0.0055351619, -0.9995587668, 0.0291827649]
gimbal_t_camera_m: [0.1453868590, 0.0007650707, 0.1585809998]
```

质量指标：PnP 重投影 RMS 0.109290 px、固定板位置一致性 RMS 0.012819 m、旋转一致性 RMS
0.497539°；最大位置残差 0.026761 m、最大旋转残差 1.104592°、平移约束条件数 2.249189。
该结果已人工写入当前车辆 `hal/imu/extrinsics.yaml`，原配置备份为求解目录中的
`deployment_before.yaml`。重启主程序后，用户在 Foxglove 中确认固定目标随小 yaw/pitch
转动时保持在同一惯性位置且无明显抖动。

本次验收只完成相机到云台外参。枪口外参仍为单位变换占位值，所以配置继续保留
`placeholder: true`，不得据此开启实机瞄准或开火。
