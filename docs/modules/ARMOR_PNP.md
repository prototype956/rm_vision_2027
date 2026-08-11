# 装甲板 PnP

`armor_pnp` 使用当前帧相机标定和 TL/TR/BR/BL 灯条端点，通过
`solvePnPGeneric + SOLVEPNP_IPPE` 计算 `T_camera_optical_armor`。物点尺寸由
`src/config/modules/armor_pnp.yaml` 管理，默认小装甲 135×55 mm、大装甲 225×55 mm。

候选必须依次满足四点深度为正、装甲正面朝向相机和距离范围约束，最终选择重投影 RMSE
最小者。`ONE` 与 `BASE_BIG` 使用大装甲，其余检测标签使用小装甲。模块不使用历史帧，
不包含滤波或跟踪；角点精修由独立 `armor_corner_refiner` 在 PnP 之前完成。

## 同帧单链路

Talos 帧存在装甲真值时，每帧同时运行：

```text
真值世界四角 -> 同帧投影 -> IPPE -> 与 T_world_armor 比较
网络四角 -> JLU PCA 精修或原子回退 -> IPPE -> 与匹配真值比较
```

检测—真值匹配使用全局一对一分配。候选首先要求队伍和标签一致，并通过 IoU、归一化
中心距离和同索引角点距离硬门槛；随后对所有检测和真值联合最小化匹配代价。无可靠匹配的
检测结果仍发布三维估计，但真值误差保持为空。门槛位于 `armor_pnp.yaml`，不能通过放宽
门槛把明显错误的关联计入精度统计。

## 仿真验收

使用 `scripts/run_simulation_vision.sh` 启动完整链路。在 Foxglove 中叠加原图、
`/simulation/ground_truth/annotations`、`/vision/pnp/corners` 和
`/vision/pnp/reprojection`，并在 3D 面板以
`world` 为固定坐标系同时观察真值与估计装甲。

输入角点、PnP 重投影和真值误差线分别由 `/vision/pnp/corners`、
`/vision/pnp/reprojection` 和 `/vision/pnp/error_vectors` 独立显示。真值基准链继续写入
`/vision/pnp/stats`，其输入四角由黄色
`/simulation/ground_truth/annotations` 单独显示。真值二维投影只保留正面朝向相机且与
图像相交的装甲；原始和最终框不绘制角点身份文字，详细角点数据保留在统计消息中。
严格遮挡仍需仿真渲染端提供深度或可见性 ID。

按 2/4/6/8/10 m 和 0/15/30/45° 采集 MCAP。`/vision/pnp/stats.summary` 每 100 帧原子更新
真值与正式检测单链的累计 P50/P95；`summary_sequence` 标识全局与全部分组共同使用的
统计快照。距离、观察角和大小装甲分组均使用匹配真值，另提供精修成功率、回退原因、
raw/final 二维角点误差和单链 PnP 求解成功/候选切换计数。逐目标记录有符号角点偏差、相机系
XYZ 误差、有符号深度误差和两个 IPPE 候选的 RMSE 间隔。真值角点链要求重投影 RMSE ≤0.1 px、
位置误差 ≤5 mm；非正面退化视角的姿态误差 ≤0.5°。只有检测角点误差与位姿误差明显相关时，
才进入 PCA 灯条精修。四端点只有全部找到才整体替换网络四角；精修后 PnP 无解时直接报告
该目标无解，不再额外运行原始角点 PnP。

MCAP 优先在 Foxglove 中检查 `/vision/pnp/stats`、逐帧 attempt 和角点调试标注；需要聚合
统计时对指定 MCAP 做一次性只读解析，输出放在 `/tmp`，项目内不维护专用分析脚本。
