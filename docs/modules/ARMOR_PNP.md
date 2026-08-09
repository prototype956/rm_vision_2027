# 装甲板 PnP

`armor_pnp` 使用当前帧相机标定和 TL/TR/BR/BL 灯条端点，通过
`solvePnPGeneric + SOLVEPNP_IPPE` 计算 `T_camera_optical_armor`。物点尺寸由
`src/config/modules/armor_pnp.yaml` 管理，默认小装甲 135×55 mm、大装甲 225×55 mm。

候选必须依次满足四点深度为正、装甲正面朝向相机和距离范围约束，最终选择重投影 RMSE
最小者。`ONE` 与 `BASE_BIG` 使用大装甲，其余检测标签使用小装甲。模块不使用历史帧，
不包含滤波、跟踪或角点精修。

## 同帧双链路

Talos 帧存在装甲真值时，每帧同时运行：

```text
真值世界四角 -> 同帧投影 -> IPPE -> 与 T_world_armor 比较
检测四角 -----------------> IPPE -> 与匹配真值比较
```

匹配要求队伍和标签一致、投影多边形相交，并选择平均角点距离最小且尚未使用的真值。
无匹配的检测结果仍发布三维估计，但真值误差保持为空。

## 仿真验收

使用 `scripts/run_simulation_vision.sh` 启动完整链路。在 Foxglove 中叠加原图、
`/simulation/ground_truth/annotations` 和 `/vision/pnp/annotations`，并在 3D 面板以
`world` 为固定坐标系同时观察真值与估计装甲。

`/vision/pnp/annotations` 只绘制检测角点产生的 PnP 重投影，避免和真值基准链混为
同一种绿色。真值基准链继续写入 `/vision/pnp/stats`，其输入四角由黄色
`/simulation/ground_truth/annotations` 单独显示。真值二维投影只保留正面朝向相机且与
图像相交的装甲；严格遮挡仍需仿真渲染端提供深度或可见性 ID。

按 2/4/6/8/10 m 和 0/15/30/45° 采集 MCAP。`/vision/pnp/stats.summary` 每 100 帧更新
真值/检测两条链的累计 P50/P95；逐次记录可再按 `source`、`distance_m`、
`viewing_angle_deg` 分组。真值角点链要求重投影 RMSE ≤0.1 px、
位置误差 ≤5 mm；非正面退化视角的姿态误差 ≤0.5°。只有检测角点误差与位姿误差明显相关时，
才进入 PCA 灯条精修。
