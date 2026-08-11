# 装甲灯条角点精修

配置 schema v4 提供三种互斥模式：`raw`、`percentile_pca_shadow` 和默认的
`gradient_axis_shadow`。三种模式都只驱动影子 PnP，正式输出仍采用网络原始角点。

`gradient_axis_shadow` 在网络左右灯条附近建立紧凑有向 ROI，以
`max(gray - ROI P50, 0)` 为权重直接求中心和协方差。PCA 只给出灯条中心轴；每个端点在
中心外 `0.4L～0.6L` 范围进行 0.25 px 双线性采样。`12～30 px` 的短灯条使用 3 条、较长
灯条使用 5 条法向扫描线；各轴向位置先取跨扫描线亮度中位数，再对融合的一维曲线做
Gaussian 平滑和三点抛物线亚像素峰值拟合。梯度内侧必须高于 ROI 背景亮度与对比度共同
确定的门槛，主峰必须与次峰充分分离，留一扫描线重算的峰值离散度不得超过 1 px。

该路径不使用颜色连通域、形态学或 `2%/98%` 像素云端点。旧逻辑仅隔离保留在
`percentile_pca_shadow`，用于 A/B 基线。

所有阈值位于 `src/config/modules/armor_corner_refiner.yaml`。精修依次检查 ROI 面积、支撑
像素、PCA 主次轴比例、轴偏差、中心偏移、融合梯度强度、峰值唯一性、重采样稳定度、固定
2 px 移动上限与最终四边形合法性。灯条短于 12 px 时跳过。四个端点必须全部通过才允许
进入精修 PnP；任意端点失败都会整块恢复原始四角，并通过 `reverted_by=armor_atomic` 保留
已经有效但未应用的端点候选。整体几何非法或精修 PnP 失败同样整块回退。

当前模块仅使用当帧 BGR 图像，不使用跟踪、历史姿态或滤波。主程序同时运行原始与精修
PnP；精修 PnP 无解但原始 PnP 有解时，也会回退到原角点并记录 `pnp_fallback`。

Foxglove 将输入角点、PnP 重投影和真值误差线分别发布到 `/vision/pnp/corners`、
`/vision/pnp/reprojection` 和 `/vision/pnp/error_vectors`。PCA 轴与梯度候选分别发布到
`/vision/corner_refiner/axes` 和 `/vision/corner_refiner/candidates`，可以独立显示或隐藏。
融合候选在图像中以较大的红/绿点显示；完整数值诊断包含主次峰、峰值离散度、请求移动、
最终移动和整块回退来源，保留在 `/vision/pnp/stats.attempts[].refinement`。

算法思想参考 FYT Vision Group 的 `LightCornerCorrector`（Apache License 2.0）：PCA
估计灯条对称轴，再沿轴寻找亮度突降端点。本实现按本项目接口和安全门槛重新编写，未复制
其代码片段。
