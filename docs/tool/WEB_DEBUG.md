# Web 在线调参工具

Web 调试模块随 `mv-vision-main` 启动，在可信有线网络中提供运行状态、带装甲框的低帧率预览，
以及检测前端参数的临时在线调整。浏览器不需要安装客户端，页面资源全部包含在视觉程序中。
Foxglove 仍负责高频曲线、3D 场景和 MCAP；Web 服务异常不会阻塞正式视觉或控制链。

## 启动与连接

服务默认监听 `0.0.0.0:8080`，使用原有脚本一键启动 Talos 仿真和视觉主程序：

```bash
./scripts/run_simulation_vision.sh
```

视觉 PC 与笔电用网线直连时，建议把视觉 PC 的有线地址固定为 `10.42.0.1/24`，笔电使用
DHCP 或手动设置为同网段地址，然后访问 `http://10.42.0.1:8080`。第一阶段不修改
NetworkManager 或 systemd 配置；比赛前应在实际设备上单独验收地址分配。

页面本身可以匿名加载，状态、参数和图像 API 都要求密码。默认密码为
`rmvision2027`，在网页登录框输入；密码只保存在当前标签页的 `sessionStorage`。
当前使用明文 HTTP，密码只能防止误访问，不能防止同链路窃听，因此不要将
端口暴露到不可信网络。

`src/config/tool/web.yaml` 控制服务开关、监听地址、端口、密码、预览帧率和 JPEG 质量。
需要轮换密码时，修改 `auth.password` 后重启程序。密码以明文保存在该配置中，不得用于
不可信网络。配置解析失败或端口被占用时，程序记录警告并继续运行视觉主链。

## 在线调参语义

第一阶段开放以下参数：

- 装甲检测器的敌方颜色、置信度门限和 NMS IoU 门限。
- 独立灯条检测器的全部现有阈值、几何、颜色和候选容量参数。
- 角点精修器的全部现有参数。

网页编辑只修改浏览器草稿。点击“统一应用”后，服务端首先使用与启动 YAML 相同的解析器校验
完整候选配置，再提交带当前 revision 的事务。视觉线程在下一张有效图像进入流水线前统一替换
三个模块的参数；状态页会显示目标 revision 和实际生效帧号。已有待处理事务或页面 revision
过期时返回冲突，刷新后的当前值不会被旧页面覆盖。

“放弃草稿”恢复当前内存配置，“恢复启动值”把程序启动时读取的值作为新事务提交。所有修改
都只存在于内存，不写回 `src/config/`；程序重启后重新使用 YAML 配置。

模型路径、OpenVINO 设备、相机、PnP、预测、火控和 MPC 参数不能通过第一阶段页面修改。

## HTTP API

除 `/` 和 `/index.html` 外，请求都需要：

```text
Authorization: Bearer <password>
```

- `GET /api/v1/status`：最新视觉、灯条、预览和调参事务状态。
- `GET /api/v1/frontend-config`：启动配置、当前配置和 revision。
- `GET /api/v1/frontend-schema`：页面控件使用的字段元数据。
- `PUT /api/v1/frontend-config`：提交 `base_revision` 和完整 `config`。
- `GET /api/v1/preview.jpg`：获取最新带框 JPEG；尚无图像时返回 `503`。

示例：

```bash
curl -H "Authorization: Bearer rmvision2027" \
  http://10.42.0.1:8080/api/v1/status
```

接口不开放 CORS，请求体上限为 64 KiB。认证失败返回 `401`，版本或待处理事务冲突返回 `409`，
配置校验失败返回 `422`，请求体超限返回 `413`。

## 验收

无需相机的 loopback HTTP 与事务验收：

```bash
./build-openvino/bin/mv-web-debug-test
```

算法集成改动还应运行 Talos 仿真，并在浏览器改变检测前端参数，确认状态中的 revision 在下一帧
更新、图像预览继续刷新、Foxglove 保持连接且 100 Hz 控制线程没有异常。网线直连和比赛现场
网络为待用户实机验证项目。
