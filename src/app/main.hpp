#pragma once

namespace mv::app {

/**
 * @brief 启动视觉检测、配置选择的相机、可选调试输出和 Talos 控制运行时。
 *
 * 函数负责初始化配置、打开相机并运行视觉处理循环。OpenCV 窗口与 Foxglove
 * 独立开关；无窗口时可通过 SIGINT/SIGTERM 正常退出并刷新 MCAP。
 *
 * @return 可直接作为进程退出状态使用的结果码。
 * @retval 0 用户正常结束程序。
 * @retval 1 捕获到标准异常。
 * @retval 2 检测器初始化失败。
 * @retval 3 相机打开失败。
 * @retval 4 相机断开连接或发生致命抓帧错误。
 * @retval 5 检测运行失败。
 * @retval 6 Talos 命令通道初始化失败。
 * @retval 7 控制线程异常退出。
 */
int Run();

}  // namespace mv::app
