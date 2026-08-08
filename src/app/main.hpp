#pragma once

namespace mv::app {

/**
 * @brief 启动 YOLO 0526 检测器、配置选择的相机和可选调试输出。
 *
 * 函数负责初始化配置、打开相机、显示 HAL 返回的有效帧，并处理用户退出或
 * 不可恢复的相机错误。OpenCV 窗口与 Foxglove 独立开关；无窗口时
 * 可通过 SIGINT/SIGTERM 正常退出并刷新 MCAP。
 *
 * @return 可直接作为进程退出状态使用的结果码。
 * @retval 0 用户正常结束预览。
 * @retval 1 捕获到标准异常。
 * @retval 2 检测器初始化失败。
 * @retval 3 相机打开失败。
 * @retval 4 相机断开连接或发生致命抓帧错误。
 * @retval 5 检测运行失败。
 */
int Run();

}  // namespace mv::app
