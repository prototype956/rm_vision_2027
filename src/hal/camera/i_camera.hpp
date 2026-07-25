/**
 * @file i_camera.hpp
 * @brief 相机硬件抽象接口 (ICamera)
 *
 * 【为什么要抽象接口而不直接用 mindvision::VideoCapture？】
 *
 *   原代码将 MindVision SDK 的具体调用分散在各业务模块里，导致：
 *   1. 单元测试无法注入假相机 —— 测试必须连接真实硬件；
 *   2. 切换相机型号（工业 → 网络摄像头）需要改动多处业务代码；
 *   3. HAS_MVSDK 宏守卫渗透到业务层，编译开关到处都是。
 *
 *   ICamera 是一个"防腐层"：
 *   - 上层（Pipeline / Algorithm）只看到 Open/Grab/Close，不知道底层是什么；
 *   - 下层（MindVisionCamera / OpenCvCamera / MockCamera）各自封装 SDK 细节；
 *   - 通过工厂函数创建具体实现，业务代码与 SDK 完全解耦。
 *
 * 【设计约定】
 *   - Grab() 是阻塞调用，内部设置超时（最多等待一帧周期）；
 *     超时返回 false，调用方可决定是否重试或报错。
 *   - Open() 失败时对象保持关闭状态，可再次调用 Open() 重试；
 *     不会抛异常——硬件错误是运行时正常事件，不是编程错误。
 *   - Close() 幂等：多次调用不产生副作用。
 *   - 析构自动调用 Close()，保证资源一定释放。
 */
#pragma once

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace mv::hal {

/**
 * @brief 相机设备的硬件无关抽象接口
 *
 * 所有具体相机类（工业相机、USB 相机、网络摄像头……）均派生自此类。
 * 上层代码通过 `std::unique_ptr<ICamera>` 持有，由工厂注入具体实现。
 */
class ICamera {
 public:
  ICamera() = default;
  virtual ~ICamera() = default;

  // C++ Core Guidelines C.67：多态基类应将拷贝/移动设为 protected 或 delete，
  // 防止通过基类指针发生对象切片（object slicing）。
  // 用 protected + default 而不是全部 delete：
  // 保留派生类自身的移动能力（用于存入容器），同时禁止跨类型切片。
  ICamera(const ICamera&) = delete;
  ICamera& operator=(const ICamera&) = delete;

 protected:
  ICamera(ICamera&&) = default;
  ICamera& operator=(ICamera&&) = default;

 public:
  /**
   * @brief 打开并初始化相机
   *
   * @param config  YAML 节点，字段由具体实现决定（见各子类文档）。
   *                例如 MindVision 需要 `exposure_us`、`resolution` 等；
   *                OpenCV 需要 `device_index` 或 `video_path`。
   * @return true   相机已就绪，可以调用 Grab()
   * @return false  打开失败（设备不存在、参数非法、权限不足等），
   *                错误原因已通过 spdlog 输出到日志，调用方只需处理 false
   */
  virtual bool Open(const YAML::Node& config) = 0;

  /**
   * @brief 关闭相机并释放所有相关资源
   *
   * 幂等：对已关闭的相机调用无副作用。
   * 析构函数保证一定会调用 Close()，因此手动调用是可选的。
   */
  virtual void Close() = 0;

  /**
   * @brief 从相机取一帧图像（阻塞，有超时）
   *
   * 内部设置硬件超时（通常 = 1 帧周期 + 裕量），
   * 超时后返回 false 而非无限等待，让调用方可以检查退出标志。
   *
   * @param[out] frame  成功时写入 BGR 格式 cv::Mat；
   *                    失败时 frame 内容未定义，调用方不应使用。
   * @return true   成功取帧，frame 有效
   * @return false  超时、相机断开或内部错误
   */
  virtual bool Grab(cv::Mat& frame) = 0;

  /**
   * @brief 查询相机是否处于已打开状态
   */
  [[nodiscard]] virtual bool IsOpen() const = 0;
};

}  // namespace mv::hal
