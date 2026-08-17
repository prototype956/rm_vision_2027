#pragma once

#include "hal/camera/i_camera.hpp"
#include "talos_config.hpp"

#include <memory>

namespace mv::hal::detail {

/**
 * @brief 独占 Talos 共享内存映射及三缓冲消费状态的内部设备对象。
 */
class TalosDevice final {
 public:
  /** @brief 创建尚未连接发布端的空设备对象。 */
  TalosDevice();
  ~TalosDevice();

  TalosDevice(const TalosDevice&) = delete;
  TalosDevice& operator=(const TalosDevice&) = delete;
  TalosDevice(TalosDevice&&) = delete;
  TalosDevice& operator=(TalosDevice&&) = delete;

  /** @brief 按配置映射共享内存并等待发布端心跳就绪。 */
  bool Open(const TalosConfig& config);
  /** @brief 解除全部映射并关闭文件描述符；允许重复调用。 */
  void Close() noexcept;
  /** @brief 等待、校验并复制一份独立持有的同步帧快照。 */
  GrabStatus Grab(CameraFrame& frame);

  /** @brief 返回 Open() 成功后生效的 Talos 输出信息。 */
  [[nodiscard]] CameraInfo Info() const;
  /** @brief 检查映射和发布端连接是否处于已打开状态。 */
  [[nodiscard]] bool IsOpen() const noexcept;

 private:
  struct Impl;                  ///< 隔离 POSIX 映射、协议视图和消费状态。
  std::unique_ptr<Impl> impl_;  ///< 当前设备唯一拥有的映射资源。
};

}  // namespace mv::hal::detail
