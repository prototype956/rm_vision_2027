#pragma once

#include "tool/foxglove/foxglove_config.hpp"
#include "tool/foxglove/runtime/subscription_registry.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string_view>

#include <filesystem>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>

namespace mv::tool::foxglove::runtime {

/**
 * @brief 共享传输会话的累计状态，不包含具体业务频道的发布计数。
 */
struct SessionSnapshot {
  std::uint64_t live_errors{0};          ///< WebSocket sink 的累计错误数。
  std::uint64_t recording_errors{0};     ///< MCAP sink 的累计错误数。
  std::uint64_t client_connects{0};      ///< WebSocket 客户端累计连接次数。
  std::uint64_t client_disconnects{0};   ///< WebSocket 客户端累计断开次数。
  std::uint64_t current_clients{0};      ///< 当前连接的客户端数量。
  bool live_active{false};               ///< WebSocket Server 当前是否可发布。
  bool recording_active{false};          ///< MCAP Writer 当前是否可写入。
  bool recording_closed_cleanly{false};  ///< Writer 是否成功写入尾部并关闭。
};

/**
 * @brief 管理所有 Foxglove 业务组件共享的实时与录制传输会话。
 *
 * 构造函数仅创建 Context；业务组件随后在对应 Context 中创建并注册频道，最后调用
 * Start() 启动 WebSocket Server 和 MCAP Writer。Stop() 必须在业务频道关闭后调用，
 * 从而保证 MCAP 收到完整的频道撤销和文件尾部。
 */
class FoxgloveSession final {
 public:
  /**
   * @brief 创建共享 Context 并预先确定本次录制路径。
   * @param config 已完成校验的 Foxglove 配置。
   */
  explicit FoxgloveSession(const Config& config);

  /** @brief 幂等关闭仍处于活动状态的 sink。 */
  ~FoxgloveSession();

  FoxgloveSession(const FoxgloveSession&) = delete;
  FoxgloveSession& operator=(const FoxgloveSession&) = delete;

  /** @brief 查询实时 Context 是否允许业务组件注册频道。 */
  [[nodiscard]] bool LiveConfigured() const noexcept;
  /** @brief 查询录制 Context 是否允许业务组件注册频道。 */
  [[nodiscard]] bool RecordingConfigured() const noexcept;
  /** @brief 获取实时频道使用的 Context；仅在 LiveConfigured() 为真时使用。 */
  [[nodiscard]] const ::foxglove::Context& LiveContext() const noexcept;
  /** @brief 获取录制频道使用的 Context；仅在 RecordingConfigured() 为真时使用。 */
  [[nodiscard]] const ::foxglove::Context& RecordingContext() const noexcept;

  /** @brief 将业务组件创建的实时频道加入订阅注册表。 */
  void RegisterLiveChannel(std::uint64_t channel_id);
  /** @brief 获取指定实时频道的当前及历史订阅状态。 */
  [[nodiscard]] SubscriptionSnapshot Subscription(std::uint64_t channel_id) const noexcept;

  /** @brief 记录实时频道初始化失败并停用实时会话。 */
  void FailLiveSetup(std::string_view message) noexcept;
  /** @brief 记录录制频道初始化失败并停用录制会话。 */
  void FailRecordingSetup(std::string_view message) noexcept;
  /** @brief 记录实时发布错误；致命 sink 错误会停用实时会话。 */
  void ReportLiveError(std::string_view operation, ::foxglove::FoxgloveError error) noexcept;
  /** @brief 记录 MCAP 写入错误；致命 sink 错误会停用录制会话。 */
  void ReportRecordingError(std::string_view operation, ::foxglove::FoxgloveError error) noexcept;

  /**
   * @brief 在所有业务频道注册完成后启动实时和录制 sink。
   *
   * 两个 sink 独立启动，任一失败只会停用自身。重复调用不会重复创建资源。
   */
  void Start() noexcept;
  /** @brief 幂等关闭 MCAP Writer 和 WebSocket Server。 */
  void Stop() noexcept;

  /** @brief 查询 WebSocket sink 当前是否可发布。 */
  [[nodiscard]] bool LiveActive() const noexcept;
  /** @brief 查询 MCAP sink 当前是否可写入。 */
  [[nodiscard]] bool RecordingActive() const noexcept;
  /** @brief 查询是否至少有一个 sink 仍处于活动状态。 */
  [[nodiscard]] bool AnyActive() const noexcept;
  /** @brief 获取共享会话的线程安全累计状态。 */
  [[nodiscard]] SessionSnapshot Snapshot() const noexcept;
  /** @brief 获取本次会话选择的 MCAP 路径，未启用录制时为空。 */
  [[nodiscard]] const std::filesystem::path& RecordingPath() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * @brief 判断 SDK 错误是否表示对应 sink 已无法继续使用。
 */
[[nodiscard]] bool IsFatalSinkError(::foxglove::FoxgloveError error) noexcept;

}  // namespace mv::tool::foxglove::runtime
