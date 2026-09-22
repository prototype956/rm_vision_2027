#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mv::hal::serial {

/** @brief 下位机串口链路参数；当前线协议固定使用 460800、8N1。 */
struct ControllerLinkConfig {
  bool enabled{true};
  std::string device;
  int baud{460800};
  int startup_sync_ms{100};
  int sync_ms{1000};
  int timeout_ms{3000};
};

/** @brief 串口和时钟同步快照，不包含任何 IMU 数学类型。 */
struct ControllerLinkState {
  bool connected{false};
  bool synchronized{false};
  std::uint64_t generation{0};  ///< 重建会话时递增，消费方据此清空历史。
  std::uint64_t crc_errors{0};
  double rate_hz{0};          ///< 姿态消息接收频率，不代表载荷有效性。
  double offset_us{0};        ///< 上位机单调时间减去 MCU 单调时间。
  double rtt_us{-1};
  std::string reason{"not_started"};
};

/** @brief 已校验版本、长度、CRC 和会话的消息信封，载荷由业务模块解释。 */
struct ControllerFrame {
  std::uint8_t type{0};
  std::uint32_t sequence{0};
  std::vector<std::uint8_t> payload;
  double receive_time_us{0};  ///< 上位机收到完整消息时的单调微秒数。
};

/**
 * @brief 独占串口与收发线程，负责分帧、会话、时间同步及断线重连。
 *
 * 回调在构造期间或收发线程中串行执行，不得销毁链路本身；消费方负责其数据的并发保护。
 * 空帧回调用于传播状态；收到消息时返回 false 可请求重建会话。
 * 析构等待回调和收发线程结束，回调不得向收发线程抛出异常。
 */
class ControllerLink final {
 public:
  using UpdateCallback = std::function<bool(
      const ControllerLinkState&, const std::optional<ControllerFrame>&)>;

  ControllerLink(ControllerLinkConfig config, UpdateCallback callback);
  ~ControllerLink();
  ControllerLink(const ControllerLink&) = delete;
  ControllerLink& operator=(const ControllerLink&) = delete;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mv::hal::serial
