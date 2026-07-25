/**
 * @file image_publisher.cpp
 * @brief ImagePublisher 实现（Mat → foxglove RawImage）
 *
 * 【关键实现细节】
 *
 *   1. 懒创建 Channel
 *      foxglove::schemas::RawImageChannel::create() 会向 WebSocket Server
 *      advertise 该 topic，只应在第一次实际使用时调用，避免向客户端发送
 *      无意义的空频道。GetOrCreateChannel() 在 Publish() 持锁后执行。
 *
 *   2. 图像数据拷贝
 *      foxglove RawImage.data 是 std::vector<std::byte>，必须 memcpy。
 *      若 img 不连续（ROI 子矩阵），img.step 与 img.cols*elemSize() 不等，
 *      此处直接使用 img.step 和 img.total()*elemSize()，Foxglove 解码时
 *      会按 step 字段正确处理行跨度，无需预先 clone 到连续缓冲区。
 *
 *   3. 编码字符串
 *      由 DetectEncoding() 从 Mat::type() 推断，未知类型回退到 "bgr8"。
 *      回退值在大多数情况下可正常显示，但颜色可能不准确。
 */
#include "tool/foxglove/detail/image_publisher.hpp"

#include "tool/foxglove/detail/utils.hpp"

#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace mv::tool::detail {

ImagePublisher::ImagePublisher(foxglove::Context ctx, bool use_jpeg, int jpeg_quality, int pub_w,
                               int pub_h)
    : ctx_(std::move(ctx)),
      use_jpeg_(use_jpeg),
      jpeg_quality_(jpeg_quality),
      pub_w_(pub_w),
      pub_h_(pub_h) {}

std::string ImagePublisher::DetectEncoding(const cv::Mat& img) {
  switch (img.type()) {
    case CV_8UC3:
      return "bgr8";
    case CV_8UC1:
      return "mono8";
    case CV_16UC1:
      return "16UC1";
    case CV_8UC4:
      return "bgra8";
    default:
      return "bgr8";  // 最通用的回退值
  }
}

foxglove::schemas::RawImageChannel* ImagePublisher::GetOrCreateChannel(const std::string& topic) {
  auto it = channels_.find(topic);
  if (it != channels_.end()) {
    return &it->second;
  }

  auto result = foxglove::schemas::RawImageChannel::create(topic, ctx_);
  if (!result.has_value()) {
    spdlog::error("[ImagePublisher] Failed to create channel '{}': {}", topic,
                  foxglove::strerror(result.error()));
    return nullptr;
  }
  auto [eit, ok] = channels_.emplace(topic, std::move(result.value()));
  return ok ? &eit->second : nullptr;
}

foxglove::schemas::CompressedImageChannel* ImagePublisher::GetOrCreateCompressedChannel(
    const std::string& topic) {
  auto it = compressed_channels_.find(topic);
  if (it != compressed_channels_.end()) {
    return &it->second;
  }

  auto result = foxglove::schemas::CompressedImageChannel::create(topic, ctx_);
  if (!result.has_value()) {
    spdlog::error("[ImagePublisher] Failed to create compressed channel '{}': {}", topic,
                  foxglove::strerror(result.error()));
    return nullptr;
  }
  auto [eit, ok] = compressed_channels_.emplace(topic, std::move(result.value()));
  return ok ? &eit->second : nullptr;
}

void ImagePublisher::Publish(const cv::Mat& img, const std::string& topic,
                             const std::string& frame_id, uint64_t ts_ns) {
  if (img.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);

  // ── JPEG 压缩路径 ─────────────────────────────────────────────────────────────
  if (use_jpeg_) {
    auto* ch = GetOrCreateCompressedChannel(topic);
    if (ch == nullptr) {
      return;
    }

    // 缩放（仅在需要时）
    const cv::Mat* src = &img;
    cv::Mat resized;
    if (pub_w_ > 0 && pub_h_ > 0 && (img.cols != pub_w_ || img.rows != pub_h_)) {
      cv::resize(img, resized, cv::Size(pub_w_, pub_h_), 0, 0, cv::INTER_AREA);
      src = &resized;
    }

    // JPEG 编码（在锁内，避免并发竞争缓冲区）
    std::vector<uchar> jpeg_buf;
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", *src, jpeg_buf, params)) {
      spdlog::warn("[ImagePublisher] JPEG encode failed for topic '{}'", topic);
      return;
    }

    foxglove::schemas::CompressedImage msg;
    msg.frame_id = frame_id;
    msg.format = "jpeg";
    msg.timestamp = ToTs(ts_ns);
    msg.data.resize(jpeg_buf.size());
    std::memcpy(msg.data.data(), jpeg_buf.data(), jpeg_buf.size());
    ch->log(msg, ts_ns);
    return;
  }

  // ── 原始 BGR8 路径 ───────────────────────────────────────────────────────────────
  auto* ch = GetOrCreateChannel(topic);
  if (ch == nullptr) {
    return;
  }

  // 构建 RawImage 消息
  foxglove::schemas::RawImage msg;
  msg.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(img.cols);
  msg.height = static_cast<uint32_t>(img.rows);
  msg.step = static_cast<uint32_t>(img.step);
  msg.encoding = DetectEncoding(img);
  msg.timestamp = ToTs(ts_ns);

  const size_t data_size = img.total() * img.elemSize();
  msg.data.resize(data_size);
  std::memcpy(msg.data.data(), img.data, data_size);

  ch->log(msg, ts_ns);
}

}  // namespace mv::tool::detail
