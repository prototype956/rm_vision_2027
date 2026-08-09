#pragma once

#include <string_view>

#include <foxglove/schemas.hpp>
#include <opencv2/core.hpp>

namespace mv::tool::foxglove::image {

/** @brief JPEG 消息及其独立测得的编码耗时。 */
struct EncodedImage {
  ::foxglove::schemas::CompressedImage message;  ///< 已拥有压缩字节的 Foxglove 消息。
  double jpeg_ms{0.0};                           ///< cv::imencode 耗时，单位为毫秒。
};

/**
 * @brief 将一帧 BGR 图像编码为 Foxglove JPEG 消息。
 *
 * 返回消息独立持有压缩数据，不依赖输入 cv::Mat 的后续生命周期。
 *
 * @param image 非空 BGR 图像；函数调用期间不得并发改写。
 * @param jpeg_quality OpenCV JPEG 质量，调用方保证范围为 [1, 100]。
 * @param frame_id 消息所属坐标系，在返回消息中复制保存。
 * @param timestamp 与同批次其他视觉消息共用的采集时间。
 * @throws std::runtime_error OpenCV 编码失败。
 */
[[nodiscard]] EncodedImage EncodeJpeg(const cv::Mat& image, int jpeg_quality,
                                      std::string_view frame_id,
                                      const ::foxglove::schemas::Timestamp& timestamp);

}  // namespace mv::tool::foxglove::image
