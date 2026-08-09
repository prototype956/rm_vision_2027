#include "tool/foxglove/image/image_message_encoder.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace mv::tool::foxglove::image {

EncodedImage EncodeJpeg(const cv::Mat& image, int jpeg_quality, std::string_view frame_id,
                        const ::foxglove::schemas::Timestamp& timestamp) {
  const auto START = std::chrono::steady_clock::now();
  std::vector<unsigned char> encoded;
  if (!cv::imencode(".jpg", image, encoded, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality})) {
    throw std::runtime_error("cv::imencode returned false");
  }

  EncodedImage result;
  result.jpeg_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - START).count();
  result.message.timestamp = timestamp;
  result.message.frame_id = frame_id;
  result.message.format = "jpeg";
  result.message.data.resize(encoded.size());
  std::memcpy(result.message.data.data(), encoded.data(), encoded.size());
  return result;
}

}  // namespace mv::tool::foxglove::image
