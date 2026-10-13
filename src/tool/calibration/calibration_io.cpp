#include "tool/calibration/calibration_io.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace mv::tool::calibration {
namespace {
std::string SessionTimestamp() {
  const auto NOW = std::chrono::system_clock::now();
  const std::time_t TIME = std::chrono::system_clock::to_time_t(NOW);
  std::tm local_time{};
  localtime_r(&TIME, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}
}  // namespace

std::filesystem::path CreateSessionDirectory(const std::filesystem::path& output_root) {
  std::filesystem::create_directories(output_root);
  const std::string BASE_NAME = SessionTimestamp();
  // 同一秒内重复启动时追加编号，避免覆盖已有样本和验收记录。
  for (int suffix = 0; suffix < 1000; ++suffix) {
    const std::string NAME = suffix == 0 ? BASE_NAME : fmt::format("{}_{:02d}", BASE_NAME, suffix);
    auto session_dir = output_root / NAME;
    std::error_code error;
    if (std::filesystem::create_directory(session_dir, error)) {
      return session_dir;
    }
    if (error && error != std::errc::file_exists) {
      throw std::filesystem::filesystem_error("cannot create calibration session", session_dir,
                                              error);
    }
  }
  throw std::runtime_error("cannot allocate a unique calibration session directory");
}

void DrawText(cv::Mat& image, const std::vector<std::string>& lines,
              const cv::Scalar& color) {
  int y = 28;
  for (const auto& line : lines) {
    cv::putText(image, line, {14, y}, cv::FONT_HERSHEY_SIMPLEX, 0.58, {0, 0, 0}, 3, cv::LINE_AA);
    cv::putText(image, line, {14, y}, cv::FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv::LINE_AA);
    y += 25;
  }
}

bool IsKey(int key, char lower, char upper) {
  return key == lower || key == upper;
}

void WriteCalibrationYaml(const std::filesystem::path& path, const YAML::Node& node) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << node;
  if (!emitter.good()) throw std::runtime_error("cannot serialize calibration metadata");
  auto temporary = path;
  temporary += ".tmp";
  std::ofstream output(temporary);
  output << emitter.c_str() << '\n';
  output.close();
  if (!output) throw std::runtime_error("cannot write " + temporary.string());
  std::filesystem::rename(temporary, path);
}

void SaveCalibrationImage(const std::filesystem::path& path, const cv::Mat& image) {
  if (!cv::imwrite(path.string(), image, {cv::IMWRITE_PNG_COMPRESSION, 3})) {
    throw std::runtime_error("cannot save image: " + path.string());
  }
}
}  // namespace mv::tool::calibration
