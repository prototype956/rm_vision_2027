#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

namespace mv::tool::calibration {
/** @brief 创建不覆盖已有结果的时间戳会话目录。 */
std::filesystem::path CreateSessionDirectory(const std::filesystem::path& output_root);
/** @brief 为采集和离线审核绘制统一文字提示。 */
void DrawText(cv::Mat& image, const std::vector<std::string>& lines,
              const cv::Scalar& color = {0, 255, 0});
bool IsKey(int key, char lower, char upper);
/** @brief 先写临时文件再原子替换清单，避免中断产生半份图像—姿态索引。 */
void WriteCalibrationYaml(const std::filesystem::path& path, const YAML::Node& node);
/** @brief 无损保存完整原图；写入失败抛出异常。 */
void SaveCalibrationImage(const std::filesystem::path& path, const cv::Mat& image);
}  // namespace mv::tool::calibration
