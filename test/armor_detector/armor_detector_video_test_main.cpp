/**
 * @file armor_detector_video_test_main.cpp
 * @brief 基于离线视频的装甲板检测与性能验收入口。
 */
#include "core/config.hpp"
#include "core/logger.hpp"
#include "modules/armor_detector/armor_detector.hpp"
#include "modules/armor_detector/armor_detector_config.hpp"
#include "tool/debug/armor_detection_overlay.hpp"
#include "tool/debug/debug_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <filesystem>
#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <optional>

namespace mv::test {
namespace {

constexpr std::size_t K_BENCHMARK_WARMUP_FRAMES = 100;
constexpr std::size_t K_BENCHMARK_MEASURED_FRAMES = 1000;
constexpr double K_P95_LIMIT_MS = 16.7;
constexpr char K_WINDOW_NAME[] = "Armor Detector Video Test";

using Clock = std::chrono::steady_clock;

// 将单调时钟时长统一换算为秒，用于累计时间和实际循环帧率。
double Seconds(Clock::duration duration) noexcept {
  return std::chrono::duration<double>(duration).count();
}

/**
 * @brief 离线视频检测和性能验收的命令行参数。
 */
struct Options {
  std::filesystem::path config_path =
      std::filesystem::path(CONFIG_FILE_PATH) / "modules/armor_detector.yaml";
  std::filesystem::path video_path;
  std::filesystem::path output_dir;
  std::int64_t start_frame{0};
  std::int64_t end_frame{-1};
  std::int64_t sample_stride{100};
  bool benchmark{false};
  bool preview{true};
};

/**
 * @brief 视频测试配置中由命令行参数覆盖的默认值。
 */
struct VideoTestDefaults {
  std::filesystem::path video_path;
  bool preview{true};
};

// 从视频测试配置读取本地视频目录、默认文件名和预览开关。
VideoTestDefaults LoadVideoTestDefaults() {
  constexpr char CONTEXT[] = "armor detector video test config";
  const auto CONFIG_PATH =
      std::filesystem::path(CONFIG_FILE_PATH) / "test/armor_detector_video_test.yaml";
  const auto ROOT = ConfigLoader::LoadFile(CONFIG_PATH);
  ConfigLoader::RejectUnknownKeys(ROOT, {"schema_version", "video_dir", "default_video", "preview"},
                                  CONTEXT);

  const auto VIDEO_DIR = ConfigLoader::Require<std::string>(ROOT, "video_dir", CONTEXT);
  const auto DEFAULT_VIDEO = ConfigLoader::Require<std::string>(ROOT, "default_video", CONTEXT);
  const bool PREVIEW = ConfigLoader::Require<bool>(ROOT, "preview", CONTEXT);
  if (VIDEO_DIR.empty()) {
    throw ConfigError("armor detector video test config.video_dir must not be empty");
  }

  const std::filesystem::path VIDEO_NAME = DEFAULT_VIDEO;
  if (VIDEO_NAME.empty() || VIDEO_NAME.is_absolute() || VIDEO_NAME.has_parent_path()) {
    throw ConfigError(
        "armor detector video test config.default_video must be a file name without directories");
  }

  const auto RESOLVED_DIR = ConfigLoader::ResolvePath(CONFIG_PATH.parent_path(), VIDEO_DIR);
  const auto VIDEO_PATH = (RESOLVED_DIR / VIDEO_NAME).lexically_normal();
  if (!std::filesystem::is_regular_file(VIDEO_PATH)) {
    throw ConfigError("default armor detector test video does not exist: " + VIDEO_PATH.string());
  }
  return {.video_path = VIDEO_PATH, .preview = PREVIEW};
}

// 为默认输出目录生成本地时间戳，避免多次运行互相覆盖。
std::string Timestamp() {
  const auto NOW = std::chrono::system_clock::now();
  const std::time_t TIME = std::chrono::system_clock::to_time_t(NOW);
  std::tm local{};
  localtime_r(&TIME, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y%m%d_%H%M%S");
  return stream.str();
}

/**
 * @brief 解析必须完整匹配的十进制整数参数。
 *
 * @param text 待解析的命令行文本。
 * @param option 参数名，用于生成错误信息。
 * @return 解析后的整数。
 * @throws std::invalid_argument 文本不是完整整数。
 * @throws std::out_of_range 数值超出 int64 范围。
 */
std::int64_t ParseInteger(const std::string& text, const std::string& option) {
  std::size_t consumed = 0;
  const auto VALUE = std::stoll(text, &consumed);
  if (consumed != text.size()) {
    throw std::invalid_argument(option + " requires an integer");
  }
  return VALUE;
}

// 打印帧区间、输出路径和 benchmark 模式的命令行用法。
void PrintUsage(const char* executable) {
  std::printf(
      "Usage: %s [--video PATH] [--config PATH] [--start-frame N] [--end-frame N]\n"
      "          [--output-dir PATH] [--sample-stride N] [--preview|--no-preview]\n"
      "          [--benchmark]\n"
      "Without overrides, video_dir/default_video/preview are read from "
      "src/config/test/armor_detector_video_test.yaml.\n"
      "Benchmark mode always disables preview.\n"
      "Frame range is [start-frame, end-frame); end-frame=-1 means EOF.\n",
      executable);
}

/**
 * @brief 解析并校验离线检测命令行参数。
 *
 * @param argc main() 接收到的参数个数。
 * @param argv main() 接收到的参数数组。
 * @return 可直接用于离线检测的参数集合。
 * @throws std::invalid_argument 缺少必需参数、参数未知或值域非法。
 */
Options ParseOptions(int argc, char** argv) {
  Options options;
  std::optional<bool> preview_override;
  options.output_dir =
      std::filesystem::path(PROJECT_ROOT_PATH) / "artifacts/detector_test" / Timestamp();

  for (int i = 1; i < argc; ++i) {
    const std::string ARGUMENT = argv[i];
    const auto REQUIRE_VALUE = [&](const std::string& option) -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument(option + " requires a value");
      }
      return argv[++i];
    };

    if (ARGUMENT == "--config") {
      options.config_path = REQUIRE_VALUE(ARGUMENT);
    } else if (ARGUMENT == "--video") {
      options.video_path = REQUIRE_VALUE(ARGUMENT);
    } else if (ARGUMENT == "--start-frame") {
      options.start_frame = ParseInteger(REQUIRE_VALUE(ARGUMENT), ARGUMENT);
    } else if (ARGUMENT == "--end-frame") {
      options.end_frame = ParseInteger(REQUIRE_VALUE(ARGUMENT), ARGUMENT);
    } else if (ARGUMENT == "--output-dir") {
      options.output_dir = REQUIRE_VALUE(ARGUMENT);
    } else if (ARGUMENT == "--sample-stride") {
      options.sample_stride = ParseInteger(REQUIRE_VALUE(ARGUMENT), ARGUMENT);
    } else if (ARGUMENT == "--benchmark") {
      options.benchmark = true;
    } else if (ARGUMENT == "--preview") {
      preview_override = true;
    } else if (ARGUMENT == "--no-preview") {
      preview_override = false;
    } else if (ARGUMENT == "--help" || ARGUMENT == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + ARGUMENT);
    }
  }

  if (options.start_frame < 0 || options.end_frame < -1 ||
      (options.end_frame >= 0 && options.end_frame <= options.start_frame)) {
    throw std::invalid_argument("invalid frame range");
  }
  if (options.sample_stride <= 0) {
    throw std::invalid_argument("--sample-stride must be positive");
  }
  const auto DEFAULTS = LoadVideoTestDefaults();
  if (options.video_path.empty()) {
    options.video_path = DEFAULTS.video_path;
  }
  options.preview = preview_override.value_or(DEFAULTS.preview);
  if (options.benchmark) {
    options.preview = false;
  }
  return options;
}

// 采用 nearest-rank 定义计算耗时分位数，输入按值传递以保留原采样顺序。
double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto RANK =
      static_cast<std::size_t>(std::ceil(percentile * static_cast<double>(values.size())));
  return values[std::min(values.size() - 1, std::max<std::size_t>(1, RANK) - 1)];
}

// 先绘制黑色粗字作为描边，保证 HUD 在明暗视频背景上都清晰可见。
void DrawOutlinedText(cv::Mat& image, const std::string& text, const cv::Point& origin,
                      double scale, const cv::Scalar& color) {
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 3,
              cv::LINE_AA);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}

// 绘制装甲检测结果以及与实机长测一致的核心实时指标。
void Draw(cv::Mat& image, const std::vector<modules::ArmorDetection>& detections,
          const modules::DetectorStats& stats, std::int64_t frame_index, double loop_fps,
          double elapsed_sec) {
  tool::DrawArmorDetections(image, detections);

  const std::vector<std::string> LINES = {
      fmt::format("frame: {}", frame_index),
      fmt::format("loop fps: {:.2f}", loop_fps),
      fmt::format("detections: {}  candidates: {}", detections.size(), stats.threshold_candidates),
      fmt::format("pre/infer/post: {:.2f}/{:.2f}/{:.2f} ms", stats.preprocess_ms,
                  stats.inference_ms, stats.postprocess_ms),
      fmt::format("total: {:.2f} ms", stats.total_ms),
      fmt::format("elapsed: {:.1f} s", elapsed_sec),
  };
  int y = 28;
  for (const auto& line : LINES) {
    DrawOutlinedText(image, line, {16, y}, 0.62, cv::Scalar(0, 255, 0));
    y += 27;
  }
}

// 将一帧的全部检测结果写为单行 JSON，便于流式处理和故障恢复。
void WriteDetections(std::ofstream& output, std::int64_t frame_index,
                     const std::vector<modules::ArmorDetection>& detections) {
  output << "{\"frame\":" << frame_index << ",\"detections\":[";
  for (std::size_t i = 0; i < detections.size(); ++i) {
    const auto& detection = detections[i];
    if (i != 0) {
      output << ',';
    }
    output << "{\"color\":\"" << modules::ArmorColorName(detection.color) << "\",\"label\":\""
           << modules::ArmorLabelName(detection.label)
           << "\",\"objectness\":" << detection.objectness << ",\"bbox\":["
           << detection.bounding_box.x << ',' << detection.bounding_box.y << ','
           << detection.bounding_box.width << ',' << detection.bounding_box.height
           << "],\"corners\":[";
    for (std::size_t point = 0; point < detection.corners.size(); ++point) {
      if (point != 0) {
        output << ',';
      }
      output << '[' << detection.corners[point].x << ',' << detection.corners[point].y << ']';
    }
    output << "]}";
  }
  output << "]}\n";
}

/**
 * @brief 对指定视频区间执行检测、保存样本并汇总性能指标。
 *
 * benchmark 模式先跳过 100 个预热帧，再固定统计 1000 帧；普通模式处理指定
 * 区间内的全部可读帧。视频解码和结果绘制耗时不计入 DetectorStats。
 *
 * @return 0 表示完成，7 表示 benchmark 的总耗时 P95 未达到 16.7 ms 门槛。
 */
int Run(int argc, char** argv) {
  const Options OPTIONS = ParseOptions(argc, argv);
  Logger::Instance().InitFromFile(std::filesystem::path(CONFIG_FILE_PATH) / "core/logger.yaml");

  const auto YAML = ConfigLoader::LoadFile(OPTIONS.config_path);
  const auto DETECTOR_CONFIG = modules::ParseArmorDetectorConfig(YAML, PROJECT_ROOT_PATH);
  modules::YoloArmorDetector detector;
  detector.Init(DETECTOR_CONFIG);

  MV_LOG_INFO("DetectorTest", "video: {}", OPTIONS.video_path.string());
  // 视频输入只负责提供 BGR 帧，检测配置和模型路径独立由 YAML 控制。
  cv::VideoCapture capture(OPTIONS.video_path.string());
  if (!capture.isOpened()) {
    throw std::runtime_error("cannot open video: " + OPTIONS.video_path.string());
  }
  if (OPTIONS.start_frame > 0 &&
      !capture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(OPTIONS.start_frame))) {
    throw std::runtime_error("video backend cannot seek to requested start frame");
  }

  const double VIDEO_FPS = capture.get(cv::CAP_PROP_FPS);
  const double FRAME_PERIOD_MS =
      std::isfinite(VIDEO_FPS) && VIDEO_FPS > 0.0 ? 1000.0 / VIDEO_FPS : 1.0;
  std::unique_ptr<tool::DebugWindow> window;
  if (OPTIONS.preview) {
    window = std::make_unique<tool::DebugWindow>(K_WINDOW_NAME, tool::WindowMode::NORMAL);
    MV_LOG_INFO("DetectorTest", "preview enabled at source FPS {:.3f}", VIDEO_FPS);
  }

  std::filesystem::create_directories(OPTIONS.output_dir);
  std::ofstream detections_file(OPTIONS.output_dir / "detections.jsonl");
  std::ofstream timings_file(OPTIONS.output_dir / "timings.csv");
  if (!detections_file || !timings_file) {
    throw std::runtime_error("cannot create detector test output files");
  }
  timings_file
      << "frame,preprocess_ms,inference_ms,postprocess_ms,total_ms,candidates,detections\n";

  std::vector<double> preprocess_times;
  std::vector<double> inference_times;
  std::vector<double> postprocess_times;
  std::vector<double> total_times;
  const auto START = Clock::now();
  std::size_t loop_frames = 0;
  std::size_t processed = 0;
  std::size_t warmed = 0;
  std::int64_t frame_index = OPTIONS.start_frame;
  bool stopped_by_user = false;
  cv::Mat frame;
  while ((OPTIONS.end_frame < 0 || frame_index < OPTIONS.end_frame) && capture.read(frame)) {
    const auto FRAME_START = Clock::now();
    // DetectorStats 必须在下一次 Detect() 前读取，否则会被后续帧覆盖。
    const auto DETECTIONS = detector.Detect(frame);
    const auto STATS = detector.LastStats();
    ++loop_frames;

    if (OPTIONS.benchmark && warmed < K_BENCHMARK_WARMUP_FRAMES) {
      // 预热帧不写报告、不保存样本，也不进入分位数统计。
      ++warmed;
      ++frame_index;
      continue;
    }

    WriteDetections(detections_file, frame_index, DETECTIONS);
    timings_file << frame_index << ',' << STATS.preprocess_ms << ',' << STATS.inference_ms << ','
                 << STATS.postprocess_ms << ',' << STATS.total_ms << ','
                 << STATS.threshold_candidates << ',' << STATS.kept_detections << '\n';
    preprocess_times.push_back(STATS.preprocess_ms);
    inference_times.push_back(STATS.inference_ms);
    postprocess_times.push_back(STATS.postprocess_ms);
    total_times.push_back(STATS.total_ms);

    // 保存样本与窗口预览共享同一张叠加图，避免同一帧重复绘制。
    const bool SAVE_SAMPLE = processed % static_cast<std::size_t>(OPTIONS.sample_stride) == 0;
    if (SAVE_SAMPLE || window) {
      const double ELAPSED_SEC = Seconds(Clock::now() - START);
      const double LOOP_FPS = static_cast<double>(loop_frames) / std::max(ELAPSED_SEC, 1.0e-6);
      cv::Mat overlay = frame.clone();
      Draw(overlay, DETECTIONS, STATS, frame_index, LOOP_FPS, ELAPSED_SEC);
      if (SAVE_SAMPLE) {
        const auto FILENAME = fmt::format("frame_{:08d}.jpg", frame_index);
        if (!cv::imwrite((OPTIONS.output_dir / FILENAME).string(), overlay)) {
          throw std::runtime_error("failed to write annotated sample: " + FILENAME);
        }
      }
      if (window) {
        window->Show(overlay);
        const double FRAME_ELAPSED_MS =
            std::chrono::duration<double, std::milli>(Clock::now() - FRAME_START).count();
        const int POLL_DELAY_MS =
            std::max(1, static_cast<int>(std::lround(FRAME_PERIOD_MS - FRAME_ELAPSED_MS)));
        if (window->Poll(POLL_DELAY_MS).exit_requested) {
          stopped_by_user = true;
        }
      }
    }
    ++processed;
    ++frame_index;

    if (stopped_by_user) {
      MV_LOG_INFO("DetectorTest", "preview exit requested after {} processed frames", processed);
      break;
    }
    if (OPTIONS.benchmark && processed >= K_BENCHMARK_MEASURED_FRAMES) {
      break;
    }
  }

  if (OPTIONS.benchmark && processed < K_BENCHMARK_MEASURED_FRAMES) {
    throw std::runtime_error(
        "benchmark requires 100 warmup frames and at least 1000 measured frames");
  }
  if (processed == 0) {
    throw std::runtime_error("no video frames were processed");
  }

  // 四个阶段分别汇总，便于区分预处理、GPU 推理和后处理瓶颈。
  const double TOTAL_MEAN =
      std::accumulate(total_times.begin(), total_times.end(), 0.0) / total_times.size();
  const double PREPROCESS_P95 = Percentile(preprocess_times, 0.95);
  const double INFERENCE_P95 = Percentile(inference_times, 0.95);
  const double POSTPROCESS_P95 = Percentile(postprocess_times, 0.95);
  const double TOTAL_P50 = Percentile(total_times, 0.50);
  const double TOTAL_P95 = Percentile(total_times, 0.95);
  const double TOTAL_P99 = Percentile(total_times, 0.99);

  std::ofstream summary(OPTIONS.output_dir / "summary.json");
  if (!summary) {
    throw std::runtime_error("cannot create detector test summary");
  }
  // summary 记录本次设备配置与性能门槛，逐帧明细保留在另外两个输出文件中。
  summary << std::fixed << std::setprecision(3) << "{\n  \"frames\":" << processed << ",\n"
          << "  \"benchmark\":" << (OPTIONS.benchmark ? "true" : "false") << ",\n"
          << "  \"device\":\"" << DETECTOR_CONFIG.device << "\",\n"
          << "  \"total_mean_ms\":" << TOTAL_MEAN << ",\n"
          << "  \"total_p50_ms\":" << TOTAL_P50 << ",\n"
          << "  \"total_p95_ms\":" << TOTAL_P95 << ",\n"
          << "  \"total_p99_ms\":" << TOTAL_P99 << ",\n"
          << "  \"preprocess_p95_ms\":" << PREPROCESS_P95 << ",\n"
          << "  \"inference_p95_ms\":" << INFERENCE_P95 << ",\n"
          << "  \"postprocess_p95_ms\":" << POSTPROCESS_P95 << ",\n"
          << "  \"p95_limit_ms\":" << K_P95_LIMIT_MS << ",\n"
          << "  \"p95_pass\":" << (TOTAL_P95 < K_P95_LIMIT_MS ? "true" : "false") << "\n}\n";
  if (!detections_file || !timings_file || !summary) {
    throw std::runtime_error("failed while writing detector test output");
  }

  MV_LOG_INFO("DetectorTest",
              "processed {} frames: total P50={:.3f} ms P95={:.3f} ms P99={:.3f} ms; "
              "stage P95 pre={:.3f} infer={:.3f} post={:.3f}",
              processed, TOTAL_P50, TOTAL_P95, TOTAL_P99, PREPROCESS_P95, INFERENCE_P95,
              POSTPROCESS_P95);
  if (OPTIONS.benchmark && TOTAL_P95 >= K_P95_LIMIT_MS) {
    MV_LOG_ERROR("DetectorTest", "P95 {:.3f} ms exceeds {:.1f} ms acceptance limit", TOTAL_P95,
                 K_P95_LIMIT_MS);
    return 7;
  }
  return 0;
}

}  // namespace
}  // namespace mv::test

int main(int argc, char** argv) {
  try {
    return mv::test::Run(argc, argv);
  } catch (const std::invalid_argument& error) {
    std::fprintf(stderr, "[DetectorTest] argument error: %s\n", error.what());
    mv::test::PrintUsage(argv[0]);
    return 2;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[DetectorTest] fatal: %s\n", error.what());
    return 1;
  }
}
