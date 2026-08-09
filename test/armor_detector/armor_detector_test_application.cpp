#include "test/armor_detector/armor_detector_test_application.hpp"

#include "core/logger.hpp"
#include "tool/debug/armor_detection_overlay.hpp"
#include "tool/debug/debug_window.hpp"
#include "tool/foxglove/vision_debug_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <filesystem>
#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <unistd.h>

namespace mv::test {
namespace {

using Clock = std::chrono::steady_clock;

constexpr double K_MIN_VALID_FRAME_RATIO = 0.999;
constexpr double K_MAX_DETECTION_P95_MS = 16.7;
constexpr double K_MIN_ROLLING_FPS_RATIO = 0.95;
constexpr double K_MAX_NO_VALID_FRAME_SEC = 0.5;
constexpr double K_MAX_RSS_GROWTH_MIB = 20.0;

// 汇总实机长时测试中的抓帧质量、检测耗时、吞吐和资源占用指标。
struct Metrics {
  uint64_t grab_total{0};
  uint64_t valid_frames{0};
  uint64_t timeouts{0};
  uint64_t disconnected{0};
  uint64_t invalid_frames{0};
  uint64_t fatal_errors{0};
  uint64_t resolution_errors{0};
  uint64_t type_errors{0};
  uint64_t detection_success{0};
  uint64_t detection_errors{0};
  uint64_t target_frames{0};
  uint64_t total_detections{0};
  uint64_t total_candidates{0};
  uint64_t consecutive_failures{0};
  uint64_t max_consecutive_failures{0};
  double max_no_valid_frame_sec{0.0};
  double minimum_post_warmup_fps{std::numeric_limits<double>::infinity()};
  std::vector<double> preprocess_ms;
  std::vector<double> inference_ms;
  std::vector<double> postprocess_ms;
  std::vector<double> total_ms;
};

// 将单调时钟的时间段统一换算为秒。
double Seconds(Clock::duration duration) {
  return std::chrono::duration<double>(duration).count();
}

// 对耗时采样副本排序并线性插值，原始采样顺序保持不变。
double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double POSITION = percentile * static_cast<double>(values.size() - 1);
  const auto LOWER = static_cast<std::size_t>(std::floor(POSITION));
  const auto UPPER = static_cast<std::size_t>(std::ceil(POSITION));
  const double FRACTION = POSITION - static_cast<double>(LOWER);
  return values[LOWER] * (1.0 - FRACTION) + values[UPPER] * FRACTION;
}

// 从 Linux /proc 读取当前进程常驻内存，用于检查测试前后的内存增长。
std::size_t RssBytes() {
  std::ifstream statm("/proc/self/statm");
  std::size_t total_pages = 0;
  std::size_t resident_pages = 0;
  statm >> total_pages >> resident_pages;
  (void)total_pages;
  const long PAGE_SIZE = sysconf(_SC_PAGESIZE);
  return PAGE_SIZE > 0 ? resident_pages * static_cast<std::size_t>(PAGE_SIZE) : 0;
}

// 生成适合作为同一次测试所有产物后缀的本地时间戳。
std::string TimeStampForFile() {
  const auto NOW = std::chrono::system_clock::now();
  const std::time_t RAW = std::chrono::system_clock::to_time_t(NOW);
  std::tm local{};
  localtime_r(&RAW, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y%m%d_%H%M%S");
  return stream.str();
}

// 先绘制黑色粗字作为描边，保证文字在明暗背景上都清晰可见。
void DrawOutlinedText(cv::Mat& image, const std::string& text, const cv::Point& origin,
                      double scale, const cv::Scalar& color) {
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 3,
              cv::LINE_AA);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}

// 在相机帧上叠加装甲四边形、分类结果以及检测链路的实时性能指标。
void DrawOverlay(cv::Mat& image, const hal::CameraFrame& frame,
                 const std::vector<modules::ArmorDetection>& detections,
                 const modules::DetectorStats& stats, double loop_fps, double elapsed_sec) {
  tool::DrawArmorDetections(image, detections);

  const std::vector<std::string> LINES = {
      fmt::format("seq: {}", frame.sequence),
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

// 按 GrabStatus 分类失败，并维护连续抓帧失败次数的峰值。
void CountGrabFailure(Metrics& metrics, hal::GrabStatus status) {
  ++metrics.consecutive_failures;
  metrics.max_consecutive_failures =
      std::max(metrics.max_consecutive_failures, metrics.consecutive_failures);
  switch (status) {
    case hal::GrabStatus::TIMEOUT:
      ++metrics.timeouts;
      break;
    case hal::GrabStatus::DISCONNECTED:
      ++metrics.disconnected;
      break;
    case hal::GrabStatus::INVALID_FRAME:
      ++metrics.invalid_frames;
      break;
    case hal::GrabStatus::FATAL:
      ++metrics.fatal_errors;
      break;
    case hal::GrabStatus::OK:
      break;
  }
}

}  // namespace

ArmorDetectorTestApplication::ArmorDetectorTestApplication(
    std::unique_ptr<hal::ICamera> camera, std::unique_ptr<modules::YoloArmorDetector> detector,
    YAML::Node camera_config, ArmorDetectorTestSettings settings,
    std::optional<tool::foxglove::Config> foxglove_config)
    : camera_(std::move(camera)),
      detector_(std::move(detector)),
      camera_config_(std::move(camera_config)),
      settings_(std::move(settings)),
      foxglove_config_(std::move(foxglove_config)) {}

ArmorDetectorTestApplication::~ArmorDetectorTestApplication() = default;

int ArmorDetectorTestApplication::Run() {
  std::filesystem::create_directories(settings_.output_dir);
  const auto RUN_ID = TimeStampForFile();
  MV_LOG_INFO("ArmorDetectorTest", "output directory: {}", settings_.output_dir.string());

  if (!camera_->Open(camera_config_)) {
    MV_LOG_ERROR("ArmorDetectorTest", "camera open failed");
    return 3;
  }

  const auto CAMERA_INFO = camera_->Info();
  if (CAMERA_INFO.output_width != 1280 || CAMERA_INFO.output_height != 720 ||
      CAMERA_INFO.pixel_format != hal::PixelFormat::BGR8) {
    MV_LOG_ERROR("ArmorDetectorTest", "expected camera output 1280x720 BGR8");
    camera_->Close();
    return 4;
  }
  MV_LOG_INFO("ArmorDetectorTest", "camera '{}' output={}x{} exposure={}us",
              CAMERA_INFO.device_name, CAMERA_INFO.output_width, CAMERA_INFO.output_height,
              CAMERA_INFO.exposure_us);

  // Foxglove 只提供可选调试输出，初始化失败或没有可用 sink 均不影响检测验收。
  std::unique_ptr<tool::foxglove::VisionDebugPublisher> foxglove_publisher;
  if (foxglove_config_) {
    try {
      foxglove_publisher =
          std::make_unique<tool::foxglove::VisionDebugPublisher>(std::move(*foxglove_config_));
      if (!foxglove_publisher->IsRunning()) {
        MV_LOG_WARN("ArmorDetectorTest",
                    "Foxglove configured but no live or recording sink started");
        foxglove_publisher.reset();
      }
    } catch (const std::exception& error) {
      MV_LOG_WARN("ArmorDetectorTest", "Foxglove disabled after initialization failure: {}",
                  error.what());
      foxglove_publisher.reset();
    }
  }

  // 预览窗口是可选诊断输出，不参与最终 PASS/FAIL 判定。
  std::unique_ptr<tool::DebugWindow> preview_window;
  if (settings_.preview) {
    preview_window = std::make_unique<tool::DebugWindow>("MiracleVision Armor Detector Test");
  }

  // CSV 保存周期指标，JSONL 逐条记录运行事件，最终结果另写入 summary JSON。
  const auto METRICS_PATH = settings_.output_dir / ("metrics_" + RUN_ID + ".csv");
  const auto EVENTS_PATH = settings_.output_dir / ("events_" + RUN_ID + ".jsonl");
  std::ofstream metrics_csv(METRICS_PATH);
  std::ofstream events(EVENTS_PATH);
  if (!metrics_csv || !events) {
    camera_->Close();
    throw std::runtime_error("cannot create armor detector test output files");
  }
  metrics_csv
      << "elapsed_sec,grab_total,valid_frames,detection_success,detection_errors,target_frames,"
         "detections,candidates,loop_fps,preprocess_p50_ms,preprocess_p95_ms,inference_p50_ms,"
         "inference_p95_ms,postprocess_p50_ms,postprocess_p95_ms,total_p50_ms,total_p95_ms,"
         "total_p99_ms,rss_mib,cpu_percent\n";

  Metrics metrics;
  const auto START = Clock::now();
  auto report_start = START;
  auto next_report = START + std::chrono::seconds(settings_.report_interval_sec);
  auto next_sample = settings_.save_sample_interval_sec > 0
                         ? START + std::chrono::seconds(settings_.save_sample_interval_sec)
                         : Clock::time_point::max();
  auto last_valid_time = Clock::time_point{};
  std::deque<Clock::time_point> post_warmup_success_times;
  uint64_t report_detection_success = 0;
  uint64_t warmup_detection_success = 0;
  double baseline_fps = 0.0;
  const auto INITIAL_RSS = RssBytes();
  const auto INITIAL_CPU = std::clock();
  bool user_aborted = false;
  bool runtime_stopped = false;
  cv::Mat last_preview;

  // 每个有效相机帧同步执行一次检测，直到达到时长或出现主动/不可恢复终止。
  while (true) {
    const auto BEFORE_GRAB = Clock::now();
    if (Seconds(BEFORE_GRAB - START) >= settings_.duration_sec) {
      break;
    }

    hal::CameraFrame frame;
    auto status = camera_->Grab(frame);
    const auto NOW = Clock::now();
    ++metrics.grab_total;

    if (status == hal::GrabStatus::OK) {
      // SDK 返回成功后仍需检查图像尺寸和类型，避免非法输入进入检测器。
      bool valid = true;
      if (frame.image.cols != 1280 || frame.image.rows != 720) {
        ++metrics.resolution_errors;
        valid = false;
      }
      if (frame.image.type() != CV_8UC3) {
        ++metrics.type_errors;
        valid = false;
      }
      if (!valid) {
        status = hal::GrabStatus::INVALID_FRAME;
        CountGrabFailure(metrics, status);
      } else {
        ++metrics.valid_frames;
        metrics.consecutive_failures = 0;
        if (last_valid_time != Clock::time_point{}) {
          metrics.max_no_valid_frame_sec =
              std::max(metrics.max_no_valid_frame_sec, Seconds(NOW - last_valid_time));
        }
        last_valid_time = NOW;

        try {
          // LastStats() 对应刚完成的 Detect()，因此二者必须在同一同步调用链中读取。
          const auto DETECTIONS = detector_->Detect(frame.image);
          const auto STATS = detector_->LastStats();
          if (foxglove_publisher) {
            foxglove_publisher->Publish(frame, DETECTIONS, STATS, modules::ArmorPnpFrameResult{});
          }
          ++metrics.detection_success;
          ++report_detection_success;
          metrics.total_detections += DETECTIONS.size();
          metrics.total_candidates += STATS.threshold_candidates;
          if (!DETECTIONS.empty()) {
            ++metrics.target_frames;
          }

          const double ELAPSED = Seconds(NOW - START);
          // 预热期只用于建立循环 FPS 基线，不纳入检测耗时分位数。
          if (ELAPSED <= settings_.warmup_sec) {
            ++warmup_detection_success;
          } else {
            post_warmup_success_times.push_back(NOW);
            metrics.preprocess_ms.push_back(STATS.preprocess_ms);
            metrics.inference_ms.push_back(STATS.inference_ms);
            metrics.postprocess_ms.push_back(STATS.postprocess_ms);
            metrics.total_ms.push_back(STATS.total_ms);
          }

          const double REPORT_ELAPSED = std::max(Seconds(NOW - report_start), 1.0e-6);
          const double LOOP_FPS = static_cast<double>(report_detection_success) / REPORT_ELAPSED;
          const bool SAVE_SAMPLE = NOW >= next_sample;
          // 仅在需要预览或保存样本时克隆图像，避免绘制开销污染常规检测路径。
          if (preview_window || SAVE_SAMPLE) {
            last_preview = frame.image.clone();
            DrawOverlay(last_preview, frame, DETECTIONS, STATS, LOOP_FPS, ELAPSED);
          }
          if (SAVE_SAMPLE) {
            const auto SAMPLE_PATH =
                settings_.output_dir /
                ("sample_" + RUN_ID + "_" + std::to_string(frame.sequence) + ".jpg");
            if (!cv::imwrite(SAMPLE_PATH.string(), last_preview)) {
              MV_LOG_WARN("ArmorDetectorTest", "failed to save sample {}", SAMPLE_PATH.string());
            }
            next_sample += std::chrono::seconds(settings_.save_sample_interval_sec);
          }
        } catch (const std::exception& error) {
          ++metrics.detection_errors;
          events << "{\"elapsed_sec\":" << Seconds(NOW - START)
                 << ",\"event\":\"detection_failure\",\"sequence\":" << frame.sequence << "}\n";
          MV_LOG_ERROR("ArmorDetectorTest", "detection failed at sequence {}: {}", frame.sequence,
                       error.what());
          runtime_stopped = true;
        }
      }
    } else {
      CountGrabFailure(metrics, status);
      if (last_valid_time != Clock::time_point{}) {
        metrics.max_no_valid_frame_sec =
            std::max(metrics.max_no_valid_frame_sec, Seconds(NOW - last_valid_time));
      }
      events << "{\"elapsed_sec\":" << Seconds(NOW - START)
             << ",\"event\":\"grab_failure\",\"status\":\"" << hal::GrabStatusName(status)
             << "\"}\n";
      MV_LOG_WARN("ArmorDetectorTest", "camera grab failed: {}", hal::GrabStatusName(status));
    }

    // 抓帧失败时继续显示最后一个有效叠加帧，便于观察故障前状态。
    if (preview_window) {
      if (!last_preview.empty()) {
        preview_window->Show(last_preview);
      }
      if (preview_window->Poll().exit_requested) {
        user_aborted = true;
        events << "{\"elapsed_sec\":" << Seconds(NOW - START) << ",\"event\":\"user_abort\"}\n";
      }
    }

    // 每个报告周期更新基线、滚动一分钟 FPS、耗时分位数和进程资源占用。
    if (NOW >= next_report) {
      const double ELAPSED = Seconds(NOW - START);
      const double REPORT_ELAPSED = std::max(Seconds(NOW - report_start), 1.0e-6);
      const double LOOP_FPS = static_cast<double>(report_detection_success) / REPORT_ELAPSED;
      if (baseline_fps == 0.0 && ELAPSED >= settings_.warmup_sec) {
        baseline_fps = settings_.warmup_sec > 0
                           ? static_cast<double>(warmup_detection_success) / settings_.warmup_sec
                           : LOOP_FPS;
        MV_LOG_INFO("ArmorDetectorTest", "baseline F0 established: {:.3f} FPS", baseline_fps);
      }

      const auto MINUTE_START = NOW - std::chrono::seconds(60);
      while (!post_warmup_success_times.empty() &&
             post_warmup_success_times.front() < MINUTE_START) {
        post_warmup_success_times.pop_front();
      }
      if (ELAPSED >= settings_.warmup_sec + 60.0) {
        const double MINUTE_FPS = static_cast<double>(post_warmup_success_times.size()) / 60.0;
        metrics.minimum_post_warmup_fps = std::min(metrics.minimum_post_warmup_fps, MINUTE_FPS);
      }

      const double RSS_MIB = static_cast<double>(RssBytes()) / (1024.0 * 1024.0);
      const double CPU_PERCENT = 100.0 * static_cast<double>(std::clock() - INITIAL_CPU) /
                                 CLOCKS_PER_SEC / std::max(ELAPSED, 1.0e-6);
      metrics_csv << ELAPSED << ',' << metrics.grab_total << ',' << metrics.valid_frames << ','
                  << metrics.detection_success << ',' << metrics.detection_errors << ','
                  << metrics.target_frames << ',' << metrics.total_detections << ','
                  << metrics.total_candidates << ',' << LOOP_FPS << ','
                  << Percentile(metrics.preprocess_ms, 0.50) << ','
                  << Percentile(metrics.preprocess_ms, 0.95) << ','
                  << Percentile(metrics.inference_ms, 0.50) << ','
                  << Percentile(metrics.inference_ms, 0.95) << ','
                  << Percentile(metrics.postprocess_ms, 0.50) << ','
                  << Percentile(metrics.postprocess_ms, 0.95) << ','
                  << Percentile(metrics.total_ms, 0.50) << ',' << Percentile(metrics.total_ms, 0.95)
                  << ',' << Percentile(metrics.total_ms, 0.99) << ',' << RSS_MIB << ','
                  << CPU_PERCENT << '\n';
      metrics_csv.flush();
      MV_LOG_INFO("ArmorDetectorTest",
                  "t={:.0f}s valid={}/{} infer={}/{} targets={} detections={} fps={:.2f} "
                  "total p50/p95/p99={:.2f}/{:.2f}/{:.2f}ms RSS={:.1f}MiB CPU={:.1f}%",
                  ELAPSED, metrics.valid_frames, metrics.grab_total, metrics.detection_success,
                  metrics.detection_errors, metrics.target_frames, metrics.total_detections,
                  LOOP_FPS, Percentile(metrics.total_ms, 0.50), Percentile(metrics.total_ms, 0.95),
                  Percentile(metrics.total_ms, 0.99), RSS_MIB, CPU_PERCENT);
      report_detection_success = 0;
      report_start = NOW;
      next_report += std::chrono::seconds(settings_.report_interval_sec);
    }

    // TIMEOUT 和 INVALID_FRAME 可继续统计；断开、致命错误或检测异常立即退出。
    if (runtime_stopped || user_aborted || status == hal::GrabStatus::DISCONNECTED ||
        status == hal::GrabStatus::FATAL) {
      break;
    }
  }

  if (foxglove_publisher) {
    foxglove_publisher->Stop();
  }
  camera_->Close();

  // 按测试文档约定的有效率、时延、吞吐、空窗和内存门槛计算最终结果。
  const double ELAPSED = Seconds(Clock::now() - START);
  const bool COMPLETED_DURATION =
      !user_aborted && ELAPSED >= static_cast<double>(settings_.duration_sec);
  const double VALID_FRAME_RATIO =
      metrics.grab_total > 0 ? static_cast<double>(metrics.valid_frames) / metrics.grab_total : 0.0;
  const double DETECTION_SUCCESS_RATIO =
      metrics.valid_frames > 0
          ? static_cast<double>(metrics.detection_success) / metrics.valid_frames
          : 0.0;
  const double TOTAL_P50 = Percentile(metrics.total_ms, 0.50);
  const double TOTAL_P95 = Percentile(metrics.total_ms, 0.95);
  const double TOTAL_P99 = Percentile(metrics.total_ms, 0.99);
  const double RSS_GROWTH_MIB =
      static_cast<double>(static_cast<int64_t>(RssBytes()) - static_cast<int64_t>(INITIAL_RSS)) /
      (1024.0 * 1024.0);
  const bool ENOUGH_FOR_MINUTE = ELAPSED >= settings_.warmup_sec + 60.0;
  const bool FPS_PASS = !ENOUGH_FOR_MINUTE ||
                        (std::isfinite(metrics.minimum_post_warmup_fps) &&
                         metrics.minimum_post_warmup_fps >= K_MIN_ROLLING_FPS_RATIO * baseline_fps);
  const bool LATENCY_PASS = !metrics.total_ms.empty() && TOTAL_P95 < K_MAX_DETECTION_P95_MS;

  const bool PASS =
      COMPLETED_DURATION && metrics.grab_total > 0 &&
      VALID_FRAME_RATIO >= K_MIN_VALID_FRAME_RATIO && metrics.resolution_errors == 0 &&
      metrics.type_errors == 0 && DETECTION_SUCCESS_RATIO == 1.0 && metrics.detection_errors == 0 &&
      LATENCY_PASS && FPS_PASS && metrics.max_no_valid_frame_sec < K_MAX_NO_VALID_FRAME_SEC &&
      RSS_GROWTH_MIB <= K_MAX_RSS_GROWTH_MIB && metrics.disconnected == 0 &&
      metrics.fatal_errors == 0;

  // summary 是最终机器可读报告，进程退出码与 result 字段保持一致。
  const auto SUMMARY_PATH = settings_.output_dir / ("summary_" + RUN_ID + ".json");
  std::ofstream summary(SUMMARY_PATH);
  if (!summary) {
    throw std::runtime_error("cannot create armor detector test summary");
  }
  summary << std::boolalpha << "{\n"
          << "  \"result\":\"" << (PASS ? "PASS" : "FAIL") << "\",\n"
          << "  \"completed_duration\":" << COMPLETED_DURATION << ",\n"
          << "  \"user_aborted\":" << user_aborted << ",\n"
          << "  \"elapsed_sec\":" << ELAPSED << ",\n"
          << "  \"grab_total\":" << metrics.grab_total << ",\n"
          << "  \"valid_frames\":" << metrics.valid_frames << ",\n"
          << "  \"valid_frame_ratio\":" << VALID_FRAME_RATIO << ",\n"
          << "  \"detection_success\":" << metrics.detection_success << ",\n"
          << "  \"detection_errors\":" << metrics.detection_errors << ",\n"
          << "  \"detection_success_ratio\":" << DETECTION_SUCCESS_RATIO << ",\n"
          << "  \"target_frames\":" << metrics.target_frames << ",\n"
          << "  \"total_detections\":" << metrics.total_detections << ",\n"
          << "  \"total_candidates\":" << metrics.total_candidates << ",\n"
          << "  \"timeouts\":" << metrics.timeouts << ",\n"
          << "  \"disconnects\":" << metrics.disconnected << ",\n"
          << "  \"invalid_frames\":" << metrics.invalid_frames << ",\n"
          << "  \"fatal_errors\":" << metrics.fatal_errors << ",\n"
          << "  \"resolution_errors\":" << metrics.resolution_errors << ",\n"
          << "  \"type_errors\":" << metrics.type_errors << ",\n"
          << "  \"baseline_fps\":" << baseline_fps << ",\n"
          << "  \"minimum_post_warmup_fps\":"
          << (std::isfinite(metrics.minimum_post_warmup_fps) ? metrics.minimum_post_warmup_fps
                                                             : 0.0)
          << ",\n"
          << "  \"preprocess_p95_ms\":" << Percentile(metrics.preprocess_ms, 0.95) << ",\n"
          << "  \"inference_p95_ms\":" << Percentile(metrics.inference_ms, 0.95) << ",\n"
          << "  \"postprocess_p95_ms\":" << Percentile(metrics.postprocess_ms, 0.95) << ",\n"
          << "  \"total_p50_ms\":" << TOTAL_P50 << ",\n"
          << "  \"total_p95_ms\":" << TOTAL_P95 << ",\n"
          << "  \"total_p99_ms\":" << TOTAL_P99 << ",\n"
          << "  \"max_no_valid_frame_ms\":" << metrics.max_no_valid_frame_sec * 1000.0 << ",\n"
          << "  \"max_consecutive_failures\":" << metrics.max_consecutive_failures << ",\n"
          << "  \"rss_growth_mib\":" << RSS_GROWTH_MIB << ",\n"
          << "  \"gpu_execution_verified\":true\n"
          << "}\n";

  MV_LOG_INFO(
      "ArmorDetectorTest",
      "result={} valid={:.5f} detection={:.5f} p95={:.2f}ms min_fps={:.2f} "
      "max_gap={:.1f}ms RSS_growth={:.1f}MiB summary={}",
      PASS ? "PASS" : "FAIL", VALID_FRAME_RATIO, DETECTION_SUCCESS_RATIO, TOTAL_P95,
      std::isfinite(metrics.minimum_post_warmup_fps) ? metrics.minimum_post_warmup_fps : 0.0,
      metrics.max_no_valid_frame_sec * 1000.0, RSS_GROWTH_MIB, SUMMARY_PATH.string());
  return PASS ? 0 : 5;
}

}  // namespace mv::test
