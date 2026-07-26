#include "test/camera/camera_test_application.hpp"

#include "core/logger.hpp"
#include "tool/debug/debug_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <unistd.h>

namespace mv::test {
namespace {

using Clock = std::chrono::steady_clock;

// 汇总长时测试中的抓帧质量、时序、连续故障和资源占用指标。
struct Metrics {
  uint64_t total{0};
  uint64_t success{0};
  uint64_t timeout{0};
  uint64_t disconnected{0};
  uint64_t invalid_frame{0};
  uint64_t fatal{0};
  uint64_t resolution_errors{0};
  uint64_t type_errors{0};
  uint64_t duplicate_frames{0};
  uint64_t max_consecutive_duplicates{0};
  uint64_t consecutive_duplicates{0};
  uint64_t consecutive_failures{0};
  uint64_t max_consecutive_failures{0};
  double max_no_valid_frame_sec{0.0};
  double minimum_post_warmup_fps{std::numeric_limits<double>::infinity()};
  std::vector<double> intervals_sec;
};

// 将单调时钟的时间段统一换算为秒。
double Seconds(Clock::duration duration) {
  return std::chrono::duration<double>(duration).count();
}

// 对帧间隔副本排序并进行线性插值，原始采样顺序保持不变。
double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) return 0.0;
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

// 稀疏采样图像并计算指纹，以较低开销发现连续重复帧。
uint64_t FrameFingerprint(const cv::Mat& image) {
  constexpr uint64_t OFFSET = 1469598103934665603ULL;
  constexpr uint64_t PRIME = 1099511628211ULL;
  uint64_t hash = OFFSET;
  const int STEP_Y = std::max(1, image.rows / 32);
  const int STEP_X = std::max(1, image.cols / 32);
  for (int y = 0; y < image.rows; y += STEP_Y) {
    const auto* row = image.ptr<uint8_t>(y);
    for (int x = 0; x < image.cols; x += STEP_X) {
      for (int channel = 0; channel < image.channels(); ++channel) {
        hash ^= row[x * image.channels() + channel];
        hash *= PRIME;
      }
    }
  }
  return hash;
}

// 生成适合作为同一次测试所有产物前缀的本地时间戳。
std::string TimeStampForFile() {
  const auto NOW = std::chrono::system_clock::now();
  const std::time_t RAW = std::chrono::system_clock::to_time_t(NOW);
  std::tm local{};
  localtime_r(&RAW, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y%m%d_%H%M%S");
  return stream.str();
}

// 在最后一帧有效图像上叠加中心标记和实时抓帧状态。
void DrawPreview(cv::Mat& image, const hal::CameraFrame& frame, const hal::CameraInfo& info,
                 hal::GrabStatus status, double fps, double elapsed_sec) {
  const cv::Scalar GREEN(0, 255, 0);
  const cv::Point CENTER(image.cols / 2, image.rows / 2);
  cv::line(image, {CENTER.x - 25, CENTER.y}, {CENTER.x + 25, CENTER.y}, GREEN, 1);
  cv::line(image, {CENTER.x, CENTER.y - 25}, {CENTER.x, CENTER.y + 25}, GREEN, 1);

  const std::vector<std::string> LINES = {
      "seq: " + std::to_string(frame.sequence),
      "fps: " + cv::format("%.2f", fps),
      "size: " + std::to_string(image.cols) + "x" + std::to_string(image.rows),
      "exposure: " + std::to_string(info.exposure_us) + " us",
      "grab: " + std::string(hal::GrabStatusName(status)),
      "elapsed: " + cv::format("%.1f s", elapsed_sec),
  };
  int y = 28;
  for (const auto& line : LINES) {
    cv::putText(image, line, {16, y}, cv::FONT_HERSHEY_SIMPLEX, 0.62, {0, 0, 0}, 3,
                cv::LINE_AA);
    cv::putText(image, line, {16, y}, cv::FONT_HERSHEY_SIMPLEX, 0.62, GREEN, 1, cv::LINE_AA);
    y += 27;
  }
}

// 按 GrabStatus 分类失败，并维护连续失败次数的峰值。
void CountFailure(Metrics& metrics, hal::GrabStatus status) {
  ++metrics.consecutive_failures;
  metrics.max_consecutive_failures =
      std::max(metrics.max_consecutive_failures, metrics.consecutive_failures);
  switch (status) {
    case hal::GrabStatus::TIMEOUT:
      ++metrics.timeout;
      break;
    case hal::GrabStatus::DISCONNECTED:
      ++metrics.disconnected;
      break;
    case hal::GrabStatus::INVALID_FRAME:
      ++metrics.invalid_frame;
      break;
    case hal::GrabStatus::FATAL:
      ++metrics.fatal;
      break;
    case hal::GrabStatus::OK:
      break;
  }
}

}  // namespace

CameraTestApplication::CameraTestApplication(std::unique_ptr<hal::ICamera> camera,
                                             YAML::Node camera_config,
                                             CameraTestSettings settings)
    : camera_(std::move(camera)),
      camera_config_(std::move(camera_config)),
      settings_(std::move(settings)) {}

CameraTestApplication::~CameraTestApplication() = default;

int CameraTestApplication::Run() {
  std::filesystem::create_directories(settings_.output_dir);
  const auto RUN_ID = TimeStampForFile();

  MV_LOG_INFO("CameraTest", "output directory: {}", settings_.output_dir.string());

  // restart_cycles 非 0 时进入独立的重复启停测试，不再执行后续长时测试。
  if (settings_.restart_cycles > 0) {
    const auto INITIAL_RSS = RssBytes();
    int completed_cycles = 0;
    uint64_t frames_grabbed = 0;
    bool restart_pass = true;
    for (int cycle = 1; cycle <= settings_.restart_cycles; ++cycle) {
      // 每个周期都完整执行 Open -> Grab N 帧 -> Close，检查资源能否反复释放。
      if (!camera_->Open(camera_config_)) {
        MV_LOG_ERROR("CameraTest", "restart cycle {}/{}: Open failed", cycle,
                     settings_.restart_cycles);
        restart_pass = false;
        break;
      }
      const auto CYCLE_INFO = camera_->Info();
      if (CYCLE_INFO.output_width != 1280 || CYCLE_INFO.output_height != 720 ||
          CYCLE_INFO.pixel_format != hal::PixelFormat::BGR8) {
        MV_LOG_ERROR("CameraTest", "restart cycle {}/{}: invalid CameraInfo", cycle,
                     settings_.restart_cycles);
        restart_pass = false;
      }
      for (int index = 0; restart_pass && index < settings_.frames_per_restart_cycle; ++index) {
        hal::CameraFrame frame;
        const auto STATUS = camera_->Grab(frame);
        if (STATUS != hal::GrabStatus::OK || frame.image.cols != 1280 ||
            frame.image.rows != 720 || frame.image.type() != CV_8UC3) {
          MV_LOG_ERROR("CameraTest", "restart cycle {}/{} frame {} failed: {}", cycle,
                       settings_.restart_cycles, index, hal::GrabStatusName(STATUS));
          restart_pass = false;
          break;
        }
        ++frames_grabbed;
      }
      camera_->Close();
      if (!restart_pass) break;
      ++completed_cycles;
      MV_LOG_INFO("CameraTest", "restart cycle {}/{} PASS ({} frames)", cycle,
                  settings_.restart_cycles, settings_.frames_per_restart_cycle);
    }

    // 所有周期完成且 RSS 增长不超过 20 MiB，重复启停测试才通过。
    const double RSS_GROWTH_MIB =
        static_cast<double>(static_cast<int64_t>(RssBytes()) -
                            static_cast<int64_t>(INITIAL_RSS)) /
        (1024.0 * 1024.0);
    restart_pass = restart_pass && completed_cycles == settings_.restart_cycles &&
                   RSS_GROWTH_MIB <= 20.0;
    const auto RESTART_SUMMARY_PATH =
        settings_.output_dir / ("restart_summary_" + RUN_ID + ".json");
    std::ofstream restart_summary(RESTART_SUMMARY_PATH);
    restart_summary << "{\n"
                    << "  \"result\": \"" << (restart_pass ? "PASS" : "FAIL") << "\",\n"
                    << "  \"requested_cycles\": " << settings_.restart_cycles << ",\n"
                    << "  \"completed_cycles\": " << completed_cycles << ",\n"
                    << "  \"frames_per_cycle\": " << settings_.frames_per_restart_cycle << ",\n"
                    << "  \"frames_grabbed\": " << frames_grabbed << ",\n"
                    << "  \"rss_growth_mib\": " << RSS_GROWTH_MIB << "\n"
                    << "}\n";
    MV_LOG_INFO("CameraTest", "restart result={} cycles={}/{} RSS_growth={:.1f}MiB summary={}",
                restart_pass ? "PASS" : "FAIL", completed_cycles, settings_.restart_cycles,
                RSS_GROWTH_MIB, RESTART_SUMMARY_PATH.string());
    return restart_pass ? 0 : 5;
  }

  // 长时测试先打开相机，再从 HAL 返回的信息确认硬件输出格式。
  if (!camera_->Open(camera_config_)) {
    MV_LOG_ERROR("CameraTest", "camera open failed; no fallback or automatic recovery will run");
    return 2;
  }

  const auto INFO = camera_->Info();
  MV_LOG_INFO("CameraTest",
              "opened '{}' sensor={}x{} output={}x{} ROI=({}, {}) exposure={}us timeout={}ms",
              INFO.device_name, INFO.sensor_width, INFO.sensor_height, INFO.output_width,
              INFO.output_height, INFO.roi_offset_x, INFO.roi_offset_y, INFO.exposure_us,
              INFO.grab_timeout_ms);
  if (INFO.output_width != 1280 || INFO.output_height != 720 ||
      INFO.pixel_format != hal::PixelFormat::BGR8) {
    MV_LOG_ERROR("CameraTest", "HAL reports invalid output; expected hardware 1280x720 BGR8");
    camera_->Close();
    return 3;
  }

  // 预览窗口是可选诊断输出，不参与最终 PASS/FAIL 判定。
  std::unique_ptr<tool::DebugWindow> preview_window;
  if (settings_.preview) {
    preview_window = std::make_unique<tool::DebugWindow>("MiracleVision Camera Test");
  }

  // CSV 记录周期指标，JSONL 逐条记录抓帧故障，最终结果另写入 summary JSON。
  std::ofstream metrics_csv(settings_.output_dir / ("metrics_" + RUN_ID + ".csv"));
  std::ofstream events(settings_.output_dir / ("events_" + RUN_ID + ".jsonl"));
  metrics_csv << "elapsed_sec,total,success,failed,fps,p50_ms,p95_ms,p99_ms,rss_mib,"
                 "cpu_percent,max_consecutive_failures,duplicate_frames\n";

  Metrics metrics;
  const auto START = Clock::now();
  auto report_start = START;
  auto next_report = START + std::chrono::seconds(settings_.report_interval_sec);
  auto next_sample =
      settings_.save_sample_interval_sec > 0
          ? START + std::chrono::seconds(settings_.save_sample_interval_sec)
          : Clock::time_point::max();
  auto last_success_time = Clock::time_point{};
  uint64_t last_fingerprint = 0;
  bool have_fingerprint = false;
  uint64_t report_success = 0;
  uint64_t warmup_success = 0;
  std::deque<Clock::time_point> post_warmup_success_times;
  double baseline_fps = 0.0;
  const auto INITIAL_RSS = RssBytes();
  const auto INITIAL_CPU = std::clock();
  hal::CameraFrame last_good_frame;
  hal::GrabStatus last_status = hal::GrabStatus::OK;

  // 主循环持续抓帧，直到达到配置时长、用户关闭预览或遇到不可恢复错误。
  while (true) {
    const auto BEFORE_GRAB = Clock::now();
    if (Seconds(BEFORE_GRAB - START) >= settings_.duration_sec) break;

    hal::CameraFrame frame;
    last_status = camera_->Grab(frame);
    const auto NOW = Clock::now();
    ++metrics.total;

    if (last_status == hal::GrabStatus::OK) {
      // SDK 返回成功后仍需检查图像尺寸和类型，异常帧按 INVALID_FRAME 统计。
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
        last_status = hal::GrabStatus::INVALID_FRAME;
        CountFailure(metrics, last_status);
      } else {
        ++metrics.success;
        ++report_success;
        metrics.consecutive_failures = 0;
        if (Seconds(NOW - START) <= settings_.warmup_sec) {
          ++warmup_success;
        } else {
          post_warmup_success_times.push_back(NOW);
        }

        if (last_success_time != Clock::time_point{}) {
          const double INTERVAL = Seconds(frame.timestamp - last_success_time);
          if (INTERVAL > 0.0) metrics.intervals_sec.push_back(INTERVAL);
          metrics.max_no_valid_frame_sec = std::max(metrics.max_no_valid_frame_sec, INTERVAL);
        }
        last_success_time = frame.timestamp;

        // 指纹只用于诊断重复帧，不直接作为本次验收的失败条件。
        const uint64_t FINGERPRINT = FrameFingerprint(frame.image);
        if (have_fingerprint && FINGERPRINT == last_fingerprint) {
          ++metrics.duplicate_frames;
          ++metrics.consecutive_duplicates;
          metrics.max_consecutive_duplicates =
              std::max(metrics.max_consecutive_duplicates, metrics.consecutive_duplicates);
        } else {
          metrics.consecutive_duplicates = 0;
        }
        last_fingerprint = FINGERPRINT;
        have_fingerprint = true;
        last_good_frame = frame;

        // 样本按成功帧保存；间隔为 0 时 next_sample 被设为无穷远。
        if (NOW >= next_sample) {
          const auto PATH = settings_.output_dir /
                            ("sample_" + RUN_ID + "_" + std::to_string(frame.sequence) + ".jpg");
          if (!cv::imwrite(PATH.string(), frame.image)) {
            MV_LOG_WARN("CameraTest", "failed to save sample {}", PATH.string());
          }
          next_sample += std::chrono::seconds(settings_.save_sample_interval_sec);
        }
      }
    } else {
      CountFailure(metrics, last_status);
      if (last_success_time != Clock::time_point{}) {
        metrics.max_no_valid_frame_sec =
            std::max(metrics.max_no_valid_frame_sec, Seconds(NOW - last_success_time));
      }
      events << "{\"elapsed_sec\":" << Seconds(NOW - START) << ",\"event\":\"grab_failure\","
             << "\"status\":\"" << hal::GrabStatusName(last_status) << "\",\"last_sequence\":"
             << last_good_frame.sequence << "}\n";
      MV_LOG_WARN("CameraTest", "grab {} at total={} last_sequence={}",
                  hal::GrabStatusName(last_status), metrics.total, last_good_frame.sequence);
    }

    // 抓帧失败时继续显示最后一帧有效图像，并在 HUD 上呈现最新状态。
    if (preview_window) {
      if (!last_good_frame.image.empty()) {
        cv::Mat preview = last_good_frame.image.clone();
        const double REPORT_ELAPSED = std::max(Seconds(NOW - report_start), 1e-6);
        DrawPreview(preview, last_good_frame, INFO, last_status,
                    static_cast<double>(report_success) / REPORT_ELAPSED, Seconds(NOW - START));
        preview_window->Show(preview);
      }
      if (preview_window->Poll().exit_requested) break;
    }

    // 每个报告周期更新滚动一分钟 FPS、帧间隔分位数及进程资源占用。
    if (NOW >= next_report) {
      const double ELAPSED = Seconds(NOW - START);
      const double REPORT_ELAPSED = std::max(Seconds(NOW - report_start), 1e-6);
      const double FPS = static_cast<double>(report_success) / REPORT_ELAPSED;
      const auto MINUTE_START = NOW - std::chrono::seconds(60);
      while (!post_warmup_success_times.empty() &&
             post_warmup_success_times.front() < MINUTE_START) {
        post_warmup_success_times.pop_front();
      }
      if (ELAPSED >= settings_.warmup_sec + 60.0) {
        const double MINUTE_FPS =
            static_cast<double>(post_warmup_success_times.size()) / 60.0;
        metrics.minimum_post_warmup_fps =
            std::min(metrics.minimum_post_warmup_fps, MINUTE_FPS);
      }
      if (baseline_fps == 0.0 && ELAPSED >= settings_.warmup_sec) {
        baseline_fps = settings_.warmup_sec > 0
                           ? static_cast<double>(warmup_success) / settings_.warmup_sec
                           : FPS;
        MV_LOG_INFO("CameraTest", "baseline F0 established: {:.3f} FPS", baseline_fps);
      }

      const double P50 = Percentile(metrics.intervals_sec, 0.50) * 1000.0;
      const double P95 = Percentile(metrics.intervals_sec, 0.95) * 1000.0;
      const double P99 = Percentile(metrics.intervals_sec, 0.99) * 1000.0;
      const double CPU =
          100.0 * static_cast<double>(std::clock() - INITIAL_CPU) / CLOCKS_PER_SEC /
          std::max(ELAPSED, 1e-6);
      const double RSS_MIB = static_cast<double>(RssBytes()) / (1024.0 * 1024.0);
      metrics_csv << ELAPSED << ',' << metrics.total << ',' << metrics.success << ','
                  << (metrics.total - metrics.success) << ',' << FPS << ',' << P50 << ',' << P95
                  << ',' << P99 << ',' << RSS_MIB << ',' << CPU << ','
                  << metrics.max_consecutive_failures << ',' << metrics.duplicate_frames << '\n';
      metrics_csv.flush();
      MV_LOG_INFO("CameraTest",
                  "t={:.0f}s success={}/{} fps={:.2f} interval p50/p95/p99={:.2f}/{:.2f}/{:.2f}ms "
                  "RSS={:.1f}MiB CPU={:.1f}%",
                  ELAPSED, metrics.success, metrics.total, FPS, P50, P95, P99, RSS_MIB, CPU);
      report_success = 0;
      report_start = NOW;
      next_report += std::chrono::seconds(settings_.report_interval_sec);
    }

    // TIMEOUT 和 INVALID_FRAME 可继续统计；断开或致命错误立即结束主循环。
    if (last_status == hal::GrabStatus::DISCONNECTED ||
        last_status == hal::GrabStatus::FATAL) {
      MV_LOG_ERROR("CameraTest", "stopping after {} (last successful sequence={})",
                   hal::GrabStatusName(last_status), last_good_frame.sequence);
      break;
    }
  }

  camera_->Close();

  const auto FINISH = Clock::now();
  const double ELAPSED = Seconds(FINISH - START);
  const double SUCCESS_RATIO =
      metrics.total > 0 ? static_cast<double>(metrics.success) / metrics.total : 0.0;
  const double RSS_GROWTH_MIB =
      static_cast<double>(static_cast<int64_t>(RssBytes()) - static_cast<int64_t>(INITIAL_RSS)) /
      (1024.0 * 1024.0);
  const double P99 = Percentile(metrics.intervals_sec, 0.99);
  const bool ENOUGH_FOR_BASELINE = ELAPSED >= settings_.warmup_sec && baseline_fps > 0.0;
  const bool ENOUGH_FOR_MINUTE = ELAPSED >= settings_.warmup_sec + 60.0;
  const bool FPS_PASS =
      !ENOUGH_FOR_MINUTE ||
      (std::isfinite(metrics.minimum_post_warmup_fps) &&
       metrics.minimum_post_warmup_fps >= 0.95 * baseline_fps);
  const bool P99_PASS = !ENOUGH_FOR_BASELINE || P99 <= 2.0 / baseline_fps;

  // 汇总配置文档约定的有效率、FPS、时延、连续空窗、内存和连接状态门槛。
  const bool PASS = metrics.total > 0 && SUCCESS_RATIO >= 0.999 &&
                    metrics.resolution_errors == 0 && metrics.type_errors == 0 && FPS_PASS &&
                    P99_PASS && metrics.max_no_valid_frame_sec < 0.5 && RSS_GROWTH_MIB <= 20.0 &&
                    metrics.disconnected == 0 && metrics.fatal == 0;

  // summary 是长时测试的最终机器可读结果，进程退出码与 result 字段保持一致。
  const auto SUMMARY_PATH = settings_.output_dir / ("summary_" + RUN_ID + ".json");
  std::ofstream summary(SUMMARY_PATH);
  summary << std::boolalpha << "{\n"
          << "  \"result\": \"" << (PASS ? "PASS" : "FAIL") << "\",\n"
          << "  \"elapsed_sec\": " << ELAPSED << ",\n"
          << "  \"total_frames\": " << metrics.total << ",\n"
          << "  \"successful_frames\": " << metrics.success << ",\n"
          << "  \"effective_frame_ratio\": " << SUCCESS_RATIO << ",\n"
          << "  \"resolution_errors\": " << metrics.resolution_errors << ",\n"
          << "  \"type_errors\": " << metrics.type_errors << ",\n"
          << "  \"timeouts\": " << metrics.timeout << ",\n"
          << "  \"disconnects\": " << metrics.disconnected << ",\n"
          << "  \"fatal_errors\": " << metrics.fatal << ",\n"
          << "  \"baseline_fps\": " << baseline_fps << ",\n"
          << "  \"minimum_post_warmup_fps\": "
          << (std::isfinite(metrics.minimum_post_warmup_fps)
                  ? metrics.minimum_post_warmup_fps
                  : 0.0)
          << ",\n"
          << "  \"frame_interval_p50_ms\": " << Percentile(metrics.intervals_sec, 0.50) * 1000.0
          << ",\n"
          << "  \"frame_interval_p95_ms\": " << Percentile(metrics.intervals_sec, 0.95) * 1000.0
          << ",\n"
          << "  \"frame_interval_p99_ms\": " << P99 * 1000.0 << ",\n"
          << "  \"max_no_valid_frame_ms\": " << metrics.max_no_valid_frame_sec * 1000.0 << ",\n"
          << "  \"max_consecutive_failures\": " << metrics.max_consecutive_failures << ",\n"
          << "  \"duplicate_frames\": " << metrics.duplicate_frames << ",\n"
          << "  \"max_consecutive_duplicates\": " << metrics.max_consecutive_duplicates << ",\n"
          << "  \"rss_growth_mib\": " << RSS_GROWTH_MIB << "\n"
          << "}\n";

  MV_LOG_INFO("CameraTest",
              "result={} effective={:.5f} p99={:.2f}ms max_gap={:.1f}ms RSS_growth={:.1f}MiB "
              "summary={}",
              PASS ? "PASS" : "FAIL", SUCCESS_RATIO, P99 * 1000.0,
              metrics.max_no_valid_frame_sec * 1000.0, RSS_GROWTH_MIB, SUMMARY_PATH.string());
  return PASS ? 0 : 4;
}

}  // namespace mv::test
