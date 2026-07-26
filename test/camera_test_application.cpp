#include "test/camera_test_application.hpp"

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

double Seconds(Clock::duration duration) {
  return std::chrono::duration<double>(duration).count();
}

double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double position = percentile * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

std::size_t RssBytes() {
  std::ifstream statm("/proc/self/statm");
  std::size_t total_pages = 0;
  std::size_t resident_pages = 0;
  statm >> total_pages >> resident_pages;
  (void)total_pages;
  const long page_size = sysconf(_SC_PAGESIZE);
  return page_size > 0 ? resident_pages * static_cast<std::size_t>(page_size) : 0;
}

uint64_t FrameFingerprint(const cv::Mat& image) {
  constexpr uint64_t OFFSET = 1469598103934665603ULL;
  constexpr uint64_t PRIME = 1099511628211ULL;
  uint64_t hash = OFFSET;
  const int step_y = std::max(1, image.rows / 32);
  const int step_x = std::max(1, image.cols / 32);
  for (int y = 0; y < image.rows; y += step_y) {
    const auto* row = image.ptr<uint8_t>(y);
    for (int x = 0; x < image.cols; x += step_x) {
      for (int channel = 0; channel < image.channels(); ++channel) {
        hash ^= row[x * image.channels() + channel];
        hash *= PRIME;
      }
    }
  }
  return hash;
}

std::string TimeStampForFile() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&raw, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y%m%d_%H%M%S");
  return stream.str();
}

void DrawPreview(cv::Mat& image, const hal::CameraFrame& frame, const hal::CameraInfo& info,
                 hal::GrabStatus status, double fps, double elapsed_sec) {
  const cv::Scalar green(0, 255, 0);
  const cv::Point center(image.cols / 2, image.rows / 2);
  cv::line(image, {center.x - 25, center.y}, {center.x + 25, center.y}, green, 1);
  cv::line(image, {center.x, center.y - 25}, {center.x, center.y + 25}, green, 1);

  const std::vector<std::string> lines = {
      "seq: " + std::to_string(frame.sequence),
      "fps: " + cv::format("%.2f", fps),
      "size: " + std::to_string(image.cols) + "x" + std::to_string(image.rows),
      "exposure: " + std::to_string(info.exposure_us) + " us",
      "grab: " + std::string(hal::GrabStatusName(status)),
      "elapsed: " + cv::format("%.1f s", elapsed_sec),
  };
  int y = 28;
  for (const auto& line : lines) {
    cv::putText(image, line, {16, y}, cv::FONT_HERSHEY_SIMPLEX, 0.62, {0, 0, 0}, 3,
                cv::LINE_AA);
    cv::putText(image, line, {16, y}, cv::FONT_HERSHEY_SIMPLEX, 0.62, green, 1, cv::LINE_AA);
    y += 27;
  }
}

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
  const auto run_id = TimeStampForFile();

  MV_LOG_INFO("CameraTest", "output directory: {}", settings_.output_dir.string());
  if (settings_.restart_cycles > 0) {
    const auto initial_rss = RssBytes();
    int completed_cycles = 0;
    uint64_t frames_grabbed = 0;
    bool restart_pass = true;
    for (int cycle = 1; cycle <= settings_.restart_cycles; ++cycle) {
      if (!camera_->Open(camera_config_)) {
        MV_LOG_ERROR("CameraTest", "restart cycle {}/{}: Open failed", cycle,
                     settings_.restart_cycles);
        restart_pass = false;
        break;
      }
      const auto cycle_info = camera_->Info();
      if (cycle_info.output_width != 1280 || cycle_info.output_height != 720 ||
          cycle_info.pixel_format != hal::PixelFormat::BGR8) {
        MV_LOG_ERROR("CameraTest", "restart cycle {}/{}: invalid CameraInfo", cycle,
                     settings_.restart_cycles);
        restart_pass = false;
      }
      for (int index = 0; restart_pass && index < settings_.frames_per_restart_cycle; ++index) {
        hal::CameraFrame frame;
        const auto status = camera_->Grab(frame);
        if (status != hal::GrabStatus::OK || frame.image.cols != 1280 ||
            frame.image.rows != 720 || frame.image.type() != CV_8UC3) {
          MV_LOG_ERROR("CameraTest", "restart cycle {}/{} frame {} failed: {}", cycle,
                       settings_.restart_cycles, index, hal::GrabStatusName(status));
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

    const double rss_growth_mib =
        static_cast<double>(static_cast<int64_t>(RssBytes()) -
                            static_cast<int64_t>(initial_rss)) /
        (1024.0 * 1024.0);
    restart_pass = restart_pass && completed_cycles == settings_.restart_cycles &&
                   rss_growth_mib <= 20.0;
    const auto restart_summary_path =
        settings_.output_dir / ("restart_summary_" + run_id + ".json");
    std::ofstream restart_summary(restart_summary_path);
    restart_summary << "{\n"
                    << "  \"result\": \"" << (restart_pass ? "PASS" : "FAIL") << "\",\n"
                    << "  \"requested_cycles\": " << settings_.restart_cycles << ",\n"
                    << "  \"completed_cycles\": " << completed_cycles << ",\n"
                    << "  \"frames_per_cycle\": " << settings_.frames_per_restart_cycle << ",\n"
                    << "  \"frames_grabbed\": " << frames_grabbed << ",\n"
                    << "  \"rss_growth_mib\": " << rss_growth_mib << "\n"
                    << "}\n";
    MV_LOG_INFO("CameraTest", "restart result={} cycles={}/{} RSS_growth={:.1f}MiB summary={}",
                restart_pass ? "PASS" : "FAIL", completed_cycles, settings_.restart_cycles,
                rss_growth_mib, restart_summary_path.string());
    return restart_pass ? 0 : 5;
  }

  if (!camera_->Open(camera_config_)) {
    MV_LOG_ERROR("CameraTest", "camera open failed; no fallback or automatic recovery will run");
    return 2;
  }

  const auto info = camera_->Info();
  MV_LOG_INFO("CameraTest",
              "opened '{}' sensor={}x{} output={}x{} ROI=({}, {}) exposure={}us timeout={}ms",
              info.device_name, info.sensor_width, info.sensor_height, info.output_width,
              info.output_height, info.roi_offset_x, info.roi_offset_y, info.exposure_us,
              info.grab_timeout_ms);
  if (info.output_width != 1280 || info.output_height != 720 ||
      info.pixel_format != hal::PixelFormat::BGR8) {
    MV_LOG_ERROR("CameraTest", "HAL reports invalid output; expected hardware 1280x720 BGR8");
    camera_->Close();
    return 3;
  }

  std::unique_ptr<tool::DebugWindow> preview_window;
  if (settings_.preview) {
    preview_window = std::make_unique<tool::DebugWindow>("MiracleVision Camera Test");
  }

  std::ofstream metrics_csv(settings_.output_dir / ("metrics_" + run_id + ".csv"));
  std::ofstream events(settings_.output_dir / ("events_" + run_id + ".jsonl"));
  metrics_csv << "elapsed_sec,total,success,failed,fps,p50_ms,p95_ms,p99_ms,rss_mib,"
                 "cpu_percent,max_consecutive_failures,duplicate_frames\n";

  Metrics metrics;
  const auto start = Clock::now();
  auto report_start = start;
  auto next_report = start + std::chrono::seconds(settings_.report_interval_sec);
  auto next_sample =
      settings_.save_sample_interval_sec > 0
          ? start + std::chrono::seconds(settings_.save_sample_interval_sec)
          : Clock::time_point::max();
  auto last_success_time = Clock::time_point{};
  uint64_t last_fingerprint = 0;
  bool have_fingerprint = false;
  uint64_t report_success = 0;
  uint64_t warmup_success = 0;
  std::deque<Clock::time_point> post_warmup_success_times;
  double baseline_fps = 0.0;
  const auto initial_rss = RssBytes();
  const auto initial_cpu = std::clock();
  hal::CameraFrame last_good_frame;
  hal::GrabStatus last_status = hal::GrabStatus::OK;

  while (true) {
    const auto before_grab = Clock::now();
    if (Seconds(before_grab - start) >= settings_.duration_sec) break;

    hal::CameraFrame frame;
    last_status = camera_->Grab(frame);
    const auto now = Clock::now();
    ++metrics.total;

    if (last_status == hal::GrabStatus::OK) {
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
        if (Seconds(now - start) <= settings_.warmup_sec) {
          ++warmup_success;
        } else {
          post_warmup_success_times.push_back(now);
        }

        if (last_success_time != Clock::time_point{}) {
          const double interval = Seconds(frame.timestamp - last_success_time);
          if (interval > 0.0) metrics.intervals_sec.push_back(interval);
          metrics.max_no_valid_frame_sec = std::max(metrics.max_no_valid_frame_sec, interval);
        }
        last_success_time = frame.timestamp;

        const uint64_t fingerprint = FrameFingerprint(frame.image);
        if (have_fingerprint && fingerprint == last_fingerprint) {
          ++metrics.duplicate_frames;
          ++metrics.consecutive_duplicates;
          metrics.max_consecutive_duplicates =
              std::max(metrics.max_consecutive_duplicates, metrics.consecutive_duplicates);
        } else {
          metrics.consecutive_duplicates = 0;
        }
        last_fingerprint = fingerprint;
        have_fingerprint = true;
        last_good_frame = frame;

        if (now >= next_sample) {
          const auto path = settings_.output_dir /
                            ("sample_" + run_id + "_" + std::to_string(frame.sequence) + ".jpg");
          if (!cv::imwrite(path.string(), frame.image)) {
            MV_LOG_WARN("CameraTest", "failed to save sample {}", path.string());
          }
          next_sample += std::chrono::seconds(settings_.save_sample_interval_sec);
        }
      }
    } else {
      CountFailure(metrics, last_status);
      if (last_success_time != Clock::time_point{}) {
        metrics.max_no_valid_frame_sec =
            std::max(metrics.max_no_valid_frame_sec, Seconds(now - last_success_time));
      }
      events << "{\"elapsed_sec\":" << Seconds(now - start) << ",\"event\":\"grab_failure\","
             << "\"status\":\"" << hal::GrabStatusName(last_status) << "\",\"last_sequence\":"
             << last_good_frame.sequence << "}\n";
      MV_LOG_WARN("CameraTest", "grab {} at total={} last_sequence={}",
                  hal::GrabStatusName(last_status), metrics.total, last_good_frame.sequence);
    }

    if (preview_window) {
      if (!last_good_frame.image.empty()) {
        cv::Mat preview = last_good_frame.image.clone();
        const double report_elapsed = std::max(Seconds(now - report_start), 1e-6);
        DrawPreview(preview, last_good_frame, info, last_status,
                    static_cast<double>(report_success) / report_elapsed, Seconds(now - start));
        preview_window->Show(preview);
      }
      if (preview_window->Poll().exit_requested) break;
    }

    if (now >= next_report) {
      const double elapsed = Seconds(now - start);
      const double report_elapsed = std::max(Seconds(now - report_start), 1e-6);
      const double fps = static_cast<double>(report_success) / report_elapsed;
      const auto minute_start = now - std::chrono::seconds(60);
      while (!post_warmup_success_times.empty() &&
             post_warmup_success_times.front() < minute_start) {
        post_warmup_success_times.pop_front();
      }
      if (elapsed >= settings_.warmup_sec + 60.0) {
        const double minute_fps =
            static_cast<double>(post_warmup_success_times.size()) / 60.0;
        metrics.minimum_post_warmup_fps =
            std::min(metrics.minimum_post_warmup_fps, minute_fps);
      }
      if (baseline_fps == 0.0 && elapsed >= settings_.warmup_sec) {
        baseline_fps = settings_.warmup_sec > 0
                           ? static_cast<double>(warmup_success) / settings_.warmup_sec
                           : fps;
        MV_LOG_INFO("CameraTest", "baseline F0 established: {:.3f} FPS", baseline_fps);
      }

      const double p50 = Percentile(metrics.intervals_sec, 0.50) * 1000.0;
      const double p95 = Percentile(metrics.intervals_sec, 0.95) * 1000.0;
      const double p99 = Percentile(metrics.intervals_sec, 0.99) * 1000.0;
      const double cpu =
          100.0 * static_cast<double>(std::clock() - initial_cpu) / CLOCKS_PER_SEC /
          std::max(elapsed, 1e-6);
      const double rss_mib = static_cast<double>(RssBytes()) / (1024.0 * 1024.0);
      metrics_csv << elapsed << ',' << metrics.total << ',' << metrics.success << ','
                  << (metrics.total - metrics.success) << ',' << fps << ',' << p50 << ',' << p95
                  << ',' << p99 << ',' << rss_mib << ',' << cpu << ','
                  << metrics.max_consecutive_failures << ',' << metrics.duplicate_frames << '\n';
      metrics_csv.flush();
      MV_LOG_INFO("CameraTest",
                  "t={:.0f}s success={}/{} fps={:.2f} interval p50/p95/p99={:.2f}/{:.2f}/{:.2f}ms "
                  "RSS={:.1f}MiB CPU={:.1f}%",
                  elapsed, metrics.success, metrics.total, fps, p50, p95, p99, rss_mib, cpu);
      report_success = 0;
      report_start = now;
      next_report += std::chrono::seconds(settings_.report_interval_sec);
    }

    if (last_status == hal::GrabStatus::DISCONNECTED ||
        last_status == hal::GrabStatus::FATAL) {
      MV_LOG_ERROR("CameraTest", "stopping after {} (last successful sequence={})",
                   hal::GrabStatusName(last_status), last_good_frame.sequence);
      break;
    }
  }

  camera_->Close();

  const auto finish = Clock::now();
  const double elapsed = Seconds(finish - start);
  const double success_ratio =
      metrics.total > 0 ? static_cast<double>(metrics.success) / metrics.total : 0.0;
  const double rss_growth_mib =
      static_cast<double>(static_cast<int64_t>(RssBytes()) - static_cast<int64_t>(initial_rss)) /
      (1024.0 * 1024.0);
  const double p99 = Percentile(metrics.intervals_sec, 0.99);
  const bool enough_for_baseline = elapsed >= settings_.warmup_sec && baseline_fps > 0.0;
  const bool enough_for_minute = elapsed >= settings_.warmup_sec + 60.0;
  const bool fps_pass =
      !enough_for_minute ||
      (std::isfinite(metrics.minimum_post_warmup_fps) &&
       metrics.minimum_post_warmup_fps >= 0.95 * baseline_fps);
  const bool p99_pass = !enough_for_baseline || p99 <= 2.0 / baseline_fps;
  const bool pass = metrics.total > 0 && success_ratio >= 0.999 &&
                    metrics.resolution_errors == 0 && metrics.type_errors == 0 && fps_pass &&
                    p99_pass && metrics.max_no_valid_frame_sec < 0.5 && rss_growth_mib <= 20.0 &&
                    metrics.disconnected == 0 && metrics.fatal == 0;

  const auto summary_path = settings_.output_dir / ("summary_" + run_id + ".json");
  std::ofstream summary(summary_path);
  summary << std::boolalpha << "{\n"
          << "  \"result\": \"" << (pass ? "PASS" : "FAIL") << "\",\n"
          << "  \"elapsed_sec\": " << elapsed << ",\n"
          << "  \"total_frames\": " << metrics.total << ",\n"
          << "  \"successful_frames\": " << metrics.success << ",\n"
          << "  \"effective_frame_ratio\": " << success_ratio << ",\n"
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
          << "  \"frame_interval_p99_ms\": " << p99 * 1000.0 << ",\n"
          << "  \"max_no_valid_frame_ms\": " << metrics.max_no_valid_frame_sec * 1000.0 << ",\n"
          << "  \"max_consecutive_failures\": " << metrics.max_consecutive_failures << ",\n"
          << "  \"duplicate_frames\": " << metrics.duplicate_frames << ",\n"
          << "  \"max_consecutive_duplicates\": " << metrics.max_consecutive_duplicates << ",\n"
          << "  \"rss_growth_mib\": " << rss_growth_mib << "\n"
          << "}\n";

  MV_LOG_INFO("CameraTest",
              "result={} effective={:.5f} p99={:.2f}ms max_gap={:.1f}ms RSS_growth={:.1f}MiB "
              "summary={}",
              pass ? "PASS" : "FAIL", success_ratio, p99 * 1000.0,
              metrics.max_no_valid_frame_sec * 1000.0, rss_growth_mib, summary_path.string());
  return pass ? 0 : 4;
}

}  // namespace mv::test
