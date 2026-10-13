#include "core/config.hpp"
#include "hal/serial/controller_link.hpp"
#include "imu_capture_support.hpp"
#include "runtime/real_geometry_config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <filesystem>
#include <numbers>
#include <poll.h>
#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;
using Path = std::filesystem::path;
constexpr double DEG = 180.0 / std::numbers::pi;
volatile std::sig_atomic_t stopped = 0;
void Stop(int) {
  stopped = 1;
}

std::string Id() {
  return std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count());
}
void Save(const Path& path, const YAML::Node& value) {
  mv::test::imu::SaveMetadata(path, value);
}
Path NewDirectory(const Path& path) {
  std::filesystem::create_directories(path.parent_path());
  if (!std::filesystem::create_directory(path))
    throw std::runtime_error("directory already exists: " + path.string());
  return path;
}

/** @brief Euler ZYX in degrees; the world vertical is the yaw axis. */
std::array<double, 3> Angles(const mv::geometry::Quaternion& q) {
  const Eigen::Matrix3d r = q.toRotationMatrix();
  return {std::atan2(r(1, 0), r(0, 0)) * DEG,
          std::atan2(-r(2, 0), std::hypot(r(0, 0), r(1, 0))) * DEG,
          std::atan2(r(2, 1), r(2, 2)) * DEG};
}

/** @brief Raw wire values are retained even if rejected; counters are session-local. */
struct Row {
  std::uint64_t segment{}, generation{}, mcu{}, packet{}, sample{}, flags{}, crc{};
  std::uint64_t packet_skip{}, sample_skip{};
  double host{}, offset{}, rtt{};
  bool synced{}, accepted{};
  std::array<double, 7> values{};
  std::string reason;
};
constexpr const char* HEADER =
    "segment,generation,host_receive_us,mcu_us,packet_sequence,sample_sequence,flags,"
    "synchronized,accepted,qw,qx,qy,qz,wx,wy,wz,host_minus_mcu_us,rtt_us,crc_errors,"
    "packet_sequence_skip,sample_sequence_skip,reason";
void WriteRow(std::ostream& out, const Row& r) {
  out << r.segment << ',' << r.generation << ',' << r.host << ',' << r.mcu << ',' << r.packet << ','
      << r.sample << ',' << r.flags << ',' << r.synced << ',' << r.accepted;
  for (double x : r.values)
    out << ',' << x;
  out << ',' << r.offset << ',' << r.rtt << ',' << r.crc << ',' << r.packet_skip << ','
      << r.sample_skip << ',' << r.reason << '\n';
}
Row ReadRow(const std::string& line) {
  std::vector<std::string> cells;
  std::istringstream input(line);
  std::string cell;
  while (std::getline(input, cell, ','))
    cells.push_back(cell);
  if (cells.size() != 22)
    throw std::runtime_error("invalid samples.csv row");
  Row r;
  r.segment = std::stoull(cells[0]);
  r.generation = std::stoull(cells[1]);
  r.host = std::stod(cells[2]);
  r.mcu = std::stoull(cells[3]);
  r.packet = std::stoull(cells[4]);
  r.sample = std::stoull(cells[5]);
  r.flags = std::stoull(cells[6]);
  r.synced = std::stoi(cells[7]) != 0;
  r.accepted = std::stoi(cells[8]) != 0;
  for (std::size_t i = 0; i < 7; ++i)
    r.values[i] = std::stod(cells[9 + i]);
  r.offset = std::stod(cells[16]);
  r.rtt = std::stod(cells[17]);
  r.crc = std::stoull(cells[18]);
  r.packet_skip = std::stoull(cells[19]);
  r.sample_skip = std::stoull(cells[20]);
  r.reason = cells[21];
  return r;
}

/** @brief Callback and UI share this mutex; callbacks never throw into the serial worker. */
struct Capture {
  std::mutex mutex;
  std::ofstream out;
  mv::test::imu::ProtocolDiagnostics diagnostics;
  mv::hal::serial::ControllerLinkState health;
  mv::geometry::GimbalExtrinsics extrinsics;
  std::optional<Clock::time_point> first, last_valid;
  std::optional<Row> previous;
  std::optional<std::uint64_t> generation, last_accepted_mcu;
  std::uint64_t segment{}, rows{}, accepted{};
  bool split_pending{true}, failed{false};
  std::array<double, 3> current{};
  std::array<double, 3> origin{}, previous_angles{}, unwrapped{};

  bool Update(const mv::hal::serial::ControllerLinkState& state,
              const std::optional<mv::hal::serial::ControllerFrame>& frame) noexcept {
    try {
      std::lock_guard lock(mutex);
      health = state;
      if (!generation || *generation != state.generation) {
        generation = state.generation;
        previous.reset();
        last_accepted_mcu.reset();
        split_pending = true;
      }
      if (!frame || (frame->type != AV_ATTITUDE && frame->type != AV_CALIBRATION_SAMPLE))
        return true;
      const auto PAYLOAD = mv::hal::serial::DecodeAttitudePayload(*frame);
      if (!PAYLOAD) {
        diagnostics.Record(state, *frame, PAYLOAD, "invalid_payload_length");
        return true;
      }
      Row r;
      r.generation = state.generation;
      r.host = frame->receive_time_us;
      r.mcu = PAYLOAD->mcu_us;
      r.sample = PAYLOAD->sample_sequence;
      r.flags = PAYLOAD->status;
      r.packet = frame->sequence;
      r.synced = state.synchronized;
      r.offset = state.offset_us;
      r.rtt = state.rtt_us;
      r.crc = state.crc_errors;
      r.values = PAYLOAD->values;
      const char* failure = mv::hal::serial::CalibrationStructureFailure(*frame);
      if (!failure)
        failure = mv::hal::serial::AttitudeValueFailure(*PAYLOAD);
      const bool rollback = previous && (r.mcu < previous->mcu || r.sample < previous->sample ||
                                         r.packet < previous->packet);
      const bool duplicate = previous && (r.mcu == previous->mcu || r.sample == previous->sample ||
                                          r.packet == previous->packet);
      if (previous && !rollback) {
        if (r.packet > previous->packet)
          r.packet_skip = r.packet - previous->packet - 1;
        if (r.sample > previous->sample)
          r.sample_skip = r.sample - previous->sample - 1;
      }
      if (rollback) {
        split_pending = true;
        previous.reset();
      }
      const auto now = Clock::now();
      if (last_valid && now - *last_valid > std::chrono::seconds(1))
        split_pending = true;
      if (last_accepted_mcu && r.mcu > *last_accepted_mcu && r.mcu - *last_accepted_mcu > 1000000)
        split_pending = true;
      r.reason = rollback    ? "rollback"
                 : duplicate ? "duplicate"
                 : failure   ? failure
                 : !r.synced ? "unsynchronized"
                             : "accepted";
      r.accepted = r.reason == "accepted";
      diagnostics.Record(state, *frame, PAYLOAD, r.reason.c_str());
      if (r.accepted) {
        const bool new_segment = split_pending;
        if (new_segment) {
          ++segment;
          split_pending = false;
        }
        auto q = mv::geometry::Quaternion(r.values[0], r.values[1], r.values[2], r.values[3]);
        q = mv::geometry::ResolveRotationOnlyWorldTGimbal(q.normalized(), extrinsics).rotation;
        const auto angles = Angles(q);
        if (new_segment) {
          origin = angles;
          previous_angles = angles;
          unwrapped = angles;
        }
        for (std::size_t j = 0; j < 3; ++j) {
          unwrapped[j] += std::remainder(angles[j] - previous_angles[j], 360.0);
          previous_angles[j] = angles[j];
          current[j] = unwrapped[j] - origin[j];
        }
        if (std::abs(angles[1]) >= 85.0)
          current.fill(std::numeric_limits<double>::quiet_NaN());
        last_accepted_mcu = r.mcu;
        if (!first)
          first = now;
        last_valid = now;
        ++accepted;
      }
      r.segment = segment;
      // Rejected duplicate/rollback frames must not advance the freshness baseline.
      if (!rollback && !duplicate)
        previous = r;
      WriteRow(out, r);
      ++rows;
      if (!out)
        failed = true;
      return !rollback;
    } catch (...) {
      std::lock_guard lock(mutex);
      failed = true;
      return false;
    }
  }
};

struct Point {
  double t{};
  std::array<double, 7> v{};  // yaw,pitch,roll,SO3 (deg), gyro xyz (rad/s).
  std::size_t count{};
};
struct Segment {
  std::vector<Point> seconds;
  std::map<int, Point> buckets;
  std::uint64_t first_mcu{}, last_mcu{}, generation{};
  std::size_t count{};
  bool euler_valid{true};
  mv::geometry::Quaternion reference;
  std::array<double, 3> origin{}, previous{}, unwrapped{};
  std::array<double, 7> start{}, end{};
  std::array<double, 3> gyro_mean{}, gyro_m2{};
};

/** @brief Unweighted least squares on per-second means, using actual MCU time. */
YAML::Node Statistics(const std::vector<Point>& points, std::size_t axis, bool fit) {
  YAML::Node n;
  if (points.empty())
    return n;
  double mt = 0, my = 0;
  for (const auto& p : points) {
    mt += p.t;
    my += p.v[axis];
  }
  mt /= static_cast<double>(points.size());
  my /= static_cast<double>(points.size());
  double xx = 0, xy = 0, yy = 0;
  for (const auto& p : points) {
    xx += (p.t - mt) * (p.t - mt);
    xy += (p.t - mt) * (p.v[axis] - my);
    yy += (p.v[axis] - my) * (p.v[axis] - my);
  }
  n["mean"] = my;
  n["stddev"] = std::sqrt(yy / static_cast<double>(points.size()));
  if (fit && points.size() > 1 && xx > 0) {
    const double slope = xy / xx;
    double residual = 0;
    for (const auto& p : points)
      residual += std::pow(p.v[axis] - my - slope * (p.t - mt), 2);
    n["slope_per_s"] = slope;
    n["slope_per_min"] = slope * 60;
    n["detrended_rms"] = std::sqrt(residual / static_cast<double>(points.size()));
    if (yy > 1e-16)
      n["r_squared"] = 1 - residual / yy;
    else
      n["r_squared"] = YAML::Node(YAML::NodeType::Null);
  }
  return n;
}

/** @brief Embedded SVG needs no browser libraries, network or Python packages. */
void Plot(std::ostream& out, const std::vector<Point>& points, std::size_t axis,
          const std::string& title) {
  if (points.empty())
    return;
  double lo = points.front().v[axis], hi = lo;
  for (const auto& p : points) {
    lo = std::min(lo, p.v[axis]);
    hi = std::max(hi, p.v[axis]);
  }
  const double range = std::max(hi - lo, 1e-9);
  const double duration = std::max(points.back().t, 1.0);
  out << "<h3>" << title << "</h3><p>min=" << lo << ", max=" << hi << "; time: 0–" << duration
      << " s</p><svg viewBox='0 0 900 220' role='img'>"
      << "<path d='M40 10V190H880' fill='none' stroke='#888'/>"
      << "<polyline fill='none' stroke='#1769aa' stroke-width='1.5' points='";
  for (const auto& p : points)
    out << 40 + p.t / duration * 840 << ',' << 180 - (p.v[axis] - lo) / range * 160 << ' ';
  out << "'/><text x='40' y='212'>0 s</text><text x='780' y='212'>" << duration
      << " s</text></svg>\n";
}

/** @brief Offline analysis uses only the frozen session and never opens a device. */
int Analyze(const Path& folder) {
  const auto metadata = mv::ConfigLoader::LoadFile(folder / "session.yaml");
  if (metadata["kind"].as<std::string>() != "imu_static_drift")
    throw std::runtime_error("not an imu drift session");
  const auto ext = mv::runtime::ParseGimbalExtrinsics(metadata["extrinsics"]);
  std::ifstream input(folder / "samples.csv");
  std::string line;
  if (!std::getline(input, line) || line != HEADER)
    throw std::runtime_error("invalid CSV header");
  std::map<std::uint64_t, Segment> segments;
  std::map<std::string, std::size_t> rejections;
  std::size_t total = 0, accepted = 0;
  std::uint64_t packet_skips = 0, sample_skips = 0;
  while (std::getline(input, line)) {
    const Row r = ReadRow(line);
    ++total;
    packet_skips += r.packet_skip;
    sample_skips += r.sample_skip;
    if (!r.accepted) {
      ++rejections[r.reason];
      continue;
    }
    auto& s = segments[r.segment];
    auto qi = mv::geometry::Quaternion(r.values[0], r.values[1], r.values[2], r.values[3]);
    if (!qi.coeffs().allFinite() || qi.norm() < 0.5 || qi.norm() > 1.5 || !r.synced)
      throw std::runtime_error("invalid accepted quaternion/sync in CSV");
    const auto q = mv::geometry::ResolveRotationOnlyWorldTGimbal(qi.normalized(), ext).rotation;
    const auto a = Angles(q);
    if (s.count == 0) {
      s.first_mcu = r.mcu;
      s.generation = r.generation;
      s.reference = q;
      s.origin = a;
      s.previous = a;
      s.unwrapped = a;
    } else {
      if (s.generation != r.generation || r.mcu <= s.last_mcu || r.mcu - s.last_mcu > 1000000)
        throw std::runtime_error("discontinuous data inside segment");
      for (std::size_t j = 0; j < 3; ++j) {
        s.unwrapped[j] += std::remainder(a[j] - s.previous[j], 360.0);
        s.previous[j] = a[j];
      }
    }
    s.euler_valid = s.euler_valid && std::abs(a[1]) < 85.0;
    s.last_mcu = r.mcu;
    const double t = static_cast<double>(r.mcu - s.first_mcu) / 1e6;
    Point p;
    p.t = t;
    for (std::size_t j = 0; j < 3; ++j)
      p.v[j] = s.unwrapped[j] - s.origin[j];
    p.v[3] = s.reference.angularDistance(q) * DEG;
    ++s.count;
    ++accepted;
    for (std::size_t j = 0; j < 3; ++j) {
      p.v[j + 4] = r.values[j + 4];
      const double delta = r.values[j + 4] - s.gyro_mean[j];
      s.gyro_mean[j] += delta / static_cast<double>(s.count);
      s.gyro_m2[j] += delta * (r.values[j + 4] - s.gyro_mean[j]);
    }
    if (s.count == 1)
      s.start = p.v;
    s.end = p.v;
    auto& bucket = s.buckets[static_cast<int>(t)];
    bucket.t += t;
    ++bucket.count;
    for (std::size_t j = 0; j < 7; ++j)
      bucket.v[j] += p.v[j];
  }
  if (!input.eof())
    throw std::runtime_error("CSV read failed");
  const auto output = NewDirectory(folder / "analysis" / Id());
  YAML::Node report;
  report["schema_version"] = 1;
  report["total_rows"] = total;
  report["accepted_rows"] = accepted;
  report["packet_sequence_skips"] = packet_skips;
  report["sample_sequence_skips"] = sample_skips;
  report["capture_status"] = metadata["status"];
  if (metadata["protocol_diagnostics"])
    report["protocol_diagnostics"] = YAML::Clone(metadata["protocol_diagnostics"]);
  report["final_link_reason"] = metadata["final_link_reason"];
  report["final_crc_errors"] = metadata["final_crc_errors"];
  report["sequence_skip_note"] = "Sequence gaps are not confirmed serial packet loss.";
  for (const auto& [reason, count] : rejections)
    report["rejected"][reason] = count;
  std::ofstream csv(output / "seconds.csv"), md(output / "README.md"), html(output / "plots.html");
  csv << std::setprecision(17)
      << "segment,time_s,count,yaw_deg,pitch_deg,roll_deg,so3_deg,wx_rad_s,wy_rad_s,wz_rad_s,euler_"
         "valid\n";
  md << "# IMU 静止漂移评估\n\n结论以机械全程静止为前提；不设置合格门限，不生成补偿配置。"
        "角速度均值是协议输出的统计，不直接等同于原始陀螺零偏。\n\n"
     << "有效记录 " << accepted << " / " << total
     << "。曲线见 plots.html；完整统计见 report.yaml。\n\n";
  html << "<!doctype html><meta charset='utf-8'><title>IMU drift</title>"
          "<style>body{max-width:1000px;margin:30px auto;font:16px "
          "sans-serif}svg{width:100%}</style>"
          "<h1>IMU 静止漂移</h1><p>各段独立起点；未跨会话拟合。姿态单位 deg，角速度 rad/s。</p>";
  constexpr std::array<const char*, 7> NAMES = {"yaw_deg",  "pitch_deg", "roll_deg", "so3_deg",
                                                "wx_rad_s", "wy_rad_s",  "wz_rad_s"};
  for (auto& [id, s] : segments) {
    for (auto& [second, p] : s.buckets) {
      (void)second;
      p.t /= static_cast<double>(p.count);
      for (auto& v : p.v)
        v /= static_cast<double>(p.count);
      s.seconds.push_back(p);
      csv << id << ',' << p.t << ',' << p.count;
      for (double v : p.v)
        csv << ',' << v;
      csv << ',' << s.euler_valid << '\n';
    }
    const double duration = static_cast<double>(s.last_mcu - s.first_mcu) / 1e6;
    const bool fit = duration >= 60;
    YAML::Node result;
    result["segment"] = id;
    result["generation"] = s.generation;
    result["duration_s"] = duration;
    result["samples"] = s.count;
    result["short_record"] = !fit;
    result["euler_valid"] = s.euler_valid;
    result["euler_singularity_pitch_limit_deg"] = 85;
    md << "## 段 " << id << "\n\n时长 " << duration << " 秒，样本 " << s.count
       << (fit ? "。\n\n" : "；短记录，不评估长期漂移。\n\n");
    html << "<hr><h2>Segment " << id << " / generation " << s.generation << "</h2>";
    if (!s.euler_valid) {
      md << "接近 Euler 俯仰奇异位置，Euler 角无效，不报告航向斜率。\n\n";
      html << "<p>Euler angles unavailable: pitch near singularity.</p>";
    }
    for (std::size_t j = 0; j < 7; ++j) {
      if (j < 3 && !s.euler_valid)
        continue;
      auto stat = Statistics(s.seconds, j, fit && j < 3);
      stat["first_to_last_change"] = s.end[j] - s.start[j];
      if (j >= 4) {
        stat["raw_sample_mean"] = s.gyro_mean[j - 4];
        stat["raw_sample_stddev"] = std::sqrt(s.gyro_m2[j - 4] / static_cast<double>(s.count));
      }
      result["statistics"][NAMES[j]] = stat;
      if (j < 3 && fit)
        md << "- " << NAMES[j] << "：" << stat["slope_per_min"].as<double>()
           << " °/min，去趋势 RMS " << stat["detrended_rms"].as<double>()
           << "°；R²=" << (stat["r_squared"].IsNull() ? "N/A" : stat["r_squared"].Scalar()) << "\n";
      Plot(html, s.seconds, j, NAMES[j]);
    }
    std::map<int, std::vector<Point>> minutes;
    for (const auto& p : s.seconds)
      minutes[static_cast<int>(p.t / 60)].push_back(p);
    for (const auto& [minute, points] : minutes) {
      YAML::Node m;
      m["minute"] = minute;
      m["second_bins"] = points.size();
      const bool enough = points.size() >= 55;
      m["partial"] = !enough;
      for (std::size_t j = 0; j < 7; ++j) {
        if (j < 3 && !s.euler_valid)
          continue;
        m[NAMES[j]] = Statistics(points, j, fit && enough && j < 3);
      }
      result["minutes"].push_back(m);
    }
    report["segments"].push_back(result);
    md << '\n';
  }
  csv.close();
  md.close();
  html.close();
  if (!csv || !md || !html)
    throw std::runtime_error("report write failed");
  Save(output / "report.yaml", report);
  std::cout << "评估报告: " << output << '\n';
  return accepted == 0 ? 2 : 0;
}

/** @brief Owns the link until capture ends, then analyzes its immutable raw log. */
int RunCapture(const std::string& device, double duration, const std::string& note,
               const std::string& motor, const std::string& thermal) {
  auto config = mv::runtime::LoadRealGeometryConfig(mv::test::imu::ConfigRoot());
  if (!device.empty())
    config.imu.device = device;
  if (!config.imu.enabled || config.imu.device.empty())
    throw std::runtime_error("selected profile IMU is disabled or has no device");
  const auto folder = NewDirectory(Path(PROJECT_ROOT_PATH) / "artifacts/imu_drift" / Id());
  auto metadata = mv::test::imu::CaptureMetadata(config, "imu_static_drift");
  metadata["duration_requested_s"] = duration;
  metadata["note"] = note;
  metadata["motor_state"] = motor;
  metadata["thermal_state"] = thermal;
  metadata["protocol"] = "attitude_v1; quaternion IMU-to-world wxyz; gyro IMU-body rad/s";
  metadata["status"] = "capturing";
  Save(folder / "session.yaml", metadata);
  Capture c;
  c.extrinsics = config.extrinsics;
  c.diagnostics.Open(folder);
  c.out.open(folder / "samples.csv");
  c.out << std::setprecision(17) << HEADER << '\n';
  if (!c.out)
    throw std::runtime_error("cannot open samples.csv");
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  const auto start = Clock::now();
  auto next_status = start;
  bool stdin_open = true;
  std::string command, finish = "duration";
  std::cout << "profile=" << config.profile << " device=" << config.imu.device
            << " baud=" << config.imu.baud
            << "\n请先停止占用同一串口的程序；固定底盘、大 yaw、小 yaw 和 pitch。\n"
            << "仅发送握手和同步，不控制云台。Q 回车/Ctrl+C 提前保存。\n输出: " << folder << '\n';
  {
    mv::hal::serial::ControllerLink link(
        {.enabled = true,
         .device = config.imu.device,
         .baud = config.imu.baud,
         .startup_sync_ms = config.imu.startup_sync_ms,
         .sync_ms = config.imu.sync_ms,
         .timeout_ms = config.imu.timeout_ms},
        [&c](const auto& state, const auto& frame) { return c.Update(state, frame); });
    while (true) {
      pollfd input{STDIN_FILENO, POLLIN, 0};
      poll(stdin_open ? &input : nullptr, stdin_open ? 1 : 0, 100);
      if (stdin_open && (input.revents & (POLLIN | POLLHUP))) {
        char buffer[256];
        const auto n = read(STDIN_FILENO, buffer, sizeof(buffer));
        if (n <= 0)
          stdin_open = false;
        else
          for (ssize_t i = 0; i < n; ++i) {
            if (buffer[i] == '\n') {
              if (command == "q" || command == "Q")
                stopped = 1;
              command.clear();
            } else if (command.size() < 256)
              command += buffer[i];
          }
      }
      if (stdin_open && (input.revents & (POLLERR | POLLNVAL)))
        stdin_open = false;
      const auto now = Clock::now();
      std::lock_guard lock(c.mutex);
      const double elapsed = c.first ? std::chrono::duration<double>(now - *c.first).count() : 0;
      if (stopped) {
        finish = "interrupted";
        break;
      }
      if (c.failed) {
        finish = "write_or_callback_error";
        break;
      }
      if (!c.first && now - start >= std::chrono::seconds(30)) {
        finish = "startup_timeout";
        break;
      }
      if (c.first && elapsed >= duration)
        break;
      if (now >= next_status) {
        c.out.flush();
        if (!c.out) {
          c.failed = true;
          finish = "write_error";
          break;
        }
        const bool fresh = c.last_valid && now - *c.last_valid < std::chrono::seconds(1);
        std::cout << "sync=" << c.health.synchronized << " fresh=" << fresh
                  << " accepted=" << c.accepted << " Hz=" << c.health.rate_hz
                  << " elapsed=" << elapsed << " segment=" << c.segment << " world_delta_ypr_deg=["
                  << c.current[0] << ',' << c.current[1] << ',' << c.current[2]
                  << "] link=" << c.health.reason << std::endl;
        next_status = now + std::chrono::seconds(1);
      }
    }
  }  // Join serial worker before finalizing files and running analysis.
  c.out.close();
  metadata["status"] = finish;
  metadata["rows"] = c.rows;
  metadata["accepted"] = c.accepted;
  metadata["final_link_reason"] = c.health.reason;
  metadata["final_crc_errors"] = c.health.crc_errors;
  metadata["final_generation"] = c.health.generation;
  metadata["protocol_diagnostics"] = c.diagnostics.Summary();
  Save(folder / "session.yaml", metadata);
  c.diagnostics.Close();
  if (c.failed || !c.out)
    throw std::runtime_error("capture failed; partial data retained");
  return Analyze(folder);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc > 1 && std::string(argv[1]) == "--help") {
      std::cout << "mv-imu-drift capture [--device DEVICE] [--duration SECONDS] "
                   "[--note TEXT] [--motor-state STATE] [--thermal-state STATE]\n"
                   "mv-imu-drift analyze SESSION_DIRECTORY\n"
                   "Default: 900 seconds; motor/thermal state unspecified. Exit: 0 success, "
                   "1 error, 2 no valid samples.\n";
      return 0;
    }
    if (argc < 2)
      throw std::runtime_error("specify capture or analyze; use --help for usage");
    const std::string mode = argv[1];
    if (mode == "analyze") {
      if (argc != 3)
        throw std::runtime_error("analyze requires session directory");
      return Analyze(mv::ConfigLoader::ResolvePath(PROJECT_ROOT_PATH, argv[2]));
    }
    if (mode != "capture")
      throw std::runtime_error("unknown command; use --help");
    std::string device, note, motor = "unspecified", thermal = "unspecified";
    double duration = 900;
    for (int i = 2; i < argc; i += 2) {
      if (i + 1 >= argc)
        throw std::runtime_error("missing option value");
      const std::string key = argv[i], value = argv[i + 1];
      if (key == "--device")
        device = value;
      else if (key == "--duration") {
        std::size_t consumed = 0;
        duration = std::stod(value, &consumed);
        if (consumed != value.size())
          throw std::runtime_error("invalid duration");
      } else if (key == "--note")
        note = value;
      else if (key == "--motor-state")
        motor = value;
      else if (key == "--thermal-state")
        thermal = value;
      else
        throw std::runtime_error("unknown option: " + key);
    }
    if (!std::isfinite(duration) || duration <= 0 || duration > 86400)
      throw std::runtime_error("duration must be in (0,86400] seconds");
    return RunCapture(device, duration, note, motor, thermal);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
