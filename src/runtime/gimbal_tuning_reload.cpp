#include "runtime/gimbal_tuning_reload.hpp"

#include "core/logger.hpp"

#include <array>
#include <chrono>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>

namespace mv::runtime {
namespace {
std::string Read(const std::filesystem::path& path) {
  if (std::filesystem::file_size(path) > 65536)
    throw std::runtime_error("gimbal tuning YAML exceeds 64 KiB");
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("cannot open gimbal tuning YAML");
  std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  if (input.bad())
    throw std::runtime_error("cannot read gimbal tuning YAML");
  return text;
}
}  // namespace

GimbalTuningReload::GimbalTuningReload(std::filesystem::path root, GimbalTuningSnapshot startup)
    : root_(std::move(root)), startup_(startup) {
  worker_ = std::jthread([this](const std::stop_token& stop) { Run(stop); });
}

std::optional<GimbalTuningSnapshot> GimbalTuningReload::TakePending() {
  std::lock_guard lock(mutex_);
  return std::exchange(pending_, std::nullopt);
}

void GimbalTuningReload::Run(const std::stop_token& stop) {
  auto last_good = startup_;
  using Texts = std::array<std::string, 3>;
  std::optional<Texts> candidate;
  std::optional<Texts> attempted;
  std::string last_error;
  const auto ALGORITHM = std::string(modules::GimbalControlModeName(startup_.control.mode));
  while (!stop.stop_requested()) {
    try {
      // File I/O and YAML parsing never execute on the 100 Hz control thread.
      const Texts TEXTS{Read(root_ / "modules/gimbal_control.yaml"),
                        Read(root_ / "modules/gimbal_control" / (ALGORITHM + ".yaml")),
                        Read(root_ / "hal/serial/gimbal.yaml")};
      if (!candidate || *candidate != TEXTS) {
        candidate = TEXTS;
      } else if (!attempted || *attempted != TEXTS) {
        attempted = TEXTS;
        GimbalTuningSnapshot next{modules::ParseGimbalControlConfig(
                                      YAML::Load(TEXTS[0]), "mindvision", YAML::Load(TEXTS[1])),
                                  hal::ParseSerialGimbalCommandSinkConfig(YAML::Load(TEXTS[2]))};
        auto immutable = next.control;
        immutable.rate_limits = startup_.control.rate_limits;
        if (startup_.control.mode == modules::GimbalControlMode::PID)
          immutable.pid = startup_.control.pid;
        else
          immutable.follower = startup_.control.follower;
        if (immutable != startup_.control || next.transport != startup_.transport)
          throw std::runtime_error(
              "mode/reference/feedback/MPC solver/transport changed: restart required; edit "
              "rejected");
        if (next != last_good) {
          {
            std::lock_guard lock(mutex_);
            pending_ = next;
          }
          last_good = next;
          MV_LOG_INFO("GimbalTuning", "validated new {} tuning; pending control cycle", ALGORITHM);
        }
        last_error.clear();
      }
    } catch (const std::exception& error) {
      if (last_error != error.what()) {
        last_error = error.what();
        MV_LOG_WARN("GimbalTuning", "rejected tuning, keeping last valid parameters: {}",
                    last_error);
      }
    } catch (...) {
      MV_LOG_WARN("GimbalTuning", "unexpected reload error; keeping last valid parameters");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}
}  // namespace mv::runtime
