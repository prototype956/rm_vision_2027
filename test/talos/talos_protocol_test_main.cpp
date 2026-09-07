#include "hal/camera/talos/talos_device.hpp"
#include "hal/camera/talos/talos_ipc_layout.hpp"
#include "hal/gimbal/talos/talos_gimbal_command_sink.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

#include <sys/mman.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace {
namespace ipc = mv::hal::detail::talos_ipc;
void Require(bool condition, const char* message) {
  if (!condition)
    throw std::runtime_error(message);
}
/** @brief 独占的临时映射；不使用运行中模拟器的共享内存路径。 */
class TemporaryMapping {
 public:
  explicit TemporaryMapping(std::size_t bytes) : bytes_(bytes) {
    char name[] = "/tmp/mv-talos-acceptance-XXXXXX";
    fd_ = mkstemp(name);
    Require(fd_ >= 0, "mkstemp failed");
    path_ = name;
    Require(ftruncate(fd_, static_cast<off_t>(bytes)) == 0, "ftruncate failed");
    address_ = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    Require(address_ != MAP_FAILED, "mmap failed");
  }
  ~TemporaryMapping() {
    if (address_ != MAP_FAILED)
      munmap(address_, bytes_);
    if (fd_ >= 0)
      close(fd_);
    unlink(path_.c_str());
  }
  TemporaryMapping(const TemporaryMapping&) = delete;
  TemporaryMapping& operator=(const TemporaryMapping&) = delete;
  [[nodiscard]] const std::string& Path() const { return path_; }
  [[nodiscard]] void* Address() const { return address_; }

 private:
  std::size_t bytes_;
  int fd_{-1};
  std::string path_;
  void* address_{MAP_FAILED};
};

std::uint64_t NowNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}
}  // namespace

int main() {
  try {
    TemporaryMapping metadata(sizeof(ipc::ShmMetaRegion));
    TemporaryMapping pixels(9);
    auto& region = *static_cast<ipc::ShmMetaRegion*>(metadata.Address());
    mv::hal::detail::TalosConfig config{.meta_path = metadata.Path(),
                                        .image_pool_path = pixels.Path(),
                                        .expected_width = 1,
                                        .expected_height = 1,
                                        .connect_timeout_ms = 1,
                                        .grab_timeout_ms = 1,
                                        .heartbeat_timeout_ms = 1000};
    YAML::Node sink_config;
    sink_config["shared_memory"]["meta_path"] = metadata.Path();
    sink_config["timeouts"]["heartbeat_ms"] = 1000;
    mv::hal::detail::TalosDevice camera;
    mv::hal::TalosGimbalCommandSink sink;
    region.header.magic = 0x54414C06;
    region.header.version = 6;
    Require(!camera.Open(config), "camera accepted v6");
    Require(!sink.Open(sink_config), "sink accepted v6");
    // A real v6 file is smaller than v7: version diagnostics must precede size diagnostics.
    TemporaryMapping old_metadata(18816);
    *static_cast<ipc::ShmHeader*>(old_metadata.Address()) = region.header;
    config.meta_path = old_metadata.Path();
    sink_config["shared_memory"]["meta_path"] = old_metadata.Path();
    Require(!camera.Open(config), "camera accepted small v6");
    Require(!sink.Open(sink_config), "sink accepted small v6");
    config.meta_path = metadata.Path();
    sink_config["shared_memory"]["meta_path"] = metadata.Path();
    region = {};
    region.header.magic = ipc::K_SHM_MAGIC;
    region.header.version = ipc::K_SHM_VERSION;
    region.header.heartbeat_ns = NowNs();
    region.header.image_width = 1;
    region.header.image_height = 1;
    region.frame.state = 1;
    region.frame.read_index = 2;
    region.gimbal_cmd.state = 1;
    region.gimbal_cmd.read_index = 2;
    Require(camera.Open(config), "camera failed v7 connect");
    Require(sink.Open(sink_config), "sink failed v7 connect");
    ipc::CapturedFrameMeta frame{};
    frame.frame_sequence = 7;
    frame.capture_timestamp_ns = NowNs();
    frame.width = 1;
    frame.height = 1;
    frame.format = ipc::K_FORMAT_BGR8;
    frame.camera_info.timestamp_ns = frame.capture_timestamp_ns;
    frame.camera_info.width = 1;
    frame.camera_info.height = 1;
    frame.camera_info.fx = 1;
    frame.camera_info.fy = 1;
    frame.world_t_gimbal.rotation.w = 1;
    frame.gimbal_t_camera_optical.rotation.w = 1;
    frame.gimbal_t_muzzle.rotation.w = 1;
    frame.ground_truth.frame_sequence = frame.frame_sequence;
    frame.ground_truth.timestamp_ns = frame.capture_timestamp_ns;
    frame.projectile_statistics.timestamp_ns = frame.capture_timestamp_ns;
    frame.combat.round_id = 2;
    frame.combat.round_started_ns = 100'000'000;
    frame.combat.sim_time_ns = 250'000'000;
    frame.combat.referee_sample_ns = 200'000'000;
    // Publish only after camera Open(), which discards any earlier queued frame.
    region.frame.slots[0] = frame;
    __atomic_store_n(&region.frame.state, ipc::K_FLAG_NEW, __ATOMIC_RELEASE);
    mv::frame::FramePacket packet;
    Require(camera.Grab(packet) == mv::hal::GrabStatus::OK, "v7 grab failed");
    Require(packet.capture.stamp.simulation_round_id == 2 && packet.simulation &&
                packet.simulation->combat &&
                packet.simulation->combat->referee_sample_ns == 200'000'000,
            "round or sample time lost while decoding");
    mv::hal::GimbalCommand command;
    command.valid = true;
    command.target_distance_m = 3;
    command.source_round_id = 2;
    command.source_frame_sequence = 7;
    command.source_capture_timestamp_ns = frame.capture_timestamp_ns;
    Require(sink.Send(command), "v7 command send failed");
    const auto& wire = region.gimbal_cmd.slots[0];
    Require(wire.source_round_id == 2 && wire.source_frame_sequence == 7 &&
                wire.source_capture_timestamp_ns == frame.capture_timestamp_ns,
            "command source provenance lost");
    // A sample from another round must reject the entire packet.
    frame.frame_sequence++;
    frame.capture_timestamp_ns++;
    frame.camera_info.timestamp_ns = frame.capture_timestamp_ns;
    frame.ground_truth.frame_sequence = frame.frame_sequence;
    frame.ground_truth.timestamp_ns = frame.capture_timestamp_ns;
    frame.projectile_statistics.timestamp_ns = frame.capture_timestamp_ns;
    frame.combat.referee_sample_ns = 0;
    region.frame.slots[1] = frame;
    __atomic_store_n(&region.frame.state, static_cast<std::uint8_t>(ipc::K_FLAG_NEW | 1),
                     __ATOMIC_RELEASE);
    Require(camera.Grab(packet) == mv::hal::GrabStatus::INVALID_FRAME,
            "cross-round sample accepted");
    std::cout << "Talos v7 ABI, v6 rejection, frame decode and command provenance passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
