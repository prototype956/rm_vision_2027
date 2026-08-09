#include "talos_device.hpp"

#include "core/logger.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

#include <fcntl.h>
#include <opencv2/imgproc.hpp>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mv::hal::detail {
namespace {

// 以下常量和结构必须与 Daedalus 的 Talos 共享内存 ABI 保持一致。元数据区使用
// 固定布局，图像像素则按三个连续缓冲区存放在独立文件中。
constexpr uint32_t K_SHM_MAGIC = 0x54414C05;
constexpr uint32_t K_SHM_VERSION = 5;
constexpr uint8_t K_FLAG_NEW = 0x80;
constexpr uint8_t K_INDEX_MASK = 0x03;
constexpr uint8_t K_FORMAT_RGB8 = 0;
constexpr uint8_t K_FORMAT_BGR8 = 1;
constexpr std::size_t K_BUFFER_COUNT = 3;
constexpr std::size_t K_GROUND_TRUTH_MAX_TARGETS = 16;
constexpr std::size_t K_GROUND_TRUTH_MAX_ARMORS = 32;

struct alignas(64) ShmHeader {
  uint32_t magic;
  uint32_t version;
  uint64_t created_ns;
  uint64_t heartbeat_ns;
  uint32_t image_width;
  uint32_t image_height;
  uint8_t pad[32];
};

struct QuaternionF32 {
  float x;  ///< Hamilton 四元数虚部 X。
  float y;  ///< Hamilton 四元数虚部 Y。
  float z;  ///< Hamilton 四元数虚部 Z。
  float w;  ///< Hamilton 四元数实部。
};

// Talos v5 的刚体变换约定与 geometry::RigidTransform 相同：parent_t_child
// 将 child 中的坐标变换到 parent，平移单位为米。
struct alignas(32) RigidTransformF32 {
  float translation[3];
  QuaternionF32 rotation;
  uint8_t pad[4];
};

struct alignas(64) CameraCalibrationMeta {
  uint64_t timestamp_ns;  ///< 所属采集快照的 Unix epoch 纳秒时间。
  double fx;              ///< 水平焦距，单位为像素。
  double fy;              ///< 垂直焦距，单位为像素。
  double cx;              ///< 主点横坐标，单位为像素。
  double cy;              ///< 主点纵坐标，单位为像素。
  double distortion[5];   ///< plumb_bob 顺序：k1、k2、p1、p2、k3。
  uint32_t width;         ///< 标定适用的图像宽度。
  uint32_t height;        ///< 标定适用的图像高度。
  uint8_t pad[24];
};

// 下列未消费的协议区仍必须占位，以保持与 Rust #[repr(C, align(...))] ABI 一致。
struct alignas(64) ChassisObservationMeta {
  uint8_t bytes[128];
};

struct alignas(32) GroundTruthTargetMeta {
  uint64_t frame_sequence;  ///< 所属图像帧序号。
  uint64_t timestamp_ns;    ///< 所属采集快照的 Unix epoch 纳秒时间。
  uint64_t id;              ///< 本次仿真运行内的目标标识。
  uint8_t team;             ///< 0 为红方，1 为蓝方。
  uint8_t armor_label;      ///< Talos 装甲类别编码。
  uint8_t is_outpost;       ///< 非零表示特殊旋转目标。
  uint8_t pad1;
  float position[3];   ///< world 坐标系位置，单位为米。
  float yaw_velocity;  ///< 绕 world +Z 的角速度，单位为弧度每秒。
  float yaw;           ///< 绕 world +Z 的航向角，单位为弧度。
  uint8_t pad[16];
};

struct alignas(64) GroundTruthRuneMeta {
  uint8_t bytes[128];
};

struct alignas(64) GroundTruthArmorMeta {
  uint64_t id;
  uint8_t team;
  uint8_t label;
  uint8_t armor_type;
  uint8_t pad1;
  float width_m;
  float height_m;
  uint8_t pad2[12];
  RigidTransformF32 world_t_armor;
  float corners_world[4][3];
  uint8_t pad3[16];
};

struct alignas(64) GroundTruthBatchMeta {
  uint64_t frame_sequence;  ///< 整批真值所属图像帧序号。
  uint64_t timestamp_ns;    ///< 整批真值所属采集快照时间。
  uint32_t target_count;    ///< targets 中的有效元素数量。
  uint32_t rune_count;
  uint32_t armor_count;
  uint32_t pad1;
  GroundTruthTargetMeta targets[K_GROUND_TRUTH_MAX_TARGETS];
  GroundTruthRuneMeta runes[4];
  GroundTruthArmorMeta armors[K_GROUND_TRUTH_MAX_ARMORS];
};

struct alignas(64) CapturedFrameMeta {
  uint64_t frame_sequence;        ///< 发布端启动后严格递增的帧序号。
  uint64_t capture_timestamp_ns;  ///< 整个快照共用的 Unix epoch 纳秒时间。
  uint32_t width;
  uint32_t height;
  uint8_t buffer_id;
  uint8_t format;
  uint8_t pad1[30];
  CameraCalibrationMeta camera_info;
  RigidTransformF32 world_t_gimbal;           ///< gimbal 到 world 的变换。
  RigidTransformF32 gimbal_t_camera_optical;  ///< camera_optical 到 gimbal 的变换。
  RigidTransformF32 gimbal_t_muzzle;          ///< muzzle 到 gimbal 的变换。
  uint8_t pad2[32];
  ChassisObservationMeta chassis_observation;
  GroundTruthBatchMeta ground_truth;
};

struct alignas(64) FrameTripleBuffer {
  uint8_t state;
  uint8_t write_index;
  uint8_t read_index;
  uint8_t pad[61];
  CapturedFrameMeta slots[3];
};

struct alignas(64) GimbalCmdMeta {
  uint8_t bytes[192];
};

struct alignas(64) RuntimeStateMeta {
  uint8_t bytes[64];
};

struct alignas(64) ShmMetaRegion {
  ShmHeader header;
  FrameTripleBuffer frame;
  GimbalCmdMeta gimbal_cmd;
  RuntimeStateMeta runtime_state;
};

// 编译期校验可防止字段或对齐方式变化后静默破坏跨进程协议。
static_assert(sizeof(ShmHeader) == 64);
static_assert(sizeof(QuaternionF32) == 16);
static_assert(sizeof(RigidTransformF32) == 32);
static_assert(sizeof(CameraCalibrationMeta) == 128);
static_assert(sizeof(GroundTruthTargetMeta) == 64);
static_assert(sizeof(GroundTruthArmorMeta) == 128);
static_assert(sizeof(GroundTruthBatchMeta) == 5696);
static_assert(sizeof(CapturedFrameMeta) == 6144);
static_assert(sizeof(FrameTripleBuffer) == 18496);
static_assert(offsetof(ShmMetaRegion, frame) == 64);
static_assert(offsetof(ShmMetaRegion, gimbal_cmd) == 18560);
static_assert(offsetof(ShmMetaRegion, runtime_state) == 18752);
static_assert(sizeof(ShmMetaRegion) == 18816);

uint64_t SystemNowNs() noexcept {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count());
}

uint8_t AtomicLoad(const uint8_t* value) noexcept {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

uint64_t AtomicLoad(const uint64_t* value) noexcept {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

[[nodiscard]] bool IsMapFailed(const void* mapping) noexcept {
  // MAP_FAILED 是 POSIX 规定的整数到指针哨兵值，并非业务代码中的地址转换。
  return mapping == MAP_FAILED;  // NOLINT(performance-no-int-to-ptr)
}

template <typename Slot, typename TripleBuffer>
bool Consume(TripleBuffer& buffer, Slot& slot) noexcept {
  // 发布端用最高位表示存在新数据、低两位表示可读槽位。CAS 抢占状态后再复制
  // 元数据，保证同一个槽位不会被本消费者重复读取。
  uint8_t expected = AtomicLoad(&buffer.state);
  if ((expected & K_FLAG_NEW) == 0) {
    return false;
  }
  const uint8_t READY_INDEX = expected & K_INDEX_MASK;
  if (READY_INDEX >= K_BUFFER_COUNT) {
    return false;
  }
  const uint8_t CONSUMED_STATE = buffer.read_index;
  if (!__atomic_compare_exchange_n(&buffer.state, &expected, CONSUMED_STATE, false,
                                   __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
    return false;
  }
  buffer.read_index = READY_INDEX;
  slot = buffer.slots[READY_INDEX];
  return true;
}

bool Finite(float value) noexcept {
  return std::isfinite(static_cast<double>(value));
}

// 除有限性外检查单位四元数范数，避免无效姿态进入后续 TF 组合和投影。
bool ValidTransform(const RigidTransformF32& transform) noexcept {
  for (const float VALUE : transform.translation) {
    if (!Finite(VALUE)) {
      return false;
    }
  }
  const auto& q = transform.rotation;
  if (!Finite(q.x) || !Finite(q.y) || !Finite(q.z) || !Finite(q.w)) {
    return false;
  }
  const double NORM = std::sqrt(static_cast<double>(q.x) * q.x + static_cast<double>(q.y) * q.y +
                                static_cast<double>(q.z) * q.z + static_cast<double>(q.w) * q.w);
  return std::abs(NORM - 1.0) <= 1.0e-3;
}

bool ValidCalibration(const CameraCalibrationMeta& calibration, uint32_t width,
                      uint32_t height) noexcept {
  if (calibration.width != width || calibration.height != height ||
      !std::isfinite(calibration.fx) || !std::isfinite(calibration.fy) ||
      !std::isfinite(calibration.cx) || !std::isfinite(calibration.cy) || calibration.fx <= 0.0 ||
      calibration.fy <= 0.0 || calibration.cx < 0.0 || calibration.cy < 0.0 ||
      calibration.cx > width || calibration.cy > height) {
    return false;
  }
  for (const double VALUE : calibration.distortion) {
    if (!std::isfinite(VALUE)) {
      return false;
    }
  }
  return true;
}

mv::geometry::RigidTransform ConvertTransform(const RigidTransformF32& value) noexcept {
  return {.translation = mv::geometry::Vector3(value.translation[0], value.translation[1],
                                               value.translation[2]),
          // Eigen 四元数构造顺序为 w/x/y/z；Talos v5 协议字段为 x/y/z/w。
          .rotation = mv::geometry::Quaternion(value.rotation.w, value.rotation.x, value.rotation.y,
                                               value.rotation.z)};
}

CameraFrame::FrameGeometry ConvertGeometry(const CapturedFrameMeta& metadata) {
  // Grab() 已完整验证数量、时间戳和数值范围，这里只负责从 ABI 类型提升到 HAL 类型。
  CameraFrame::FrameGeometry geometry;
  const auto& camera = metadata.camera_info;
  geometry.calibration = {
      .width = camera.width,
      .height = camera.height,
      .fx = camera.fx,
      .fy = camera.fy,
      .cx = camera.cx,
      .cy = camera.cy,
      .distortion = {camera.distortion[0], camera.distortion[1], camera.distortion[2],
                     camera.distortion[3], camera.distortion[4]}};
  geometry.world_t_gimbal = ConvertTransform(metadata.world_t_gimbal);
  geometry.gimbal_t_camera_optical = ConvertTransform(metadata.gimbal_t_camera_optical);
  geometry.gimbal_t_muzzle = ConvertTransform(metadata.gimbal_t_muzzle);

  const auto& truth = metadata.ground_truth;
  geometry.targets.reserve(truth.target_count);
  for (std::size_t index = 0; index < truth.target_count; ++index) {
    const auto& target = truth.targets[index];
    geometry.targets.push_back({.id = target.id,
                                .team = target.team,
                                .armor_label = target.armor_label,
                                .is_outpost = target.is_outpost != 0,
                                .position_world = mv::geometry::Vector3(
                                    target.position[0], target.position[1], target.position[2]),
                                .yaw = target.yaw,
                                .yaw_velocity = target.yaw_velocity});
  }
  geometry.armors.reserve(truth.armor_count);
  for (std::size_t index = 0; index < truth.armor_count; ++index) {
    const auto& armor = truth.armors[index];
    CameraFrame::GroundTruthArmor converted{
        .id = armor.id,
        .team = armor.team,
        .label = armor.label,
        .type = static_cast<CameraFrame::ArmorType>(armor.armor_type),
        .width_m = armor.width_m,
        .height_m = armor.height_m,
        .world_t_armor = ConvertTransform(armor.world_t_armor)};
    for (std::size_t corner = 0; corner < converted.corners_world.size(); ++corner) {
      converted.corners_world[corner] =
          mv::geometry::Vector3(armor.corners_world[corner][0], armor.corners_world[corner][1],
                                armor.corners_world[corner][2]);
    }
    geometry.armors.push_back(std::move(converted));
  }
  return geometry;
}

}  // namespace

struct TalosDevice::Impl {
  TalosConfig config;                           ///< 最近一次 Open() 接收的类型化配置。
  int meta_fd{-1};                              ///< 可读写的元数据文件描述符。
  int image_pool_fd{-1};                        ///< 只读图像池文件描述符。
  void* meta_mapping{nullptr};                  ///< 元数据 mmap 起始地址。
  void* image_pool_mapping{nullptr};            ///< 图像池 mmap 起始地址。
  std::size_t image_pool_size{0};               ///< 已映射图像池的字节数。
  ShmMetaRegion* meta{nullptr};                 ///< 带类型的协议元数据视图。
  CameraInfo info;                              ///< Open() 成功后对上层公开的信息。
  bool is_open{false};                          ///< 两个映射和发布端心跳均有效。
  std::optional<uint64_t> last_frame_sequence;  ///< 最近接收帧序号，用于拒绝重复或回退帧。
  std::optional<uint64_t> last_capture_timestamp_ns;  ///< 最近采集时间，用于检查严格单调性。
  uint64_t invalid_frames{0};  ///< 本次连接以来被完整性校验拒绝的帧数。

  /** 按映射、文件描述符的逆依赖顺序释放资源，并恢复关闭状态。 */
  void ResetMappings() noexcept {
    if (image_pool_mapping != nullptr) {
      ::munmap(image_pool_mapping, image_pool_size);
    }
    if (meta_mapping != nullptr) {
      ::munmap(meta_mapping, sizeof(ShmMetaRegion));
    }
    if (image_pool_fd >= 0) {
      ::close(image_pool_fd);
    }
    if (meta_fd >= 0) {
      ::close(meta_fd);
    }
    meta_fd = -1;
    image_pool_fd = -1;
    meta_mapping = nullptr;
    image_pool_mapping = nullptr;
    image_pool_size = 0;
    meta = nullptr;
    is_open = false;
    last_frame_sequence.reset();
    last_capture_timestamp_ns.reset();
    invalid_frames = 0;
  }

  bool HeartbeatFresh() const noexcept {
    if (meta == nullptr) {
      return false;
    }
    const uint64_t HEARTBEAT_NS = AtomicLoad(&meta->header.heartbeat_ns);
    const uint64_t NOW_NS = SystemNowNs();
    const uint64_t MAX_AGE_NS = static_cast<uint64_t>(config.heartbeat_timeout_ms) * 1'000'000ULL;
    // 发布端时间略领先时视为有效，以容忍系统时钟的小幅回拨或跨进程采样误差。
    return HEARTBEAT_NS != 0 && (HEARTBEAT_NS >= NOW_NS || NOW_NS - HEARTBEAT_NS <= MAX_AGE_NS);
  }

  /** 尝试建立一次完整映射；任一步失败都会回滚本次获得的全部资源。 */
  bool TryConnect(std::string& error) {
    ResetMappings();
    meta_fd = ::open(config.meta_path.c_str(), O_RDWR | O_CLOEXEC);
    if (meta_fd < 0) {
      error = "cannot open metadata '" + config.meta_path + "'";
      return false;
    }

    struct stat meta_stat {};
    if (::fstat(meta_fd, &meta_stat) != 0 ||
        meta_stat.st_size < static_cast<off_t>(sizeof(ShmMetaRegion))) {
      error = "metadata file has an invalid size";
      ResetMappings();
      return false;
    }
    meta_mapping =
        ::mmap(nullptr, sizeof(ShmMetaRegion), PROT_READ | PROT_WRITE, MAP_SHARED, meta_fd, 0);
    if (IsMapFailed(meta_mapping)) {
      meta_mapping = nullptr;
      error = "cannot map Talos metadata";
      ResetMappings();
      return false;
    }
    meta = static_cast<ShmMetaRegion*>(meta_mapping);

    const uint32_t VERSION = meta->header.version;
    const uint32_t IMAGE_WIDTH = meta->header.image_width;
    const uint32_t IMAGE_HEIGHT = meta->header.image_height;
    if (meta->header.magic != K_SHM_MAGIC || VERSION != K_SHM_VERSION) {
      error = "unsupported Talos metadata magic/version";
      ResetMappings();
      return false;
    }
    if (IMAGE_WIDTH != static_cast<uint32_t>(config.expected_width) ||
        IMAGE_HEIGHT != static_cast<uint32_t>(config.expected_height)) {
      error = "Talos output resolution does not match expected " +
              std::to_string(config.expected_width) + "x" + std::to_string(config.expected_height);
      ResetMappings();
      return false;
    }

    // 在计算三张 RGB/BGR 图像所需空间前先排除 size_t 乘法溢出。
    const auto WIDTH = static_cast<std::size_t>(IMAGE_WIDTH);
    const auto HEIGHT = static_cast<std::size_t>(IMAGE_HEIGHT);
    if (WIDTH > SIZE_MAX / HEIGHT || WIDTH * HEIGHT > SIZE_MAX / 3 / K_BUFFER_COUNT) {
      error = "Talos image dimensions overflow the image-pool size";
      ResetMappings();
      return false;
    }
    image_pool_size = WIDTH * HEIGHT * 3 * K_BUFFER_COUNT;
    image_pool_fd = ::open(config.image_pool_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (image_pool_fd < 0) {
      error = "cannot open image pool '" + config.image_pool_path + "'";
      ResetMappings();
      return false;
    }
    struct stat pool_stat {};
    if (::fstat(image_pool_fd, &pool_stat) != 0 ||
        pool_stat.st_size < static_cast<off_t>(image_pool_size)) {
      error = "Talos image-pool file has an invalid size";
      ResetMappings();
      return false;
    }
    image_pool_mapping = ::mmap(nullptr, image_pool_size, PROT_READ, MAP_SHARED, image_pool_fd, 0);
    if (IsMapFailed(image_pool_mapping)) {
      image_pool_mapping = nullptr;
      error = "cannot map Talos image pool";
      ResetMappings();
      return false;
    }
    if (!HeartbeatFresh()) {
      error = "Talos heartbeat is not active";
      ResetMappings();
      return false;
    }

    info.device_name = "talos:" + config.meta_path;
    info.sensor_width = config.expected_width;
    info.sensor_height = config.expected_height;
    info.output_width = config.expected_width;
    info.output_height = config.expected_height;
    info.roi_offset_x = 0;
    info.roi_offset_y = 0;
    info.exposure_us = 0;
    info.grab_timeout_ms = config.grab_timeout_ms;
    info.pixel_format = PixelFormat::BGR8;
    is_open = true;
    return true;
  }
};

TalosDevice::TalosDevice() : impl_(std::make_unique<Impl>()) {}

TalosDevice::~TalosDevice() {
  Close();
}

bool TalosDevice::Open(const TalosConfig& config) {
  if (impl_->is_open) {
    return true;
  }
  impl_->config = config;

  // 发布端可能仍在创建或扩容共享内存文件，因此在配置的窗口内轮询连接。
  const auto DEADLINE = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(impl_->config.connect_timeout_ms);
  std::string last_error = "Talos publisher is unavailable";
  do {
    if (impl_->TryConnect(last_error)) {
      MV_LOG_INFO("HAL.Camera.Talos", "connected to Talos v{} at {}x{} BGR8",
                  impl_->meta->header.version, impl_->info.output_width, impl_->info.output_height);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < DEADLINE);

  MV_LOG_ERROR("HAL.Camera.Talos", "connection timed out after {} ms: {}",
               impl_->config.connect_timeout_ms, last_error);
  impl_->ResetMappings();
  return false;
}

void TalosDevice::Close() noexcept {
  const bool WAS_OPEN = impl_->is_open;
  impl_->ResetMappings();
  if (WAS_OPEN) {
    MV_LOG_INFO("HAL.Camera.Talos", "closed");
  }
}

GrabStatus TalosDevice::Grab(CameraFrame& frame) {
  if (!impl_->is_open || impl_->meta == nullptr) {
    return GrabStatus::DISCONNECTED;
  }

  const auto DEADLINE =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(impl_->config.grab_timeout_ms);
  while (std::chrono::steady_clock::now() < DEADLINE) {
    if (!impl_->HeartbeatFresh()) {
      MV_LOG_ERROR("HAL.Camera.Talos", "publisher heartbeat expired");
      Close();
      return GrabStatus::DISCONNECTED;
    }

    CapturedFrameMeta metadata{};
    if (!Consume<CapturedFrameMeta>(impl_->meta->frame, metadata)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    const auto INVALID = [&]() {
      const uint64_t COUNT = ++impl_->invalid_frames;
      if (COUNT == 1 || COUNT % 100 == 0) {
        MV_LOG_WARN("HAL.Camera.Talos", "rejected invalid Talos v5 frame #{} (seq={})", COUNT,
                    metadata.frame_sequence);
      }
      return GrabStatus::INVALID_FRAME;
    };

    // 图像、标定、TF 和真值必须来自同一个原子发布的采集快照。任一子结构不同步，
    // 整帧都不能交给上层，否则 Foxglove 的三维实体和二维重投影将产生假误差。
    const auto& truth = metadata.ground_truth;
    bool truth_valid = truth.frame_sequence == metadata.frame_sequence &&
                       truth.timestamp_ns == metadata.capture_timestamp_ns &&
                       truth.target_count <= K_GROUND_TRUTH_MAX_TARGETS &&
                       truth.armor_count <= K_GROUND_TRUTH_MAX_ARMORS;
    for (std::size_t index = 0; truth_valid && index < truth.target_count; ++index) {
      const auto& target = truth.targets[index];
      truth_valid = target.frame_sequence == metadata.frame_sequence &&
                    target.timestamp_ns == metadata.capture_timestamp_ns && Finite(target.yaw) &&
                    Finite(target.yaw_velocity) && Finite(target.position[0]) &&
                    Finite(target.position[1]) && Finite(target.position[2]);
    }
    for (std::size_t index = 0; truth_valid && index < truth.armor_count; ++index) {
      const auto& armor = truth.armors[index];
      truth_valid = armor.team <= 1 && armor.label <= 7 && armor.armor_type <= 1 &&
                    Finite(armor.width_m) && Finite(armor.height_m) && armor.width_m > 0.0F &&
                    armor.height_m > 0.0F && ValidTransform(armor.world_t_armor);
      for (std::size_t corner = 0; truth_valid && corner < 4; ++corner) {
        truth_valid = Finite(armor.corners_world[corner][0]) &&
                      Finite(armor.corners_world[corner][1]) &&
                      Finite(armor.corners_world[corner][2]);
      }
      if (truth_valid) {
        const auto pose = ConvertTransform(armor.world_t_armor);
        const std::array<mv::geometry::Vector3, 4> expected{
            mv::geometry::Vector3(-armor.width_m * 0.5, armor.height_m * 0.5, 0.0),
            mv::geometry::Vector3(armor.width_m * 0.5, armor.height_m * 0.5, 0.0),
            mv::geometry::Vector3(armor.width_m * 0.5, -armor.height_m * 0.5, 0.0),
            mv::geometry::Vector3(-armor.width_m * 0.5, -armor.height_m * 0.5, 0.0)};
        for (std::size_t corner = 0; truth_valid && corner < expected.size(); ++corner) {
          const mv::geometry::Vector3 actual(armor.corners_world[corner][0],
                                             armor.corners_world[corner][1],
                                             armor.corners_world[corner][2]);
          truth_valid =
              (actual - mv::geometry::TransformPoint(pose, expected[corner])).norm() <= 0.002;
        }
      }
    }

    if (metadata.capture_timestamp_ns == 0 ||
        metadata.camera_info.timestamp_ns != metadata.capture_timestamp_ns ||
        metadata.width != static_cast<uint32_t>(impl_->info.output_width) ||
        metadata.height != static_cast<uint32_t>(impl_->info.output_height) ||
        metadata.buffer_id >= K_BUFFER_COUNT ||
        (metadata.format != K_FORMAT_RGB8 && metadata.format != K_FORMAT_BGR8) ||
        !ValidCalibration(metadata.camera_info, metadata.width, metadata.height) ||
        !ValidTransform(metadata.world_t_gimbal) ||
        !ValidTransform(metadata.gimbal_t_camera_optical) ||
        !ValidTransform(metadata.gimbal_t_muzzle) || !truth_valid ||
        (impl_->last_frame_sequence.has_value() &&
         metadata.frame_sequence <= *impl_->last_frame_sequence) ||
        (impl_->last_capture_timestamp_ns.has_value() &&
         metadata.capture_timestamp_ns <= *impl_->last_capture_timestamp_ns)) {
      return INVALID();
    }

    const std::size_t FRAME_SIZE =
        static_cast<std::size_t>(metadata.width) * static_cast<std::size_t>(metadata.height) * 3;
    const std::size_t OFFSET = static_cast<std::size_t>(metadata.buffer_id) * FRAME_SIZE;
    if (OFFSET > impl_->image_pool_size || FRAME_SIZE > impl_->image_pool_size - OFFSET) {
      return INVALID();
    }

    const auto* source = static_cast<const uint8_t*>(impl_->image_pool_mapping) + OFFSET;
    const cv::Mat SHARED_IMAGE(static_cast<int>(metadata.height), static_cast<int>(metadata.width),
                               CV_8UC3, const_cast<uint8_t*>(source));
    // 发布端会复用共享三缓冲槽位，返回前必须复制或转换到独立拥有的 cv::Mat。
    if (metadata.format == K_FORMAT_BGR8) {
      frame.image = SHARED_IMAGE.clone();
    } else {
      cv::cvtColor(SHARED_IMAGE, frame.image, cv::COLOR_RGB2BGR);
    }
    if (frame.image.empty()) {
      return GrabStatus::FATAL;
    }
    impl_->last_frame_sequence = metadata.frame_sequence;
    impl_->last_capture_timestamp_ns = metadata.capture_timestamp_ns;
    frame.receive_steady_time = std::chrono::steady_clock::now();
    frame.capture_timestamp_ns = metadata.capture_timestamp_ns;
    frame.geometry = ConvertGeometry(metadata);
    frame.sequence = metadata.frame_sequence;
    frame.source_invalid_frames = impl_->invalid_frames;
    return GrabStatus::OK;
  }
  return GrabStatus::TIMEOUT;
}

CameraInfo TalosDevice::Info() const {
  return impl_->info;
}

bool TalosDevice::IsOpen() const noexcept {
  return impl_->is_open;
}

}  // namespace mv::hal::detail
