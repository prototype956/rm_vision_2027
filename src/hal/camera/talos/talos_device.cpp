#include "talos_device.hpp"

#include "core/logger.hpp"

#include <chrono>
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
constexpr uint32_t K_MIN_VERSION = 2;
constexpr uint32_t K_MAX_VERSION = 3;
constexpr uint8_t K_FLAG_NEW = 0x80;
constexpr uint8_t K_INDEX_MASK = 0x03;
constexpr uint8_t K_FORMAT_RGB8 = 0;
constexpr uint8_t K_FORMAT_BGR8 = 1;
constexpr std::size_t K_BUFFER_COUNT = 3;
constexpr std::size_t K_SYNC_POSE_COUNT = 4;
constexpr std::size_t K_META_REGION_SIZE = 3712;

struct alignas(64) ShmHeader {
  uint32_t magic;
  uint32_t version;
  uint64_t created_ns;
  uint64_t heartbeat_ns;
  uint32_t image_width;
  uint32_t image_height;
  uint8_t pad[32];
};

struct alignas(32) ImageMeta {
  uint64_t sequence;
  uint64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
  uint8_t buffer_id;
  uint8_t format;
  uint8_t pad[6];
};

struct alignas(64) ImageTripleBuffer {
  uint8_t state;
  uint8_t write_index;
  uint8_t read_index;
  uint8_t pad[61];
  ImageMeta slots[3];
};

struct alignas(64) PoseMeta {
  uint64_t frame_sequence;
  float position[3];
  float quaternion[4];
  uint64_t timestamp_ns;
  uint8_t pad[16];
};

struct alignas(64) PoseTripleBuffer {
  uint8_t state;
  uint8_t write_index;
  uint8_t read_index;
  uint8_t pad[61];
  PoseMeta slots[3];
};

struct alignas(64) ShmMetaPrefix {
  ShmHeader header;
  ImageTripleBuffer image;
  PoseTripleBuffer poses[5];
};

// 编译期校验可防止字段或对齐方式变化后静默破坏跨进程协议。
static_assert(sizeof(ShmHeader) == 64);
static_assert(sizeof(ImageMeta) == 32);
static_assert(sizeof(ImageTripleBuffer) == 192);
static_assert(sizeof(PoseMeta) == 64);
static_assert(sizeof(PoseTripleBuffer) == 256);
static_assert(offsetof(ShmMetaPrefix, image) == 64);
static_assert(offsetof(ShmMetaPrefix, poses) == 256);
static_assert(sizeof(ShmMetaPrefix) == 1536);

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

}  // namespace

struct TalosDevice::Impl {
  TalosConfig config;                 ///< 最近一次 Open() 接收的类型化配置。
  int meta_fd{-1};                    ///< 可读写的元数据文件描述符。
  int image_pool_fd{-1};              ///< 只读图像池文件描述符。
  void* meta_mapping{nullptr};        ///< 元数据 mmap 起始地址。
  void* image_pool_mapping{nullptr};  ///< 图像池 mmap 起始地址。
  std::size_t image_pool_size{0};     ///< 已映射图像池的字节数。
  ShmMetaPrefix* meta{nullptr};       ///< 带类型的协议元数据视图。
  CameraInfo info;                    ///< Open() 成功后对上层公开的信息。
  bool is_open{false};                ///< 两个映射和发布端心跳均有效。

  /** 按映射、文件描述符的逆依赖顺序释放资源，并恢复关闭状态。 */
  void ResetMappings() noexcept {
    if (image_pool_mapping != nullptr) {
      ::munmap(image_pool_mapping, image_pool_size);
    }
    if (meta_mapping != nullptr) {
      ::munmap(meta_mapping, K_META_REGION_SIZE);
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
        meta_stat.st_size < static_cast<off_t>(K_META_REGION_SIZE)) {
      error = "metadata file has an invalid size";
      ResetMappings();
      return false;
    }
    meta_mapping =
        ::mmap(nullptr, K_META_REGION_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, meta_fd, 0);
    if (IsMapFailed(meta_mapping)) {
      meta_mapping = nullptr;
      error = "cannot map Talos metadata";
      ResetMappings();
      return false;
    }
    meta = static_cast<ShmMetaPrefix*>(meta_mapping);

    const uint32_t VERSION = meta->header.version;
    const uint32_t IMAGE_WIDTH = meta->header.image_width;
    const uint32_t IMAGE_HEIGHT = meta->header.image_height;
    if (meta->header.magic != K_SHM_MAGIC || VERSION < K_MIN_VERSION || VERSION > K_MAX_VERSION) {
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

    ImageMeta image_meta{};
    if (!Consume<ImageMeta>(impl_->meta->image, image_meta)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    bool poses_synchronized = true;
    // 当前视觉帧只有在四辆机器人位姿都属于同一仿真帧时才可交给上层。
    for (std::size_t index = 0; index < K_SYNC_POSE_COUNT; ++index) {
      PoseMeta pose{};
      if (!Consume<PoseMeta>(impl_->meta->poses[index], pose) ||
          pose.frame_sequence != image_meta.sequence) {
        poses_synchronized = false;
      }
    }

    if (!poses_synchronized ||
        image_meta.width != static_cast<uint32_t>(impl_->info.output_width) ||
        image_meta.height != static_cast<uint32_t>(impl_->info.output_height) ||
        image_meta.buffer_id >= K_BUFFER_COUNT ||
        (image_meta.format != K_FORMAT_RGB8 && image_meta.format != K_FORMAT_BGR8)) {
      return GrabStatus::INVALID_FRAME;
    }

    const std::size_t FRAME_SIZE = static_cast<std::size_t>(image_meta.width) *
                                   static_cast<std::size_t>(image_meta.height) * 3;
    const std::size_t OFFSET = static_cast<std::size_t>(image_meta.buffer_id) * FRAME_SIZE;
    if (OFFSET > impl_->image_pool_size || FRAME_SIZE > impl_->image_pool_size - OFFSET) {
      return GrabStatus::INVALID_FRAME;
    }

    const auto* source = static_cast<const uint8_t*>(impl_->image_pool_mapping) + OFFSET;
    const cv::Mat SHARED_IMAGE(static_cast<int>(image_meta.height),
                               static_cast<int>(image_meta.width), CV_8UC3,
                               const_cast<uint8_t*>(source));
    // 发布端会复用共享三缓冲槽位，返回前必须复制或转换到独立拥有的 cv::Mat。
    if (image_meta.format == K_FORMAT_BGR8) {
      frame.image = SHARED_IMAGE.clone();
    } else {
      cv::cvtColor(SHARED_IMAGE, frame.image, cv::COLOR_RGB2BGR);
    }
    if (frame.image.empty()) {
      return GrabStatus::FATAL;
    }
    frame.timestamp = std::chrono::steady_clock::now();
    frame.sequence = image_meta.sequence;
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
