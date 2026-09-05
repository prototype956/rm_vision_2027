#pragma once

#include "runtime/vision_pipeline.hpp"

#include <cstdint>
#include <mutex>
#include <string>

#include <optional>

namespace mv::runtime {

/** @brief Web 或其他控制面提交的一次完整前端调参事务。 */
struct VisionTuningRequest {
  std::uint64_t request_id{0};
  std::uint64_t target_revision{0};
  VisionFrontendTuningConfig config;
};

enum class VisionTuningSubmitStatus { ACCEPTED, REVISION_CONFLICT, PENDING_REQUEST };

struct VisionTuningSubmitResult {
  VisionTuningSubmitStatus status{VisionTuningSubmitStatus::REVISION_CONFLICT};
  std::uint64_t request_id{0};
  std::uint64_t target_revision{0};
};

struct VisionTuningApplied {
  std::uint64_t request_id{0};
  std::uint64_t source_sequence{0};
};

/** @brief 供状态页读取的调参事务和当前生效配置快照。 */
struct VisionTuningSnapshot {
  VisionFrontendTuningConfig startup_config;
  VisionFrontendTuningConfig active_config;
  std::uint64_t active_revision{1};
  bool pending{false};
  std::uint64_t pending_request_id{0};
  std::uint64_t target_revision{0};
  std::optional<std::uint64_t> last_applied_sequence;
  std::string last_error;
};

/** @brief 跨 Web 与视觉线程交接至多一个待处理调参事务。 */
class VisionTuningMailbox final {
 public:
  explicit VisionTuningMailbox(VisionFrontendTuningConfig startup_config);

  [[nodiscard]] VisionTuningSubmitResult Submit(std::uint64_t base_revision,
                                                const VisionFrontendTuningConfig& config);
  [[nodiscard]] std::optional<VisionTuningRequest> TryTakePending();
  void MarkApplied(const VisionTuningApplied& applied);
  void MarkFailed(std::uint64_t request_id, std::string message);
  [[nodiscard]] VisionTuningSnapshot Snapshot() const;

 private:
  mutable std::mutex mutex_;
  VisionFrontendTuningConfig startup_config_;
  VisionFrontendTuningConfig active_config_;
  std::uint64_t active_revision_{1};
  std::uint64_t next_request_id_{1};
  std::optional<VisionTuningRequest> pending_;
  bool pending_taken_{false};
  std::optional<std::uint64_t> last_applied_sequence_;
  std::string last_error_;
};

}  // namespace mv::runtime
