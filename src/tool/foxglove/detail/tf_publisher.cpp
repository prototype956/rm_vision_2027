/**
 * @file tf_publisher.cpp
 * @brief TfPublisher 实现
 *
 * 推送到 Foxglove 标准 /tf topic（FrameTransforms schema）。
 * 每次调用 Publish() 发送一条 FrameTransform，Foxglove 3D 面板
 * 会自动根据 parent/child 关系构建坐标系树。
 */
#include "tool/foxglove/detail/tf_publisher.hpp"

#include "tool/foxglove/detail/utils.hpp"

#include <spdlog/spdlog.h>

namespace mv::tool::detail {

TfPublisher::TfPublisher(foxglove::Context ctx) : ctx_(std::move(ctx)) {}

void TfPublisher::EnsureChannel() {
  if (!tf_ch_.has_value()) {
    auto res = foxglove::schemas::FrameTransformsChannel::create("/tf", ctx_);
    if (res.has_value()) {
      tf_ch_.emplace(std::move(res.value()));
    } else {
      spdlog::error("[TfPublisher] Failed to create /tf channel: {}",
                    foxglove::strerror(res.error()));
    }
  }
}

void TfPublisher::Publish(const std::string& parent, const std::string& child,
                          const Eigen::Matrix4d& T, uint64_t ts_ns) {
  std::lock_guard<std::mutex> lock(mtx_);
  EnsureChannel();
  if (!tf_ch_.has_value()) {
    return;
  }

  // 从 4x4 变换矩阵提取平移和旋转四元数
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d t = T.block<3, 1>(0, 3);
  Eigen::Quaterniond q(R);
  q.normalize();

  foxglove::schemas::FrameTransform tf;
  tf.timestamp = ToTs(ts_ns);
  tf.parent_frame_id = parent;
  tf.child_frame_id = child;
  tf.translation = foxglove::schemas::Vector3{t.x(), t.y(), t.z()};
  tf.rotation = foxglove::schemas::Quaternion{q.x(), q.y(), q.z(), q.w()};

  foxglove::schemas::FrameTransforms msg;
  msg.transforms.push_back(std::move(tf));

  tf_ch_->log(msg, ts_ns);
}

}  // namespace mv::tool::detail
