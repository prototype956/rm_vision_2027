#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mv::geometry {

using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;

/**
 * @brief 将子坐标系中的点和向量变换到父坐标系。
 *
 * 对于名为 parent_t_child 的实例，translation 是子坐标系原点在父坐标系
 * 中的位置，rotation 将子坐标系向量旋转到父坐标系。
 */
struct RigidTransform {
  Vector3 translation{Vector3::Zero()};
  Quaternion rotation{Quaternion::Identity()};
};

/** @brief 复合 parent_t_middle 与 middle_t_child，得到 parent_t_child。 */
[[nodiscard]] inline RigidTransform Compose(const RigidTransform& parent_t_middle,
                                            const RigidTransform& middle_t_child) noexcept {
  return {.translation =
              parent_t_middle.translation + parent_t_middle.rotation * middle_t_child.translation,
          .rotation = parent_t_middle.rotation * middle_t_child.rotation};
}

/** @brief 对 parent_t_child 求逆，得到 child_t_parent。 */
[[nodiscard]] inline RigidTransform Inverse(const RigidTransform& parent_t_child) noexcept {
  const Quaternion CHILD_Q_PARENT = parent_t_child.rotation.conjugate();
  return {.translation = -(CHILD_Q_PARENT * parent_t_child.translation),
          .rotation = CHILD_Q_PARENT};
}

/** @brief 将 child 中的点变换到 parent。 */
[[nodiscard]] inline Vector3 TransformPoint(const RigidTransform& parent_t_child,
                                            const Vector3& point_child) noexcept {
  return parent_t_child.translation + parent_t_child.rotation * point_child;
}

/** @brief 将 child 中的向量旋转到 parent，不应用平移。 */
[[nodiscard]] inline Vector3 TransformVector(const RigidTransform& parent_t_child,
                                             const Vector3& vector_child) noexcept {
  return parent_t_child.rotation * vector_child;
}

}  // namespace mv::geometry
