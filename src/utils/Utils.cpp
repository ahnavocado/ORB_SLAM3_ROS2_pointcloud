//
// Created by vuong on 16/12/2021.
//
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>

#include "utils/Utils.h"
namespace Utils {

void toTransformMsg(const Sophus::SE3f& Twc, geometry_msgs::msg::Transform* tf)
{
  // translation
  tf->translation.x = static_cast<double>(Twc.translation().x());
  tf->translation.y = static_cast<double>(Twc.translation().y());
  tf->translation.z = static_cast<double>(Twc.translation().z());

  // rotation (Sophus -> Eigen quaternion)
  const auto q = Twc.unit_quaternion();
  tf->rotation.w = static_cast<double>(q.w());
  tf->rotation.x = static_cast<double>(q.x());
  tf->rotation.y = static_cast<double>(q.y());
  tf->rotation.z = static_cast<double>(q.z());
}

rclcpp::Time toROSTime(double timestamp)
{
  // 초 → 나노초 정수
  const int64_t nsec = static_cast<int64_t>(timestamp * 1e9);
  return rclcpp::Time(nsec);
}
}