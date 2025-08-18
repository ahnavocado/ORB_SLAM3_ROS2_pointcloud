//
// Created by vuong on 16/12/2021.
//

#ifndef ORBSLAM3_ROS2_POINTCLOUD_UTILS_H 
#define ORBSLAM3_ROS2_POINTCLOUD_UTILS_H

// From ORB_SLAM3
#include <MapPoint.h>

// From ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>

// From Sophus
#include <sophus/se3.hpp>

namespace Utils {

void toTransformMsg(Sophus::SE3f Twc, geometry_msgs::msg::Transform* tf);
rclcpp::Time toROSTime(double timestamp);
//void TftoTarget()
};

#endif  // ORBSLAM3_ROS2_POINTCLOUD_UTILS_H

