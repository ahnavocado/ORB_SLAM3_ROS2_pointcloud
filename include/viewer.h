//
// Created by vuong on 4/27/22.
// Ported to ROS2
//

#ifndef ORB_SLAM3_ROS_VIEWER_H
#define ORB_SLAM3_ROS_VIEWER_H

// std
#include <string>
#include <vector>
#include <mutex>
#include <thread>

// ros2
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/srv/empty.hpp>

//ORB_SLAM3
#include <LocalMapping.h>
#include <System.h>
#include <MapPoint.h>
#include <FrameDrawer.h>
#include <MapDrawer.h>

//eigen
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;


class viewer {
public:
    viewer(rclcpp::Node* node, ORB_SLAM3::LocalMapping *pLocalMapping, ORB_SLAM3::FrameDrawer *pFrameDrawer,
           ORB_SLAM3::MapDrawer *pMapDrawer, bool is_imu = false);

    void run();
    void publish_path(const rclcpp::Time &stamp);
    void publish_map_point(const rclcpp::Time &stamp);
    void publish_local_map_point(const rclcpp::Time &stamp);
    void publish_debug_image(const rclcpp::Time &stamp);
    void setIsStart(bool isStart);
    void setStamp(const rclcpp::Time &stamp);
    void request_terminate();
    bool is_terminated();
    void set_params();
    void set_advertising();
    void publish_reference_cloud();

    Sophus::SE3f T_ros_cam_se3_;
    ORB_SLAM3::KeyFrame *pRefKF;
    Sophus::SE3f mTcw;

private:
    rclcpp::Time stamp_;
    bool is_start_ = false;

    // ORB_SLAM3 pointers
    bool is_imu_ = false;
    ORB_SLAM3::FrameDrawer *mpFrameDrawer_ = nullptr;
    ORB_SLAM3::LocalMapping *mpLocalMapping_ = nullptr;
    ORB_SLAM3::MapDrawer *mpMapDrawer_ = nullptr;

    // ros pointers
    rclcpp::Node* node_;
    image_transport::ImageTransport it_;

    // Publishers
    image_transport::Publisher debug_frame_publisher;
    std::string debug_frame_topic = "debug_image";

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    std::string path_topic = "path";
    nav_msgs::msg::Path path;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher;
    std::string map_points_topic = "map_points";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_points_publisher;
    std::string local_map_points_topic = "local_map_points";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reference_map_points_publisher;
    std::string reference_map_points_topic = "reference_map_points";

    // Service
    void SaveGlobalMapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response);
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_map_saver;

    // frame id
    std::string map_frame_id_;
    std::string left_camera_frame_id_;

    // ref cloud
    bool use_reference_cloud_;
    std::string reference_cloud_path_;
    PointCloudXYZ reference_cloud;

    int min_observations_;
    bool only_quality_observations_;
    bool judge_optimized_;
    int min_local_points_;

private:
    Eigen::Matrix4f T_ros_cam = (Eigen::Matrix4f() << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 0, 1).finished();

    mutable std::mutex mtx_terminate_;
    bool terminate_is_requested();
    void terminate();
    bool terminate_is_requested_ = false;
    bool is_terminated_ = true;
};

#endif//ORB_SLAM3_ROS_VIEWER_H
