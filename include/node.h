#ifndef NODE_H
#define NODE_H

#include <System.h>
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "MapDrawer.h"
#include "FrameDrawer.h"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Eigen>
#include <sophus/se3.hpp>

#include "viewer.h" // Assuming this is also ported or compatible
#include "utils/Utils.h" // Assuming this is also ported or compatible

// For rosbag2
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>


class node : public rclcpp::Node {
public:
    node(ORB_SLAM3::System::eSensor sensor, const rclcpp::NodeOptions &options);
    ~node();

    void Update(Sophus::SE3f Tcw, double timestamp);

private:
    // ROS2 specific members
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mPosePub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr trajectory_saver_service_;

    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Clock::SharedPtr clock_;

    // ORB_SLAM3 system
    ORB_SLAM3::System *mORB_SLAM3;
    ORB_SLAM3::System::eSensor mSensor;
    ORB_SLAM3::Atlas *mpAtlas;
    ORB_SLAM3::LocalMapping *mpLocalMapping;
    ORB_SLAM3::MapDrawer *mpMapDrawer;

    // Parameters
    std::string strVocFile, strSettingsFile, strOutputFile;
    bool mbViewer;
    std::string map_frame_id_, odom_frame_id_, left_cam_frame_id_;
    double transform_tolerance_;
    bool mbIMU;
    bool should_start_publish_ = false;

    // Coordinate transforms
    Eigen::Matrix4f T_ros_cam = Eigen::Matrix4f::Identity();
    Sophus::SE3f T_ros_cam_se3_;
    Eigen::Matrix3d rot_ros_to_cv_map_frame_;
    Eigen::Matrix4f T_ROS_ORB = Eigen::Matrix4f::Identity();
    Sophus::SE3f spT_ROS_ORB;

    // Viewer
    viewer *ros_viewer_;

    // Camera Info
    float fx, fy, cx, cy, k1, k2, t1, t2;
    int camWidth, camHeight;

    // Service callback
    void SaveTrajectorySrv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response);

    // Publishing functions
    void PublishPoseAsTransform(const Sophus::SE3f &Twc, double timestamp);
    void PublishPoseAsOdometry(const Sophus::SE3d &Twc, double timestamp);

    // Helper functions
    void GetCamInfo(cv::FileStorage &fSettings);
    void ParseCamInfo(sensor_msgs::msg::CameraInfo &msg) const;
    void SavingTrajectory();
    bool SaveTrajectoryBag(const std::string &file_path);

    // Bag writing
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

#endif // NODE_H
