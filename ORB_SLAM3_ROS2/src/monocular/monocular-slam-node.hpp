#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

#include "viewer.hpp" 

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(const std::string& vocab,
                      const std::string& settings,
                      bool use_viewer = true);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;
    
    // ORB_SLAM3 system
    ORB_SLAM3::System *mORB_SLAM3;
    ORB_SLAM3::System::eSensor mSensor;
    ORB_SLAM3::Atlas *mpAtlas;
    ORB_SLAM3::LocalMapping *mpLocalMapping;
    ORB_SLAM3::MapDrawer *mpMapDrawer;

    cv_bridge::CvImagePtr m_cvImPtr;

    // Viewer
    viewer *ros_viewer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
};

#endif