#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"
#include "viewer.h"

#include "System.h"


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }
    // Initializes ROS2
    rclcpp::init(argc, argv);

    bool visualization = true;
    // Creates the main ORB_SLAM3 system object
    mORB_SLAM3 = new ORB_SLAM3::System(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    // Creates the ROS2 node, passing the SLAM system to it
    auto node = std::make_shared<MonocularSlamNode>(&SLAM);

    
    mpLocalMapping = mORB_SLAM3->mpLocalMapper;
    mpMapDrawer = mORB_SLAM3->mpMapDrawer;
    mpAtlas = mORB_SLAM3->mpAtlas; 


    // Initialize ROS viewer
    // change the last parameter to false if you don't want to use IMU true if you want to use IMU
    
    ros_viewer_ = new viewer(this, mpLocalMapping, mORB_SLAM3->mpFrameDrawer, mpMapDrawer, false);

    std::cout << "============================ " << std::endl;

    // Starts the ROS2 event loop, which allows the node to process data
    rclcpp::spin(node);

    // Shuts down ROS2 when the node is stopped
    rclcpp::shutdown();
    delete mORB_SLAM3;
    delete ros_viewer_;

    return 0;
}