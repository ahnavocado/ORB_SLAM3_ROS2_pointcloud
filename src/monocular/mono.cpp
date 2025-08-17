#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

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
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    // Creates the ROS2 node, passing the SLAM system to it
    auto node = std::make_shared<MonocularSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;


    // Starts the ROS2 event loop, which allows the node to process data
    rclcpp::spin(node);

    // Shuts down ROS2 when the node is stopped
    rclcpp::shutdown();

    return 0;
}