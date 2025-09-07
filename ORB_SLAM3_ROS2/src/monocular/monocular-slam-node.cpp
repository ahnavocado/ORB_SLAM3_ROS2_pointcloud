#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(const std::string& vocab,
                    const std::string& settings,
                bool use_viewer)
:   Node("ORB_SLAM3_ROS2_pointcloud")
{

    // Creates a subscription to the "camera" topic.
    // For every message received, it calls the "GrabImage" function.
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    bool visualization = false;
    // bool visualization = use_viewer;
    // Creates the main ORB_SLAM3 system object
    mORB_SLAM3 = new ORB_SLAM3::System(vocab, settings, ORB_SLAM3::System::MONOCULAR, visualization);

    // Get pointers to SLAM components
    mpLocalMapping = mORB_SLAM3->mpLocalMapper;
    mpMapDrawer = mORB_SLAM3->mpMapDrawer;
    mpAtlas = mORB_SLAM3->mpAtlas;

    // Initialize ROS viewer
    // ros_viewer_ = new viewer(this, mpLocalMapping, mORB_SLAM3->mpFrameDrawer, mpMapDrawer, mbIMU);

    // auto self = this->shared_from_this(); 
    // ros_viewer_ = new viewer(self, mpLocalMapping, mORB_SLAM3->mpFrameDrawer, mpMapDrawer, false);


    // Create Publishers
    // mPosePub = this->create_publisher<nav_msgs::msg::Odometry>(this->get_name() + std::string("/Pose"), 10);
    // ready_pub_ = this->create_publisher<std_msgs::msg::Empty>("ready_to_go", 10);


}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent (- ORB_SLAM3_ROS2_pointcloud)"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}