#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(const std::string& vocab,
                    const std::string& settings,
                bool use_viewer)
:   Node("ORB_SLAM3_ROS2_pointcloud")
{

    bool visualization = true;
    // bool visualization = use_viewer;
    // Creates the main ORB_SLAM3 system object
    mORB_SLAM3 = new ORB_SLAM3::System(vocab, settings, ORB_SLAM3::System::MONOCULAR, visualization);


    // Get pointers to SLAM components
    mpLocalMapping = mORB_SLAM3->mpLocalMapper;
    mpMapDrawer = mORB_SLAM3->mpMapDrawer;
    mpAtlas = mORB_SLAM3->mpAtlas;
    

    // Create Publishers
    // mPosePub = this->create_publisher<nav_msgs::msg::Odometry>(this->get_name() + std::string("/Pose"), 10);
    // ready_pub_ = this->create_publisher<std_msgs::msg::Empty>("ready_to_go", 10);


}

void MonocularSlamNode::init_viewer()
{
   // Initialize ROS viewer
    // ros_viewer_ = new viewer(this, mpLocalMapping, mORB_SLAM3->mpFrameDrawer, mpMapDrawer, mbIMU);
  auto self = this->shared_from_this();  
  
    // ROS2 logger로 출력 (노드 이름과 함께)
    RCLCPP_INFO(this->get_logger(), "mpLocalMapping pointer: %p", mpLocalMapping);
    RCLCPP_INFO(this->get_logger(), "mpMapDrawer pointer: %p", mpMapDrawer);
    RCLCPP_INFO(this->get_logger(), "mpAtlas pointer: %p", mpAtlas);


//   ros_viewer_ = std::make_shared<viewer>(self, mpLocalMapping, mORB_SLAM3->mpFrameDrawer, mpMapDrawer, false);
  ros_viewer_ = new viewer(self, mpLocalMapping, mORB_SLAM3->mpFrameDrawer, mpMapDrawer, false);
  viewer_thread_ = std::thread(&viewer::run, ros_viewer_);
  
}
void MonocularSlamNode::init_subscribers()
{
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, _1));

    RCLCPP_INFO(this->get_logger(), "Image subscriber initialized on topic: 'camera'");
}

MonocularSlamNode::~MonocularSlamNode()
{
    if (ros_viewer_) {
        std::cout << "1 You have come to a wrong place " << std::endl;
        ros_viewer_->request_terminate();
    }

    if (viewer_thread_.joinable()) {
        std::cout << "2 You have come to a wrong place " << std::endl;
        viewer_thread_.join();
    }   


    // Stop all threads
    mORB_SLAM3->Shutdown();

    // Save camera trajectory
    mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    delete mORB_SLAM3;
    mORB_SLAM3 = nullptr;

    delete ros_viewer_;
    ros_viewer_ = nullptr;
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

    // std::cout<<"one frame has been sent (ORB_SLAM3_ROS2_pointcloud)"<<std::endl;
    mORB_SLAM3->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    Sophus::SE3f Tcw = mORB_SLAM3->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));


    if (ros_viewer_)
    {
        std::cout << "0 ros_viewer_ is madeeeeeee" << std::endl;
        ros_viewer_->mTcw = Tcw;
        ros_viewer_->setStamp(msg->header.stamp);
        ros_viewer_->setIsStart(true);
    }


}