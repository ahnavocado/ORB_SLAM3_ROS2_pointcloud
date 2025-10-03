#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(const std::string& vocab,
                                     const std::string& settings,
                                     bool use_viewer)
:   Node("ORB_SLAM3_ROS2_pointcloud")
{
    bool visualization = true;
    // bool visualization = use_viewer;


    RCLCPP_INFO(this->get_logger(), "Initializing MonocularSlamNode...");
    // Create the main ORB_SLAM3 system object
    mORB_SLAM3 = new ORB_SLAM3::System(vocab, settings, ORB_SLAM3::System::MONOCULAR, visualization);
    RCLCPP_INFO(this->get_logger(), "Initialized MonocularSlamNode...");


    // Get pointers to SLAM components for later usage
    mpLocalMapping = mORB_SLAM3->mpLocalMapper;
    mpMapDrawer    = mORB_SLAM3->mpMapDrawer;
    mpAtlas        = mORB_SLAM3->mpAtlas;

    // Example publishers (currently commented out)
    // mPosePub = this->create_publisher<nav_msgs::msg::Odometry>(this->get_name() + std::string("/Pose"), 10);
    // ready_pub_ = this->create_publisher<std_msgs::msg::Empty>("ready_to_go", 10);

    // Log initialization
    RCLCPP_INFO(this->get_logger(), "MonocularSlamNode initialized with vocab: %s, settings: %s",
                vocab.c_str(), settings.c_str());
}

void MonocularSlamNode::init_viewer()
{
    // Initialize custom viewer component
    auto self = this->shared_from_this();

    // Logging pointers to verify initialization
    RCLCPP_DEBUG(this->get_logger(), "mpLocalMapping pointer: %p", mpLocalMapping);
    RCLCPP_DEBUG(this->get_logger(), "mpMapDrawer pointer: %p", mpMapDrawer);
    RCLCPP_DEBUG(this->get_logger(), "mpAtlas pointer: %p", mpAtlas);

    // Start the viewer in a separate thread
    ros_viewer_ = new viewer(self, mpLocalMapping, mORB_SLAM3->mpFrameDrawer, mpMapDrawer, false);
    viewer_thread_ = std::thread(&viewer::run, ros_viewer_);

    RCLCPP_INFO(this->get_logger(), "Viewer thread started");
}

void MonocularSlamNode::init_subscribers()
{
    // Subscribe to the camera topic and bind callback
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, _1));

    RCLCPP_INFO(this->get_logger(), "Image subscriber initialized on topic: 'camera'");
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Gracefully stop viewer thread if running
    if (ros_viewer_) {
        RCLCPP_WARN(this->get_logger(), "Viewer termination requested");
        ros_viewer_->request_terminate();
    }

    if (viewer_thread_.joinable()) {
        RCLCPP_WARN(this->get_logger(), "Joining viewer thread...");
        viewer_thread_.join();
    }

    // Stop SLAM threads
    RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3...");
    mORB_SLAM3->Shutdown();

    // Save trajectory
    RCLCPP_INFO(this->get_logger(), "Saving keyframe trajectory to KeyFrameTrajectory.txt");
    mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    delete mORB_SLAM3;
    mORB_SLAM3 = nullptr;

    delete ros_viewer_;
    ros_viewer_ = nullptr;
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Convert ROS2 image message to OpenCV Mat
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Run tracking with the monocular image
    Sophus::SE3f Tcw = mORB_SLAM3->TrackMonocular(
        m_cvImPtr->image,
        Utility::StampToSec(msg->header.stamp));

    RCLCPP_DEBUG(this->get_logger(), "Frame processed at stamp: %ld", msg->header.stamp.sec);

    // Update viewer with the latest pose
    if (ros_viewer_)
    {
        ros_viewer_->mTcw = Tcw;
        ros_viewer_->setStamp(msg->header.stamp);
        ros_viewer_->setIsStart(true);
        RCLCPP_DEBUG(this->get_logger(), "Viewer updated with new pose");
    }
}
