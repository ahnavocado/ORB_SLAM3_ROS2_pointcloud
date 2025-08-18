#include "node.h"
#include "viewer.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

// Constructor: Replaces the old constructor and the Init() method
node::node(ORB_SLAM3::System::eSensor sensor, const rclcpp::NodeOptions &options)
        : rclcpp::Node("orbslam3_node", options), mSensor(sensor) {

    RCLCPP_INFO(this->get_logger(), "Starting ORB_SLAM3 ROS2 node...");

    // Initialize ROS2 specific members
    clock_ = this->get_clock();
    tf_ = std::make_unique<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    mbIMU = (sensor == ORB_SLAM3::System::IMU_MONOCULAR ||
             sensor == ORB_SLAM3::System::IMU_STEREO);

    // Declare and retrieve parameters
    this->declare_parameter<std::string>("Vocab_path", "vocabulary/ORBvoc.txt");
    this->declare_parameter<std::string>("Params", "config/default.yaml");
    this->declare_parameter<bool>("Visualize", true);
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("left_camera_frame_id", "camera_link");
    this->declare_parameter<double>("transform_tolerance", 0.5);
    this->declare_parameter<std::string>("trajectory_output_path", "");


    this->get_parameter("Vocab_path", strVocFile);
    this->get_parameter("Params", strSettingsFile);
    this->get_parameter("Visualize", mbViewer);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("left_camera_frame_id", left_cam_frame_id_);
    this->get_parameter("transform_tolerance", transform_tolerance_);
    this->get_parameter("trajectory_output_path", strOutputFile);


    RCLCPP_INFO(this->get_logger(), "Map frame id: %s", map_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Odom frame id: %s", odom_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Left camera frame id: %s", left_cam_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings file: %s", strSettingsFile.c_str());


    T_ros_cam.block<3, 3>(0, 0) = Eigen::Quaternionf(T_ros_cam.block<3, 3>(0, 0))
            .normalized()
            .toRotationMatrix();
    T_ros_cam_se3_ = Sophus::SE3f(T_ros_cam);

    // Init ORB_SLAM3
    mORB_SLAM3 = new ORB_SLAM3::System(strVocFile, strSettingsFile, mSensor, mbViewer);

    // Retrieve Camera Info
    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    GetCamInfo(fSettings);

    // Get pointers to SLAM components
    mpLocalMapping = mORB_SLAM3->mpLocalMapper;
    mpMapDrawer = mORB_SLAM3->mpMapDrawer;
    mpAtlas = mORB_SLAM3->mpAtlas; 

    // Initialize ROS viewer
    ros_viewer_ = new viewer(this, mpLocalMapping, mORB_SLAM3->mpFrameDrawer, mpMapDrawer, mbIMU);

    // Coordinate transformations
    rot_ros_to_cv_map_frame_ = (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
    T_ROS_ORB << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
    T_ROS_ORB.block<3, 3>(0, 0) = Eigen::Quaternionf(T_ROS_ORB.block<3, 3>(0, 0)).normalized().toRotationMatrix();
    spT_ROS_ORB = Sophus::SE3f(T_ROS_ORB);

    // Create Publishers
    mPosePub = this->create_publisher<nav_msgs::msg::Odometry>(this->get_name() + std::string("/Pose"), 10);
    ready_pub_ = this->create_publisher<std_msgs::msg::Empty>("ready_to_go", 10);

    // Create Service
    trajectory_saver_service_ = this->create_service<std_srvs::srv::Empty>(
        "/orbslam3_ros2_pointcloud/save_trajectory",
        std::bind(&node::SaveTrajectorySrv, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "orbslam3_ros2_pointcloud started!");
}

node::~node() {
    // Destructor logic if needed
    delete mORB_SLAM3;
    delete ros_viewer_;
}

void node::SaveTrajectorySrv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                             std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request; // Unused parameter
    (void)response; // Unused parameter
    RCLCPP_INFO(this->get_logger(), "Save trajectory service called.");
    SavingTrajectory();
    if(!strOutputFile.empty())
    {
        SaveTrajectoryBag(strOutputFile + ".bag");
    }
}


void node::Update(Sophus::SE3f Tcw, double timestamp) {
    if (!should_start_publish_ && mbIMU) {
        if (!mpAtlas->GetCurrentMap()->GetIniertialBA2()) {
            should_start_publish_ = false;
            return;
        } else {
            should_start_publish_ = true;
            ready_pub_->publish(std_msgs::msg::Empty());
        }
    } else if (!should_start_publish_ && !mbIMU)
        should_start_publish_ = true;

    rclcpp::Time ros_time(static_cast<int64_t>(timestamp * 1e9));
    ros_viewer_->setStamp(ros_time);
    ros_viewer_->setIsStart(true);

    PublishPoseAsTransform(Tcw.inverse(), timestamp);
}

void node::PublishPoseAsTransform(const Sophus::SE3f &Twc, double timestamp) {
    geometry_msgs::msg::TransformStamped tfMsg;
    nav_msgs::msg::Odometry pose_msg;

    Sophus::SE3f Two = Twc * T_ros_cam_se3_.inverse();
    Eigen::Affine3d map_to_camera_affine(
            Eigen::Translation3d(Two.translation().cast<double>()) * Two.unit_quaternion().matrix().cast<double>());

    if (!mbIMU)
        map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame_);

    auto stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_id_;
    pose_msg.child_frame_id = left_cam_frame_id_;

    if (!mbIMU)
        pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine * rot_ros_to_cv_map_frame_.inverse());
    else
        pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine);

    mPosePub->publish(pose_msg);

    try {
        geometry_msgs::msg::TransformStamped map_to_camera_link_msg;
        if (!mbIMU)
            map_to_camera_link_msg = tf2::toMsg(map_to_camera_affine * rot_ros_to_cv_map_frame_.inverse());
        else
            map_to_camera_link_msg = tf2::toMsg(map_to_camera_affine);

        map_to_camera_link_msg.header.stamp = stamp;
        map_to_camera_link_msg.header.frame_id = odom_frame_id_;
        map_to_camera_link_msg.child_frame_id = left_cam_frame_id_;
        tf_broadcaster_->sendTransform(map_to_camera_link_msg);
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

void node::PublishPoseAsOdometry(const Sophus::SE3d &Twc, double timestamp) {
    nav_msgs::msg::Odometry PoseMsg;
    PoseMsg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    PoseMsg.header.frame_id = "map";

    PoseMsg.pose.pose.orientation.x = Twc.unit_quaternion().x();
    PoseMsg.pose.pose.orientation.y = Twc.unit_quaternion().y();
    PoseMsg.pose.pose.orientation.z = Twc.unit_quaternion().z();
    PoseMsg.pose.pose.orientation.w = Twc.unit_quaternion().w();

    PoseMsg.pose.pose.position.x = Twc.translation().x();
    PoseMsg.pose.pose.position.y = Twc.translation().y();
    PoseMsg.pose.pose.position.z = Twc.translation().z();

    mPosePub->publish(PoseMsg);
}

void node::ParseCamInfo(sensor_msgs::msg::CameraInfo &msg) const {
    msg.width = camWidth;
    msg.height = camHeight;
    msg.distortion_model = "plumb_bob";
    msg.k[0] = fx; msg.k[1] = 0.0; msg.k[2] = cx;
    msg.k[3] = 0.0; msg.k[4] = fy; msg.k[5] = cy;
    msg.k[6] = 0.0; msg.k[7] = 0.0; msg.k[8] = 1.0;
    msg.d.resize(4);
    msg.d[0] = k1; msg.d[1] = k2; msg.d[2] = t1; msg.d[3] = t2;
    msg.r[0] = 1.0; msg.r[1] = 0.0; msg.r[2] = 0.0;
    msg.r[3] = 0.0; msg.r[4] = 1.0; msg.r[5] = 0.0;
    msg.r[6] = 0.0; msg.r[7] = 0.0; msg.r[8] = 1.0;
    msg.p[0] = fx; msg.p[1] = 0.0; msg.p[2] = cx; msg.p[3] = 0.0;
    msg.p[4] = 0.0; msg.p[5] = fy; msg.p[6] = cy; msg.p[7] = 0.0;
    msg.p[8] = 0.0; msg.p[9] = 0.0; msg.p[10] = 1.0; msg.p[11] = 0.0;
}

void node::GetCamInfo(cv::FileStorage &fSettings) {
    bool b_miss_params = false;
    cv::FileNode node = fSettings["Camera1.fx"];
    if (!node.empty() && node.isReal()) { fx = node.real(); }
    else { std::cerr << "*Camera.fx parameter missing*" << std::endl; b_miss_params = true; }
    node = fSettings["Camera1.fy"];
    if (!node.empty() && node.isReal()) { fy = node.real(); }
    else { std::cerr << "*Camera.fy parameter missing*" << std::endl; b_miss_params = true; }
    node = fSettings["Camera1.cx"];
    if (!node.empty() && node.isReal()) { cx = node.real(); }
    else { std::cerr << "*Camera.cx parameter missing*" << std::endl; b_miss_params = true; }
    node = fSettings["Camera1.cy"];
    if (!node.empty() && node.isReal()) { cy = node.real(); }
    else { std::cerr << "*Camera.cy parameter missing*" << std::endl; b_miss_params = true; }
    node = fSettings["Camera1.k1"];
    if (!node.empty() && node.isReal()) { k1 = node.real(); }
    else { std::cerr << "*Camera.k1 parameter missing*" << std::endl; b_miss_params = true; }
    node = fSettings["Camera1.k2"];
    if (!node.empty() && node.isReal()) { k2 = node.real(); }
    else { std::cerr << "*Camera.k2 parameter missing*" << std::endl; b_miss_params = true; }
    node = fSettings["Camera1.p1"];
    if (!node.empty() && node.isReal()) { t1 = node.real(); }
    else { std::cerr << "*Camera.p1 parameter missing*" << std::endl; b_miss_params = true; }
    node = fSettings["Camera1.p2"];
    if (!node.empty() && node.isReal()) { t2 = node.real(); }
    else { std::cerr << "*Camera.p2 parameter missing*" << std::endl; b_miss_params = true; }
    camWidth = (int) fSettings["Camera1.width"].real();
    camHeight = (int) fSettings["Camera1.height"].real();
    if(b_miss_params) {
        RCLCPP_WARN(this->get_logger(), "MISSING CAMERA PARAMS");
    }
}

void node::SavingTrajectory() {
    mORB_SLAM3->Shutdown();
    RCLCPP_INFO(this->get_logger(), "Saving trajectory...");
    if (strOutputFile.empty()) {
        mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        mORB_SLAM3->SaveTrajectoryTUM("FrameTrajectory.txt");
    } else {
        mORB_SLAM3->SaveKeyFrameTrajectoryTUM("kf_" + strOutputFile + ".txt");
        mORB_SLAM3->SaveTrajectoryTUM("f_" + strOutputFile + ".txt");
    }
    RCLCPP_INFO(this->get_logger(), "Saved trajectory!");
}

bool node::SaveTrajectoryBag(const std::string &file_path) {
    RCLCPP_INFO(this->get_logger(), "Saving as Bag file .... %s", file_path.c_str());

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions storage_options({file_path, "sqlite3"});
    rclcpp::ConverterOptions converter_options;
    writer_->open(storage_options, converter_options);

    std::string topic_kf_pose = std::string(this->get_name()) + "/KF_Pose";
    std::string topic_kf_info = std::string(this->get_name()) + "/KF_CamInfo";

    writer_->create_topic({topic_kf_pose, "nav_msgs/msg/Odometry", rmw_get_serialization_format(), ""});
    writer_->create_topic({topic_kf_info, "sensor_msgs/msg/CameraInfo", rmw_get_serialization_format(), ""});

    std::vector<ORB_SLAM3::KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

    rclcpp::Serialization<nav_msgs::msg::Odometry> odometry_serialization;
    rclcpp::Serialization<sensor_msgs::msg::CameraInfo> caminfo_serialization;

    sensor_msgs::msg::CameraInfo leftInfo;
    leftInfo.header.frame_id = "left_camera";
    ParseCamInfo(leftInfo);

    for (auto pKF: vpKFs) {
        if (pKF->isBad()) continue;

        rclcpp::Time timestamp(static_cast<int64_t>(pKF->mTimeStamp * 1e9));

        nav_msgs::msg::Odometry pose_msg;
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = odom_frame_id_;
        pose_msg.child_frame_id = left_cam_frame_id_;

        Sophus::SE3f spTcw = pKF->GetPoseInverse();
        pose_msg.pose.pose.orientation.x = spTcw.unit_quaternion().x();
        pose_msg.pose.pose.orientation.y = spTcw.unit_quaternion().y();
        pose_msg.pose.pose.orientation.z = spTcw.unit_quaternion().z();
        pose_msg.pose.pose.orientation.w = spTcw.unit_quaternion().w();
        pose_msg.pose.pose.position.x = spTcw.translation().x();
        pose_msg.pose.pose.position.y = spTcw.translation().y();
        pose_msg.pose.pose.position.z = spTcw.translation().z();

        auto serialized_pose_msg = std::make_shared<rclcpp::SerializedMessage>();
        odometry_serialization.serialize_message(&pose_msg, serialized_pose_msg.get());
        writer_->write(serialized_pose_msg, topic_kf_pose, timestamp);

        leftInfo.header.stamp = timestamp;
        auto serialized_info_msg = std::make_shared<rclcpp::SerializedMessage>();
        caminfo_serialization.serialize_message(&leftInfo, serialized_info_msg.get());
        writer_->write(serialized_info_msg, topic_kf_info, timestamp);
    }

    writer_->close();
    RCLCPP_INFO(this->get_logger(), "Successfully saved in: %s", file_path.c_str());
    return true;
}
