#include "viewer.hpp"

viewer::viewer(rclcpp::Node::SharedPtr node, ORB_SLAM3::LocalMapping *pLocalMapping, ORB_SLAM3::FrameDrawer *pFrameDrawer,
               ORB_SLAM3::MapDrawer *pMapDrawer, bool is_imu) :
        node_(node), it_(node), mpLocalMapping_(pLocalMapping), mpFrameDrawer_(pFrameDrawer), mpMapDrawer_(pMapDrawer),
        is_imu_(is_imu), pRefKF(nullptr) {

    // Initialize timestamp using the node's clock (ROS time or system time depending on use_sim_time)
    stamp_ = node_->get_clock()->now();

    // Load parameters and internal transforms (also logs parameter values via RCLCPP_* macros)
    set_params();

    // Create publishers and services (topics/services are advertised here)
    set_advertising();

    // Optionally publish a latched reference point cloud at startup
    if (use_reference_cloud_)
        publish_reference_cloud();
}

void viewer::set_params() {
    // Declare parameters with defaults; then read their values and log them.
    node_->declare_parameter<std::string>("map_frame_id", "map");
    node_->declare_parameter<std::string>("left_camera_frame_id", "camera_link");
    node_->declare_parameter<bool>("use_reference_cloud", false);
    node_->declare_parameter<std::string>("reference_cloud_path", "");

    node_->get_parameter("map_frame_id", map_frame_id_);
    RCLCPP_INFO(node_->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
    node_->get_parameter("left_camera_frame_id", left_camera_frame_id_);
    RCLCPP_INFO(node_->get_logger(), "left_camera_frame_id: %s", left_camera_frame_id_.c_str());
    node_->get_parameter("use_reference_cloud", use_reference_cloud_);
    RCLCPP_INFO(node_->get_logger(), "use_reference_cloud: %d", use_reference_cloud_);
    node_->get_parameter("reference_cloud_path", reference_cloud_path_);
    RCLCPP_INFO(node_->get_logger(), "reference_cloud_path: %s", reference_cloud_path_.c_str());

    // Load reference cloud if requested (no log on failure here; consider adding RCLCPP_ERROR if needed)
    if(use_reference_cloud_){
        pcl::io::loadPCDFile(reference_cloud_path_, reference_cloud);
    }

    // Ensure the rotation block of T_ros_cam is a valid rotation (normalized quaternion)
    T_ros_cam.block<3, 3>(0, 0) = Eigen::Quaternionf(T_ros_cam.block<3, 3>(0, 0))
            .normalized()
            .toRotationMatrix();
    T_ros_cam_se3_ = Sophus::SE3f(T_ros_cam);
}


void viewer::set_advertising() {
    // Image publisher for debug frames (sensor_msgs/Image) via image_transport
    debug_frame_publisher = it_.advertise(debug_frame_topic, 1);

    // Path publisher (nav_msgs/Path) for trajectory visualization
    path_publisher = node_->create_publisher<nav_msgs::msg::Path>(path_topic, 1);

    // Map points publishers (global and local) as PointCloud2
    map_points_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(map_points_topic, 1);
    local_map_points_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(local_map_points_topic, 1);

    // Reference cloud publisher uses transient_local QoS to behave like a latched topic
    reference_map_points_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(reference_map_points_topic, rclcpp::QoS(1).transient_local());

    // Service for saving the global map (PCD)
    global_map_saver = node_->create_service<std_srvs::srv::Empty>("/vslam2/save_global_map", 
        std::bind(&viewer::SaveGlobalMapSrv, this, std::placeholders::_1, std::placeholders::_2));
}

void viewer::SaveGlobalMapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    // Collect all valid map points from the active map and dump them to a PCD file
    (void)request;
    (void)response;
    ORB_SLAM3::Map *pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    const std::vector<ORB_SLAM3::MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
    PointCloudXYZ::Ptr p_global_cloud(new PointCloudXYZ);
    for (size_t i = 0; i < vpMPs.size(); i++) {
        const auto lm = vpMPs.at(i);
        if (lm->isBad()) continue;

        Eigen::Vector3f pos_w;
        if (is_imu_)
            pos_w = lm->GetWorldPos();
        else
            pos_w = T_ros_cam_se3_ * lm->GetWorldPos();
        pcl::PointXYZ point(pos_w.x(), pos_w.y(), pos_w.z());
        p_global_cloud->push_back(point);
    }
    pcl::io::savePCDFileASCII("global_map.pcd", *p_global_cloud);
    RCLCPP_INFO(node_->get_logger(), "Global cloud saved");
}

void viewer::publish_reference_cloud() {
    // Publish the preloaded reference cloud once; transient_local QoS keeps it available to late subscribers
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(reference_cloud, cloud_msg);
    cloud_msg.header.frame_id = map_frame_id_;
    cloud_msg.header.stamp = node_->get_clock()->now();

    reference_map_points_publisher->publish(cloud_msg);
    RCLCPP_INFO(node_->get_logger(), "Publish reference cloud as latched");
}

void viewer::run() {
    // Main loop of the viewer thread; throttled by Rate
    is_terminated_ = false;
    rclcpp::Rate rate(30);
    
    while (rclcpp::ok()) {
        if (is_start_) {
            // Publish visualization outputs for the most recent frame
            publish_local_map_point(stamp_);
            publish_map_point(stamp_);
            publish_path(stamp_);
            publish_debug_image(stamp_); 
            setIsStart(false);

            // NOTE: To switch this to ROS2 logging, replace with:
            // RCLCPP_DEBUG(node_->get_logger(), "viewer running");
            std::cout<<"viewer running"<<std::endl;  // kept by request; see note above
        }
        if (terminate_is_requested()) {
            break;
        }
        rate.sleep();
    }
    // Mark this viewer as terminated (thread exit)
    terminate();
}

void viewer::publish_debug_image(const rclcpp::Time &stamp) {
    // Render current frame overlay from FrameDrawer and publish as bgr8 image
    cv::Mat im = mpFrameDrawer_->DrawFrame(1);
    if (im.empty()) {
        return;
    }
    std_msgs::msg::Header header;
    header.stamp = stamp;
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(header, "bgr8", im).toImageMsg();
    debug_frame_publisher.publish(img_msg);
    // Consider: RCLCPP_DEBUG(node_->get_logger(), "Published debug image");
}

void viewer::publish_path(const rclcpp::Time &stamp) {
    // Build a nav_msgs/Path from all keyframes in the active map (expressed in map_frame coordinates)
    ORB_SLAM3::Map *pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    const std::vector<ORB_SLAM3::KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_id_;
    path.poses.clear();
    for (const auto &keyfrm: vpKFs) {
        if (!keyfrm) {
            continue;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = map_frame_id_;
        const auto Twc = T_ros_cam_se3_ * keyfrm->GetPoseInverse();

        // Orientation and translation copied into PoseStamped
        pose.pose.orientation.x = Twc.unit_quaternion().x();
        pose.pose.orientation.y = Twc.unit_quaternion().y();
        pose.pose.orientation.z = Twc.unit_quaternion().z();
        pose.pose.orientation.w = Twc.unit_quaternion().w();

        pose.pose.position.x = Twc.translation().x();
        pose.pose.position.y = Twc.translation().y();
        pose.pose.position.z = Twc.translation().z();

        path.poses.push_back(pose);
    }
    path_publisher->publish(path);
    // Consider: RCLCPP_DEBUG(node_->get_logger(), "Published path with %zu poses", path.poses.size());
}

void viewer::setStamp(const rclcpp::Time &stamp) {
    // Update the timestamp used for outgoing messages
    stamp_ = stamp;
}

void viewer::setIsStart(bool isStart) {
    // Flag to trigger a single round of publications in the run() loop
    is_start_ = isStart;
}

void viewer::request_terminate() {
    // Thread-safe request to stop the viewer loop
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool viewer::is_terminated() {
    // Thread-safe check whether terminate() has been called
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool viewer::terminate_is_requested() {
    // Thread-safe check of termination request flag
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void viewer::terminate() {
    // Thread-safe finalize flags upon thread exit
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    is_terminated_ = true;
    is_start_ = false;
}

void viewer::publish_local_map_point(const rclcpp::Time &stamp) {
    // Publish local map points around the reference keyframe (left camera frame)
    if (!mpMapDrawer_) {
        RCLCPP_ERROR(node_->get_logger(), "CRASH_DEBUG: mpMapDrawer_ is null inside publish_map_point!");
        return;
    }
    if (!mpMapDrawer_->mpAtlas) {
        RCLCPP_ERROR(node_->get_logger(), "CRASH_DEBUG: mpMapDrawer_->mpAtlas is null inside publish_map_point!");
        return;
    }
    ORB_SLAM3::Map *pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    if (pRefKF == nullptr) {
        RCLCPP_WARN(node_->get_logger(), "Reference KF is null");
        return;
    }

    const std::set<ORB_SLAM3::MapPoint *> &spLocalMPs = pRefKF->GetMapPoints();
    if (spLocalMPs.empty()) return;

    PointCloudXYZ::Ptr p_local_cloud(new PointCloudXYZ);

    for (auto spLocalMP: spLocalMPs) {
        if (spLocalMP->isBad()) continue;

        // Compute positions in camera coordinates if IMU is used; otherwise map to camera with T_ros_cam
        Eigen::Vector3f pos_w, pos_cam;
        if (is_imu_) {
            pos_w = spLocalMP->GetWorldPos();
            pos_cam = T_ros_cam_se3_ * mTcw * spLocalMP->GetWorldPos();
        } else
            pos_w = T_ros_cam_se3_ * spLocalMP->GetWorldPos();

        pcl::PointXYZ point;
        point.x = pos_cam(0);
        point.y = pos_cam(1);
        point.z = pos_cam(2);
        p_local_cloud->push_back(point);
    }

    if (p_local_cloud->size() < 1)
        return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*p_local_cloud, cloud_msg);
    cloud_msg.header.frame_id = left_camera_frame_id_;
    cloud_msg.header.stamp = stamp;

    local_map_points_publisher->publish(cloud_msg);
    // Consider: RCLCPP_DEBUG(node_->get_logger(), "Published local map cloud with %zu points", p_local_cloud->size());
}

void viewer::publish_map_point(const rclcpp::Time &stamp) {

    // Publish all map points in the global map frame
    if (!mpMapDrawer_) {
        RCLCPP_ERROR(node_->get_logger(), "CRASH_DEBUG: mpMapDrawer_ is null inside publish_map_point!");
        return;
    }

    if (!mpMapDrawer_->mpAtlas) {
        RCLCPP_ERROR(node_->get_logger(), "CRASH_DEBUG: mpMapDrawer_->mpAtlas is null inside publish_map_point!");
        return;
    }
    
    ORB_SLAM3::Map *pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    
    if (!pActiveMap) {
        RCLCPP_INFO(node_->get_logger(), "publish_map_point: pActiveMap is null!");
        return;
    }

    const std::vector<ORB_SLAM3::MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
    const std::vector<ORB_SLAM3::MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
    std::set<ORB_SLAM3::MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    RCLCPP_INFO(node_->get_logger(),"Attempting to publish %zu map points (Reference: %zu).", vpMPs.size(), spRefMPs.size());

    if (vpMPs.empty()) return;

    PointCloudXYZ::Ptr p_global_cloud(new PointCloudXYZ);

    for (size_t i = 0; i < vpMPs.size(); i++) {
        const auto lm = vpMPs.at(i);
        if (lm->isBad()) continue;

        Eigen::Vector3f pos_w;
        if (is_imu_)
            pos_w = lm->GetWorldPos();
        else
            pos_w = T_ros_cam_se3_ * lm->GetWorldPos();
        pcl::PointXYZ point(pos_w.x(), pos_w.y(), pos_w.z());
        p_global_cloud->push_back(point);
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*p_global_cloud, cloud_msg);
    cloud_msg.header.frame_id = map_frame_id_;
    cloud_msg.header.stamp = stamp;

    map_points_publisher->publish(cloud_msg);
    // Consider: RCLCPP_DEBUG(node_->get_logger(), "Published global map cloud with %zu points", p_global_cloud->size());
}
