#include "viewer.h"

viewer::viewer(rclcpp::Node* node, ORB_SLAM3::LocalMapping *pLocalMapping, ORB_SLAM3::FrameDrawer *pFrameDrawer,
               ORB_SLAM3::MapDrawer *pMapDrawer, bool is_imu) :
        node_(node), it_(node), mpLocalMapping_(pLocalMapping), mpFrameDrawer_(pFrameDrawer), mpMapDrawer_(pMapDrawer),
        is_imu_(is_imu), pRefKF(nullptr) {

    stamp_ = node_->get_clock()->now();
    // set parameters and variable
    set_params();

    // broadcasting
    set_advertising();

    // publish reference map 
    if (use_reference_cloud_)
        publish_reference_cloud();
}

void viewer::set_params() {
    // Declare and retrieve frame id parameters
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

    if(use_reference_cloud_){
        pcl::io::loadPCDFile(reference_cloud_path_, reference_cloud);
    }

    T_ros_cam.block<3, 3>(0, 0) = Eigen::Quaternionf(T_ros_cam.block<3, 3>(0, 0))
            .normalized()
            .toRotationMatrix();
    T_ros_cam_se3_ = Sophus::SE3f(T_ros_cam);
}


void viewer::set_advertising() {
    debug_frame_publisher = it_.advertise(debug_frame_topic, 1);

    path_publisher = node_->create_publisher<nav_msgs::msg::Path>(path_topic, 1);

    map_points_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(map_points_topic, 1);

    local_map_points_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(local_map_points_topic, 1);

    reference_map_points_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(reference_map_points_topic, rclcpp::QoS(1).transient_local());

    global_map_saver = node_->create_service<std_srvs::srv::Empty>("/orbslam3_ros2_pointcloud/save_global_map", 
        std::bind(&viewer::SaveGlobalMapSrv, this, std::placeholders::_1, std::placeholders::_2));
}

void viewer::SaveGlobalMapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
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
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(reference_cloud, cloud_msg);
    cloud_msg.header.frame_id = map_frame_id_;
    cloud_msg.header.stamp = node_->get_clock()->now();

    reference_map_points_publisher->publish(cloud_msg);
    RCLCPP_INFO(node_->get_logger(), "Publish reference cloud as latched");
}

void viewer::run() {
    is_terminated_ = false;
    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        if (is_start_) {
            publish_local_map_point(stamp_);
            publish_debug_image(stamp_);
            setIsStart(false);
        }
        if (terminate_is_requested()) {
            break;
        }
        rate.sleep();
    }
    terminate();
}

void viewer::publish_debug_image(const rclcpp::Time &stamp) {
    cv::Mat im = mpFrameDrawer_->DrawFrame(1);
    if (im.empty()) {
        return;
    }
    std_msgs::msg::Header header;
    header.stamp = stamp;
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(header, "bgr8", im).toImageMsg();
    debug_frame_publisher.publish(img_msg);
}

void viewer::publish_path(const rclcpp::Time &stamp) {
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
}

void viewer::setStamp(const rclcpp::Time &stamp) {
    stamp_ = stamp;
}

void viewer::setIsStart(bool isStart) {
    is_start_ = isStart;
}

void viewer::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool viewer::is_terminated() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool viewer::terminate_is_requested() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void viewer::terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    is_terminated_ = true;
    is_start_ = false;
}

void viewer::publish_local_map_point(const rclcpp::Time &stamp) {
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
}

void viewer::publish_map_point(const rclcpp::Time &stamp) {
    ORB_SLAM3::Map *pActiveMap = mpMapDrawer_->mpAtlas->GetCurrentMap();
    if (!pActiveMap) return;

    const std::vector<ORB_SLAM3::MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
    const std::vector<ORB_SLAM3::MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
    std::set<ORB_SLAM3::MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

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
}
