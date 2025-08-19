cmake_minimum_required(VERSION 3.5)
project(orbslam3_ros2_pointcloud)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.8/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)

include_directories(
  include
)
include_directories(SYSTEM
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)

add_executable(mono
  src/node.cpp
  src/viewer.cpp
  src/utils/Utils.cpp
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin image_transport geometry_msgs)

target_compile_options(mono PRIVATE
  -Wno-error                # 모든 경고를 에러로 취급하지 않음
  -Wno-error=deprecated-declarations
  -Wno-deprecated-declarations
  -Wno-error=reorder
  -Wno-error=unused-parameter
)

add_executable(rgbd
  src/node.cpp
  src/viewer.cpp
  src/utils/Utils.cpp
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin image_transport geometry_msgs)

target_compile_options(rgbd PRIVATE
  -Wno-error                # 모든 경고를 에러로 취급하지 않음
  -Wno-error=deprecated-declarations
  -Wno-deprecated-declarations
  -Wno-error=reorder
  -Wno-error=unused-parameter
)

# add_executable(stereo
#   src/stereo/stereo.cpp
#   src/stereo/stereo-slam-node.cpp
# )
# ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters
# ORB_SLAM3 Pangolin)

# add_executable(stereo-inertial
#   src/stereo-inertial/stereo-inertial.cpp
#   src/stereo-inertial/stereo-inertial-node.cpp
# )
# ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

install(TARGETS mono rgbd
  DESTINATION lib/${PROJECT_NAME})
# install(TARGETS mono rgbd stereo stereo-inertial
#   DESTINATION lib/${PROJECT_NAME})
# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()
