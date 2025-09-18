# ORB_SLAM3_ROS2_pointcloud

ROS 2 wrapper for **ORB-SLAM3** with point cloud generation support.

---

## 📺 Demo Video

https://youtu.be/hwASetrvGZs?si=L7dGaWlm62rIHIhy

---

## 📋 Prerequisites

This repository was tested with:

- **Ubuntu 22.04**
- **ROS 2 Humble**




---

## 🔨 Build ORB-SLAM3


 Build ORB_SLAM3
  - Go to this [repo](https://github.com/UZ-SLAMLab/ORB_SLAM3) and follow build instruction.
  ```bash
cd ~/colcon_ws/src/ORB_SLAM3_ROS2_pointcloud/ORB_SLAM3
./build.sh
```
  

---

## ⚙️ Build Instructions

Prepare workspace:
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/ahnavocado/ORB_SLAM3_ROS2_pointcloud.git
```
Change this [line ](https://github.com/ahnavocado/ORB_SLAM3_ROS2_pointcloud/blob/94455bf3e98338a72042468e501140ef2120613c/ORB_SLAM3_ROS2/CMakeLists.txt#L5) to your own `python site-packages` path

Change this [line](https://github.com/ahnavocado/ORB_SLAM3_ROS2_pointcloud/blob/94455bf3e98338a72042468e501140ef2120613c/ORB_SLAM3_ROS2/CMakeModules/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path

Build:
```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-ignore orbslam2 \
  --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error -Wno-error=deprecated-declarations -Wno-deprecated-declarations -Wno-error=reorder -Wno-error=unused-parameter"
```

Source environment:
```bash
source install/setup.bash
```

Extract vocabulary:
```bash
tar -xvzf ./src/ORB_SLAM3_ROS2_pointcloud/vocabulary/ORBvoc.txt.tar.gz -C ./src/ORB_SLAM3_ROS2_pointcloud/vocabulary/
```

---

## ▶️ Usage

###  Run Monocular SLAM
```bash
ros2 run orbslam3_ros2_pointcloud mono \
  ./src/ORB_SLAM3_ROS2_pointcloud/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt \
  ./src/ORB_SLAM3_ROS2_pointcloud/ORB_SLAM3_ROS2/config/monocular/TUM1.yaml \
  --ros-args -r camera:=/camera/image_raw
```


The ORB-SLAM3 ROS 2 node subscribes to `/camera/image_raw` and `/camera/camera_info` for SLAM execution.

---


## 🐞 Troubleshooting

- **setup.cfg warnings**
  ```
  UserWarning: Unknown distribution option: 'tests_require'
  ```
  → Caused by setuptools version mismatch:
  ```bash
  pip install setuptools==58.2.0
  ```


---

## 📦 Supported Modes

This repository currently supports:

- **Monocular (mono)**
- **Stereo (stereo) (To be Developed)**


---

## 🙏 Acknowledgments

- [ORB-SLAM3 (UZ-SLAMLab)](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ros2_orbslam3](https://github.com/zang09/ORB_SLAM3_ROS2.git)

This project extends ORB-SLAM3 with ROS 2 integration and point cloud generation.

---
## ▶️ Testing with EuRoC Image

Check out this [repo](https://github.com/ahnavocado/gamja_dataset) for additional info

