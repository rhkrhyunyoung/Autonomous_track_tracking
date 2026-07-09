# Autonomous Track Tracking (C++)

![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros) ![C++](https://img.shields.io/badge/cpp-17-orange?logo=c%2B%2B) ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green?logo=opencv) ![RealSense](https://img.shields.io/badge/RealSense-D455-lightgrey)

##  Technical Features

### 1. Visual Perception (Blob-based Tracking)
*   **Robust Contour Analysis**: Replaced traditional line-fitting with **Blob/Contour Tracking** (`cv::findContours`) to ensure navigation stability when track edges are partially buried or distorted by steep banking.
*   **IPM (Inverse Perspective Mapping)**: Projects 3D road surfaces into a 2D Bird's Eye View (BEV) to calculate precise curvature and heading errors.
*   **Dynamic Path Interpolation**: Real-time virtual path generation based on a fixed 914.4mm track width, allowing consistent centering even when only one boundary is visible.
*   **High-Frequency Processing**: Optimized C++ backend achieves 30fps+ on embedded systems, reducing control loop latency.

### 2. Control & Motion Stability
*   **PD Control Algorithm**: Fine-tuned Proportional-Derivative control for smooth heading adjustment and oscillation suppression.
*   **Active Slip Compensation**: Integrates real-time IMU Z-axis angular velocity to compensate for track-type robot slippage during high-speed cornering.
*   **Advanced Signal Filtering**: 
    *   **Exponential Moving Average (EMA)**: Dampens sensor noise and prevents jitter.
    *   **Jump Guard**: Reject outlier target coordinates (>150px shift) caused by environmental artifacts.

##  System Architecture

Designed with a **modular Topic-Subscriber architecture** to enable multi-node concurrency without hardware conflict:

1.  **Sense**: RealSense D455 streams aligned RGB-Depth and IMU data.
2.  **Perceive**: `autonomous_track_tracking` (this node) identifies the track and calculates `cmd_vel_nav`.
3.  **Recognize**: YOLOv8 handles mission-critical objects (Red flags, traffic signals).
4.  **Act**: `Mission Manager` arbitrates all navigation and mission inputs for final vehicle control.commands (`cmd_vel`).

##  Getting Started

*   ROS 2 Humble
*   OpenCV 4.x
*   cv_bridge
*   Eigen3 (`sudo apt install libeigen3-dev`)

```
# realsence
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true depth_module.depth_fps:=15.0 rgb_module.rgb_fps:=15.0
#run code
ros2 run linetracing_cpp main_node
```
