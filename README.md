# 🚁 PX4 ROS2 Offboard Control

## 📋 Overview
Multi-vehicle offboard control system for PX4 autopilot using ROS2. Supports autonomous flight, swarm operations, and real-time position monitoring.

https://github.com/user-attachments/assets/f64cdb6e-4cb2-478a-b3f2-e52be3298e29

<img width="957" height="792" alt="Screenshot from 2025-10-05 16-57-21" src="https://github.com/user-attachments/assets/2be3b12d-b2d6-4884-9ab9-afb669cc1390" />

## ✨ Features
- **Multi-Vehicle Support**: Control multiple drones simultaneously
- **ArUco World Integration**: Specialized environment for computer vision applications
- **Advanced Camera System**: Dual camera bridges (image + camera_info) for full sensor_msgs support
- **Automated Launch**: One-click system startup with complete camera integration
- **Real-time Monitoring**: GPS and local position tracking
- **Swarm Operations**: Coordinated multi-drone missions in ArUco environment
- **Computer Vision Ready**: Full OpenCV/ROS2 camera integration with calibration data
- **Neighbor Communication**: Custom interface for GPS sharing between drones in swarm
- **Custom Interfaces**: ROS2 message definitions for advanced swarm coordination

## 🚀 Quick Start

### Prerequisites
```bash
sudo apt install ros-humble-desktop-full ros-humble-px4-msgs
```

### Build
```bash
git clone https://github.com/semihberat/OffboardControl.git
cd OffboardControl
colcon build --packages-select px4_ros_com custom_interfaces
source install/setup.bash
```

### Launch System
```bash
# Complete system startup (PX4 ArUco World + ROS2 + Camera Bridges)
./start_complete_system.sh

# Or step-by-step:
./start_multi_drones.sh              # Start PX4 instances in ArUco world
ros2 launch px4_ros_com multi_robot_start.py  # Launch ROS2 nodes + Dual camera bridges
```

## 📸 Advanced Camera System

### Dual Camera Streams
```bash
# Check available camera topics (image + camera_info)
ros2 topic list | grep -E "image|camera_info"

# View camera with calibration data
ros2 run image_view image_view image:=/world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image

# Camera info for OpenCV calibration
ros2 topic echo /world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/camera_info

# All camera streams available:
# - Image streams for each drone
# - Camera_info streams for calibration data
# - ArUco marker detection ready
```

### Computer Vision Integration
```bash
# Example: ArUco marker detection
ros2 run cv_bridge cv_bridge_test image:=/world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image
```

## 🎮 Usage

### Single Vehicle
```bash
ros2 run px4_ros_com main --ros-args -p px4_namespace:="/fmu/"
```

### Multi-Vehicle
```bash
ros2 launch px4_ros_com multi_robot_start.py
```

## � Demo Video
Check out the 3-drone swarm demonstration: `Screencast from 10-01-2025 09_50_40 PM.mp4`

## 🔧 Configuration
- **Namespaces**: `/fmu/`, `/px4_1/`, `/px4_2/`, `/px4_3/`
- **System IDs**: Automatically extracted from namespaces
- **Control Frequency**: 20Hz
- **Target Altitude**: 5m (NED: -5.0)

## 🐛 Troubleshooting

### Vehicle Not Taking Off
```bash
# Check PX4 status
ros2 topic echo /px4_1/fmu/out/vehicle_status_v1 --once

# Verify topics
ros2 topic hz /px4_1/fmu/in/offboard_control_mode
```

### Neighbor Communication Issues
```bash
# Check neighbor GPS sharing
ros2 topic echo /neighbors_gps

# Verify custom interface
ros2 interface show custom_interfaces/msg/NeighborsInfo
```

### QoS Issues
All publishers use `sensor_data` QoS profile for PX4 compatibility.

## 📚 API Reference

### OffboardController Class
```cpp
class OffboardController : public rclcpp::Node {
public:
    void arm();
    void disarm();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z, float yaw);
    void publish_gps_to_neighbors();  // Share GPS data with swarm
    
    VehicleLocalPosition vehicle_local_position_;  // Current position
    rclcpp::Publisher<custom_interfaces::msg::NeighborsInfo>::SharedPtr neighbors_gps_publisher_;
};
```

### Custom Interfaces
```bash
# NeighborsInfo.msg - Swarm communication interface
uint8[] neighbors_ids          # Array of neighbor drone IDs
px4_msgs/SensorGps[] neighbors_gps  # GPS data from each neighbor
```

## 🎯 Mission Logic
```cpp
if (vehicle_local_position_.z > -4.0f) {
    // Takeoff - climb to target altitude
    publish_trajectory_setpoint(0.0, 0.0, -5.0, 3.14);
} else {
    // Navigate - move to waypoint
    publish_trajectory_setpoint(5.0, 5.0, -5.0, 3.14);
}
```

## � Support
- **GitHub Issues**: [Create an issue](https://github.com/semihberat/OffboardControl/issues)
- **PX4 Community**: [PX4 Discuss Forum](https://discuss.px4.io/)

---
**⚠️ Safety**: Educational/research use only. Follow local regulations and test in simulation first.
