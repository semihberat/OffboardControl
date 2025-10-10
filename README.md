# 🚁 PX4 ROS2 Offboard Control

## 📋 Overview
Multi-vehicle offboard control system for PX4 autopilot using ROS2. Supports autonomous flight, swarm operations, and real-time position monitoring.

https://github.com/user-attachments/assets/f64cdb6e-4cb2-478a-b3f2-e52be3298e29

<img width="957" height="792" alt="Screenshot from 2025-10-05 16-57-21" src="https://github.com/user-attachments/assets/2be3b12d-b2d6-4884-9ab9-afb669cc1390" />

## ✨ Features
- **Multi-Vehicle Support**: Control multiple drones simultaneously
- **Camera Integration**: Real-time camera feeds from all drones via ros_gz_bridge
- **Automated Launch**: One-click system startup with camera bridges
- **Real-time Monitoring**: GPS and local position tracking
- **Swarm Operations**: Coordinated multi-drone missions
- **Computer Vision Ready**: OpenCV-compatible camera streams

## 🚀 Quick Start

### Prerequisites
```bash
sudo apt install ros-humble-desktop-full ros-humble-px4-msgs
```

### Build
```bash
git clone https://github.com/semihberat/OffboardControl.git
cd OffboardControl
colcon build --packages-select px4_ros_com
source install/setup.bash
```

### Launch System
```bash
# Complete system startup (PX4 + ROS2 + Camera Bridges)
./start_complete_system.sh

# Or step-by-step:
./start_multi_drones.sh              # Start PX4 instances
ros2 launch px4_ros_com multi_robot_start.py  # Launch ROS2 nodes + Camera bridges
```

## 📸 Camera Features

### View Camera Streams
```bash
# Check available camera topics
ros2 topic list | grep camera

# View specific drone camera
ros2 run image_view image_view image:=/px4_1/camera/image

# All cameras available at:
# /px4_1/camera/image
# /px4_2/camera/image  
# /px4_3/camera/image
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
    
    VehicleLocalPosition vehicle_local_position_;  // Current position
};
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
