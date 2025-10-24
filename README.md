# 🚁 PX4 ROS2 Offboard Control

Multi-vehicle offboard control system for PX4 autopilot using ROS2. Supports autonomous flight, swarm operations, and real-time neighbor communication.

https://github.com/user-attachments/assets/f64cdb6e-4cb2-478a-b3f2-e52be3298e29

<img width="3232" height="1124" alt="rosgraph" src="https://github.com/user-attachments/assets/fc17132e-0c9e-4f35-8df6-8d8e964a3701" />

## ✨ Features
- **Multi-Vehicle Support**: Control up to 5 drones simultaneously
- **Swarm Communication**: Custom neighbor GPS sharing via `NeighborsInfo` interface
- **ArUco World Integration**: Specialized environment for computer vision applications
- **Advanced Camera System**: Dual camera bridges (image + camera_info) for full sensor_msgs support
- **Automated Launch**: One-click system startup with complete camera integration
- **Real-time Monitoring**: GPS and local position tracking
- **Computer Vision Ready**: Full OpenCV/ROS2 camera integration with calibration data
- **Dynamic Configuration**: Runtime parameter adjustment via ROS2 parameters
- **Namespace Support**: Clean topic organization with `/px4_{id}/` namespaces

## 🚀 Quick Start

### Build
```bash
git clone https://github.com/semihberat/OffboardControl.git
cd OffboardControl
colcon build --packages-select px4_ros_com custom_interfaces
source install/setup.bash
```

### Launch System
```bash
./start_multi_drones.sh                      # Start PX4 instances in ArUco world
ros2 launch px4_ros_com multi_robot_start.py # Launch ROS2 nodes + Camera bridges
```

## 📁 Project Structure (project reorganized)

```
px4_ros_com/
├── src/
│   ├── controller/           # core controllers (uav_controller, offboard logic)
│   ├── formulations/         # math & algorithm helpers (e.g., CalculateCentralPoint)
│   ├── lib/                  # shared libraries (frame_transforms, helpers)
│   ├── object_detection/     # vision modules and scripts
│   └── path_planner/         # path planning modules and planners
├── launch/                   # launch files (multi_robot_start.py)
├── config/                   # configuration parameters (multi_robot_params.yaml)
└── start_multi_drones.sh     # PX4 SITL startup script
```

Note: the package `main_class` executable is built from `src/controller/uav_controller.cpp` (see `CMakeLists.txt`).

## 📊 ROS2 Topic Architecture

### 🔍 **Topic Discovery**
```bash
# List all active topics
ros2 topic list

# Filter PX4 topics only
ros2 topic list | grep px4

# Show topic tree structure
ros2 topic list -t
```

### 📡 **Topic Hierarchy**

#### **Per-Drone Topics Structure** (`/px4_{1-5}/`)
```
/px4_1/fmu/in/                    # PX4 Command Input Topics
├── offboard_control_mode         # Control mode commands
├── trajectory_setpoint           # Position/velocity setpoints  
└── vehicle_command               # Vehicle control commands

/px4_1/fmu/out/                   # PX4 Data Output Topics
├── vehicle_global_position       # GPS coordinates
├── vehicle_local_position_v1     # Local NED position
└── vehicle_status_v1             # Flight status

/px4_1/neighbors_info             # Swarm Communication Topic
└── custom_interfaces/msg/NeighborsInfo  # Neighbor GPS sharing
```

#### **Camera Topics** (`/world/aruco/model/`)
```
/world/aruco/model/x500_mono_cam_down_{1-5}/link/camera_link/sensor/imager/
├── image                         # Camera image stream
└── camera_info                   # Camera calibration data
```

### 🔍 **Topic Monitoring Commands**

#### **Vehicle Status Monitoring**
```bash
# Monitor drone position
ros2 topic echo /px4_1/fmu/out/vehicle_local_position_v1

# Check GPS coordinates
ros2 topic echo /px4_1/fmu/out/vehicle_global_position

# Vehicle arming/flight mode status
ros2 topic echo /px4_1/fmu/out/vehicle_status_v1

# Topic frequency monitoring
ros2 topic hz /px4_1/fmu/out/vehicle_local_position_v1
```

#### **Swarm Communication Monitoring**
```bash
# Monitor neighbor GPS sharing
ros2 topic echo /px4_1/neighbors_info

# Check neighbor GPS data structure
ros2 interface show custom_interfaces/msg/NeighborsInfo

# Monitor all swarm topics
ros2 topic list | grep neighbors
```

#### **Command Input Verification**
```bash
# Verify trajectory commands are being sent
ros2 topic echo /px4_1/fmu/in/trajectory_setpoint

# Monitor control mode commands
ros2 topic echo /px4_1/fmu/in/offboard_control_mode

# Check vehicle commands (arm/disarm/mode changes)
ros2 topic echo /px4_1/fmu/in/vehicle_command
```

#### **Camera System Monitoring**
```bash
# Monitor camera image stream
ros2 topic echo /world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image

# Check camera calibration info
ros2 topic echo /world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/camera_info

# View image with RViz/image_view
ros2 run image_view image_view image:=/world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image
```

### Computer Vision Integration
```bash
# Example: ArUco marker detection
ros2 run cv_bridge cv_bridge_test image:=/world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image
```

## 🎮 Usage Guide

### Single Vehicle Control
```bash
# Launch single drone (sys_id=1)
ros2 run px4_ros_com main_class --ros-args -p sys_id:=1 -p number_of_drones:=1
```

### Multi-Vehicle Swarm
```bash
# Launch complete 5-drone swarm with cameras
ros2 launch px4_ros_com multi_robot_start.py
```

### Parameter Configuration
```bash
# Check current parameters
ros2 param list /drone1

# Get specific parameter
ros2 param get /drone1 sys_id

# Set parameter at runtime
ros2 param set /drone1 number_of_drones 3
```

## 🌐 Node Information

### **Active ROS2 Nodes**
```bash
# List all running nodes
ros2 node list

# Expected nodes for 5-drone setup:
# /drone1, /drone2, /drone3, /drone4, /drone5
# /camera_bridge_1, /camera_bridge_2, ...
# /camera_info_bridge_1, /camera_info_bridge_2, ...
```

### **Node Details**
```bash
# Get detailed node information
ros2 node info /drone1

# Check node's topics, services, actions
ros2 node info /drone1 --include-hidden
```

## 🎯 Mission Logic

The system implements altitude-based mission phases:

```cpp
if (vehicle_local_position_.z > -4.0f) {
    // Phase 1: Takeoff - climb to target altitude
    publish_trajectory_setpoint(0.0, 0.0, -5.0, 3.14);
} else {
    // Phase 2: Navigation - move to waypoint + share GPS
    publish_gps_to_neighbors();
    publish_trajectory_setpoint(5.0, 5.0, -5.0, 3.14);
}
```

## 🔧 Configuration

- **System IDs**: 1-5 (automatically assigned per drone)
- **Control Frequency**: 100ms (10Hz)
- **Target Altitude**: -5.0m (NED coordinate system)
- **Swarm Size**: Configurable via `number_of_drones` parameter
- **QoS Profile**: `sensor_data` for PX4 compatibility

## 🐛 Troubleshooting

### Vehicle Not Taking Off
```bash
# Check PX4 connection
ros2 topic hz /px4_1/fmu/out/vehicle_status_v1

# Verify command publishing
ros2 topic hz /px4_1/fmu/in/trajectory_setpoint

# Monitor arming status
ros2 topic echo /px4_1/fmu/out/vehicle_status_v1 --once
```

### Swarm Communication Issues
```bash
# Verify neighbor topic creation
ros2 topic list | grep neighbors_info

# Check neighbor GPS data flow
ros2 topic hz /px4_1/neighbors_info

# Monitor GPS sharing between drones
ros2 topic echo /px4_1/neighbors_info | head -20
```

### Camera System Debug
```bash
# Check camera bridge status
ros2 node list | grep camera

# Verify image stream
ros2 topic hz /world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image

# Test camera_info data
ros2 topic echo /world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/camera_info --once
```

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

## 💡 Support
- **GitHub Issues**: [Create an issue](https://github.com/semihberat/OffboardControl/issues)
- **PX4 Community**: [PX4 Discuss Forum](https://discuss.px4.io/)

---
**⚠️ Safety**: Educational/research use only. Follow local regulations and test in simulation first.
