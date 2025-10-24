#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <custom_interfaces/msg/neighbors_info.hpp>

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        neighbors_gps_publisher_ = this->create_publisher<custom_interfaces::msg::NeighborsInfo>("neighbors_info", 10);
    }

private:
    rclcpp::Publisher<custom_interfaces::msg::NeighborsInfo>::SharedPtr neighbors_gps_publisher_;
};