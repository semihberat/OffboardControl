#pragma once
#ifndef COMMAND_APPROVEMENT_HPP
#define COMMAND_APPROVEMENT_HPP

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class CommandApprovement : public rclcpp::Node
{
public:
	CommandApprovement(std::string px4_namespace) :
		Node("command_approvement_service")

	{
		
	}



private:
		
};

/* 
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CommandApprovement>("/fmu/"));

	rclcpp::shutdown();
	return 0;
} 
*/



#endif // OFFBOARD_CONTROLLER_HPP