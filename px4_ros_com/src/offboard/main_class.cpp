
#include "controllers/offboard_controller.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <custom_interfaces/msg/neighbors_info.hpp>

// isimlendirme yaparken basina / koymak ros2'de namespace kullaniminda daha kati olmasi icin onlem aliyor
// eger / koymazsaniz node ismi namespace'in basina ekleniyor ve bu bazen istenmeyen durumlara yol acabiliyor
// ornegin node ismi offboard_control iken namespace /my_ns ise node ismi /my_ns/offboard_control oluyor

using namespace std::placeholders;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public OffboardController
{
public:
	OffboardControl() : OffboardController()
	{

		// Listener for GPS and Local Position
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		std::string gpstpc = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_global_position";
		std::string lpstpc = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_local_position_v1";

		// Subscribers
		vehicle_gps_subscriptions_ = this->create_subscription<VehicleGlobalPosition>(gpstpc, qos,
																					  std::bind(&OffboardControl::gps_callback, this, _1));

		local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(lpstpc, qos,
																					   std::bind(&OffboardControl::local_position_callback, this, _1));
		// Listen to neighbor GPS topics
		for (uint8_t i = 1; i <= number_of_drones; i++)
		{
			if (i != sys_id)
			{
				std::string member_topic = "/px4_" + std::to_string(i) + "/fmu/out/vehicle_global_position";
				sub = this->create_subscription<VehicleGlobalPosition>(member_topic, qos,
																			std::bind(&OffboardControl::neighbor_gps_callback, this, _1));
			}
		}
		
		// Publishers
		neighbors_gps_publisher_ = this->create_publisher<custom_interfaces::msg::NeighborsInfo>("/neighbors_info", 10);

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publisher_callback, this));
	}

	VehicleLocalPosition vehicle_local_position_;
	VehicleGlobalPosition vehicle_gps_position_;
	// Here is main function where the publisher and subscriber nodes are created and initialized.
private:
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_gps_subscriptions_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr sub;
	// Publisher Callback
	void publisher_callback()
	{
		if (offboard_setpoint_counter_ == 10)
		{
			// Change to Offboard mode after 10 setpoints
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// Arm the vehicle
			this->arm();
		}

		// offboard_control_mode needs to be paired with trajectory_setpoint
		publish_offboard_control_mode();

		if (vehicle_local_position_.z > -4.0f)
		{

			publish_trajectory_setpoint(0.0, 0.0, -5.0, 3.14);
		}
		else
		{
			// Just log any word to debug
			publish_trajectory_setpoint(5.0, 5.0, -5.0, 3.14); // hover at 5 meters
		}

		// stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 11)
		{
			offboard_setpoint_counter_++;
		}
	}

	// Subscriber Callbacks
	void gps_callback(const VehicleGlobalPosition::SharedPtr msg)
	{
		vehicle_gps_position_ = *msg;
	}

	void local_position_callback(const VehicleLocalPosition::SharedPtr msg)
	{
		vehicle_local_position_ = *msg;
	}

	void neighbor_gps_callback(const VehicleGlobalPosition::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "Received neighbor GPS: Lat = %lf, Lon = %lf, Alt = %lf",
					msg->lat, msg->lon, msg->alt);
		// Publish the GPS data to neighbors
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}