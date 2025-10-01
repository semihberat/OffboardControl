
#include "controllers/offboard_controller.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>


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

		std::string gpstpc = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_gps_position";
		std::string lpstpc = "/px4_" + std::to_string(sys_id) + "/fmu/out/vehicle_local_position";

		subscription_ = this->create_subscription<SensorGps>(gpstpc, qos,
															 std::bind(&OffboardControl::gps_callback, this, _1));

		local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(lpstpc, qos,
																					   std::bind(&OffboardControl::local_position_callback, this, _1));
		// Timer for publishing setpoints
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publisher_callback, this));																		
	}

	VehicleLocalPosition vehicle_local_position_;

	// Here is main function where the publisher and subscriber nodes are created and initialized.
private:
	rclcpp::Subscription<SensorGps>::SharedPtr subscription_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
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

			publish_trajectory_setpoint(0.0,0.0,-5.0,3.14);
		}
		else
		{
			//Just log any word to debug
			publish_trajectory_setpoint(5.0, 5.0, -5.0, 3.14); // hover at 5 meters
		}

		// stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 11)
		{
			offboard_setpoint_counter_++;
		}
	}
	
	// Subscriber Callbacks
	void gps_callback(const SensorGps::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "GPS: lat: %.7f, lon: %.7f, alt: %.2f",
					msg->latitude_deg, msg->longitude_deg, msg->altitude_ellipsoid_m);
	}

	void local_position_callback(const VehicleLocalPosition::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "Local Position: x: %.2f, y: %.2f, z: %.2f",
					msg->x, msg->y, msg->z);
		vehicle_local_position_ = *msg;
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