
#include "controllers/offboard_controller.hpp"
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

using namespace std::placeholders;

class OffboardControl : public OffboardController
{
public:
	OffboardControl() : OffboardController()
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publisher_callback, this));
		
		subscription_ = this->create_subscription<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", qos,
		std::bind(&OffboardControl::gps_callback, this,_1));

		local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
			std::bind(&OffboardControl::local_position_callback, this,_1));
	}

// Here is main function where the publisher and subscriber nodes are created and initialized.
private:
	rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
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
		publish_trajectory_setpoint();

		// stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 11)
		{
			offboard_setpoint_counter_++;
		}
	}

	void gps_callback(const px4_msgs::msg::SensorGps::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "GPS: lat: %.7f, lon: %.7f, alt: %.2f", 
		msg->latitude_deg, msg->longitude_deg, msg->altitude_ellipsoid_m);
	}

	void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "Local Position: x: %.2f, y: %.2f, z: %.2f", 
		msg->x, msg->y, msg->z);
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