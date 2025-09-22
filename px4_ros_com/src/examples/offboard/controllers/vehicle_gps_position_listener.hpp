#pragma once
#ifndef VEHICLE_GPS_POSITION_HPP
#define VEHICLE_GPS_POSITION_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

using namespace std::placeholders;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * @brief Vehicle GPS position uORB topic data callback
 */
class VehicleGpsPositionListener : public rclcpp::Node
{
public:
	VehicleGpsPositionListener(std::string px4_namespace) : Node("vehicle_global_position_listener")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<SensorGps>(px4_namespace + "out/vehicle_gps_position", qos,
															 std::bind(&VehicleGpsPositionListener::gps_callback, this, _1));

		local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(px4_namespace + "out/vehicle_local_position", qos,
																					   std::bind(&VehicleGpsPositionListener::local_position_callback, this, _1));
	}
	VehicleLocalPosition vehicle_local_position_;

private:
	rclcpp::Subscription<SensorGps>::SharedPtr subscription_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;

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

#endif // VEHICLE_GPS_POSITION_HPP