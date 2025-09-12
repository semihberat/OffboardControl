#pragma once
#ifndef VEHICLE_GPS_POSITION_HPP
#define VEHICLE_GPS_POSITION_HPP
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

/**
 * @brief Vehicle GPS position uORB topic data callback
 */
class VehicleGpsPositionListener : public rclcpp::Node
{
public:
	explicit VehicleGpsPositionListener() : Node("vehicle_global_position_listener")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		subscription_ = this->create_subscription<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED VEHICLE GPS POSITION DATA"   << std::endl;
			std::cout << "=================================="   << std::endl;
			std::cout << "ts: "  << msg->timestamp    << std::endl;
			std::cout << "lat: " << msg->latitude_deg  << std::endl;
			std::cout << "lon: " << msg->longitude_deg << std::endl;
			std::cout << "alt: " << msg->altitude_msl_m  << std::endl;
			std::cout << "alt_ellipsoid: " << msg->altitude_ellipsoid_m << std::endl;
			std::cout << "s_variance_m_s: " << msg->s_variance_m_s << std::endl;
			std::cout << "c_variance_rad: " << msg->c_variance_rad << std::endl;
			std::cout << "fix_type: " << msg->fix_type << std::endl;
			std::cout << "eph: " << msg->eph << std::endl;
			std::cout << "epv: " << msg->epv << std::endl;
			std::cout << "hdop: " << msg->hdop << std::endl;
			std::cout << "vdop: " << msg->vdop << std::endl;
			std::cout << "noise_per_ms: " << msg->noise_per_ms << std::endl;
			std::cout << "vel_m_s: " << msg->vel_m_s << std::endl;
			std::cout << "vel_n_m_s: " << msg->vel_n_m_s << std::endl;
			std::cout << "vel_e_m_s: " << msg->vel_e_m_s << std::endl;
			std::cout << "vel_d_m_s: " << msg->vel_d_m_s << std::endl;
			std::cout << "cog_rad: " << msg->cog_rad << std::endl;
			std::cout << "vel_ned_valid: " << msg->vel_ned_valid << std::endl;
			std::cout << "timestamp_time_relative: " << msg->timestamp_time_relative << std::endl;
			std::cout << "time_utc_usec: " << msg->time_utc_usec << std::endl;
			std::cout << "satellites_used: " << msg->satellites_used << std::endl;
			std::cout << "heading: " << msg->heading << std::endl;
			std::cout << "heading_offset: " << msg->heading_offset << std::endl;
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr subscription_;
};

#endif // VEHICLE_GPS_POSITION_HPP