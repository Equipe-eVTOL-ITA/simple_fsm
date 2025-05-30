#ifndef DRONE_HPP_
#define DRONE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <custom_msgs/msg/position.hpp>

#include <Eigen/Eigen>

namespace DronePX4
{
enum ARMING_STATE
{
	DISARMED = 1,
	ARMED = 2
};

enum CONTROLLER_TYPE
{
  	NO_CONTROLLER = 0,        // No controller defined
  	POSITION = 1,             // Position control
  	VELOCITY = 2,             // Velocity control
  	BODY_RATES = 3,           // Body rates (rad/s) and thrust [-1, 1] controller
};
}

class Drone
{
public:
	Drone();
	~Drone();


	/*
		Getters
	*/
	Eigen::Vector3d getLocalPosition();

	Eigen::Vector3d getOrientation();
	
	float getGroundSpeed();
	
	DronePX4::ARMING_STATE getArmingState();

	/*
		Setters
	*/
	void log(const std::string& info);

	void setOffboardControlMode(DronePX4::CONTROLLER_TYPE type);

	void setLocalPosition(float x, float y, float z, float yaw);

	void setOffboardMode();

	void toOffboardSync();

	void arm();
	void disarm();

	void armSync();
	void disarmSync();

	void setLocalVelocity(float vx, float vy, float vz, float yaw_rate = 0.0f);



private:
	/// Send command to PX4
	/// \param[in] command Command ID
	/// \param[in] target_system System which should execute the command
	/// \param[in] target_component Component which should execute the command, 0 for all components
	/// \param[in] source_system System sending the command
	/// \param[in] source_component Component sending the command
	/// \param[in] confirmation 0: First transmission of this command
	/// 1-255: Confirmation transmissions
	/// \param[in] param1 Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
	/// \param[in] param2 Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
	/// \param[in] param3 Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
	/// \param[in] param4 Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum.
	/// \param[in] param5 Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum.
	/// \param[in] param6 Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum.
	/// \param[in] param7 Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum.
	void sendCommand(
		uint32_t command, uint8_t target_system, uint8_t target_component, uint8_t source_system,
		uint8_t source_component, uint8_t confirmation, bool from_external,
		float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f,
		float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f,
		float param7 = 0.0f);

	void destroy();

	// Orchestration
	std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
	rclcpp::Node::SharedPtr px4_node_;
	std::thread spin_thread_;

	// Subscribers
	
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
	
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	

	// Publishers

	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
		
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr vehicle_offboard_control_mode_pub_;

	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr vehicle_trajectory_setpoint_pub_;

	
	// LAB 7 - POSITION PUBLISHER	
	rclcpp::Publisher<custom_msgs::msg::Position>::SharedPtr position_pub_;
	rclcpp::TimerBase::SharedPtr position_timer_;


	// VARIAVEIS PRIVADAS DOS SUBSCRIBERS
	
	std::chrono::time_point<std::chrono::high_resolution_clock> odom_timestamp_;
	float current_pos_x_{0.0f};
	float current_pos_y_{0.0f};
	float current_pos_z_{0.0f};
	float current_vel_x_{0.0f};
	float current_vel_y_{0.0f};
	float current_vel_z_{0.0f};
	float ground_speed_{0.0f};
	float roll_{0.0f};
	float pitch_{0.0f};
	float yaw_{0.0f};
	DronePX4::ARMING_STATE arming_state_{DronePX4::ARMING_STATE::DISARMED};


	// PX4 communication parameters
	int target_system_{1};
	uint8_t target_component_{1};
	uint8_t source_system_{255};
	uint8_t source_component_{0};
	uint8_t confirmation_{1};
	bool from_external_{true};
};

#endif