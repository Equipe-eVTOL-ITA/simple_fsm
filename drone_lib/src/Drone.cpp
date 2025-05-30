#include "drone/Drone.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "tf2/utils.h"


Drone::Drone() {
	// Create executor and node
	this->exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	this->px4_node_ = std::make_shared<rclcpp::Node>("Drone");
	this->exec_->add_node(px4_node_);
	
	// Start the executor in a separate thread
	this->spin_thread_ = std::thread([this]() {
		this->exec_->spin();
	});

	// Configure QoS for PX4 communication
	rclcpp::QoS px4_qos(5);
	px4_qos.best_effort();
	px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

	// QoS for custom messages
	rclcpp::QoS custom_qos(10);

	// Subscribe to vehicle status
	this->vehicle_status_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status",
		px4_qos,
		[this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
			switch (msg->arming_state) {
				case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
					this->arming_state_ = DronePX4::ARMING_STATE::ARMED;
					break;
				case px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED:
					this->arming_state_ = DronePX4::ARMING_STATE::DISARMED;
					break;
				default:
					this->arming_state_ = DronePX4::ARMING_STATE::DISARMED;
					break;
			}
		}
	);

	// Subscribe to vehicle odometry
	this->vehicle_odometry_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
		"/fmu/out/vehicle_odometry",
		px4_qos,
		[this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
			this->odom_timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
				std::chrono::nanoseconds(msg->timestamp));
			
			// Update position and velocity
			this->current_pos_x_ = msg->position[0];
			this->current_pos_y_ = msg->position[1];
			this->current_pos_z_ = msg->position[2];
			this->current_vel_x_ = msg->velocity[0];
			this->current_vel_y_ = msg->velocity[1];
			this->current_vel_z_ = msg->velocity[2];
			
			// Calculate ground speed
			this->ground_speed_ = std::sqrt(
				std::pow(msg->velocity[0], 2) + std::pow(msg->velocity[1], 2));

			// Extract orientation from quaternion if valid
			if (!std::isnan(msg->q[0])) {
				double yaw = 0, pitch = 0, roll = 0;
				// PX4 uses WXYZ, TF2 uses XYZW
				tf2::getEulerYPR(
					tf2::Quaternion(msg->q[1], msg->q[2], msg->q[3], msg->q[0]),
					yaw, pitch, roll
				);
				this->yaw_ = static_cast<float>(yaw);
				this->pitch_ = static_cast<float>(pitch);
				this->roll_ = static_cast<float>(roll);
			}
		}
	);

	// Create publishers
	this->vehicle_command_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleCommand>(
		"/fmu/in/vehicle_command", px4_qos);

	this->vehicle_offboard_control_mode_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
		"/fmu/in/offboard_control_mode", px4_qos);
		
	this->vehicle_trajectory_setpoint_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
		"/fmu/in/trajectory_setpoint", px4_qos);

	this->position_pub_ = this->px4_node_->create_publisher<custom_msgs::msg::Position>(
		"/position", custom_qos);

	// Create timer for position publishing
	this->position_timer_ = this->px4_node_->create_wall_timer(
		std::chrono::milliseconds(50),  // 20 Hz
		[this]() {
			custom_msgs::msg::Position msg;

			// NED coordinates
			msg.x_ned = this->current_pos_x_;
			msg.y_ned = this->current_pos_y_;
			msg.z_ned = this->current_pos_z_;
			msg.yaw_ned = this->yaw_;

			msg.vx_ned = this->current_vel_x_;
			msg.vy_ned = this->current_vel_y_;
			msg.vz_ned = this->current_vel_z_;

			this->position_pub_->publish(msg);
		});
}

Drone::~Drone() {
	this->destroy();
}

void Drone::destroy() {
	if (this->exec_) {
		this->exec_->cancel();
		// Give some time for graceful shutdown
		rclcpp::sleep_for(std::chrono::milliseconds(100));
		this->exec_ = nullptr;
		rclcpp::shutdown();
		if (this->spin_thread_.joinable()) {
			this->spin_thread_.join();
		}
	}
}

// GETTERS

Eigen::Vector3d Drone::getLocalPosition() {
	return Eigen::Vector3d({
		this->current_pos_x_,
		this->current_pos_y_,
		this->current_pos_z_
	});
}

Eigen::Vector3d Drone::getOrientation() {
	return Eigen::Vector3d({
		this->roll_,
		this->pitch_,
		this->yaw_
	});
}

float Drone::getGroundSpeed() {
	return this->ground_speed_;
}

DronePX4::ARMING_STATE Drone::getArmingState() {
	return this->arming_state_;
}

// PRIVATE FUNCTIONS

void Drone::sendCommand(
	uint32_t command, uint8_t target_system, uint8_t target_component, uint8_t source_system,
	uint8_t source_component, uint8_t confirmation, bool from_external,
	float param1, float param2, float param3,
	float param4, float param5, float param6,
	float param7)
{
	px4_msgs::msg::VehicleCommand msg;
	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000.0;
	msg.command = command;

	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.confirmation = confirmation;
	msg.source_system = source_system;
	msg.target_system = target_system;
	msg.target_component = target_component;
	msg.from_external = from_external;
	msg.source_component = source_component;

	this->vehicle_command_pub_->publish(msg);
}

// SETTERS

void Drone::setOffboardControlMode(DronePX4::CONTROLLER_TYPE type) {
	px4_msgs::msg::OffboardControlMode msg;
	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.direct_actuator = false;

	if (type == DronePX4::CONTROLLER_TYPE::POSITION) {
		msg.position = true;
	} else if (type == DronePX4::CONTROLLER_TYPE::VELOCITY) {
		msg.velocity = true;
	} else if (type == DronePX4::CONTROLLER_TYPE::BODY_RATES) {
		msg.body_rate = true;
	} else {
		RCLCPP_WARN(this->px4_node_->get_logger(), "No controller is defined");
	}

	this->vehicle_offboard_control_mode_pub_->publish(msg);
}

void Drone::setLocalPosition(float x, float y, float z, float yaw) {
	this->setOffboardControlMode(DronePX4::CONTROLLER_TYPE::POSITION);

	px4_msgs::msg::TrajectorySetpoint msg;

	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

	msg.position[0] = x;
	msg.position[1] = y;
	msg.position[2] = z;
	msg.yaw = yaw;

	// Set velocity setpoints to NaN (not used for position control)
	msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
	msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
	msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
	msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

	msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

	this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void Drone::setOffboardMode() {
	this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		1.0f,
		6.0f
	);
}

void Drone::toOffboardSync() {
	// Send position setpoints for a short time before switching to offboard mode
	for (int i = 0; i < 20; i++) {
		setLocalPosition(
			current_pos_x_,
			current_pos_y_,
			current_pos_z_,
			std::numeric_limits<float>::quiet_NaN());
		setOffboardControlMode(DronePX4::CONTROLLER_TYPE::POSITION);
		rclcpp::sleep_for(std::chrono::milliseconds(100));
	}
	setOffboardMode();
}

void Drone::arm() {
    this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		1.0f
	);
}

void Drone::disarm() {
  	this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		0.0f
	);
}

void Drone::armSync() {
	while (getArmingState() != DronePX4::ARMING_STATE::ARMED && rclcpp::ok()) {
		this->arm();
		rclcpp::sleep_for(std::chrono::milliseconds(100));
	}
}

void Drone::disarmSync() {
	while (getArmingState() != DronePX4::ARMING_STATE::DISARMED && rclcpp::ok()) {
		this->disarm();
		rclcpp::sleep_for(std::chrono::milliseconds(100));
	}
}

void Drone::setLocalVelocity(float vx, float vy, float vz, float yaw_rate) {
	this->setOffboardControlMode(DronePX4::CONTROLLER_TYPE::VELOCITY);
	
	px4_msgs::msg::TrajectorySetpoint msg;

	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

	msg.position[0] = std::numeric_limits<float>::quiet_NaN();
	msg.position[1] = std::numeric_limits<float>::quiet_NaN();
	msg.position[2] = std::numeric_limits<float>::quiet_NaN();
	msg.yaw = std::numeric_limits<float>::quiet_NaN();

	msg.velocity[0] = vx;
	msg.velocity[1] = vy;
	msg.velocity[2] = vz;
	msg.yawspeed = yaw_rate;

	msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

	this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void Drone::log(const std::string &info) {
	RCLCPP_INFO(this->px4_node_->get_logger(), info.c_str());
}
