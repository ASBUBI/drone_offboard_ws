#ifndef OFFBOARD_CONTROL_HPP_
#define OFFBOARD_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

#include <vector>
#include <chrono>
#include <iostream>
#include <cmath>

struct Setpoint {
    float x;
    float y;
    float z;
    float yaw;
}

class OffboardControl : public rclcpp::Node
{
    public:
        OffboardControl();

        // ROS2 main loop function (set freq)
        void timer_callback();

        // UAV related methods
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
        void arm();
        void disarm();
        void engage_offboard_mode();
        void engage_landing();

        // Publishers
        void publish_trajectory_setpoint(const struct Setpoint & setpoint);

        // Subscribers callbacks
        void vehicle_status_callback(const px4_msgs::msg::VehicleStatus & msg);
        void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition & msg);

        bool check_setpoint_distance(const px4_msgs::msg::VehicleLocalPosition & msg, const struct Setpoint & setpoint);
        void define_setpoint(const float x, const float y, const float z, const float yaw);

    private:
        px4_msgs::msg::OffboardControlMode ctrl_mode_msg_;

        // Loop-timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
        
        // Publishers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

        // Drone proprietary vars
        unsigned short vehicle_flight_mode_;
        unsigned short setpoints_reached_ = 0;
        unsigned float setpoint_tolerance_;
        auto current_setpoint_ = std::make_shared<Setpoint>(Setpoint{});
};

#endif