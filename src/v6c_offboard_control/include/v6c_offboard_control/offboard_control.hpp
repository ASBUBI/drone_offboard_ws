#ifndef OFFBOARD_CONTROL_HPP_
#define OFFBOARD_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include <vector>
#include <chrono>
#include <iostream>

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

        void publish_trajectory_setpoint(std::vector<float> position_setpoint, float yaw_setpoint);

        // Subscribers callbacks
        void vehicle_status_callback(const px4_msgs::msg::VehicleStatus & msg);

    private:
        px4_msgs::msg::OffboardControlMode ctrl_mode_msg_;
        unsigned short vehicle_flight_mode_;

        rclcpp::TimerBase::SharedPtr timer_;

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        
        // Publishers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

};

#endif