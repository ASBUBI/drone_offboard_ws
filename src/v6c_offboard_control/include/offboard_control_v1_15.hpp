#ifndef OFFBOARD_CONTROL_HPP_
#define OFFBOARD_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/srv/vehicle_command.hpp"

#include <vector>
#include <chrono>
#include <iostream>
#include <cmath>
#include <unistd.h>

struct Point {
    float x;
    float y;
    float z;
};

enum class State {
    init,
    offboard_requested,
    wait_for_stable_offboard,
    arm_requested,
    armed,
    takeoff,
    setpoint1,
    setpoint2,
    approach_landing,
    land,
    landed_check
};

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

        // Publishers
        void publish_trajectory_setpoint(const float & x, const float & y, const float & z, const float & yaw);
        void publish_offboard_control_mode();

        // Subscribers callbacks
        void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition & msg);

        // Services
        void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
        void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture response);

        bool check_setpoint_distance(const struct Point & position, const struct Point & setpoint);
        bool check_altitude_for_landing(const struct Point & position);
        void define_setpoint(const float x, const float y, const float z);

    private:
        // Loop-timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
        
        // Publishers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

        //Client for Services
        rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
        uint8_t service_result_;
        bool service_done_;

        // Drone proprietary vars
        unsigned short vehicle_flight_mode_;
        unsigned short setpoint_reached_ = 0;
        float setpoint_tolerance_;
        struct Point current_setpoint_;
        struct Point local_pos_;
        State state_;
};

#endif