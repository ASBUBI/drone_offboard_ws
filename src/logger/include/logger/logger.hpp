#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>

class Logger : public rclcpp::Node
{
    public:
        Logger();
        void mocap_odometry_callback(const px4_msgs::msg::VehicleOdometry & msg);
        void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry & msg);
        void land_detection_callback(const px4_msgs::msg::VehicleLandDetected & msg);
        void trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint & msg);
        void vehicle_status_callback(const px4_msgs::msg::VehicleStatus & msg);
        void save_to_file();

    private:
        std::string filename_;
        std::filesystem::path file_path_;
        std::ofstream file_;
        nlohmann::json data_;

        // Subscriptions
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr mocap_odometry_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detection_sub_;
        rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
};  

#endif

/**
 * TEMPLATE to add topics to be logged
 * • define rclcpp::Subscription
 * • create callback method
 * • include message header
 * • inside callback define structure to be saved as json in data_.push_back()
 * • link for json structures -> https://github.com/nlohmann/json?tab=readme-ov-file#creating-json-objects-from-json-literals
 * • if using same json structure (data_) the topics are loaded into file automatically by save_to_file
*/