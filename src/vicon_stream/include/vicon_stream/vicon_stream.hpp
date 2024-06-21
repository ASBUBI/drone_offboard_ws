#ifndef VICON_CLIENT_HPP_
#define VICON_CLIENT_HPP_

#include "DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
// #include "px4_msgs/msg/timesync_status.hpp"

#include <chrono>
#include <iostream>
#include <csignal>
#include <unistd.h>

class ViconClient : public rclcpp::Node
{
    public:
        ViconClient();

        bool connect();
        bool disconnect();
        void stream();

        void timer_callback();

        void publish_vehicle_odometry(ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation position, ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion);
        // void timesync_callback(const px4_msgs::msg::TimesyncStatus & msg);

        // Publishers
        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_pub_;

        // Subscribers
        // rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;

    private:
        ViconDataStreamSDK::CPP::Client vicon_client_;
        std::string hostname_;
        unsigned int buffer_size_;
        // unsigned int timesync_;

        rclcpp::TimerBase::SharedPtr timer_;
};

#endif

/**
 * Introducing modification for timesync wrt to previous build
 */