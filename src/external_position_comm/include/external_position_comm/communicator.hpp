#ifndef COMMUNICATOR_HPP_
#define COMMUNICATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include <chrono>
#include <csignal>
#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <cstdlib>

#define MAXLINE 1024

struct Position {
    double translation[3];
    double rotation_q[4];
};

class Communicator : public rclcpp::Node
{
    public:
        Communicator();

        void timer_callback();
        void connect();
        void disconnect();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_pub_;
        
        // locqt Server Rx
        struct Position position_;
        uint32_t frame_number_;
        uint8_t frame_ID_;

        // UDP protocol 
        int sock_;
        uint16_t port_;
        struct sockaddr_in server_;
        char buffer_[MAXLINE];   
        socklen_t socket_length_;     
};

#endif