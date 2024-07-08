/**
 * Payload Data Structure from locqt Server (bytes)
 * 0-3      ->      frame_number
 * 4        ->      frame_ID
 * 5-12     ->      translation.x
 * 13-20    ->      translation.y   
 * 21-28    ->      translation.z
 * 29-36    ->      rotation.quaternion.x
 * 37-44    ->      rotation.quaternion.y
 * 45-52    ->      rotation.quaternion.z
 * 53-60    ->      rotation.quaternion.scalar
 */

#include "external_position_comm/communicator.hpp"

Communicator::Communicator() : Node("external_position_comm")
{
    // Parameter def
    this->declare_parameter<uint16_t>("port", 51001);
    this->get_parameter<uint16_t>("port", port_);

    // 33Hz
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&Communicator::timer_callback, this));

    // Publishers
    this->odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/uwb_position", 10);
}

void Communicator::timer_callback()
{
    ssize_t rx_length = recvfrom(sock_, (char *) buffer_, sizeof(buffer_), MSG_WAITALL, (struct sockaddr *) &server_, &socket_length_);
    RCLCPP_INFO(this->get_logger(), "rx_length: %ld", rx_length);
    buffer_[rx_length] = '\0';

    // Unpack from locqt Server
    memcpy(&frame_number_, buffer_+0, 4);
    memcpy(&frame_ID_, buffer_+4, 1);

    memcpy(&position_.translation[0], buffer_+5, 8);
    memcpy(&position_.translation[1], buffer_+13, 8);
    memcpy(&position_.translation[2], buffer_+21, 8);

    memcpy(&position_.rotation_q[0], buffer_+29, 8);
    memcpy(&position_.rotation_q[1], buffer_+37, 8);
    memcpy(&position_.rotation_q[2], buffer_+45, 8);
    memcpy(&position_.rotation_q[3], buffer_+53, 8);

    // ROS2 publishing
    px4_msgs::msg::VehicleOdometry msg{};
    msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    msg.position = {
        static_cast<float>(position_.translation[0]),
        -static_cast<float>(position_.translation[1]),
        -static_cast<float>(position_.translation[2]) 
    };
    msg.q = {
        static_cast<float>(position_.rotation_q[3]),
        static_cast<float>(position_.rotation_q[0]),
        -static_cast<float>(position_.rotation_q[1]),
        -static_cast<float>(position_.rotation_q[2])
    };
    msg.position_variance = {0.001, 0.001, 0.001};

    odometry_pub_->publish(msg);

    // DEBUG
    RCLCPP_INFO(this->get_logger(), "Position received: [%.2f, %.2f, %.2f]", msg.position[0], msg.position[1], msg.position[2]);
}

void Communicator::connect()
{
    // Signal Handler
    signal(SIGINT, [](int sig_num){exit(sig_num);});
    
    // Socket init
    if( (sock_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "Socket creation failed"); 
        exit(EXIT_FAILURE);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Socket created");
    }

    // Zero-out server address structure
    std::memset(&server_, 0, sizeof(server_));

    server_.sin_addr.s_addr = INADDR_ANY; //inet_addr("192.168.50.56");;
    server_.sin_family = AF_INET;
    server_.sin_port = htons(port_);
    socket_length_ = sizeof(server_);

    // Bind the socket to any valid IP address and a specific port
    RCLCPP_INFO(this->get_logger(), "Waiting for bind...");
    if( bind(sock_, (const struct sockaddr *)&server_, sizeof(server_)) < 0 ) 
    { 
        RCLCPP_ERROR(this->get_logger(),"Bind failed"); 
        sleep(1);
        // exit(EXIT_FAILURE); 
    }
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Bind completed to ADDR: %d, PORT: %u", ntohl(server_.sin_addr.s_addr), ntohs(server_.sin_port));
    }
}

void Communicator::disconnect()
{
    close(sock_);
}

int main(int argc, char *  argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();

    node->connect();
    rclcpp::spin(node);
    node->disconnect();

    rclcpp::shutdown();
    return 0;
}