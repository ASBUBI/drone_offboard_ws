#include "vicon_stream/vicon_stream.hpp"

ViconClient::ViconClient() : Node("vicon_stream")
{
    // Parameters definition
    this->declare_parameter<std::string>("hostname","192.168.50.56");
    this->declare_parameter<int>("buffer_size",200);
    this->get_parameter("hostname", hostname_);
    this->get_parameter("buffer_size", buffer_size_);

    //QoS policies for subscriptions
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Should be between 30 Hz (if sent with covariances) and 50 Hz ---> source px4.docs
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&ViconClient::timer_callback, this));

    // Publishers
    this->vehicle_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

    //Subscriners
    // this->timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
    //     "/fmu/out/timesync_status", qos, std::bind(&ViconClient::timesync_callback, this, std::placeholders::_1)
    // );

}

bool ViconClient::connect()
{
    // Signal handler to catch SIGINT if stuck not connecting  
    signal(SIGINT, [](int sig_num){exit(sig_num);});

    std::cout << "Connecting to " + hostname_ + "..." << std::endl;
    while(!vicon_client_.IsConnected().Connected)
    {
        bool ok = (vicon_client_.Connect(hostname_).Result == ViconDataStreamSDK::CPP::Result::Success);
        if(!ok)
        {
            std::cout << "Connection failed, reconnecting... " << std::endl;
            sleep(1);
        }
    }

    std::cout << "Connection successfully estabilished with " + hostname_ + " on port 801 (default)" << std::endl;

    // Vicon client initializations
    vicon_client_.EnableSegmentData();
    vicon_client_.EnableMarkerData();
    vicon_client_.EnableUnlabeledMarkerData();
    vicon_client_.EnableMarkerRayData();
    vicon_client_.EnableDeviceData();
    vicon_client_.EnableDebugData();
    vicon_client_.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
    vicon_client_.SetBufferSize(buffer_size_);

    std::cout << "Vicon client initializations completed" << std::endl;

    return true;
}

bool ViconClient::disconnect()
{
    sleep(1);

    // Disable initializations for client
    vicon_client_.DisableSegmentData();
    vicon_client_.DisableMarkerData();
    vicon_client_.DisableUnlabeledMarkerData();
    vicon_client_.DisableMarkerRayData();
    vicon_client_.DisableDeviceData();
    vicon_client_.DisableDebugData();
    vicon_client_.DisableCentroidData();

    std::cout << "Disconnecting from " + hostname_ + "..." << std::endl; 
    vicon_client_.Disconnect();
    if(!vicon_client_.IsConnected().Connected)
    {
        std::cout << "Successfully disconnected!" << std::endl;
        return true;
    }
    return false;
}

void ViconClient::stream()
{
    auto frame = vicon_client_.GetFrame();

    if(frame.Result == ViconDataStreamSDK::CPP::Result::Success)
    {
        unsigned int subject_count = vicon_client_.GetSubjectCount().SubjectCount;
        for(unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
        {
            std::string subject_name = vicon_client_.GetSubjectName(subject_index).SubjectName;
            unsigned int segment_count = vicon_client_.GetSegmentCount(subject_name).SegmentCount;
            for(unsigned int segment_index = 0; segment_index < segment_count; ++segment_index)
            {
                std::string segment_name = vicon_client_.GetSegmentName(subject_name, segment_index).SegmentName;

                auto trans = vicon_client_.GetSegmentGlobalTranslation(subject_name, segment_name);
                auto rot = vicon_client_.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
                publish_vehicle_odometry(trans, rot);
            }
        }
    }
}

// ROS2 main loop callback (200Hz)
void ViconClient::timer_callback()
{
    if(vicon_client_.IsConnected().Connected)
    {
        stream();
    }
}

void ViconClient::publish_vehicle_odometry(ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation position, ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion attitude_q)
{
    px4_msgs::msg::VehicleOdometry msg{};
    msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // msg.timestamp = timesync_;
    
    // To check || Need to define ViconClient axis mapping to FRD before
    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    msg.position = {
        static_cast<float>(position.Translation[0]) / float(1000),
        -static_cast<float>(position.Translation[1]) / float(1000),
        -static_cast<float>(position.Translation[2]) / float(1000) 
    };
    msg.q = {
        static_cast<float>(attitude_q.Rotation[3]),
        static_cast<float>(attitude_q.Rotation[0]),
        -static_cast<float>(attitude_q.Rotation[1]),
        -static_cast<float>(attitude_q.Rotation[2])
    };
    msg.position_variance = {0.001, 0.001, 0.001};

    vehicle_odometry_pub_->publish(msg);
}

// void ViconClient::timesync_callback(const px4_msgs::msg::TimesyncStatus & msg)
// {
//     timesync_ = msg.timestamp;
// }

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViconClient>();
    node->connect();

    rclcpp::spin(node);

    node->disconnect();
    rclcpp::shutdown();
    return 0;
}

/**
 * TODO:
 *  • Logger drone position tracked by Vicon in new Node to not interfere with communication for local position -> ESSENTIAL
 *  • Maybe retrieve data from drone controls to understand what happens and what it does
 *  • Ensure safety kill is working properly -> ABSOLUTELY NECESSARY
*/