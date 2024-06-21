#include "logger/logger.hpp"

Logger::Logger() : Node("logger")
{
    // Parameters definition
    const auto now = std::chrono::system_clock::now();
    const std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    this->filename_ =  std::string(std::ctime(&t_c)) +".json"; //as_string ?

    // Extract file location
    char* path_ptr = std::getenv("COLCON_PREFIX_PATH");
    char* ptr = std::strtok(path_ptr, ":");
    while(ptr != nullptr)
    {
        if(std::string(ptr).find("drone_offboard_ws") != std::string::npos)
        {
            this->file_path_ = std::filesystem::path(std::string(ptr)).parent_path() / "src/logger/logs";
        }
        ptr = std::strtok(nullptr,":");
    }
    if(!std::filesystem::exists(this->file_path_)) std::filesystem::create_directory(this->file_path_);
    this->file_ = std::ofstream(file_path_ / filename_);

    // Subscriptions
    
    this->mocap_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", rclcpp::SensorDataQoS(),
        std::bind(&Logger::mocap_odometry_callback, this, std::placeholders::_1)
    );

    this->vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&Logger::vehicle_odometry_callback, this, std::placeholders::_1)
    );

    this->land_detection_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
        std::bind(&Logger::land_detection_callback, this, std::placeholders::_1)
    );

    this->trajectory_setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS(),
        std::bind(&Logger::trajectory_setpoint_callback, this, std::placeholders::_1)
    );

    this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
        std::bind(&Logger::vehicle_status_callback, this, std::placeholders::_1)
    );

}

void Logger::mocap_odometry_callback(const px4_msgs::msg::VehicleOdometry & msg)
{
    data_.push_back({
        {"topic", "mocap_odometry"},
        {"position",{
            {"x",msg.position[0]},
            {"y",msg.position[1]},
            {"z",msg.position[2]}
        }},
        {"q", {
            {"q0",msg.q[0]},
            {"q1",msg.q[1]},
            {"q2",msg.q[2]},
            {"q3",msg.q[3]}
        }},
        {"timestamp", msg.timestamp}
    });
}

void Logger::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry & msg)
{
    data_.push_back({
        {"topic", "vehicle_odometry"},
        {"position",{
            {"x",msg.position[0]},
            {"y",msg.position[1]},
            {"z",msg.position[2]}
        }},
        {"q", {
            {"q0",msg.q[0]},
            {"q1",msg.q[1]},
            {"q2",msg.q[2]},
            {"q3",msg.q[3]}
        }},
        {"timestamp", msg.timestamp}
    });
}

void Logger::land_detection_callback(const px4_msgs::msg::VehicleLandDetected & msg)
{
    data_.push_back({
        {"topic","land_detection"},
        {"timestamp",msg.timestamp},
        {"freefall", msg.freefall},
        {"ground_contact",msg.ground_contact},
        {"maybe_landed", msg.maybe_landed},
        {"landed", msg.landed},
        {"in_ground_effect", msg.in_ground_effect},
        {"vertical_movement", msg.vertical_movement},
        {"horizontal_movement", msg.horizontal_movement},
        {"rotational_movement", msg.rotational_movement},
        {"at_rest", msg.at_rest},
        {"in_descend", msg.in_descend},
        {"has_low_throttle", msg.has_low_throttle}
    });
}

void Logger::trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint & msg)
{
    data_.push_back({
        {"topic", "trajectory_setpoint"},
        {"timestamp", msg.timestamp},
        {"position", {
            {"x", msg.position[0]},
            {"y", msg.position[1]},
            {"z", msg.position[2]}
        }},
        {"yaw", msg.yaw}
    });
}

void Logger::vehicle_status_callback(const px4_msgs::msg::VehicleStatus & msg)
{
    data_.push_back({
        {"topic", "vehicle_status"},
        {"timestamp", msg.timestamp},
        {"armed_time", msg.armed_time},
        {"takeoff_time", msg.takeoff_time},
        {"arming_state", msg.arming_state},
        {"nav_state_user_intention", msg.nav_state_user_intention},
        {"nav_state", msg.nav_state},
        {"failure_detector_status", msg.failure_detector_status},
        {"failsafe", msg.failsafe}
    });
}

void Logger::save_to_file()
{
    file_ << data_;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Logger>();
    rclcpp::spin(node);
    node->save_to_file();
    rclcpp::shutdown();
    return 0;
}