#include "v6c_offboard_control/offboard_control.hpp"

OffboardControl::OffboardControl() : Node("v6c_offboard_control")
{
    // Parameters definition
    this->declare_parameter<float>("setpoint_tolerance", 0.1);
    this->get_parameter<float>("setpoint_tolerance", setpoint_tolerance_);

    // 50Hz rate
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&OffboardControl::timer_callback, this));

    // Type of control modes enabled
    this->ctrl_mode_msg_.position = true;
    this->ctrl_mode_msg_.velocity = false;
    this->ctrl_mode_msg_.acceleration = false;
    this->ctrl_mode_msg_.attitude = false;
    this->ctrl_mode_msg_.body_rate = false;
    this->ctrl_mode_msg_.thrust_and_torque = false;
    this->ctrl_mode_msg_.direct_actuator = false;
    this->offboard_ctrl_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());

    // Vehicle command init
    this->vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command",10);

    // Vehicle Status subscribtion -> to understand vehicle current flight mode
    this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "fmu/out/vehicle_status", rclcpp::SensorDataQoS(), 
        std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1)
    );
    // Vehicle local position subscription - trajectory setpoint definition
    this->vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1)
    );
}

/**
 * Drone APIs
*/

// MAIN LOOP
void OffboardControl::timer_callback()
{
    if(vehicle_flight_mode_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
    {
        // OffboardControlMode publish - Heartbeat
        ctrl_mode_msg_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        offboard_ctrl_mode_pub_->publish(ctrl_mode_msg_);
    }

    switch(setpoints_reached_)
    {
        case 0:
            // Setup the drone to takeoff (sent as first setpoint)
            arm();
            engage_offboard_mode();
            // Define first setpoint (takeoff)
            define_setpoint(0.0, 0.0, -1.0, 0.0);
            publish_trajectory_setpoint(current_setpoint_);
            break;

        case 1:           
            define_setpoint(-1.0, 0.0, -1.0, 0.0);
            publish_trajectory_setpoint(current_setpoint_);           
            break;
        
        case 2:
            define_setpoint(-1.0, 0.0, 0.1, 0.0);
            publish_trajectory_setpoint(current_setpoint_);
            break;

        case 3:
            engage_landing();
            break;
    }
}

// MAVLink common message set
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    vehicle_command_pub_->publish(msg);
}

void OffboardControl::arm()
{   // Run pre-arm checks
    // publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS)
    
    // Send Arming command
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(
        this->get_logger(),
        "Arm command sent!"
    );

    sleep(1); // wait 1 second after arm command is sent
}

void OffboardControl::disarm()
{
    // Send Disrming command
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(
        this->get_logger(),
        "Disarm command sent!"
    );
    
}

void OffboardControl::engage_offboard_mode()
{
    // Send command to switch to offboard mode
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);

    RCLCPP_INFO(
        this->get_logger(),
        "Switching to OFFBOARD mode..."
    );

    sleep(1); // wait 1 second after engaging offboard mode
}

void OffboardControl::engage_landing()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

    RCLCPP_INFO(
        this->get_logger(),
        "Switching to AUTO-LAND mode..."
    );
}

void OffboardControl::publish_trajectory_setpoint(const struct Setpoint & setpoint)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    // NED local world frame
    msg.position = {
        setpoint.x,
        setpoint.y,
        setpoint.z
    };
    msg.yaw = setpoint.yaw; // [-PI; PI]
}

void OffboardControl::vehicle_status_callback(const px4_msgs::msg::VehicleStatus & msg)
{
    switch(vehicle_flight_mode_ = msg.nav_state)
    {
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL: //0
            RCLCPP_INFO(
                this->get_logger(),
                "Mode: MANUAL\n"
            );
            break;
            
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER: //4
            RCLCPP_INFO(
                this->get_logger(),
                "Mode: AUTO-LOITER\n"
            );
            break;

        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD: //14
            RCLCPP_INFO(
                this->get_logger(),
                "Mode: OFFBOARD\n"
            );
            break;

        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND: //18
            RCLCPP_INFO(
                this->get_logger(),
                "Mode: AUTO-LAND\n"
            );
            break;

        default:
            RCLCPP_INFO(
                this->get_logger(),
                "Mode: NOT-DEFINED by script\n"
            );
            break;
    }
}

void OffboardControl::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition & msg)
{
    // check if UAV is inside sphere of radius "tolerance" centered in the setpoint
    if(check_setpoint_distance(msg, current_setpoint_))
    {
        ++setpoints_reached_;
    }
    
}

bool OffboardControl::check_setpoint_distance(const px4_msgs::msg::VehicleLocalPosition & msg, const struct Setpoint & setpoint)
{
    auto square = [](float value) {return value*value;};

    return ( sqrt(square(msg.x - setpoint.x) + square(msg.y - setpoint.y) + (msg.z - setpoint.z)) <= setpoint_tolerance_ );
}

void OffboardControl::define_setpoint(const float x, const float y, const float z, const float yaw)
{
    current_setpoint_.x = x;
    current_setpoint_.y = y;
    current_setpoint_.z = z;
    current_setpoint_.yaw = yaw;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<OffboardControl>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

/**
 * Maybe switch VehicleStatus message to service call instead of topic pub/sub
 * 
*/