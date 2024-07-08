/**
 * Timesync is performed through the uXRCE-DDS protocol as of release/1.14, otherwise
 * it should be implemented
 */


#include "v6c_offboard_control/offboard_control.hpp"

OffboardControl::OffboardControl() : Node("v6c_offboard_control")
{
    // Parameters definition
    this->declare_parameter<float>("setpoint_tolerance", 0.1);
    this->declare_parameter<std::string>("filepath", "/home/fabio/drone_offboard_ws/src/v6c_offboard_control/config/trajectory.txt");
    this->get_parameter<float>("setpoint_tolerance", setpoint_tolerance_);
    this->get_parameter<std::string>("filepath", filepath_);

    // 33Hz rate
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&OffboardControl::timer_callback, this));

    //QoS policies for subscriptions
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Heartbeat publisher
    this->offboard_ctrl_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    // Vehicle command publisher
    this->vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command",10);

    // Trajectory Setpoint publisher
    this->trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);

    // Vehicle local position subscription - trajectory setpoint definition
    this->vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos,
        std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1)
    );

    // Vehicle status subscription  ->  nav_state arming_state
    this->vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", qos,
        std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1)
    );

    // Trajectory definition from file
    std::ifstream file (filepath_);
    if(file.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "File FOUND");
        float f;
        struct Point temp;
        for(uint8_t i=1; file >> f; i++)
        {
            switch(i%3)
            {
                case 2 :
                    temp.x = f;
                    break;
                case 1 :
                    temp.y = f;
                    break;
                case 0 :
                    temp.z = f;
                    trajectory.push_back(temp);
                    RCLCPP_INFO(this->get_logger(), "Setpoint: [%.1f, %.1f, %.1f]", temp.x, temp.y, temp.z);
                    break;
                default :
                    break;
            }
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "File NOT FOUND, skip");
        // rclcpp::shutdown();
    }
}

// MAIN LOOP
void OffboardControl::timer_callback()
{
    static uint8_t timer_steps = 0;

    // Heartbeat
    publish_offboard_control_mode();

    switch(state_)
    {
        case State::init :
            engage_offboard_mode();
            state_ = State::offboard_check;
            break;
        
        case State::offboard_check :
            if(nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) // 14
            {
                RCLCPP_INFO(this->get_logger(), "Entered OFFBOARD mode");
                state_ = State::wait_for_stable_offboard;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to enter OFFBOARD mode, requesting again...");
                engage_offboard_mode();
            }
            break;
        
        case State::wait_for_stable_offboard :
            if(++timer_steps > 10)
            {
                arm();
                state_ = State::arm_check;
            }
            break;
        
        case State::arm_check:
            if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) // 2
            {
                if(++timer_steps > 100)
                {
                    RCLCPP_INFO(this->get_logger(), "Vehicle ARMED");
                    state_ = State::armed;
                }              
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to ARM, requesting again...");
                arm();
            }
            break;
        
        case State::armed :
            RCLCPP_INFO(this->get_logger(),"Takeoff");
            publish_trajectory_setpoint(local_pos_.x, local_pos_.y, -5.0, -3.14);
            state_ = State::takeoff;
            break;

        case State::takeoff :
        // check for divergence of UAV wrt setpoint defined >>> implement more checks
            if(!setpoint_reached_)
            {
                setpoint_reached_ = check_setpoint_distance(local_pos_, current_setpoint_) ? 1 : 0;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Takeoff setpoint reached at: [%1.1f, %1.1f, %1.1f]", current_setpoint_.x, current_setpoint_.y, current_setpoint_.z);
                setpoint_reached_ = 0;
                publish_trajectory_setpoint(0.0, 5.0, -5.0, -3.14);
                state_ = State::setpoint1;
            }
            break;

        case State::setpoint1 :
            if(!setpoint_reached_)
            {
                setpoint_reached_ = check_setpoint_distance(local_pos_, current_setpoint_) ? 1 : 0;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Setpoint1 reached at: [%1.1f, %1.1f, %1.1f]", current_setpoint_.x, current_setpoint_.y, current_setpoint_.z);
                setpoint_reached_ = 0;
                publish_trajectory_setpoint(-5.0, -5.0, -5.0, -3.14);
                state_ = State::setpoint2;
            }
            break;

        case State::setpoint2 :
            if(!setpoint_reached_)
            {
                setpoint_reached_ = check_setpoint_distance(local_pos_, current_setpoint_) ? 1 : 0;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Setpoint2 reached at: [%1.1f, %1.1f, %1.1f]", current_setpoint_.x, current_setpoint_.y, current_setpoint_.z);
                setpoint_reached_ = 0;
                publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14);
                state_ = State::approach_landing;
            }
            break;

        case State::approach_landing :
            if(!setpoint_reached_)
            {
                setpoint_reached_ = check_setpoint_distance(local_pos_, current_setpoint_) ? 1 : 0;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Home reached at: [%1.1f, %1.1f, %1.1f]", current_setpoint_.x, current_setpoint_.y, current_setpoint_.z);
                setpoint_reached_ = 0;
                publish_trajectory_setpoint(0.0, 0.0, -0.1, -3.14);
                state_ = State::land;
            }
            break;

        case State::land :
            if(!setpoint_reached_)
            {
                setpoint_reached_ = check_altitude_for_landing(local_pos_) ? 1 : 0;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Land altitude reached at: [%1.3f, %1.3f, %1.3f]", local_pos_.x, local_pos_.y, local_pos_.z);
                setpoint_reached_ = 0;
                disarm();
                state_ = State::landed_check;
            }
            break;

        case State::landed_check :
            if(arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY) // 1
            {
                RCLCPP_INFO(this->get_logger(), "Disarm command executed, hopefully you a had a safe land! :)");
                rclcpp::shutdown();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Disarm command NOT executed, DANGER: REITERATE!");
                disarm();
            }
            break;

        default:
            break;
    }
}

/**
 * Drone APIs
*/

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
	msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    vehicle_command_pub_->publish(msg);
}

void OffboardControl::arm()
{   
    RCLCPP_INFO(this->get_logger(), "Requesting ARM command");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196);
}

void OffboardControl::disarm()
{
    RCLCPP_INFO(this->get_logger(), "Requesting DISARM command");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196);
}

void OffboardControl::engage_offboard_mode()
{
    // Send command to switch to offboard mode
    RCLCPP_INFO(this->get_logger(), "Requesting switch to OFFBOARD mode");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1., 6.);
}

void OffboardControl::publish_trajectory_setpoint(const float & x, const float & y, const float & z, const float & yaw)
{
    current_setpoint_.x = x;
    current_setpoint_.y = y;
    current_setpoint_.z = z;

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    // NED local world frame
    msg.position = {
        x,
        y,
        z
    };
    msg.yaw = yaw; // [-PI; PI]
    trajectory_setpoint_pub_->publish(msg);
}

void OffboardControl::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode ctrl_mode_msg_;
    ctrl_mode_msg_.position = true;  
    ctrl_mode_msg_.velocity = false;
    ctrl_mode_msg_.acceleration = false;
    ctrl_mode_msg_.attitude = false; 
    ctrl_mode_msg_.body_rate = false;
    ctrl_mode_msg_.actuator = false;
    ctrl_mode_msg_.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    offboard_ctrl_mode_pub_->publish(ctrl_mode_msg_);
}

void OffboardControl::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition & msg)
{
    local_pos_.x = msg.x;
    local_pos_.y = msg.y;
    local_pos_.z = msg.z;
}

void OffboardControl::vehicle_status_callback(const px4_msgs::msg::VehicleStatus & msg)
{
    nav_state_ = msg.nav_state;
    arming_state_ = msg.arming_state;
}

bool OffboardControl::check_setpoint_distance(const struct Point & position, const struct Point & setpoint)
{
    auto square = [](float value) {return value*value;};

    return ( sqrt(square(position.x - setpoint.x) + square(position.y - setpoint.y) + (position.z - setpoint.z)) <= setpoint_tolerance_ );
}

bool OffboardControl::check_altitude_for_landing(const struct Point & position)
{
    return position.z >= -0.3 ? true : false;
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
 * 4Future: topics for each drone will be published in its namespace (ex. tag, aux1, aux2 ecc..)
 *
 * 
 * IMPROVE: • implement VehicleCommand.srv even for release/1.14, currently working on @main
 *          • define setpoints trajectory as a file to load on node init and save as a vector +
 *            single State-machine for handling setpoints
*/