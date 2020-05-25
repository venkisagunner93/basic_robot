#include "front_steering_command_controller/front_steering_command_controller.h"

using namespace front_steering_command_controller;

FrontSteeringCommandController::FrontSteeringCommandController() : kinematics_(Kinematics(State())) {}

bool FrontSteeringCommandController::init(hardware_interface::FrontSteeringCommandInterface* hw, ros::NodeHandle& nh)
{
    RobotDimensions robot_dimensions;
    robot_dimensions.length = 0.5;  // in meters
    robot_dimensions.width = 0.3;  // in meters
    kinematics_.setRobotDimensions(robot_dimensions);

    const std::vector<std::string>& joint_names = hw->getNames();
    
    for(int i = 0; i < joint_names.size(); i++)
    {
        front_steering_command_handle_.push_back(hw->getHandle(joint_names[i]));
        ROS_INFO_STREAM("[Front Steering Command Controller]: " << joint_names[i].c_str() << " joint created");
    }

    cmd_vel_subscriber_ = nh.subscribe("base/cmd_vel", 100, &FrontSteeringCommandController::cmdVelCallback, this);

    return true;
}

void FrontSteeringCommandController::update(const ros::Time& time, const ros::Duration& period)
{
    FrontWheel front_wheel = kinematics_.computeFrontWheelHubAngles(control_input_);
    
    for(unsigned int i = 0; i < front_steering_command_handle_.size(); i++)
    {
        if(front_steering_command_handle_[i].getName() == "left_hub")
        {
            front_steering_command_handle_[i].setWheelAngle(front_wheel.left_hub_angle);
        }
        else if(front_steering_command_handle_[i].getName() == "right_hub")
        {
            front_steering_command_handle_[i].setWheelAngle(front_wheel.right_hub_angle);
        }
    }
}

void FrontSteeringCommandController::cmdVelCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
    control_input_ = msg;
}

PLUGINLIB_EXPORT_CLASS(front_steering_command_controller::FrontSteeringCommandController, controller_interface::ControllerBase);