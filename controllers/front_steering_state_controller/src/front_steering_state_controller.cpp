#include "front_steering_state_controller/front_steering_state_controller.h"

using namespace front_steering_state_controller;

FrontSteeringStateController::FrontSteeringStateController() : kinematics_(Kinematics(State())) {}

bool FrontSteeringStateController::init(hardware_interface::FrontSteeringStateInterface* hw, ros::NodeHandle& nh)
{
    RobotDimensions robot_dimensions;
    robot_dimensions.length = 0.5;  // in meters
    robot_dimensions.width = 0.3;  // in meters
    kinematics_.setRobotDimensions(robot_dimensions);

    const std::vector<std::string>& joint_names = hw->getNames();
    
    for(int i = 0; i < joint_names.size(); i++)
    {
        front_steering_state_handle_.push_back(hw->getHandle(joint_names[i]));
        ROS_INFO_STREAM("[Rear Drive State Controller]: " << joint_names[i].c_str() << " joint created");
    }

    cmd_vel_subscriber_ = nh.subscribe("/base/cmd_vel", 100, &FrontSteeringStateController::cmdVelCallback, this);

    return true;
}

void FrontSteeringStateController::update(const ros::Time& time, const ros::Duration& period)
{
    kinematics_.computeFrontWheelHubAngles(control_input_);
}

void FrontSteeringStateController::cmdVelCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
    control_input_ = msg;
}

PLUGINLIB_EXPORT_CLASS(front_steering_state_controller::FrontSteeringStateController, controller_interface::ControllerBase);