#include "rear_drive_command_controller/rear_drive_command_controller.h"

using namespace rear_drive_command_controller;

RearDriveCommandController::RearDriveCommandController() : kinematics_(Kinematics(State())) {}

bool RearDriveCommandController::init(hardware_interface::RearDriveCommandInterface* hw, ros::NodeHandle& nh)
{
    const std::vector<std::string>& joint_names = hw->getNames();
    
    for(int i = 0; i < joint_names.size(); i++)
    {
        rear_drive_command_handle_.push_back(hw->getHandle(joint_names[i]));
        ROS_INFO_STREAM("[Rear Drive Command Controller]: " << joint_names[i].c_str() << " joint created");
    }

    cmd_vel_subscriber_ = nh.subscribe("/base/cmd_vel", 100, &RearDriveCommandController::cmdVelCallback, this);

    return true;
}

void RearDriveCommandController::update(const ros::Time& time, const ros::Duration& period)
{
    for(unsigned int i = 0; i < rear_drive_command_handle_.size(); i++)
    {
        rear_drive_command_handle_[i].setVelocity(control_input_.drive.speed);
    }
}

void RearDriveCommandController::cmdVelCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
    control_input_ = msg;
}

PLUGINLIB_EXPORT_CLASS(rear_drive_command_controller::RearDriveCommandController, controller_interface::ControllerBase);