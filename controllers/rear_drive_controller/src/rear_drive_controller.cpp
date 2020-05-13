#include "rear_drive_controller/rear_drive_controller.h"

bool RearDriveController::init(hardware_interface::RearDriveInterface* hw, ros::NodeHandle& nh)
{
    left_rear_drive_ = hw->getHandle("left_wheel");
    right_rear_drive_ = hw->getHandle("right_wheel");
    cmd_vel_subscriber_ = nh.subscribe("base/cmd_vel", 100, &RearDriveController::cmdVelCallback, this);
}

void RearDriveController::update(const ros::Time& time, const ros::Duration& period)
{
    
}

void RearDriveController::cmdVelCallback(const ackermann_msgs::AckermannDriveStamped& msg)
{
    control_input_ = msg;
}

void RearDriveController::publishVelFeedback() const
{

}

PLUGINLIB_EXPORT_CLASS(RearDriveController, controller_interface::ControllerBase)