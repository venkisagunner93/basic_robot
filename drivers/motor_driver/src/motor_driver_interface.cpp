#include "motor_driver/motor_driver_interface.h"

MotorDriverInterface::MotorDriverInterface()
{
    std::vector<std::string> joint_names = {"left_wheel", "right_wheel"};

    rear_drive_feedback_.registerHandle(hardware_interface::RearDriveStateHandle(joint_names[0], &position_feedback_[0], &velocity_feedback_[0]));
    rear_drive_feedback_.registerHandle(hardware_interface::RearDriveStateHandle(joint_names[1], &position_feedback_[1], &velocity_feedback_[1]));

    registerInterface(&rear_drive_feedback_);

    rear_drive_command_.registerHandle(hardware_interface::RearDriveCommandHandle(rear_drive_feedback_.getHandle(joint_names[0]), &position_command_[0], &velocity_command_[0]));
    rear_drive_command_.registerHandle(hardware_interface::RearDriveCommandHandle(rear_drive_feedback_.getHandle(joint_names[1]), &position_command_[1], &velocity_command_[1]));
    
    registerInterface(&rear_drive_command_);
}

void MotorDriverInterface::read(const ros::Time& time, const ros::Duration& period)
{
    hardware_interface::RobotHW::read(time, period);
}

void MotorDriverInterface::write(const ros::Time& time, const ros::Duration& period) 
{
    hardware_interface::RobotHW::write(time, period);
}