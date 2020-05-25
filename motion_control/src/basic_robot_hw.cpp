#include "motion_control/basic_robot_hw.h"

BasicRobotHW::BasicRobotHW()
{
    std::vector<std::string> rear_joint_names = {"left_wheel", "right_wheel"};
    std::vector<std::string> front_joint_names = {"left_hub", "right_hub"};

    rear_drive_feedback_.registerHandle(hardware_interface::RearDriveStateHandle(rear_joint_names[0], &position_feedback_[0], &velocity_feedback_[0]));
    rear_drive_feedback_.registerHandle(hardware_interface::RearDriveStateHandle(rear_joint_names[1], &position_feedback_[1], &velocity_feedback_[1]));

    registerInterface(&rear_drive_feedback_);

    rear_drive_command_.registerHandle(hardware_interface::RearDriveCommandHandle(rear_drive_feedback_.getHandle(rear_joint_names[0]), &position_command_[0], &velocity_command_[0]));
    rear_drive_command_.registerHandle(hardware_interface::RearDriveCommandHandle(rear_drive_feedback_.getHandle(rear_joint_names[1]), &position_command_[1], &velocity_command_[1]));
    
    registerInterface(&rear_drive_command_);

    front_steering_feedback_.registerHandle(hardware_interface::FrontSteeringStateHandle(front_joint_names[0], &wheel_angle_feedback_[0]));
    front_steering_feedback_.registerHandle(hardware_interface::FrontSteeringStateHandle(front_joint_names[1], &wheel_angle_feedback_[1]));

    registerInterface(&front_steering_feedback_);

    front_steering_command_.registerHandle(hardware_interface::FrontSteeringCommandHandle(front_steering_feedback_.getHandle(front_joint_names[0]), &wheel_angle_command_[0]));
    front_steering_command_.registerHandle(hardware_interface::FrontSteeringCommandHandle(front_steering_feedback_.getHandle(front_joint_names[1]), &wheel_angle_command_[1]));
    
    registerInterface(&front_steering_command_);
}

void BasicRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
    hardware_interface::RobotHW::read(time, period);
}

void BasicRobotHW::write(const ros::Time& time, const ros::Duration& period) 
{
    hardware_interface::RobotHW::write(time, period);
}