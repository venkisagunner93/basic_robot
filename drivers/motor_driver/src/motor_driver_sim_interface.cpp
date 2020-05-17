#include "motor_driver/motor_driver_sim_interface.h"
#include <ros/ros.h>

void MotorDriverSimInterface::read(const ros::Time& time, const ros::Duration& period)
{
    
}

void MotorDriverSimInterface::write(const ros::Time& time, const ros::Duration& period)
{
    velocity_feedback_[0] = velocity_command_[0];
    velocity_feedback_[1] = velocity_command_[1];
}

float MotorDriverSimInterface::getLoopRate() const
{
    return 0.1;
}