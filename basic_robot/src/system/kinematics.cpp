#include "system/kinematics.h"

Kinematics::Kinematics(const State& initial_state)
{
    state_ = initial_state;
    last_time_ = ros::Time::now();
}

State Kinematics::computeStates(ackermann_msgs::AckermannDriveStamped& control_input)
{   
    if(control_input.drive.steering_angle > 30.0)
    {
        control_input.drive.steering_angle = 30.0;
        ROS_WARN("Steering angle cannot be greater than 30.0 degrees");
    }
    else if(control_input.drive.steering_angle < -30.0)
    {
        control_input.drive.steering_angle = -30.0;
        ROS_WARN("Steering angle cannot be less than -30.0 degrees");
    }
    else
    {
        control_input.drive.steering_angle = control_input.drive.steering_angle * M_PI / 180;
    }
    
    ros::Duration dt = ros::Time::now() - last_time_;

    state_.x = state_.x + control_input.drive.speed * cos(state_.theta) * dt.toSec();
    state_.y = state_.y + control_input.drive.speed * sin(state_.theta) * dt.toSec();
    state_.theta = state_.theta + control_input.drive.speed * tan(control_input.drive.steering_angle) * dt.toSec() / 0.3;
    
    last_time_ = ros::Time::now();

    return state_;
}