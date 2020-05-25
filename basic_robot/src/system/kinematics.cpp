#include "system/kinematics.h"

Kinematics::Kinematics(const State& initial_state)
{
    state_ = initial_state;
    last_time_ = ros::Time::now();
}

void Kinematics::setRobotDimensions(const RobotDimensions& robot_dimensions)
{
    robot_dimensions_ = robot_dimensions;
}

RobotDimensions Kinematics::getRobotDimensions() const
{
    return robot_dimensions_;
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
    state_.theta = state_.theta + control_input.drive.speed * tan(control_input.drive.steering_angle) * dt.toSec() / robot_dimensions_.width;
    
    last_time_ = ros::Time::now();

    return state_;
}

FrontWheel Kinematics::computeFrontWheelHubAngles(const ackermann_msgs::AckermannDriveStamped& control_input)
{
    FrontWheel front_wheel;
    
    // Instantaneous center of curvature
    float icc = robot_dimensions_.length / tan(control_input.drive.steering_angle * M_PI / 180);

    front_wheel.right_hub_angle = atan2(robot_dimensions_.length, (icc - robot_dimensions_.width / 2));
    front_wheel.left_hub_angle = atan2(robot_dimensions_.length, (icc + robot_dimensions_.width / 2));

    return front_wheel;
}

RearWheel Kinematics::computeRearWheelVelocities(const ackermann_msgs::AckermannDriveStamped& control_input)
{
    RearWheel rear_wheel;

    rear_wheel.right_velocity = control_input.drive.speed * (1 - (robot_dimensions_.width * tan(control_input.drive.steering_angle * M_PI / 180) \
                                                                    / (2 * robot_dimensions_.length)));
    rear_wheel.left_velocity = control_input.drive.speed * (1 + (robot_dimensions_.width * tan(control_input.drive.steering_angle * M_PI / 180) \
                                                                    / (2 * robot_dimensions_.length)));

    return rear_wheel;
}