#include "planner/manual_planner.h"

using namespace planner_plugins;

bool ManualPlanner::initialize(ros::NodeHandle& nh)
{
    cmd_vel_publisher_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/base/cmd_vel", 100, true);
    joystick_subscriber_ = nh.subscribe("/joy", 100, &ManualPlanner::joystickCallback, this);

    return true;
}

void ManualPlanner::setLimits(const Limits& limits)
{
    limits_ = limits;
}

void ManualPlanner::joystickCallback(const sensor_msgs::Joy& msg)
{
    setpoint_.header.stamp = ros::Time::now();
    setpoint_.drive.speed = msg.axes[1] * limits_.max_velocity;
    setpoint_.drive.steering_angle = msg.axes[2] * limits_.max_steering_angle;
}

void ManualPlanner::sendSetpoints()
{
    cmd_vel_publisher_.publish(setpoint_);
}

PLUGINLIB_EXPORT_CLASS(planner_plugins::ManualPlanner, planner_interface::PlannerBase);