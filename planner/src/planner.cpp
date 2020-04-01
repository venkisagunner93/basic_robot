#include "planner/planner.h"

Planner::Planner(ros::NodeHandle& nh) : nh_(nh)
{
    loadParameters();
    initializeTransports();
}

void Planner::loadParameters()
{
    if(nh_.hasParam("sim/update_rate"))
    {
        nh_.getParam("sim/update_rate", update_rate_);
    }
    else
    {
        update_rate_ = 20;
    }
}

void Planner::initializeTransports()
{
    cmd_vel_publisher_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("base/cmd_vel", 100, true);
}

int Planner::getUpdateRate() const
{
    return update_rate_;
}

void Planner::publishSetpoints(const ackermann_msgs::AckermannDriveStamped& setpoint)
{
    cmd_vel_publisher_.publish(setpoint);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    Planner planner(nh);

    ackermann_msgs::AckermannDriveStamped setpoint;

    if(argc > 1)
    {
        setpoint.header.frame_id = "base_link";
        setpoint.drive.speed = atof(argv[1]);
        setpoint.drive.steering_angle = atof(argv[2]);
    }

    while(ros::ok())
    {
        ros::Rate(planner.getUpdateRate()).sleep();
        setpoint.header.stamp = ros::Time::now();
        planner.publishSetpoints(setpoint);
        ros::spinOnce();
    }
}