#include <controller_manager/controller_manager.h>
#include "motion_control/basic_robot_sim.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_control_sim");
    ros::NodeHandle nh;

    BasicRobotSim sim;
    controller_manager::ControllerManager cm(&sim);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
        sim.read(ros::Time::now(), ros::Duration(1));
        cm.update(ros::Time::now(), ros::Duration(1));
        sim.write(ros::Time::now(), ros::Duration(1));
        ros::Duration(0.01).sleep();
    }

    spinner.stop();
}