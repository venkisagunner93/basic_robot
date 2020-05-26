#include <controller_manager/controller_manager.h>
#include "motion_control/basic_robot_hw.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_control_sim");
    ros::NodeHandle nh;

    BasicRobotHW hw;
    controller_manager::ControllerManager cm(&hw);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
        hw.read(ros::Time::now(), ros::Duration(1));
        cm.update(ros::Time::now(), ros::Duration(1));
        hw.write(ros::Time::now(), ros::Duration(1));
        ros::Duration(1).sleep();
    }

    spinner.stop();
}