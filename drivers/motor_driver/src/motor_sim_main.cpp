#include "motor_driver/motor_driver_sim_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh;

    MotorDriverSimInterface motor_driver_sim_interface;
    controller_manager::ControllerManager cm(&motor_driver_sim_interface);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(motor_driver_sim_interface.getLoopRate());

    while (ros::ok())
    {
        motor_driver_sim_interface.read(ros::Time::now(), ros::Duration(1));
        cm.update(ros::Time::now(), ros::Duration(0.1));
        motor_driver_sim_interface.write(ros::Time::now(), ros::Duration(1));
        ros::Duration(motor_driver_sim_interface.getLoopRate()).sleep();
    }

    spinner.stop();
}