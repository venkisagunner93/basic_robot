#include "motor_driver/motor_driver_hw_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh;

    MotorDriverHwInterface motor_driver_hw_interface;
    controller_manager::ControllerManager cm(&motor_driver_hw_interface);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        motor_driver_hw_interface.read(ros::Time::now(), ros::Duration(1));
        cm.update(ros::Time::now(), ros::Duration(0.1));
        motor_driver_hw_interface.write(ros::Time::now(), ros::Duration(1));
        ros::Duration(motor_driver_hw_interface.getLoopRate()).sleep();
    }

    spinner.stop();
}