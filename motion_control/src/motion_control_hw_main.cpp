#include "motion_control/motion_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_control_hw");
    ros::NodeHandle nh;  

    std::vector<BasicRobotHW*> hardware;
    RearDriveHW rear_drive_hw;
    FrontSteeringHW front_steering_hw;
    
    hardware.push_back(&rear_drive_hw);
    hardware.push_back(&front_steering_hw);

    MotionControl motion_control(hardware);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        motion_control.read();
        motion_control.updateControllers();
        motion_control.write();
        ros::Duration(motion_control.getLoopRate()).sleep();
    }

    spinner.stop();
}