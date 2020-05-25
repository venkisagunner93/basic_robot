#include "motion_control/motion_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_control_sim");
    ros::NodeHandle nh;  

    std::vector<BasicRobotSim*> sim;
    RearDriveSim rear_drive_sim;
    FrontSteeringSim front_steering_sim;
    
    sim.push_back(&rear_drive_sim);
    sim.push_back(&front_steering_sim);

    MotionControl motion_control(sim);

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