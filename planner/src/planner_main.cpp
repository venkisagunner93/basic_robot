#include "planner/planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    Planner planner(nh);
    
    while(ros::ok())
    {
        planner.publishSetpoints();
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }

    return 0;
}