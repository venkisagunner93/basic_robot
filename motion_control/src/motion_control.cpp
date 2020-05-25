#include "motion_control/motion_control.h"

MotionControl::MotionControl(const std::vector<BasicRobotHW*>& hardware) : hardware_(hardware)
{
    BasicRobotHW basic_robot_hw;
    controller_manager_ = std::make_shared<controller_manager::ControllerManager>(&basic_robot_hw);
}

void MotionControl::readFromHW() const
{
    for(int i = 0; i < hardware_.size(); i++)
    {
        hardware_[i]->read(ros::Time::now(), ros::Duration(1));
    }
}

void MotionControl::updateControllers() const
{
    controller_manager_->update(ros::Time::now(), ros::Duration(0.1));
}

void MotionControl::writeToHW() const
{
    for(int i = 0; i < hardware_.size(); i++)
    {
        hardware_[i]->write(ros::Time::now(), ros::Duration(1));
    }
}

double MotionControl::getLoopRate() const
{
    return loop_rate_;
}