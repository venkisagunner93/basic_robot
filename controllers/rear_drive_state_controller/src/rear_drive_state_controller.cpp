#include "rear_drive_state_controller/rear_drive_state_controller.h"

using namespace rear_drive_state_controller;

bool RearDriveStateController::init(hardware_interface::RearDriveStateInterface* hw, ros::NodeHandle& nh)
{
    const std::vector<std::string>& joint_names = hw->getNames();
    
    for(int i = 0; i < joint_names.size(); i++)
    {
        rear_drive_state_handle_.push_back(hw->getHandle(joint_names[i]));
        ROS_INFO_STREAM("[Rear Drive State Controller]: " << joint_names[i].c_str() << " joint created");
    }

    vel_feedback_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<ackermann_msgs::AckermannDriveStamped>>(nh, "/base/vel_feedback", true);

    return true;
}

void RearDriveStateController::update(const ros::Time& time, const ros::Duration& period)
{
    ackermann_msgs::AckermannDriveStamped msg;

    for(unsigned int i = 0; i < rear_drive_state_handle_.size(); i++)
    {
        msg.drive.speed = rear_drive_state_handle_[i].getVelocity();
    }

    publishVelFeedback(msg);
}

void RearDriveStateController::publishVelFeedback(const ackermann_msgs::AckermannDriveStamped& msg) const
{
    if(vel_feedback_publisher_->trylock())
    {
        vel_feedback_publisher_->msg_ = msg;
        vel_feedback_publisher_->unlockAndPublish();
    }
}

PLUGINLIB_EXPORT_CLASS(rear_drive_state_controller::RearDriveStateController, controller_interface::ControllerBase);