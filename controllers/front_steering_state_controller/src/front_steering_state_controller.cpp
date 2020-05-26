#include "front_steering_state_controller/front_steering_state_controller.h"

using namespace front_steering_state_controller;

FrontSteeringStateController::FrontSteeringStateController() : kinematics_(Kinematics(State())) {}

bool FrontSteeringStateController::init(hardware_interface::FrontSteeringStateInterface* hw, ros::NodeHandle& nh)
{
    RobotDimensions robot_dimensions;
    robot_dimensions.length = 0.5;  // in meters
    robot_dimensions.width = 0.3;  // in meters
    kinematics_.setRobotDimensions(robot_dimensions);

    const std::vector<std::string>& joint_names = hw->getNames();
    
    for(int i = 0; i < joint_names.size(); i++)
    {
        front_steering_state_handle_.push_back(hw->getHandle(joint_names[i]));
        ROS_INFO_STREAM("[Front Steering State Controller]: " << joint_names[i].c_str() << " joint created");
    }

    heading_feedback_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<ackermann_msgs::AckermannDriveStamped>>(nh, "/base/heading_feedback", true);
    
    return true;
}

void FrontSteeringStateController::update(const ros::Time& time, const ros::Duration& period)
{
    std::vector<double> steering_angles;
    ackermann_msgs::AckermannDriveStamped msg;

    for(unsigned int i = 0; i < front_steering_state_handle_.size(); i++)
    {
        steering_angles.push_back(front_steering_state_handle_[i].getWheelAngle());
    }

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "front_steering_state_controller";
    msg.drive.steering_angle = kinematics_.computeHeading(steering_angles);

    publishHeadingFeedback(msg);
}

void FrontSteeringStateController::publishHeadingFeedback(const ackermann_msgs::AckermannDriveStamped& msg) const
{
    if(heading_feedback_publisher_->trylock())
    {
        heading_feedback_publisher_->msg_ = msg;
        heading_feedback_publisher_->unlockAndPublish();
    }
}

PLUGINLIB_EXPORT_CLASS(front_steering_state_controller::FrontSteeringStateController, controller_interface::ControllerBase);