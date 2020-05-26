#include "motion_control/basic_robot_sim.h"

BasicRobotSim::BasicRobotSim() : kinematics_(Kinematics(State())) 
{
    RobotDimensions robot_dimensions;
    robot_dimensions.length = 0.5;  // in meters
    robot_dimensions.width = 0.3;  // in meters
    kinematics_.setRobotDimensions(robot_dimensions);
}

void BasicRobotSim::read(const ros::Time& time, const ros::Duration& period)
{
    broadcastNoisyPose();
}

void BasicRobotSim::write(const ros::Time& time, const ros::Duration& period) 
{
    broadcastActualPose();
}

void BasicRobotSim::broadcastActualPose()
{
    control_input_.drive.speed = kinematics_.computeVelocity(std::vector<double>({velocity_command_[0], velocity_command_[1]}));
    control_input_.drive.steering_angle = kinematics_.computeHeading(std::vector<double>({wheel_angle_command_[0], wheel_angle_command_[1]}));

    State new_state = kinematics_.computeStates(control_input_);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = new_state.x;
    transformStamped.transform.translation.y = new_state.y;
    transformStamped.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, new_state.theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    broadcaster_.sendTransform(transformStamped);
}

void BasicRobotSim::broadcastNoisyPose()
{
    std::default_random_engine generator(boost::chrono::system_clock::now().time_since_epoch().count());
    noise_parameters_.mean = 0.02;
    noise_parameters_.std_dev = 0.1;
    std::normal_distribution<double> distribution(noise_parameters_.mean, noise_parameters_.std_dev);

    RearWheel rear_wheel = kinematics_.computeRearWheelVelocities(control_input_);
    FrontWheel front_wheel = kinematics_.computeFrontWheelHubAngles(control_input_);

    velocity_feedback_[0] = rear_wheel.left_velocity + distribution(generator);
    velocity_feedback_[1] = rear_wheel.right_velocity + distribution(generator);

    wheel_angle_feedback_[0] = front_wheel.left_hub_angle + distribution(generator);
    wheel_angle_feedback_[1] = front_wheel.right_hub_angle + distribution(generator);
}