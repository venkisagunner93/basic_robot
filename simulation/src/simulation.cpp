#include "simulation/simulation.h"

Simulation::Simulation(ros::NodeHandle& nh) : nh_(nh)
{
    loadParameters();

    State initial_state;
    initial_state.x = 0;
    initial_state.y = 0;
    initial_state.theta = 0;
    kinematics_ = std::make_shared<Kinematics>(initial_state);

    initializeTransports();
}

Simulation::~Simulation()
{
    stopSim();
}

void Simulation::initializeTransports()
{
    
    vel_feedback_publisher_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("base/vel_feedback", 100, true);
}

void Simulation::loadParameters()
{
    if(nh_.hasParam("sim/update_rate"))
    {
        nh_.getParam("sim/update_rate", update_rate_);
    }
    else
    {
        update_rate_ = 20;
    }
    if(nh_.hasParam("sim/noise/mean"))
    {
        nh_.getParam("sim/noise/mean", noise_parameters_.mean);
    }
    else
    {
        noise_parameters_.mean = 0.0;
    }
    if(nh_.hasParam("sim/noise/std_dev"))
    {
        nh_.getParam("sim/noise/std_dev", noise_parameters_.std_dev);
    }
    else
    {
        noise_parameters_.std_dev = 0.01;
    }
}

void Simulation::startSim()
{
    sim_thread_ = std::make_shared<boost::thread>(boost::bind(&Simulation::simThread, this));
    is_sim_running_ = true;
}

void Simulation::stopSim()
{
    is_sim_running_ = false;
}

void Simulation::simThread()
{
    while(is_sim_running_)
    {
        sendActualPose();
        sendNoisyPose();
        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000 / update_rate_));
    }
}

int Simulation::getUpdateRate() const
{
    return update_rate_;
}

void Simulation::sendActualPose()
{
    State new_state = kinematics_->computeStates(control_input_);

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

void Simulation::sendNoisyPose()
{
    std::default_random_engine generator(boost::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> distribution(noise_parameters_.mean, noise_parameters_.std_dev);

    ackermann_msgs::AckermannDriveStamped noisy_feedback = control_input_;

    noisy_feedback.header.stamp = ros::Time::now();
    noisy_feedback.header.frame_id = "base_link_estimate";
    noisy_feedback.drive.speed = noisy_feedback.drive.speed + distribution(generator);
    noisy_feedback.drive.steering_angle = noisy_feedback.drive.steering_angle + distribution(generator);

    State noisy_state = kinematics_->computeStates(noisy_feedback);
    noisy_state.x = noisy_state.x + distribution(generator);
    noisy_state.y = noisy_state.y + distribution(generator);
    noisy_state.theta = noisy_state.theta + distribution(generator);

    vel_feedback_publisher_.publish(noisy_feedback);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link_estimate";
    transformStamped.transform.translation.x = noisy_state.x;
    transformStamped.transform.translation.y = noisy_state.y;
    transformStamped.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, noisy_state.theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    broadcaster_.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_robot_sim");
    ros::NodeHandle nh;
    Simulation sim(nh);
    sim.startSim();
    while(ros::ok())
    {
        ros::Rate(sim.getUpdateRate()).sleep();
        ros::spinOnce();
    }
    sim.stopSim();
    return 0;
}