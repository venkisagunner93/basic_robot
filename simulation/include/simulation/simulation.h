/**
 * @file simulation.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's simulation
 * @version 0.1
 * @date 2020-03-29
 * @copyright Copyright (c) 2020
 */

#ifndef SIMULATION_H
#define SIMULATION_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <boost/thread.hpp>
#include <random>
#include "simulation/kinematics.h"

struct NoiseParameters
{
    float mean;
    float std_dev;
};

class Simulation
{
    public:
        /**
         * @brief Construct a new Simulation object
         * A constructor for basic robot's simulation
         * @param nh 
         */
        Simulation(ros::NodeHandle& nh);
        /**
         * @brief Destroy the Simulation object
         * A destructor for basic robot's simulation
         */
        virtual ~Simulation();
        /**
         * @brief Start sim
         * A method to start simulation
         */
        void startSim();
        /**
         * @brief Stop sim
         * A method to stop simulation
         */
        void stopSim();
        /**
         * @brief Get update rate
         * A method to get update rate
         * @return int 
         */
        int getUpdateRate() const;

    private:
        /**
         * @brief nh
         * ROS Nodehandle instance
         */
        ros::NodeHandle nh_;
        /**
         * @brief Update rate
         * Update rate variable for simulation
         */
        int update_rate_;
        /**
         * @brief Noise parameters
         * Noise parameters to simulate gaussian noise
         */
        NoiseParameters noise_parameters_;
        /**
         * @brief Broadcaster
         * A transform tree broadcaster
         */
        tf2_ros::TransformBroadcaster broadcaster_;
        /**
         * @brief Kinematics
         * Instance for kinematics class
         */
        std::shared_ptr<Kinematics> kinematics_;
        /**
         * @brief Cmd Vel Subscriber
         * A ROS subscriber instance for base/cmd_vel topic
         */
        ros::Subscriber cmd_vel_subscriber_;
        /**
         * @brief Vel feedback publisher
         * A ROS publisher instance for base/vel_feedback topic
         */
        ros::Publisher vel_feedback_publisher_;
        /**
         * @brief Control input
         * Control Input for the system
         */
        ackermann_msgs::AckermannDriveStamped control_input_;
        /**
         * @brief Sim thread
         * A thread to run simulator continuously
         */
        std::shared_ptr<boost::thread> sim_thread_;
        /**
         * @brief Is sim running
         * A flag to turn ON/OFF sim
         */
        bool is_sim_running_;
        /**
         * @brief Initialize Transports
         * A method to initialize publishers and subscribers
         */
        void initializeTransports();
        /**
         * @brief Load parameters
         * A method to load parameters from Parameter server
         */
        void loadParameters();
        /**
         * @brief Cmd Vel Callback
         * A ROS subscriber callback for base/cmd_vel topic
         * @param control_input 
         */
        void cmdVelCallback(const ackermann_msgs::AckermannDriveStamped& control_input);
        /**
         * @brief Sim thread
         * A thread for running simulator
         */
        void simThread();
        /**
         * @brief Send Actual Pose
         * A method to send actual robot pose
         */
        void sendActualPose();
        /**
         * @brief Send Noisy Pose
         * A method to send noisy robot pose
         */
        void sendNoisyPose();
};

#endif  // SIMULATION_H