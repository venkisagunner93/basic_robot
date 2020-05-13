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

#include "basic_robot/basic_robot.h"

/**
 * @brief A basic robot's simulation class
 */
class Simulation
{
    public:
        /**
         * @brief A constructor for basic robot's simulation
         * @param nh - Nodehandle for ROS communication
         */
        Simulation(ros::NodeHandle& nh);
        /**
         * @brief A destructor for basic robot's simulation
         */
        virtual ~Simulation();
        /**
         * @brief A method to start simulation
         */
        void startSim();
        /**
         * @brief A method to stop simulation
         */
        void stopSim();
        /**
         * @brief A method to get update rate
         * @return int - Update rate
         */
        int getUpdateRate() const;

    private:
        /**
         * @brief ROS Nodehandle instance
         */
        ros::NodeHandle nh_;
        /**
         * @brief Update rate variable for simulation
         */
        int update_rate_;
        /**
         * @brief Noise parameters to simulate gaussian noise
         */
        NoiseParameters noise_parameters_;
        /**
         * @brief A transform tree broadcaster
         */
        tf2_ros::TransformBroadcaster broadcaster_;
        /**
         * @brief Instance for kinematics class
         */
        std::shared_ptr<Kinematics> kinematics_;
        /**
         * @brief A ROS subscriber instance for base/cmd_vel topic
         */
        ros::Subscriber cmd_vel_subscriber_;
        /**
         * @brief A ROS publisher instance for base/vel_feedback topic
         */
        ros::Publisher vel_feedback_publisher_;
        /**
         * @brief Control Input for the system
         */
        ackermann_msgs::AckermannDriveStamped control_input_;
        /**
         * @brief A thread to run simulator continuously
         */
        std::shared_ptr<boost::thread> sim_thread_;
        /**
         * @brief A flag to turn ON/OFF sim
         */
        bool is_sim_running_;
        /**
         * @brief A method to initialize publishers and subscribers
         */
        void initializeTransports();
        /**
         * @brief A method to load parameters from Parameter server
         */
        void loadParameters();
        /**
         * @brief A ROS subscriber callback for base/cmd_vel topic
         * @param control_input - Control input to the drives
         */
        void cmdVelCallback(const ackermann_msgs::AckermannDriveStamped& control_input);
        /**
         * @brief A thread for running simulator
         */
        void simThread();
        /**
         * @brief A method to send actual robot pose
         */
        void sendActualPose();
        /**
         * @brief A method to send noisy robot pose
         */
        void sendNoisyPose();
};

#endif  // SIMULATION_H