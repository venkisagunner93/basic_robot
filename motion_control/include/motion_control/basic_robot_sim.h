/**
 * @file basic_robot_sim.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's simulation
 * @version 0.1
 * @date 2020-05-24
 * @copyright Copyright (c) 2020
 */

#ifndef BASIC_ROBOT_SIM_H
#define BASIC_ROBOT_SIM_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/chrono/chrono.hpp>
#include <random>

#include "basic_robot/basic_robot.h"
#include "motion_control/basic_robot_base.h"

/**
 * @brief A basic robot's simulation class
 */
class BasicRobotSim : public BasicRobotBase
{
    public:
        /**
         * @brief Construct a new basic robot sim instance
         */
        BasicRobotSim();
        /**
         * @brief A method for reading from simulation
         * @param time - Current time
         * @param period - Max period allowed
         */
        void read(const ros::Time& time, const ros::Duration& period);
        /**
         * @brief A method for writing to simulation
         * @param time - Current time
         * @param period - Max period allowed
         */
        void write(const ros::Time& time, const ros::Duration& period);

    private:
        /**
         * @brief Control input to basic robot
         */
        ackermann_msgs::AckermannDriveStamped control_input_;
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
        Kinematics kinematics_;
        /**
         * @brief A method to broadcast actual robot pose
         */
        void broadcastActualPose();
        /**
         * @brief A method to broadcast noisy robot pose
         */
        void broadcastNoisyPose();
};

#endif  // BASIC_ROBOT_SIM_H