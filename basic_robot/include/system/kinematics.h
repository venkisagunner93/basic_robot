/**
 * @file kinematics.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's kinematics
 * @version 0.1
 * @date 2020-03-28
 * @copyright Copyright (c) 2020
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>

#include "basic_robot/basic_robot_types.h"

/**
 * @brief A basic robot's kinematics class
 */
class Kinematics
{
    public:
        /**
         * @brief A constructor for Kinematics class
         * @param initial_state - Initial state of basic robot
         */
        Kinematics(const State& initial_state);
        /**
         * @brief A destructor for Kinematics class
         */
        virtual ~Kinematics() {}
        /**
         * @brief A method to set the robot dimensions
         * @param robot_dimensions - Dimensions of basic robot
         */
        void setRobotDimensions(const RobotDimensions& robot_dimensions);
        /**
         * @brief A method to get the robot dimensions
         * @return RobotDimensions - Dimensions of basic robot
         */
        RobotDimensions getRobotDimensions() const;
        /**
         * @brief A method to compute new states of the system
         * @param control_input - Control input provided to the system
         * @return State - State of the system computed using kinematics
         */
        State computeStates(ackermann_msgs::AckermannDriveStamped& control_input);
        /**
         * @brief A method to compute front wheel hub angles
         * @param control_input - Control input provided to the system
         * @return FrontWheel - Steering angles of the front wheels
         */
        FrontWheel computeFrontWheelHubAngles(const ackermann_msgs::AckermannDriveStamped& control_input);
        /**
         * @brief A method to compute rear wheel velocities
         * @param control_input - Control input provided to the system
         * @return RearWheel - Rear wheel differential velocities
         */
        RearWheel computeRearWheelVelocities(const ackermann_msgs::AckermannDriveStamped& control_input);

    private:
        /**
         * @brief Last recorded time for dt calculation
         */
        ros::Time last_time_;
        /**
         * @brief State of basic robot
         */
        State state_;
        /**
         * @brief Robot dimensions
         */
        RobotDimensions robot_dimensions_;
};

#endif  // KINEMATICS_H