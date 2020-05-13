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
         * @param initial_state - Initial state of the basic robot
         */
        Kinematics(const State& initial_state);
        /**
         * @brief A destructor for Kinematics class
         */
        virtual ~Kinematics() {}
        /**
         * @brief A method to compute new states of the system
         * @param control_input - Control input provided to the system
         * @return State - State of the system computed using kinematics
         */
        State computeStates(ackermann_msgs::AckermannDriveStamped& control_input);

    private:
        /**
         * @brief Last recorded time for dt calculation
         */
        ros::Time last_time_;
        /**
         * @brief State of basic robot
         */
        State state_;
};

#endif  // KINEMATICS_H