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

struct State
{
    float x;
    float y;
    float theta;
};

class Kinematics
{
    public:
        /**
         * @brief Construct a new Kinematics object
         * A constructor for Kinematics class
         * @param initial_state 
         */
        Kinematics(const State& initial_state);
        /**
         * @brief Destroy the Kinematics object
         * A destructor for Kinematics class
         */
        virtual ~Kinematics() {}
        /**
         * @brief Compute states
         * A method to compute new states of the system
         * @param control_input 
         * @return State 
         */
        State computeStates(ackermann_msgs::AckermannDriveStamped control_input);

    private:
        /**
         * @brief Last time
         * Last recorded time for dt calculation
         */
        ros::Time last_time_;
        /**
         * @brief State
         * States of the system
         */
        State state_;
};

#endif  // KINEMATICS_H