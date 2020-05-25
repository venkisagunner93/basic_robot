/**
 * @file front_steering_sim.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's front steering simulation
 * @version 0.1
 * @date 2020-05-24
 * @copyright Copyright (c) 2020
 */

#ifndef FRONT_STEERING_SIM_H
#define FRONT_STEERING_SIM_H

#include "motion_control/sim/basic_robot_sim.h"

/**
 * @brief A basic robot's front steering simulation class
 */
class FrontSteeringSim : public BasicRobotSim
{
    public:
        /**
         * @brief A method for reading from front steering simulation
         * @param time - Current time
         * @param period - Max period allowed
         */
        void read(const ros::Time& time, const ros::Duration& period);
        /**
         * @brief A method for writing to front steering simulation
         * @param time - Current time
         * @param period - Max period allowed
         */
        void write(const ros::Time& time, const ros::Duration& period);
};

#endif  // FRONT_STEERING_SIM_H