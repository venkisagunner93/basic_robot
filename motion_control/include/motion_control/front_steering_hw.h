/**
 * @file front_steering_hw.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's front steering hardware
 * @version 0.1
 * @date 2020-05-24
 * @copyright Copyright (c) 2020
 */

#ifndef FRONT_STEERING_HW_H
#define FRONT_STEERING_HW_H

#include "motion_control/basic_robot_hw.h"

/**
 * @brief A basic robot's front steering hardware class
 */
class FrontSteeringHW : public BasicRobotHW
{
    public:
        /**
         * @brief A method for reading from front steering hardware
         * @param time - Current time
         * @param period - Max period allowed
         */
        void read(const ros::Time& time, const ros::Duration& period);
        /**
         * @brief A method for writing to front steering hardware
         * @param time - Current time
         * @param period - Max period allowed
         */
        void write(const ros::Time& time, const ros::Duration& period);
};

#endif  // FRONT_STEERING_HW_H