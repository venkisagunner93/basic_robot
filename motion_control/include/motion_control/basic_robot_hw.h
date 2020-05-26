/**
 * @file basic_robot_hw.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's hardware
 * @version 0.1
 * @date 2020-05-24
 * @copyright Copyright (c) 2020
 */

#ifndef BASIC_ROBOT_HW_H
#define BASIC_ROBOT_HW_H

#include "basic_robot/basic_robot.h"
#include "motion_control/basic_robot_base.h"

/**
 * @brief A basic robot's hardware class
 */
class BasicRobotHW : public BasicRobotBase
{
    public:
        /**
         * @brief A method for reading from hardware
         * @param time - Current time
         * @param period - Max period allowed
         */
        void read(const ros::Time& time, const ros::Duration& period);
        /**
         * @brief A method for writing to hardware
         * @param time - Current time
         * @param period - Max period allowed
         */
        void write(const ros::Time& time, const ros::Duration& period);
};

#endif  // BASIC_ROBOT_HW_H