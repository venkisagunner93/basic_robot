/**
 * @file rear_drive_hw.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's rear drive hardware
 * @version 0.1
 * @date 2020-05-24
 * @copyright Copyright (c) 2020
 */

#ifndef REAR_DRIVE_HW_H
#define REAR_DRIVE_HW_H

#include "motion_control/hw/basic_robot_hw.h"

/**
 * @brief A basic robot's rear drive hardware class
 */
class RearDriveHW : public BasicRobotHW
{
    public:
        /**
         * @brief A method for reading from rear drive hardware
         * @param time - Current time
         * @param period - Max period allowed
         */
        void read(const ros::Time& time, const ros::Duration& period);
        /**
         * @brief A method for writing to rear drive hardware
         * @param time - Current time
         * @param period - Max period allowed
         */
        void write(const ros::Time& time, const ros::Duration& period);
};

#endif  // REAR_DRIVE_HW_H