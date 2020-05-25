/**
 * @file rear_drive_sim.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's rear drive simulation
 * @version 0.1
 * @date 2020-05-24
 * @copyright Copyright (c) 2020
 */

#ifndef REAR_DRIVE_SIM_H
#define REAR_DRIVE_SIM_H

#include "motion_control/sim/basic_robot_sim.h"

/**
 * @brief A basic robot's rear drive simulation class
 */
class RearDriveSim : public BasicRobotSim
{
    public:
        /**
         * @brief A method for reading from rear drive simulation
         * @param time - Current time
         * @param period - Max period allowed
         */
        void read(const ros::Time& time, const ros::Duration& period);
        /**
         * @brief A method for writing to rear drive simulation
         * @param time - Current time
         * @param period - Max period allowed
         */
        void write(const ros::Time& time, const ros::Duration& period);
};

#endif  // REAR_DRIVE_SIM_H