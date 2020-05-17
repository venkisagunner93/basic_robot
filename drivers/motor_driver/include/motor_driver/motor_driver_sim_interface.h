/**
 * @file motor_driver_sim_interface.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's motor driver simulation interface
 * @version 0.1
 * @date 2020-05-16
 * @copyright Copyright (c) 2020
 */

#ifndef MOTOR_DRIVER_SIM_INTERFACE_H
#define MOTOR_DRIVER_SIM_INTERFACE_H

#include "motor_driver/motor_driver_interface.h"

/**
 * @brief A basic robot's motor driver Sim interface class
 */
class MotorDriverSimInterface : public MotorDriverInterface
{
    public:
        /**
         * @brief A method for reading from the motors sim
         * @param time - Current time
         * @param period - Max period allowed
         */
        void read(const ros::Time& time, const ros::Duration& period);
        /**
         * @brief A method for writing to the motors sim
         * @param time - Current time
         * @param period - Max period allowed
         */
        void write(const ros::Time& time, const ros::Duration& period);
        /**
         * @brief Get the Loop Rate object
         * @return float - Loop rate
         */
        float getLoopRate() const;
};

#endif  // MOTOR_DRIVER_SIM_INTERFACE_H