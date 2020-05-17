/**
 * @file motor_driver_hw_interface.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's motor driver hardware interface
 * @version 0.1
 * @date 2020-05-14
 * @copyright Copyright (c) 2020
 */

#ifndef MOTOR_DRIVER_HW_INTERFACE_H
#define MOTOR_DRIVER_HW_INTERFACE_H

#include "motor_driver/motor_driver_interface.h"

/**
 * @brief A basic robot's motor driver HW interface class
 */
class MotorDriverHwInterface : public MotorDriverInterface
{
    public:
        /**
         * @brief A method for reading from the motors
         * @param time - Current time
         * @param period - Max period allowed
         */
        void read(const ros::Time& time, const ros::Duration& period) override;
        /**
         * @brief A method for writing to the motors
         * @param time - Current time
         * @param period - Max period allowed
         */
        void write(const ros::Time& time, const ros::Duration& period) override;
        /**
         * @brief Get the Loop Rate object
         * @return float - Loop rate
         */
        float getLoopRate() const;
};

#endif  // MOTOR_DRIVER_HW_INTERFACE_H