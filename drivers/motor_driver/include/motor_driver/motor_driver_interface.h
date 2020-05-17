/**
 * @file motor_driver_interface.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's motor driver interface
 * @version 0.1
 * @date 2020-05-16
 * @copyright Copyright (c) 2020
 */

#ifndef MOTOR_DRIVER_INTERFACE_H
#define MOTOR_DRIVER_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>

#include "rear_drive_command_interface/rear_drive_command_interface.h"
#include "rear_drive_state_interface/rear_drive_state_interface.h"

/**
 * @brief A basic robot's motor driver interface class
 */
class MotorDriverInterface : public hardware_interface::RobotHW
{
    public:
        /**
         * @brief Construct a new Motor Driver Interface object
         */
        MotorDriverInterface();
        /**
         * @brief A virtual function for reading from the motors
         * @param time - Current time
         * @param period - Max period allowed
         */
        virtual void read(const ros::Time& time, const ros::Duration& period) override;
        /**
         * @brief A virtual function for writing to the motors
         * @param time - Current time
         * @param period - Max period allowed
         */
        virtual void write(const ros::Time& time, const ros::Duration& period) override;
        /**
         * @brief Get the Loop Rate
         * @return float - Loop rate
         */
        virtual float getLoopRate() const {};

    protected:
        /**
         * @brief Position command array
         */
        double position_command_[2];
        /**
         * @brief Velocity command array
         */
        double velocity_command_[2];
        /**
         * @brief Position feedback array
         */
        double position_feedback_[2];
        /**
         * @brief Velocity feedback array
         */
        double velocity_feedback_[2];
        
    private:
        /**
         * @brief Rear drive command interface instance
         */
        hardware_interface::RearDriveCommandInterface rear_drive_command_;
        /**
         * @brief Rear drive state interface instance
         */
        hardware_interface::RearDriveStateInterface rear_drive_feedback_;
};

#endif  // MOTOR_DRIVER_INTERFACE_H