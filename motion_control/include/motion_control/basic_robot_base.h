/**
 * @file basic_robot_base.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's base
 * @version 0.1
 * @date 2020-05-25
 * @copyright Copyright (c) 2020
 */

#ifndef BASIC_ROBOT_BASE_H
#define BASIC_ROBOT_BASE_H

#include <hardware_interface/robot_hw.h>

#include "rear_drive_command_interface/rear_drive_command_interface.h"
#include "rear_drive_state_interface/rear_drive_state_interface.h"
#include "front_steering_command_interface/front_steering_command_interface.h"
#include "front_steering_state_interface/front_steering_state_interface.h"

/**
 * @brief A basic robot's base class for hardware and sim
 */
class BasicRobotBase : public hardware_interface::RobotHW
{
    public:
        /**
         * @brief Construct a new Basic Robot Base instance
         */
        explicit BasicRobotBase();
        /**
         * @brief A virtual function for reading from front steering hardware
         * @param time - Current time
         * @param period - Max period allowed
         */
        virtual void read(const ros::Time& time, const ros::Duration& period) override;
        /**
         * @brief A virtual function for writing to front steering hardware
         * @param time - Current time
         * @param period - Max period allowed
         */
        virtual void write(const ros::Time& time, const ros::Duration& period) override;
    
    protected:
        /**
         * @brief Position feedback array
         */
        double position_feedback_[2];
        /**
         * @brief Velocity feedback array
         */
        double velocity_feedback_[2];
        /**
         * @brief Position command array
         */
        double position_command_[2];
        /**
         * @brief Velocity command array
         */
        double velocity_command_[2];
        /**
         * @brief Wheel angle feedback array
         */
        double wheel_angle_feedback_[2];
        /**
         * @brief Wheel angle command array
         */
        double wheel_angle_command_[2];
    
    private:
        /**
         * @brief Rear drive command interface instance
         */
        hardware_interface::RearDriveCommandInterface rear_drive_command_;
        /**
         * @brief Rear drive state interface instance
         */
        hardware_interface::RearDriveStateInterface rear_drive_feedback_;
        /**
         * @brief Front steering command interface instance
         */
        hardware_interface::FrontSteeringCommandInterface front_steering_command_;
        /**
         * @brief Front steering state interface instance
         */
        hardware_interface::FrontSteeringStateInterface front_steering_feedback_;
};

#endif  // BASIC_ROBOT_BASE_H