/**
 * @file motion_control.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's motion control
 * @version 0.1
 * @date 2020-05-23
 * @copyright Copyright (c) 2020
 */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <controller_manager/controller_manager.h>

#include "motion_control/rear_drive_hw.h"
#include "motion_control/front_steering_hw.h"

/**
 * @brief A basic robot's motion control class
 */
class MotionControl
{
    public:
        /**
         * @brief Construct a new Motion Control instance
         * @param drivers - A vector of drivers used for motion control
         */
        explicit MotionControl(const std::vector<BasicRobotHW*>& drivers);
        /**
         * @brief A method to read from all hardware
         */
        void readFromHW() const;
        /**
         * @brief A method to write to all hardware
         */
        void writeToHW() const;
        /**
         * @brief A method to update all hardware controllers
         */
        void updateControllers() const;
        /**
         * @brief Get the Loop Rate
         * @return double 
         */
        double getLoopRate() const;
    
    private:
        /**
         * @brief Motion control loop rate
         */
        double loop_rate_;
        /**
         * @brief A vector of pointers to hardware
         */
        std::vector<BasicRobotHW*> hardware_;
        /**
         * @brief Controller manager to update all controllers
         */
        std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

#endif  // MOTION_CONTROL_H