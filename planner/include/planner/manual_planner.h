/**
 * @file manual_planner.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's manual planning
 * @version 0.1
 * @date 2020-05-30
 * @copyright Copyright (c) 2020
 */

#ifndef MANUAL_PLANNER_H
#define MANUAL_PLANNER_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>

#include "planner/planner_base.h"

namespace planner_plugins
{
    /**
     * @brief A basic robot's manual planner class for teleoperation
     */
    class ManualPlanner : public planner_interface::PlannerBase
    {
        public:
            /**
             * @brief A method to initialize plugin
             * @param nh - ROS NodeHandle for communication
             * @return true 
             * @return false 
             */
            bool initialize(ros::NodeHandle& nh);
            /**
             * @brief A method to publish setpoints
             */
            void sendSetpoints();
            /**
             * @brief A method to set limits for basic robot
             * @param limits - Limits for basic robot
             */
            void setLimits(const Limits& limits);
            
        private:
            /**
             * @brief ROS publisher instance for base/cmd_vel topic
             */
            ros::Publisher cmd_vel_publisher_;
            /**
             * @brief ROS subscriber instance for /joy topic
             */
            ros::Subscriber joystick_subscriber_;
            /**
             * @brief Setpoint
             */
            ackermann_msgs::AckermannDriveStamped setpoint_;
            /**
             * @brief 
             */
            Limits limits_;
            /**
             * @brief A callback method for joystick hardware
             * @param msg - Joy message from joystick hardware
             */
            void joystickCallback(const sensor_msgs::Joy& msg);
    };
}

#endif  // MANUAL_PLANNER_H
