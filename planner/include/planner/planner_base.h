/**
 * @file planner_base.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's planning
 * @version 0.1
 * @date 2020-05-30
 * @copyright Copyright (c) 2020
 */

#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include <pluginlib/class_list_macros.h>
#include "basic_robot/basic_robot.h"

namespace planner_interface
{
    /**
     * @brief A basic robot's planner base class
     */
    class PlannerBase
    {
        public:
            /**
             * @brief Destroy the Planner Base instance
             */
            virtual ~PlannerBase(){};
            /**
             * @brief A virtual function for initializing plugins
             * @param nh - ROS NodeHandle for communication
             * @return true
             * @return false
             */
            virtual bool initialize(ros::NodeHandle &nh) = 0;
            /**
             * @brief A virtual function to send set points
             */
            virtual void sendSetpoints() = 0;
            /**
             * @brief A virtual function to set the limits for the system
             * @param limits - Limits for basic robot
             */
            virtual void setLimits(const Limits& limits) = 0;
    };
}

#endif  // PLANNER_BASE_H