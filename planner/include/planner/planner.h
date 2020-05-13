/**
 * @file planner.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's navigation
 * @version 0.1
 * @date 2020-03-31
 * @copyright Copyright (c) 2020
 */

#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "basic_robot/basic_robot.h"

/**
 * @brief A basic robot's planner class
 */
class Planner
{
    public:
        /**
         * @brief A constructor for planner class
         * @param nh - ROS Nodehandle for communication
         */
        Planner(ros::NodeHandle& nh);
        /**
         * @brief A destructor for planner class
         */
        virtual ~Planner() {}
        /**
         * @brief A method to get update rate
         * @return int - Update rate
         */
        int getUpdateRate() const;
        /**
         * @brief A method to publish set points
         * @param setpoint - Setpoint to be published
         */
        void publishSetpoints(const ackermann_msgs::AckermannDriveStamped& setpoint);
    
    private:
        /**
         * @brief ROS Nodehandle instance
         */
        ros::NodeHandle nh_;
        /**
         * @brief Update rate variable for planner
         */
        int update_rate_;
        /**
         * @brief ROS publisher instance for base/cmd_vel topic
         */
        ros::Publisher cmd_vel_publisher_;
        /**
         * @brief A method to load parameters from Parameter server
         */
        void loadParameters();
        /**
         * @brief A method to initialize publishers and subscribers
         */
        void initializeTransports();
};

#endif  // PLANNER_H