/**
 * @file planner.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A Planner class responsible for basic robot's navigation
 * @version 0.1
 * @date 2020-03-31
 * @copyright Copyright (c) 2020
 */

#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class Planner
{
    public:
        /**
         * @brief Construct a new Planner object
         * A constructor for planner class
         * @param nh 
         */
        Planner(ros::NodeHandle& nh);
        /**
         * @brief Destroy the Planner object
         * A destructor for planner class
         */
        virtual ~Planner() {}
        /**
         * @brief Get update rate
         * A method to get update rate
         * @return int 
         */
        int getUpdateRate() const;
        /**
         * @brief Publish set points
         * A method to publish set points
         * @param setpoint 
         */
        void publishSetpoints(const ackermann_msgs::AckermannDriveStamped& setpoint);
    
    private:
        /**
         * @brief nh
         * ROS Nodehandle instance
         */
        ros::NodeHandle nh_;
        /**
         * @brief Update rate
         * Update rate variable for planner
         */
        int update_rate_;
        /**
         * @brief Cmd Vel publisher
         * ROS publisher instance for base/cmd_vel topic
         */
        ros::Publisher cmd_vel_publisher_;
        /**
         * @brief Load parameters
         * A method to load parameters from Parameter server
         */
        void loadParameters();
        /**
         * @brief Initialize Transports
         * A method to initialize publishers and subscribers
         */
        void initializeTransports();
};

#endif  // PLANNER_H