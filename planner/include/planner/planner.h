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
#include <pluginlib/class_loader.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "basic_robot/basic_robot.h"
#include "basic_robot/Mode.h"
#include "basic_robot/SetMode.h"

#include "planner/planner_base.h"

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
        explicit Planner(ros::NodeHandle& nh);
        /**
         * @brief A destructor for planner class
         */
        virtual ~Planner();
        /**
         * @brief A wrapper function around planner plugin publish function
         */
        void publishSetpoints();
    
    private:
        /**
         * @brief ROS Nodehandle instance
         */
        ros::NodeHandle nh_;
        /**
         * @brief Planner plugin
         */
        boost::shared_ptr<planner_interface::PlannerBase> planner_plugin_;
        /**
         * @brief Planner class loader
         */
        pluginlib::ClassLoader<planner_interface::PlannerBase> planner_class_loader_;
        /**
         * @brief Planner plugin type
         */
        std::string planner_plugin_type_;
        /**
         * @brief Basic robot's limits
         */
        Limits limits_;
        /**
         * @brief ROS service server instance for mode of operation
         */
        ros::ServiceServer mode_server_;
        /**
         * @brief A method to load parameters from Parameter server
         */
        void loadParameters();
        /**
         * @brief A method to initialize planner plugin
         */
        void initializePlannerPlugin();
        /**
         * @brief A service callback to hotswap planner plugin on the go
         * @param basic_robot::SetModeRequest 
         * @param basic_robot::SetModeResponse 
         * @return true 
         * @return false 
         */
        bool modeServiceCallback(basic_robot::SetModeRequest& request, basic_robot::SetModeResponse& response);
};

#endif  // PLANNER_H