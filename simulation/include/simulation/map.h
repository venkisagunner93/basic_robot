/**
 * @file map.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class to load map from map server
 * @version 0.1
 * @date 2020-03-28
 * @copyright Copyright (c) 2020
 */

#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

/**
 * @brief A basic robot's map class
 */
class Map
{
    public:
        /**
         * @brief A constructor for Map class
         * @param nh - Nodehandle for ROS communication
         */
        Map(ros::NodeHandle &nh);
        /**
         * @brief A destructor for Map class
         */
        virtual ~Map(){}
        /**
         * @brief A method to print map from map server
         */
        void printMap();
    
    private:
        /**
         * @brief ROS Node handle
         */
        ros::NodeHandle nh_;
        /**
         * @brief ROS Subscriber instance for map
         */
        ros::Subscriber map_server_subscriber_;
        /**
         * @brief Map instance to store map from map server
         */
        nav_msgs::OccupancyGrid map_;
        /**
         * @brief A subscriber callback to load map from map server
         * @param msg - A message that holds occupancy grid messages
         */
        void mapServerCallback(const nav_msgs::OccupancyGrid& msg);
};

#endif  // MAP_H