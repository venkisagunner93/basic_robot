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

class Map
{
    public:
        /**
         * @brief Construct a new Map object
         * A constructor for Map class
         * @param nh 
         */
        Map(ros::NodeHandle &nh);
        /**
         * @brief Destroy the Map object
         * A destructor for Map class
         */
        virtual ~Map(){}
        /**
         * @brief Print Map
         * A method to print map from map server
         */
        void printMap();
    
    private:
        /**
         * @brief nh
         * ROS Node handle
         */
        ros::NodeHandle nh_;
        /**
         * @brief Map server subscriber
         * ROS Subscriber instance for map
         */
        ros::Subscriber map_server_subscriber_;
        /**
         * @brief Map
         * Map instance to store map from map server
         */
        nav_msgs::OccupancyGrid map_;
        /**
         * @brief Map server callback
         * A subscriber callback to load map from map server
         * @param msg 
         */
        void mapServerCallback(const nav_msgs::OccupancyGrid& msg);
};

#endif  // MAP_H