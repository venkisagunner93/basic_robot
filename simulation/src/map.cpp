#include "simulation/map.h"

Map::Map(ros::NodeHandle& nh) : nh_(nh)
{
    map_server_subscriber_ = nh_.subscribe("/basic_map", 100, &Map::mapServerCallback, this);
}

void Map::mapServerCallback(const nav_msgs::OccupancyGrid& msg)
{
    map_ = msg;
}

void Map::printMap()
{
    ROS_INFO_STREAM(map_);
}

