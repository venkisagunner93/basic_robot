/**
 * @file basic_robot_types.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A file for basic robot's types
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef BASIC_ROBOT_TYPES_H
#define BASIC_ROBOT_TYPES_H

/**
 * @brief A struct to represent the robot state
 */
struct State
{
    float x; /**< X coordinate in space */
    float y; /**< Y coordinate in space */
    float theta; /**< Orientation of robot around Z axis (yaw) */
    /**
     * @brief Construct a new State object
     * @param p_x - X coordinate in space
     * @param p_y - Y coordinate in space
     * @param p_theta - Orientation of robot around Z axis (yaw)
     */
    State(float p_x = 0, float p_y = 0, float p_theta = 0) : x(p_x), y(p_y), theta(p_theta) {}
};

/**
 * @brief A struct to represent noise parameters
 */
struct NoiseParameters
{
    float mean; /**< Mean of the noise */
    float std_dev; /**< Standard deviation of the noise */
};

#endif  // BASIC_ROBOT_H