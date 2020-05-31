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

/**
 * @brief A struct to represent robot front wheel hub angles
 */
struct FrontWheel
{
    float right_hub_angle; /**< Right hub angle */
    float left_hub_angle; /**< Left hub angle */
};

/**
 * @brief A struct to represent robot rear wheel velocities
 */
struct RearWheel
{
    float right_velocity; /**< Right wheel velocity */
    float left_velocity; /**< Left wheel velocity */
};

/**
 * @brief A struct to represent basic robot dimensions
 */
struct RobotDimensions
{
    float length; /**< Length of basic robot */
    float width; /**< Width of basic robot */
};

/**
 * @brief A struct to represent basic robot limits
 */
struct Limits
{
    double max_velocity; /**< Maximum velocity of basic robot */
    double max_steering_angle; /**< Maximum steering angle of basic robot */
};

#endif  // BASIC_ROBOT_H