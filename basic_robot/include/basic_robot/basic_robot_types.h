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
typedef struct
{
    float x; /**< X coordinate in space */
    float y; /**< Y coordinate in space */
    float theta; /**< Orientation of robot around Z axis (yaw) */
} State;

/**
 * @brief A struct to represent noise parameters
 */
typedef struct
{
    float mean; /**< Mean of the noise */
    float std_dev; /**< Standard deviation of the noise */
} NoiseParameters;

#endif  // BASIC_ROBOT_H