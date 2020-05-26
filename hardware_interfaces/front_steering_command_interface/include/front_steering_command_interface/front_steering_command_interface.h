/**
 * @file front_steering_command_interface.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's front steering command
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef FRONT_STEERING_COMMAND_INTERFACE_H
#define FRONT_STEERING_COMMAND_INTERFACE_H

#include "front_steering_state_interface/front_steering_state_interface.h"

namespace hardware_interface
{
    /**
     * @brief A basic robot's front steering command handle class
     */
    class FrontSteeringCommandHandle : public FrontSteeringStateHandle
    {
        public:
            /**
             * @brief Construct a new default Front Steering Command Handle object
             */
            FrontSteeringCommandHandle() = default;
            /**
             * @brief Construct a new Front Steering Command Handle object
             * @param front_steering_state_handle - Front steering state handle instance
             * @param wheel_angle - Wheel angle of front steering command handle 
             */
            FrontSteeringCommandHandle(const FrontSteeringStateHandle& front_steering_state_handle, double* wheel_angle) : 
                FrontSteeringStateHandle(front_steering_state_handle), wheel_angle_(wheel_angle)
            {
                if(!wheel_angle)
                {
                    throw hardware_interface::HardwareInterfaceException("Unable to create handle " + front_steering_state_handle.getName() + ". Wheel angle data pointer is null");
                }
            }
            /**
             * @brief Set the wheel_angle of front steering command handle
             * @param wheel_angle - Wheel angle data
             */
            void setWheelAngle(double wheel_angle)
            {
                assert(wheel_angle_);
                *wheel_angle_ = wheel_angle;
            }
        
        private:
            /**
             * @brief Front steering command position
             */
            double* wheel_angle_ = {nullptr};
    };

    /**
     * @brief A basic robot's front steering command interface class
     */
    class FrontSteeringCommandInterface : public HardwareResourceManager<FrontSteeringCommandHandle> {};
}

#endif  // FRONT_STEERING_COMMAND_INTERFACE_H