/**
 * @file front_steering_state_interface.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's front steering state
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef FRONT_STEERING_STATE_INTERFACE_H
#define FRONT_STEERING_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{
    /**
     * @brief A basic robot's front steering state handle class
     */
    class FrontSteeringStateHandle
    {
        public:
            /**
             * @brief Construct a new default Front Steering State Handle object
             */
            FrontSteeringStateHandle() = default;
            /**
             * @brief Construct a new Front Steering State Handle object
             * @param name - Name of front steering state handle
             * @param wheel_angle - Wheel angle of front steering state handle 
             */
            FrontSteeringStateHandle(const std::string& name, double* wheel_angle) : name_(name), wheel_angle_(wheel_angle)
            {
                if(!wheel_angle)
                {
                    throw hardware_interface::HardwareInterfaceException("Unable to create handle " + name + ". Wheel angle data pointer is null");
                }
            }
            /**
             * @brief Get the name of front steering state handle
             * @return std::string - Name of front steering state handle
             */
            std::string getName() const
            {
                return name_;
            }
            /**
             * @brief Get the wheel angle of front steering state handle
             * @return double - Wheel angle data
             */
            double getWheelAngle() const
            {
                assert(wheel_angle_);
                return *wheel_angle_;
            }
            /**
             * @brief Get the wheel angle Ptr of front steering state handle
             * @return const double* - Position pointer
             */
            const double* getWheelAnglePtr() const
            {
                return wheel_angle_;
            }
        
        private:
            /**
             * @brief Front steering state handle name
             */
            std::string name_;
            /**
             * @brief Front steering state position
             */
            const double* wheel_angle_ = {nullptr};
    };

    /**
     * @brief A basic robot's front steering state interface class
     */
    class FrontSteeringStateInterface : public HardwareResourceManager<FrontSteeringStateHandle> {};
}

#endif  // FRONT_STEERING_STATE_INTERFACE_H