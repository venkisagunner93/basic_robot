/**
 * @file rear_drive_command_interface.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's rear drive command
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef REAR_DRIVE_COMMAND_INTERFACE_H
#define REAR_DRIVE_COMMAND_INTERFACE_H

#include "rear_drive_state_interface/rear_drive_state_interface.h"

namespace hardware_interface
{
    /**
     * @brief A basic robot's rear drive command handle class
     */
    class RearDriveCommandHandle : public RearDriveStateHandle
    {
        public:
            /**
             * @brief Construct a new default Rear Drive Command Handle object
             */
            RearDriveCommandHandle() = default;
            /**
             * @brief Construct a new Rear Drive Command Handle object
             * @param rear_drive_state_handle - Rear drive state handle instance
             * @param position - Position of rear drive command handle 
             * @param velocity - Velocity of rear drive command handle 
             */
            RearDriveCommandHandle(const RearDriveStateHandle& rear_drive_state_handle, double* position, double* velocity) : 
                RearDriveStateHandle(rear_drive_state_handle), position_(position), velocity_(velocity)
            {
                if(!position)
                {
                    throw hardware_interface::HardwareInterfaceException("Unable to create handle " + rear_drive_state_handle.getName() + ". Position data pointer is null");
                }

                if(!velocity)
                {
                    throw hardware_interface::HardwareInterfaceException("Unable to create handle " + rear_drive_state_handle.getName() + ". Velocity data pointer is null");
                }
            }
            /**
             * @brief Set the position of rear drive command handle
             * @param position - Position data
             */
            void setPosition(double position)
            {
                assert(position_);
                *position_ = position;
            }
            /**
             * @brief Set the velocity of rear drive command handle
             * @param velocity - Velocity data
             */
            void setVelocity(double velocity)
            {
                assert(velocity_);
                *velocity_ = velocity;
            }
        
        private:
            /**
             * @brief Rear drive command position
             */
            double* position_ = {nullptr};
            /**
             * @brief Rear drive command velocity
             */
            double* velocity_ = {nullptr};
    };

    /**
     * @brief A basic robot's rear drive command interface class
     */
    class RearDriveCommandInterface : public HardwareResourceManager<RearDriveCommandHandle> {};
}

#endif  // REAR_DRIVE_COMMAND_INTERFACE_H