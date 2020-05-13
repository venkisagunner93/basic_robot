/**
 * @file rear_drive_interface.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A basic robot's rear drive hw interface class
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef REAR_DRIVE_INTERFACE_H
#define REAR_DRIVE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{
    /**
     * @brief A basic robot's rear drive interface class
     */
    class RearDriveHandle
    {
        public:
            /**
             * @brief Construct a new default Rear Drive Handle object
             */
            RearDriveHandle() = default;
            /**
             * @brief Construct a new Rear Drive Handle object
             * @param name - Name of rear drive handle
             * @param position - Position of rear drive handle
             * @param velocity - Velocity of rear drive handle
             */
            RearDriveHandle(const std::string& name, const double* position, const double* velocity)
            {
                if(!position)
                {
                    throw hardware_interface::HardwareInterfaceException("Unable to create handle " + name + ". Position data pointer is null");
                }

                if(!velocity)
                {
                    throw hardware_interface::HardwareInterfaceException("Unable to create handle " + name + ". Velocity data pointer is null");
                }
            }
            /**
             * @brief Get the name of rear drive handle
             * @return std::string - Name of rear drive handle
             */
            std::string getName() const
            {
                return name_;
            }
            /**
             * @brief Get the position of rear drive handle
             * @return double - Position data
             */
            double getPosition() const
            {
                assert(position_);
                return *position_;
            }
            /**
             * @brief Get the velocity of rear drive handle
             * @return double - Velocity data
             */
            double getVelocity() const
            {
                assert(velocity_);
                return *velocity_;
            }
            /**
             * @brief Get the Position Ptr of rear drive handle
             * @return const double* - Position pointer
             */
            const double* getPositionPtr() const 
            {
                return position_;
            }
            /**
             * @brief Get the Velocity Ptr of rear drive handle
             * @return const double* - Velocity pointer
             */
            const double* getVelocityPtr() const
            {
                return velocity_;
            }
        
        private:
            /**
             * @brief Rear drive handle name
             */
            std::string name_;
            /**
             * @brief Rear drive position
             */
            const double* position_ = {nullptr};
            /**
             * @brief Rear drive velocity
             */
            const double* velocity_ = {nullptr};
    };

    /**
     * @brief A basic robot's rear drive interface class
     */
    class RearDriveInterface : public HardwareResourceManager<RearDriveHandle> {};
}


#endif  // REAR_DRIVE_INTERFACE_H