/**
 * @file rear_drive_state_controller.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's rear drive state controller
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef REAR_DRIVE_STATE_CONTROLLER_H
#define REAR_DRIVE_STATE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "basic_robot/basic_robot.h"
#include "rear_drive_state_interface/rear_drive_state_interface.h"

namespace rear_drive_state_controller
{
    /**
     * @brief A basic robot's rear drive state controller class
     */
    class RearDriveStateController : public controller_interface::Controller<hardware_interface::RearDriveStateInterface>
    {
        public:
            /**
             * @brief Construct a new Rear Drive State Controller object
             */
            RearDriveStateController();
            /**
             * @brief A method to initialize rear drive state controller plugin
             * @param hw - Rear drive state interface instance
             * @param nh - ROS NodeHandle for communications
             * @return true 
             * @return false 
             */
            bool init(hardware_interface::RearDriveStateInterface* hw, ros::NodeHandle &nh);
            /**
             * @brief A method to update data sent and received from actual hardware
             * @param time - Current time
             * @param period - Maximum period allowed
             */
            void update(const ros::Time& time, const ros::Duration& period);
            /**
             * @brief A method to run while starting rear drive controller
             * @param time - Current time or start time
             */
            void starting(const ros::Time& time) {}
            /**
             * @brief A method to run for stopping rear drive controller
             * @param time - Current time or stop time
             */
            void stopping(const ros::Time& time) {}
        
        private:
            /**
             * @brief Vector of rear drive HW interface state handle
             */
            std::vector<hardware_interface::RearDriveStateHandle> rear_drive_state_handle_;
            /**
             * @brief Basic robot's kinematics instance
             */
            Kinematics kinematics_;
            /**
             * @brief Real time publisher for velocity feedback
             */
            std::shared_ptr<realtime_tools::RealtimePublisher<ackermann_msgs::AckermannDriveStamped>> vel_feedback_publisher_;
            /**
             * @brief A method to publish velocity feedback from rear drive
             * @param msg - Feedback message from rear drive
             */
            void publishVelFeedback(const ackermann_msgs::AckermannDriveStamped& msg) const;
    };
}

#endif  // REAR_DRIVE_STATE_CONTROLLER_H