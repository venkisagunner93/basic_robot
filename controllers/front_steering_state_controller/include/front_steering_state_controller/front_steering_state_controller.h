/**
 * @file front_steering_state_controller.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's front steering state controller
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef FRONT_STEERING_STATE_CONTROLLER_H
#define FRONT_STEERING_STATE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "basic_robot/basic_robot.h"
#include "system/kinematics.h"
#include "front_steering_state_interface/front_steering_state_interface.h"

namespace front_steering_state_controller
{
    /**
     * @brief A basic robot's front steering state controller class
     */
    class FrontSteeringStateController : public controller_interface::Controller<hardware_interface::FrontSteeringStateInterface>
    {
        public:
            /**
             * @brief Construct a new Front Steering State Controller object
             */
            FrontSteeringStateController();
            /**
             * @brief A method to initialize front steering state controller plugin
             * @param hw - Front steering state interface instance
             * @param nh - ROS NodeHandle for communications
             * @return true 
             * @return false 
             */
            bool init(hardware_interface::FrontSteeringStateInterface* hw, ros::NodeHandle &nh);
            /**
             * @brief A method to update data sent and received from actual hardware
             * @param time - Current time
             * @param period - Maximum period allowed
             */
            void update(const ros::Time& time, const ros::Duration& period);
            /**
             * @brief A method to run while starting front steering controller
             * @param time - Current time or start time
             */
            void starting(const ros::Time& time) {}
            /**
             * @brief A method to run for stopping front steering controller
             * @param time - Current time or stop time
             */
            void stopping(const ros::Time& time) {}
        
        private:
            /**
             * @brief Vector of front steering HW interface state handle
             */
            std::vector<hardware_interface::FrontSteeringStateHandle> front_steering_state_handle_;
            /**
             * @brief Basic robot's kinematics instance
             */
            Kinematics kinematics_;
            /**
             * @brief Real time publisher for heading feedback
             */
            std::shared_ptr<realtime_tools::RealtimePublisher<ackermann_msgs::AckermannDriveStamped>> heading_feedback_publisher_;
            /**
             * @brief A method to publish heading feedback from front steering
             * @param msg - Feedback message from rear drive
             */
            void publishHeadingFeedback(const ackermann_msgs::AckermannDriveStamped& msg) const;
    };
}

#endif  // FRONT_STEERING_STATE_CONTROLLER_H