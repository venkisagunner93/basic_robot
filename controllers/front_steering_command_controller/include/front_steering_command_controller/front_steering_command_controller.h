/**
 * @file front_steering_command_controller.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for basic robot's front steering command controller
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef FRONT_STEERING_COMMAND_CONTROLLER_H
#define FRONT_STEERING_COMMAND_CONTROLLER_H

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "basic_robot/basic_robot.h"
#include "system/kinematics.h"
#include "front_steering_command_interface/front_steering_command_interface.h"

namespace front_steering_command_controller
{
    /**
     * @brief A basic robot's front steering command controller class
     */
    class FrontSteeringCommandController : public controller_interface::Controller<hardware_interface::FrontSteeringCommandInterface>
    {
        public:
            /**
             * @brief Construct a new Front Drive Command Controller object
             */
            FrontSteeringCommandController();
            /**
             * @brief A method to initialize front steering command controller plugin
             * @param hw - Front steering command interface instance
             * @param nh - ROS NodeHandle for communications
             * @return true 
             * @return false 
             */
            bool init(hardware_interface::FrontSteeringCommandInterface* hw, ros::NodeHandle &nh);
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
             * @brief Vector of front steering HW interface command handle
             */
            std::vector<hardware_interface::FrontSteeringCommandHandle> front_steering_command_handle_;
            /**
             * @brief Control Input for the system
             */
            ackermann_msgs::AckermannDriveStamped control_input_;
            /**
             * @brief Basic robot's kinematics instance
             */
            Kinematics kinematics_;
            /**
             * @brief ROS Subscriber for base/cmd_vel topic
             */
            ros::Subscriber cmd_vel_subscriber_;
            /**
             * @brief A callback method for cmd_vel subscriber
             * @param msg - Ackermann drive message
             */
            void cmdVelCallback(const ackermann_msgs::AckermannDriveStamped& msg);
    };
}

#endif  // FRONT_STEERING_COMMAND_CONTROLLER_H