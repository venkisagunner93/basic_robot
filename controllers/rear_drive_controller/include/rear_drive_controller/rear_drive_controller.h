/**
 * @file rear_drive_controller.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A basic robot's rear drive controller class
 * @version 0.1
 * @date 2020-05-10
 * @copyright Copyright (c) 2020
 */

#ifndef REAR_DRIVE_CONTROLLER_H
#define REAR_DRIVE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "basic_robot/basic_robot.h"
#include "rear_drive_interface/rear_drive_interface.h"

/**
 * @brief A class for basic robot's rear drive controller
 */
class RearDriveController : public controller_interface::Controller<hardware_interface::RearDriveInterface>
{
    public:
        /**
         * @brief A method to initialize rear drive controller plugin
         * @param hw - Rear drive HW interface instance
         * @param nh - ROS NodeHandle for communications
         * @return true 
         * @return false 
         */
        bool init(hardware_interface::RearDriveInterface* hw, ros::NodeHandle &nh);
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
         * @brief Left rear drive HW interface handle
         */
        hardware_interface::RearDriveHandle left_rear_drive_;
        /**
         * @brief Right rear drive HW interface handle
         */
        hardware_interface::RearDriveHandle right_rear_drive_;
        /**
         * @brief Control Input for the system
         */
        ackermann_msgs::AckermannDriveStamped control_input_;
        /**
         * @brief Real time publisher for base/vel_feedback topic
         */
        boost::shared_ptr<realtime_tools::RealtimePublisher<ackermann_msgs::AckermannDriveStamped>> vel_feedback_publisher_;
        /**
         * @brief ROS Subscriber for base/cmd_vel topic
         */
        ros::Subscriber cmd_vel_subscriber_;
        /**
         * @brief A callback method for cmd_vel subscriber
         * @param msg - Ackermann drive message
         */
        void cmdVelCallback(const ackermann_msgs::AckermannDriveStamped& msg);
        /**
         * @brief A method to publish velocity feedback
         */
        void publishVelFeedback() const;
        
};

#endif  // REAR_DRIVE_CONTROLLER_H