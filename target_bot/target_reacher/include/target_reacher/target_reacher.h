#ifndef __TARGET_REACHER__
#define __TARGET_REACHER__

/**
 * @file target_reacher.h
 * @author Shreejay Badshah (sbadshah@umd.edu)
 * @brief  ROS2 header file to move robot to two predefined targets using callbacks.
 * @version 0.1
 * @date 2022-12-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

/**
 * @brief This class defines moves robot to predefined target locations.
 * 
 */
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {
        m_reached_goal = false;
        m_reached_first_goal = false;
        m_bot_controller = bot_controller;
        m_aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        m_aruco_target_y = this->declare_parameter<double>("aruco_target.y");
        m_final_destination0_x = this->declare_parameter<double>("final_destination.aruco_0.x");
        m_final_destination0_y = this->declare_parameter<double>("final_destination.aruco_0.y");
        m_final_destination1_x = this->declare_parameter<double>("final_destination.aruco_1.x");
        m_final_destination1_y = this->declare_parameter<double>("final_destination.aruco_1.y");
        m_final_destination2_x = this->declare_parameter<double>("final_destination.aruco_2.x");
        m_final_destination2_y = this->declare_parameter<double>("final_destination.aruco_2.y");
        m_final_destination3_x = this->declare_parameter<double>("final_destination.aruco_3.x");
        m_final_destination3_y = this->declare_parameter<double>("final_destination.aruco_3.y");
        m_frame_id = this->declare_parameter<std::string>("final_destination.frame_id");
        
        m_bot_controller->set_goal(m_aruco_target_x, m_aruco_target_y);

        m_goal_subscriber = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 10, 
                                                std::bind(&TargetReacher::goal_callback, this, std::placeholders::_1));
        
        m_msg = geometry_msgs::msg::Twist();
        m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);
        
        m_aruco_subscriber = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, 
                                                std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));
        
        m_tf_final_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_final_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

        // Call on_timer function every second
        m_timer_listener = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / 1)), 
                                            std::bind(&TargetReacher::listener_callback, this));
    }

private:
    // attributes
    bool m_reached_goal;
    bool m_reached_first_goal;
    double m_aruco_target_x;
    double m_aruco_target_y;
    double m_final_destination0_x;
    double m_final_destination0_y;
    double m_final_destination1_x;
    double m_final_destination1_y;
    double m_final_destination2_x;
    double m_final_destination2_y;
    double m_final_destination3_x;
    double m_final_destination3_y;
    std::string m_frame_id;
    std::shared_ptr<BotController> m_bot_controller;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_goal_subscriber;
    geometry_msgs::msg::Twist m_msg;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr m_aruco_subscriber;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_final_broadcaster;
    rclcpp::TimerBase::SharedPtr m_timer_broadcaster{nullptr};
    rclcpp::TimerBase::SharedPtr m_timer_listener{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> m_tf_final_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber;
    bool first_goal_reached;
    
    /**
     * @brief Callback function for robot to see if it reached the target or not
     * 
     * @param msg 
     */
    void goal_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Callback function for robot to look for aruco marker
     * 
     * @param msg 
     */
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Broadcast frame of the final destination
     * 
     * @param id 
     */
    void final_destination_broadcast(int id);

    /**
     * @brief Callback function for robot to move to the final goal.
     * 
     */
    void listener_callback();
};

#endif