#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"

// Implementation of method goal_callback() from the class TargetReacher
void TargetReacher::goal_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    {
        if(!m_reached_goal){
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            m_msg.angular.z = 0.2;
            m_reached_first_goal = true;
           
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Final Goal reached");
            m_msg.angular.z = 0.0;
        }
        m_reached_goal = true;
        m_publisher->publish(m_msg);

        
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Goal not reached");
    }
}

// Implementation of method aruco_callback() from the class TargetReacher
void TargetReacher::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    if(m_reached_first_goal)
    {
        auto a = 0;
        if(msg->marker_ids.at(0)==0){a = 0;}
        else if(msg->marker_ids.at(0)==1){a = 1;}
        else if(msg->marker_ids.at(0)==2){a = 2;}
        else if(msg->marker_ids.at(0)==3){a = 3;}
        final_destination_broadcast(a);     
    }
}


// Implementation of method final_destination_broadcast() from the class TargetReacher
void TargetReacher::final_destination_broadcast(int id)
{
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = m_frame_id;
    t.child_frame_id = "final_destination";
    if(id==0){
        t.transform.translation.x = m_final_destination0_x;
        t.transform.translation.y = m_final_destination0_y;
        t.transform.translation.z = 0.0;
    }
    else if(id==1){
        t.transform.translation.x = m_final_destination1_x;
        t.transform.translation.y = m_final_destination1_x;
        t.transform.translation.z = 0.0;
    }
    else if(id==2){
        t.transform.translation.x = m_final_destination2_x;
        t.transform.translation.y = m_final_destination2_x;
        t.transform.translation.z = 0.0;
    }
    else if(id==3){
        t.transform.translation.x = m_final_destination3_x;
        t.transform.translation.y = m_final_destination3_x;
        t.transform.translation.z = 0.0;
    }
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    
    // Send the transformation
    m_tf_final_broadcaster->sendTransform(t);
}

// Implementation of method listener_callback() from the class TargetReacher
void TargetReacher::listener_callback()
{
    if(m_reached_goal)
    {
        geometry_msgs::msg::TransformStamped t;
        
        // Look up for the transformation between odom and object frames
        try
        {
            t = m_tf_buffer->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s","robot1/odom", "final_destination", ex.what());
            return;
        }
        m_bot_controller->set_goal(t.transform.translation.x, t.transform.translation.y);
        RCLCPP_INFO(this->get_logger(), "Position of object in world: [%f, %f, %f]", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    }
}

