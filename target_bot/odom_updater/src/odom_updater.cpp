#include <rclcpp/rclcpp.hpp>
#include "odom_updater/odom_updater.h"

void OdomUpdater::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
  geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to orresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "/robot1/odom";
    t.child_frame_id = "/robot1/base_footprint";

    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    m_tf_broadcaster->sendTransform(t);
}





