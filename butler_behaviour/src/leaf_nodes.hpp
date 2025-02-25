
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <iostream>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>



geometry_msgs::msg::TransformStamped lookupTransform(tf2_ros::Buffer& tf_buffer_, std::shared_ptr<rclcpp::Node> node_,char *source_frame, char *target_frame){
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(source_frame, target_frame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
        return transform;
    }
    RCLCPP_INFO(node_->get_logger(), "Transform: x=%f, y=%f", transform.transform.translation.x, transform.transform.translation.y);
        return transform;
}