
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
#include <chrono>
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


BT::NodeStatus sendGoal(auto client_,auto goal_msg){

        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available!");
            return BT::NodeStatus::FAILURE;
        }

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&SetNavGoal::resultCallback, this, std::placeholders::_1);

        future_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
        sleep(3);
        // RCLCPP_INFO(node_->get_logger(), "Sent goal %d", goal_number);
        auto success = client_->async_get_result(future_goal_handle_.get()).get().code;
        std::cout <<"Action " << (success == rclcpp_action::ResultCode::SUCCEEDED)<<"\n" ;
        if (success == rclcpp_action::ResultCode::SUCCEEDED)
        {
            std::cout<< "Finished" << std::endl;
            sleep(1);
            return BT::NodeStatus::SUCCESS;
        }
        std::cout<<"Running"<<std::endl;
        return BT::NodeStatus::RUNNING;
}

void resultCallback(const GoalHandle::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "Goal reached successfully!");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Goal failed!");
    }
}