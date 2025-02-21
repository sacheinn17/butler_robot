#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unordered_map>

class MoveToWaypoint : public BT::SyncActionNode {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    MoveToWaypoint(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_nav2_action_client")) {

        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

        /*This constructor defines the required way points*/
        waypoints_ = {
            {"kitchen", {2.0, 3.0, 1.57}},
            {"table_1", {4.0, 1.5, 0.0}},
            {"table_2", {6.0, 2.0, -1.57}},
            {"table_3", {3.0, 5.0, 3.14}},
            {"exit", {0.0, 0.0, 0.0}}
        };
    }

    static BT::PortsList providedPorts() {
        /*The ports for bt board*/
        return {BT::InputPort<std::string>("target")};
    }

    BT::NodeStatus tick() {
        std::string target;
        if (!getInput<std::string>("target", target)) {
            RCLCPP_ERROR(node_->get_logger(), "No target provided!");
            return BT::NodeStatus::FAILURE;
        }

        if (waypoints_.find(target) == waypoints_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid target: %s", target.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto [x, y, yaw] = waypoints_[target];

        RCLCPP_INFO(node_->get_logger(), "Navigating to %s: (%.2f, %.2f, %.2f)", target.c_str(), x, y, yaw);
        return send_goal(x, y, yaw);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    std::unordered_map<std::string, std::tuple<double, double, double>> waypoints_; //waypoints

    BT::NodeStatus send_goal(double x, double y, double yaw) {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available!");
            return BT::NodeStatus::FAILURE;
        }

        nav2_msgs::action::NavigateToPose::Goal goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw);

        std::shared_future<std::shared_ptr<MoveToWaypoint::GoalHandleNavigate>> future_goal_handle = client_ptr_->async_send_goal(goal_msg);
        std::future_status status = future_goal_handle.wait_for(std::chrono::seconds(10));

        if (status != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send goal!");
            return BT::NodeStatus::FAILURE;
        }

        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal rejected by server!");
            return BT::NodeStatus::FAILURE;
        }

        auto result_future = client_ptr_->async_get_result(goal_handle);
        auto result_status = result_future.wait_for(std::chrono::seconds(30));

        if (result_status != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "Goal execution timeout!");
            return BT::NodeStatus::FAILURE;
        }

        MoveToWaypoint::GoalHandleNavigate::WrappedResult result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Goal reached successfully!");
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_WARN(node_->get_logger(), "Goal execution failed!");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.z = sin(yaw / 2.0);
        q.w = cos(yaw / 2.0);
        return q;
    }
};


int main(){

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MoveToWaypoint>("MoveToWaypoint");
      auto tree = factory.createTreeFromFile("./../bt_tree.xml");

  tree.tickWhileRunning();
    
        return 0;
}