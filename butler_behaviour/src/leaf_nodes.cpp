#include "leaf_nodes.hpp"



float positions[5][2] = {
    {2.0,2.0}, //kitchen
    {4.0,4.0}, //dock
    {6.0,6.0}, //table 1
    {8.0,8.0}, //table 2
    {10.0,10.0} //table 3
};

std::vector<int> orders = {};
std::vector<int> cancelled_orders = {};

class SetNavGoal : public BT::ThreadedAction {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit SetNavGoal(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node) : BT::ThreadedAction(name, config), node_(node) {
        client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
        order_subscription = node_->create_subscription<std_msgs::msg::Int32>("orders", 10, std::bind(&SetNavGoal::order_callback, this, std::placeholders::_1));
        order_cancel_subscription = node_->create_subscription<std_msgs::msg::Int32>("orders_cancelled", 10, std::bind(&SetNavGoal::order_cancel_callback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("goal_number") };
    }

    BT::NodeStatus tick() override {
        int goal_number;
        if (!getInput("goal_number", goal_number)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing goal_number input!");
            return BT::NodeStatus::FAILURE;
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available!");
            return BT::NodeStatus::FAILURE;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = getGoalPose(goal_number);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&SetNavGoal::resultCallback, this, std::placeholders::_1);

        future_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(node_->get_logger(), "Sent goal %d", goal_number);
        auto success = client_->async_get_result(future_goal_handle_.get()).get().code;
        std::cout <<"Action " << (success == rclcpp_action::ResultCode::SUCCEEDED)<<"\n" ;
        if (success == rclcpp_action::ResultCode::SUCCEEDED)
        {
            std::cout<< "Finished" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        std::cout<<"Running"<<std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void halt() override {
        RCLCPP_WARN(node_->get_logger(), "Halting navigation goal");
    }

    void order_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        orders.push_back(msg->data + 2);
    }

    void order_cancel_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (orders[0] == msg->data+2) {
            cancelled_orders.push_back(msg->data+2);
        }
    } 

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr order_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr order_cancel_subscription;
    std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;

    void resultCallback(const GoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Goal reached successfully!");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Goal failed!");
        }
    }

    geometry_msgs::msg::PoseStamped getGoalPose(int goal_number) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = positions[goal_number][0];
        pose.pose.position.y = positions[goal_number][1];
        return pose;
    }
};

BT::NodeStatus printVal(){
    std::cout << "Hello World!" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus charge(){
    std::cout << "Charge full" << std::endl; //emualte battery monitoring
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus checkLocation(std::shared_ptr<rclcpp::Node>& node_,tf2_ros::Buffer& tf_buffer_,int x, int y){
    geometry_msgs::msg::TransformStamped transform;
    
    try{
    transform = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    if (transform.transform.translation.x < x+0.1 && transform.transform.translation.x > x-0.1 && transform.transform.translation.y < y+0.1 && transform.transform.translation.y > y-0.1){
        std::cout << "In location" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    std::cout << "Not in location" << std::endl;
    RCLCPP_INFO(node_->get_logger(), "Transform: x=%f, y=%f", transform.transform.translation.x, transform.transform.translation.y);
    return BT::NodeStatus::FAILURE;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
        return BT::NodeStatus::FAILURE;
}
}

BT::NodeStatus checkOrders(){
    if (orders.size() < 0){
        std::cout << "No orders available to cancel" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (orders[0] == cancelled_orders[0]){
        std::cout << "Order on table" << orders[0] <<" cancelled" << std::endl;

        orders.erase(orders.begin());
        cancelled_orders.erase(cancelled_orders.begin());

        return BT::NodeStatus::SUCCESS;
    }
    else{
        std::cout << "Current order is not getting executed" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus pendingOrders(){
    if (orders.size() > 0){
        std::cout << "Orders available" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    std::cout << "No orders" << std::endl;
    return BT::NodeStatus::FAILURE;
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_nav_goal");

    tf2_ros::Buffer tf_buffer_{node->get_clock()};
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SetNavGoal>("go_to_kitchen", node);
    factory.registerNodeType<SetNavGoal>("go_to_table", node);
    factory.registerSimpleCondition("printVal", std::bind(printVal));
    factory.registerSimpleCondition("charge", std::bind(charge));
    factory.registerSimpleCondition("pending_orders", std::bind(pendingOrders));
    factory.registerSimpleCondition("in_dock", [&node, &tf_buffer_](BT::TreeNode &tree_node) -> BT::NodeStatus {return checkLocation(node,tf_buffer_,0,0);});
    factory.registerSimpleCondition("on_kitchen", [&node, &tf_buffer_](BT::TreeNode &tree_node) -> BT::NodeStatus {return checkLocation(node,tf_buffer_,2.0,2.0);});
    factory.registerSimpleCondition("order_cancelled", std::bind(checkOrders));


    auto tree = factory.createTreeFromFile("/home/sac/projects/ros/butler_robot/src/butler_behaviour/src/bt_tree_test.xml");

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        tree.tickOnce();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
