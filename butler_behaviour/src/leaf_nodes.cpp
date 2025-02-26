#include "leaf_nodes.hpp"

float positions[6][2] = {
    {2.0,2.0}, //kitchen
    {4.0,4.0}, //dock
    {6.0,6.0}, //table 1
    {8.0,8.0}, //table 2
    {9.0,9.0}, //table 3
    {10.0,10.0},
};

std::vector<int> pending_orders = {2,3,1,2,1,3,4,3,2,1};
std::vector<int> cancelled_orders;
std::vector<int> executing_orders_vec;
std::vector<int> waste;
class SetNavGoal : public BT::ThreadedAction {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit SetNavGoal(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node,rclcpp_action::Client<NavigateToPose>::SharedPtr client) : BT::ThreadedAction(name, config), node_(node) {
        
        order_subscription = node_->create_subscription<std_msgs::msg::Int32>("pending_orders", 10, std::bind(&SetNavGoal::order_callback, this, std::placeholders::_1));
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
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = getGoalPose(goal_number);

        return sendGoal(client_,goal_msg);

    }

    void halt() override {
        sleep(1);
        RCLCPP_WARN(node_->get_logger(), "Halting navigation goal");
    }

    void order_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        pending_orders.push_back(msg->data + 2);
    }

    void order_cancel_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (pending_orders[0] == msg->data+2) {
            cancelled_orders.push_back(msg->data+2);
        }
    } 

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr order_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr order_cancel_subscription;
    std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;

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
    std::cout << "checking location " << std::endl;

    try{
    transform = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    if (transform.transform.translation.x < x+0.5 && transform.transform.translation.x > x-0.5 || transform.transform.translation.y < y+0.5 && transform.transform.translation.y > y-0.5){
        std::cout << "In location" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    sleep(1);
    std::cout << "Not in location" << std::endl;
    RCLCPP_INFO(node_->get_logger(), "Transform: x=%f, y=%f", transform.transform.translation.x, transform.transform.translation.y);
    RCLCPP_INFO(node_->get_logger(), "Target: x=%d, y=%d", x,y);
    
    return BT::NodeStatus::FAILURE;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
        return BT::NodeStatus::FAILURE;
}
}

BT::NodeStatus order_cancelled(){
    sleep(1);
    std::cout << "Checking Available pending_orders!" << std::endl;

    if (pending_orders.size() < 0 ){
        std::cout << "No pending_orders available to cancel" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (cancelled_orders.size()>0 && pending_orders[0] == cancelled_orders[0]){
        std::cout << "Order on table" << pending_orders[0] <<" cancelled" << std::endl;

        pending_orders.erase(pending_orders.begin());
        cancelled_orders.erase(cancelled_orders.begin());

        return BT::NodeStatus::SUCCESS;
    }
    else{
        std::cout << "Current order is not getting executed" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus executingOrders(){

    std::cout << "Checking for current executing pending_orders" << std::endl;
    sleep(1);
    if (executing_orders_vec.size() > 0){
        std::cout << "pending_orders being executed" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    std::cout << "No pending_orders being executed" << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus pendingOrders(){
    std::cout << "Checking for any pending pending_orders" << std::endl;
    sleep(1);
    if (pending_orders.size() > 0){
        std::cout << "pending_orders available" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    std::cout << "No pending_orders" << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus pop_pending_orders(){
    sleep(1);
    if (pending_orders.size() > 0){
        std::cout << "Popping pending_orders" << pending_orders[0] << std::endl;
        pending_orders.erase(pending_orders.end());
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus executeOrder(auto client){
    sleep(1);

if (executing_orders_vec.size()>0){
nav2_msgs::action::NavigateToPose::Goal goal_msg = nav2_msgs::action::NavigateToPose::Goal();
goal_msg.pose.header.frame_id = "map";
goal_msg.pose.pose.position.x = positions[executing_orders_vec[0]+2][0];
goal_msg.pose.pose.position.x = positions[executing_orders_vec[0]+2][0];
std::cout << "Executing pending_orders" << std::endl;
            
  return sendGoal(client,goal_msg);
}
  return BT::NodeStatus::SUCCESS;

}

BT::NodeStatus orderConfiremd(){
    sleep(1);
std::cout << "Checking for order confirmation" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus add_to_waste(){
    sleep(1);

    if (executing_orders_vec.size()>0){
    std::cout<<"Adding to waste "<<executing_orders_vec[0]<<"\n";
    int order = executing_orders_vec[0];
        waste.push_back(order);
        std::cout<<"size of waste "<<waste.size()<<"\n";
        executing_orders_vec.erase(executing_orders_vec.begin());
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus emptyWaste(){
    std::cout<<"Checking for waste to get emptied\n";
    sleep(1);
    if (waste.size()>0){
    std::cout<<"Emptying Waste\n";
    waste.erase(waste.begin());
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus takeOrders(){
    if(pending_orders.size() > 0){
        std::cout<<"Taking pending_orders"<<"\n";
        executing_orders_vec.push_back(pending_orders[0]);
        pending_orders.erase(pending_orders.begin());
        sleep(1);
        return BT::NodeStatus::SUCCESS;
    }
    else{
        std::cout<<"No pending orders\n";
        return BT::NodeStatus::FAILURE;
    }
    sleep(1);

}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_nav_goal");

    tf2_ros::Buffer tf_buffer_{node->get_clock()};
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    auto client  = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose");
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SetNavGoal>("go_to_kitchen", node,client);
    factory.registerNodeType<SetNavGoal>("go_to_table", node,client);
    factory.registerNodeType<SetNavGoal>("go_to_dock", node,client);
    factory.registerSimpleCondition("printVal", std::bind(printVal));
    factory.registerSimpleCondition("charge", std::bind(charge));
    factory.registerSimpleCondition("pending_orders", std::bind(pendingOrders));
    factory.registerSimpleCondition("in_dock", [&node, &tf_buffer_](BT::TreeNode &tree_node) -> BT::NodeStatus {std::cout<<"in dock ? ";return checkLocation(node,tf_buffer_,0,0);});
    factory.registerSimpleCondition("on_kitchen", [&node, &tf_buffer_](BT::TreeNode &tree_node) -> BT::NodeStatus {std::cout<<"On kitchen ? ";return checkLocation(node,tf_buffer_,0.6,3.0);});
    factory.registerSimpleCondition("order_cancelled", std::bind(order_cancelled));
    factory.registerSimpleCondition("on_table", [&node, &tf_buffer_](BT::TreeNode &tree_node) -> BT::NodeStatus {std::cout<<"On table ? ";return checkLocation(node,tf_buffer_,positions[executing_orders_vec[0]][0],positions[executing_orders_vec[0]][1]);});
    factory.registerSimpleCondition("executing_orders", std::bind(executingOrders));
    factory.registerSimpleCondition("order_confirmed", std::bind(orderConfiremd));
    factory.registerSimpleAction("pop_pending_orders", std::bind(pop_pending_orders));
    factory.registerSimpleAction("execute_order", std::bind(executeOrder<decltype(client)>, client));
    factory.registerSimpleAction("add_to_waste", std::bind(add_to_waste));
    factory.registerSimpleAction("empty_waste", std::bind(emptyWaste));
    factory.registerSimpleAction("take_orders", std::bind(takeOrders));

    auto tree = factory.createTreeFromFile("/home/sac/projects/ros/butler_robot/src/butler_behaviour/src/bt_tree_test.xml");

    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
        tree.tickOnce();
        // BT::Groot2Publisher publisher(tree);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
