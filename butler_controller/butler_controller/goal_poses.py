import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from custom_interfaces.action import Move  # Placeholder for action messages
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer,GoalResponse

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # Publisher for sending goal poses
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Define fixed goal for kitchen
        self.kitchen_goal = self.create_goal_pose(2.0, 3.0, 1.0, 0.0)

        # Table goal positions (Example: Table 1, Table 2, etc.)
        self.table_goals = {
            0: self.create_goal_pose(2.0, 1.5, 0.0, 1.0), 
            1: self.create_goal_pose(4.0, 2.0, 0.0, 1.0),
            2: self.create_goal_pose(6.0, 2.5, 0.0, 1.0),
            3: self.create_goal_pose(8.0, 3.0, 0.0, 1.0),
            4: self.create_goal_pose(10.0, 3.5, 0.0, 1.0),
        }

        # Action Servers
        self.move_action_server = ActionServer(
            self, Move, 'go_to_kitchen', 
            goal_callback=self.execute_go_to_pose
            execute_callback=self.execute_go_to_kitchen
            cancel_callback=self.cancel_callback)


        self.get_logger().info('Navigation Action Server Ready!')

    def execute_go_to_pose(self, goal_handle):
        """Handles the go_to_pose action request."""

        object = goal_handle.request.pose
        if object == 0:
            self.get_logger().info('Going to the Kitchen...')
        elif object == 1:
            self.get_logger().info('Going to the Dock...')
        else:
            self.get_logger().info(f'Going to the Table... {object-2}')
        self.goal_publisher.publish(self.table_goals[object])
        return GoalResponse.ACCEPT
        

    def create_goal_pose(self, x, y, z, w):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = z
        goal.pose.orientation.w = w
        return goal

    async def execute_go_to_kitchen(self, goal_handle):
        """Handles the go_to_kitchen action request."""
        self.get_logger().info('Going to the Kitchen...')
        self.goal_publisher.publish(self.kitchen_goal)
        goal_handle.succeed()
        return Move.Result()

    async def execute_go_to_table(self, goal_handle):
        """Handles the go_to_table action request."""
        # Extract table number from request
        table_number = goal_handle.request.data
        if table_number in self.table_goals:
            self.get_logger().info(f'Going to Table {table_number}...')
            self.goal_publisher.publish(self.table_goals[table_number])
            goal_handle.succeed()
        else:
            self.get_logger().warn(f'Table {table_number} does not exist!')
            goal_handle.abort()
        return Move.Result()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
