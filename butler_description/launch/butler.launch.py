from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
import xacro
def generate_launch_description():
    description = 'butler_description'
    robot_urdf = 'butler.xacro'

    urdf_path = os.path.join(get_package_share_path(description),
                             'urdf', robot_urdf)
    # rviz_config_path = os.path.join(get_package_share_path(description),
    #                                 'rviz', 'urdf_config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    # robot_description = xacro.process_file(robot_description).toxml()

    # print(robot_description.value[0])
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output='screen',
        parameters=[{'use_sim_time' : True}]
    )


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])