#butler - Autonomous Navigation and Tracking Robot 
#The Simulation bringup file for the butler robot

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    description = 'butler_description'
    bringup = 'butler_bringup'

    butler_description = os.path.join(get_package_share_path(description))
    gazebo_description = os.path.join(get_package_share_path('ros_gz_sim'))
    # rviz_config_path = os.path.join(get_package_share_path(description),
    #                                 'rviz', 'urdf_config.rviz')
    
    butler_brignup = os.path.join(get_package_share_path(bringup))
    butler_launch = os.path.join(butler_description, 'launch', 'butler.launch.py')
    gz_launch = os.path.join(gazebo_description, 'launch', 'gz_sim.launch.py')
    # world = os.path.join(butler_brignup, 'empty.sdf')
    world = os.path.join(butler_brignup,"worlds","hotel.sdf")
    rviz2_config_path = os.path.join(butler_brignup,"config","butler_rviz.rviz")

    robot_controllers = os.path.join(get_package_share_path(description),'config','controller_config.yaml',)
    butler_description_launch = IncludeLaunchDescription(launch_description_source = butler_launch)

    start_gazebo = IncludeLaunchDescription(launch_description_source = gz_launch,launch_arguments= {'gz_args': f"{world} -r",}.items())

    parameter_bridge_config = os.path.join(butler_brignup, 'config', 'gz_ros_topic_bridge.yaml')
    map_config = os.path.join(butler_brignup, 'config', 'map_config.yaml')
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
            arguments=[
                "-topic", "/robot_description",
        ],
        parameters=[{"/use_sim_time":True}]
    )
    
    map_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint_broadcaster',
            arguments=['5', '2.5', '0', '0', '0', '0.7071', '0.7071','map', 'odom']
        )



    ros_gz_bridge = Node(
        package=  'ros_gz_bridge',
        executable="parameter_bridge",
        parameters=[
            {"config_file":parameter_bridge_config}]
    )
   
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz2_config_path]
    )

    launch_map = Node(
        package="nav2_map_server",
        executable="map_server",
        name='map_server',
            parameters=[
            {"yaml_filename":map_config}]
    )

    nav2 = IncludeLaunchDescription(
        launch_description_source = os.path.join(butler_brignup,"launch","bringup_launch.py"),
    )

    return LaunchDescription([
        butler_description_launch,
        start_gazebo,
        spawn_robot,
        ros_gz_bridge,
        rviz2_node,
        map_tf,
        # map_tf2,
        # localize_robot,
        # joint_pub,
        launch_map,
        nav2
    ])