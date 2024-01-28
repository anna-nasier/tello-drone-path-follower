from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pub = Node(name = "minimal_publisher", package = "navigation", executable = "publisher.py")
    nav = Node(
            package='navigation',
            # namespace='turtlesim1',
            executable='cpp_executable',
            name='navigation_node'
        )

    composable_node = ComposableNode(
        name='navigation_node',
        package='navigation', plugin='Navigation',
        parameters=[{'use_sim_time': True}]
        )
    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node], 
        output='screen'
        # prefix="xterm -e gdb -ex=r --args"
    )
    
    # uncomment the pu one for the real robot 
    return launch.LaunchDescription([
        DeclareLaunchArgument("use_multithread", default_value="false"), # changed from true 
        # pub,
        nav
        ])