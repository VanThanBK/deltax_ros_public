import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # return LaunchDescription([
    #     Node(
    #         package='deltax_driver',
    #         executable='robot_driver',
    #         name='robot_driver',
    #         output='screen'),
    # ])

    return LaunchDescription([
        Node(
            package='deltax_driver',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
    ])