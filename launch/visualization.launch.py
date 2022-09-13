import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    visualization_node = Node(package='cslam_visualization',
                               executable='visualization_node.py',
                               name='cslam_visualization',
                               parameters=[])
    
    # TODO: launch RVIZ2 with custom config file

    return [
        visualization_node,
    ]


def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
