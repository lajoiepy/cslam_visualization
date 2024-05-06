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
                               parameters=[LaunchConfiguration('config')])
    
    return [
        visualization_node,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
                              default_value=os.path.join(
                                  get_package_share_directory('cslam_visualization'),
                                  'config/'),
                              description=''),
        DeclareLaunchArgument('config_file',
                              default_value='lidar.yaml',
                              description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('config_file')
                              ],
                              description=''),
        OpaqueFunction(function=launch_setup)
    ])
