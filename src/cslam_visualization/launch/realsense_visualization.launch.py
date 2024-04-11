import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys

def launch_setup(context, *args, **kwargs):

    visualization_node = Node(package='cslam_visualization',
                               executable='visualization_node.py',
                               name='cslam_visualization',
                               parameters=[LaunchConfiguration('config')])

    zenoh_dds_brigde_process = ExecuteProcess(
                        cmd=['zenoh-bridge-dds', '-d', LaunchConfiguration('domain_id').perform(context), '--allow', '/cslam/.*']
                    )

    print(LaunchConfiguration('rviz_config').perform(context))
    rviz_node = Node(package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config').perform(context)],
        )
    
    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID',  LaunchConfiguration('domain_id').perform(context)),
        visualization_node,
        rviz_node,
        zenoh_dds_brigde_process,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
                              default_value=os.path.join(
                                  get_package_share_directory('cslam_visualization'),
                                  'config/'),
                              description=''),
        DeclareLaunchArgument('config_file',
                              default_value='realsense.yaml',
                              description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('config_file')
                              ],
                              description=''),
        DeclareLaunchArgument('domain_id',
                              default_value='10',
                              description=''),
        DeclareLaunchArgument('rviz_config_file',
                              default_value='realsense.rviz',
                              description=''),
        DeclareLaunchArgument('rviz_config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('rviz_config_file')
                              ],
                              description=''),
        OpaqueFunction(function=launch_setup)
    ])
