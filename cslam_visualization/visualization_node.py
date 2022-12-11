#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
from cslam_visualization.pose_graph_visualizer import PoseGraphVisualizer
from cslam_visualization.keypoints3d_visualizer import Keypoints3DVisualizer
from cslam_visualization.pointcloud_visualizer import PointCloudVisualizer

if __name__ == '__main__':

    rclpy.init(args=None)
    node = Node('visualizer')
    node.declare_parameters(
            namespace='',
            parameters=[('nb_colors', 10),
                        ('visualization_update_period_ms', 100),
                        ('enable_keypoints_visualization', False),
                        ('enable_pointclouds_visualization', False),
                        ('produce_mesh', False),
                        ('voxel_size', 0.5),
                        ('rotation_to_sensor_frame', None),
                        ('pose_graph_markers_size', 0.1)])
    params = {}
    params['nb_colors'] = node.get_parameter(
        'nb_colors').value
    params['visualization_update_period_ms'] = node.get_parameter(
        'visualization_update_period_ms').value
    params['enable_keypoints_visualization'] = node.get_parameter(
        'enable_keypoints_visualization').value
    params['enable_pointclouds_visualization'] = node.get_parameter(
        'enable_pointclouds_visualization').value
    params['voxel_size'] = node.get_parameter(
        'voxel_size').value
    params['rotation_to_sensor_frame'] = node.get_parameter(
        'rotation_to_sensor_frame').value
    params['pose_graph_markers_size'] = node.get_parameter(
        'pose_graph_markers_size').value
    params['produce_mesh'] = node.get_parameter(
        'produce_mesh').value
    pose_graph_viz = PoseGraphVisualizer(node, params)
    keypoints_viz = []
    if params['enable_keypoints_visualization']:
        keypoints_viz = Keypoints3DVisualizer(node, params, pose_graph_viz)
    pointcloud_viz = []
    if params['enable_pointclouds_visualization']:
        pointcloud_viz = PointCloudVisualizer(node, params, pose_graph_viz)
    node.get_logger().info('Initialization done.')
    rclpy.spin(node)
    rclpy.shutdown()
