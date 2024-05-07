#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
from cslam_visualization.pose_graph_visualizer import PoseGraphVisualizer
from cslam_visualization.keypoints3d_visualizer import Keypoints3DVisualizer
from cslam_visualization.pointcloud_visualizer import PointCloudVisualizer

def extract_params(node, initial_params):
    params = {}
    for param in initial_params:
        params[param[0]] = node.get_parameter(param[0]).value
    return params

if __name__ == '__main__':

    rclpy.init(args=None)
    node = Node('visualizer')
    initial_params = [('nb_colors', 10),
                        ('visualization_update_period_ms', 100),
                        ('enable_keypoints_visualization', False),
                        ('enable_pointclouds_visualization', False),
                        ('produce_mesh', False),
                        ('voxel_size', 0.5),
                        ('rotation_to_sensor_frame', [1.0, 0.0, 0.0, 0.0]),
                        ('pose_graph_markers_size', 0.1),
                        ('pose_graph_subsampling_factor', 1)]
    node.declare_parameters(
            namespace='',
            parameters=initial_params)
    params = extract_params(node, initial_params) 
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
