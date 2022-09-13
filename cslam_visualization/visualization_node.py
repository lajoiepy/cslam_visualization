#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
from cslam_visualization.pose_graph_visualizer import PoseGraphVisualizer
from cslam_visualization.keypoints3d_visualizer import Keypoints3DVisualizer

if __name__ == '__main__':

    rclpy.init(args=None)
    node = Node('visualizer')
    node.declare_parameters(
            namespace='',
            parameters=[('nb_colors', 10),
                        ('visualization_update_period_ms', 100)])
    params = {}
    params['nb_colors'] = node.get_parameter(
        'nb_colors').value
    params['visualization_update_period_ms'] = node.get_parameter(
        'visualization_update_period_ms').value
    pose_graph_viz = PoseGraphVisualizer(node, params)
    keypoints_viz = Keypoints3DVisualizer(node, params, pose_graph_viz)
    node.get_logger().info('Initialization done.')
    rclpy.spin(node)
    rclpy.shutdown()
