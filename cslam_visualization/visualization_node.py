#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
import cslam_visualization.pose_graph_visualizer as pgv

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
    pose_graph_viz = pgv.PoseGraphVisualizer(node, params)
    node.get_logger().info('Initialization done.')
    rclpy.spin(node)
    rclpy.shutdown()
