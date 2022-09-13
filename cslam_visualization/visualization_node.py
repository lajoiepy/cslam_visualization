#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
import cslam_visualization.pose_graph_visualizer as pgv

if __name__ == '__main__':

    rclpy.init(args=None)
    viz = pgv.PoseGraphVisualizer()
    viz.get_logger().info('Initialization done.')
    rclpy.spin(viz)
    rclpy.shutdown()
