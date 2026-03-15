#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message

import zenoh

from cslam_common_interfaces.msg import InterRobotLoopClosure
from cslam_visualization.pose_graph_visualizer import PoseGraphVisualizer
from cslam_visualization.pointcloud_visualizer import PointCloudVisualizer
from cslam_visualization.result_saver import ResultSaver

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
                        ('mesh_voxel_size', 0.05),
                        ('mesh_poisson_depth', 8),
                        ('rotation_to_sensor_frame', [1.0, 0.0, 0.0, 0.0]),
                        ('pose_graph_markers_size', 0.1),
                        ('pose_graph_subsampling_factor', 1),
                        ('use_real_colors', False),
                        ('enable_result_saving', False),
                        ('save_dir', '/tmp/cslam_results'),
                        ('max_nb_robots', 3),
                        ('gt_paths', ['', '', '']),
                        ('save_period_sec', 30.0)]
    node.declare_parameters(
            namespace='',
            parameters=initial_params)
    params = extract_params(node, initial_params)

    zenoh_session = zenoh.open(zenoh.Config())

    pose_graph_viz = PoseGraphVisualizer(node, params, zenoh_session)
    keypoints_viz = []
    if params['enable_keypoints_visualization']:
        from cslam_visualization.keypoints3d_visualizer import Keypoints3DVisualizer
        keypoints_viz = Keypoints3DVisualizer(node, params, pose_graph_viz)
    pointcloud_viz = []
    if params['enable_pointclouds_visualization']:
        pointcloud_viz = PointCloudVisualizer(node, params, pose_graph_viz, zenoh_session)
    mesh_viz = []
    if params['produce_mesh']:
        from cslam_visualization.mesh_visualizer import MeshVisualizer
        mesh_viz = MeshVisualizer(node, params, pose_graph_viz)
    result_saver = []
    if params['enable_result_saving']:
        result_saver = ResultSaver(node, params, pose_graph_viz, zenoh_session)

    def _inter_robot_lc_cb(sample):
        try:
            msg = deserialize_message(
                bytes(sample.payload.to_bytes()), InterRobotLoopClosure)
            status = "SUCCESS" if msg.success else "FAILED"
            node.get_logger().info(
                f"[Inter-robot LC] {status} — "
                f"r{msg.robot0_id}/kf{msg.robot0_keyframe_id} <-> "
                f"r{msg.robot1_id}/kf{msg.robot1_keyframe_id}")
        except Exception as e:
            node.get_logger().warn(f"inter_robot_lc deserialize error: {e}")

    _sub_lc = zenoh_session.declare_subscriber(
        "cslam/*/inter_robot_loop_closure", _inter_robot_lc_cb)

    node.get_logger().info('Initialization done.')
    rclpy.spin(node)
    zenoh_session.close()
    rclpy.shutdown()
