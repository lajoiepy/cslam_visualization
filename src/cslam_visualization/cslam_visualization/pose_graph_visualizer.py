import zenoh
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from cslam_common_interfaces.msg import PoseGraph
from distinctipy import distinctipy

import rerun as rr

class PoseGraphVisualizer():

    def __init__(self, node, params, zenoh_session):
        self.node = node
        self.params = params
        self.nb_colors = self.params["nb_colors"]
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]
        self.colors = distinctipy.get_colors(self.nb_colors, colorblind_type="Deuteranomaly")
        self.robot_pose_graphs = {}
        self.origin_robot_ids = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)

        def _pose_graph_cb(sample):
            try:
                msg = deserialize_message(bytes(sample.payload.to_bytes()), PoseGraph)
                self.pose_graph_callback(msg)
            except Exception as e:
                self.node.get_logger().warn(f"PoseGraphVisualizer: deserialize error: {e}")

        self._sub_pose_graph = zenoh_session.declare_subscriber(
            "cslam/*/viz/pose_graph", _pose_graph_cb)

        rr.init("cslam_visualization")
        rr.spawn(connect=True)

        # Declare coordinate convention: optical frame (z-forward, x-right, y-down).
        rr.log("global_map", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

        self.viz_counter = 0.0 # TODO: Use timestamp from PoseGraph message instead

    def pose_graph_callback(self, msg):
        self.origin_robot_ids[msg.robot_id] = msg.origin_robot_id

        for pose in msg.values:
            if pose.key.keyframe_id % self.params["pose_graph_subsampling_factor"] != 0:
                continue
            rid = pose.key.robot_id
            if rid not in self.robot_pose_graphs:
                self.robot_pose_graphs[rid] = {}
            self.robot_pose_graphs[rid][pose.key.keyframe_id] = pose
        # self.robot_pose_graphs_edges[msg.robot_id] = msg.edges

    def robot_pose_graphs_to_rerun(self):
        """Converts a PoseGraph messages to a MarkerArray message"""
        # Nodes (poses)
        for robot_id, pose_graph in self.robot_pose_graphs.items():
            rr.set_time("stable_time", sequence=int(self.viz_counter))
            linestrips_points = []
            # Sort by keyframe_id so the trajectory line connects poses in order.
            for _, node in sorted(pose_graph.items()):
                linestrips_points.append([node.pose.position.x, node.pose.position.y, node.pose.position.z])
                tf = rr.Transform3D(translation=[node.pose.position.x, node.pose.position.y, node.pose.position.z], rotation=rr.Quaternion(xyzw = [node.pose.orientation.x, node.pose.orientation.y, node.pose.orientation.z, node.pose.orientation.w]))
                rr.log(
                    "global_map/robot_" + str(robot_id) + "_map/poses/pose_" + str(node.key.keyframe_id),
                    tf,
                )

            # Rerun
            rr.log("global_map/robot_" + str(robot_id) + "_map/pose_graph",  rr.LineStrips3D([linestrips_points], colors=[self.colors[robot_id % self.nb_colors]]))


    def visualization_callback(self):
        self.robot_pose_graphs_to_rerun()
        self.viz_counter += 1.0
