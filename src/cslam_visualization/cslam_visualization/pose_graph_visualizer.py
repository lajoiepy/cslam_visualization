import rclpy
from rclpy.node import Node
from rclpy.time import Duration, Time

from cslam_common_interfaces.msg import PoseGraph
from distinctipy import distinctipy

import rerun as rr

class PoseGraphVisualizer():

    def __init__(self, node, params):
        self.node = node
        self.params = params
        self.nb_colors = self.params["nb_colors"]
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]  
        self.colors = distinctipy.get_colors(self.nb_colors, colorblind_type="Deuteranomaly")
        self.pose_graph_subscriber = self.node.create_subscription(
            PoseGraph, '/cslam/viz/pose_graph', self.pose_graph_callback, 10)
        self.robot_pose_graphs = {}
        # self.robot_pose_graphs_edges = {}
        self.origin_robot_ids = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)
        
        rr.init("cslam_visualization", spawn=False)
        rr.connect()

        self.viz_counter = 0.0 # TODO: Use timestamp from PoseGraph message instead

    def pose_graph_callback(self, msg):
        self.origin_robot_ids[msg.robot_id] = msg.origin_robot_id
        if msg.robot_id not in self.robot_pose_graphs:
            self.robot_pose_graphs[msg.robot_id] = {}

        for pose in msg.values:
            if pose.key.keyframe_id % self.params["pose_graph_subsampling_factor"] != 0:
                continue
            self.robot_pose_graphs[msg.robot_id][pose.key.keyframe_id] = pose
        # self.robot_pose_graphs_edges[msg.robot_id] = msg.edges

    def robot_pose_graphs_to_rerun(self):
        """Converts a PoseGraph messages to a MarkerArray message"""            
        # Nodes (poses)
        for robot_id, pose_graph in self.robot_pose_graphs.items():
            rr.set_time_seconds("stable_time", self.viz_counter)
            linestrips_points = []
            for _, node in pose_graph.items():
                linestrips_points.append([node.pose.position.x, node.pose.position.y, node.pose.position.z])
                tf = rr.Transform3D(translation=[node.pose.position.x, node.pose.position.y, node.pose.position.z], rotation=rr.Quaternion(xyzw = [node.pose.orientation.x, node.pose.orientation.y, node.pose.orientation.z, node.pose.orientation.w]))
                # TODO: downsample the poses
                rr.log(
                    "global_map/robot_" + str(robot_id) + "_map/poses/pose_" + str(node.key.keyframe_id),
                    tf,
                )

            # Rerun
            rr.log("global_map/robot_" + str(robot_id) + "_map/pose_graph",  rr.LineStrips3D([linestrips_points], colors=[self.colors[robot_id % self.nb_colors]]))
            

    def visualization_callback(self):
        self.robot_pose_graphs_to_rerun()
        self.viz_counter += 1.0
