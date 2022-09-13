import rclpy
from rclpy.node import Node

from cslam_common_interfaces.msg import PoseGraph
from visualization_msgs.msg import MarkerArray, Marker
from distinctipy import distinctipy


class PoseGraphVisualizer():

    def __init__(self, node, params):
        self.node = node
        self.params = params
        self.pose_graph_markers_publisher = self.node.create_publisher(
            MarkerArray, "/viz/pose_graph_markers", 10)
        self.nb_colors = self.params["nb_colors"]
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]  
        self.colors = distinctipy.get_colors(self.nb_colors)
        self.pose_graph_subscriber = self.node.create_subscription(
            PoseGraph, '/viz/pose_graph', self.pose_graph_callback, 10)
        self.robot_pose_graphs = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)

    def pose_graph_callback(self, msg):
        self.node.get_logger().info('Received pose graph from robot ' +
                               str(msg.robot_id))
        self.robot_pose_graphs[msg.robot_id] = msg

    def robot_pose_graphs_to_marker_array(self):
        """Converts a PoseGraph messages to a MarkerArray message"""
        marker_array = MarkerArray()

        # Nodes (poses)
        for robot_id, pose_graph in self.robot_pose_graphs.items():
            print('robot_id: ', robot_id)
            color = self.colors[robot_id % self.nb_colors]
            marker = Marker()
            marker.header.frame_id = "robot" + str(pose_graph.origin_robot_id) + "_map"
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = "poses"
            marker.id = robot_id
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.frame_locked = True
            for node in pose_graph.values:
                marker.points.append(node.pose.position)
            marker_array.markers.append(marker)

        # Edges (constraints)
        for robot_id, pose_graph in self.robot_pose_graphs.items():
            color = self.colors[robot_id % self.nb_colors]
            marker = Marker()
            marker.header.frame_id = "robot" + str(pose_graph.origin_robot_id) + "_map"
            marker.header.stamp = rclpy.time.Time().to_msg()
            marker.ns = "edges"
            marker.id = robot_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.5
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.frame_locked = True
            for edge in pose_graph.edges:
                if edge.key_from.robot_id in self.robot_pose_graphs and edge.key_to.robot_id in self.robot_pose_graphs:
                    if edge.key_from.keyframe_id < len(
                            self.robot_pose_graphs[edge.key_from.robot_id].
                            values) and edge.key_to.keyframe_id < len(
                                self.robot_pose_graphs[
                                    edge.key_to.robot_id].values):
                        marker.points.append(
                            self.robot_pose_graphs[edge.key_from.robot_id].
                            values[edge.key_from.keyframe_id].pose.position)
                        marker.points.append(
                            self.robot_pose_graphs[edge.key_to.robot_id].
                            values[edge.key_to.keyframe_id].pose.position)
            marker_array.markers.append(marker)

        return marker_array

    def visualization_callback(self):
        marker_array = self.robot_pose_graphs_to_marker_array()
        self.pose_graph_markers_publisher.publish(marker_array)
