import rclpy
from rclpy.node import Node
import numpy as np

from cslam_common_interfaces.msg import LocalImageDescriptors
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math
from cslam_visualization.utils.transform import Transform
from distinctipy import distinctipy
import copy

class Keypoints3DVisualizer():

    def __init__(self, node, params, pose_graph_viz):
        self.node = node
        self.params = params
        self.pose_graph_viz = pose_graph_viz
        self.keypoints_markers_publisher = self.node.create_publisher(
            MarkerArray, "/cslam/viz/keypoints_markers", 10)
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]  
        self.local_descriptors_subscriber = self.node.create_subscription(
            LocalImageDescriptors, '/cslam/viz/local_descriptors', self.local_descriptors_callback, 10)
        self.local_descriptors = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)

        self.previous_poses = {}

    def local_descriptors_callback(self, msg):
        if msg.robot_id not in self.local_descriptors:
            self.local_descriptors[msg.robot_id] = []
        self.local_descriptors[msg.robot_id].append(msg)

    def pose_to_transform(self, pose):
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        pos = [pose.position.x, pose.position.y, pose.position.z]
        return Transform(quat=quat, pos=pos)

    def point_to_array(self, point):
        pos = np.array([point.x, point.y, point.z, 1.0])
        return pos

    def check_exists_or_new(self, robot_id, keyframe_id):
        if robot_id not in self.previous_poses:
            return True
        if keyframe_id not in self.previous_poses[robot_id]:
            return True
        new = self.pose_graph_viz.robot_pose_graphs[robot_id][keyframe_id].pose.position
        previous = self.previous_poses[robot_id][keyframe_id].pose.position
        dist = np.linalg.norm([new.x-previous.x, new.y-previous.y, new.z-previous.z])
        if dist > 1e-1:
            return True
        return False

    def keypoints_to_marker_array(self):
        """Converts a PoseGraph messages to a MarkerArray message"""
        marker_array = MarkerArray()

        # Nodes (poses)
        for robot_id, sensor_data in self.local_descriptors.items():
            if robot_id in self.pose_graph_viz.robot_pose_graphs:
                for keypoints in sensor_data:
                    if self.check_exists_or_new(robot_id, keypoints.keyframe_id):
                        color = self.pose_graph_viz.colors[robot_id % self.pose_graph_viz.nb_colors]
                        marker = Marker()
                        marker.header.frame_id = "robot" + str(self.pose_graph_viz.origin_robot_ids[robot_id]) + "_map"
                        marker.header.stamp = rclpy.time.Time().to_msg()
                        marker.ns = "keypoints_robot" + str(robot_id)
                        marker.id = keypoints.keyframe_id
                        marker.type = Marker.SPHERE_LIST
                        marker.action = Marker.ADD
                        marker.scale.x = 0.2
                        marker.scale.y = 0.2
                        marker.scale.z = 0.2
                        marker.color.r = color[0] * 0.6
                        marker.color.g = color[1] * 0.6
                        marker.color.b = color[2] * 0.6
                        marker.color.a = 1.0
                        marker.frame_locked = False
                        for kp in keypoints.data.points:
                            if keypoints.keyframe_id in self.pose_graph_viz.robot_pose_graphs[robot_id]: 
                                if not math.isnan(kp.x) and not math.isnan(kp.y) and not math.isnan(kp.z):                  
                                    # Offset by pose
                                    t = self.pose_to_transform(self.pose_graph_viz.robot_pose_graphs[robot_id][keypoints.keyframe_id].pose)
                                    a_kp = self.point_to_array(kp)
                                    proj = t.projection(a_kp)
                                    p = Point()
                                    p.x = proj[0]
                                    p.y = proj[1]
                                    p.z = proj[2]
                                    marker.points.append(p)
                        marker_array.markers.append(marker)
                        self.previous_poses = copy.deepcopy(self.pose_graph_viz.robot_pose_graphs)

        return marker_array

    def visualization_callback(self):
        marker_array = self.keypoints_to_marker_array()
        self.keypoints_markers_publisher.publish(marker_array)
        self.node.get_logger().info("Publish {} markers".format(len(marker_array.markers)))
