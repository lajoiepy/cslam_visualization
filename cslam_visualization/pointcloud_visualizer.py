import rclpy
from rclpy.node import Node
import numpy as np

from cslam_common_interfaces.msg import KeyframePointCloud
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math
from cslam_visualization.utils.transform import Transform
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from distinctipy import distinctipy
import copy
from tf2_ros import TransformBroadcaster

class PointCloudVisualizer():

    def __init__(self, node, params, pose_graph_viz):
        self.node = node
        self.params = params
        self.pose_graph_viz = pose_graph_viz
        self.pointcloud_publisher = self.node.create_publisher(
            PointCloud2, "/viz/pose_pointcloud", 10)
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]  
        self.pointclouds_subscriber = self.node.create_subscription(
            KeyframePointCloud, '/viz/keyframe_pointcloud', self.pointclouds_callback, 10)
        self.pointclouds = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)

        self.previous_poses = {}
        self.tfs_to_publish = []
        self.pointclouds_to_publish = []
        self.tf_broadcaster = TransformBroadcaster(self.node)

    def pointclouds_callback(self, msg):
        if msg.robot_id not in self.pointclouds:
            self.pointclouds[msg.robot_id] = []
        self.pointclouds[msg.robot_id].append(msg)

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

    def keyframe_pointcloud_to_pose_pointcloud(self):
        """Offsets the pointclouds to the robot poses"""
        # Nodes (poses)
        for robot_id, sensor_data in self.pointclouds.items():
            if robot_id in self.pose_graph_viz.robot_pose_graphs:
                for pcl in sensor_data:
                    if self.check_exists_or_new(robot_id, pcl.keyframe_id):
                        if pcl.keyframe_id in self.pose_graph_viz.robot_pose_graphs[robot_id]:
                            tf_to_publish = TransformStamped()
                            tf_to_publish.header.frame_id = "robot" + str(self.pose_graph_viz.origin_robot_ids[robot_id]) + "_map"
                            tf_to_publish.header.stamp = rclpy.time.Time().to_msg()
                            tf_to_publish.child_frame_id = "robot" + str(robot_id) + "_keyframe" + str(pcl.keyframe_id)
                            t = self.pose_to_transform(self.pose_graph_viz.robot_pose_graphs[robot_id][pcl.keyframe_id].pose)
                            tf_to_publish.transform = t.to_msg()

                            pcl.pointcloud.header.stamp = tf_to_publish.header.stamp
                            pcl.pointcloud.header.frame_id = tf_to_publish.child_frame_id

                            self.pointclouds_to_publish.append(pcl.pointcloud)
                            self.tfs_to_publish.append(tf_to_publish)
                            self.previous_poses = copy.deepcopy(self.pose_graph_viz.robot_pose_graphs)

    def visualization_callback(self):
        self.keyframe_pointcloud_to_pose_pointcloud()
        self.node.get_logger().info("Publishing " + str(len(self.pointclouds_to_publish)) + " pointclouds")
        for pcl in self.pointclouds_to_publish:
            self.pointcloud_publisher.publish(pcl)
        for tf in self.tfs_to_publish:
            self.tf_broadcaster.sendTransform(tf)
        
