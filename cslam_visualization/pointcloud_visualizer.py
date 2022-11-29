import rclpy
from rclpy.node import Node
import numpy as np

from cslam_common_interfaces.msg import VizPointCloud
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
from cslam_visualization.utils.transform import Transform
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from distinctipy import distinctipy
import copy
from tf2_ros import TransformBroadcaster
from cslam.utils.point_cloud2 import read_points
from struct import pack, unpack
import open3d

class PointCloudVisualizer():

    def __init__(self, node, params, pose_graph_viz):
        self.node = node
        self.params = params
        self.pose_graph_viz = pose_graph_viz
        self.markers_publisher = self.node.create_publisher(
            Marker, "/viz/cloudmarker", 10)
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]  
        self.pointclouds_subscriber = self.node.create_subscription(
            VizPointCloud, '/viz/keyframe_pointcloud', self.pointclouds_callback, 10)
        self.pointclouds = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)

        self.previous_poses = {}
        self.tfs_to_publish = []
        self.markers_to_publish = []
        self.pointclouds_keys_published = set()
        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.rotation_to_sensor_frame = Transform(quat=[1, 0, 0, 0], pos=[0, 0, 0])
        if self.params["rotation_to_sensor_frame"] is not None:
            self.rotation_to_sensor_frame = Transform(quat=self.params["rotation_to_sensor_frame"], pos=self.rotation_to_sensor_frame.position())
            self.node.get_logger().info("rotation_to_sensor_frame: {} ".format(self.params["rotation_to_sensor_frame"]))


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

    def rgb_value_to_color(self, rgb):
        b = pack('f', rgb)
        val = unpack('i', b)[0]
        color = ColorRGBA()
        color.r = float((val >> 16) & 0x0000ff) / 255
        color.g = float((val >> 8)  & 0x0000ff) / 255
        color.b = float((val)       & 0x0000ff) / 255
        color.a = 1.0
        return color

    def get_robot_color(self, robot_id):
        rgb = self.pose_graph_viz.colors[robot_id % self.pose_graph_viz.nb_colors]
        color = ColorRGBA()
        color.r = rgb[0] * 0.6
        color.g = rgb[1] * 0.6
        color.b = rgb[2] * 0.6
        color.a = 1.0
        return color

    def pointcloud_to_marker(self, robot_id, keyframe_id, pointcloud):
        """Converts a pointcloud to a marker"""
        marker = Marker()
        marker.header.frame_id = pointcloud.header.frame_id
        marker.header.stamp = pointcloud.header.stamp
        marker.type = Marker.CUBE_LIST #Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = self.params["voxel_size"]
        marker.scale.y = self.params["voxel_size"]
        marker.scale.z = self.params["voxel_size"]

        for point in read_points(pointcloud, skip_nans=True):
            pt = Point()
            pt.x = float(point[0])
            pt.y = float(point[1])
            pt.z = float(point[2])
            marker.points.append(pt)
            if len(point) == 4:
                marker.colors.append(self.rgb_value_to_color(point[3]))
            else:
                marker.colors.append(self.get_robot_color(robot_id))
        marker.frame_locked = True
        marker.ns = "keypoints_robot" + str(robot_id)
        marker.id = keyframe_id
        return marker                      


    def check_exists_or_new(self, robot_id, keyframe_id):
        if robot_id not in self.previous_poses:
            return True
        if keyframe_id not in self.previous_poses[robot_id]:
            return True
        if ("keypoints_robot" + str(robot_id), keyframe_id) not in self.pointclouds_keys_published:
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
                            t = t * self.rotation_to_sensor_frame
                            tf_to_publish.transform = t.to_msg()

                            pcl.pointcloud.header.stamp = tf_to_publish.header.stamp
                            pcl.pointcloud.header.frame_id = tf_to_publish.child_frame_id
                            marker = self.pointcloud_to_marker(robot_id, pcl.keyframe_id, pcl.pointcloud)

                            self.markers_to_publish.append(marker)
                            self.tfs_to_publish.append(tf_to_publish)
                            self.previous_poses = copy.deepcopy(self.pose_graph_viz.robot_pose_graphs)
                            self.pointclouds[robot_id].remove(pcl)

    def visualization_callback(self):
        self.keyframe_pointcloud_to_pose_pointcloud()
        self.node.get_logger().info("Publishing " + str(len(self.markers_to_publish)) + " pointclouds")
        for pc in self.markers_to_publish:
            self.pointclouds_keys_published.add((pc.ns, pc.id))
            self.markers_publisher.publish(pc)
        for tf in self.tfs_to_publish:
            self.tf_broadcaster.sendTransform(tf)
        self.markers_to_publish = []
        
