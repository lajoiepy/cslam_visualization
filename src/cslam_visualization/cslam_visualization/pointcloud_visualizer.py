import rclpy
from rclpy.node import Node
import numpy as np

from cslam_common_interfaces.msg import VizPointCloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from distinctipy import distinctipy
import copy
import open3d as o3d
import rerun as rr
from numpy.lib.recfunctions import structured_to_unstructured
import matplotlib
import time

# currently need to calculate the color manually
# see https://github.com/rerun-io/rerun/issues/4409
# TODO: add option
#cmap = matplotlib.colormaps["viridis"]
norm = matplotlib.colors.Normalize(
    vmin=-5.0,
    vmax=20.0,
)

class PointCloudVisualizer():

    def __init__(self, node, params, pose_graph_viz):
        self.node = node
        self.params = params
        self.pose_graph_viz = pose_graph_viz
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]  
        self.pointclouds_subscriber = self.node.create_subscription(
            VizPointCloud, '/cslam/viz/keyframe_pointcloud', self.pointclouds_callback, 10)
        self.pointclouds = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)

        self.previous_poses = {}
        self.pointclouds_keys_published = set()

        self.viz_counter = 0.0 # TODO: To get correct logging timestamps, use timestamp from PoseGraph message instead of counter

        colors = distinctipy.get_colors(self.params["nb_colors"], colorblind_type="Deuteranomaly")
        self.colormaps = {}
        for i in range(self.params["nb_colors"]):
            self.colormaps[i] = matplotlib.colors.LinearSegmentedColormap.from_list("cmap"+str(i), [colors[i], 'white'], N=256)


    def pointclouds_callback(self, msg):
        if msg.robot_id not in self.pointclouds:
            self.pointclouds[msg.robot_id] = []
        self.pointclouds[msg.robot_id].append(msg)      

    def check_exists_or_new(self, robot_id, keyframe_id):
        if robot_id not in self.previous_poses:
            return True
        if keyframe_id not in self.previous_poses[robot_id]:
            return True
        if (robot_id, keyframe_id) not in self.pointclouds_keys_published:
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
            if robot_id not in self.pose_graph_viz.robot_pose_graphs:
                continue
            for pcl in sensor_data:
                if not self.check_exists_or_new(robot_id, pcl.keyframe_id):
                    continue
                if pcl.keyframe_id not in self.pose_graph_viz.robot_pose_graphs[robot_id]:  
                    continue                      
                
                pts = point_cloud2.read_points(pcl.pointcloud, field_names=["x", "y", "z"], skip_nans=True)
                pts = structured_to_unstructured(pts)
                
                rr.set_time_seconds("stable_time", self.viz_counter)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(pts)
                pcd = pcd.voxel_down_sample(voxel_size=self.params['voxel_size'])
                pts = np.asarray(pcd.points)
                cmap = self.colormaps[robot_id % self.params["nb_colors"]]
                pts_colors = cmap(norm(pts[:, 2]))
                
                rr.log("global_map/robot_" + str(robot_id) + "_map/poses/pose_" + str(pcl.keyframe_id) + "/points", rr.Points3D(pts, colors=pts_colors)) 

                self.previous_poses = copy.deepcopy(self.pose_graph_viz.robot_pose_graphs)
                self.pointclouds[robot_id].remove(pcl)
                self.pointclouds_keys_published.add((robot_id, pcl.keyframe_id))

    def visualization_callback(self):
        self.keyframe_pointcloud_to_pose_pointcloud()
        self.viz_counter += 1.0
