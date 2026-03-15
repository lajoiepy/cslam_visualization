import copy

import numpy as np
import open3d as o3d
import rerun as rr
import matplotlib
import matplotlib.colors

import zenoh
from rclpy.serialization import deserialize_message
from cslam_common_interfaces.msg import VizPointCloud
from sensor_msgs_py import point_cloud2
from distinctipy import distinctipy

norm = matplotlib.colors.Normalize(vmin=0.0, vmax=15.0)


class PointCloudVisualizer():

    def __init__(self, node, params, pose_graph_viz, zenoh_session):
        self.node = node
        self.params = params
        self.pose_graph_viz = pose_graph_viz
        self.visualizer_update_period_ms_ = self.params["visualization_update_period_ms"]
        self.use_real_colors = self.params.get("use_real_colors", False)

        self.pointclouds = {}
        self.timer = self.node.create_timer(
            self.visualizer_update_period_ms_ / 1000.0,
            self.visualization_callback)

        self.previous_poses = {}
        self.pointclouds_keys_published = set()

        self.viz_counter = 0.0

        def _pcl_cb(sample):
            try:
                msg = deserialize_message(bytes(sample.payload.to_bytes()), VizPointCloud)
                self.pointclouds_callback(msg)
            except Exception as e:
                self.node.get_logger().warn(f"PointCloudVisualizer: deserialize error: {e}")

        self._sub_pcl = zenoh_session.declare_subscriber(
            "cslam/*/viz/keyframe_pointcloud", _pcl_cb)

        colors = distinctipy.get_colors(self.params["nb_colors"], colorblind_type="Deuteranomaly")
        self.colormaps = {}
        for i in range(self.params["nb_colors"]):
            self.colormaps[i] = matplotlib.colors.LinearSegmentedColormap.from_list(
                "cmap" + str(i), [colors[i], 'white'], N=256)

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
        dist = np.linalg.norm([new.x - previous.x, new.y - previous.y, new.z - previous.z])
        return dist > 1e-1

    @staticmethod
    def _unpack_pcl_rgb(pts_struct):
        """Unpack PCL's packed-float32 RGB into an (N, 3) uint8 array (R, G, B)."""
        # PCL packs color as a float32 whose bytes are [B, G, R, 0] (little-endian).
        # .copy() is required before .view() because structured-array fields may be
        # non-contiguous, which makes .view() raise a ValueError.
        rgb_bytes = pts_struct["rgb"].copy().view(np.uint8).reshape(-1, 4)
        return rgb_bytes[:, [2, 1, 0]]  # → R, G, B

    def keyframe_pointcloud_to_pose_pointcloud(self):
        """Place keyframe point clouds at their optimised poses."""
        for robot_id, sensor_data in list(self.pointclouds.items()):
            if robot_id not in self.pose_graph_viz.robot_pose_graphs:
                continue

            # Iterate over a copy so we can safely remove processed items.
            for pcl in list(sensor_data):
                try:
                    if not self.check_exists_or_new(robot_id, pcl.keyframe_id):
                        continue
                    if pcl.keyframe_id not in self.pose_graph_viz.robot_pose_graphs[robot_id]:
                        continue

                    # Read XYZ (and RGB if available).
                    has_rgb = any(f.name == "rgb" for f in pcl.pointcloud.fields)
                    if has_rgb:
                        field_names = ["x", "y", "z", "rgb"]
                    else:
                        field_names = ["x", "y", "z"]
                    pts_struct = point_cloud2.read_points(
                        pcl.pointcloud, field_names=field_names, skip_nans=True)
                    xyz = np.column_stack(
                        [np.array(pts_struct["x"]), np.array(pts_struct["y"]), np.array(pts_struct["z"])]).astype(np.float64)

                    if xyz.shape[0] == 0:
                        sensor_data.remove(pcl)
                        self.pointclouds_keys_published.add((robot_id, pcl.keyframe_id))
                        continue

                    rr.set_time("stable_time", sequence=int(self.viz_counter))
                    path = ("global_map/robot_" + str(robot_id) +
                            "_map/poses/pose_" + str(pcl.keyframe_id))

                    # --- Depth-coloured view (robot colour + depth gradient) ---
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(xyz)
                    pcd = pcd.voxel_down_sample(voxel_size=self.params['voxel_size'])
                    pts_d = np.asarray(pcd.points)
                    cmap = self.colormaps[robot_id % self.params["nb_colors"]]
                    depth_colors = cmap(norm(pts_d[:, 2]))
                    rr.log(path + "/points", rr.Points3D(pts_d, colors=depth_colors))

                    # --- Real-colour view ---
                    if self.use_real_colors and has_rgb:
                        try:
                            real_rgb = self._unpack_pcl_rgb(pts_struct)  # (N, 3) uint8
                            pcd_c = o3d.geometry.PointCloud()
                            pcd_c.points = o3d.utility.Vector3dVector(xyz)
                            pcd_c.colors = o3d.utility.Vector3dVector(real_rgb / 255.0)
                            pcd_c = pcd_c.voxel_down_sample(voxel_size=self.params['voxel_size'])
                            pts_c = np.asarray(pcd_c.points)
                            colors_c = (np.asarray(pcd_c.colors) * 255).astype(np.uint8)
                            rr.log(path + "/points_rgb", rr.Points3D(pts_c, colors=colors_c))
                        except Exception as e:
                            self.node.get_logger().warn(
                                f"PointCloudVisualizer: RGB unpack failed for robot {robot_id} kf {pcl.keyframe_id}: {e}")

                    self.previous_poses = copy.deepcopy(self.pose_graph_viz.robot_pose_graphs)
                    sensor_data.remove(pcl)
                    self.pointclouds_keys_published.add((robot_id, pcl.keyframe_id))
                except Exception as e:
                    self.node.get_logger().warn(
                        f"PointCloudVisualizer: processing failed for robot {robot_id} kf {pcl.keyframe_id}: {e}")

    def visualization_callback(self):
        self.keyframe_pointcloud_to_pose_pointcloud()
        self.viz_counter += 1.0
