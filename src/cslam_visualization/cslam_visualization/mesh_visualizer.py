import numpy as np
import open3d as o3d
import rerun as rr

from cslam_common_interfaces.msg import VizPointCloud
from sensor_msgs_py import point_cloud2
from distinctipy import distinctipy


def _quat_to_rot(q):
    """geometry_msgs Quaternion (x, y, z, w) → 3×3 rotation matrix."""
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)    ],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z),  2*(y*z - x*w)    ],
        [2*(x*z - y*w),     2*(y*z + x*w),       1 - 2*(x*x + y*y)],
    ])


class MeshVisualizer():
    """Reconstructs and visualises a surface mesh from accumulated RGB-D keyframes.

    Point clouds are stored in sensor frame.  On each visualisation tick the
    current (post-PGO) keyframe poses are used to transform them into the world
    frame before running Open3D Poisson surface reconstruction.  The mesh is
    logged to Rerun at ``global_map/mesh``.
    """

    # Minimum number of points required to attempt Poisson reconstruction.
    _MIN_POINTS = 500

    def __init__(self, node, params, pose_graph_viz):
        self.node = node
        self.params = params
        self.pose_graph_viz = pose_graph_viz

        update_period_ms = self.params["visualization_update_period_ms"]
        self.voxel_size = self.params.get("mesh_voxel_size",
                                          self.params.get("voxel_size", 0.05))
        self.poisson_depth = int(self.params.get("mesh_poisson_depth", 8))
        self.use_real_colors = self.params.get("use_real_colors", False)

        nb_colors = self.params["nb_colors"]
        robot_colors_f = distinctipy.get_colors(nb_colors, colorblind_type="Deuteranomaly")
        # Pre-convert to (3,) float64 arrays in [0, 1].
        self.robot_colors = {i: np.array(robot_colors_f[i % nb_colors]) for i in range(nb_colors)}

        # Raw sensor-frame point clouds: {(robot_id, kf_id): (xyz, rgb_or_None)}
        self._sensor_clouds: dict = {}
        # Set of keys whose pose has changed enough to warrant a mesh rebuild.
        self._needs_rebuild = False

        self._sub = node.create_subscription(
            VizPointCloud,
            '/cslam/viz/keyframe_pointcloud',
            self._pointcloud_callback,
            10)

        self._timer = node.create_timer(
            update_period_ms / 1000.0,
            self._visualization_callback)

        self._viz_counter = 0

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------

    def _pointcloud_callback(self, msg):
        key = (msg.robot_id, msg.keyframe_id)
        has_rgb = any(f.name == "rgb" for f in msg.pointcloud.fields)
        field_names = ["x", "y", "z", "rgb"] if has_rgb else ["x", "y", "z"]
        pts = point_cloud2.read_points(
            msg.pointcloud, field_names=field_names, skip_nans=True)

        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        if xyz.shape[0] == 0:
            return

        rgb = None
        if has_rgb and self.use_real_colors:
            try:
                # PCL packs color as float32 with bytes [B, G, R, 0] (little-endian).
                rgb_bytes = pts["rgb"].copy().view(np.uint8).reshape(-1, 4)
                rgb = rgb_bytes[:, [2, 1, 0]].astype(np.float64) / 255.0  # R, G, B ∈ [0, 1]
            except Exception:
                rgb = None

        self._sensor_clouds[key] = (xyz, rgb)
        self._needs_rebuild = True

    # ------------------------------------------------------------------
    # World-frame assembly and reconstruction
    # ------------------------------------------------------------------

    def _build_global_cloud(self):
        """Transform all sensor-frame clouds to world frame and merge."""
        all_xyz = []
        all_rgb = []

        for (robot_id, kf_id), (xyz_s, rgb_s) in self._sensor_clouds.items():
            pg = self.pose_graph_viz.robot_pose_graphs
            if robot_id not in pg or kf_id not in pg[robot_id]:
                continue

            pose = pg[robot_id][kf_id].pose
            R = _quat_to_rot(pose.orientation)
            t = np.array([pose.position.x, pose.position.y, pose.position.z])
            xyz_w = (R @ xyz_s.T).T + t
            all_xyz.append(xyz_w)

            if rgb_s is not None and rgb_s.shape[0] == xyz_s.shape[0]:
                all_rgb.append(rgb_s)
            else:
                # Fall back to robot colour.
                color = self.robot_colors[robot_id % len(self.robot_colors)]
                all_rgb.append(np.tile(color, (xyz_s.shape[0], 1)))

        if not all_xyz:
            return None

        xyz = np.concatenate(all_xyz, axis=0)
        rgb = np.concatenate(all_rgb, axis=0)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(np.clip(rgb, 0, 1))
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        return pcd

    def _reconstruct_and_log(self):
        pcd = self._build_global_cloud()
        if pcd is None or len(pcd.points) < self._MIN_POINTS:
            return

        try:
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.voxel_size * 3, max_nn=30))
            pcd.orient_normals_consistent_tangent_plane(30)

            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                pcd, depth=self.poisson_depth)

            # Remove low-density vertices (spurious surfaces outside the scan volume).
            densities_np = np.asarray(densities)
            threshold = np.quantile(densities_np, 0.05)
            vertices_to_remove = densities_np < threshold
            mesh.remove_vertices_by_mask(vertices_to_remove)
            mesh.compute_vertex_normals()

            vertices = np.asarray(mesh.vertices)
            triangles = np.asarray(mesh.triangles)
            normals = np.asarray(mesh.vertex_normals)
            colors_np = np.asarray(mesh.vertex_colors)

            if vertices.shape[0] == 0 or triangles.shape[0] == 0:
                return

            rr.set_time("stable_time", sequence=self._viz_counter)
            rr.log(
                "global_map/mesh",
                rr.Mesh3D(
                    vertex_positions=vertices.astype(np.float32),
                    triangle_indices=triangles.astype(np.uint32),
                    vertex_normals=normals.astype(np.float32),
                    vertex_colors=(colors_np * 255).astype(np.uint8)
                    if colors_np.shape[0] == vertices.shape[0]
                    else None,
                ))
        except Exception as e:
            self.node.get_logger().warn(f"MeshVisualizer: reconstruction failed: {e}")

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _visualization_callback(self):
        if not self._needs_rebuild:
            return
        if not self.pose_graph_viz.robot_pose_graphs:
            return
        self._needs_rebuild = False
        self._reconstruct_and_log()
        self._viz_counter += 1
