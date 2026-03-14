"""Saves C-SLAM results to disk.

Periodically writes to `save_dir`:
  trajectory_robot{i}.tum   Estimated trajectory in TUM format.
  trajectory_comparison.png  Side-by-side estimated vs GT top-down plot.
  map.ply                    Accumulated coloured 3-D map in world frame.

Parameters (ROS):
  save_dir        str         Output directory.  Default: /tmp/cslam_results
  max_nb_robots   int         Number of robots.  Default: 3
  gt_paths        str[]       TUM ground-truth file per robot (empty = skip).
  save_period_sec float       Seconds between automatic saves.  Default: 30.0
"""

import os
import copy

import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors

from sensor_msgs_py import point_cloud2
from cslam_common_interfaces.msg import KeyframeOdom, VizPointCloud


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _quat_to_rot(qx, qy, qz, qw):
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


def _pose_to_matrix(pose):
    T = np.eye(4)
    T[:3, :3] = _quat_to_rot(pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w)
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def _load_tum(path):
    entries = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split()
            if len(p) != 8:
                continue
            T = np.eye(4)
            T[:3, :3] = _quat_to_rot(float(p[4]), float(p[5]),
                                      float(p[6]), float(p[7]))
            T[:3, 3] = [float(p[1]), float(p[2]), float(p[3])]
            entries.append((float(p[0]), T))
    entries.sort(key=lambda e: e[0])
    return entries


def _nearest_gt(entries, ts):
    lo, hi = 0, len(entries) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if entries[mid][0] < ts:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(entries[lo-1][0] - ts) < abs(entries[lo][0] - ts):
        lo -= 1
    return entries[lo][1]


def _umeyama(src, dst):
    """Rigid Umeyama: dst ≈ R @ src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    U, _, Vt = np.linalg.svd(sc.T @ dc)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1., 1., d]) @ U.T
    return R, mu_d - R @ mu_s


def _unpack_pcl_rgb(pts_struct):
    """PCL packed float32 RGB → (N, 3) uint8 [R, G, B]."""
    rgb_bytes = pts_struct["rgb"].copy().view(np.uint8).reshape(-1, 4)
    return rgb_bytes[:, [2, 1, 0]]


# ---------------------------------------------------------------------------
# ResultSaver
# ---------------------------------------------------------------------------

class ResultSaver:

    def __init__(self, node, params, pose_graph_viz):
        self.node = node
        self.params = params
        self.pose_graph_viz = pose_graph_viz

        self.max_nb_robots = params.get("max_nb_robots", 3)
        self.save_dir = params.get("save_dir", "/tmp/cslam_results")
        self.gt_paths = params.get("gt_paths", [])
        os.makedirs(self.save_dir, exist_ok=True)

        # keyframe_id → ROS timestamp (sec) per robot
        self.kf_timestamps = {i: {} for i in range(self.max_nb_robots)}

        # (robot_id, keyframe_id) → (xyz Nx3 float32, colors Nx3 uint8 or None)
        self._cloud_cache = {}

        for i in range(self.max_nb_robots):
            self.node.create_subscription(
                KeyframeOdom,
                f"/r{i}/cslam/keyframe_odom",
                self._make_kf_odom_cb(i), 10)

        self.node.create_subscription(
            VizPointCloud,
            "/cslam/viz/keyframe_pointcloud",
            self._pcl_cb, 10)

        save_period = params.get("save_period_sec", 30.0)
        self.node.create_timer(save_period, self._save_cb)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _make_kf_odom_cb(self, robot_id):
        def cb(msg):
            stamp = msg.odom.header.stamp
            self.kf_timestamps[robot_id][msg.id] = stamp.sec + stamp.nanosec * 1e-9
        return cb

    def _pcl_cb(self, msg):
        try:
            has_rgb = any(f.name == "rgb" for f in msg.pointcloud.fields)
            self.node.get_logger().debug(
                f"ResultSaver: received pointcloud robot={msg.robot_id} kf={msg.keyframe_id} has_rgb={has_rgb}")
            if has_rgb:
                pts_s = point_cloud2.read_points(
                    msg.pointcloud, field_names=["x", "y", "z", "rgb"],
                    skip_nans=True)
                xyz = np.column_stack(
                    [np.array(pts_s["x"]), np.array(pts_s["y"]), np.array(pts_s["z"])]).astype(np.float32)
                colors = _unpack_pcl_rgb(pts_s)
            else:
                pts_s = point_cloud2.read_points(
                    msg.pointcloud, field_names=["x", "y", "z"],
                    skip_nans=True)
                xyz = np.column_stack(
                    [np.array(pts_s["x"]), np.array(pts_s["y"]), np.array(pts_s["z"])]).astype(np.float32)
                colors = None
            self._cloud_cache[(msg.robot_id, msg.keyframe_id)] = (xyz, colors)
        except Exception as e:
            self.node.get_logger().warn(f"ResultSaver: pointcloud callback failed: {e}")

    def _save_cb(self):
        nb_robots = len(self.pose_graph_viz.robot_pose_graphs)
        nb_clouds = len(self._cloud_cache)
        self.node.get_logger().info(
            f"ResultSaver: save timer fired. robots={nb_robots} cached_clouds={nb_clouds}")
        try:
            self.save_all()
        except Exception as e:
            import traceback
            self.node.get_logger().warn(
                f"ResultSaver: save failed: {e}\n{traceback.format_exc()}")

    # ── Public API ───────────────────────────────────────────────────────────

    def save_all(self):
        self.save_trajectories_tum()
        self.save_gt_comparison_figure()
        self.save_3d_map_ply()
        self.node.get_logger().info(
            f"ResultSaver: results saved to {self.save_dir}")

    # ── Trajectories ─────────────────────────────────────────────────────────

    def save_trajectories_tum(self):
        for robot_id, pg in self.pose_graph_viz.robot_pose_graphs.items():
            path = os.path.join(self.save_dir, f"trajectory_robot{robot_id}.tum")
            ts_map = self.kf_timestamps.get(robot_id, {})
            with open(path, "w") as f:
                f.write("# timestamp tx ty tz qx qy qz qw\n")
                for kf_id, node in sorted(pg.items()):
                    ts = ts_map.get(kf_id, float(kf_id))
                    p = node.pose.position
                    q = node.pose.orientation
                    f.write(f"{ts:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                            f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")

    # ── Figure ───────────────────────────────────────────────────────────────

    def save_gt_comparison_figure(self):
        robot_ids = sorted(self.pose_graph_viz.robot_pose_graphs.keys())
        if not robot_ids:
            return

        n = len(robot_ids)
        fig, axes = plt.subplots(1, n, figsize=(6 * n, 5), squeeze=False)
        est_palette = ["tab:blue", "tab:orange", "tab:green",
                       "tab:red", "tab:purple", "tab:brown"]
        gt_palette  = ["steelblue", "sandybrown", "mediumseagreen",
                       "salmon", "mediumpurple", "saddlebrown"]

        for col, robot_id in enumerate(robot_ids):
            ax = axes[0][col]
            pg = self.pose_graph_viz.robot_pose_graphs[robot_id]
            ts_map = self.kf_timestamps.get(robot_id, {})

            est_pts = np.array([
                [n.pose.position.x, n.pose.position.y, n.pose.position.z]
                for _, n in sorted(pg.items())
            ])
            if est_pts.size == 0:
                continue

            c_est = est_palette[robot_id % len(est_palette)]
            c_gt  = gt_palette[robot_id % len(gt_palette)]

            # Top-down view: X (right) vs Z (forward) in optical frame.
            ax.plot(est_pts[:, 0], est_pts[:, 2],
                    color=c_est, linewidth=1.5, label="Estimated")
            ax.plot(est_pts[0, 0], est_pts[0, 2], "o",
                    color=c_est, markersize=7)
            ax.plot(est_pts[-1, 0], est_pts[-1, 2], "s",
                    color=c_est, markersize=7)

            # Ground truth
            gt_path = (self.gt_paths[robot_id]
                       if robot_id < len(self.gt_paths) else "")
            if gt_path and os.path.isfile(gt_path) and ts_map:
                gt_entries = _load_tum(gt_path)
                gt_raw = []
                est_for_align = []
                for kf_id, node in sorted(pg.items()):
                    ts = ts_map.get(kf_id)
                    if ts is None:
                        continue
                    T_gt = _nearest_gt(gt_entries, ts)
                    gt_raw.append(T_gt[:3, 3])
                    est_for_align.append(
                        [node.pose.position.x,
                         node.pose.position.y,
                         node.pose.position.z])

                if len(gt_raw) >= 3:
                    gt_pts = np.array(gt_raw)
                    est_align = np.array(est_for_align)
                    # Align GT into estimated frame (Umeyama).
                    R, t = _umeyama(gt_pts, est_align)
                    gt_aligned = (R @ gt_pts.T).T + t
                    ax.plot(gt_aligned[:, 0], gt_aligned[:, 2],
                            "--", color=c_gt, linewidth=1.5,
                            label="GT (aligned)")
                    ax.plot(gt_aligned[0, 0], gt_aligned[0, 2], "o",
                            color=c_gt, markersize=7)

            ax.set_title(f"Robot {robot_id}")
            ax.set_xlabel("X  (m)")
            ax.set_ylabel("Z  (m)  [forward]")
            ax.set_aspect("equal")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        fig.suptitle("Estimated vs Ground Truth — top-down (X–Z, optical frame)")
        fig.tight_layout()
        out = os.path.join(self.save_dir, "trajectory_comparison.png")
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)

    # ── 3-D map ──────────────────────────────────────────────────────────────

    def save_3d_map_ply(self):
        combined = o3d.geometry.PointCloud()

        for (robot_id, kf_id), (pts_cam, colors_cam) in self._cloud_cache.items():
            pg = self.pose_graph_viz.robot_pose_graphs.get(robot_id, {})
            node = pg.get(kf_id)
            if node is None or pts_cam.shape[0] == 0:
                continue

            T = _pose_to_matrix(node.pose)
            pts_world = (T[:3, :3] @ pts_cam.T).T + T[:3, 3]

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts_world.astype(np.float64))
            if colors_cam is not None:
                pcd.colors = o3d.utility.Vector3dVector(
                    colors_cam.astype(np.float64) / 255.0)
            combined += pcd

        if len(combined.points) == 0:
            return

        voxel = self.params.get("voxel_size", 0.05)
        combined = combined.voxel_down_sample(voxel)
        out = os.path.join(self.save_dir, "map.ply")
        o3d.io.write_point_cloud(out, combined)
