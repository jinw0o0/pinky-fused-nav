#!/usr/bin/env python3
"""Stage A (WALL method) — measure the depth-camera PITCH error from a flat wall.

Robust where the floor method fails: a wall returns IR strongly (near-perpendicular
incidence), while the up-tilted floor returns little (grazing) and falls below the
FOV. A real vertical wall, transformed into base_footprint, must have a HORIZONTAL
normal (nz=0). Any nz means the cloud is mis-rotated => that elevation angle IS the
camera pitch error.

Place the robot SQUARE-ON to a flat wall ~1.0-1.8 m away, nothing else in the FOV.
Run on the Pi (camera + robot_state_publisher must be up):
  source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/setup.bash
  python3 stage_a_wall_pitch.py
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

TARGET = "base_footprint"
CUR_TILT = -4.9  # current depth_cam_tilt_deg in URDF (wall-calibrated 2026-06-20; negative = up)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class WallPitch(Node):
    def __init__(self):
        super().__init__("stage_a_wall_pitch")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.sub = self.create_subscription(
            PointCloud2, "/camera/depth/points", self.cb, 10)
        self.done = False
        self.frames = 0

    def cb(self, msg):
        if self.done:
            return
        self.frames += 1
        try:
            tf = self.buf.lookup_transform(TARGET, msg.header.frame_id, rclpy.time.Time())
        except Exception as e:  # noqa
            if self.frames % 15 == 0:
                self.get_logger().warn(f"waiting TF {TARGET}<-{msg.header.frame_id}: {e}")
            return
        t, q = tf.transform.translation, tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        raw = np.asarray(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))
        if raw.size == 0:
            return
        if raw.dtype.names:
            pts = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float64)
        else:
            pts = raw.reshape(-1, 3).astype(np.float64)
        P = (R @ pts.T).T + T  # -> base_footprint

        print(f"[debug] N={len(P)}  x[{P[:,0].min():.2f},{P[:,0].max():.2f}]"
              f"  z[{P[:,2].min():.2f},{P[:,2].max():.2f}]")
        # candidate wall band: in front, off the floor, not the ceiling
        Q = P[(P[:, 0] > 0.3) & (P[:, 0] < 3.0) & (np.abs(P[:, 1]) < 1.2)
              & (P[:, 2] > 0.05) & (P[:, 2] < 1.4)]
        if len(Q) < 500:
            self.get_logger().warn(f"only {len(Q)} wall-band points -> face a flat wall ~1.2 m")
            return

        # RANSAC a near-VERTICAL plane (the wall): normal nearly horizontal -> |nz| small.
        rng = np.random.default_rng(0)
        best_inl, best_cnt = None, 0
        for _ in range(800):
            s = Q[rng.choice(len(Q), 3, replace=False)]
            n = np.cross(s[1] - s[0], s[2] - s[0])
            nl = np.linalg.norm(n)
            if nl < 1e-6:
                continue
            n = n / nl
            if abs(n[2]) > 0.5:          # want vertical-ish wall, reject floor/ceiling
                continue
            d = -n.dot(s[0])
            inl = np.abs(Q @ n + d) < 0.02
            c = int(inl.sum())
            if c > best_cnt:
                best_cnt, best_inl = c, inl
        if best_inl is None or best_cnt < 400:
            self.get_logger().warn(
                f"no vertical wall plane found ({best_cnt} inliers). Face a flat wall square-on.")
            return

        # Refine the normal by PCA on inliers (smallest-variance axis = plane normal).
        W = Q[best_inl]
        c0 = W.mean(axis=0)
        _, _, Vt = np.linalg.svd(W - c0, full_matrices=False)
        n = Vt[-1]
        if n[0] > 0:                      # orient normal to point back toward the robot
            n = -n
        # elevation of the wall normal above horizontal == camera pitch error
        pitch_err = float(np.degrees(np.arcsin(np.clip(n[2], -1, 1))))
        yaw_err = float(np.degrees(np.arctan2(n[1], -n[0])))
        rms = float(np.sqrt(np.mean((W @ n - n.dot(c0)) ** 2)))

        print("\n==== STAGE A WALL PITCH (base_footprint) ====")
        print(f" wall inliers            : {len(W)} of {len(Q)} band pts")
        print(f" wall mean distance      : {c0[0]:.2f} m   (z span {W[:,2].min():.2f}..{W[:,2].max():.2f})")
        print(f" wall normal (base)      : [{n[0]:+.3f} {n[1]:+.3f} {n[2]:+.3f}]")
        print(f" plane fit RMS           : {rms*100:.2f} cm")
        print(f" PITCH error (normal elev): {pitch_err:+.2f} deg   (target |.| < 0.3)")
        print(f" YAW error (off head-on)  : {yaw_err:+.2f} deg   (just keep it small while aiming)")
        # nz>0 => wall normal tilts UP => cloud under-rotated => camera tilted MORE up
        # than modeled => make depth_cam_tilt_deg MORE negative by pitch_err.
        print(f"\n current depth_cam_tilt_deg = {CUR_TILT}")
        print(f" => suggested new value   ~= {CUR_TILT - pitch_err:+.2f} deg "
              f"(adjust by {-pitch_err:+.2f})")
        print(" (nz>0 = wall leans back at top = camera tilted up MORE than -8 deg)\n")
        self.done = True


def main():
    rclpy.init()
    n = WallPitch()
    start = time.time()
    while rclpy.ok() and not n.done and (time.time() - start) < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.done:
        n.get_logger().error("no wall fit: check cloud/TF, and face a flat wall square-on ~1.2 m")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
