#!/usr/bin/env python3
"""Stage A calibration — fit the floor plane in base_footprint from one depth cloud.

Reports residual PITCH/ROLL and the floor z-offset so depth_cam_tilt_deg (URDF) can
be corrected. Requires the camera (/camera/depth/points) AND robot_state_publisher
(TF base_footprint<-camera_depth_optical_frame) running. The robot MUST sit on a
FLAT floor with clear floor ahead (no objects in the ~0.6-3 m forward FOV).

Run on the Pi:
  source /opt/ros/jazzy/setup.bash && python3 stage_a_floor_fit.py
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


def fit_floor(low, thr=0.03, iters=800, seed=0):
    """RANSAC near-horizontal floor plane sitting near z=0, then lstsq refine.
    The z0 gate rejects elevated tilted surfaces (tables/shelves). Returns a dict
    (pitch/roll/z0/rms/ninl/coef) or None if no floor plane found."""
    if len(low) < 120:
        return None
    rng = np.random.default_rng(seed)
    best_inl, best_cnt = None, 0
    for _ in range(iters):
        s = low[rng.choice(len(low), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nl = np.linalg.norm(n)
        if nl < 1e-6:
            continue
        n = n / nl
        if abs(n[2]) < 0.95:            # near-horizontal only
            continue
        d = -n.dot(s[0])
        if abs(-d / n[2]) > 0.15:       # plane height at origin ~floor (rejects tables)
            continue
        inl = np.abs(low @ n + d) < thr
        if int(inl.sum()) > best_cnt:
            best_cnt, best_inl = int(inl.sum()), inl
    if best_inl is None or best_cnt < 80:
        return None
    F = low[best_inl]
    A = np.column_stack([F[:, 0], F[:, 1], np.ones(len(F))])
    coef, *_ = np.linalg.lstsq(A, F[:, 2], rcond=None)
    a, b, c = coef
    rms = float(np.sqrt(np.mean((F[:, 2] - A @ coef) ** 2)))
    return dict(pitch=float(np.degrees(np.arctan(a))),
                roll=float(np.degrees(np.arctan(b))),
                z0=float(c), rms=rms, ninl=int(len(F)))


class FloorFit(Node):
    def __init__(self):
        super().__init__("stage_a_floor_fit")
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
        # --- scene debug (helps tell floor vs wall/object) ---
        print(f"[debug] N={len(P)}  x[{P[:,0].min():.2f},{P[:,0].max():.2f}]"
              f"  y[{P[:,1].min():.2f},{P[:,1].max():.2f}]"
              f"  z[{P[:,2].min():.2f},{P[:,2].max():.2f}]")
        print(f"[debug] floor-like |z|<0.05: {int(np.sum(np.abs(P[:,2])<0.05))}"
              f"   above-floor z>0.30 (wall/obj): {int(np.sum(P[:,2]>0.30))}"
              f"   total: {len(P)}")
        print(f"[debug] TF used T={np.round(T,3)}")
        r = np.hypot(P[:, 0], P[:, 1])
        # --- floor-height vs forward distance (central strip): the bias-vs-tilt tell ---
        # A real tilt error makes floor z rise LINEARLY from the nearest bin. Far-range
        # depth bias instead shows ~flat-near then rising only in the far bins.
        strip = P[(np.abs(P[:, 1]) < 0.4) & (P[:, 0] > 0.3)]
        print("[range-bin] floor z by forward distance (central |y|<0.4 strip):")
        edges = [0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.5, 3.0, 4.0]
        for lo_e, hi_e in zip(edges[:-1], edges[1:]):
            seg = strip[(strip[:, 0] >= lo_e) & (strip[:, 0] < hi_e)]
            fl = seg[seg[:, 2] < 0.15]      # floor candidates in this bin
            if len(fl) > 20:
                print(f"   x[{lo_e:.1f},{hi_e:.1f}) floor_n={len(fl):5d}  "
                      f"z_med={np.median(fl[:, 2]) * 100:+.1f}cm  "
                      f"z_p10={np.percentile(fl[:, 2], 10) * 100:+.1f}cm")
            else:
                print(f"   x[{lo_e:.1f},{hi_e:.1f}) floor_n={len(fl):5d}  (sparse)")

        # Fit NEAR and FAR bands separately. NEAR (accurate depth) drives the
        # correction; comparing to FAR exposes range-dependent depth bias.
        near = P[(P[:, 2] > -0.15) & (P[:, 2] < 0.20) & (r > 0.55) & (r < 1.6) & (P[:, 0] > 0.2)]
        far = P[(P[:, 2] > -0.15) & (P[:, 2] < 0.25) & (r >= 1.6) & (r < 2.6) & (P[:, 0] > 0.2)]
        print(f"[bands] near(0.55-1.6m) n={len(near)}   far(1.6-2.6m) n={len(far)}")
        fn = fit_floor(near)
        ff = fit_floor(far)
        if fn is None and ff is None:
            self.get_logger().warn(
                f"no floor plane (near n={len(near)}, far n={len(far)}). Place the robot "
                f"so PLAIN floor is visible ~0.6-1.5 m straight ahead (not a far-open room).")
            return

        print("\n==== STAGE A FLOOR FIT (base_footprint) ====")
        for name, f in (("NEAR 0.55-1.6m", fn), ("FAR  1.6-2.6m ", ff)):
            if f is None:
                print(f" [{name}] no floor plane")
            else:
                print(f" [{name}] inl={f['ninl']:5d}  pitch={f['pitch']:+.2f}  "
                      f"roll={f['roll']:+.2f}  z0={f['z0'] * 100:+.2f}cm  rms={f['rms'] * 100:.2f}cm")

        ref = fn if fn is not None else ff
        used = "NEAR" if fn is not None else "FAR (near unavailable)"
        print(f"\n using {used} band for correction:")
        print(f" residual PITCH (fwd)    : {ref['pitch']:+.2f} deg   (target |.| < 0.3)")
        print(f" residual ROLL  (side)   : {ref['roll']:+.2f} deg")
        print(f" floor z at origin (z0)  : {ref['z0'] * 100:+.2f} cm  (target |.| < 0.5)")
        print(f" current depth_cam_tilt_deg = {CUR_TILT}")
        print(f" => suggested new value   ~= {CUR_TILT - ref['pitch']:+.2f} deg "
              f"(adjust by {-ref['pitch']:+.2f})")
        if fn is not None and ff is not None and abs(fn['pitch'] - ff['pitch']) > 1.5:
            print(f" [!] near vs far pitch differ by {abs(fn['pitch'] - ff['pitch']):.1f} deg"
                  f" => far-range depth bias present; trust the NEAR band only.")
        print()
        self.done = True


def main():
    rclpy.init()
    n = FloorFit()
    start = time.time()
    while rclpy.ok() and not n.done and (time.time() - start) < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.done:
        n.get_logger().error(
            "no valid fit: no cloud/TF, or not enough floor points (placement?)")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
