#!/usr/bin/env python3
"""Phase 1a gyro check — integrate /imu_raw angular_velocity.z over a manual rotation.

Verifies the deg/s -> rad/s fix in pinky_imu_bno055/src/main_node.cpp. Rotate the robot
EXACTLY 360 deg by hand (one smooth turn) anytime during the window, then stop. The
integrated yaw should read ~+/-6.28 rad. If it reads ~+/-360, the deg/s bug remains.
Peak |wz| for a hand turn should be a few rad/s (NOT tens), another tell.

  python3 phase1a_gyro_check.py
"""
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

TIMEOUT = 60.0       # overall wait for a rotation
START_THR = 0.30     # rad/s -> rotation has begun (arm + integrate)
STOP_THR = 0.05      # rad/s -> considered still
STOP_HOLD = 1.5      # s of stillness after moving -> finalize


class Gyro(Node):
    def __init__(self):
        super().__init__("phase1a_gyro_check")
        self.sub = self.create_subscription(Imu, "/imu_raw", self.cb, 50)
        self.sum = 0.0
        self.last = None
        self.peak = 0.0
        self.armed = False
        self.still_since = None
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z
        self.peak = max(self.peak, abs(wz))
        dt = (t - self.last) if self.last is not None else 0.0
        self.last = t

        if not self.armed:
            if abs(wz) > START_THR:
                self.armed = True
                print(f"  >> rotation detected (wz={wz:+.2f} rad/s), integrating...")
            return
        if 0.0 < dt < 0.5:
            self.sum += wz * dt
        # stop detection
        if abs(wz) < STOP_THR:
            if self.still_since is None:
                self.still_since = t
            elif t - self.still_since > STOP_HOLD:
                self.done = True
        else:
            self.still_since = None
            print(f"  ... cum={self.sum:+.3f} rad ({math.degrees(self.sum):+6.1f} deg)  "
                  f"peak|wz|={self.peak:.2f} rad/s")


def main():
    rclpy.init()
    n = Gyro()
    print("Auto-detect armed. ROTATE the robot one smooth full turn (360 deg), then stop.")
    t0 = time.time()
    while rclpy.ok() and not n.done and (time.time() - t0) < TIMEOUT:
        rclpy.spin_once(n, timeout_sec=0.1)
    print("\n==== PHASE 1a GYRO RESULT ====")
    if not n.armed:
        print(" no rotation detected within timeout (did the robot turn?).")
    print(f" integrated yaw = {n.sum:+.3f} rad  ({math.degrees(n.sum):+.1f} deg)")
    print(f" peak |wz|      = {n.peak:.2f} rad/s")
    print(" PASS if ~+/-6.28 rad for a 360 turn (deg/s bug would give ~+/-360).")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
