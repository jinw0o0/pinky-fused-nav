#!/usr/bin/env python3
"""Phase 1a watchdog check — motors must stop within ~0.5 s after cmd_vel stops.

Commands a gentle IN-PLACE spin for HOLD s, then STOPS publishing /cmd_vel and measures
how long until the wheels (/joint_states velocity) return to ~0. The cmd_vel watchdog in
bringup.py zeroes the motors after cmd_vel_timeout (0.5 s). PASS = wheels stop ~0.5-0.7 s
after the last command; FAIL = wheels keep spinning (watchdog not working).

SAFETY: hold the robot with its WHEELS OFF the ground (or on clear floor; it spins in
place, does not drive forward).

  python3 phase1a_watchdog_check.py
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

W = 1.0           # angular.z [rad/s], gentle in-place spin
HOLD = 2.0        # [s] command motion
WATCH = 3.0       # [s] observe after stopping cmd_vel
STOP_VEL = 0.10   # [rad/s] wheel speed considered stopped
WHEELS = ("l_wheel_joint", "r_wheel_joint")


class Watchdog(Node):
    def __init__(self):
        super().__init__("phase1a_watchdog_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 50)
        self.wheel_vel = 0.0
        self.samples = []  # (t, max|wheel vel|)

    def js_cb(self, msg):
        # store wheel POSITIONS; we derive speed from position deltas ourselves
        # (the measured velocity field reads ~0 on this driver even while spinning).
        pos = {}
        for nm, p in zip(msg.name, msg.position):
            if nm in WHEELS:
                pos[nm] = p
        if pos:
            self.samples.append((time.time(),
                                 pos.get("l_wheel_joint", 0.0),
                                 pos.get("r_wheel_joint", 0.0)))


def main():
    rclpy.init()
    n = Watchdog()

    # wait for bringup to match our publisher AND for joint_states to flow to us
    tw0 = time.time()
    while (n.pub.get_subscription_count() < 1 or len(n.samples) < 5) and time.time() - tw0 < 6.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f"cmd_vel subscribers matched = {n.pub.get_subscription_count()}, "
          f"joint_states samples = {len(n.samples)}")

    print(f"commanding in-place spin (wz={W} rad/s) for {HOLD}s...")
    t0 = time.time()
    while time.time() - t0 < HOLD:
        tw = Twist()
        tw.angular.z = W
        n.pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.05)
    t_stop = time.time()
    print("STOPPED publishing cmd_vel. watching wheels...")
    while time.time() - t_stop < WATCH:
        rclpy.spin_once(n, timeout_sec=0.02)

    # derive |wheel speed| from consecutive position deltas (rad/s)
    spd = []  # (t, speed)
    for (t1, l1, r1), (t2, l2, r2) in zip(n.samples, n.samples[1:]):
        dt = t2 - t1
        if dt > 0:
            spd.append((t2, max(abs(l2 - l1), abs(r2 - r1)) / dt))
    moved = max((s for t, s in spd if t <= t_stop), default=0.0)
    after = [(t, s) for t, s in spd if t > t_stop]
    last_motion = max((t for t, s in after if s > STOP_VEL), default=None)

    print("\n==== PHASE 1a WATCHDOG RESULT ====")
    print(f" max wheel speed while commanding = {moved:.2f} rad/s  (>0 means motors drove)")
    if moved < STOP_VEL:
        print(" wheels never spun in our samples — measurement missed it; retry.")
    elif last_motion is None:
        print(f" wheels already stopped within {STOP_VEL} rad/s right after cmd_vel — "
              f"stop latency < ~0.1 s (watchdog OK).")
    else:
        print(f" wheels stopped {last_motion - t_stop:.2f} s after last cmd_vel  "
              f"(PASS if <~0.7; watchdog timeout = 0.5)")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
