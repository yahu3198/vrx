#!/usr/bin/env python3
"""Step-1 check 3: a fake /wamv/manifold_ref, no sidecar needed.

Waits for the fault (/wamv/thruster_health, any health < 99.9 %), then after
--delay seconds publishes a straight line from the vessel's current pose to
--goal (default (-582, 214), inside Zone 2 between the docks) over --duration
seconds at constant speed, followed by a --hold seconds zero-velocity hold,
in the exact layout WAMV_MPC::manifold_ref_cb expects:

    data[0] = t0 (wall-clock seconds, row 0 applies), data[1] = dt (0.05),
    data[2:] = rows of [x, y, psi_bounded, u, v, r, Tp, Ts]

The MPC must be running with ref_source:=manifold. Expected in the MPC log:
"[manifold] reference received: N rows (k skipped)", then "tracking external
reference", and /wamv/error_pose yaw staying small (no jump near +-6.28).

    source /opt/ros/<distro>/setup.bash
    python3 fake_ref.py            # or --delay 5 --goal -582 214
"""
import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class FakeRef(Node):
    def __init__(self, a):
        super().__init__("fake_manifold_ref")
        self.a = a
        self.pose = None
        self.t_fault = None
        self.sent = False
        self.create_subscription(Odometry, "/wamv/sensors/position/ground_truth_odometry", self.on_odom, 10)
        self.create_subscription(Float64MultiArray, "/wamv/thruster_health", self.on_health, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/wamv/manifold_ref", 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info("waiting for the fault on /wamv/thruster_health")

    def on_odom(self, m):
        q = m.pose.pose.orientation
        psi = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, psi)

    def on_health(self, m):
        if self.t_fault is None and len(m.data) >= 2 and min(m.data[:2]) < 99.9:
            self.t_fault = time.time()
            self.get_logger().warn(f"fault seen, publishing in {self.a.delay:.1f} s")

    def tick(self):
        if self.sent or self.t_fault is None or self.pose is None:
            return
        if time.time() - self.t_fault < self.a.delay:
            return
        x0, y0, _ = self.pose
        gx, gy = self.a.goal
        dt = 0.05
        n = int(self.a.duration / dt)
        L = math.hypot(gx - x0, gy - y0)
        psi = math.atan2(gy - y0, gx - x0)          # bounded (-pi, pi]
        u = L / self.a.duration
        rows = []
        for k in range(n + 1):
            s = k / n
            rows += [x0 + s * (gx - x0), y0 + s * (gy - y0), psi, u, 0.0, 0.0, 0.0, 0.0]
        for _ in range(int(self.a.hold / dt)):
            rows += [gx, gy, psi, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg = Float64MultiArray()
        msg.data = [self.t_fault, dt] + rows        # t0 = fault time, so `delay` rows are skipped
        self.pub.publish(msg)
        self.sent = True
        self.get_logger().info(f"published {len(rows) // 8} rows: ({x0:.1f},{y0:.1f}) -> ({gx},{gy}), "
                               f"psi={psi:.2f}, u={u:.2f} m/s, t0={self.t_fault:.2f}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--goal", type=float, nargs=2, default=[-582.0, 214.0])
    p.add_argument("--duration", type=float, default=150.0)
    p.add_argument("--hold", type=float, default=30.0)
    p.add_argument("--delay", type=float, default=3.0, help="seconds after the fault (mimics sidecar latency)")
    a = p.parse_args()
    rclpy.init()
    n = FakeRef(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
