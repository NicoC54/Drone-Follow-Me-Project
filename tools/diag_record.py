#!/usr/bin/env python3
"""Records what the phase-4 diagnosis needs, from a running launch, into an .npz file.

    tools/diag_record.py out/diag.npz [duration_s]

Detections, ground truth of both drones, follower navigation, gimbal angles, target estimate and
mission phase, flight-controller setpoint, all with their SIM-TIME stamps. Analysis: tools/diag_analyse.py.
"""
import sys
import time

import numpy as np
import rclpy
from follow_me_msgs.msg import Detection, TargetEstimate
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


class Rec(Node):
    def __init__(self):
        super().__init__("diag_record", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.d = {k: [] for k in ("det", "lead", "fol", "nav", "gim", "est", "phase", "sp", "dbg")}
        self.create_subscription(Detection, "/follower/detection", self.on_det, 50)
        self.create_subscription(Odometry, "/leader/odom_truth", lambda m: self.d["lead"].append(self.odom(m)), 200)
        self.create_subscription(Odometry, "/follower/odom_truth", lambda m: self.d["fol"].append(self.odom(m)), 200)
        self.create_subscription(Odometry, "/follower/odom", lambda m: self.d["nav"].append(self.odom(m)), 200)
        self.create_subscription(JointState, "/follower/joint_states", self.on_js, qos_profile_sensor_data)
        self.create_subscription(TargetEstimate, "/target/estimate", self.on_est, 50)
        self.create_subscription(String, "/mission/phase", self.on_phase, 10)
        # final acceleration + yaw setpoint sent to the flight controller (after the safety filter)
        self.create_subscription(Twist, "/follower/fc/accel_yaw_setpoint", self.on_sp, 50)
        self.create_subscription(Float64MultiArray, "/mission/debug", lambda m: self.d["dbg"].append(list(m.data)), 50)

    @staticmethod
    def odom(m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        return [stamp(m.header), p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z]

    def on_det(self, m):
        self.d["det"].append([stamp(m.header), float(m.detected), m.u, m.v, m.size, float(m.size_valid), m.thickness])

    def on_js(self, m):
        n = dict(zip(m.name, m.position))
        if "gimbal_yaw" in n and "gimbal_pitch" in n:
            self.d["gim"].append([stamp(m.header), n["gimbal_yaw"], n["gimbal_pitch"]])

    def on_est(self, m):
        c = list(m.covariance)
        self.d["est"].append([stamp(m.header), m.status, m.position.x, m.position.y, m.position.z,
                              m.velocity.x, m.velocity.y, m.velocity.z, float(m.camera_locked),
                              m.target_size, m.target_size_sigma] + c)

    def on_sp(self, m):
        self.d["sp"].append([self.get_clock().now().nanoseconds * 1e-9, m.linear.x, m.linear.y, m.linear.z,
                             m.angular.z])

    def on_phase(self, m):
        names = ["WAIT", "MIDCOURSE", "HANDOVER", "FOLLOW", "LOST"]
        self.d["phase"].append([self.get_clock().now().nanoseconds * 1e-9,
                                names.index(m.data) if m.data in names else -1])


def main():
    out = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 110.0
    rclpy.init()
    n = Rec()
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    np.savez(out, **{k: np.array(v, dtype=float) for k, v in n.d.items()})
    print({k: len(v) for k, v in n.d.items()})


if __name__ == "__main__":
    main()
