#!/usr/bin/env python3
"""Demo video from a running interception: chase camera (left) + onboard camera with HUD (right).

    terminal 1:  ros2 launch follow_me_gazebo intercept.launch.py chase_cam:=true
    terminal 2:  tools/record_video.py docs/media/demo.mp4 [speedup=2] [max_sim_s=125]

One output frame per chase-camera frame (15 Hz of sim time), written at 15 x speedup fps: the
2-minute flight becomes a 1-minute video. H.264 (browser-playable), no ffmpeg needed.
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from follow_me_msgs.msg import TargetEstimate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

W, H, BANNER = 640, 360, 56
PHASE_BGR = {"WAIT": (163, 168, 169), "MIDCOURSE": (52, 104, 235), "HANDOVER": (0, 161, 237),
             "FOLLOW": (122, 175, 27), "LOST": (163, 168, 169)}
CAPTION = {
    "WAIT": "Waiting for a radar track",
    "MIDCOURSE": "Ground radar track only: flying to the predicted intercept point",
    "HANDOVER": "Radar cues the camera; learning the unknown target's size",
    "FOLLOW": "Camera locked: radar + camera fusion, following at a safe distance",
    "LOST": "Track lost: holding position",
}


def to_bgr(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
    a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding.lower().startswith("rgb") else a.copy()
    return cv2.resize(a, (W, H)) if (a.shape[1], a.shape[0]) != (W, H) else a


class Recorder(Node):
    def __init__(self, path, fps, max_sim):
        super().__init__("record_video", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"avc1"), fps, (2 * W, H + BANNER))
        if not self.out.isOpened():
            raise RuntimeError("cannot open the H.264 writer")
        self.max_sim, self.t0, self.n = max_sim, None, 0
        self.hud, self.phase, self.dist, self.size = None, "WAIT", None, None
        self.create_subscription(Image, "/follower/hud_image", lambda m: setattr(self, "hud", to_bgr(m)),
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/mission/phase", lambda m: setattr(self, "phase", m.data), 10)
        self.create_subscription(Float64, "/monitor/distance", lambda m: setattr(self, "dist", m.data), 10)
        self.create_subscription(TargetEstimate, "/target/estimate", self.on_est, 10)
        self.create_subscription(Image, "/follower/chase_camera/image", self.on_chase, qos_profile_sensor_data)

    def on_est(self, m):
        self.size = (m.target_size, m.target_size_sigma) if m.target_size_sigma < 0.25 else None

    def on_chase(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.t0 is None:
            self.t0 = t
        ts = t - self.t0
        if ts > self.max_sim:
            return
        frame = np.zeros((H + BANNER, 2 * W, 3), np.uint8)
        frame[:] = (24, 24, 22)
        frame[BANNER:, :W] = to_bgr(msg)
        if self.hud is not None:
            frame[BANNER:, W:] = self.hud
        col = PHASE_BGR.get(self.phase, (200, 200, 200))
        cv2.rectangle(frame, (12, 12), (34, 34), col, cv2.FILLED)
        txt = lambda s, x, y, sc, c=(236, 236, 236), th=1: cv2.putText(  # noqa: E731
            frame, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, sc, c, th, cv2.LINE_AA)
        txt(self.phase, 44, 31, 0.7, col, 2)
        txt(CAPTION.get(self.phase, ""), 210, 30, 0.55)
        info = f"t = {ts:5.1f} s"
        if self.dist is not None:
            info += f"    distance {self.dist:5.1f} m"
        if self.size is not None:
            info += f"    learnt size {self.size[0]:.2f} +/- {2 * self.size[1]:.2f} m"
        txt(info, 210, 50, 0.45, (180, 180, 176))
        txt("chase camera", 10, BANNER + H - 10, 0.45, (236, 236, 236))
        txt("onboard camera + HUD", W + 10, BANNER + H - 34, 0.45, (236, 236, 236))
        self.out.write(frame)
        self.n += 1


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "docs/media/demo.mp4"
    speedup = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    max_sim = float(sys.argv[3]) if len(sys.argv) > 3 else 125.0
    rclpy.init()
    r = Recorder(path, 15.0 * speedup, max_sim)
    t_wall = time.time()
    try:
        while rclpy.ok() and (r.t0 is None or r.n < 15.0 * max_sim - 1) and time.time() - t_wall < 3 * max_sim + 60:
            rclpy.spin_once(r, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    r.out.release()
    print(f"{r.n} frames -> {path} ({r.n / (15.0 * speedup):.0f} s of video)")


if __name__ == "__main__":
    main()
