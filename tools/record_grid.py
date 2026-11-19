#!/usr/bin/env python3
"""ONE Gazebo flight, four synchronised views in one GIF (README): chase camera, onboard camera +
HUD, RViz (what the algorithms see) and a top view of the whole engagement.

    sudo apt install xvfb imagemagick          # once
    source ros2_ws/install/setup.bash
    tools/record_grid.py record out/grid [launch args...]   # ~2 min: isolated flight, frames + tracks
    tools/record_grid.py compose out/grid docs/media/grid.gif [--title T] [--start S] [--end E] [--speedup X]

    # the two README situations
    tools/record_grid.py record out/demo1 target:=quad_x target_scale:=0.8 target_path:=random target_seed:=54 radars:=3
    tools/record_grid.py record out/demo2 target:=octo target_scale:=1.5 target_path:=random \
        target_maneuvers:=2 target_seed:=107 radars:=3

Why one composed GIF and not several: GIFs side by side in a page start when each one is loaded
and drift apart at every loop; and the former media came from three different flights.
Every source is stamped with the SIMULATION time (the common clock): chase / onboard images by
their header stamps, RViz screenshots (grabbed in a virtual display) by the node's sim clock at
grab time, tracks by their odometry stamps.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

import numpy as np

DISPLAY, SCREEN = ":77", "1600x900x24"
VIEW_BOX = (622, 62, 1400, 870)  # RViz 3D view (follow_me.rviz layout, see capture_rviz.py)
CAM_STEP = 0.1  # [s of sim] camera frames kept (x2 at 20 frames/s needs one every 0.1 s)
RVIZ_DISTANCE = 45.0  # [m] RViz follow camera for the grid (follow_me.rviz: 20)
RVIZ_CROP_W, RVIZ_CROP_BOTTOM = 0.70, 0.87  # RViz band in FOLLOW (fractions of the view): close-up
RVIZ_FAR = (1.0, 0.85)  # before FOLLOW: full width, higher band (the far target sits at 30-50 %)
ENV = {"ROS_DOMAIN_ID": "78", "GZ_PARTITION": "rviz_capture"}  # isolated from the user's runs
RADAR_SITES = [(-60.0, -40.0), (450.0, -100.0), (230.0, 420.0)]  # = intercept.launch.py
PHASES = ["WAIT", "MIDCOURSE", "HANDOVER", "FOLLOW", "LOST"]
PHASE_RGB = {"WAIT": (169, 168, 163), "MIDCOURSE": (235, 104, 52), "HANDOVER": (237, 161, 0),
             "FOLLOW": (27, 175, 122), "LOST": (169, 168, 163)}
CAPTION = {"WAIT": "Waiting for a radar track",
           "MIDCOURSE": "Radar track only: flying to the predicted intercept point",
           "HANDOVER": "Radar cues the camera; learning the target's size",
           "FOLLOW": "Camera locked: radar + camera fusion, following at 8 m",
           "LOST": "Track lost"}


# ----------------------------------------------------------------------------------- record
DEFAULT_ARGS = ["target:=hexa", "target_scale:=1.3", "target_seed:=1"]


def record(out, launch_args=None, sim_end=95.0, wall_max=900.0):
    launch_args = launch_args or DEFAULT_ARGS
    for tool in ("Xvfb", "import"):
        if not shutil.which(tool):
            sys.exit(f"{tool} not found (sudo apt install xvfb imagemagick)")
    os.makedirs(os.path.join(out, "cam"), exist_ok=True)
    os.makedirs(os.path.join(out, "rviz"), exist_ok=True)
    env = dict(os.environ, DISPLAY=DISPLAY, LIBGL_ALWAYS_SOFTWARE="1", **ENV)
    env.pop("WAYLAND_DISPLAY", None)
    os.environ.update(ENV)  # this process's ROS node joins the isolated domain
    # RViz: same configuration, camera further back (20 m showed little once cropped to 16:9)
    from ament_index_python.packages import get_package_share_directory
    with open(os.path.join(get_package_share_directory("follow_me_gazebo"), "rviz", "follow_me.rviz")) as f:
        cfg = f.read()
    cfg = cfg.replace("      Distance: 20\n", f"      Distance: {RVIZ_DISTANCE}\n", 1)
    rviz_cfg = os.path.join(out, "follow_me_grid.rviz")
    with open(rviz_cfg, "w") as f:
        f.write(cfg)
    xvfb = subprocess.Popen(["Xvfb", DISPLAY, "-screen", "0", SCREEN, "-nolisten", "tcp"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    launch = subprocess.Popen(
        ["ros2", "launch", "follow_me_gazebo", "intercept.launch.py", "gui:=false", "rviz:=true",
         "chase_cam:=true", f"rviz_config:={rviz_cfg}", *launch_args],
        env=env, stdout=open(os.path.join(out, "launch.log"), "w"), stderr=subprocess.STDOUT,
        start_new_session=True)

    import cv2
    import rclpy
    from follow_me_msgs.msg import TargetEstimate
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64, String

    def to_bgr(msg):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding.lower().startswith("rgb") else a.copy()

    def stamp(m):
        return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec

    class Rec(Node):
        def __init__(self):
            super().__init__("record_grid",
                             parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
            self.hud, self.phase, self.dist, self.size = None, "WAIT", None, None
            self.next_cam, self.cams = 0.0, []
            self.tracks = {"fol": [], "lead": [], "est": [], "phase": []}
            sub = self.create_subscription
            sub(Image, "/follower/hud_image", lambda m: setattr(self, "hud", to_bgr(m)), qos_profile_sensor_data)
            sub(Image, "/follower/chase_camera/image", self.on_chase, qos_profile_sensor_data)
            sub(String, "/mission/phase", self.on_phase, 10)
            sub(Float64, "/monitor/distance", lambda m: setattr(self, "dist", m.data), 10)
            sub(TargetEstimate, "/target/estimate", self.on_est, 20)
            sub(Odometry, "/follower/odom_truth", lambda m: self.odom("fol", m), 50)
            sub(Odometry, "/leader/odom_truth", lambda m: self.odom("lead", m), 50)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def odom(self, key, m):
            p = m.pose.pose.position
            if not self.tracks[key] or stamp(m) - self.tracks[key][-1][0] >= 0.1:
                self.tracks[key].append([stamp(m), p.x, p.y, p.z])

        def on_phase(self, m):
            if m.data != self.phase:
                self.tracks["phase"].append([self.now(), PHASES.index(m.data) if m.data in PHASES else 0])
            self.phase = m.data

        def on_est(self, m):
            self.size = (m.target_size, m.target_size_sigma) if m.target_size_sigma < 0.25 else None
            if not self.tracks["est"] or stamp(m) - self.tracks["est"][-1][0] >= 0.1:
                self.tracks["est"].append([stamp(m), m.position.x, m.position.y, m.position.z,
                                           float(m.camera_locked)])

        def on_chase(self, msg):
            t = stamp(msg)
            if t < self.next_cam or self.hud is None:
                return
            self.next_cam = t + CAM_STEP
            path = os.path.join(out, "cam", f"{t:08.2f}.jpg")
            cv2.imwrite(path, np.hstack([to_bgr(msg), cv2.resize(self.hud, (640, 360))]),
                        [cv2.IMWRITE_JPEG_QUALITY, 92])
            self.cams.append({"t": t, "file": os.path.basename(path), "phase": self.phase,
                              "dist": self.dist, "size": self.size})

    rclpy.init()
    node = Rec()
    stop = threading.Event()
    rviz_frames = []

    def grab():  # RViz screenshots, stamped with the sim clock at grab time
        while not stop.is_set():
            t = node.now()
            if t > 1.0:
                path = os.path.join(out, "rviz", f"{t:08.2f}.png")
                if subprocess.run(["import", "-display", DISPLAY, "-window", "root", "-crop",
                                   f"{VIEW_BOX[2] - VIEW_BOX[0]}x{VIEW_BOX[3] - VIEW_BOX[1]}+{VIEW_BOX[0]}+{VIEW_BOX[1]}",
                                   path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
                    rviz_frames.append({"t": 0.5 * (t + node.now()), "file": os.path.basename(path)})
            time.sleep(0.05)

    th = threading.Thread(target=grab, daemon=True)
    th.start()
    t_wall = time.time()
    try:
        while rclpy.ok() and node.now() < sim_end and time.time() - t_wall < wall_max:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        stop.set()
        th.join(timeout=5)
        try:
            os.killpg(launch.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        time.sleep(8)
        try:
            os.killpg(launch.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        xvfb.terminate()
    with open(os.path.join(out, "index.json"), "w") as f:
        json.dump({"cams": node.cams, "rviz": rviz_frames, "tracks": node.tracks, "launch_args": launch_args}, f)
    print(f"sim {node.now():.0f} s in {time.time() - t_wall:.0f} s wall: "
          f"{len(node.cams)} camera frames, {len(rviz_frames)} RViz frames -> {out}")
    rclpy.shutdown()


# ---------------------------------------------------------------------------------- compose
def speed_profile(t, t_hand, t_follow):
    """Playback speed by mission phase: the straight mid-course goes fast, the camera lock and the
    entry into FOLLOW are slow enough to be read, the follow manoeuvres in between."""
    if t < t_hand:
        return 6.0
    if t < t_follow - 4.0:
        return 4.0
    if t < t_follow + 6.0:
        return 2.0
    return 3.0


def compose(src, dst, t_start=None, t_end=None, speedup=3.0, tile=(480, 270), step=0.4, title="", colors=96,
            fps=None):
    import cv2
    from PIL import Image as PImage

    with open(os.path.join(src, "index.json")) as f:
        idx = json.load(f)
    cams, rv, tr = idx["cams"], idx["rviz"], idx["tracks"]
    n_radars = next((int(a.split(":=")[1]) for a in idx.get("launch_args", []) if a.startswith("radars:=")), 1)
    sites = RADAR_SITES[:n_radars]
    fol, lead, est = (np.array(tr[k]) for k in ("fol", "lead", "est"))
    ph_changes = tr["phase"]
    tc = np.array([c["t"] for c in cams])
    trv = np.array([r["t"] for r in rv])

    def phase_at(t):
        p = 0
        for tt, k in ph_changes:
            if tt <= t:
                p = int(k)
        return p

    if t_start is None:  # default: end of mid-course -> 15 s into FOLLOW
        hand = next((tt for tt, k in ph_changes if k == 2), tc[0])
        follow = next((tt for tt, k in ph_changes if k == 3), hand + 20)
        t_start, t_end = hand - 12.0, follow + 15.0
    W, H = tile
    BAN = 62 if title else 44

    # top view: fixed frame over the whole engagement (the follower and target paths so far)
    allxy = np.vstack([fol[:, 1:3], lead[:, 1:3]])  # framed on the paths: the manoeuvres stay visible
    lo, hi = allxy.min(0) - 25, allxy.max(0) + 25
    scale = min((W - 20) / (hi[0] - lo[0]), (H - 20) / (hi[1] - lo[1]))
    off = np.array([(W - scale * (hi[0] - lo[0])) / 2, (H - scale * (hi[1] - lo[1])) / 2])

    def px(xy):
        q = (np.asarray(xy) - lo) * scale + off
        return np.c_[q[..., 0], H - q[..., 1]].astype(np.int32) if q.ndim > 1 else (int(q[0]), int(H - q[1]))

    def top_view(t):
        img = np.full((H, W, 3), (251, 252, 252), np.uint8)
        for s in sites:  # radar sites off the frame: drawn on its edge
            x, y = px(s)
            x, y = int(np.clip(x, 8, W - 8)), int(np.clip(y, 26, H - 8))
            cv2.drawMarker(img, (x, y), (52, 104, 235), cv2.MARKER_TRIANGLE_UP, 12, 2)
        for arr, col in ((lead, (72, 73, 227)), (fol, (214, 120, 42))):
            k = arr[:, 0] <= t
            if k.sum() > 1:
                cv2.polylines(img, [px(arr[k, 1:3])], False, col, 2, cv2.LINE_AA)
                cv2.circle(img, px(arr[k][-1, 1:3]), 5, col, -1, cv2.LINE_AA)
        k = est[:, 0] <= t
        if k.any():
            e = est[k][-1]
            col = (122, 175, 27) if e[4] > 0 else (52, 104, 235)
            cv2.circle(img, px(e[1:3]), 7, col, 1, cv2.LINE_AA)
        put = lambda s, x, y, c: cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, c, 1, cv2.LINE_AA)  # noqa
        put("top view (whole engagement)", 8, 16, (82, 81, 78))
        for i, (s, c) in enumerate((("follower", (214, 120, 42)), ("target", (72, 73, 227)),
                                    ("o estimate", (82, 81, 78)), ("^ ground radar", (52, 104, 235)))):
            put(s, 8, 34 + 16 * i, c)
        return img

    def fit(img, w, h):  # center crop to w:h, then resize
        ih, iw = img.shape[:2]
        if iw / ih > w / h:
            nw = int(ih * w / h)
            img = img[:, (iw - nw) // 2:(iw - nw) // 2 + nw]
        else:
            nh = int(iw * h / w)
            img = img[(ih - nh) // 2:(ih - nh) // 2 + nh]
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

    def label(img, s):
        cv2.rectangle(img, (0, img.shape[0] - 20), (8 + 7 * len(s), img.shape[0]), (24, 24, 22), -1)
        cv2.putText(img, s, (4, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (236, 236, 236), 1, cv2.LINE_AA)

    t_follow = next((tt for tt, k in ph_changes if k == 3), 1e9)
    t_hand = next((tt for tt, k in ph_changes if k == 2), t_follow)
    if fps:  # variable speed: one frame every speed / fps seconds of flight
        times, t = [], t_start
        while t < t_end:
            times.append(t)
            t += speed_profile(t, t_hand, t_follow) / fps
    else:
        times = list(np.arange(t_start, t_end, step))
    frames = []
    for t in times:
        speed = speed_profile(t, t_hand, t_follow) if fps else speedup
        c = cams[int(np.argmin(abs(tc - t)))]
        r = rv[int(np.argmin(abs(trv - t)))]
        both = cv2.imread(os.path.join(src, "cam", c["file"]))
        chase, hud = fit(both[:, :640], W, H), fit(both[:, 640:], W, H)
        rv_img = cv2.imread(os.path.join(src, "rviz", r["file"]))[:, :-12]  # right edge: window border
        rh, rw = rv_img.shape[:2]  # 16:9 band of the view: wide and high while the target is far
        a = np.clip((t - t_follow) / 2.0, 0.0, 1.0)  # (mid-course, hand-over), tight in FOLLOW
        crop_w = (1 - a) * RVIZ_FAR[0] + a * RVIZ_CROP_W
        crop_bottom = (1 - a) * RVIZ_FAR[1] + a * RVIZ_CROP_BOTTOM
        cw = int(crop_w * rw)
        ch = int(cw * H / W)
        y1 = int(crop_bottom * rh)
        x0 = (rw - cw) // 2
        rviz = cv2.resize(rv_img[max(y1 - ch, 0):y1, x0:x0 + cw], (W, H), interpolation=cv2.INTER_AREA)
        label(chase, "Gazebo: chase camera")
        label(hud, "onboard camera + HUD")
        label(rviz, "RViz: estimate, 3-sigma, aim, field of view")
        grid = np.vstack([np.hstack([chase, hud]), np.hstack([rviz, top_view(t)])])
        ban = np.full((BAN, 2 * W, 3), (22, 24, 24), np.uint8)
        name = PHASES[phase_at(t)]
        rgb = PHASE_RGB[name]
        y0 = 18 if title else 0
        if title:
            cv2.putText(ban, title, (10, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 200, 240), 1, cv2.LINE_AA)
        cv2.rectangle(ban, (10, 12 + y0), (28, 30 + y0), rgb[::-1], -1)
        cv2.putText(ban, name, (36, 28 + y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, rgb[::-1], 2, cv2.LINE_AA)
        cv2.putText(ban, CAPTION[name], (190, 20 + y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (236, 236, 236), 1,
                    cv2.LINE_AA)
        info = f"t = {t - t_start:4.1f} s   x{speed:.0f}"
        if c["dist"] is not None:
            info += f"   distance {c['dist']:5.1f} m"
        if c["size"]:
            info += f"   learnt size {c['size'][0]:.2f} m"
        cv2.putText(ban, info, (190, 37 + y0), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 176), 1, cv2.LINE_AA)
        frame = np.vstack([ban, grid])
        frames.append(PImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                      .quantize(colors=colors, method=PImage.Quantize.FASTOCTREE, dither=PImage.Dither.NONE))
    frame_ms = int(1000 / fps) if fps else int(1000 * step / speedup)
    frames[0].save(dst, save_all=True, append_images=frames[1:], duration=frame_ms, loop=0, optimize=True)
    print(f"{len(frames)} frames, {len(frames) * frame_ms / 1000:.1f} s, {os.path.getsize(dst) / 1e6:.1f} MB -> {dst}")


if __name__ == "__main__":
    import argparse
    if len(sys.argv) >= 3 and sys.argv[1] == "record":
        record(sys.argv[2], sys.argv[3:] or None)
    elif len(sys.argv) >= 4 and sys.argv[1] == "compose":
        ap = argparse.ArgumentParser()
        ap.add_argument("src")
        ap.add_argument("dst")
        ap.add_argument("--title", default="")
        ap.add_argument("--start", type=float)
        ap.add_argument("--end", type=float)
        ap.add_argument("--speedup", type=float, default=3.0)
        ap.add_argument("--step", type=float, default=0.4, help="sim seconds between frames")
        ap.add_argument("--tile", default="480x270", help="size of each view [px]")
        ap.add_argument("--colors", type=int, default=96, help="palette size per frame")
        ap.add_argument("--fps", type=float, help="variable speed by phase at this frame rate (overrides --speedup)")
        a = ap.parse_args(sys.argv[2:])
        compose(a.src, a.dst, a.start, a.end, a.speedup, tile=tuple(int(v) for v in a.tile.split("x")),
                step=a.step, title=a.title, colors=a.colors, fps=a.fps)
    else:
        sys.exit(__doc__)
