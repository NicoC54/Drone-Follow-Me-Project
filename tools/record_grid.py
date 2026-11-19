#!/usr/bin/env python3
"""ONE Gazebo flight, four synchronised views in one GIF (README): chase camera, onboard camera +
HUD, RViz (what the algorithms see) and a top view of the whole engagement.

    sudo apt install xvfb imagemagick          # once
    source ros2_ws/install/setup.bash
    tools/record_grid.py record out/grid [launch args...]   # ~2 min: isolated flight, frames + tracks
    tools/record_grid.py compose out/grid docs/media/grid.gif [--title T] [--start S] [--end E] [--speedup X]
    tools/record_grid.py story out/grid docs/media/estimation.gif [--title T]   # estimation story GIF

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
    from follow_me_msgs.msg import RadarScan, TargetEstimate
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
            self.tracks = {"fol": [], "lead": [], "est": [], "phase": [], "radar": []}
            sub = self.create_subscription
            sub(Image, "/follower/hud_image", lambda m: setattr(self, "hud", to_bgr(m)), qos_profile_sensor_data)
            sub(Image, "/follower/chase_camera/image", self.on_chase, qos_profile_sensor_data)
            sub(String, "/mission/phase", self.on_phase, 10)
            sub(Float64, "/monitor/distance", lambda m: setattr(self, "dist", m.data), 10)
            sub(TargetEstimate, "/target/estimate", self.on_est, 20)
            sub(Odometry, "/follower/odom_truth", lambda m: self.odom("fol", m), 50)
            sub(Odometry, "/leader/odom_truth", lambda m: self.odom("lead", m), 50)
            sub(RadarScan, "/radar/scan", self.on_scan, 20)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def odom(self, key, m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            if not self.tracks[key] or stamp(m) - self.tracks[key][-1][0] >= 0.05:
                self.tracks[key].append([stamp(m), p.x, p.y, p.z, q.w, q.x, q.y, q.z])

        def on_scan(self, m):  # every plot in world coordinates (target, own echo, clutter)
            for pl in m.plots:
                ce = np.cos(pl.elevation)
                self.tracks["radar"].append([stamp(m), m.site.x + pl.range * ce * np.cos(pl.azimuth),
                                             m.site.y + pl.range * ce * np.sin(pl.azimuth),
                                             m.site.z + pl.range * np.sin(pl.elevation)])

        def on_phase(self, m):
            if m.data != self.phase:
                self.tracks["phase"].append([self.now(), PHASES.index(m.data) if m.data in PHASES else 0])
            self.phase = m.data

        def on_est(self, m):
            self.size = (m.target_size, m.target_size_sigma) if m.target_size_sigma < 0.25 else None
            if not self.tracks["est"] or stamp(m) - self.tracks["est"][-1][0] >= 0.05:
                c = list(m.covariance)  # position block of the 6x6 covariance (row-major)
                self.tracks["est"].append([stamp(m), m.position.x, m.position.y, m.position.z,
                                           float(m.camera_locked), c[0], c[1], c[2], c[7], c[8], c[14],
                                           m.target_size, m.target_size_sigma, float(m.status)])

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


# ------------------------------------------------------------------------------- live plots
def make_plotter(tr, t_start, t_end, width, height, speeds=None):
    """Live plots of one flight, drawn up to time t: range error along the line of sight against the
    filter's own 3-sigma on that axis (a consistent filter keeps the black curve under the orange
    one), the learnt size +- 2 sigma, and the distance to the target with the 4 m limit."""
    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fol, lead, est = (np.array(tr[k]) for k in ("fol", "lead", "est"))
    ph = tr["phase"]
    t_follow = next((tt for tt, k in ph if k == 3), 1e9)
    t_hand = next((tt for tt, k in ph if k == 2), t_follow)
    t_lock = next((e[0] for e in est if e[4] > 0), t_follow)
    tt = est[:, 0]
    truth = np.c_[[np.interp(tt, lead[:, 0], lead[:, k]) for k in (1, 2, 3)]].T
    own = np.c_[[np.interp(tt, fol[:, 0], fol[:, k]) for k in (1, 2, 3)]].T
    los = truth - own
    los /= np.maximum(np.linalg.norm(los, axis=1, keepdims=True), 1e-6)
    err_los = np.abs(np.sum((est[:, 1:4] - truth) * los, axis=1))
    P = np.zeros((len(est), 3, 3))
    for (i, j), col in {(0, 0): 5, (0, 1): 6, (0, 2): 7, (1, 1): 8, (1, 2): 9, (2, 2): 10}.items():
        P[:, i, j] = P[:, j, i] = est[:, col]
    sig3 = 3.0 * np.sqrt(np.maximum(np.einsum("ni,nij,nj->n", los, P, los), 0.0))
    inside = (err_los <= sig3)
    tracked = est[:, 13] > 0
    has_size = (est[:, 11] > 0) & (est[:, 12] < 0.35)
    tf_ = fol[:, 0]
    dist = np.linalg.norm(fol[:, 1:4] - np.c_[[np.interp(tf_, lead[:, 0], lead[:, k]) for k in (1, 2, 3)]].T, axis=1)

    plt.rcParams.update({"font.size": 8, "axes.titlesize": 9, "axes.titleweight": "bold",
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, axs = plt.subplots(1, 3, figsize=(width / 100, height / 100), dpi=100)
    fig.patch.set_facecolor("#fcfcfb")
    for ax in axs:
        ax.set_facecolor("#fcfcfb")
        ax.set_xlim(0, t_end - t_start)
        ax.grid(True, color="#e6e5e1", lw=0.6)
        ax.axvspan(t_hand - t_start, t_follow - t_start, color="#eda100", alpha=0.10, lw=0)
        ax.axvspan(t_follow - t_start, t_end - t_start, color="#1baf7a", alpha=0.10, lw=0)
        if t_start < t_lock < t_end:
            ax.axvline(t_lock - t_start, color="#52514e", lw=0.9, ls=":")
        ax.axvline(t_follow - t_start, color="#1baf7a", lw=1.0, ls="--")
        ax.set_xlabel("time [s]", labelpad=1)
    # playback speed of the GIF, marked on every plot: the cursor moves at a varying pace on a
    # flight-time axis (user feedback: "the curves are not in sync with the video speed")
    if speeds is not None:
        import matplotlib.transforms as mtransforms
        sp = speeds
        bounds = [(t_start, t_hand, sp[0]), (t_hand, t_follow - 4.0, sp[1]), (t_follow - 4.0, t_follow + 6.0, sp[2]),
                  (t_follow + 6.0, t_end, sp[3])]
        for ax in axs:
            tr_ = mtransforms.blended_transform_factory(ax.transData, ax.transAxes)
            for a, b, v in bounds:
                a, b = max(a, t_start), min(b, t_end)
                if b - a < 0.5:
                    continue
                # ABOVE the plot area (inside, the labels hid the curves and the legends)
                ax.plot([a - t_start, b - t_start], [1.03, 1.03], transform=tr_, color="#8a8984", lw=2.0,
                        solid_capstyle="butt", clip_on=False)
                ax.plot([a - t_start] * 2, [1.01, 1.05], transform=tr_, color="#fcfcfb", lw=1.2, clip_on=False)
                ax.text(0.5 * (a + b) - t_start, 1.05, f"x{v:g}", transform=tr_, ha="center", va="bottom",
                        fontsize=6.5, color="#52514e", clip_on=False)
    a0, a1, a2 = axs
    a0.set_title("range error vs the filter's 3-sigma", loc="left", pad=16)
    a0.set_yscale("log")
    a0.set_ylim(0.02, 30)
    a0.set_ylabel("[m]", labelpad=1)
    l_sig, = a0.plot([], [], color="#eb6834", lw=1.2, label="3-sigma (filter)")
    l_err, = a0.plot([], [], color="#0b0b0b", lw=1.2, label="true error")
    if t_start < t_lock < t_end:
        a0.text(t_lock - t_start + 0.4, 6, "camera:\nbearing", color="#52514e", fontsize=7, va="top")
    a0.text(t_follow - t_start + 0.4, 6, "camera:\nrange", color="#1baf7a", fontsize=7, va="top")
    a0.legend(loc="lower right", frameon=True, facecolor="#fcfcfb", edgecolor="none", framealpha=0.9, fontsize=7)
    a1.set_title("unknown size, learnt in flight", loc="left", pad=16)
    a1.set_ylim(0.0, max(2.0, 1.4 * est[has_size, 11].max() if has_size.any() else 2.0))
    a1.set_ylabel("[m]", labelpad=1)
    band = [None]
    l_size, = a1.plot([], [], color="#2a78d6", lw=1.4, label="learnt size, +-2 sigma")
    a1.legend(loc="upper right", frameon=False, fontsize=7)
    a2.set_title("distance to the target", loc="left", pad=16)
    a2.set_yscale("log")
    a2.set_ylim(2, 600)
    a2.set_ylabel("[m]", labelpad=1)
    a2.axhline(4.0, color="#e34948", lw=1.0)
    a2.text(0.5, 2.6, "4 m minimum", color="#e34948", fontsize=7)
    l_dist, = a2.plot([], [], color="#2a78d6", lw=1.4)
    cur = [ax.axvline(0, color="#52514e", lw=0.8) for ax in axs]
    fig.tight_layout(pad=0.4)

    def plots(t):
        k = (tt <= t) & (tt >= t_start) & tracked
        l_err.set_data(tt[k] - t_start, np.maximum(err_los[k], 0.02))
        l_sig.set_data(tt[k] - t_start, np.maximum(sig3[k], 0.02))
        ks = k & has_size
        l_size.set_data(tt[ks] - t_start, est[ks, 11])
        if band[0] is not None:
            band[0].remove()
        band[0] = a1.fill_between(tt[ks] - t_start, est[ks, 11] - 2 * est[ks, 12], est[ks, 11] + 2 * est[ks, 12],
                                  color="#2a78d6", alpha=0.2, lw=0)
        kd = (tf_ <= t) & (tf_ >= t_start)
        l_dist.set_data(tf_[kd] - t_start, dist[kd])
        for c in cur:
            c.set_xdata([t - t_start, t - t_start])
        fig.canvas.draw()
        img = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba())[:, :, :3], cv2.COLOR_RGB2BGR)
        return cv2.resize(img, (width, height)) if img.shape[:2] != (height, width) else img

    k_fol = (tt > t_follow + 3) & tracked
    stats = {"inside_3sigma_follow": float(inside[k_fol].mean()) if k_fol.any() else float("nan"),
             "range_err_follow": float(np.median(err_los[k_fol])) if k_fol.any() else float("nan"),
             "range_err_radar": float(np.median(err_los[(tt < t_lock) & (tt > t_lock - 15) & tracked]))
             if ((tt < t_lock) & tracked).any() else float("nan")}
    return plots, (lambda: plt.close(fig)), stats


# ---------------------------------------------------------------------------------- compose
FOL_BGR, TGT_BGR, RADAR_BGR, INK_BGR = (214, 120, 42), (72, 73, 227), (52, 104, 235), (82, 81, 78)


def legend(img, x, y, extra=(), scale=0.4, step=16):
    """Legend drawn with the SAME symbols as the plot (a "^" in text did not look like the triangle)."""
    import cv2
    rows = [("line", FOL_BGR, "follower"), ("line", TGT_BGR, "target (truth)"),
            ("ring", (122, 175, 27), "estimate"), ("tri", RADAR_BGR, "ground radar"), *extra]
    for i, (kind, col, text) in enumerate(rows):
        yy = y + step * i
        if kind == "line":
            cv2.line(img, (x, yy), (x + 16, yy), col, 2, cv2.LINE_AA)
            cv2.circle(img, (x + 16, yy), 4, col, -1, cv2.LINE_AA)
        elif kind == "ring":
            cv2.circle(img, (x + 8, yy), 6, col, 1, cv2.LINE_AA)
        elif kind == "tri":
            cv2.drawMarker(img, (x + 8, yy + 2), col, cv2.MARKER_TRIANGLE_UP, 11, 2)
        elif kind == "dot":
            cv2.circle(img, (x + 8, yy), 2, col, -1, cv2.LINE_AA)
        elif kind == "ellipse":
            cv2.ellipse(img, (x + 8, yy), (8, 4), 0, 0, 360, col, 1, cv2.LINE_AA)
        cv2.putText(img, text, (x + 24, yy + 4), cv2.FONT_HERSHEY_SIMPLEX, scale, col if kind != "dot" else INK_BGR,
                    1, cv2.LINE_AA)


def horizon_tilt_deg(q):
    """Roll of the body (camera) about its forward axis w.r.t. the horizontal, from (w, x, y, z)."""
    w, x, y, z = q
    up_b = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])  # world up, body frame
    return np.degrees(np.arctan2(up_b[1], up_b[2]))


def level_horizon(img, tilt_deg):
    """The chase camera is rigidly attached to the follower: the whole world rolled at every brake.
    Counter-rotate the image so that the horizon stays level and the DRONE is seen tilting."""
    import cv2
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w / 2, h / 2), -tilt_deg, 1.0 + 0.6 * min(abs(np.radians(tilt_deg)), 0.6))
    return cv2.warpAffine(img, m, (w, h), borderMode=cv2.BORDER_REPLICATE)


SPEEDS = (6.0, 4.0, 2.0, 3.0)  # mid-course, hand-over, around the FOLLOW entry, follow (--speeds)


def speed_profile(t, t_hand, t_follow, speeds=None):
    """Playback speed by mission phase: the straight mid-course goes fast, the camera lock and the
    entry into FOLLOW are slow enough to be read, the follow manoeuvres in between."""
    mid, hand, entry, follow = speeds or SPEEDS
    if t < t_hand:
        return mid
    if t < t_follow - 4.0:
        return hand
    if t < t_follow + 6.0:
        return entry
    return follow


def compose(src, dst, t_start=None, t_end=None, speedup=3.0, tile=(480, 270), step=0.4, title="", colors=96,
            fps=None, plots_h=0, speeds=None):
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
        legend(img, 8, 30)
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
    plotter = None
    if plots_h and len(est) and est.shape[1] >= 14:  # live estimation plots under the grid (same flight)
        plotter, close_plots, stats = make_plotter(tr, t_start, t_end, 2 * W, plots_h,
                                                   (speeds or SPEEDS) if fps else None)
        print("estimation:", {k: round(v, 3) for k, v in stats.items()})
    if fps:  # variable speed: one frame every speed / fps seconds of flight
        times, t = [], t_start
        while t < t_end:
            times.append(t)
            t += speed_profile(t, t_hand, t_follow, speeds) / fps
    else:
        times = list(np.arange(t_start, t_end, step))
    frames = []
    for t in times:
        speed = speed_profile(t, t_hand, t_follow, speeds) if fps else speedup
        c = cams[int(np.argmin(abs(tc - t)))]
        r = rv[int(np.argmin(abs(trv - t)))]
        both = cv2.imread(os.path.join(src, "cam", c["file"]))
        chase_raw = both[:, :640]
        if fol.shape[1] >= 8:  # attitude recorded (since 2026-11-19 evening): level the horizon
            chase_raw = level_horizon(chase_raw, horizon_tilt_deg(fol[int(np.argmin(abs(fol[:, 0] - c["t"]))), 4:8]))
        chase, hud = fit(chase_raw, W, H), fit(both[:, 640:], W, H)
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
        label(chase, "Gazebo: chase camera (horizon levelled)" if fol.shape[1] >= 8 else "Gazebo: chase camera")
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
        frame = np.vstack([ban, grid] + ([plotter(t)] if plotter else []))
        frames.append(PImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                      .quantize(colors=colors, method=PImage.Quantize.FASTOCTREE, dither=PImage.Dither.NONE))
    if plotter:
        close_plots()
    frame_ms = int(1000 / fps) if fps else int(1000 * step / speedup)
    frames[0].save(dst, save_all=True, append_images=frames[1:], duration=frame_ms, loop=0, optimize=True)
    print(f"{len(frames)} frames, {len(frames) * frame_ms / 1000:.1f} s, {os.path.getsize(dst) / 1e6:.1f} MB -> {dst}")


# ------------------------------------------------------------------------------------ story
def compose_story(src, dst, title="", t_start=None, t_end=None, fps=20.0, colors=96):
    """The estimation story of one flight, for a GNC reader: an auto-zooming top view (radar plots,
    3-sigma ellipse, truth, estimate), the onboard camera, and live plots of the estimate error
    against the filter's own 3-sigma, the learnt size and the distance to the target."""
    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image as PImage

    with open(os.path.join(src, "index.json")) as f:
        idx = json.load(f)
    cams, tr = idx["cams"], idx["tracks"]
    fol, lead, est = (np.array(tr[k]) for k in ("fol", "lead", "est"))
    radar = np.array(tr["radar"]) if tr.get("radar") else np.zeros((0, 4))
    n_radars = next((int(a.split(":=")[1]) for a in idx.get("launch_args", []) if a.startswith("radars:=")), 1)
    sites = RADAR_SITES[:n_radars]
    ph = tr["phase"]
    t_follow = next((tt for tt, k in ph if k == 3), 1e9)
    t_hand = next((tt for tt, k in ph if k == 2), t_follow)
    t_lock = next((e[0] for e in est if e[4] > 0), t_follow)
    t_start = t_hand - 18.0 if t_start is None else t_start
    t_end = t_follow + 20.0 if t_end is None else t_end
    tc = np.array([c["t"] for c in cams])

    def at(arr, t, cols):
        return np.array([np.interp(t, arr[:, 0], arr[:, k]) for k in cols])

    # series for the plots
    tt = est[:, 0]
    truth = np.c_[[np.interp(tt, lead[:, 0], lead[:, k]) for k in (1, 2, 3)]].T
    err = np.linalg.norm(est[:, 1:4] - truth, axis=1)
    sig3 = 3.0 * np.sqrt(np.maximum(est[:, 5] + est[:, 8] + est[:, 10], 0.0))
    tf_ = fol[:, 0]
    dist = np.linalg.norm(fol[:, 1:4] - np.c_[[np.interp(tf_, lead[:, 0], lead[:, k]) for k in (1, 2, 3)]].T, axis=1)
    has_size = (est[:, 11] > 0) & (est[:, 12] < 0.35)  # learning has started (prior sigma 0.4 m)
    tracked = est[:, 13] > 0

    W, H, PH = 480, 270, 300
    BAN = 62 if title else 44

    # ---- plots panel (matplotlib, updated per frame) ----
    plt.rcParams.update({"font.size": 8, "axes.titlesize": 9, "axes.titleweight": "bold",
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, axs = plt.subplots(1, 3, figsize=(9.6, PH / 100), dpi=100)
    fig.patch.set_facecolor("#fcfcfb")
    for ax in axs:
        ax.set_facecolor("#fcfcfb")
        ax.set_xlim(t_start - t_start, t_end - t_start)
        ax.grid(True, color="#e6e5e1", lw=0.6)
        ax.axvspan(t_hand - t_start, t_follow - t_start, color="#eda100", alpha=0.10, lw=0)
        ax.axvspan(t_follow - t_start, t_end - t_start, color="#1baf7a", alpha=0.10, lw=0)
        ax.axvline(t_lock - t_start, color="#52514e", lw=0.9, ls=":")
        ax.axvline(t_follow - t_start, color="#1baf7a", lw=1.0, ls="--")
        ax.set_xlabel("time [s]")
    a0, a1, a2 = axs
    a0.set_title("estimate error vs the filter's 3-sigma", loc="left")
    a0.set_yscale("log")
    a0.set_ylim(0.05, 40)
    a0.set_ylabel("[m]")
    l_sig, = a0.plot([], [], color="#eb6834", lw=1.2, label="3-sigma (filter)")
    l_err, = a0.plot([], [], color="#0b0b0b", lw=1.4, label="true error")
    a0.text(t_lock - t_start + 0.5, 22, "camera:\nbearing", color="#52514e", fontsize=7, va="top")
    a0.text(t_follow - t_start + 0.5, 22, "camera:\nrange (size\nlearnt)", color="#1baf7a", fontsize=7, va="top")
    a0.legend(loc="lower right", frameon=False, fontsize=7)
    a1.set_title("unknown target size, learnt in flight", loc="left")
    a1.set_ylim(0.0, max(2.0, 1.4 * est[has_size, 11].max() if has_size.any() else 2.0))
    a1.set_ylabel("[m]")
    band = [None]
    l_size, = a1.plot([], [], color="#2a78d6", lw=1.4, label="learnt size, +-2 sigma")
    a1.legend(loc="upper right", frameon=False, fontsize=7)
    a2.set_title("distance to the target", loc="left", pad=16)
    a2.set_yscale("log")
    a2.set_ylim(2, 600)
    a2.set_ylabel("[m]")
    a2.axhline(4.0, color="#e34948", lw=1.0)
    a2.text(0.5, 3.0, "4 m minimum", color="#e34948", fontsize=7)
    l_dist, = a2.plot([], [], color="#2a78d6", lw=1.4)
    cur = [ax.axvline(0, color="#52514e", lw=0.8) for ax in axs]
    fig.tight_layout(pad=0.6)

    def plots(t):
        x0 = t_start
        k = (tt <= t) & (tt >= t_start) & tracked
        l_err.set_data(tt[k] - x0, np.maximum(err[k], 0.05))
        l_sig.set_data(tt[k] - x0, np.maximum(sig3[k], 0.05))
        ks = k & has_size
        l_size.set_data(tt[ks] - x0, est[ks, 11])
        if band[0] is not None:
            band[0].remove()
        band[0] = a1.fill_between(tt[ks] - x0, est[ks, 11] - 2 * est[ks, 12], est[ks, 11] + 2 * est[ks, 12],
                                  color="#2a78d6", alpha=0.2, lw=0)
        kd = (tf_ <= t) & (tf_ >= t_start)
        l_dist.set_data(tf_[kd] - x0, dist[kd])
        for c in cur:
            c.set_xdata([t - x0, t - x0])
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)

    # ---- auto-zooming top view ----
    zoom = [None]

    def top_view(t):
        img = np.full((H, W, 3), (251, 252, 252), np.uint8)
        pt, pf = at(lead, t, (1, 2)), at(fol, t, (1, 2))
        d = np.linalg.norm(pt - pf)
        hw_target = float(np.clip(0.75 * d + 12.0, 18.0, 230.0))
        zoom[0] = hw_target if zoom[0] is None else zoom[0] + 0.15 * (hw_target - zoom[0])
        hw = zoom[0]
        centre = 0.5 * (pt + pf)
        sc = (H / 2 - 14) / hw

        def px(xy):
            q = (np.asarray(xy, float) - centre) * sc
            if q.ndim > 1:
                return np.c_[W / 2 + q[:, 0], H / 2 - q[:, 1]].astype(np.int32)
            return int(W / 2 + q[0]), int(H / 2 - q[1])

        for s_ in sites:
            x, y = px(s_)
            if 0 <= x < W and 0 <= y < H:
                cv2.drawMarker(img, (x, y), RADAR_BGR, cv2.MARKER_TRIANGLE_UP, 12, 2)
        rk = (radar[:, 0] <= t) & (radar[:, 0] > t - 2.5) if len(radar) else np.zeros(0, bool)
        for r_ in radar[rk]:
            cv2.circle(img, px(r_[1:3]), 2, (150, 150, 150), -1, cv2.LINE_AA)
        for arr, col in ((lead, TGT_BGR), (fol, FOL_BGR)):
            k = (arr[:, 0] <= t) & (arr[:, 0] > t - 25)
            if k.sum() > 1:
                cv2.polylines(img, [px(arr[k, 1:3])], False, col, 2, cv2.LINE_AA)
            cv2.circle(img, px(at(arr, t, (1, 2))), 5, col, -1, cv2.LINE_AA)
        k = np.nonzero((tt <= t) & tracked)[0]
        if len(k):
            e = est[k[-1]]
            col = (122, 175, 27) if e[4] > 0 else (52, 104, 235)
            P = np.array([[e[5], e[6]], [e[6], e[8]]])
            vals, vecs = np.linalg.eigh(P)
            ax_len = (3 * np.sqrt(np.maximum(vals, 1e-6)) * sc).astype(int)
            ang = -np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1]))
            c_ = px(e[1:3])
            cv2.ellipse(img, c_, (max(int(ax_len[1]), 2), max(int(ax_len[0]), 2)), ang, 0, 360, col, 2, cv2.LINE_AA)
            cv2.drawMarker(img, c_, col, cv2.MARKER_CROSS, 8, 1)
            cv2.putText(img, "radar + camera" if e[4] > 0 else "radar only", (c_[0] + 8, c_[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)
        # scale bar
        nice = [1, 2, 5, 10, 20, 50, 100, 200]
        L = next((n for n in nice if n * sc > W / 6), 200)
        cv2.line(img, (W - 20 - int(L * sc), H - 14), (W - 20, H - 14), INK_BGR, 2)
        cv2.putText(img, f"{L} m", (W - 20 - int(L * sc), H - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, INK_BGR, 1,
                    cv2.LINE_AA)
        cv2.putText(img, "top view (auto zoom)", (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, INK_BGR, 1, cv2.LINE_AA)
        legend(img, 8, 32, extra=(("ellipse", (52, 104, 235), "3-sigma, radar only"),
                                  ("ellipse", (122, 175, 27), "3-sigma, + camera"),
                                  ("dot", (150, 150, 150), "radar plots")), scale=0.36, step=13)
        return img

    def fit(img, w, h):
        ih, iw = img.shape[:2]
        if iw / ih > w / h:
            nw = int(ih * w / h)
            img = img[:, (iw - nw) // 2:(iw - nw) // 2 + nw]
        else:
            nh = int(iw * h / w)
            img = img[(ih - nh) // 2:(ih - nh) // 2 + nh]
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

    times, t = [], t_start
    while t < t_end:
        times.append(t)
        t += speed_profile(t, t_hand, t_follow) / fps
    frames = []
    for t in times:
        c = cams[int(np.argmin(abs(tc - t)))]
        hud = fit(cv2.imread(os.path.join(src, "cam", c["file"]))[:, 640:], W, H)
        cv2.rectangle(hud, (0, H - 20), (180, H), (24, 24, 22), -1)
        cv2.putText(hud, "onboard camera + HUD", (4, H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (236, 236, 236), 1,
                    cv2.LINE_AA)
        top = np.hstack([top_view(t), hud])
        p_img = plots(t)
        p_img = cv2.resize(p_img, (2 * W, PH)) if p_img.shape[1] != 2 * W else p_img
        ban = np.full((BAN, 2 * W, 3), (22, 24, 24), np.uint8)
        name = PHASES[next((int(k) for tt_, k in reversed(ph) if tt_ <= t), 0)]
        rgb = PHASE_RGB[name]
        y0 = 18 if title else 0
        if title:
            cv2.putText(ban, title, (10, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 200, 240), 1, cv2.LINE_AA)
        cv2.rectangle(ban, (10, 12 + y0), (28, 30 + y0), rgb[::-1], -1)
        cv2.putText(ban, name, (36, 28 + y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, rgb[::-1], 2, cv2.LINE_AA)
        cv2.putText(ban, CAPTION[name], (190, 20 + y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (236, 236, 236), 1,
                    cv2.LINE_AA)
        k = np.nonzero((tt <= t) & tracked)[0]
        info = f"t = {t - t_start:4.1f} s   x{speed_profile(t, t_hand, t_follow):.0f}"
        if len(k):
            info += f"   estimate error {err[k[-1]]:5.2f} m (3-sigma {sig3[k[-1]]:5.2f} m)"
        cv2.putText(ban, info, (190, 37 + y0), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 176), 1, cv2.LINE_AA)
        frame = np.vstack([ban, top, p_img])
        frames.append(PImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                      .quantize(colors=colors, method=PImage.Quantize.FASTOCTREE, dither=PImage.Dither.NONE))
    plt.close(fig)
    frames[0].save(dst, save_all=True, append_images=frames[1:], duration=int(1000 / fps), loop=0, optimize=True)
    print(f"{len(frames)} frames, {len(frames) / fps:.1f} s, {os.path.getsize(dst) / 1e6:.1f} MB -> {dst}")


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
        ap.add_argument("--plots", type=int, default=0, help="height [px] of the live estimation plots (0: none)")
        ap.add_argument("--speeds", default=None, help="playback speeds: mid-course,hand-over,entry,follow")
        a = ap.parse_args(sys.argv[2:])
        compose(a.src, a.dst, a.start, a.end, a.speedup, tile=tuple(int(v) for v in a.tile.split("x")),
                step=a.step, title=a.title, colors=a.colors, fps=a.fps, plots_h=a.plots,
                speeds=tuple(float(v) for v in a.speeds.split(",")) if a.speeds else None)
    elif len(sys.argv) >= 4 and sys.argv[1] == "story":
        ap = argparse.ArgumentParser()
        ap.add_argument("src")
        ap.add_argument("dst")
        ap.add_argument("--title", default="")
        ap.add_argument("--start", type=float)
        ap.add_argument("--end", type=float)
        ap.add_argument("--fps", type=float, default=20.0)
        ap.add_argument("--colors", type=int, default=96)
        a = ap.parse_args(sys.argv[2:])
        compose_story(a.src, a.dst, a.title, a.start, a.end, a.fps, a.colors)
    else:
        sys.exit(__doc__)
