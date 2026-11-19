#!/usr/bin/env python3
"""RViz screenshots and GIF for the README, captured in a VIRTUAL display (nothing opens on screen).

    sudo apt install xvfb            # once
    source ros2_ws/install/setup.bash
    tools/capture_rviz.py docs/media  # ~2.5 min: rviz_handover.png, rviz_follow.png, rviz.gif

Starts Xvfb, the phase-4 launch (Gazebo headless, RViz on the virtual display, isolated
ROS_DOMAIN_ID / GZ_PARTITION), grabs the screen at 2 Hz with ImageMagick and labels every frame
with the mission phase.
"""
import os
import shutil
import signal
import subprocess
import sys
import time

from PIL import Image

DISPLAY, SCREEN = ":77", "1600x900x24"
RVIZ_BOX = (0, 0, 1400, 900)  # RViz window (follow_me.rviz: Window Geometry 1400 x 900)
VIEW_BOX = (622, 62, 1400, 870)  # its 3D view only (for the GIF): right of the 600 px dock column


def main():
    dst = sys.argv[1] if len(sys.argv) > 1 else "docs/media"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 125.0
    for tool in ("Xvfb", "import"):
        if not shutil.which(tool):
            sys.exit(f"{tool} not found (sudo apt install xvfb imagemagick)")
    tmp = os.path.join(os.environ.get("TMPDIR", "/tmp"), "rviz_capture")
    os.makedirs(tmp, exist_ok=True)
    env = dict(os.environ, DISPLAY=DISPLAY, ROS_DOMAIN_ID="78", GZ_PARTITION="rviz_capture",
               LIBGL_ALWAYS_SOFTWARE="1")
    env.pop("WAYLAND_DISPLAY", None)  # WSLg: force X11 on the virtual display
    xvfb = subprocess.Popen(["Xvfb", DISPLAY, "-screen", "0", SCREEN, "-nolisten", "tcp"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    launch = subprocess.Popen(
        ["ros2", "launch", "follow_me_gazebo", "intercept.launch.py", "gui:=false", "rviz:=true",
         "target:=hexa", "target_scale:=1.3", "target_seed:=1"],
        env=env, stdout=open(os.path.join(tmp, "launch.log"), "w"), stderr=subprocess.STDOUT,
        start_new_session=True)
    phase_proc = subprocess.Popen(["ros2", "topic", "echo", "--no-arr", "/mission/phase", "--field", "data"],
                                  env=dict(env, PYTHONUNBUFFERED="1"), stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, text=True,
                                  start_new_session=True)
    os.set_blocking(phase_proc.stdout.fileno(), False)
    frames, phase, t0 = [], "WAIT", time.time()
    try:
        while time.time() - t0 < duration:
            time.sleep(0.5)
            try:
                for line in phase_proc.stdout.read().splitlines():
                    if line.strip() and line.strip() != "---":
                        phase = line.strip()
            except (TypeError, AttributeError):
                pass
            path = os.path.join(tmp, f"f{len(frames):04d}.png")
            if subprocess.run(["import", "-display", DISPLAY, "-window", "root", path],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
                frames.append((time.time() - t0, phase, path))
    finally:
        for p in (phase_proc, launch):
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(8)
        for p in (phase_proc, launch):
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        xvfb.terminate()

    print(f"{len(frames)} frames")
    os.makedirs(dst, exist_ok=True)

    def last_of(ph):
        sel = [f for f in frames if f[1] == ph]
        return sel[len(sel) // 2] if sel else None

    for ph, name in (("HANDOVER", "rviz_handover.png"), ("FOLLOW", "rviz_follow.png")):
        f = last_of(ph)
        if f:
            Image.open(f[2]).convert("RGB").crop(RVIZ_BOX).save(os.path.join(dst, name))
            print("wrote", name, f"(t = {f[0]:.0f} s)")
    # GIF: the second half of the hand-over and the start of the follow, 4x speed, 800 px wide
    sel = [f for f in frames if f[1] in ("HANDOVER", "FOLLOW")][::2][:48]
    if sel:
        imgs = [Image.open(f[2]).convert("RGB").crop(VIEW_BOX).resize((560, 582))
                .quantize(colors=96, method=Image.Quantize.MEDIANCUT) for f in sel]
        imgs[0].save(os.path.join(dst, "rviz.gif"), save_all=True, append_images=imgs[1:], duration=250, loop=0,
                     optimize=True)
        print("wrote rviz.gif", len(imgs), "frames")


if __name__ == "__main__":
    main()
