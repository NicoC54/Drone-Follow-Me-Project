#!/usr/bin/env python3
"""Visual upgrades of the Gazebo worlds (idempotent, deterministic):

  1. spinning propellers: each rotor disc becomes a 1 g link on a revolute joint driven at a
     constant speed by gz-sim-joint-controller-system (visual only: 4 g and 1e-7 kg m^2 do not
     change the flight dynamics; the thrust is still computed by the flight-controller plugin);
  2. scenery (follow_me.sdf only): a small town and groves of trees, from a fixed seed. No hills:
     with a 300 m camera far clip they cannot fit outside the flight area (tested: the leader
     flew into one and the track was lost).

Why no scenery in intercept.sdf: the phase-4 detector finds a DARK SILHOUETTE by thresholding.
Tested in Gazebo with trees and hills: groups of trees on the horizon were taken for the target
(false camera lock, wrong learnt size, safety distance violated). A thresholding detector needs a
clean background; a cluttered horizon calls for a learnt detector (future work, see the course).

    python3 tools/gen_world_extras.py            # rewrites both worlds in place

Roof colours have a low saturation: the phase 2-3 detector looks for a RED beacon.
"""
import math
import random
import re
from pathlib import Path

WORLDS = Path(__file__).resolve().parent.parent / "ros2_ws/src/follow_me_gazebo/worlds"
BEGIN, END = "<!-- SCENERY (tools/gen_world_extras.py) -->", "<!-- /SCENERY -->"
ROTOR_SPEED = 40.0  # rad/s, visual only (the real ~900 rad/s would alias at 30 fps anyway)


def mat(r, g, b, a=1.0):
    c = f"{r:.2f} {g:.2f} {b:.2f} {a:.2f}"
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>"


# ------------------------------------------------------------------------------- propellers
CAMERA_HIDDEN = 0x2  # visibility flag of the follower's own blades: masked by its onboard camera


def rotor_block(model, xs, z, gravity):
    links, plugins = [], []
    # The follower's camera hangs below its rotors: its own blades would cross the image as dark
    # blobs (= false silhouettes). Hidden for the onboard camera only, still seen in the GUI.
    hide = f"<visibility_flags>{CAMERA_HIDDEN}</visibility_flags>" if model == "follower" else ""
    for k, (x, y) in enumerate(xs, start=1):
        name = f"rotor_{k}"
        links.append(f"""      <link name="{name}">
        <pose>{x} {y} {z} 0 0 0</pose>
        <gravity>{str(gravity).lower()}</gravity>
        <inertial><mass>0.001</mass>
          <inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <visual name="disc"><geometry><cylinder><radius>0.14</radius><length>0.006</length></cylinder></geometry>
          {mat(0.25, 0.25, 0.25, 0.35)}</visual>
        <visual name="blade"><pose>0 0 0.006 0 0 0</pose><geometry><box><size>0.27 0.026 0.004</size></box></geometry>
          {mat(0.05, 0.05, 0.05)}{hide}</visual>
        <visual name="hub"><geometry><cylinder><radius>0.02</radius><length>0.03</length></cylinder></geometry>
          {mat(0.6, 0.6, 0.62)}{hide}</visual>
      </link>
      <joint name="{name}_joint" type="revolute">
        <parent>base_link</parent><child>{name}</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>""")
        sign = 1 if k in (1, 2) else -1  # X quad: diagonal pairs spin in opposite directions
        if k in (2, 4):
            sign = -sign
        plugins.append(f"""      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>{name}_joint</joint_name>
        <initial_velocity>{sign * ROTOR_SPEED}</initial_velocity>
      </plugin>""")
    return "\n".join(links), "\n".join(plugins)


def upgrade_rotors(text, model, gravity):
    start = text.index(f'<model name="{model}">')
    end = text.index("</model>", start)
    body = text[start:end]
    if 'link name="rotor_1"' in body:
        return text  # already done
    pat = re.compile(r'\s*<visual name="rotor_\d"><pose>(\S+) (\S+) (\S+) 0 0 0</pose>.*?</visual>', re.S)
    found = pat.findall(body)
    assert len(found) == 4, (model, found)
    body = pat.sub("", body)
    links, plugins = rotor_block(model, [(x, y) for x, y, _ in found], found[0][2], gravity)
    i = body.index("</link>") + len("</link>")  # after base_link
    comment = "\n      <!-- Spinning propellers (visual only: 4 x 1 g; thrust comes from the flight model) -->\n"
    body = body[:i] + comment + links + body[i:] + plugins + "\n    "
    return text[:start] + body + text[end:]


# ------------------------------------------------------------------------------- scenery
def box(name, x, y, sx, sy, h, yaw, rgb, roof=None):
    roof_v = ""
    if roof:
        roof_v = (f'<visual name="roof"><pose>0 0 {h / 2 + 0.15:.2f} 0 0 0</pose><geometry><box><size>{sx + 0.4:.1f} '
                  f'{sy + 0.4:.1f} 0.3</size></box></geometry>{mat(*roof)}</visual>')
    return (f'    <model name="{name}"><static>true</static><pose>{x:.1f} {y:.1f} {h / 2:.2f} 0 0 {yaw:.2f}</pose>'
            f'<link name="link"><visual name="v"><geometry><box><size>{sx:.1f} {sy:.1f} {h:.1f}</size></box></geometry>'
            f'{mat(*rgb)}</visual>{roof_v}</link></model>')


def tree(name, x, y, h, rng):
    crown = rng.choice([(0.33, 0.47, 0.27), (0.36, 0.5, 0.28), (0.30, 0.44, 0.25)])
    trunk_h = 0.35 * h
    if rng.random() < 0.5:  # conifer
        crown_geo = f"<cone><radius>{0.28 * h:.2f}</radius><length>{0.75 * h:.2f}</length></cone>"
        cz = trunk_h + 0.75 * h / 2 - 0.1 * h
    else:  # broadleaf
        crown_geo = f"<sphere><radius>{0.32 * h:.2f}</radius></sphere>"
        cz = trunk_h + 0.25 * h
    return (f'    <model name="{name}"><static>true</static><pose>{x:.1f} {y:.1f} 0 0 0 0</pose><link name="link">'
            f'<visual name="trunk"><pose>0 0 {trunk_h / 2:.2f} 0 0 0</pose><geometry><cylinder><radius>{0.04 * h:.2f}'
            f'</radius><length>{trunk_h:.2f}</length></cylinder></geometry>{mat(0.42, 0.34, 0.25)}</visual>'
            f'<visual name="crown"><pose>0 0 {cz:.2f} 0 0 0</pose><geometry>{crown_geo}</geometry>{mat(*crown)}</visual>'
            f'</link></model>')


def hill(name, x, y, rx, ry, h, rgb):
    return (f'    <model name="{name}"><static>true</static><pose>{x:.0f} {y:.0f} 0 0 0 0</pose><link name="link">'
            f'<visual name="v"><geometry><ellipsoid><radii>{rx:.0f} {ry:.0f} {h:.0f}</radii></ellipsoid></geometry>'
            f'{mat(*rgb)}</visual></link></model>')


def scenery(seed, centre, keep_out, horizon):
    """keep_out: (xmin, xmax, ymin, ymax) of the flight area: nothing tall inside it."""
    rng = random.Random(seed)
    out = []
    cx, cy = centre

    def free(x, y, margin=0.0):
        x0, x1, y0, y1 = keep_out
        return not (x0 - margin < x < x1 + margin and y0 - margin < y < y1 + margin)

    # small town south of the flight area: a street grid, light facades, terracotta / grey roofs
    facades = [(0.86, 0.84, 0.78), (0.80, 0.78, 0.72), (0.90, 0.88, 0.84), (0.76, 0.74, 0.70)]
    roofs = [(0.45, 0.47, 0.50), (0.50, 0.44, 0.40), (0.38, 0.40, 0.44)]  # slate / brown, low saturation
    town_x, town_y = cx - 40, keep_out[2] - 130
    n = 0
    for i in range(6):
        for j in range(4):
            if rng.random() < 0.2:
                continue
            x = town_x + i * 28 + rng.uniform(-3, 3)
            y = town_y + j * 26 + rng.uniform(-3, 3)
            if not free(x, y, 20):
                continue
            h = rng.uniform(5, 16)
            out.append(box(f"building_{n}", x, y, rng.uniform(10, 16), rng.uniform(9, 14), h, rng.choice([0, 0.05, -0.05]),
                           rng.choice(facades), rng.choice(roofs)))
            n += 1
    # trees: scattered groves outside the flight area
    k = 0
    while k < 140:
        gx, gy = cx + rng.uniform(-480, 480), cy + rng.uniform(-420, 420)
        if not free(gx, gy, 40):
            continue
        for _ in range(rng.randint(3, 8)):
            x, y = gx + rng.gauss(0, 12), gy + rng.gauss(0, 12)
            if free(x, y, 30):
                out.append(tree(f"tree_{k}", x, y, rng.uniform(6, 13), rng))
                k += 1
    # hills on the horizon, inside the camera far clip. Not in the phase-4 world: the target is
    # seen near the horizon, and a dark hill behind it ruins the silhouette detector.
    for m in range(9 if horizon else 0):
        a = 2 * 3.14159 * m / 9 + rng.uniform(-0.2, 0.2)
        r = rng.uniform(0.75, 0.95) * horizon
        out.append(hill(f"hill_{m}", cx + r * math.cos(a), cy + r * math.sin(a), rng.uniform(120, 220),
                        rng.uniform(90, 160), rng.uniform(25, 60), (0.40, 0.50, 0.36)))
    return "\n".join(out)


def mask_onboard_camera(text):
    if "<visibility_mask>" in text:
        return text
    mask = 0xFFFFFFFF & ~CAMERA_HIDDEN
    return re.sub(r'(<sensor name="camera" type="camera">.*?<camera>)',
                  lambda m: m.group(1) + f"\n            <visibility_mask>{mask}</visibility_mask>", text, count=1, flags=re.S)


def upgrade_scenery(text, block):
    text = re.sub(re.escape(BEGIN) + r".*?" + re.escape(END) + r"\n?", "", text, flags=re.S)
    anchor = "    <!-- ===================== LEADER"
    return text.replace(anchor, f"    {BEGIN}\n{block}\n    {END}\n\n{anchor}", 1)


def main():
    # follow_me.sdf: phases 1-3 around the origin, camera far clip 300 m
    # intercept.sdf: phase 4 engagement between the radar (-60,-40) and the target start (330,260)
    for fname, seed, centre, keep_out, horizon in (
            ("follow_me.sdf", 11, (0, 60), (-120, 120, -60, 200), None),
            ("intercept.sdf", None, None, None, None)):
        p = WORLDS / fname
        t = p.read_text()
        t = upgrade_rotors(t, "leader", gravity=False)
        t = upgrade_rotors(t, "follower", gravity=True)
        if seed is not None:
            t = upgrade_scenery(t, scenery(seed, centre, keep_out, horizon))
        t = mask_onboard_camera(t)
        p.write_text(t)
        print("updated", p)


if __name__ == "__main__":
    main()
