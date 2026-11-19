"""PHASE 4: radar-cued interception of an unknown drone, then camera follow.

    ros2 launch follow_me_gazebo intercept.launch.py              # Gazebo GUI + RViz
    ros2 launch follow_me_gazebo intercept.launch.py gui:=false rviz:=false
    ros2 launch follow_me_gazebo intercept.launch.py target:=hexa target_scale:=1.5
    ros2 launch follow_me_gazebo intercept.launch.py target_seed:=4    # reproducible random target
    ros2 launch follow_me_gazebo intercept.launch.py target_path:=random              # random flight
    ros2 launch follow_me_gazebo intercept.launch.py target_path:=random target_maneuvers:=2 target_seed:=7
    ros2 launch follow_me_gazebo intercept.launch.py radars:=3         # three ground radars

Chain: radar_node (ground radar) -> fusion_node (radar + camera, out-of-sequence replay)
       -> intercept_node (WAIT / MIDCOURSE to the PIP / HANDOVER / FOLLOW) -> flight controller.
       detector_node runs in "silhouette" mode inside the window cued by fusion_node.
(The phase-3 follow scenario with the red beacon stays available: follow_me.launch.py.)
"""
import math
import os
import random
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import target_models  # noqa: E402  (installed next to this file)


# Chase camera for the demo video (tools/record_video.py): 5 m behind and 1.5 m above the
# follower, rigidly attached to its body (which yaws towards the target). Visualisation only.
CHASE_CAMERA = """
        <sensor name="chase_camera" type="camera">
          <pose>-5.0 0 1.5 0 0.12 0</pose>
          <always_on>1</always_on>
          <update_rate>15</update_rate>
          <topic>/follower/chase_camera/image</topic>
          <camera>
            <horizontal_fov>1.2</horizontal_fov>
            <image><width>640</width><height>360</height><format>R8G8B8</format></image>
            <clip><near>0.2</near><far>800</far></clip>
          </camera>
        </sensor>
"""


def add_chase_camera(world_text):
    i = world_text.index('<model name="follower">')
    j = world_text.index("</link>", i)  # end of the follower's base_link
    return world_text[:j] + CHASE_CAMERA + "      " + world_text[j:]


# Ground radar sites, = follow_me_core InterceptConfig (main radar) + secondRadarSite + thirdRadarSite("north").
RADAR_SITES = [(-60.0, -40.0, 3.0), (450.0, -100.0, 3.0), (230.0, 420.0, 3.0)]
MAST_SDF = """<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static><link name="link">
  <visual name="mast"><pose>0 0 1.5 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>3</length></cylinder></geometry>
    <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
  <visual name="antenna"><pose>0 0 3.2 0 0.3 0.6</pose><geometry><box><size>0.1 1.6 1.0</size></box></geometry>
    <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>
</link></model></sdf>"""


def spawn_radars(context, common):
    """radars:=1|2|3 -> one radar_node per site (interleaved scans) + a mast for each extra site."""
    n = max(1, min(3, int(LaunchConfiguration("radars").perform(context))))
    actions = [LogInfo(msg=f"[radars] {n} ground radar(s): " + ", ".join(str(p[:2]) for p in RADAR_SITES[:n]))]
    for i in range(n):
        x, y, z = RADAR_SITES[i]
        actions.append(Node(
            package="follow_me_ros", executable="radar_node", name="radar_node" if i == 0 else f"radar_node_{i}",
            output="screen",
            parameters=common + [{"radar.position": [x, y, z], "seed": 3 + 10 * i, "radar_id": i,
                                  "scan_offset": i * 0.5 / n}]))
        if i > 0:  # the main mast is in intercept.sdf
            actions.append(Node(package="ros_gz_sim", executable="create", output="log",
                                arguments=["-world", "follow_me", "-name", f"radar_mast_{i}",
                                           "-string", MAST_SDF.format(name=f"radar_mast_{i}"),
                                           "-x", str(x), "-y", str(y), "-z", "0"]))
    return actions


# Start of the target (= follow_me_core InterceptConfig): ~420 m from the follower.
TARGET_START = (330.0, 260.0, 40.0)
TARGET_HEADING_DEG = 200.0          # scripted flight plan: nearly head-on
HEADING_TO_FOLLOWER_DEG = 218.0     # bearing from the target start to the follower start
HEADING_OFF_DEG = (35.0, 100.0)     # random flight: ALWAYS oblique, 35-100 deg off head-on on either
                                    # side (head-on excluded: it forces a turn-around, journal 27;
                                    # fully fleeing excluded: 2 m/s of speed margin, > 120 s to catch up)


def spawn_target(context, common):
    """Spawns the unknown target drone (random or chosen shape and size) and its pilot
    (scripted or random flight plan). ONE seed draws everything: a run can be replayed."""
    seed_arg = LaunchConfiguration("target_seed").perform(context)
    seed = int(seed_arg) if seed_arg not in ("", "random") else time.time_ns() % 1_000_000
    kind, scale = target_models.choose(LaunchConfiguration("target").perform(context),
                                       LaunchConfiguration("target_scale").perform(context), seed)
    physics = LaunchConfiguration("target_physics").perform(context).lower() in ("true", "1")
    path = LaunchConfiguration("target_path").perform(context)
    level = int(LaunchConfiguration("target_maneuvers").perform(context)) if path == "random" else 0
    heading = TARGET_HEADING_DEG
    if level > 0:
        rng = random.Random(seed)
        off = rng.uniform(*HEADING_OFF_DEG) * rng.choice((-1.0, 1.0))
        heading = (HEADING_TO_FOLLOWER_DEG + off) % 360.0
    sdf = target_models.target_sdf(kind, scale, physics)
    x, y, z = TARGET_START
    return [
        LogInfo(msg=f"[target] seed {seed}: {kind}, scale {scale:.2f}, span ~{target_models.span(kind, scale):.2f} m, "
                    f"{'quadrotor dynamics' if physics else 'kinematic'}, "
                    f"{'random flight (level %d)' % level if level else 'scripted flight'}, heading {heading:.0f} deg "
                    f"(unknown to the tracker: watch the learnt size W)"),
        Node(package="ros_gz_sim", executable="create", output="log",
             arguments=["-world", "follow_me", "-name", "leader", "-string", sdf,
                        "-x", str(x), "-y", str(y), "-z", str(z), "-Y", f"{math.radians(heading):.4f}"]),
        Node(package="follow_me_ros", executable="leader_node", name="leader_node", output="screen",
             parameters=common + [{"duration": 120.0, "physics": physics, "maneuvers": level,
                                   "maneuver_seed": seed, "start_heading_deg": heading}]),
    ]


def generate_launch_description():
    pkg_gz = get_package_share_directory("follow_me_gazebo")
    pkg_ros = get_package_share_directory("follow_me_ros")
    world = os.path.join(pkg_gz, "worlds", "intercept.sdf")
    bridge_cfg = os.path.join(pkg_gz, "config", "bridge.yaml")
    rviz_cfg = LaunchConfiguration("rviz_config")
    params = os.path.join(pkg_ros, "config", "follow_me.yaml")
    params_intercept = os.path.join(pkg_ros, "config", "intercept.yaml")  # phase-4 overrides

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    foxglove = LaunchConfiguration("foxglove")
    common = [params, params_intercept, {"use_sim_time": True}]
    inject = {"inject.p_miss": LaunchConfiguration("p_miss"),
              "inject.p_outlier": LaunchConfiguration("p_outlier"),
              "inject.latency_ms": LaunchConfiguration("latency_ms")}

    def start_gazebo(context):
        """Gazebo with intercept.sdf, or with a copy that adds a chase camera (chase_cam:=true)."""
        world_file = world
        if LaunchConfiguration("chase_cam").perform(context).lower() in ("true", "1"):
            world_file = os.path.join(tempfile.gettempdir(), "follow_me_intercept_chase.sdf")
            with open(world, encoding="utf-8") as f:
                text = f.read()
            with open(world_file, "w", encoding="utf-8") as f:
                f.write(add_chase_camera(text))
        flags = "-r " if LaunchConfiguration("gui").perform(context) == "true" else "-r -s "
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            # -r: run immediately; -s: server only (no GUI)
            launch_arguments={"gz_args": flags + world_file, "on_exit_shutdown": "true"}.items())]

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
                  parameters=[{"config_file": bridge_cfg, "use_sim_time": True}], output="screen")

    def node(name, extra=None):
        return Node(package="follow_me_ros", executable=name, name=name, output="screen",
                    parameters=common + ([extra] if extra else []))

    # Make the Gazebo GUI camera follow the follower drone.
    follow_cam = TimerAction(period=6.0, condition=IfCondition(gui), actions=[ExecuteProcess(
        cmd=["gz", "service", "-s", "/gui/follow", "--reqtype", "gz.msgs.StringMsg",
             "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", 'data: "follower"'],
        output="log")])

    # The follower's flight-controller plugin (libfollow_me_flight_controller.so).
    plugin_path = AppendEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH", os.path.join(get_package_prefix("follow_me_gazebo"), "lib"))

    return LaunchDescription([
        plugin_path,
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(pkg_gz, "rviz", "follow_me.rviz"),
                              description="RViz configuration (record_grid.py: a zoomed-out copy)"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("target", default_value="random", description="Target drone: quad_x, hexa, octo or random"),
        DeclareLaunchArgument("target_scale", default_value="random", description="Target scale (1.0 = 1.1 m quad) or random (0.6-1.8)"),
        DeclareLaunchArgument("target_seed", default_value="random",
                              description="Seed of the random target: shape, size, flight plan, heading (reproducible runs)"),
        DeclareLaunchArgument("target_path", default_value="scripted",
                              description="Target flight: scripted (reference flight) or random"),
        DeclareLaunchArgument("target_maneuvers", default_value="1",
                              description="Random flight: 1 moderate, 2 aggressive (offline Monte-Carlo levels)"),
        DeclareLaunchArgument("radars", default_value="1",
                              description="Number of ground radars (1-3): trilateration with 2-3 sites"),
        DeclareLaunchArgument("chase_cam", default_value="false",
                              description="Add a chase camera behind the follower (demo video)"),
        DeclareLaunchArgument("target_physics", default_value="true",
                              description="true: the target is a quadrotor flown by its flight controller; false: kinematic"),
        DeclareLaunchArgument("foxglove", default_value="false",
                              description="Start foxglove_bridge (ws://localhost:8765; apt install ros-jazzy-foxglove-bridge)"),
        DeclareLaunchArgument("p_miss", default_value="0.0", description="Injected miss probability"),
        DeclareLaunchArgument("p_outlier", default_value="0.0", description="Injected false detection probability"),
        DeclareLaunchArgument("latency_ms", default_value="0", description="Injected detector processing delay [ms]"),
        OpaqueFunction(function=start_gazebo),
        OpaqueFunction(function=spawn_target, args=[common]),
        bridge,
        OpaqueFunction(function=spawn_radars, args=[common]),
        node("detector_node", inject),
        node("fusion_node"),
        node("intercept_node"),
        node("monitor_node"),
        node("hud_node"),
        # Foxglove Studio: open ws://localhost:8765 and import docs/foxglove/follow_me_layout.json
        Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
             parameters=[{"use_sim_time": True, "port": 8765}], condition=IfCondition(foxglove), output="log"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             parameters=[{"use_sim_time": True}], condition=IfCondition(rviz), output="log"),
        follow_cam,
    ])
