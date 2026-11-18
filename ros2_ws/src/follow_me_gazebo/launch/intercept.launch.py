"""PHASE 4: radar-cued interception of an unknown drone, then camera follow.

    ros2 launch follow_me_gazebo intercept.launch.py              # Gazebo GUI + RViz
    ros2 launch follow_me_gazebo intercept.launch.py gui:=false rviz:=false

Chain: radar_node (ground radar) -> fusion_node (radar + camera, out-of-sequence replay)
       -> intercept_node (WAIT / MIDCOURSE to the PIP / HANDOVER / FOLLOW) -> flight controller.
       detector_node runs in "silhouette" mode inside the window cued by fusion_node.
(The phase-3 follow scenario with the red beacon stays available: follow_me.launch.py.)
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gz = get_package_share_directory("follow_me_gazebo")
    pkg_ros = get_package_share_directory("follow_me_ros")
    world = os.path.join(pkg_gz, "worlds", "intercept.sdf")
    bridge_cfg = os.path.join(pkg_gz, "config", "bridge.yaml")
    rviz_cfg = os.path.join(pkg_gz, "rviz", "follow_me.rviz")
    params = os.path.join(pkg_ros, "config", "follow_me.yaml")
    params_intercept = os.path.join(pkg_ros, "config", "intercept.yaml")  # phase-4 overrides

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    foxglove = LaunchConfiguration("foxglove")
    common = [params, params_intercept, {"use_sim_time": True}]
    inject = {"inject.p_miss": LaunchConfiguration("p_miss"),
              "inject.p_outlier": LaunchConfiguration("p_outlier"),
              "inject.latency_ms": LaunchConfiguration("latency_ms")}

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={
            # -r: run immediately; -s: server only (no GUI)
            "gz_args": PythonExpression(["'-r ", world, "' if '", gui, "' == 'true' else '-r -s ", world, "'"]),
            "on_exit_shutdown": "true",
        }.items(),
    )

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
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("foxglove", default_value="false",
                              description="Start foxglove_bridge (ws://localhost:8765; apt install ros-jazzy-foxglove-bridge)"),
        DeclareLaunchArgument("p_miss", default_value="0.0", description="Injected miss probability"),
        DeclareLaunchArgument("p_outlier", default_value="0.0", description="Injected false detection probability"),
        DeclareLaunchArgument("latency_ms", default_value="0", description="Injected detector processing delay [ms]"),
        gz_sim,
        bridge,
        node("leader_node", {"duration": 120.0}),
        node("radar_node"),
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
