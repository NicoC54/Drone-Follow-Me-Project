"""Bring up the whole follow-me simulation.

    ros2 launch follow_me_gazebo follow_me.launch.py              # Gazebo GUI + RViz
    ros2 launch follow_me_gazebo follow_me.launch.py gui:=false   # Gazebo server only
    ros2 launch follow_me_gazebo follow_me.launch.py rviz:=false
    ros2 launch follow_me_gazebo follow_me.launch.py p_outlier:=0.02 p_miss:=0.1   # fault injection
    ros2 launch follow_me_gazebo follow_me.launch.py latency_ms:=300               # slow detector
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
    world = os.path.join(pkg_gz, "worlds", "follow_me.sdf")
    bridge_cfg = os.path.join(pkg_gz, "config", "bridge.yaml")
    rviz_cfg = os.path.join(pkg_gz, "rviz", "follow_me.rviz")
    params = os.path.join(pkg_ros, "config", "follow_me.yaml")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    common = [params, {"use_sim_time": True}]
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
        DeclareLaunchArgument("p_miss", default_value="0.0", description="Injected miss probability"),
        DeclareLaunchArgument("p_outlier", default_value="0.0", description="Injected false detection probability"),
        DeclareLaunchArgument("latency_ms", default_value="0", description="Injected detector processing delay [ms]"),
        gz_sim,
        bridge,
        node("leader_node"),
        node("detector_node", inject),
        node("tracker_node"),
        node("follow_node"),
        node("monitor_node"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             parameters=[{"use_sim_time": True}], condition=IfCondition(rviz), output="log"),
        follow_cam,
    ])
