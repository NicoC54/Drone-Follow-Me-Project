# Drone Follow-Me: radar-cued interception of an unknown drone

[![CI](https://github.com/NicoC54/Drone-Follow-Me-Project/actions/workflows/ci.yml/badge.svg)](https://github.com/NicoC54/Drone-Follow-Me-Project/actions/workflows/ci.yml)
![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C) ![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E) ![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)

A ground radar picks up an **unknown drone** 480 m away. A quadrotor flies to the predicted
intercept point, hands over to its own camera, **learns the target's size in flight**, then follows
it at a safe distance. Everything between the sensors and the motors is written here, in C++.

![Gazebo: radar track only, then camera hand-over, then follow](docs/media/demo.gif)

*Gazebo, 15 s: chase camera (left), onboard camera with HUD (right). Mid-course on the radar
track, hand-over to the camera while learning the target's size, then follow at 8 m.
**▶ [Full 1-minute video](docs/media/demo.mp4)**.*

| Top view of the whole engagement | RViz: what the algorithms see |
|---|---|
| ![Radar-cued interception then camera follow](docs/media/intercept.gif) | ![RViz during the hand-over](docs/media/rviz.gif) |
| Follower (blue) flies to the predicted intercept point, then settles 8 m behind the target (red). | Target **estimate** and its 3σ ellipsoid (orange: radar only, green: radar + camera), aim point, camera field of view, true distance (green → red near 4 m). |

**[Interactive 3D replay](https://nicoc54.github.io/Drone-Follow-Me-Project/replay/)** ·
**[Results](docs/RESULTS.md)** · **[Usage](docs/USAGE.md)** · **[Design decisions](DESIGN.md)** ·
**[Course (FR)](docs/cours/00_INDEX.md)**

## What is built from scratch

- **Flight**: 6-DOF quadrotor, flight controller (thrust vector, quaternion attitude, PI rates,
  mixer with desaturation) as a Gazebo plugin at 500 Hz, **15-state ESKF** on IMU/GNSS/baro/mag.
- **Tracking**: **1-3 ground radars** (trilateration) + camera fusion with **exact out-of-sequence replay**, own-echo rejection,
  M-of-N track start, **IMM**, unknown target size learnt online, chi2 gating.
- **Guidance**: mission state machine, **predicted-intercept-point** guidance, hand-over driven by
  range uncertainty, follow guidance with bumpless entry, **CBF-like safety filter** on the
  estimate's uncertainty.
- **Vision**: silhouette detector in the radar-cued window, aspect-invariant apparent size.
- **Validation**: 51 tests, offline simulator (a run in ~0.1 s), **Monte-Carlo campaigns**,
  random target flights in Gazebo too (one seed replays a run), ground-truth diagnosis tools.

## Key results

| | |
|---|---|
| 300 randomised interceptions (offline) | **300/300** reach follow, **0** below 4 m |
| Random target manoeuvres, moderate / aggressive | 4 / 49 runs out of 300 below 4 m |
| Gazebo, physical target (rendered images) | estimate error **0.49 m**, min distance 5.0 m |
| Gazebo phase-4 gap, after ground-truth diagnosis | error 1.36 → 0.26 m, NEES 37 → 7.5 |
| 1 → 3 ground radars (trilateration), radar-phase error | horizontal 2.4 → **0.9 m**, vertical 3.2 → **1.5 m** |

## Gallery

| | |
|---|---|
| ![Distance to the target](docs/media/intercept_distance.png) | ![Monte-Carlo](docs/media/montecarlo.png) |
| **Distance** (log scale): 480 m → 8 m standoff, never below the 4 m minimum. Bands = mission phases. | **300 randomised interceptions**: minimum distance, time to follow, follow quality. |

**More ground radars, better radar phase.** A radar measures range precisely (2 m) but angles
coarsely (0.5°, 0.8°). Sites seen from different directions cross their ranges, which removes the
angle error. The gain matters until the camera takes over, not after:

![Error along the target's path with 1, 2, 3 radars](docs/media/radar_gain.png)

Estimation details, random manoeuvres, radar maps, every radar plot: [docs/RESULTS.md](docs/RESULTS.md).

## Architecture

```mermaid
flowchart LR
  subgraph Sensors
    R[Ground radar<br/>2 Hz, 0.3 s late]
    C[Gimbal camera<br/>Gazebo images]
  end
  R -->|plots| F
  C --> D[Silhouette detector<br/>cued window]
  D -->|bearing + size| F[FusionTracker<br/>IMM, OOSM replay,<br/>size learning]
  F -->|window| D
  F -->|target estimate + covariance| M[Mission<br/>PIP / hand-over / follow]
  M --> S[Safety filter<br/>3 sigma, relative braking]
  S -->|acceleration + yaw| FC[Flight controller<br/>Gazebo plugin, 500 Hz]
  N[ESKF navigation] --> FC
  N --> F
  FC --> Q[Quadrotor 6-DOF]
```

The algorithms live in a ROS-free C++ library (`ros2_ws/src/follow_me_core`), used by the
offline simulator, the unit tests and the ROS 2 nodes alike.

## Quick start

```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build && colcon test && colcon test-result
source install/setup.bash
ros2 launch follow_me_gazebo intercept.launch.py                     # Gazebo + RViz, random target model
ros2 launch follow_me_gazebo intercept.launch.py radars:=3           # three ground radars
ros2 launch follow_me_gazebo intercept.launch.py target_path:=random # random target flight
./build/follow_me_core/follow_me_offline_sim --intercept --montecarlo 300 --out ../out   # no ROS
```

All commands (phase 1-3 scenarios, figures, Foxglove, diagnosis tools): [docs/USAGE.md](docs/USAGE.md).
