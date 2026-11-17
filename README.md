# drone-follow-me

Teaching simulation: a follower drone tracks a leader drone in the air at a safe distance,
using only a gimbal camera. The whole GNC chain is there and kept deliberately small:

```
truth (leader + follower)                                    vehicles.py
   -> camera: pixel position + apparent size, noise,         camera.py
      missed and false detections
   -> EKF on the leader [p, v] + gating + track management   estimation.py
   -> guidance: follow point 8 m behind / 2 m above          guidance.py
   -> control: cascaded P/PI + separation safety filter      control.py
   -> follower point-mass dynamics (lag, drag, wind, gusts)  vehicles.py
```

Two implementations:
- **Python** (`follow_me/`): compact reference and sandbox, matplotlib plots.
- **C++ / ROS 2 Jazzy / Gazebo Harmonic** (`ros2_ws/`): the same algorithms in a C++ library,
  wrapped in ROS 2 nodes, with real camera images rendered by Gazebo. The follower is a 6-DOF
  quadrotor flown by its own flight controller (Gazebo plugin: 15-state ESKF navigation on
  IMU/GNSS/baro/mag + attitude/rate control + mixer).
  See [docs/UNDERSTANDING_ROS.md](docs/UNDERSTANDING_ROS.md) and
  [docs/UNDERSTANDING_FLIGHT.md](docs/UNDERSTANDING_FLIGHT.md).

Learning sheets (French): [docs/UNDERSTANDING.md](docs/UNDERSTANDING.md) (Python, GNC principles),
[docs/UNDERSTANDING_ROS.md](docs/UNDERSTANDING_ROS.md) (C++ / ROS 2 / Gazebo),
[docs/UNDERSTANDING_FLIGHT.md](docs/UNDERSTANDING_FLIGHT.md) (quadrotor, flight controller, navigation).

The Python loop is in `follow_me/sim.py` and every parameter is in `follow_me/config.py`.
Design choices and their reasons are in [DESIGN.md](DESIGN.md).

## Run

Requirements: Python 3.10+, numpy, matplotlib (pytest for the tests).

```bash
python -m follow_me                       # metrics + 3 figures
python -m follow_me --animate             # + top-view animation
python -m follow_me --occlusion 40 46     # blind the camera: COAST -> SEARCH -> reacquire
python -m follow_me --seed 3 --no-plots --save out/   # headless, figures to out/
python -m pytest tests
```

## Run the C++ / ROS 2 / Gazebo version

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build && colcon test && colcon test-result --verbose
source install/setup.bash
ros2 launch follow_me_gazebo follow_me.launch.py                 # Gazebo + RViz
ros2 launch follow_me_gazebo follow_me.launch.py p_miss:=0.1 p_outlier:=0.02
./build/follow_me_core/follow_me_offline_sim --seed 3            # C++ offline sim, no ROS
./build/follow_me_core/follow_me_offline_sim --perfect-nav       # same, with perfect navigation
```

## What to look at (Python)

| Figure | What it teaches |
|---|---|
| Overview, separation | distance vs setpoint and `d_min`; grey/yellow bands = SEARCH/COAST |
| Estimation, errors vs ±3σ | is the EKF consistent? (errors should stay inside the bounds, NEES ≈ 6) |
| Estimation, NIS | the chi2 gate separating real detections from false ones |
| Camera & control, image plane | where the leader sits in the image, outliers |
| Camera & control, a_cmd | how estimation noise propagates into the commands |

## Experiments to try

- `estimator.q_accel`: 0.3 vs 1.5 vs 10. Lag in turns vs noisy commands.
- `camera.sigma_size_px` × 5: range becomes poor; watch the ±3σ bands and the distance error.
- `control.ki = 0`: steady lag behind a moving leader (drag + wind).
- `guidance.follow_dist = 5`: the safety filter starts working in the turns and stops.
- `estimator.gate_chi2 = 1e9` (no gating): what the outliers do to the estimate.
- `--occlusion 70 73` during the sharp turn: how fast the constant-velocity prediction diverges.
