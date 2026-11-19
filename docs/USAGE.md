# Usage

Back to the [README](../README.md). Results and figures: [RESULTS.md](RESULTS.md).

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

# phase 4: radar-cued interception of an unknown drone
ros2 launch follow_me_gazebo intercept.launch.py                 # Gazebo + RViz (+ foxglove:=true)
ros2 launch follow_me_gazebo intercept.launch.py target:=octo target_scale:=0.7   # chosen target
ros2 launch follow_me_gazebo intercept.launch.py target_path:=random               # random flight plan + heading
ros2 launch follow_me_gazebo intercept.launch.py target_path:=random target_maneuvers:=2 target_seed:=7  # aggressive, replayable
./build/follow_me_core/follow_me_offline_sim --intercept --seed 2 --out ../out
./build/follow_me_core/follow_me_offline_sim --intercept --montecarlo 300 --seed 7 --out ../out
./build/follow_me_core/follow_me_offline_sim --intercept --montecarlo 300 --maneuvers 2 --out ../out/maneuvers/L2
./build/follow_me_core/follow_me_offline_sim --intercept --maneuvers 2 --mc-run 17 --out ../out   # replay one run
tools/diag_record.py out/diag.npz && tools/diag_analyse.py out/diag.npz   # Gazebo: truth vs detections
```

### Figures, GIF, web replay, Foxglove

```bash
python3 tools/make_figures.py out/ docs/media/      # README figures + GIF + docs/replay/replay.json
python3 -m http.server -d docs 8000                 # then http://localhost:8000/replay/
tools/record_demo.sh out/demo                       # MCAP recording of a running launch (Ctrl+C)
python3 tools/gen_world_extras.py                   # regenerate propellers + scenery in the worlds
```

### Several radars, experiments

```bash
ros2 launch follow_me_gazebo intercept.launch.py radars:=3          # 1-3 ground radars (masts spawned)
./build/follow_me_core/follow_me_offline_sim --intercept --montecarlo 300 --set radar.count=3 --out ../out/r3
# any algorithm variant, on the same random draws as the reference:
./build/follow_me_core/follow_me_offline_sim --intercept --montecarlo 100 \
    --set fusion.model=imm_singer --set intercept.a_ff_gain=1 --out ../out/ff
```
Keys: `fusion.model` (imm_cv, imm_singer, singer, cv), `fusion.q_calm`, `fusion.q_jerk`,
`fusion.singer_tau`, `fusion.q_accel`, `intercept.a_ff_gain`, `intercept.follow_a_max`,
`intercept.follow_jerk_max`, `radar.count` (1-3), `radar.third` (north, center).

### Demo video

```bash
ros2 launch follow_me_gazebo intercept.launch.py chase_cam:=true target:=hexa target_scale:=1.3 gui:=false rviz:=false
tools/record_video.py docs/media/demo.mp4      # chase camera + onboard HUD, H.264, ~1 min
```

RViz screenshots and GIF, in a virtual display (nothing opens on screen; `sudo apt install xvfb`):
```bash
tools/capture_rviz.py docs/media     # rviz_handover.png, rviz_follow.png, rviz.gif (~2.5 min)
# README GIFs: one flight, 4 synchronised views (~2 min each)
tools/record_grid.py record out/demo1 target:=quad_x target_scale:=0.8 target_path:=random target_seed:=54 radars:=3
tools/record_grid.py record out/demo2 target:=octo target_scale:=1.5 target_path:=random target_maneuvers:=2 target_seed:=107 radars:=3
tools/record_grid.py compose out/demo1 docs/media/demo1.gif --title "Situation 1/2: ..." --start 22 --end 65.5 --fps 20 --tile 480x270 --colors 80
tools/record_grid.py compose out/demo2 docs/media/demo2.gif --title "Situation 2/2: ..." --start 17 --end 53 --fps 20 --tile 480x270 --colors 80
```

- RViz shows the algorithms: predicted intercept point, approach line, predicted target path,
  camera field of view, 3σ ellipsoids of the estimate and of every radar plot.
- The camera panel shows a HUD (`hud_node`): mission phase, range, track source, learnt size, lock.
- Foxglove Studio: `sudo apt install ros-jazzy-foxglove-bridge`, launch with `foxglove:=true`, open
  `ws://localhost:8765` and import [docs/foxglove/follow_me_layout.json](foxglove/follow_me_layout.json)
  (also works on an MCAP recording).

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

## The four phases

The project grew in four phases (each one documented in the [course](cours/00_INDEX.md), French):
1. Python sandbox: camera on a red beacon, EKF, guidance, control, safety filter.
2. C++ / ROS 2 / Gazebo: the same algorithms on real rendered images.
3. Real quadrotor: own flight controller (Gazebo plugin) + 15-state ESKF navigation.
4. Ground radar + predicted-intercept-point guidance + camera hand-over on an unknown drone.

The original phase-1 chain (Python, `follow_me/`), kept deliberately small:

```
truth (leader + follower)                                    vehicles.py
   -> camera: pixel position + apparent size, noise,         camera.py
      missed and false detections
   -> EKF on the leader [p, v] + gating + track management   estimation.py
   -> guidance: follow point 8 m behind / 2 m above          guidance.py
   -> control: cascaded P/PI + separation safety filter      control.py
   -> follower point-mass dynamics (lag, drag, wind, gusts)  vehicles.py
```
Learning sheets (French): [UNDERSTANDING.md](UNDERSTANDING.md) (GNC principles),
[UNDERSTANDING_ROS.md](UNDERSTANDING_ROS.md) (C++ / ROS 2 / Gazebo),
[UNDERSTANDING_FLIGHT.md](UNDERSTANDING_FLIGHT.md) (quadrotor, flight controller, navigation).
