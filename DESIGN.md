# Drone follow-me: design log

Learning project. Architecture questions first, answers and corrections recorded here, then code.

## Scope (phase 1, MVP)
- ~~Ground subject (person or bike)~~ **Airborne leader drone** moving with turns, speed changes, climbs, descents and a hover.
- 3D point-mass follower drone, gimbal camera measuring the leader's pixel position and apparent size (noise, missed detections, false detections).
- EKF estimating the leader's position and velocity.
- Controller keeping the follower at a set distance and height from the leader, gimbal keeping it centered, safety filter enforcing a minimum separation.
- Metrics: estimation error, placement error, time with the leader in the camera frame, minimum separation.

## Decisions

### D1. Target is a drone in the air, not a ground subject (2026-11-17)
A ground subject gives range "for free" (intersect the line of sight with the ground plane).
An airborne target does not: a bearing-only camera cannot observe range.
**Choice:** the leader has a known size (0.5 m); the detector returns the bounding-box size `s`,
so `depth = fx * W / s`. Consequence worth seeing in the plots: range noise grows as depth^2
(`sigma_depth = depth^2 * sigma_s / (fx W)`), so range is always the weak axis of the estimate.

### D2. Frames and conventions
World ENU (x East, y North, z Up). Camera frame x right, y down, z optical axis.
The gimbal is stabilised and its yaw/pitch are expressed directly in the world frame
(the follower's attitude is not modelled, see D4).

### D3. Measurement model `z = [u, v, s]`, EKF state `x = [p, v]` of the leader (world frame)
- Constant-velocity process model with white acceleration noise `q` (tuning knob: higher q =
  more reactive in turns but noisier velocity, which then shows up in the control commands).
- Analytic Jacobian `H = J_proj @ R_cw` (unit tested against finite differences).
- Joseph-form covariance update.
- Chi2 gating on the innovation (99.9 %, 3 dof) rejects false detections.
- Track management: SEARCH -> TRACK -> COAST (no update for 0.5 s) -> SEARCH (3 s).
  A track starts only when two detections within 0.3 s agree (chi2 test): a single false
  detection in SEARCH used to start a ghost track and the follower flew away chasing it.
  After 10 consecutive rejections the track is dropped (the filter is wrong, not the sensor).
- The follower's own position/velocity is assumed perfectly known (GNSS/INS). Next step: noisy self-state.

### D4. Follower dynamics: point mass driven by an acceleration command
`a_cmd` is the net acceleration (gravity compensated). The attitude loop is a first-order lag
(0.15 s). Disturbances: linear drag on airspeed with a mean wind + Ornstein-Uhlenbeck gusts.
No yaw/attitude, no rotor model: that belongs to a later phase.

### D5. Guidance: a follow point behind the leader
Follow point = leader + 8 m behind (along the leader's horizontal velocity) + 2 m above.
- When the leader slows below ~2 m/s its heading becomes noise, so "behind" blends towards the
  current bearing leader -> follower (stop = keep your place around it).
- The bearing of the follow point goes through a first-order filter with a rate limit (45 deg/s).
  A pure rate limit (bang-bang) was tried first: its omega jumped by +/-45 deg/s at every step
  and, fed forward in the velocity setpoint, saturated the accelerations. Fixed.
- Velocity setpoint = leader velocity + velocity of the rotating offset point (feed-forward).

### D6. Control: cascaded P (position) / PI (velocity)
Integral term removes the steady lag caused by drag and wind. Gains give a ~1.4 rad/s, zeta ~0.9 loop.

### D7. Safety: a separate filter after the controller
Works on the estimate, made conservative by `k_sigma` standard deviations along the line of sight.
Inside a buffer zone (d_min + 3 m) it removes the command component towards the leader, then adds
repulsion + damping of the closing speed. Separation from performance: the controller can be
re-tuned without touching the safety guarantee. Not a formal guarantee (no CBF), see next steps.

### D8. Gimbal-lost behaviour
SEARCH: hover in place and sweep the gimbal yaw at 40 deg/s. Simple and safe, but the leader keeps
flying away (see `--occlusion 70 73`: reacquired at 80 m). Next step: search around the last
predicted position, or keep flying the last velocity for a while.

## Phase 2: C++ / ROS 2 Jazzy / Gazebo Harmonic (2026-11-17)

### D9. Port to C++, ROS 2 Jazzy and Gazebo Harmonic
User choice: Gazebo over MuJoCo "to use ROS 2", and C++ over Python (much stronger in C++).
The Python package stays as the reference implementation and fast sandbox.

### D10. Pure C++ core + thin ROS nodes (PX4-style)
`follow_me_core` (Eigen only) holds every algorithm; `follow_me_ros` nodes only convert messages.
The core is unit tested without ROS, including an EKF step that matches the Python numbers to 1e-9,
and a C++ offline closed-loop sim whose metrics match the Python sim.

### D11. Gazebo is the truth, drones are kinematic
VelocityControl + gravity off: Gazebo integrates positions and publishes odometry. The follower's
acceleration dynamics (lag, drag, wind, gusts) live in `vehicle_node`, which is where PX4 SITL would
plug in later. Only `monitor_node` reads the leader's true odometry.

### D12. Real vision instead of a noise model
The leader carries a 0.5 m red sphere (same apparent size from any direction). `detector_node`
segments red in HSV and fits an enclosing circle -> z = [u, v, s]. Fault injection (misses, false
detections) is kept as a launch argument to exercise the gating.

### D13. Detector calibration (found with a ground-truth diagnostic)
First closed-loop run: 0.45 m constant estimation error even with the leader hovering. Comparing
detections with the projection of the true position showed s biased by +1.28 px (std 0.13 px),
i.e. 0.51 m of range bias. Fixes: half-pixel convention on u, v; no "+1" on the diameter; a
calibrated `size_bias_px = 0.3`. Range bias down to 0.10 m. Measurement noise retuned from the
measured statistics (sigma_size 1.0 -> 0.5 px). Lesson: a Kalman filter does not remove a bias.

### D14. Camera rotation uses the full body attitude
The gimbal motors' reaction torque makes the follower body roll slightly (tilted horizon in the
image). R_cw = R_body * Rz(gimbal yaw) * R(-y, gimbal pitch), unit tested against the core model.

### D15. Fuse each detection at its capture time
Bug seen with the Gazebo GUI + RViz open: track lost as soon as the leader moved, follower flying
away. Measured detection latency (capture -> tracker): 134 ms mean, 790 ms max, because the detector
fell behind (debug image published to 2 viewers) and frames queued up. The tracker fused old images
with the camera pose of "now": inconsistent measurements, chi2 rejections, track lost, COAST drift.
Fix: stamped camera-pose history + `TimedTracker` (EKF lives at the last measurement time,
output predicted to now) + detector queue of 1 image. Offline sim with 0.4 s latency: valid track
48 % -> 99.5 %, position error 15.8 m -> 0.37 m (naive vs fixed, both unit tested).

### D16. One stamped history per sensor stream; freshest state for control
Still with the GUI: TRACK -> SEARCH while the leader was in view, 30-40 m estimate jumps, jerky
follower. A ground-truth reprojection at the image stamp gave 0.2 px / 0 ms offset: Gazebo and the
detector were synchronised, the fusion was not. tracker_node paired each odometry sample with the
LATEST gimbal angles; under load the 500 Hz joint_states queue lagged the 100 Hz odometry by tens of
ms (about 14 px while the gimbal turns). Fix: `TimeSeries` per stream (odometry, gimbal), each
interpolated at the image stamp. Control nodes use queues of 1 (freshest state); vehicle_node
sub-steps when starved. GUI run: valid track 63 % -> 99.4 %, 2051/2052 detections used.

## Phase 3: real quadrotor, own flight controller, own navigation (2026-11-17)

### D17. Own flight controller rather than PX4 SITL
For a GNC role the attitude/rate control, mixer and navigation are what gets discussed; with PX4
they would be a black box. PX4 SITL stays a later comparison ("my GNC vs PX4 + EKF2").

### D18. Flight controller = Gazebo system plugin, companion computer = ROS nodes
ROS nodes were measured stalling ~100 ms under GUI load: fatal for a 250-500 Hz rate loop. The
plugin (follow_me_flight_controller) runs navigation + attitude/rate control + motor model in the
physics loop (every 2 ms) by calling follow_me_core. ROS sends an acceleration + yaw setpoint
(/follower/fc/accel_yaw_setpoint); the plugin holds position if setpoints stop for 0.5 s.
Gazebo integrates the 6-DOF rigid body (gravity on, 1.5 kg); the plugin applies the rotor wrench
computed by the same MotorModel/rotorWrench as the offline sim.

### D19. Cascaded controller + mixer with yaw-last desaturation
Thrust vector (tilt limit 35 deg) -> desired attitude -> quaternion P loop -> rate PI loop with
gyroscopic term -> allocation inverse. First mixer dropped yaw entirely when saturated: at full
tilt the vehicle stopped turning (unit test). Now it keeps the largest feasible fraction of the
yaw torque (bisection), then shifts the collective, then scales the differential part.

### D20. Navigation: 15-state error-state EKF
IMU-driven nominal state (p, v, q, b_a, b_g), error state with body-frame attitude error.
GNSS pos+vel 10 Hz, baro 25 Hz, magnetometer yaw 50 Hz, chi2-gated, Joseph form, error injection.
Sensors are simulated from the truth inside the plugin (deterministic, same models offline).
Test on 60 s of aggressive flight: 0.12 m / 0.37 deg tilt / 0.16 deg yaw, NEES 15.6 (ideal 15).
The whole GNC (attitude control, position control, tracker camera pose) runs on this estimate;
/follower/odom is now the navigation estimate, the truth is /follower/odom_truth (monitor only).

### D21. Navigation uncertainty propagated into the target tracker
The camera pose comes from the navigation. Ignoring its uncertainty made the target EKF
over-confident (NEES ~44). Fix: R_extra = H P_nav_pos H^T + (f sigma_att)^2 on u, v.
NEES back to 6-9 (still > 6: navigation errors are time-correlated, not white).

### D22. Stopping-distance speed limit + safety memory after a track loss
Added while chasing the 2.3-2.8 m minimum distance of the first Gazebo flights (see D23): (1)
closing speed <= sqrt(2 a_brake (d - 2 sigma - d_min)) at any distance (CBF-like invariant set),
(2) for 3 s after a loss the filter keeps protecting against the last known leader position
with a growing uncertainty. Sound, kept, but it was NOT the root cause.

### D23. Self-occlusion: camera lowered, truncation check, bearing-only fusion
Root cause of the 2.3-2.8 m approaches: during the start-up acceleration (25-35 deg tilt) the
gimbal looked "up" relative to the body and the follower's own airframe hid part of the beacon.
Smaller apparent size -> range overestimated by up to 6 m -> the follower kept charging; the
safety filter, working on that wrong and over-confident estimate, could not help. (A first
"render lag" hypothesis came from comparing frames of one run with distances of another run.)
Fixes: camera 25 cm below the body, gimbal up-tilt <= 40 deg; detector flags truncated beacons
(fill ratio < 0.75 or touching the image border); tracker then fuses (u, v) only (2-dof gate)
and never starts a track on it. Result: minimum distance 6.42-6.48 m over 3 runs (GUI included).

## Next steps (ideas)
- Noisy follower self-state (GNSS/INS), and camera mounted on a body with real attitude.
- Leader acceleration in the state (CA model) or IMM CV/coordinated-turn.
- Control barrier function for the separation instead of the heuristic filter.
- Smarter search, occlusion-aware guidance (keep the leader away from the sun / frame edges).
- Monte-Carlo runs over seeds with metric histograms.
