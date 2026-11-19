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

## Phase 4: unknown drone, ground radar, PIP and camera hand-over (2026-11-17, core done)

### D24. Architecture change: radar-cued interception, then vision follow
The red beacon of known size (H2) is dropped: the target is an arbitrary drone. A ground
surveillance radar detects it at ~500 m; the follower flies to the predicted intercept point
(PIP) of the target's standoff point, then hands over to its camera and follows below the target
(silhouette against the sky, no colour cue).

### D25. Raw radar plots, own-ship echo, track start
The radar sends plots (range, az, el, range rate; 2 Hz; 0.3 s latency; Pd 0.9; Poisson clutter),
including the follower's own echo. Association: nearest neighbour between two known objects (own
position from the navigation, target from the EKF), chi2 gates. Track start: two compatible plots
on consecutive scans, initial velocity from their difference.

### D26. Exact out-of-sequence fusion by replay
3 s of measurements + a filter snapshot after each; a late measurement restores the snapshot
before its stamp, then re-fuses everything after it. Unit test: arrival order == capture order to
1e-9. Replaces "drop older than the filter time" (TimedTracker) for the two-sensor case.

### D27. Unknown size learnt online
Camera = bearing only until W is known: W = s * z / f with z the radar plot DEPTH in the camera
frame (first version used the radar range: W = 13 m instead of 0.6 m, caught by a test), radar
uncertainty projected on the camera axis, scalar Kalman filter with a small random walk. Then the
camera is a full [u, v, s] measurement again (range to the decimetre at short distance).

### D28. Mission state machine and PIP guidance
WAIT -> MIDCOURSE (PIP at 12 m/s) -> HANDOVER (< 80 m, slow, gimbal cued by the radar, never
closer than 25 m without camera lock) -> FOLLOW (camera locked: 5 consecutive updates); back to
HANDOVER if the camera is lost for 3 s, LOST without track. PIP: smallest positive root of
(|v|^2 - V^2) tau^2 + 2 (r.v) tau + |r|^2 = 0, pure pursuit if none. Offline results (8 seeds):
track at 1.3 s, FOLLOW at ~23 s from ~480 m, min distance 6.1-6.5 m, size within 5 %.
Gazebo/ROS integration: see D29-D31.

### D29. ROS/Gazebo integration of phase 4
intercept.launch.py + intercept.sdf (grey quadrotor leader, no beacon, ~480 m away; radar mast)
+ intercept.yaml on top of follow_me.yaml. radar_node simulates the ground radar from the ground
truth (Gazebo has no radar) and delays scans by the datalink latency; fusion_node runs the
FusionTracker and publishes a region of interest; detector_node "silhouette" mode finds a dark
blob against the sky inside it; intercept_node runs the mission. Phase 3 stays untouched.

### D30. Tentative tracks + resolution-aware apparent size
Gazebo showed (1) a wrong initial track from two coincident plots -> tentative track confirmed by
3 radar hits; (2) the apparent size of an unknown drone depends on the range (thin arms below one
pixel, body and rotors split into blobs, the detector kept the central one): the size learnt at
80 m was half the close-range one, the camera then pulled the estimate, the radar got gated out.
Fix: merge neighbouring blobs; learn W and use the camera range only for targets >= 10 px.

### D31. Hand-over criterion and aspect noise
Hand-over complete = camera locked AND size known to 8 % (locking at 80 m with radar-only range
gave a 4.38 m minimum distance -> 6.38 m). 10 % relative noise on the silhouette size (rotating X
shape). Open issue: target NEES ~27 and 1.3-1.9 m follow error in Gazebo vs 0.2-0.6 m offline.

### D32. Visualisation: everything generated from the code, nothing drawn by hand
- The offline intercept sim logs 10 Hz state + every radar plot (`--out DIR`);
  `tools/make_figures.py` turns them into the README figures, the GIF and `docs/replay/replay.json`.
  Regenerating the media is one command, so they cannot drift from the code.
- `docs/replay/index.html`: three.js replay of the same log (GitHub Pages from `/docs`), no build step.
- RViz markers show the algorithms, not only the vehicles: predicted intercept point + approach
  line + constant-velocity target prediction (intercept_node), camera field of view (fusion_node),
  3-sigma ellipsoid of every radar plot (radar_node).
- `hud_node` draws the HUD on the detector image. Separate node, pure observer: nothing
  subscribes to its output in the loop, so the demo layer can never change the closed loop.
- Foxglove: `foxglove:=true` launch argument + layout + `tools/record_demo.sh` (MCAP, zstd).

### D33. Monte-Carlo campaign (offline)
`--intercept --montecarlo N`: wind 0-6 m/s random heading, radar latency 0.1-0.6 s, clutter 0-4
plots/scan, target size 0.4-1.2 m, new noise seeds. 300 runs: 300/300 reach FOLLOW, 0 below the
4 m minimum (worst 5.28 m), FOLLOW after 39 +/- 13 s, median follow distance error 0.78 m RMS.

### D34. Camera gates against scenery
Adding trees to the phase-4 world broke the interception in Gazebo: 2 px trees on the horizon at
480 m (the drone is 0.4 px there) were fused as bearings and pulled the radar track away, then the
target plots were classified as clutter. Two gates in FusionTracker::processCamera:
(1) detectability: camera ignored while f (W + 2 sigma_W) / z < 2 px; (2) plausibility: a blob
implying an object > 3 m at the track depth (taken 3 sigma closer) is rejected. Offline results
unchanged (0 rejection); unit test with synthetic clutter.

### D35. Gazebo visuals: what is in each world, and why
- Spinning propellers: 1 g links on revolute joints driven by JointController (4 g and 1e-7 kg m2:
  no effect on the flight). The follower's own blades are hidden to its onboard camera by a
  visibility flag/mask (they crossed the image as dark blobs), still visible in the GUI.
- follow_me.sdf (red-beacon colour detector): town + groves, generated by
  `tools/gen_world_extras.py` (fixed seed; low-saturation roofs so nothing looks red). Hills
  removed: with a 300 m far clip they overlapped the flight area (the leader flew into one).
- intercept.sdf keeps its plain ground. Even with D34, trees on the horizon were learnt as the
  target size during the hand-over (FOLLOW at 80 m with a wrong size, 2.9 m minimum distance).
  A thresholding silhouette detector needs a clean background: a cluttered horizon calls for a
  learnt detector. Kept as a documented limitation rather than a hidden tuning.

### D36. Phase-4 Gazebo estimation gap: diagnosed and closed
Method: `tools/diag_record.py` (detections, truth, gimbal, estimate, sim time) +
`tools/diag_analyse.py` (truth reprojected at each frame). Detections were unbiased (lateral 0,
vertical 3 cm, no time offset); the estimate followed its range measurement f W / s; the error
was in the APPARENT SIZE, three causes:
1. Aspect: the bounding-box width of an X quadrotor grows with its relative yaw (+17 % at 32 deg).
   A slow error the filter averaged as white noise (NEES 37). -> detector `size_mode: moment`:
   4 x horizontal standard deviation of the blob pixels; the sum of squared offsets of N rotors
   on a circle is yaw-invariant. Spread 7 % -> 3.5 %. (Inflating the noise for correlation,
   `size_aspect_tau`, made the filter consistent but 0.9 m inaccurate with 3.4 m minimum
   distances offline: kept as an option, off.)
2. Resolution: thresholding erodes small blobs (W = s z / f: 0.47 m at 50 m, 0.98 m at 10 m);
   W learnt at hand-over range started 18 % short. -> `size_res_px` (4 px absolute uncertainty in
   the size learning) + hand-over hold range driven by resolution (approach until ~20 px, never
   closer than 12 m) so that small targets still converge (0.45 m targets waited forever at 25 m).
3. Bearing-only updates ranged the target by parallax from a moving camera with a small attitude
   bias (classic Cartesian bearings-only EKF over-confidence). -> bearing updates keep the range:
   gain projected across the line of sight, Joseph form (`updateBearingKeepRange`).
Result (Gazebo, grey quad): estimate RMSE 1.36 -> 0.26 m, NEES 37 -> 7.5, distance error 0.87 m.
Also: the leader stopped instantly at the end of its plan (infinite deceleration); it now brakes.

### D37. Random manoeuvres and replayable Monte-Carlo
`--maneuvers 1|2`: random segments (speed 2-10 m/s, turn rate up to 30 / 70 deg/s, climb, hover,
leader acceleration 2.5 / 5 m/s^2, altitude kept in 25-60 m). Each run has its own RNG stream:
`--mc-run I` replays run I alone with logs. New metrics: NEES during the hand-over, time and phase
of the minimum distance.

### D38. IMM (calm + manoeuvre constant-velocity models)
Two CV models (q = 3 and 40) with Markov switching (sojourn 10 s / 4 s), model-matched updates for
the radar, the camera and the bearing-only camera, likelihood-weighted probabilities, inside the
OOSM replay. Measured honestly: median NEES unchanged; at the aggressive level the 90th percentile
of the hand-over NEES drops 21.6 -> 15.7; the violations do NOT change -> they were not caused by
the target model. Kept for consistency in manoeuvres; `imm: false` gives the single CV model.

### D39. Safety against an agile target
Violations classified run by run (estimate error / sigma at the critical moment, closing speed):
1. stopping-distance constraint assumed a constant target velocity; a target turning away less
   quickly ate the margin -> guaranteed relative deceleration a_brake - a_target (4.5 - 2.5);
2. hand-over hold range ignored the range uncertainty (radar-only sigma 3-4 m) -> hold at
   d_min + 3 m + 3 sigma_range; margins at 3 sigma (k_sigma 2 -> 3).
Runs below 4 m (300 each): scripted 0, moderate 26 -> 5 (1.7 %), aggressive 63 -> 80/300 still
(27 %): a target braking at 5 m/s^2 and turning at 70 deg/s 8 m away is as agile as the follower;
no 4 m guarantee is possible at that standoff. Next: standoff scaled with the target's observed
agility, or evasive guidance.

### D40. Unknown targets in Gazebo: random shape and size
The phase-4 leader is no longer in intercept.sdf: intercept.launch.py spawns it
(launch/target_models.py): quad_x, hexa or octo, scale 0.6-1.8, random or chosen
(`target`, `target_scale`, `target_seed`). Checked: hexa x1.5 (2.2 m span) learnt 1.58 m in
"moment" units (ratio 0.67 like the quad), FOLLOW at 35 s, 7.8 m minimum; octo x0.7 learnt
0.89 m, FOLLOW at 44 s, estimate RMSE 0.43 m, NEES 9, 6.7 m minimum.

### D41. Mid-course command low-pass
The PIP is p_t + v_t tau, tau ~ 30 s: radar velocity noise times the time-to-go made the aim
point jump by tens of metres at each scan; the follower hit its 35 deg tilt limit in cruise
(Gazebo: p95 33 deg). First-order low-pass on the velocity command (1.5 s mid-course, 1 s
hand-over), started from the current velocity. Cruise tilt p95 33 -> 15 deg (offline and Gazebo),
interception time unchanged. Hand-over tilt (~17 deg mean) is physical: matching a target that
turns at 15-20 deg/s at 9 m/s; a closing-speed ramp was tried and removed (no gain, later FOLLOW).

### D42. The target is a real quadrotor
Offline: TargetDrone = FollowerSim (6-DOF quadrotor + flight controller, truth navigation) flying
the scenario as a reference: a = a_ref + 2.5 (v_ref - v) + 0.6 (p_ref - p), yaw = reference
heading, 6 m/s^2 max; same wind as the follower. Gazebo: the spawned target carries the
follower's FlightController plugin; leader_node is its pilot (same law, ground truth).
`target_physics:=false` / `--kinematic-target` keeps the old kinematic target.
Effects: the target tilts (Gazebo mean 9 deg, max 27 deg); the moment size is tilt-invariant too
(W 1.012 / 1.014 / 1.003 / 0.994 m for 0-5 / 5-10 / 10-15 / 15-30 deg). Gazebo quad x1: estimate
RMSE 0.65 m, NEES 14, 5.1 m minimum (kinematic: 0.26 m, 7.5): the remaining along-LOS bias
(-1 m at 9-11 m) appears when the target accelerates away -> constant-velocity lag; next: a
constant-acceleration model in the IMM. Offline 300 runs: 0 below 4 m (worst 4.87 m); random
manoeuvres moderate 4/300, aggressive 69/300 (80 kinematic: a real target is thrust-limited too).

### D43. Size noise recalibrated; showcase (CI, video, short README)
- The second-moment size spreads 3.6 % frame to frame + 1.7 % slowly (Gazebo, physical target);
  size_rel_noise 10 % (set for the bounding box) -> 5 %. Gazebo physical quad: estimate RMSE
  0.65 -> 0.49 m, NEES 14 -> 12.6, along-LOS bias -0.50 -> -0.26 m. Offline 300 runs: worst
  5.04 m, distance error 0.85 m; aggressive manoeuvres 69 -> 49/300 below 4 m. The rest is the
  constant-velocity lag when the target accelerates (no residual pure delay: best shift 0.05 s).
- CI (GitHub Actions): Python tests; ROS 2 Jazzy container, rosdep, build, 48 tests, and a
  50-run Monte-Carlo that fails on any run below 4 m (regression guard of the whole loop).
- Demo video generated from the simulation: chase camera added to a copy of the world
  (chase_cam:=true), composed with the onboard HUD by tools/record_video.py (H.264, no ffmpeg).
- README cut to a showcase page (pitch, results, mermaid architecture, quick start);
  details in docs/RESULTS.md and docs/USAGE.md.

### D44. "Jerks" when the target stops: an investigation, and a measured non-fix
Measured first (Gazebo + offline): while the target hovers the follower is calm (2-7 deg tilt);
the jolts (20-30 deg) come when it STOPS or RESTARTS (distance 8.6 -> 6.8 m, then 10.9 m).
Hypotheses, each measured:
1. safety buffer chattering (k_sigma 3 put the standoff at the buffer edge): rejected, k_sigma
   2/3 and buffer 1.5-3 m change nothing offline;
2. estimate noise: rejected, follower acceleration uncorrelated with the estimate's velocity
   jumps, 90 % of its power below 1 Hz (a slow swing, not a jitter);
3. estimate LAG of constant-velocity models: 9-state filter (TargetEkfCA: CV with a = 0, Singer
   acceleration model) inside the IMM, acceleration feed-forward to the guidance. Result:
   follows tighter (distance error 0.83 -> 0.66 m, stop excursion 6.8 -> 7.7 m) but the follower
   works HARDER (tilt p95 26 -> 28-30 deg) and NEES 8 -> 11. A Singer model sees braking (-1.9 of
   -2.5 m/s^2) but also +-1.5 m/s^2 of noise in steady flight; the IMM mixture is quiet but sees
   only -0.7 (unit test). 4. smooth command (cap 3-4 m/s^2 + jerk limit): tilt p95 26 -> 18 deg
   but the follower can no longer keep its place behind a turning 10 m/s target (2-11 m error).
Conclusion: those accelerations are NEEDED to hold 8 m behind a target that stops and turns.
Defaults unchanged (model imm_cv, no feed-forward, no cap); every variant stays available
(`--set fusion.model=imm_singer intercept.a_ff_gain=1 intercept.follow_a_max=3`).

### D45. Several ground radars (trilateration)
Each scan carries the site of its radar; the tracker converts and associates per scan (nothing
else changes: association, OOSM replay and track start are in world coordinates). Simulation:
N radars, interleaved scans, datalink delivering in ARRIVAL order (different latencies).
Sites: (-60,-40), (450,-100) (lines of sight crossing at ~90 deg), (230,420). 200 runs/level:
horizontal radar-phase error 2.4-3.6 m -> 1.1-1.4 m (2 radars) -> 0.9-1.1 m (3); vertical
3.2-3.7 -> 2.7-2.9 -> 1.5-1.7 m: two ground radars barely help the altitude (low elevation angles,
VDOP), a third one CLOSE to the area does. FOLLOW 4-9 s earlier; aggressive manoeuvres 30 -> 24
-> 17 runs below 4 m. Gazebo `radars:=3`: mid-course error 4-6 m -> 1-3 m, FOLLOW at 37 s instead
of 43 s. First reading of the metric (3.84 -> 3.80 m "no gain") mixed horizontal, vertical and the
start transient: split metrics. GDOP maps: docs/media/radar_geometry.png (tools/make_figures.py).

### D46. RViz and HUD readability
HUD drawn at 2x in two lines of large type (readable in a 600 px panel); dock layout stored in
the RViz config (Qt state generated by tools/rviz_layout.py: RViz keeps panel sizes nowhere
else); radar-plot 3-sigma "pancakes" hidden by default; labelled drones with halos, true distance
on the link, estimate label (source + learnt size), phase-coloured mission label; the aim point
is labelled "PIP" in mid-course and "standoff point (8 m behind target)" in the hand-over, where
there is no rendezvous (approach 4 m/s < target 8 m/s) and the follower slides to its place.

### D47. Attitude reversals in FOLLOW: bumpless transfer at the follow entry
- Symptom (Gazebo): thrust axis swinging 60 deg in 0.4 s (up to 212 deg/s). The position loop
  saturates at 2.4 m/s of velocity error (kv 2.5, a_max 6): any setpoint step is bang-bang.
- Measured triggers: (1) HANDOVER -> FOLLOW: two laws aiming at different points, command jump of
  4.5-9.8 m/s^2 in one 10 ms step (offline: the 3 worst runs); (2) braking target seen edge-on:
  apparent size -10 % for 0.3-0.6 s, the estimate moves the wrong way; (3) safety filter release.
- Rejected: jerk limit after a phase change (Gazebo: target lost 2/2, the delay let the position
  error grow, then the follower charged in); closing-speed limit in FOLLOW (distance error
  1.10 -> 1.55 m); correlated size noise 0.3 s (estimate 0.40 -> 0.33 m, reversals unchanged);
  silhouette thickness as an edge-on flag (catches 45 % of size dips for 14 % rejected frames).
- Kept: bumpless transfer on entering FOLLOW only: the controller's velocity command starts from
  its previous value, time constant 0.8 s decaying to 0 in 2 s, closing speed <= 4 m/s meanwhile
  (at every phase change it also altered the hand-over approach: offline run at 3.96 m). Gazebo:
  one-step jump 4.5-9.8 -> 0.4-0.7 m/s^2; offline 300 runs: worst swing (median) 260 -> 214 deg/s
  (L0), 223 -> 172 (L1), same runs below 4 m, same distance error. Visible swings remain: the
  follower arrives in FRONT of the target (journal 27) and must turn around at full authority.
- Tools: /mission/debug, offline control.csv, follow_swing_p99/max, silhouette thickness.

### D48. Random target flights in Gazebo
`target_path:=random` (`target_maneuvers:=1|2`): the leader node flies `randomScenario` (the
offline Monte-Carlo generator) from a start heading drawn within +-100 deg of the bearing to the
follower (head-on to crossing; fully fleeing excluded: 2 m/s speed margin). One `target_seed`
draws shape, size, flight plan and heading. Default stays `scripted` (reference flight, documented
numbers). The offline Monte-Carlo does not draw the start heading yet.

## Next steps (ideas)
- Hand-over geometry that brings the follower in from behind (removes the turn-around at the
  follow entry, D47).
- Range-rate (Doppler) of several radars to estimate the target velocity directly (D45).
- Standoff adapted to the target's agility, or evasive guidance (D39, D44).
- Learnt detector (small CNN / YOLO on the cued ROI) so that phase 4 works over scenery (D35).
- Standoff adapted to the target's agility (D39), evasive guidance against a target that turns
  back on the follower.
- Leader acceleration in the state (CA model) or IMM CV/coordinated-turn.
- Control barrier function for the separation instead of the heuristic filter.
- Occlusion-aware guidance (keep the leader away from the sun / frame edges).
