"""Main loop: truth -> sensor -> estimation -> guidance -> control -> truth."""
import numpy as np

from .camera import Gimbal, MeasKind, PinholeCamera, look_at
from .config import Config
from .control import PositionController, SafetyFilter, saturate
from .estimation import TargetTracker, TrackStatus
from .guidance import FollowGuidance
from .vehicles import Follower, Leader


def run(cfg: Config = None) -> dict:
    cfg = cfg or Config()
    rng = np.random.default_rng(cfg.sim.seed)
    dt = cfg.sim.dt

    leader = Leader(cfg.leader)
    follower = Follower(cfg.follower, rng)
    cam = PinholeCamera(cfg.camera)
    gimbal = Gimbal(cfg.camera)      # starts looking East, at the horizon
    tracker = TargetTracker(cfg.estimator, cam)
    guidance = FollowGuidance(cfg.guidance)
    controller = PositionController(cfg.control, cfg.follower.a_max, cfg.follower.v_max)
    safety = SafetyFilter(cfg.safety, cfg.follower.a_max)

    n_steps = int(round(cfg.sim.duration / dt))
    cam_period = 1.0 / cfg.camera.rate_hz
    next_cam_t = 0.0
    hold_pos = None
    nan3 = np.full(3, np.nan)
    log = {k: [] for k in (
        "t", "p_leader", "v_leader", "p_follower", "v_follower", "x_hat", "P",
        "status", "p_des", "a_cmd", "safety_active", "gimbal",
        "z", "z_true", "meas_kind", "accepted", "in_frame", "nis")}

    for k in range(n_steps):
        t = k * dt

        # ---- 1. Estimation: predict, then update when a camera frame arrives ----
        tracker.predict(dt)
        z, kind, accepted, nis = nan3, -1, False, np.nan
        if t >= next_cam_t - 1e-9:
            next_cam_t += cam_period
            R_cw = gimbal.R_cw()
            z_meas, kind = cam.measure(leader.p, follower.p, R_cw, rng)
            if any(t0 <= t < t1 for t0, t1 in cfg.camera.occlusions):
                z_meas, kind = None, MeasKind.MISSED   # e.g. sun glare, cloud, occluding object
            if z_meas is not None:
                z = z_meas
                accepted = tracker.on_measurement(z_meas, t, follower.p, R_cw)
                nis = tracker.last_nis
        tracker.update_status(t)

        # ---- 2. Guidance + gimbal pointing ----
        safety_active = False
        if tracker.has_estimate:
            p_t, v_t = tracker.ekf.x[:3], tracker.ekf.x[3:]
            p_des, v_des = guidance.compute(follower.p, p_t, v_t, dt)
            hold_pos = None
            gimbal.point(*look_at(follower.p, p_t), dt)
        else:
            if hold_pos is None:           # just lost it: hover here and search
                hold_pos = follower.p.copy()
                guidance.reset()
            p_des, v_des = hold_pos, np.zeros(3)
            gimbal.sweep(dt)

        # ---- 3. Control + safety ----
        a_cmd = controller.compute(follower.p, follower.v, p_des, v_des, dt)
        if tracker.has_estimate:
            a_cmd, safety_active = safety.filter(a_cmd, follower.p, follower.v, p_t, v_t,
                                                 tracker.ekf.P[:3, :3])
        a_cmd = saturate(a_cmd, cfg.follower.a_max)

        # ---- logging (state at time t, before propagation) ----
        z_true, rc = cam.h(leader.p, follower.p, gimbal.R_cw())
        log["t"].append(t)
        log["p_leader"].append(leader.p.copy())
        log["v_leader"].append(leader.v.copy())
        log["p_follower"].append(follower.p.copy())
        log["v_follower"].append(follower.v.copy())
        log["x_hat"].append(tracker.ekf.x.copy() if tracker.has_estimate else np.full(6, np.nan))
        log["P"].append(tracker.ekf.P.copy() if tracker.has_estimate else np.full((6, 6), np.nan))
        log["status"].append(int(tracker.status))
        log["p_des"].append(np.asarray(p_des, dtype=float).copy())
        log["a_cmd"].append(a_cmd.copy())
        log["safety_active"].append(safety_active)
        log["gimbal"].append((gimbal.yaw, gimbal.pitch))
        log["z"].append(z)
        log["z_true"].append(z_true if rc[2] > 0 else nan3)
        log["meas_kind"].append(int(kind))
        log["accepted"].append(accepted)
        log["in_frame"].append(cam.visible(z_true, rc))
        log["nis"].append(nis)

        # ---- 4. Propagate the truth ----
        follower.step(a_cmd, dt)
        leader.step(t, dt)

    out = {k: np.array(v) for k, v in log.items()}
    out["cfg"] = cfg
    return out


__all__ = ["run", "TrackStatus", "MeasKind"]
