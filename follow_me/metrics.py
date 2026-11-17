"""Scalar performance indicators computed from a simulation log."""
import numpy as np

from .camera import MeasKind
from .estimation import TrackStatus


def compute_metrics(log, settle_time=5.0):
    cfg = log["cfg"]
    t = log["t"]
    has_est = log["status"] != TrackStatus.SEARCH
    steady = has_est & (t >= settle_time)

    err = log["x_hat"] - np.hstack([log["p_leader"], log["v_leader"]])
    e_pos = np.linalg.norm(err[:, :3], axis=1)
    e_vel = np.linalg.norm(err[:, 3:], axis=1)
    # NEES: e^T P^-1 e, should average ~6 (state dimension) for a consistent filter
    nees = np.full(len(t), np.nan)
    for i in np.flatnonzero(has_est):
        nees[i] = err[i] @ np.linalg.solve(log["P"][i], err[i])

    dist = np.linalg.norm(log["p_follower"] - log["p_leader"], axis=1)
    d_ref = np.hypot(cfg.guidance.follow_dist, cfg.guidance.height_offset)
    kinds = log["meas_kind"]
    outliers = kinds == MeasKind.OUTLIER

    return {
        "estimation position RMSE [m]": np.sqrt(np.mean(e_pos[steady] ** 2)),
        "estimation velocity RMSE [m/s]": np.sqrt(np.mean(e_vel[steady] ** 2)),
        "mean NEES (ideal 6)": np.nanmean(nees[steady]),
        "distance error RMS [m]": np.sqrt(np.mean((dist[steady] - d_ref) ** 2)),
        "min distance [m]": dist.min(),
        "time below d_min [s]": np.sum(dist < cfg.safety.d_min) * cfg.sim.dt,
        "time leader in frame [%]": 100 * log["in_frame"].mean(),
        "time with valid track [%]": 100 * has_est.mean(),
        "safety filter active [%]": 100 * log["safety_active"].mean(),
        "outliers rejected": f"{np.sum(outliers & ~log['accepted'])}/{np.sum(outliers)}",
    }, {"e_pos": e_pos, "e_vel": e_vel, "nees": nees, "dist": dist, "d_ref": d_ref}


def print_metrics(m):
    w = max(len(k) for k in m)
    for k, v in m.items():
        print(f"  {k:<{w}}  {v:8.2f}" if isinstance(v, float) else f"  {k:<{w}}  {v:>8}")
