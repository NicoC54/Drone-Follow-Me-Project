"""Matplotlib figures: overview, estimation, camera & control. Plus a 2D animation."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .camera import MeasKind
from .estimation import TrackStatus

C_LEADER, C_FOLLOWER, C_EST, C_DES = "tab:red", "tab:blue", "tab:orange", "tab:green"


def _shade_status(ax, t, status):
    """Grey background where there is no track, light yellow where coasting."""
    for value, color in ((TrackStatus.SEARCH, "0.85"), (TrackStatus.COAST, "#fff3c4")):
        mask = status == value
        if mask.any():
            ax.fill_between(t, 0, 1, where=mask, color=color, transform=ax.get_xaxis_transform(),
                            step="post", lw=0)


def fig_overview(log, extra):
    cfg, t = log["cfg"], log["t"]
    pl, pf, xh = log["p_leader"], log["p_follower"], log["x_hat"]
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle("Overview")

    ax = fig.add_subplot(2, 2, 1, projection="3d")
    ax.plot(*pl.T, color=C_LEADER, label="leader (truth)")
    ax.plot(*pf.T, color=C_FOLLOWER, label="follower")
    ax.plot(*xh[:, :3].T, color=C_EST, lw=0.8, alpha=0.7, label="leader (estimate)")
    ax.set_xlabel("x East [m]"); ax.set_ylabel("y North [m]"); ax.set_zlabel("z Up [m]")
    ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(pl[:, 0], pl[:, 1], color=C_LEADER, label="leader")
    ax.plot(pf[:, 0], pf[:, 1], color=C_FOLLOWER, label="follower")
    for i in range(0, len(t), int(5 / cfg.sim.dt)):   # a "link" every 5 s
        ax.plot([pl[i, 0], pf[i, 0]], [pl[i, 1], pf[i, 1]], color="0.6", lw=0.8)
        ax.annotate(f"{t[i]:.0f}s", pl[i, :2], fontsize=7, color=C_LEADER)
    ax.set_aspect("equal"); ax.set_title("Top view (grey: link every 5 s)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 2, 3)
    _shade_status(ax, t, log["status"])
    ax.plot(t, extra["dist"], color="k", label="true distance")
    ax.axhline(extra["d_ref"], color=C_DES, ls="--", label="setpoint")
    ax.axhline(cfg.safety.d_min, color="tab:red", ls="--", label="d_min")
    ax.axhspan(cfg.safety.d_min, cfg.safety.d_min + cfg.safety.buffer, color="tab:red", alpha=0.08)
    ax.set_ylabel("follower-leader distance [m]"); ax.set_xlabel("t [s]")
    ax.set_title("Separation (grey: SEARCH, yellow: COAST)"); ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(t, np.linalg.norm(log["v_leader"][:, :2], axis=1), color=C_LEADER, label="leader speed")
    ax.plot(t, np.linalg.norm(log["v_follower"][:, :2], axis=1), color=C_FOLLOWER, label="follower speed")
    ax2 = ax.twinx()
    ax2.plot(t, pf[:, 2] - pl[:, 2], color="tab:purple", lw=0.8, label="height above leader")
    ax2.axhline(cfg.guidance.height_offset, color="tab:purple", ls=":")
    ax2.set_ylabel("dz [m]", color="tab:purple")
    ax.set_xlabel("t [s]"); ax.set_ylabel("horizontal speed [m/s]")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    return fig


def fig_estimation(log, extra):
    t, err = log["t"], log["x_hat"] - np.hstack([log["p_leader"], log["v_leader"]])
    sig = np.sqrt(np.diagonal(log["P"], axis1=1, axis2=2))
    fig, axes = plt.subplots(3, 3, figsize=(14, 8), sharex=True)
    fig.suptitle("Estimation: error (truth - estimate) vs +/- 3 sigma of the EKF")
    labels = ["x", "y", "z"]
    for j in range(3):
        for row, off, unit in ((0, 0, "m"), (1, 3, "m/s")):
            ax = axes[row, j]
            _shade_status(ax, t, log["status"])
            ax.plot(t, err[:, off + j], color=C_EST, lw=0.8)
            ax.plot(t, 3 * sig[:, off + j], "k--", lw=0.6)
            ax.plot(t, -3 * sig[:, off + j], "k--", lw=0.6)
            ax.set_ylabel(f"{'p' if off == 0 else 'v'}{labels[j]} err [{unit}]")
            lim = np.nanpercentile(np.abs(err[:, off + j]), 99) * 2 + 0.1
            ax.set_ylim(-lim, lim)
    ax = axes[2, 0]
    ax.semilogy(t, extra["nees"], color="0.3", lw=0.5)
    ax.axhline(6, color=C_DES, ls="--", label="expected (6)")
    ax.set_ylabel("NEES"); ax.legend(fontsize=8)
    ax = axes[2, 1]
    nis = log["nis"]
    ok = ~np.isnan(nis)
    acc = log["accepted"]
    ax.semilogy(t[ok & acc], nis[ok & acc], ".", ms=2, color=C_FOLLOWER, label="accepted")
    ax.semilogy(t[ok & ~acc], nis[ok & ~acc], "x", ms=4, color="tab:red", label="rejected")
    ax.axhline(log["cfg"].estimator.gate_chi2, color="k", ls="--", label="gate")
    ax.set_ylabel("NIS"); ax.legend(fontsize=8)
    ax = axes[2, 2]
    ax.plot(t, extra["e_pos"], color=C_EST, lw=0.8, label="|pos err| [m]")
    ax.plot(t, extra["e_vel"], color="tab:brown", lw=0.8, label="|vel err| [m/s]")
    ax.set_ylim(0, np.nanpercentile(extra["e_vel"], 99) * 1.5 + 0.1); ax.legend(fontsize=8)
    for ax in axes[2]:
        ax.set_xlabel("t [s]")
    fig.tight_layout()
    return fig


def fig_camera_control(log):
    cfg, t = log["cfg"], log["t"]
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle("Camera and control")

    ax = fig.add_subplot(2, 2, 1)
    zt, z, kind = log["z_true"], log["z"], log["meas_kind"]
    ax.plot(zt[:, 0], zt[:, 1], color="0.7", lw=0.6, label="true leader projection")
    det, outl = kind == MeasKind.DETECTION, kind == MeasKind.OUTLIER
    ax.plot(z[det, 0], z[det, 1], ".", ms=2, color=C_FOLLOWER, label="detections")
    ax.plot(z[outl, 0], z[outl, 1], "x", color="tab:red", label="outliers")
    ax.set_xlim(0, cfg.camera.width); ax.set_ylim(cfg.camera.height, 0)
    ax.set_aspect("equal"); ax.set_title("Image plane"); ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 2, 2)
    _shade_status(ax, t, log["status"])
    ax.plot(t, zt[:, 0] - cfg.camera.width / 2, label="u - cx")
    ax.plot(t, zt[:, 1] - cfg.camera.height / 2, label="v - cy")
    ax.axhline(cfg.camera.width / 2, color="k", ls=":"); ax.axhline(-cfg.camera.width / 2, color="k", ls=":")
    ax.set_ylim(-cfg.camera.width / 2 * 1.2, cfg.camera.width / 2 * 1.2)
    ax.set_ylabel("true pixel offset from centre [px]"); ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 2, 3)
    _shade_status(ax, t, log["status"])
    for j, n in enumerate("xyz"):
        ax.plot(t, log["a_cmd"][:, j], lw=0.8, label=f"a_cmd {n}")
    ax.plot(t, np.linalg.norm(log["a_cmd"], axis=1), "k", lw=0.8, label="|a_cmd|")
    ax.axhline(cfg.follower.a_max, color="k", ls=":")
    sa = log["safety_active"]
    if sa.any():
        ax.fill_between(t, 0, 1, where=sa, color="tab:red", alpha=0.2,
                        transform=ax.get_xaxis_transform(), lw=0, label="safety active")
    ax.set_xlabel("t [s]"); ax.set_ylabel("[m/s^2]"); ax.legend(fontsize=8, ncol=3)

    ax = fig.add_subplot(2, 2, 4)
    e_des = np.linalg.norm(log["p_des"] - log["p_follower"], axis=1)
    ax.plot(t, e_des, color=C_DES, label="|p_des - p| (tracking of the guidance)")
    ax.set_ylabel("[m]")
    ax2 = ax.twinx()
    ax2.plot(t, np.degrees(np.unwrap(log["gimbal"][:, 0])), color="0.5", lw=0.8)
    ax2.plot(t, np.degrees(log["gimbal"][:, 1]), color="0.5", lw=0.8, ls="--")
    ax2.set_ylabel("gimbal yaw (solid) / pitch (dashed) [deg]", color="0.4")
    ax.set_xlabel("t [s]"); ax.legend(fontsize=8)
    fig.tight_layout()
    return fig


def animate(log, speed=4.0, fps=25):
    """Top view + side view, played `speed` times faster than real time."""
    cfg, t = log["cfg"], log["t"]
    pl, pf, xh, pd = log["p_leader"], log["p_follower"], log["x_hat"], log["p_des"]
    step = max(1, int(round(speed / fps / cfg.sim.dt)))
    idx = np.arange(0, len(t), step)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [2, 1]})
    ax1.plot(pl[:, 0], pl[:, 1], color=C_LEADER, alpha=0.15)
    ax1.set_aspect("equal"); ax1.set_xlabel("x [m]"); ax1.set_ylabel("y [m]")
    trail_l, = ax1.plot([], [], color=C_LEADER)
    trail_f, = ax1.plot([], [], color=C_FOLLOWER)
    dot_l, = ax1.plot([], [], "o", color=C_LEADER, label="leader")
    dot_f, = ax1.plot([], [], "o", color=C_FOLLOWER, label="follower")
    dot_e, = ax1.plot([], [], "+", color=C_EST, ms=12, mew=2, label="estimate")
    dot_d, = ax1.plot([], [], "x", color=C_DES, label="follow point")
    fov_l, = ax1.plot([], [], color=C_FOLLOWER, lw=0.6)
    ring = plt.Circle((0, 0), cfg.safety.d_min, color="tab:red", fill=False, ls="--")
    ax1.add_patch(ring)
    ax1.legend(loc="upper left", fontsize=8)
    title = ax1.set_title("")

    dist = np.linalg.norm(pf - pl, axis=1)
    ax2.plot(t, dist, color="0.8")
    ax2.axhline(cfg.safety.d_min, color="tab:red", ls="--")
    ax2.axhline(np.hypot(cfg.guidance.follow_dist, cfg.guidance.height_offset), color=C_DES, ls="--")
    cur, = ax2.plot([], [], "k")
    ax2.set_xlabel("t [s]"); ax2.set_ylabel("distance [m]")
    half_fov = np.arctan(cfg.camera.width / 2 / cfg.camera.fx)
    names = {0: "SEARCH", 1: "TRACK", 2: "COAST"}

    def update(i):
        a = max(0, i - int(8 / cfg.sim.dt))
        trail_l.set_data(pl[a:i + 1, 0], pl[a:i + 1, 1])
        trail_f.set_data(pf[a:i + 1, 0], pf[a:i + 1, 1])
        dot_l.set_data([pl[i, 0]], [pl[i, 1]])
        dot_f.set_data([pf[i, 0]], [pf[i, 1]])
        dot_e.set_data([xh[i, 0]], [xh[i, 1]])
        dot_d.set_data([pd[i, 0]], [pd[i, 1]])
        ring.center = (pl[i, 0], pl[i, 1])
        yaw, L = log["gimbal"][i, 0], 15.0
        p = pf[i, :2]
        fov_l.set_data([p[0] + L * np.cos(yaw + half_fov), p[0], p[0] + L * np.cos(yaw - half_fov)],
                       [p[1] + L * np.sin(yaw + half_fov), p[1], p[1] + L * np.sin(yaw - half_fov)])
        c = (pl[i, :2] + pf[i, :2]) / 2
        ax1.set_xlim(c[0] - 25, c[0] + 25); ax1.set_ylim(c[1] - 25, c[1] + 25)
        cur.set_data(t[:i + 1], dist[:i + 1])
        title.set_text(f"t = {t[i]:5.1f} s   {names[int(log['status'][i])]}"
                       f"{'   SAFETY' if log['safety_active'][i] else ''}")
        return trail_l, trail_f, dot_l, dot_f, dot_e, dot_d, fov_l, ring, cur, title

    anim = FuncAnimation(fig, update, frames=idx, interval=1000 / fps, blit=False)
    return fig, anim
