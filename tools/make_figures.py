#!/usr/bin/env python3
"""Figures and GIF of the radar-cued interception (phase 4), from the offline simulator logs.

    # 1. produce the logs (C++ offline simulator, ~0.1 s per run)
    ros2_ws/build/follow_me_core/follow_me_offline_sim --intercept --seed 2 --out out/
    ros2_ws/build/follow_me_core/follow_me_offline_sim --intercept --montecarlo 300 --seed 7 --out out/
    # 2. figures + GIF (+ JSON for the web replay)
    python3 tools/make_figures.py out/ docs/media/

Outputs: intercept_overview.png, intercept_distance.png, intercept_estimation.png,
montecarlo.png + montecarlo_summary.json (if out/montecarlo.csv exists), radar_geometry.png,
radar_gain.png, intercept.gif, and
replay.json (written to ../replay/ next to the destination when it exists: docs/replay/).
"""
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Ellipse  # noqa: E402

# ---- palette (dataviz reference instance, light mode) ----
SURFACE = "#fcfcfb"
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1"
FOLLOWER, LEADER = "#2a78d6", "#e34948"          # categorical slots 1 and 8: entity identity
PHASE_NAMES = ["WAIT", "MIDCOURSE", "HANDOVER", "FOLLOW", "LOST"]
PHASE_COLORS = ["#a9a8a3", "#eb6834", "#eda100", "#1baf7a", "#a9a8a3"]  # fixed order
RADAR_TARGET, RADAR_OWN, RADAR_CLUTTER = "#e87ba4", "#52514e", "#c9c8c3"
D_MIN = 4.0


def style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
        "axes.spines.right": False, "font.size": 10, "axes.titlesize": 12, "axes.titleweight": "bold",
        "axes.titlecolor": INK, "legend.frameon": False, "legend.fontsize": 9, "lines.linewidth": 2.0,
    })


def load(out):
    s = np.genfromtxt(out / "intercept.csv", delimiter=",", names=True)
    r = np.genfromtxt(out / "radar.csv", delimiter=",", names=True)
    return s, r


def phase_segments(t, phase):
    """[(t0, t1, phase), ...] for background bands."""
    segs, start = [], 0
    for i in range(1, len(t) + 1):
        if i == len(t) or phase[i] != phase[start]:
            segs.append((t[start], t[min(i, len(t) - 1)], int(phase[start])))
            start = i
    return segs


# ---------------------------------------------------------------------------------------------
def fig_overview(s, r, dst):
    fig, ax = plt.subplots(figsize=(9, 7))
    for kind, color, marker, size, label in ((0, RADAR_CLUTTER, "x", 10, "radar plot: clutter"),
                                             (2, RADAR_OWN, ".", 14, "radar plot: own echo"),
                                             (1, RADAR_TARGET, ".", 14, "radar plot: target")):
        m = r["kind"] == kind
        ax.scatter(r["x"][m], r["y"][m], s=size, c=color, marker=marker, linewidths=1, label=label, zorder=2)
    ax.plot(s["lx"], s["ly"], color=LEADER, label="unknown drone (truth)", zorder=3)
    for p in range(5):  # follower path coloured by mission phase
        m = s["phase"] == p
        if m.any():
            seg = np.where(m, 1.0, np.nan)
            ax.plot(s["fx"] * seg, s["fy"] * seg, color=PHASE_COLORS[p], lw=2.5,
                    label=f"follower: {PHASE_NAMES[p]}", zorder=4)
    mid = (s["phase"] == 1) & (np.arange(len(s)) % 30 == 0)  # PIP aim points every 3 s
    ax.scatter(s["ax"][mid], s["ay"][mid], marker="D", s=30, facecolor=SURFACE,
               edgecolor=PHASE_COLORS[1], lw=1.5, label="predicted intercept point", zorder=5)
    ax.scatter([-60], [-40], marker="^", s=120, c=INK2, zorder=6)
    ax.annotate("ground radar", (-60, -40), xytext=(8, -14), textcoords="offset points", color=INK2)
    ax.scatter([s["lx"][0]], [s["ly"][0]], s=60, c=LEADER, zorder=6)
    ax.annotate(f"drone detected\n{s['dist'][0]:.0f} m away", (s["lx"][0], s["ly"][0]),
                xytext=(10, 4), textcoords="offset points", color=INK2)
    # Frame the engagement (clutter is spread over the whole radar coverage: crop to the action)
    xs = np.concatenate([s["lx"], s["fx"], [-60.0]])
    ys = np.concatenate([s["ly"], s["fy"], [-40.0]])
    cx, cy = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2
    half = 0.5 * max(xs.max() - xs.min(), ys.max() - ys.min()) + 60
    ax.set_xlim(cx - half * 1.25, cx + half * 1.25)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title("Radar-cued interception, then camera follow (top view)", loc="left")
    ax.legend(loc="upper left", ncol=1, fontsize=8)
    fig.tight_layout()
    fig.savefig(dst / "intercept_overview.png", dpi=130)
    plt.close(fig)


def shade_phases(ax, s):
    fig = ax.get_figure()
    width_in = ax.get_position().width * fig.get_figwidth()
    span = s["t"][-1] - s["t"][0]
    for t0, t1, p in phase_segments(s["t"], s["phase"]):
        ax.axvspan(t0, t1, color=PHASE_COLORS[p], alpha=0.12, lw=0)
        if (t1 - t0) / span * width_in > 0.065 * len(PHASE_NAMES[p]) + 0.1:  # label only if it fits
            ax.text((t0 + t1) / 2, 1.01, PHASE_NAMES[p], transform=ax.get_xaxis_transform(),
                    ha="center", va="bottom", color=INK2, fontsize=8)


def fig_distance(s, dst):
    fig, ax = plt.subplots(figsize=(9, 3.6))
    shade_phases(ax, s)
    ax.plot(s["t"], s["dist"], color=FOLLOWER)
    ax.axhline(np.hypot(8, 2), color=INK2, lw=1, ls="--")
    ax.axhline(D_MIN, color=INK, lw=1)
    ax.text(s["t"][-1], np.hypot(8, 2) * 1.12, "standoff 8.2 m", ha="right", va="bottom", color=INK2, fontsize=8)
    ax.text(s["t"][-1], D_MIN * 0.9, "minimum 4 m", ha="right", va="top", color=INK, fontsize=8)
    ax.set_yscale("log")
    ax.set_ylim(1.5, 800)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("follower-target distance [m]")
    ax.set_title(f"Distance to the target (min {s['dist'].min():.1f} m)", loc="left", pad=16)
    fig.tight_layout()
    fig.savefig(dst / "intercept_distance.png", dpi=130)
    plt.close(fig)


def fig_estimation(s, dst, true_size):
    track = s["track"] > 0
    e = np.sqrt((s["ex"] - s["lx"]) ** 2 + (s["ey"] - s["ly"]) ** 2 + (s["ez"] - s["lz"]) ** 2)
    sig = np.sqrt(s["pxx"] + s["pyy"] + s["pzz"])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 3.6))
    shade_phases(a1, s)
    a1.plot(s["t"][track], e[track], color=FOLLOWER, label="estimation error")
    a1.plot(s["t"][track], 3 * sig[track], color=INK2, lw=1, ls="--", label="filter 3σ (total)")
    a1.set_yscale("log")
    a1.set_xlabel("time [s]")
    a1.set_ylabel("target position error [m]")
    a1.set_title("Radar alone, then radar + camera", loc="left", pad=16)
    a1.legend(loc="upper right")
    shade_phases(a2, s)
    known = s["size_sigma"] < 0.25
    a2.fill_between(s["t"][known], (s["size"] - 2 * s["size_sigma"])[known],
                    (s["size"] + 2 * s["size_sigma"])[known], color=FOLLOWER, alpha=0.18, lw=0, label="±2σ")
    a2.plot(s["t"][known], s["size"][known], color=FOLLOWER, label="learnt size")
    a2.axhline(true_size, color=INK, lw=1, ls="--", label=f"true size {true_size:.1f} m")
    a2.set_xlabel("time [s]")
    a2.set_ylabel("target size [m]")
    a2.set_title("Unknown target size, learnt online", loc="left", pad=16)
    a2.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(dst / "intercept_estimation.png", dpi=130)
    plt.close(fig)


def fig_montecarlo(out, dst):
    path = out / "montecarlo.csv"
    if not path.exists():
        return None
    m = np.genfromtxt(path, delimiter=",", names=True)
    n = len(m)
    ok = m["t_follow"] > 0
    viol = int(np.sum(m["min_distance"] < D_MIN))
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.4))
    specs = [(m["min_distance"], "minimum distance [m]", f"Safety: {viol}/{n} runs below 4 m", D_MIN),
             (m["t_follow"][ok], "time to FOLLOW [s]", f"Interception: {ok.sum()}/{n} runs reach FOLLOW", None),
             (m["follow_dist_err_rms"][ok], "distance error in FOLLOW, RMS [m]", "Follow quality", None)]
    for ax, (x, label, title, ref) in zip(axes, specs):
        ax.hist(x, bins=25, color=FOLLOWER, edgecolor=SURFACE, linewidth=2)
        if ref is not None:
            ax.axvline(ref, color=INK, lw=1)
            ax.text(ref, ax.get_ylim()[1] * 0.95, " 4 m", color=INK, va="top", fontsize=8)
        ax.set_xlabel(label)
        ax.set_ylabel("runs")
        ax.set_title(title, loc="left", fontsize=11)
    fig.suptitle(f"Monte-Carlo: {n} randomised interceptions (wind 0-6 m/s, radar latency 0.1-0.6 s, "
                 f"clutter 0-4/scan, target size 0.4-1.2 m)", x=0.01, ha="left", fontsize=10, color=INK2)
    fig.tight_layout()
    fig.savefig(dst / "montecarlo.png", dpi=130)
    plt.close(fig)
    return {"runs": n, "follow_reached": int(ok.sum()), "violations": viol,
            "min_distance_min": float(m["min_distance"].min()),
            "t_follow_mean": float(np.mean(m["t_follow"][ok])), "t_follow_std": float(np.std(m["t_follow"][ok])),
            "dist_err_median": float(np.median(m["follow_dist_err_rms"][ok]))}


def fig_maneuvers(out, dst):
    """Robustness to the target's manoeuvres: out/maneuvers/L{0,1,2}/montecarlo.csv."""
    rows = []
    for lvl, name in ((0, "scripted"), (1, "random,\nmoderate"), (2, "random,\naggressive")):
        path = out / "maneuvers" / f"L{lvl}" / "montecarlo.csv"
        if not path.exists():
            return None
        m = np.genfromtxt(path, delimiter=",", names=True)
        ok = m["t_follow"] > 0
        rows.append(dict(level=lvl, name=name, n=len(m), follow=int(ok.sum()),
                         viol=int(np.sum(m["min_distance"] < D_MIN)),
                         nees=float(np.median(m["handover_nees"])), nees90=float(np.percentile(m["handover_nees"], 90)),
                         err=float(np.median(m["follow_est_err_rms"][ok]))))
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.4))
    x = np.arange(len(rows))
    v = [100.0 * r["viol"] / r["n"] for r in rows]
    axes[0].bar(x, v, color=FOLLOWER, width=0.55)
    for xi, vi, r in zip(x, v, rows):
        axes[0].text(xi, vi + 0.6, f"{r['viol']}/{r['n']}", ha="center", va="bottom", fontsize=9, color=INK)
    axes[0].set_xticks(x, [r["name"] for r in rows])
    axes[0].set_ylabel("runs below 4 m [%]")
    axes[0].set_title("Safety vs target agility", loc="left", fontsize=11)
    axes[1].bar(x - 0.15, [r["nees"] for r in rows], width=0.3, color=FOLLOWER, label="median")
    axes[1].bar(x + 0.15, [r["nees90"] for r in rows], width=0.3, color=PHASE_COLORS[1], label="90th percentile")
    axes[1].axhline(6.0, color=INK, lw=1, ls="--")
    axes[1].text(len(rows) - 0.55, 6.3, "ideal 6", fontsize=8, color=INK, ha="right")
    axes[1].set_xticks(x, [r["name"] for r in rows])
    axes[1].set_ylabel("target NEES in hand-over")
    axes[1].set_title("Filter consistency (IMM)", loc="left", fontsize=11)
    axes[1].legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(dst / "montecarlo_maneuvers.png", dpi=130)
    plt.close(fig)
    return rows


RADAR_SITES = [(-60.0, -40.0, 3.0), (450.0, -100.0, 3.0), (230.0, 420.0, 3.0)]  # = intercept.launch.py
SIGMA_R, SIGMA_AZ, SIGMA_EL = 2.0, np.radians(0.5), np.radians(0.8)               # = GroundRadarParams


def radar_plot_info(p, site):
    """Information matrix (inverse covariance) of one radar plot at position p (range + 2 angles)."""
    d = p - site
    r, rh = np.linalg.norm(d), np.linalg.norm(d[:2])
    u_r = d / r                                                   # along the line of sight
    if rh < 1e-6:  # right above the radar: azimuth undefined, any horizontal direction
        rh, u_az = 1e-3, np.array([1.0, 0.0, 0.0])
    else:
        u_az = np.array([-d[1], d[0], 0.0]) / rh                  # horizontal, across
    u_el = np.cross(u_r, u_az)                                    # vertical-ish, across
    return (np.outer(u_r, u_r) / SIGMA_R ** 2 + np.outer(u_az, u_az) / (rh * SIGMA_AZ) ** 2
            + np.outer(u_el, u_el) / (r * SIGMA_EL) ** 2)


def fig_radar_geometry(s, dst, altitude=40.0):
    """Precision of ONE scan of 1, 2, 3 ground radars over the engagement area (GDOP maps)."""
    xs, ys = np.linspace(-150, 550, 141), np.linspace(-200, 500, 141)
    fig, axes = plt.subplots(2, 3, figsize=(12, 7.6), sharex=True, sharey=True)
    levels = [0.5, 1, 2, 3, 5, 8, 12, 20]
    cmap = plt.get_cmap("Blues")  # one hue: light = precise, dark = large error
    for k in range(3):
        H, V = np.zeros((len(ys), len(xs))), np.zeros((len(ys), len(xs)))
        for i, y in enumerate(ys):
            for j, x in enumerate(xs):
                p = np.array([x, y, altitude])
                I = sum(radar_plot_info(p, np.array(site)) for site in RADAR_SITES[:k + 1])
                P = np.linalg.inv(I)
                H[i, j], V[i, j] = np.sqrt(P[0, 0] + P[1, 1]), np.sqrt(P[2, 2])
        for row, (field, name) in enumerate(((H, "horizontal"), (V, "vertical"))):
            ax = axes[row, k]
            cf = ax.contourf(xs, ys, np.clip(field, 0.3, 25), levels=levels, cmap=cmap, extend="both")
            cs = ax.contour(xs, ys, field, levels=[1, 2, 5, 10], colors=INK, linewidths=0.9)
            ax.clabel(cs, fmt="%g m", fontsize=8)
            ax.plot(s["lx"], s["ly"], color=LEADER, lw=2.0)
            for site in RADAR_SITES[:k + 1]:
                ax.scatter([site[0]], [site[1]], marker="^", s=80, c=PHASE_COLORS[1], edgecolor=SURFACE,
                           linewidths=1.5, zorder=5)
            ax.set_aspect("equal")
            ax.grid(False)
            if row == 0:
                ax.set_title(f"{k + 1} radar{'s' if k else ''}", loc="left")
            if k == 0:
                ax.set_ylabel(f"{name} error, 1σ\nNorth [m]")
            if row == 1:
                ax.set_xlabel("East [m]")
    fig.colorbar(cf, ax=axes, shrink=0.8, label="1σ position error of one scan [m]")
    fig.suptitle(f"Ground-radar geometry at {altitude:.0f} m altitude (red: target path). Ranges are precise "
                 "(2 m), angles coarse (0.5°/0.8°):\nseveral sites cross ranges against angles (trilateration); "
                 "a close site also fixes the altitude.", x=0.01, ha="left", fontsize=10, color=INK2)
    fig.savefig(dst / "radar_geometry.png", dpi=120, bbox_inches="tight")
    plt.close(fig)


def fig_radar_gain(s, dst):
    """The same numbers as the maps, read only where it matters: along the target's path, in time.

    1σ position error of ONE scan of 1, 2, 3 radars at the true target position (analytic, as the
    maps). Stops at FOLLOW: from then on the camera dominates and the radar count barely matters.
    """
    t_end = s["t"][np.argmax(s["phase"] == 3)] if np.any(s["phase"] == 3) else s["t"][-1]
    k = (s["t"] <= t_end + 5) & (s["phase"] >= 1)
    t, P = s["t"][k], np.c_[s["lx"][k], s["ly"][k], s["lz"][k]]
    blues = ["#9ec5f4", "#4f93e6", "#1c4f94"]  # one hue, light -> dark = more radars
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
    for k_r in range(3):
        H, V = [], []
        for p in P:
            C = np.linalg.inv(sum(radar_plot_info(p, np.array(site)) for site in RADAR_SITES[:k_r + 1]))
            H.append(np.sqrt(C[0, 0] + C[1, 1]))
            V.append(np.sqrt(C[2, 2]))
        for ax, field in zip(axes, (H, V)):
            ax.plot(t, field, color=blues[k_r], lw=2.0)
            ax.text(t[-1] + 0.8, field[-1], f"{k_r + 1} radar{'s' if k_r else ''}", color=INK2,
                    fontsize=9, va="center")
    for ax, name in zip(axes, ("Horizontal", "Vertical")):
        shade_phases(ax, {"t": t, "phase": s["phase"][k]})
        ax.set_title(f"{name} error, 1σ of one scan", loc="left", pad=16)
        ax.set_xlabel("time [s]")
        ax.set_xlim(t[0], t[-1] + 9)
        ax.set_ylim(0, None)
    axes[0].set_ylabel("error [m]")
    fig.suptitle("Along the target's path, before the camera takes over: every added radar cuts the error.",
                 x=0.01, ha="left", fontsize=10, color=INK2)
    fig.tight_layout()
    fig.savefig(dst / "radar_gain.png", dpi=120, bbox_inches="tight")
    plt.close(fig)


def cov_ellipse_xy(row):
    P = np.array([[row["pxx"], row["pxy"]], [row["pxy"], row["pyy"]]])
    w, v = np.linalg.eigh(P)
    w = np.maximum(w, 1e-6)
    ang = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
    return 2 * 3 * np.sqrt(w[1]), 2 * 3 * np.sqrt(w[0]), ang  # 3-sigma width, height, angle


def make_gif(s, r, dst, fps=10, step_s=1.0):
    idx = np.arange(0, len(s), int(round(step_s / (s["t"][1] - s["t"][0]))))
    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.9)

    def draw(i):
        ax.clear()
        ax.set_facecolor(SURFACE)
        row = s[i]
        t = row["t"]
        a = max(0, i - 300)
        ax.plot(s["lx"][a:i + 1], s["ly"][a:i + 1], color=LEADER, lw=1.5)
        ax.plot(s["fx"][a:i + 1], s["fy"][a:i + 1], color=FOLLOWER, lw=1.5)
        rr = r[(r["t"] > t - 3) & (r["t"] <= t)]
        for kind, color, marker in ((0, RADAR_CLUTTER, "x"), (2, RADAR_OWN, "."), (1, RADAR_TARGET, ".")):
            m = rr["kind"] == kind
            ax.scatter(rr["x"][m], rr["y"][m], s=30, c=color, marker=marker, zorder=2)
        if row["track"] > 0:
            w, h, ang = cov_ellipse_xy(row)
            ax.add_patch(Ellipse((row["ex"], row["ey"]), max(w, 0.3), max(h, 0.3), angle=ang,
                                 facecolor=PHASE_COLORS[3] if row["locked"] > 0 else PHASE_COLORS[1],
                                 alpha=0.25, edgecolor="none", zorder=3))
            if int(row["phase"]) in (1, 2):
                ax.plot([row["fx"], row["ax"]], [row["fy"], row["ay"]], color=PHASE_COLORS[1], lw=1, ls=":")
                ax.scatter([row["ax"]], [row["ay"]], marker="D", s=40, facecolor=SURFACE,
                           edgecolor=PHASE_COLORS[1], lw=1.5, zorder=4)
        ax.scatter([row["lx"]], [row["ly"]], s=70, c=LEADER, edgecolor=SURFACE, lw=2, zorder=5)
        ax.scatter([row["fx"]], [row["fy"]], s=70, c=FOLLOWER, edgecolor=SURFACE, lw=2, zorder=5)
        cx, cy = (row["lx"] + row["fx"]) / 2, (row["ly"] + row["fy"]) / 2
        half = max(25.0, 0.65 * row["dist"])
        ax.set_xlim(cx - half * 4 / 3, cx + half * 4 / 3)
        ax.set_ylim(cy - half, cy + half)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_visible(False)
        # scale bar (the zoom follows the engagement): 1-2-5 length about 1/5 of the view
        target = 0.4 * half
        bar = min((k * 10 ** e for e in range(0, 4) for k in (1, 2, 5)), key=lambda b: abs(b - target))
        x0, y0 = cx + half * 4 / 3 - bar - 0.08 * half, cy - half + 0.12 * half
        ax.plot([x0, x0 + bar], [y0, y0], color=INK, lw=2, solid_capstyle="butt")
        ax.text(x0 + bar / 2, y0 + 0.03 * half, f"{bar:g} m", ha="center", va="bottom", fontsize=8, color=INK)
        p = int(row["phase"])
        status = "camera locked" if row["locked"] > 0 else ("radar track" if row["track"] > 0 else "no track")
        ax.set_title(f"t = {t:5.1f} s    {PHASE_NAMES[p]}    {status}    distance {row['dist']:.0f} m",
                     loc="left", fontsize=10, color=INK)
        ax.text(0.01, 0.01, "blue: follower   red: unknown drone   pink: radar plots of the target   "
                "◇ predicted intercept point", transform=ax.transAxes, fontsize=7, color=INK2)
        return []

    anim = FuncAnimation(fig, draw, frames=idx, interval=1000 / fps)
    anim.save(dst / "intercept.gif", writer=PillowWriter(fps=fps), dpi=80)
    plt.close(fig)


def export_replay(s, r, dst, true_size):
    """Compact JSON for docs/replay (three.js web replay)."""
    def rnd(a, k=2):
        return [round(float(x), k) for x in a]
    data = {
        "dt": float(s["t"][1] - s["t"][0]), "radar_site": [-60, -40, 3], "true_size": true_size,
        "phase_names": PHASE_NAMES, "phase_colors": PHASE_COLORS,
        "t": rnd(s["t"], 1), "phase": [int(p) for p in s["phase"]],
        "leader": [rnd(s[k]) for k in ("lx", "ly", "lz")],
        "follower": [rnd(s[k]) for k in ("fx", "fy", "fz")],
        "follower_q": [rnd(s[k], 3) for k in ("qw", "qx", "qy", "qz")],
        "estimate": [rnd(s[k]) for k in ("ex", "ey", "ez")],
        "cov": [rnd(s[k], 3) for k in ("pxx", "pxy", "pxz", "pyy", "pyz", "pzz")],
        "track": [int(x) for x in s["track"]], "locked": [int(x) for x in s["locked"]],
        "size": rnd(s["size"], 3), "size_sigma": rnd(s["size_sigma"], 3), "dist": rnd(s["dist"], 1),
        "aim": [rnd(s[k]) for k in ("ax", "ay", "az")],
        "t_go": rnd(s["t_go"], 1),
        "radar": {"t": rnd(r["t"], 1), "x": rnd(r["x"], 1), "y": rnd(r["y"], 1), "z": rnd(r["z"], 1),
                  "kind": [int(k) for k in r["kind"]]},
    }
    replay_dir = dst.parent / "replay" if (dst.parent / "replay").is_dir() else dst  # docs/media -> docs/replay
    (replay_dir / "replay.json").write_text(json.dumps(data, separators=(",", ":")))


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
    dst = Path(sys.argv[2] if len(sys.argv) > 2 else "docs/media")
    true_size = float(sys.argv[3]) if len(sys.argv) > 3 else 0.6
    dst.mkdir(parents=True, exist_ok=True)
    style()
    s, r = load(out)
    fig_overview(s, r, dst)
    fig_distance(s, dst)
    fig_estimation(s, dst, true_size)
    mc = fig_montecarlo(out, dst)
    man = fig_maneuvers(out, dst)
    if man:
        (dst / "montecarlo_maneuvers.json").write_text(json.dumps(man, indent=2))
        print("Manoeuvres:", man)
    if mc:
        (dst / "montecarlo_summary.json").write_text(json.dumps(mc, indent=2))
        print("Monte-Carlo:", mc)
    fig_radar_geometry(s, dst)
    fig_radar_gain(s, dst)
    make_gif(s, r, dst)
    export_replay(s, r, dst, true_size)
    print("written to", dst)


if __name__ == "__main__":
    main()
