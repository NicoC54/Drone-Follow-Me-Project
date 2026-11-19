#!/usr/bin/env python3
"""Phase-4 diagnosis: project the TRUE target into the image at each frame time and compare with
the detections; split the estimate error along / across the line of sight.

    tools/diag_analyse.py out/diag.npz [fx] [cam_offset_z]
"""
import sys

import numpy as np

FX, CX, CY = 400.0, 320.0, 180.0


def interp(tab, t, cols):
    return np.stack([np.interp(t, tab[:, 0], tab[:, c]) for c in cols], axis=-1)


def quat_to_R(q):  # q = [w, x, y, z], array (N, 4)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def camera(fol, gim, t, off_z):
    """Camera centre and world->camera rotation (same convention as ros_utils::cameraRotation)."""
    p = interp(fol, t, [1, 2, 3])
    q = interp(fol, t, [4, 5, 6, 7])
    yaw, pitch = interp(gim, t, [1]).ravel(), interp(gim, t, [2]).ravel()
    Rb = quat_to_R(q)
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    Rz = np.zeros((len(t), 3, 3)); Rz[:, 0, 0] = cy; Rz[:, 0, 1] = -sy; Rz[:, 1, 0] = sy; Rz[:, 1, 1] = cy; Rz[:, 2, 2] = 1
    # rotation of angle `pitch` about -y
    Ry = np.zeros((len(t), 3, 3)); Ry[:, 0, 0] = cp; Ry[:, 0, 2] = -sp; Ry[:, 1, 1] = 1; Ry[:, 2, 0] = sp; Ry[:, 2, 2] = cp
    Rwl = Rb @ Rz @ Ry
    Rcw = np.stack([-Rwl[:, :, 1], -Rwl[:, :, 2], Rwl[:, :, 0]], 1)  # rows: right, down, forward
    pos = p + np.einsum("nij,j->ni", Rb, np.array([0, 0, off_z]))
    return pos, Rcw


def project(target, pos, Rcw):
    rc = np.einsum("nij,nj->ni", Rcw, target - pos)
    return FX * rc[:, 0] / rc[:, 2] + CX, FX * rc[:, 1] / rc[:, 2] + CY, rc[:, 2]


def stats(name, x):
    print(f"  {name:28s} mean {np.mean(x):+7.2f}   std {np.std(x):6.2f}   median {np.median(x):+7.2f}")


def main():
    d = np.load(sys.argv[1])
    off_z = float(sys.argv[3]) if len(sys.argv) > 3 else -0.25
    det, lead, fol, gim, est, nav = d["det"], d["lead"], d["fol"], d["gim"], d["est"], d["nav"]
    t0 = max(lead[0, 0], fol[0, 0], gim[0, 0])
    det = det[(det[:, 1] > 0) & (det[:, 0] > t0) & (det[:, 0] < min(lead[-1, 0], fol[-1, 0], gim[-1, 0]))]
    t = det[:, 0]
    pos, Rcw = camera(fol, gim, t, off_z)
    L = interp(lead, t, [1, 2, 3])
    u, v, z = project(L, pos, Rcw)
    du, dv = det[:, 2] - u, det[:, 3] - v
    W = det[:, 4] * z / FX
    near = (z < 30) & (det[:, 5] > 0)
    print(f"detections: {len(det)}   (< 30 m and size valid: {near.sum()})")
    print("pixel residual detection - projected truth (< 30 m):")
    stats("du [px]", du[near]); stats("dv [px]", dv[near])
    stats("du * z/f [m] (lateral)", (du * z / FX)[near]); stats("dv * z/f [m] (vertical)", (dv * z / FX)[near])
    print("apparent size -> W = s z / f (< 30 m):")
    stats("W [m]", W[near])
    # Time alignment: residual vs a time shift of the truth
    print("time shift of the truth that minimises the residual (< 30 m):")
    best = None
    for dt in np.arange(-0.15, 0.151, 0.01):
        pos2, R2 = camera(fol, gim, t + dt, off_z)
        u2, v2, _ = project(interp(lead, t + dt, [1, 2, 3]), pos2, R2)
        r = np.sqrt(np.mean((det[near, 2] - u2[near]) ** 2 + (det[near, 3] - v2[near]) ** 2))
        if best is None or r < best[1]:
            best = (dt, r)
        if abs(dt) < 1e-9:
            print(f"  dt = 0       rms {r:.2f} px")
    print(f"  best dt = {best[0]:+.2f} s  rms {best[1]:.2f} px")

    # Estimate error in FOLLOW, along / across the line of sight
    ph = d["phase"]
    t_follow = ph[ph[:, 1] == 3, 0].min() + 5.0 if (ph[:, 1] == 3).any() else 1e18
    lock = (est[:, 8] > 0) & (est[:, 0] > t_follow)
    e_t = est[lock, 0]
    e_t = e_t[(e_t > t0 + 1) & (e_t < lead[-1, 0])]
    sel = np.isin(est[:, 0], e_t)
    E = est[sel]
    Lt = interp(lead, E[:, 0], [1, 2, 3])
    # Gazebo's OdometryPublisher gives the twist in the BODY frame: rotate it to the world frame
    Vt = np.einsum("nij,nj->ni", quat_to_R(interp(lead, E[:, 0], [4, 5, 6, 7])), interp(lead, E[:, 0], [8, 9, 10]))
    Ft = interp(fol, E[:, 0], [1, 2, 3])
    ep = E[:, 2:5] - Lt
    ev = E[:, 5:8] - Vt
    los = Lt - Ft
    los /= np.linalg.norm(los, axis=1, keepdims=True)
    along = np.einsum("ni,ni->n", ep, los)
    lat = np.linalg.norm(ep - along[:, None] * los, axis=1)
    P = E[:, 11:47].reshape(-1, 6, 6)
    s_along = np.sqrt(np.einsum("ni,nij,nj->n", los, P[:, :3, :3], los))
    print(f"estimate in FOLLOW (+5 s), camera locked ({len(E)} samples):")
    stats("error along LOS [m]", along); stats("sigma along LOS [m]", s_along)
    stats("lateral error [m]", lat)
    stats("velocity error |ev| [m/s]", np.linalg.norm(ev, axis=1))
    nees_p = np.einsum("ni,ni->n", ep, np.linalg.solve(P[:, :3, :3], ep[..., None])[..., 0])
    nees_v = np.einsum("ni,ni->n", ev, np.linalg.solve(P[:, 3:, 3:], ev[..., None])[..., 0])
    e6 = np.hstack([ep, ev])
    nees = np.einsum("ni,ni->n", e6, np.linalg.solve(P, e6[..., None])[..., 0])
    print(f"  NEES position (ideal 3) {np.mean(nees_p):6.1f}   velocity (ideal 3) {np.mean(nees_v):6.1f}   "
          f"full (ideal 6) {np.mean(nees):6.1f}")
    print(f"  learnt size {E[-1, 9]:.3f} +/- {E[-1, 10]:.3f} m")
    rng = np.linalg.norm(Lt - Ft, axis=1)
    print("  along-LOS error by range:")
    for a, b in ((0, 7), (7, 9), (9, 11), (11, 15), (15, 40)):
        m = (rng >= a) & (rng < b)
        if m.sum() > 20:
            print(f"    {a:2d}-{b:2d} m  n={m.sum():5d}  mean {along[m].mean():+6.2f}  std {along[m].std():5.2f}  sigma {s_along[m].mean():4.2f}")
    print("  implied W = s z / f by range (detections):")
    for a, b in ((0, 7), (7, 9), (9, 11), (11, 15), (15, 30), (30, 80)):
        m = (z >= a) & (z < b) & (det[:, 5] > 0)
        if m.sum() > 20:
            print(f"    {a:2d}-{b:2d} m  n={m.sum():5d}  W {W[m].mean():5.3f} +/- {W[m].std():5.3f}   dv {dv[m].mean():+5.2f} px")
    # navigation error (follower), for completeness
    N = interp(nav, E[:, 0], [1, 2, 3])
    stats("follower nav error [m]", np.linalg.norm(N - Ft, axis=1))


if __name__ == "__main__":
    main()
