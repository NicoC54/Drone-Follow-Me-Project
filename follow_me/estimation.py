"""Estimation: EKF on the leader state + track management.

State x = [px, py, pz, vx, vy, vz] of the LEADER in world frame.
The follower's own position is assumed perfectly known (GNSS/INS) in this version.
"""
from enum import IntEnum

import numpy as np

from .camera import PinholeCamera
from .config import EstimatorConfig


class TrackStatus(IntEnum):
    SEARCH = 0  # no valid estimate: hover and sweep the gimbal
    TRACK = 1   # recent measurement accepted
    COAST = 2   # no recent measurement: follow the prediction


class TargetEKF:
    """Constant-velocity model driven by white acceleration noise of density q."""

    def __init__(self, q_accel):
        self.q = q_accel
        self.x = np.zeros(6)
        self.P = np.eye(6)

    def predict(self, dt):
        I = np.eye(3)
        F = np.eye(6)
        F[:3, 3:] = dt * I
        Q = self.q * np.block([[dt ** 3 / 3 * I, dt ** 2 / 2 * I],
                               [dt ** 2 / 2 * I, dt * I]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def innovation(self, z, cam: PinholeCamera, cam_pos, R_cw):
        z_hat, rc = cam.h(self.x[:3], cam_pos, R_cw)
        if rc[2] <= 0.1:          # predicted behind the camera: linearisation meaningless
            return None
        H = np.zeros((3, 6))
        H[:, :3] = cam.H_pos(rc, R_cw)   # pixels do not depend on velocity
        S = H @ self.P @ H.T + cam.R
        return z - z_hat, H, S

    def update(self, y, H, S, R):
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        IKH = np.eye(6) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T   # Joseph form: stays symmetric PSD


class TargetTracker:
    """Wraps the EKF with initialisation, outlier gating and track status."""

    def __init__(self, cfg: EstimatorConfig, cam: PinholeCamera):
        self.cfg, self.cam = cfg, cam
        self.ekf = TargetEKF(cfg.q_accel)
        self.status = TrackStatus.SEARCH
        self.t_last = -np.inf
        self.n_rejects = 0
        self.last_nis = np.nan
        self.candidate = None   # (t, p, P) of an unconfirmed detection while in SEARCH

    @property
    def has_estimate(self):
        return self.status != TrackStatus.SEARCH

    def predict(self, dt):
        if self.has_estimate:
            self.ekf.predict(dt)

    def _try_init(self, z, cam_pos, R_cw, t):
        """Two-step initialisation: a single detection may be a false alarm, so a track is
        only started when a second detection lands close to the first one (chi2 test
        with room for the leader's possible motion in between)."""
        p, P_pos = self.cam.backproject(z, cam_pos, R_cw)
        c, cand = self.cfg, self.candidate
        self.candidate = (t, p, P_pos)
        if cand is None or t - cand[0] > c.confirm_window:
            return False
        dt = t - cand[0]
        S = P_pos + cand[2] + (c.init_vel_sigma * dt) ** 2 * np.eye(3)
        d = p - cand[1]
        if d @ np.linalg.solve(S, d) > c.gate_chi2:
            return False
        self.ekf.x = np.concatenate([p, np.zeros(3)])
        self.ekf.P = np.zeros((6, 6))
        self.ekf.P[:3, :3] = P_pos
        self.ekf.P[3:, 3:] = c.init_vel_sigma ** 2 * np.eye(3)
        self.status, self.t_last, self.n_rejects, self.candidate = TrackStatus.TRACK, t, 0, None
        return True

    def on_measurement(self, z, t, cam_pos, R_cw):
        """Returns True if the measurement was used (update or initialisation)."""
        if not self.has_estimate:
            return self._try_init(z, cam_pos, R_cw, t)
        out = self.ekf.innovation(z, self.cam, cam_pos, R_cw)
        if out is not None:
            y, H, S = out
            self.last_nis = float(y @ np.linalg.solve(S, y))   # normalised innovation squared
            if self.last_nis <= self.cfg.gate_chi2:
                self.ekf.update(y, H, S, self.cam.R)
                self.status, self.t_last, self.n_rejects = TrackStatus.TRACK, t, 0
                return True
        # Rejected. Many rejections in a row means the filter, not the sensor, is wrong:
        # drop the track and let the confirmation logic start a new one.
        self.n_rejects += 1
        if self.n_rejects >= self.cfg.max_rejects:
            self.status, self.candidate = TrackStatus.SEARCH, None
            return self._try_init(z, cam_pos, R_cw, t)
        return False

    def update_status(self, t):
        if not self.has_estimate:
            return
        age = t - self.t_last
        if age > self.cfg.lost_time:
            self.status = TrackStatus.SEARCH
        elif age > self.cfg.coast_time:
            self.status = TrackStatus.COAST
