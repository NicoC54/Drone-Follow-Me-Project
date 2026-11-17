"""Gimbal camera: pinhole projection, detector model, Jacobian, back-projection.

Measurement z = [u, v, s]:
    u, v : pixel coordinates of the bounding-box centre
    s    : apparent size of the target in pixels (s = fx * W / depth)

Camera frame: x_c right, y_c down, z_c along the optical axis.
"""
from enum import IntEnum

import numpy as np

from .config import CameraConfig


class MeasKind(IntEnum):
    NOT_VISIBLE = 0  # outside the field of view, too far or too small
    MISSED = 1       # visible but the detector missed it
    DETECTION = 2    # true (noisy) detection
    OUTLIER = 3      # false detection


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rot_world_to_cam(yaw, pitch):
    """Rotation matrix R_cw such that r_cam = R_cw @ r_world (rows = camera axes in world)."""
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    forward = np.array([cp * cy, cp * sy, sp])
    right = np.array([sy, -cy, 0.0])
    down = np.cross(forward, right)
    return np.vstack([right, down, forward])


def look_at(p_from, p_to):
    """Yaw and pitch (world frame) that point the optical axis from p_from to p_to."""
    r = p_to - p_from
    return np.arctan2(r[1], r[0]), np.arctan2(r[2], np.hypot(r[0], r[1]))


class PinholeCamera:
    def __init__(self, cfg: CameraConfig):
        self.cfg = cfg
        self.cx, self.cy = cfg.width / 2, cfg.height / 2
        self.R = np.diag([cfg.sigma_px ** 2, cfg.sigma_px ** 2, cfg.sigma_size_px ** 2])

    # --- measurement model h(x) and its Jacobian, shared by the simulator and the EKF ---
    def h(self, p_target, cam_pos, R_cw):
        rc = R_cw @ (p_target - cam_pos)
        x, y, z = rc
        c = self.cfg
        return np.array([c.fx * x / z + self.cx, c.fy * y / z + self.cy, c.fx * c.target_size / z]), rc

    def H_pos(self, rc, R_cw):
        """d h / d p_target  (3x3).  Chain rule: d h/d rc  @  d rc/d p  = J @ R_cw."""
        x, y, z = rc
        c = self.cfg
        J = np.array([
            [c.fx / z, 0.0, -c.fx * x / z ** 2],
            [0.0, c.fy / z, -c.fy * y / z ** 2],
            [0.0, 0.0, -c.fx * c.target_size / z ** 2],
        ])
        return J @ R_cw

    def backproject(self, z_meas, cam_pos, R_cw):
        """Invert h: pixel + size -> world position, and its covariance (first order)."""
        u, v, s = z_meas
        c = self.cfg
        depth = c.fx * c.target_size / s
        rc = np.array([(u - self.cx) * depth / c.fx, (v - self.cy) * depth / c.fy, depth])
        sig_lat = depth * c.sigma_px / c.fx
        sig_depth = depth * c.sigma_size_px / s   # grows with depth^2: range is the weak axis
        P_cam = np.diag([sig_lat ** 2, sig_lat ** 2, sig_depth ** 2])
        return cam_pos + R_cw.T @ rc, R_cw.T @ P_cam @ R_cw

    def visible(self, z, rc):
        c = self.cfg
        return (rc[2] > 0.1 and rc[2] <= c.max_range
                and 0 <= z[0] < c.width and 0 <= z[1] < c.height and z[2] >= c.min_size_px)

    # --- detector ---
    def measure(self, p_target, cam_pos, R_cw, rng):
        c = self.cfg
        if rng.random() < c.p_outlier:
            z = np.array([rng.uniform(0, c.width), rng.uniform(0, c.height),
                          rng.uniform(c.min_size_px, 60.0)])
            return z, MeasKind.OUTLIER
        z_true, rc = self.h(p_target, cam_pos, R_cw)
        if not self.visible(z_true, rc):
            return None, MeasKind.NOT_VISIBLE
        if rng.random() < c.p_miss:
            return None, MeasKind.MISSED
        noise = rng.standard_normal(3) * [c.sigma_px, c.sigma_px, c.sigma_size_px]
        return z_true + noise, MeasKind.DETECTION


class Gimbal:
    """Stabilised 2-axis gimbal; yaw/pitch are world-frame angles, rate limited."""

    def __init__(self, cfg: CameraConfig, yaw=0.0, pitch=0.0):
        self.cfg = cfg
        self.yaw, self.pitch = yaw, pitch
        self.rate_max = np.radians(cfg.gimbal_rate_max_deg)

    def point(self, yaw_des, pitch_des, dt):
        k, rmax = self.cfg.gimbal_gain, self.rate_max
        self.yaw = wrap_angle(self.yaw + np.clip(k * wrap_angle(yaw_des - self.yaw), -rmax, rmax) * dt)
        self.pitch += np.clip(k * (pitch_des - self.pitch), -rmax, rmax) * dt
        self.pitch = np.clip(self.pitch, np.radians(self.cfg.gimbal_pitch_min_deg),
                             np.radians(self.cfg.gimbal_pitch_max_deg))

    def sweep(self, dt):
        """Search pattern: rotate in yaw, bring pitch back to the horizon."""
        self.point(self.yaw + np.radians(self.cfg.search_rate_deg) / self.cfg.gimbal_gain, 0.0, dt)

    def R_cw(self):
        return rot_world_to_cam(self.yaw, self.pitch)
