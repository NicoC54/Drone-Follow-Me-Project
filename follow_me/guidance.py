"""Guidance: where should the follower be, given the estimated leader state?"""
import numpy as np

from .camera import wrap_angle
from .config import GuidanceConfig


class FollowGuidance:
    """Follow point at `follow_dist` behind the leader, `height_offset` above it.

    "Behind" is defined by the leader's velocity when it moves fast enough. When it
    slows down, its heading is meaningless (noise), so we blend towards the current
    bearing leader -> follower, i.e. "stay where you are around it".
    The bearing of the follow point is rate limited so it never jumps around the leader.
    """

    def __init__(self, cfg: GuidanceConfig):
        self.cfg = cfg
        self.angle = None   # bearing of the follow point, seen from the leader

    def reset(self):
        self.angle = None

    def compute(self, p_f, p_t, v_t, dt):
        c = self.cfg
        rel = (p_f - p_t)[:2]
        rel_dir = rel / max(np.linalg.norm(rel), 1e-6)
        speed = np.linalg.norm(v_t[:2])
        w = np.clip((speed - c.v_heading_lo) / (c.v_heading_hi - c.v_heading_lo), 0.0, 1.0)
        behind = -v_t[:2] / speed if speed > 1e-6 else np.zeros(2)
        d = w * behind + (1 - w) * rel_dir
        if np.linalg.norm(d) < 1e-6:
            d = rel_dir
        if self.angle is None:
            self.angle = np.arctan2(rel_dir[1], rel_dir[0])
        # First-order tracking of the desired bearing, with a rate limit. A pure rate limit
        # (bang-bang) would make omega jump by +/- max rate at every step on a noisy heading.
        wmax = np.radians(c.dir_rate_max_deg)
        omega = np.clip(c.dir_gain * wrap_angle(np.arctan2(d[1], d[0]) - self.angle), -wmax, wmax)
        self.angle = wrap_angle(self.angle + omega * dt)

        u = np.array([np.cos(self.angle), np.sin(self.angle)])
        p_des = p_t + np.array([c.follow_dist * u[0], c.follow_dist * u[1], c.height_offset])
        # Feed-forward velocity: leader velocity + velocity of the rotating offset point
        v_des = v_t + c.follow_dist * omega * np.array([-u[1], u[0], 0.0])
        return p_des, v_des
