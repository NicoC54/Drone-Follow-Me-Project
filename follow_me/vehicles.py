"""Truth models: the leader (scripted) and the follower (3D point mass)."""
import numpy as np

from .config import FollowerConfig, LeaderConfig


class Leader:
    """Kinematic aircraft following a list of (speed, turn rate, climb rate) segments."""

    def __init__(self, cfg: LeaderConfig):
        self.cfg = cfg
        self.p = np.array(cfg.start_pos, dtype=float)
        self.psi = np.radians(cfg.start_heading_deg)
        self.speed = 0.0
        self.turn = 0.0
        self.vz = 0.0
        self.v = np.zeros(3)

    def _segment(self, t):
        seg = self.cfg.scenario[0]
        for s in self.cfg.scenario:
            if t >= s.t_start:
                seg = s
        return seg

    def step(self, t, dt):
        c, seg = self.cfg, self._segment(t)
        dv = np.clip(seg.speed - self.speed, -c.max_accel * dt, c.max_accel * dt)
        self.speed += dv
        self.turn += (np.radians(seg.turn_rate) - self.turn) * dt / c.tau_turn
        self.vz += (seg.climb_rate - self.vz) * dt / c.tau_climb
        self.psi += self.turn * dt
        self.v = np.array([self.speed * np.cos(self.psi), self.speed * np.sin(self.psi), self.vz])
        self.p = self.p + self.v * dt


class Follower:
    """3D point mass driven by an acceleration command.

    Thrust/attitude are abstracted: the command is the net acceleration (gravity already
    compensated). The attitude loop is modelled as a first-order lag `tau_accel`.
    Disturbances: linear drag on airspeed (mean wind) + correlated gusts.
    """

    def __init__(self, cfg: FollowerConfig, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        self.p = np.array(cfg.start_pos, dtype=float)
        self.v = np.zeros(3)
        self.a = np.zeros(3)       # achieved acceleration (before disturbances)
        self.gust = np.zeros(3)
        self.wind = np.array(cfg.wind, dtype=float)

    def step(self, a_cmd, dt):
        c = self.cfg
        self.a += (a_cmd - self.a) * dt / c.tau_accel
        # Ornstein-Uhlenbeck gust: stationary std = gust_sigma
        self.gust += (-self.gust * dt / c.gust_tau
                      + c.gust_sigma * np.sqrt(2 * dt / c.gust_tau) * self.rng.standard_normal(3))
        acc = self.a - c.drag * (self.v - self.wind) + self.gust
        self.v = self.v + acc * dt
        self.p = self.p + self.v * dt
