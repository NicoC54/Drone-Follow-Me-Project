"""Control: cascaded position/velocity controller + separation safety filter."""
import numpy as np

from .config import ControlConfig, SafetyConfig


def saturate(vec, max_norm):
    n = np.linalg.norm(vec)
    return vec if n <= max_norm else vec * (max_norm / n)


class PositionController:
    """Outer loop: position error -> velocity command (P + feed-forward).
    Inner loop: velocity error -> acceleration command (PI).

    Without integral term, drag and wind leave a steady-state lag behind a moving leader.
    """

    def __init__(self, cfg: ControlConfig, a_max, v_max):
        self.cfg, self.a_max, self.v_max = cfg, a_max, v_max
        self.integral = np.zeros(3)

    def reset(self):
        self.integral[:] = 0.0

    def compute(self, p, v, p_des, v_des, dt):
        c = self.cfg
        v_cmd = saturate(v_des + c.kp_pos * (p_des - p), self.v_max)
        e_v = v_cmd - v
        self.integral = saturate(self.integral + c.ki * e_v * dt, c.i_max)
        return saturate(c.kv * e_v + self.integral, self.a_max)


class SafetyFilter:
    """Last layer before the motors: enforce the minimum separation.

    Works on the ESTIMATED leader position, made conservative by subtracting
    k_sigma standard deviations of the estimate along the line of sight.
    Inside the buffer zone it (1) removes the part of the command that pushes
    towards the leader and (2) adds a repulsive + closing-speed damping term.
    """

    def __init__(self, cfg: SafetyConfig, a_max):
        self.cfg, self.a_max = cfg, a_max

    def filter(self, a_cmd, p_f, v_f, p_t, v_t, P_pos):
        c = self.cfg
        rel = p_f - p_t
        d = max(np.linalg.norm(rel), 1e-6)
        n = rel / d                                    # unit vector leader -> follower
        sigma = np.sqrt(max(n @ P_pos @ n, 0.0))
        margin = d - c.k_sigma * sigma - c.d_min
        if margin >= c.buffer:
            return a_cmd, False
        s = np.clip(1.0 - margin / c.buffer, 0.0, 2.0)  # 0 at buffer edge, 1 at d_min
        a = a_cmd.copy()
        towards = -(a @ n)
        if towards > 0:
            a += min(s, 1.0) * towards * n
        closing = -((v_f - v_t) @ n)
        a += (c.k_rep * s + c.k_damp * max(closing, 0.0) * min(s, 1.0)) * n
        return saturate(a, self.a_max), True
