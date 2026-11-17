"""All tunable parameters in one place.

Conventions used everywhere in the project:
- World frame: x = East, y = North, z = Up (ENU), metres.
- Angles: radians in code, degrees only in config fields whose name ends in `_deg`.
"""
from dataclasses import dataclass, field


@dataclass
class Segment:
    """One leg of the leader's flight plan, active from `t_start` until the next segment."""
    t_start: float      # [s]
    speed: float        # commanded horizontal speed [m/s]
    turn_rate: float    # commanded turn rate [deg/s], positive = left (counter-clockwise)
    climb_rate: float   # commanded vertical speed [m/s]


def default_scenario() -> list:
    return [
        Segment(0.0, 0.0, 0.0, 0.0),     # hover: follower acquires the target
        Segment(3.0, 5.0, 0.0, 0.0),     # straight cruise
        Segment(14.0, 5.0, 20.0, 0.0),   # left turn
        Segment(21.0, 8.0, 0.0, 0.0),    # faster
        Segment(30.0, 8.0, 0.0, 2.0),    # climb
        Segment(36.0, 8.0, -25.0, 0.0),  # S-turn, right...
        Segment(43.0, 6.0, 25.0, 0.0),   # ...then left
        Segment(50.0, 4.0, 0.0, 0.0),    # slow down
        Segment(58.0, 0.0, 0.0, 0.0),    # stop (heading of the target becomes undefined)
        Segment(66.0, 7.0, -40.0, -1.0), # sharp turn + descent from hover
        Segment(72.0, 10.0, 0.0, 0.0),   # dash
        Segment(82.0, 3.0, 0.0, 0.0),    # slow
    ]


@dataclass
class SimConfig:
    dt: float = 0.01          # dynamics / control step [s] (100 Hz)
    duration: float = 90.0    # [s]
    seed: int = 0


@dataclass
class LeaderConfig:
    start_pos: tuple = (0.0, 0.0, 20.0)
    start_heading_deg: float = 0.0
    max_accel: float = 2.5    # longitudinal acceleration limit [m/s^2]
    tau_turn: float = 0.7     # first-order lag on turn rate [s]
    tau_climb: float = 1.0    # first-order lag on climb rate [s]
    scenario: list = field(default_factory=default_scenario)


@dataclass
class FollowerConfig:
    start_pos: tuple = (-12.0, -5.0, 18.0)
    tau_accel: float = 0.15   # lag between commanded and achieved acceleration (attitude loop) [s]
    a_max: float = 6.0        # [m/s^2]
    v_max: float = 14.0       # [m/s]
    drag: float = 0.15        # linear drag coefficient on airspeed [1/s]
    wind: tuple = (2.0, -1.0, 0.0)  # mean wind [m/s]
    gust_sigma: float = 0.4   # std of gust acceleration (Ornstein-Uhlenbeck) [m/s^2]
    gust_tau: float = 2.0     # gust correlation time [s]


@dataclass
class CameraConfig:
    rate_hz: float = 30.0
    width: int = 1280
    height: int = 720
    fx: float = 800.0         # focal length [px]  -> ~77 deg horizontal FOV
    fy: float = 800.0
    target_size: float = 0.5  # known leader span [m], gives range from apparent size
    min_size_px: float = 3.0  # below this the detector cannot see the target
    max_range: float = 80.0   # [m]
    sigma_px: float = 2.0     # pixel noise on the bounding-box centre [px]
    sigma_size_px: float = 1.0  # noise on the apparent size [px]
    p_miss: float = 0.10      # probability of a missed detection
    p_outlier: float = 0.02   # probability of a false detection somewhere in the image
    occlusions: list = field(default_factory=list)  # [(t0, t1), ...] windows with no detection
    # Gimbal (stabilised, angles expressed in world frame)
    gimbal_gain: float = 8.0          # proportional gain [1/s]
    gimbal_rate_max_deg: float = 120.0
    gimbal_pitch_min_deg: float = -90.0
    gimbal_pitch_max_deg: float = 60.0
    search_rate_deg: float = 40.0     # yaw sweep rate when the target is lost


@dataclass
class EstimatorConfig:
    q_accel: float = 1.5       # white-acceleration spectral density of the CV model [m^2/s^3]
    init_vel_sigma: float = 5.0  # [m/s]
    gate_chi2: float = 16.27   # chi2(3 dof) at 99.9 %: reject innovations beyond this
    max_rejects: int = 10      # consecutive rejections before dropping the track
    confirm_window: float = 0.3  # 2nd detection must come within this delay to start a track [s]
    coast_time: float = 0.5    # no update for this long -> COAST [s]
    lost_time: float = 3.0     # no update for this long -> SEARCH [s]


@dataclass
class GuidanceConfig:
    follow_dist: float = 8.0       # horizontal stand-off behind the leader [m]
    height_offset: float = 2.0     # follower flies this much above the leader [m]
    v_heading_lo: float = 0.5      # below this leader speed, keep the current bearing [m/s]
    v_heading_hi: float = 2.0      # above this, place the follower straight behind [m/s]
    dir_gain: float = 1.0           # bandwidth of the follow-point bearing filter [1/s]
    dir_rate_max_deg: float = 45.0  # max rotation rate of the follow point around the leader


@dataclass
class SafetyConfig:
    d_min: float = 4.0     # hard minimum separation [m]
    buffer: float = 3.0    # the filter starts acting at d_min + buffer [m]
    k_sigma: float = 2.0   # shrink the measured distance by k_sigma * estimation std
    k_rep: float = 6.0     # repulsive acceleration at d_min [m/s^2]
    k_damp: float = 2.0    # gain on closing speed [1/s]


@dataclass
class ControlConfig:
    kp_pos: float = 0.8    # position -> velocity gain [1/s]
    kv: float = 2.5        # velocity -> acceleration gain [1/s]
    ki: float = 0.4        # velocity integral gain [1/s^2] (cancels drag and wind)
    i_max: float = 2.0     # integral clamp [m/s^2]


@dataclass
class Config:
    sim: SimConfig = field(default_factory=SimConfig)
    leader: LeaderConfig = field(default_factory=LeaderConfig)
    follower: FollowerConfig = field(default_factory=FollowerConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    estimator: EstimatorConfig = field(default_factory=EstimatorConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
