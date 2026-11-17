import numpy as np
import pytest

from follow_me.camera import PinholeCamera, rot_world_to_cam
from follow_me.config import CameraConfig, Config
from follow_me.metrics import compute_metrics
from follow_me.sim import run


def test_rotation_is_orthonormal_and_points_forward():
    R = rot_world_to_cam(0.7, -0.3)
    assert np.allclose(R @ R.T, np.eye(3))
    assert np.isclose(np.linalg.det(R), 1.0)
    target = np.array([np.cos(-0.3) * np.cos(0.7), np.cos(-0.3) * np.sin(0.7), np.sin(-0.3)])
    assert np.allclose(R @ target, [0, 0, 1])


def test_measurement_jacobian_matches_finite_differences():
    cam = PinholeCamera(CameraConfig())
    R, cam_pos, p = rot_world_to_cam(0.3, 0.1), np.zeros(3), np.array([8.0, 3.0, 1.5])
    _, rc = cam.h(p, cam_pos, R)
    H = cam.H_pos(rc, R)
    eps = 1e-6
    H_num = np.column_stack([(cam.h(p + eps * e, cam_pos, R)[0] - cam.h(p - eps * e, cam_pos, R)[0])
                             / (2 * eps) for e in np.eye(3)])
    assert np.allclose(H, H_num, rtol=1e-5, atol=1e-6)


def test_backprojection_inverts_projection():
    cam = PinholeCamera(CameraConfig())
    R, cam_pos, p = rot_world_to_cam(-1.0, 0.2), np.array([1.0, 2.0, 3.0]), np.array([-5.0, -12.0, 5.0])
    z, _ = cam.h(p, cam_pos, R)
    p_back, _ = cam.backproject(z, cam_pos, R)
    assert np.allclose(p_back, p)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_closed_loop_keeps_safe_distance_and_tracks(seed):
    cfg = Config()
    cfg.sim.seed = seed
    m, _ = compute_metrics(run(cfg))
    assert m["min distance [m]"] > cfg.safety.d_min
    assert m["estimation position RMSE [m]"] < 0.5
    assert m["distance error RMS [m]"] < 2.0
    assert m["time with valid track [%]"] > 95
