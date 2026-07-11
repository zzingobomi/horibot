"""cross_calibrate 순수 수학 round-trip — 정답을 아는 합성 geometry 로 검증.

두 robot base 의 상대 pose(정답)를 정해두고, 공유 보드 관측을 노이즈와 함께
합성 → 평균/합성/투영 파이프라인이 정답을 복원하는지. 부호/역행렬 방향 실수는
여기서만 잡힌다 (실 데이터는 정답을 모름).
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from scripts.cross_calibrate import (
    average_board_observations,
    compose_a_from_b,
    project_planar,
)


def _T(
    x: float,
    y: float,
    z: float,
    yaw_deg: float = 0.0,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rot.from_euler(
        "ZYX", [yaw_deg, pitch_deg, roll_deg], degrees=True
    ).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def _perturb(
    T: np.ndarray, rng: np.random.Generator, rot_deg: float, t_mm: float
) -> np.ndarray:
    """관측 노이즈 — 좌측 (관측 프레임) 소회전 + 평행이동."""
    dR = Rot.from_rotvec(
        rng.normal(size=3) * np.deg2rad(rot_deg) / np.sqrt(3)
    ).as_matrix()
    out = T.copy()
    out[:3, :3] = dR @ T[:3, :3]
    out[:3, 3] = T[:3, 3] + rng.normal(size=3) * (t_mm / 1000.0) / np.sqrt(3)
    return out


def test_round_trip_recovers_ground_truth_base_pose():
    """정답 T_a←b 를 심고 노이즈 관측 8장씩 → 복원 오차 < (0.5°, 5mm)."""
    T_a_b_true = _T(0.35, -0.12, 0.008, yaw_deg=32.0)  # 정답 배치
    T_a_board = _T(0.18, 0.06, 0.02, yaw_deg=15.0, pitch_deg=35.0)  # 보드 임의 자세
    T_b_board = np.linalg.inv(T_a_b_true) @ T_a_board

    rng = np.random.default_rng(7)
    obs_a = [_perturb(T_a_board, rng, 0.5, 4.0) for _ in range(8)]
    obs_b = [_perturb(T_b_board, rng, 0.5, 4.0) for _ in range(8)]

    stats_a = average_board_observations(obs_a)
    stats_b = average_board_observations(obs_b)
    T_est = compose_a_from_b(stats_a.T_base_board, stats_b.T_base_board)

    dT = np.linalg.inv(T_a_b_true) @ T_est
    rot_err = np.degrees(np.linalg.norm(Rot.from_matrix(dT[:3, :3]).as_rotvec()))
    t_err_mm = np.linalg.norm(dT[:3, 3]) * 1000.0
    assert rot_err < 0.5, f"회전 복원 오차 {rot_err:.3f}°"
    assert t_err_mm < 5.0, f"이동 복원 오차 {t_err_mm:.2f}mm"

    pose = project_planar(T_est)
    assert abs(pose.x - 0.35) < 0.005
    assert abs(pose.y - (-0.12)) < 0.005
    assert abs(pose.yaw_deg - 32.0) < 0.5
    # 정답이 planar 라 잔차 roll/pitch 도 노이즈 수준
    assert abs(pose.roll_deg) < 0.5 and abs(pose.pitch_deg) < 0.5


def test_project_planar_exact_on_noise_free_pose():
    """노이즈 0 인 planar pose → 투영이 정확히 복원 (수식 자체 검증)."""
    pose = project_planar(_T(0.4, -0.1, 0.0, yaw_deg=-90.0))
    assert pose.x == 0.4 and pose.y == -0.1 and pose.z == 0.0
    assert abs(pose.yaw_deg - (-90.0)) < 1e-9
    assert abs(pose.roll_deg) < 1e-9 and abs(pose.pitch_deg) < 1e-9


def test_project_planar_reports_tilt_residual():
    """base 가 기울어져 있으면 (planar 가정 위반) roll/pitch 잔차로 표면화 —
    조용히 yaw 로 뭉개지 않는다."""
    pose = project_planar(_T(0.3, 0.0, 0.0, yaw_deg=10.0, pitch_deg=4.0))
    assert abs(pose.pitch_deg - 4.0) < 1e-6
    assert abs(pose.yaw_deg - 10.0) < 1e-6


def test_average_scatter_flags_outlier_observation():
    """관측 중 하나가 크게 튀면 per-obs 흩어짐 max 에 그대로 드러난다 —
    리포트의 '보드 이동 의심' 경고 근거."""
    T = _T(0.2, 0.0, 0.1, pitch_deg=30.0)
    rng = np.random.default_rng(3)
    obs = [_perturb(T, rng, 0.3, 2.0) for _ in range(7)]
    bad = T.copy()
    bad[:3, 3] += [0.03, 0.0, 0.0]  # 30mm 튄 관측 (보드가 밀렸다면 이 모양)
    obs.append(bad)

    stats = average_board_observations(obs)
    assert max(stats.per_obs_t_mm) > 20.0  # outlier 가 지표에 표면화
    assert np.median(stats.per_obs_t_mm) < 10.0  # 나머지는 정상
