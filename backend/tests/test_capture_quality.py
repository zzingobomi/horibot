"""Phase 1 Traffic Light (capture_quality) 단위 테스트 — 스펙 MVP1 검증.

GREEN/YELLOW/RED 판정이 스펙 기준 (검출/tilt/pose·rotation·translation diversity)
대로 나오는지. 순수 함수라 빠름.
"""

from __future__ import annotations

import cv2
import numpy as np

from modules.calibration import thresholds as T
from modules.calibration.capture_quality import evaluate_capture_quality


def _R(deg_axis):
    return cv2.Rodrigues(np.array(deg_axis, dtype=float))[0]


GOOD_TILT = (T.TILT_MIN_DEG + T.TILT_MAX_DEG) / 2  # 범위 한가운데


def test_not_detected_red():
    q = evaluate_capture_quality(
        detected=False, tilt_deg=None, current_joints_rad=None,
        current_R_t2c=None, current_t_t2c=None,
        existing_joints_rad=[], existing_R_t2c=[], existing_t_t2c=[],
    )
    assert q.verdict == "red"


def test_tilt_too_frontal_red():
    q = evaluate_capture_quality(
        detected=True, tilt_deg=T.TILT_MIN_DEG - 5, current_joints_rad=[0] * 6,
        current_R_t2c=np.eye(3), current_t_t2c=np.zeros(3),
        existing_joints_rad=[], existing_R_t2c=[], existing_t_t2c=[],
    )
    assert q.verdict == "red"
    assert "tilt" in q.reasons[0]


def test_tilt_too_steep_red():
    q = evaluate_capture_quality(
        detected=True, tilt_deg=T.TILT_MAX_DEG + 5, current_joints_rad=[0] * 6,
        current_R_t2c=np.eye(3), current_t_t2c=np.zeros(3),
        existing_joints_rad=[], existing_R_t2c=[], existing_t_t2c=[],
    )
    assert q.verdict == "red"


def test_first_pose_green():
    q = evaluate_capture_quality(
        detected=True, tilt_deg=GOOD_TILT, current_joints_rad=[0.1] * 6,
        current_R_t2c=np.eye(3), current_t_t2c=np.array([0, 0, 0.3]),
        existing_joints_rad=[], existing_R_t2c=[], existing_t_t2c=[],
    )
    assert q.verdict == "green"


def test_nearly_identical_pose_red():
    """기존과 거의 같은 자세 (joint diff 작음 + 회전 차이 작음) → RED."""
    joints = [0.5, 0.3, 0.2, 0.1, 0.0, 0.0]
    R = _R([0.1, 0.2, 0.0])
    t = np.array([0.0, 0.0, 0.3])
    q = evaluate_capture_quality(
        detected=True, tilt_deg=GOOD_TILT,
        current_joints_rad=joints, current_R_t2c=R, current_t_t2c=t,
        existing_joints_rad=[list(joints)], existing_R_t2c=[R], existing_t_t2c=[t],
    )
    assert q.verdict == "red", q
    assert "거의 같은" in q.reasons[0]


def test_weak_rotation_diversity_yellow():
    """다른 joint 인데 board 회전은 기존과 거의 같음 → YELLOW (회전 더)."""
    existing_j = [0.0, 0.3, 0.2, 0.1, 0.0, 0.0]
    cur_j = [1.2, 0.3, 0.2, 0.1, 0.0, 0.0]  # J1 크게 다름 (joint diversity 충분)
    R = _R([0.1, 0.2, 0.0])  # 회전 동일
    t_exist = np.array([0.0, 0.0, 0.3])
    t_cur = np.array([0.10, 0.0, 0.3])  # translation 충분히 다름
    q = evaluate_capture_quality(
        detected=True, tilt_deg=GOOD_TILT,
        current_joints_rad=cur_j, current_R_t2c=R, current_t_t2c=t_cur,
        existing_joints_rad=[existing_j], existing_R_t2c=[R], existing_t_t2c=[t_exist],
    )
    assert q.verdict == "yellow", q
    assert any("회전" in r for r in q.reasons)


def test_full_diversity_green():
    """joint·회전·translation 다 충분히 다름 → GREEN."""
    existing_j = [0.0, 0.3, 0.2, 0.1, 0.0, 0.0]
    cur_j = [1.0, -0.2, 0.6, 0.5, 0.3, 0.4]
    q = evaluate_capture_quality(
        detected=True, tilt_deg=GOOD_TILT,
        current_joints_rad=cur_j,
        current_R_t2c=_R([0.6, -0.4, 0.3]), current_t_t2c=np.array([0.12, -0.08, 0.35]),
        existing_joints_rad=[existing_j],
        existing_R_t2c=[_R([0.1, 0.2, 0.0])], existing_t_t2c=[np.array([0.0, 0.0, 0.3])],
    )
    assert q.verdict == "green", q
