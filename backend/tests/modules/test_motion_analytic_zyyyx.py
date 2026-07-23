"""OMX 5축 Z·YYY·X closed-form IK — 실 URDF PyBullet 대조 (sim).

잠그는 것 (omx_handover_prep.md §8-2 / 구현순서 3):
  ① 부팅 활성 — omx_f URDF 에서 해석기가 실제로 붙는다 (침묵 수치 폴백 회귀).
     실측 정정 (2026-07-23): EAIK 가 omx 5축도 분해한다 → 기본 빌드는 EAIK,
     Z·YYY·X 는 **EAIK 부재 환경(Pi 소스빌드 실패) 백업** — 백업 경로는
     EAIK 를 끈 픽스처로 따로 잠근다.
  ② **완전성**: 무작위 관절 구성의 FK pose 를 되풀면 해가 나온다 (수치 IK 의
     "해 있는데 못 찾음" false-negative 클래스가 소멸했는지 — 흉터 5·6)
  ③ 정밀도: 해의 FK 재확인 — 위치/자세 잔차 (polish 계약)
  ④ top-down 파지 manifold — J5 roll 격자 전체 도달 (§5.1: top-down 은 제약
     5개 = 관절 5개 = 정확 도달)
  ⑤ 도달불가 확정 — 5축이 못 만드는 자세(위치-방위 모순)는 None
  ⑥ so101 (6R) 은 기존 EAIK 경로 그대로 (회귀 없음)
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from apps.config import _ROBOT_DIR, load_robots
from modules.motion.adapters.analytic_zyyyx import ZyyyxAnalyticIk
from modules.motion.adapters.pybullet import PybulletKinematics

pytestmark = pytest.mark.sim  # PyBullet/URDF 부팅 — fast loop 제외

_TOPDOWN = np.column_stack([[0, 0, -1], [0, 1, 0], [1, 0, 0]])


def _omx_urdf():
    robots = load_robots()
    t = robots["omx_f_0"].type
    return _ROBOT_DIR / t / "urdf" / f"{t}.urdf"


@pytest.fixture(scope="module")
def omx_kin():
    """omx 백업(Z·YYY·X) 경로 강제 — EAIK 빌드를 꺼서 closed-form 을 검증.

    기본 환경은 EAIK 가 omx 도 분해해 백업이 안 밟힌다 — Pi(EAIK 소스빌드
    실패) 환경의 실 경로를 여기서 잠근다. module scope 라 monkeypatch 픽스처
    대신 수동 patch/restore.
    """
    import modules.motion.adapters.pybullet as pb

    orig = pb.AnalyticIk.try_build
    pb.AnalyticIk.try_build = staticmethod(lambda *a, **k: None)  # type: ignore[method-assign]
    try:
        kin = PybulletKinematics(_omx_urdf())
        kin.initialize()
    finally:
        pb.AnalyticIk.try_build = orig  # type: ignore[method-assign]
    yield kin
    kin.close()


@pytest.fixture(scope="module")
def so_kin():
    robots = load_robots()
    t = robots["so101_6dof_0"].type
    kin = PybulletKinematics(_ROBOT_DIR / t / "urdf" / f"{t}.urdf")
    kin.initialize()
    yield kin
    kin.close()


def test_omx_backup_analytic_active(omx_kin):
    """① EAIK 부재 환경에서 Z·YYY·X closed-form 이 붙는다 (침묵 수치 폴백 회귀)."""
    assert isinstance(omx_kin._analytic, ZyyyxAnalyticIk), (
        "omx 백업 해석 IK 미활성 — 수치 폴백은 침묵 회귀 (부팅 로그 IK=해석적 확인)"
    )


def test_omx_default_build_has_analytic():
    """① 기본 빌드(EAIK 가용)도 해석 경로 활성 — 어느 구현이든 수치 폴백이면 회귀."""
    kin = PybulletKinematics(_omx_urdf())
    kin.initialize()
    try:
        assert kin._analytic is not None
    finally:
        kin.close()


def test_so101_still_uses_eaik(so_kin):
    """⑥ so101 은 기존 EAIK 경로 유지 (Z·YYY·X 가 6R 을 가로채면 회귀)."""
    assert so_kin._analytic is not None
    assert not isinstance(so_kin._analytic, ZyyyxAnalyticIk)


def test_fk_roundtrip_completeness(omx_kin):
    """② + ③ 무작위 구성 → FK → ik() 가 해를 찾고, 그 해의 FK 가 목표와 일치.

    해석 branch 열거가 완전하면 "존재하는 해를 못 찾는" 일이 없어야 한다.
    (수치 IK 는 이 성질이 없어서 walk/restart 가 자랐다 — motion.md §11.)
    """
    rng = np.random.default_rng(7)
    limits = omx_kin.joint_limits()
    found, total = 0, 0
    for _ in range(40):
        q = [
            float(rng.uniform(lo + 0.1, hi - 0.1)) for lo, hi in limits
        ]
        if omx_kin.self_collision(q):
            continue  # 자기충돌 구성은 ik 가 정당히 기각할 수 있음 — 표본 제외
        pos, quat = omx_kin.fk(q)
        total += 1
        sol = omx_kin.ik(pos, quat, current_joint_angles=[0.0] * 5)
        if sol is None:
            continue
        found += 1
        got_pos, got_quat = omx_kin.fk(sol)
        assert np.linalg.norm(np.array(got_pos) - np.array(pos)) < 0.010, (
            q, sol,
        )
        dot = abs(sum(a * b for a, b in zip(got_quat, quat)))
        assert 2.0 * math.acos(min(1.0, dot)) < math.radians(5.0), (q, sol)
    assert total >= 20, "유효 표본 부족 — 리밋/충돌 세팅 확인"
    # 완전성: FK 로 만든 pose 는 정의상 도달 가능 — 전부 찾아야 한다
    assert found == total, f"해 놓침 {total - found}/{total} — 완전성 계약 위반"


def test_topdown_manifold_reaches_roll_grid(omx_kin):
    """④ top-down(-z 접근) × J5 roll 격자 — 도달영역 안 파지 pose 전부 해결.

    §5.1: 접근축이 수직이면 방위각 degenerate → 제약 5 = 관절 5 = 정확 도달.
    격자점은 §8-1 실측 도달영역(26×44cm, centroid (0.208, 0)) 안쪽만.
    """
    ok, tried = 0, 0
    for r, az_deg in ((0.18, 0.0), (0.24, 0.0), (0.21, 25.0), (0.21, -25.0)):
        az = math.radians(az_deg)
        pos = (r * math.cos(az), r * math.sin(az), 0.02)
        for roll_deg in range(0, 180, 30):
            yaw = az + math.radians(roll_deg)
            rot = Rotation.from_euler("z", yaw).as_matrix() @ _TOPDOWN
            q = Rotation.from_matrix(rot).as_quat()
            quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            tried += 1
            sol = omx_kin.ik(pos, quat, current_joint_angles=[0.0] * 5)
            if sol is not None:
                got_pos, _ = omx_kin.fk(sol)
                assert (
                    np.linalg.norm(np.array(got_pos) - np.array(pos)) < 0.010
                )
                ok += 1
    # 전 격자 도달을 요구하진 않는다 (리밋/자기충돌 경계) — 대다수는 도달해야
    assert ok >= tried * 0.8, f"top-down 도달 {ok}/{tried} — manifold 회귀 의심"


def test_unreachable_orientation_confirmed_none(omx_kin):
    """⑤ 위치(방위 0°)와 자세(방위 60° 요구)가 모순인 pose — 5축 도달불가 확정.

    해석 경로는 "못 찾음" 이 아니라 "없음" 을 돌려준다 (전 branch 자세 잔차
    기각). 수치처럼 예산 복권이 아니다.
    """
    pos = (0.25, 0.0, 0.10)
    rot = Rotation.from_euler("z", math.radians(60.0)).as_matrix()  # 순수 yaw 자세
    q = Rotation.from_matrix(rot).as_quat()
    sol = omx_kin.ik(
        pos, (float(q[0]), float(q[1]), float(q[2]), float(q[3])),
        current_joint_angles=[0.0] * 5,
    )
    assert sol is None
