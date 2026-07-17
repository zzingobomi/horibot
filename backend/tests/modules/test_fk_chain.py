"""FkChain (Motion·Calibration 공용 기구학 모델) gate 테스트.

핵심 게이트: **offset 0 (zero_offset=0, link_correction=0, sag=0) 일 때
FkChain.fk ≈ PybulletKinematics.fk** (tolerance 내).

이게 통과하면 offline BA 가 이후 흔드는 모든 항(joint/link/sag offset)이 "기본 FK 를
망치지 않는다"는 안전망 — 두 FK 가 같은 baseline 에서 출발함을 보장. FkChain 과
PyBullet 이 URDF 를 독립적으로 파싱하므로, chain joint **순서 일치**도 함께 검증됨
(순서 어긋나면 FK 불일치로 fail).

순수 compute (yourdfpy + PyBullet DIRECT) — 하드웨어 불필요, 회사 검증 가능.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from apps.config import load_robots
from modules.motion.adapters.pybullet import PybulletKinematics
from modules.motion.fk_chain import FkChain
from modules.motor.contract import MotorKind

_URDF = (
    Path(__file__).resolve().parents[3]  # tests/modules → backend → repo root
    / "robot"
    / "so101_6dof"
    / "urdf"
    / "so101_6dof.urdf"
)

# FK 두 구현 일치 tolerance — 독립 파싱이라 완전 동일하진 않아도 수치상 무시 수준.
_POS_TOL_M = 1e-4  # 0.1mm
_ROT_TOL_DEG = 0.05

# PyBullet+yourdfpy URDF 부팅 (module-scope 1회) — 마커 정의 그대로 sim
pytestmark = pytest.mark.sim

# limit 내 대표 자세 (rad) — so101_6dof motors.yaml URDF limit 참고.
_POSES = [
    [0.0, 0.5, -1.5, 0.0, 0.0, 1.57],
    [1.0, 0.2, -0.5, 0.8, -0.7, 0.5],
    [-1.2, 2.0, -2.5, -1.0, 1.2, 2.5],
    [0.3, 1.0, -1.0, 1.4, -1.4, 3.0],
    [-0.6, 0.0, -3.0, -0.5, 0.9, 0.2],
]


def _arm_joint_names() -> list[str]:
    """실 배선과 동일하게 motors.yaml (RobotConfig) 에서 arm joint 이름 추출."""
    robot = load_robots()["so101_6dof_0"]
    return [m.name for m in robot.motors if m.kind != MotorKind.GRIPPER]


def _rot_angle_deg(r_a: np.ndarray, r_b: np.ndarray) -> float:
    """두 회전행렬 사이 각도 차 (deg)."""
    r_rel = r_a @ r_b.T
    tr = (np.trace(r_rel) - 1.0) * 0.5
    return float(np.rad2deg(np.arccos(np.clip(tr, -1.0, 1.0))))


@pytest.fixture(scope="module")
def kin():
    k = PybulletKinematics(_URDF)
    k.initialize()
    yield k
    k.close()


@pytest.fixture(scope="module")
def fk_chain() -> FkChain:
    return FkChain(_URDF, _arm_joint_names(), tcp_link_name="tcp")


def test_arm_names_and_dof(fk_chain: FkChain, kin: PybulletKinematics):
    # motors.yaml arm(gripper 제외) ↔ URDF tcp chain 이 같은 관절 수 — 이름
    # 리스트 하드코드는 config 미러라 잠그지 않는다 (순서 오류는 FK 일치
    # 테스트가 불일치로 잡음).
    assert fk_chain.n_arm == kin.dof == len(_arm_joint_names())


@pytest.mark.parametrize("angles", _POSES)
def test_fk_matches_pybullet(
    fk_chain: FkChain, kin: PybulletKinematics, angles: list[float]
):
    """offset 0 baseline 에서 FkChain.fk == PybulletKinematics.fk."""
    r_fk, t_fk = fk_chain.fk(np.array(angles))

    rot_pb, pos_pb = kin.fk_to_matrix(angles)
    r_pb = np.array(rot_pb, dtype=np.float64)
    t_pb = np.array(pos_pb, dtype=np.float64)

    pos_err = float(np.linalg.norm(t_fk - t_pb))
    rot_err = _rot_angle_deg(r_fk, r_pb)

    assert pos_err < _POS_TOL_M, f"pos 오차 {pos_err*1000:.4f}mm @ {angles}"
    assert rot_err < _ROT_TOL_DEG, f"rot 오차 {rot_err:.4f}° @ {angles}"


def test_fk_matches_pybullet_random(fk_chain: FkChain, kin: PybulletKinematics):
    """joint_limits 내 랜덤 자세 다수 — 두 FK 가 전 영역에서 일치 (seeded)."""
    rng = np.random.default_rng(42)
    limits = kin.joint_limits()  # [(lo, hi)] × 6
    for _ in range(30):
        angles = [float(rng.uniform(lo, hi)) for lo, hi in limits]
        r_fk, t_fk = fk_chain.fk(np.array(angles))
        rot_pb, pos_pb = kin.fk_to_matrix(angles)
        pos_err = float(np.linalg.norm(t_fk - np.array(pos_pb)))
        rot_err = _rot_angle_deg(r_fk, np.array(rot_pb, dtype=np.float64))
        assert pos_err < _POS_TOL_M, f"pos 오차 {pos_err*1000:.4f}mm @ {angles}"
        assert rot_err < _ROT_TOL_DEG, f"rot 오차 {rot_err:.4f}° @ {angles}"


def test_zero_offsets_are_noop(fk_chain: FkChain):
    """link_trans/rot=0 명시 == None (기본 FK). BA baseline 불변 안전망."""
    angles = np.array([0.2, 0.4, -1.0, 0.3, -0.3, 1.0])
    r0, t0 = fk_chain.fk(angles)
    r1, t1 = fk_chain.fk(
        angles,
        link_trans=np.zeros((6, 3)),
        link_rot=np.zeros((6, 3)),
    )
    assert np.allclose(t0, t1) and np.allclose(r0, r1)
