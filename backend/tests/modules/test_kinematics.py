"""PybulletKinematics (Motion D1) test — robot so101_6dof URDF.

순수 compute (PyBullet DIRECT) — 하드웨어 불필요, 회사 검증 가능.
검증: dof=6 (gripper 제외) / FK·IK roundtrip / joint_limits / unreachable→None.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from modules.motion.adapters.pybullet import (
    IK_POS_ERROR_LIMIT,
    PybulletKinematics,
)

_URDF = (
    Path(__file__).resolve().parents[3]  # tests/modules → backend → repo root
    / "robot"
    / "so101_6dof"
    / "urdf"
    / "so101_6dof.urdf"
)

# PyBullet+URDF 부팅 (~3s/test) — 마커 정의 그대로 sim
pytestmark = pytest.mark.sim


@pytest.fixture
def kin():
    k = PybulletKinematics(_URDF)
    k.initialize()
    yield k
    k.close()


def test_dof_is_6_gripper_excluded(kin: PybulletKinematics):
    # tcp ancestor chain = joint1..6. gripper(joint7)는 sibling 가지 → 제외.
    assert kin.dof == 6
    assert kin.tcp_link_name == "tcp"
    assert len(kin.joint_limits()) == 6


def test_fk_ik_roundtrip(kin: PybulletKinematics):
    joints = [0.1, 0.3, -0.4, 0.1, 0.2, 0.0]
    pos, quat = kin.fk(joints)
    assert len(pos) == 3 and len(quat) == 4

    solved = kin.ik(pos, quat, current_joint_angles=joints)
    assert solved is not None, "reachable pose IK 실패"
    assert len(solved) == 6

    # redundancy 로 joint 값은 다를 수 있음 → POSE 일치로 검증
    pos2, _ = kin.fk(solved)
    err = float(np.linalg.norm(np.array(pos) - np.array(pos2)))
    assert err < 1e-2, f"FK/IK roundtrip pose 오차 {err}"


def test_ik_unreachable_returns_none(kin: PybulletKinematics):
    # 팔 길이 밖 (2m) → 수렴 실패 (seed + restart 모두) → None
    assert kin.ik((2.0, 2.0, 2.0), None) is None


def test_ik_reachable_from_bad_seed_solves(kin: PybulletKinematics):
    """도달 가능한 target 은 seed 가 나빠도 IK 가 해를 찾아야 함 (multi-restart).

    single-seed local 솔버는 나쁜 basin 의 seed 에서 존재하는 해를 놓친다 —
    "잡을 수 있는 위치인데 IK 실패" 회귀. J2 를 앞으로 접은 낮은 target 을 FK 로
    만들어 확실히 reachable 하게 하고, 정반대 zero-seed 로 요청해 재시도가
    실제로 해를 살리는지 검증. multi-restart 를 seeded-only 로 되돌리면 실패.
    """
    reachable = [0.5, 2.0, -2.5, 1.0, -1.0, 1.5]  # J2 앞으로 접은 낮은 자세
    target_pos, _ = kin.fk(reachable)

    sol = kin.ik(target_pos, None, current_joint_angles=[0.0] * 6)  # 나쁜 seed
    assert sol is not None, "reachable target 인데 IK 실패 (multi-restart 회귀?)"
    pos2, _ = kin.fk(sol)
    err = float(np.linalg.norm(np.array(target_pos) - np.array(pos2)))
    assert err < 1e-2, f"IK 해의 FK 오차 {err}"


def _pos_err(kin: PybulletKinematics, joints: list[float], target) -> float:
    kin._set_chain(joints)
    actual, _ = kin._ee_state()
    return float(np.linalg.norm(np.array(actual) - np.array(target)))


def test_conditional_refine_recovers_pose_hard_residual(kin: PybulletKinematics):
    """docs/motion.md §10.B/§10.D 회귀 — 주 경로 conditional refine.

    자세가 빡센 목표에서 단발(raw) 수치 IK 는 위치를 내주고 자세를 맞추는 해로
    수렴 → 위치잔차가 크게(일부는 게이트 10mm 초과) 뜬다. 결과-seed 재해(refine)가
    회수한다. nominal 파지 자세에서 손목(J4/J5)을 tilt 사다리로 돌린 reachable
    목표(FK 생성) 스윕에서 집계로 검증 — 개별 config 은 solver 편차가 커 brittle.

    refine 을 떼면(_ik_from_seed==raw) refined==raw 라 세 단언(게이트 초과 회수 /
    최대 잔차 감소 / 평균 잔차 감소)이 동시에 깨진다.
    """
    nominal = [0.2, 0.9, -1.3, 0.6, 0.8, 0.3]
    raw_errs, refined_errs = [], []
    recovered_over_gate = 0  # 단발은 게이트 초과(기각)인데 refine 이 살린 수
    for tilt_deg in range(0, 70, 10):
        for roll_deg in (-40, 0, 40):
            cfg = list(nominal)
            cfg[3] += math.radians(tilt_deg)
            cfg[4] += math.radians(roll_deg)
            pos, quat = kin.fk(cfg)  # reachable by construction

            raw = kin._ik_raw(pos, quat, nominal)  # 단발 1회 (게이트/refine 없음)
            raw_err = _pos_err(kin, raw, pos)
            refined = kin._ik_from_seed(pos, quat, nominal)  # 주 경로(refine+게이트)
            refined_err = (
                _pos_err(kin, refined, pos) if refined is not None else math.inf
            )
            raw_errs.append(raw_err)
            refined_errs.append(refined_err)
            # ★ 불변식(§6, 2026-07-20 전멸 사고 회귀): 단발이 게이트를 통과하던
            #   (≤10mm) 후보를 refine 이 None 으로 뒤집거나 악화시키면 안 된다.
            #   마지막 반복(발산 포함)을 반환하면 7mm 던 후보가 12mm 로 뒤집혀
            #   게이트 초과 None → 여기서 잡힌다. best 추적이 이를 막는다.
            #   (단발>10mm 후보의 None 은 게이트 정상 동작 — ik() 에선 walk 로 감.)
            if raw_err <= IK_POS_ERROR_LIMIT:
                assert refined is not None, (
                    f"단발 통과({raw_err*1000:.1f}mm)를 refine 이 None 으로 뒤집음 "
                    f"(tilt={tilt_deg}° roll={roll_deg}°, best 미추적 회귀?)"
                )
                assert refined_err <= raw_err + 1e-4, (
                    f"refine 이 후보 악화(tilt={tilt_deg}° roll={roll_deg}°): "
                    f"단발 {raw_err*1000:.1f}mm → refine {refined_err*1000:.1f}mm"
                )
            if raw_err > IK_POS_ERROR_LIMIT and refined_err <= IK_POS_ERROR_LIMIT:
                recovered_over_gate += 1

    # 이하 집계는 게이트 통과분만 (inf 제외)
    refined_errs = [e for e in refined_errs if e != math.inf]
    assert raw_errs and refined_errs, "스윕 후보가 하나도 안 풀림 (nominal/URDF 확인)"
    # ① §6 핵심: 단발이면 게이트에 걸려 기각(false negative)됐을 후보를 refine 이
    #    살린다 — "refine 이 IK 실패를 줄인다"의 실증.
    assert recovered_over_gate >= 1, (
        f"refine 이 게이트 초과 후보를 하나도 회수 못함 "
        f"(단발 max {max(raw_errs)*1000:.1f}mm, refine max {max(refined_errs)*1000:.1f}mm)"
    )
    # ② 최대/평균 잔차가 refine 으로 줄어야 (회귀 가드 — refine 제거 시 동률 → 실패)
    assert max(refined_errs) < max(raw_errs), "refine 이 최대 잔차를 못 줄임"
    mean_raw = sum(raw_errs) / len(raw_errs)
    mean_refined = sum(refined_errs) / len(refined_errs)
    assert mean_refined < mean_raw * 0.7, (
        f"refine 평균 개선 부족: {mean_raw*1000:.1f}mm → {mean_refined*1000:.1f}mm"
    )


def test_refine_easy_solution_stays_within_limit(kin: PybulletKinematics):
    """쉬운 해(seed 근처)는 단발로 이미 임계 이내 → refine 스킵해도 정상 통과.

    비용 0 경로가 여전히 올바른 해를 반환하는지 (refine 분기가 쉬운 해를
    망가뜨리지 않는지) 확인.
    """
    seed = [0.1, 0.3, -0.4, 0.1, 0.2, 0.0]
    pos, quat = kin.fk(seed)  # seed 자체가 해 → 단발 즉시 수렴
    sol = kin._ik_from_seed(pos, quat, seed)
    assert sol is not None
    assert _pos_err(kin, sol, pos) <= IK_POS_ERROR_LIMIT
