"""산업 arm sim 전용 pybullet FK/IK — 완전 자립.

집 motion/kinematics 스택을 import 하지 않는다. DIRECT 모드 pybullet 클라이언트 1개로
URDF 를 로드해 순수 기구학(FK/IK)만 제공 — 실물/틱/캘리브레이션 개념 없음. URDF 만
주면 어떤 6축 산업 arm 도 그대로 동작(범용 수치 IK). URDF 는 `tcp` 이름 link 필수
(프로젝트 규약).

IK 는 **해를 항상 반환**하고 위치오차 판정은 호출자에게 맡긴다 — MoveL 직선 추종에서
수치 IK 가 목표점을 얼마나 못 맞추는지(=경로 휨)를 트레이스로 보이게 하려면 여기서
잘라내면 안 되기 때문. 산업 arm 은 구형 손목이라 seed 단발 IK 로 대개 깨끗이 수렴 —
집 비산업 로봇처럼 random restart / continuous walk 로 헤맬 필요가 없다.
"""

from __future__ import annotations

import math
from pathlib import Path

import pybullet as p

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

_TCP_LINK = "tcp"
# 결과-seed 재해(refine) — 수치 IK 는 위치+자세를 함께 최소화해 자세가 빡센 지점에서
# 위치를 수 mm 내주고 수렴한다(so101 MoveL 꺾임의 근본). 그 해를 다시 seed 로 넣어
# 재호출하면 더 나은 basin 에서 출발해 sub-mm 로 조여진다. **결정적·단일 branch**
# (so101 의 random restart / continuous walk 카오스와 다름 — 산업 arm 은 1~2회면 수렴).
_IK_REFINE_ITERS = 4
_IK_REFINE_TOL_M = 2e-4  # 0.2mm — 위치 refine 중단 (쉬운 해는 1회로 끝 = 싸다)
_IK_REFINE_TOL_RAD = math.radians(0.05)  # 0.05° — 자세 refine 중단


def quat_angle_rad(a: Quat, b: Quat) -> float:
    """두 quaternion 사이 geodesic 각도(rad). q, -q 동일 회전이라 |dot| 사용."""
    dot = min(1.0, abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]))
    return 2.0 * math.acos(dot)


class ArmKinematics:
    """URDF → pybullet FK/IK. 인스턴스당 DIRECT 클라이언트 1개 (physicsClientId 격리)."""

    def __init__(self, urdf_path: Path, joint_names: list[str]) -> None:
        self._cid = p.connect(p.DIRECT)
        self._rid = p.loadURDF(
            str(urdf_path), useFixedBase=True, physicsClientId=self._cid
        )
        name2idx: dict[str, int] = {}
        self._tcp_idx: int | None = None
        for j in range(p.getNumJoints(self._rid, physicsClientId=self._cid)):
            info = p.getJointInfo(self._rid, j, physicsClientId=self._cid)
            name2idx[info[1].decode()] = j
            if info[12].decode() == _TCP_LINK:
                self._tcp_idx = j
        if self._tcp_idx is None:
            raise ValueError(f"URDF 에 '{_TCP_LINK}' link 없음: {urdf_path}")
        try:
            self._arm = [name2idx[n] for n in joint_names]
        except KeyError as e:  # joint 이름 오타/불일치 = 부팅 fail-fast
            raise ValueError(f"URDF 에 joint {e} 없음: {urdf_path}") from e
        self._lo = [
            p.getJointInfo(self._rid, i, physicsClientId=self._cid)[8]
            for i in self._arm
        ]
        self._hi = [
            p.getJointInfo(self._rid, i, physicsClientId=self._cid)[9]
            for i in self._arm
        ]
        self._ranges = [h - lo for lo, h in zip(self._lo, self._hi)]

    @property
    def dof(self) -> int:
        return len(self._arm)

    def _set(self, q: list[float]) -> None:
        for idx, v in zip(self._arm, q):
            p.resetJointState(self._rid, idx, v, physicsClientId=self._cid)

    def fk(self, q: list[float]) -> tuple[Vec3, Quat]:
        self._set(q)
        ls = p.getLinkState(
            self._rid,
            self._tcp_idx,
            computeForwardKinematics=True,
            physicsClientId=self._cid,
        )
        pos, quat = ls[4], ls[5]
        return (
            (float(pos[0]), float(pos[1]), float(pos[2])),
            (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
        )

    def _solve_once(
        self, pos: Vec3, quat: Quat | None, seed: list[float]
    ) -> list[float]:
        """단발 수치 IK (seed 상태에서 출발). 해를 항상 반환."""
        self._set(seed)
        kw = dict(
            lowerLimits=self._lo,
            upperLimits=self._hi,
            jointRanges=self._ranges,
            restPoses=list(seed),
            maxNumIterations=200,
            residualThreshold=1e-5,
            physicsClientId=self._cid,
        )
        if quat is not None:
            sol = p.calculateInverseKinematics(
                self._rid, self._tcp_idx, list(pos), list(quat), **kw
            )
        else:
            sol = p.calculateInverseKinematics(
                self._rid, self._tcp_idx, list(pos), **kw
            )
        return [float(v) for v in sol[: len(self._arm)]]

    def ik(self, pos: Vec3, quat: Quat | None, seed: list[float]) -> list[float]:
        """수치 IK + 결정적 refine. 해를 항상 반환(도달성 판정은 호출자).

        결과를 seed 로 재해해 위치오차를 sub-mm 로 조인다 (쉬운 해는 1회로 종료 =
        비용 0). seed 연쇄로 경로 추종 시 해 연속성(구성 플립 억제). quat=None 이면
        position-only."""
        sol = self._solve_once(pos, quat, seed)
        for _ in range(_IK_REFINE_ITERS):
            r_pos, r_quat = self.fk(sol)
            pos_ok = math.dist(r_pos, pos) <= _IK_REFINE_TOL_M
            ori_ok = quat is None or quat_angle_rad(r_quat, quat) <= _IK_REFINE_TOL_RAD
            if pos_ok and ori_ok:
                break
            sol = self._solve_once(pos, quat, sol)
        return sol

    def close(self) -> None:
        try:
            p.disconnect(physicsClientId=self._cid)
        except Exception:  # noqa: BLE001 — 종료 정리 best-effort
            pass
