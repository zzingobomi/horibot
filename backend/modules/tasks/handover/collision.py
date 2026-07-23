"""cross-robot 충돌 체커 — 두 URDF 를 한 PyBullet 세계에 놓고 링크 간 근접 판정.

handover task 전용의 **독립 유틸** (2026-07-17 신설, 실물 미검증). motion 모듈을
침범하지 않는다 — 각 robot 의 motion 모듈은 분산 host 에 자기 robot 만 알고,
cross-robot 게이트를 그 안에 넣으려면 peer 상태 구독 배선이 필요해 지금은
과하다. 정석 통합 자리는 motion resolve_reachable 의 ③b(장애물) 옆
(contract.py 가 예약해 둔 슬롯) — 아래 TODO 와 motion/module.py 주석 참조.

2026-07-23 정밀화 (omx handover 재설계 — 근접 그리퍼-대-그리퍼 핸드오프):
  ① **그리퍼 상태 반영** — 옛 "벌림(상한) 고정" 은 handover 국면(펜 든 omx 는
     거의 닫힘)에 과보수라 핸드오프 자체를 기각한다. grip_a/grip_b (0=닫힘,
     1=벌림) 로 실제 개구를 준다. 기본 1.0 = 옛 보수 의미 유지.
  ② **mimic 반영** — omx 미러 손가락(gripper_joint_2, multiplier=-1)을 URDF
     mimic 태그로 따른다. 옛 "전부 상한" 은 미러 관절에선 손가락 교차(비물리
     자세)였다 — 보수도 정밀도 아닌 그냥 틀린 형상.
  ③ **판정별 margin override** — 핸드오프 접근은 기본 2cm 로는 성립 불가
     (두 그리퍼가 같은 펜을 cm 간격으로 문다). 호출자가 국면별로 준다.

TODO(cross-robot): 실물 검증 후 —
  ① motion resolve_reachable 에 peer robot 점유(joints+base_pose) 게이트 추가
  ② 이 체커는 그 게이트의 로컬 프로토타입으로 흡수/폐기
  ③ margin 은 실물 특성화로 튜닝 (기본 2cm 는 크로스캘 σ_t ~8mm + FK 오차의
     보수 기본값 — 추측이므로 실측 전 신뢰 금지)

판정 의미론: 두 로봇의 링크 표면 간 최소 거리가 margin 미만이면 "충돌 위험".
"""

from __future__ import annotations

import logging
import math
import threading
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pybullet as p

logger = logging.getLogger(__name__)

# 링크 간 안전 여유 — 미만이면 충돌 위험 판정 (모듈 docstring TODO: 실측 튜닝 전
# 보수 기본값. 크로스캘 σ_t ~8mm + 양쪽 FK/backlash ~1cm 급 근거).
_DEFAULT_MARGIN_M = 0.02
# 경로 표본 간격 — 관절 최대 이동 기준 (충돌 검사만이라 촘촘해도 싸다).
_PATH_STEP_RAD = math.radians(6.0)


@dataclass(frozen=True)
class BasePose:
    """robot base 의 world(=so101 base) frame 자세 — robots.yaml base_pose 투영."""

    x: float
    y: float
    z: float
    yaw_rad: float


def _parse_mimic(urdf_path: str) -> dict[str, tuple[str, float, float]]:
    """URDF mimic 태그 → follower joint name → (leader, multiplier, offset).

    PyBullet 은 mimic 을 로드하지 않는다 (constraint 를 손으로 걸어야 함) —
    체커는 정적 자세 배치만 하므로 값 계산만 따라가면 충분하다.
    """
    out: dict[str, tuple[str, float, float]] = {}
    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError:  # 손상 URDF 는 로드 단계에서 이미 터진다 — 방어만
        return out
    for joint in root.iter("joint"):
        mim = joint.find("mimic")
        name = joint.get("name")
        if mim is None or name is None:
            continue
        leader = mim.get("joint")
        if leader is None:
            continue
        out[name] = (
            leader,
            float(mim.get("multiplier", "1")),
            float(mim.get("offset", "0")),
        )
    return out


@dataclass
class _Body:
    """로드된 robot 하나의 관절 배치 정보."""

    body: int
    movable: list[int]
    lower: list[float]
    upper: list[float]
    names: list[str]
    mimic: dict[str, tuple[str, float, float]]


class CrossRobotChecker:
    """두 robot(a=world 원점, b=base_pose) 의 구성 쌍 충돌 판정.

    lazy init — 생성은 값싸고 첫 판정 때 PyBullet(DIRECT) 로드. thread-safe
    (판정은 짧은 lock 구간). joints 배열은 각 robot 의 **팔 관절**(TcpState.joints
    순서 = URDF movable 선두 — PybulletKinematics 체인 규약과 동일 가정) 이며,
    나머지 movable(그리퍼)은 grip_a/grip_b(0=닫힘..1=벌림, 기본 1.0=보수) 로
    놓고 mimic follower 는 leader 를 따른다 (모듈 docstring 정밀화 ①②).
    """

    def __init__(
        self,
        urdf_a: Path | str,
        urdf_b: Path | str,
        base_b: BasePose,
        margin_m: float = _DEFAULT_MARGIN_M,
    ) -> None:
        self._urdf_a = str(urdf_a)
        self._urdf_b = str(urdf_b)
        self._base_b = base_b
        self.margin_m = margin_m
        self._client: int | None = None
        self._a: _Body | None = None
        self._b: _Body | None = None
        self._lock = threading.Lock()

    # ─── lifecycle ────────────────────────────────────────────────

    def _ensure_init(self) -> None:
        if self._client is not None:
            return
        self._client = p.connect(p.DIRECT)
        body_a = p.loadURDF(
            self._urdf_a, useFixedBase=True, physicsClientId=self._client
        )
        qz = math.sin(self._base_b.yaw_rad / 2.0)
        qw = math.cos(self._base_b.yaw_rad / 2.0)
        body_b = p.loadURDF(
            self._urdf_b,
            basePosition=(self._base_b.x, self._base_b.y, self._base_b.z),
            baseOrientation=(0.0, 0.0, qz, qw),
            useFixedBase=True,
            physicsClientId=self._client,
        )
        self._a = self._load_body(body_a, self._urdf_a)
        self._b = self._load_body(body_b, self._urdf_b)
        logger.info(
            "CrossRobotChecker init — a=%s b=%s base_b=(%.3f,%.3f,%.3f,%.1f°) "
            "margin=%.0fmm mimic(a=%d,b=%d)",
            Path(self._urdf_a).name, Path(self._urdf_b).name,
            self._base_b.x, self._base_b.y, self._base_b.z,
            math.degrees(self._base_b.yaw_rad), self.margin_m * 1000,
            len(self._a.mimic), len(self._b.mimic),
        )

    def _load_body(self, body: int, urdf_path: str) -> _Body:
        assert self._client is not None
        idx, lower, upper, names = [], [], [], []
        for j in range(p.getNumJoints(body, physicsClientId=self._client)):
            info = p.getJointInfo(body, j, physicsClientId=self._client)
            if info[2] != p.JOINT_FIXED:
                idx.append(j)
                lower.append(float(info[8]))
                upper.append(float(info[9]))
                names.append(info[1].decode())
        return _Body(
            body=body, movable=idx, lower=lower, upper=upper, names=names,
            mimic=_parse_mimic(urdf_path),
        )

    def close(self) -> None:
        with self._lock:
            if self._client is not None:
                p.disconnect(physicsClientId=self._client)
                self._client = None

    # ─── 판정 ─────────────────────────────────────────────────────

    def _set_config(self, b: _Body, joints: list[float], grip_frac: float) -> None:
        """팔 관절 = joints 그대로, 잔여 movable(그리퍼) = 개구 fraction, mimic
        follower = leader 추종. 2-pass — follower 의 leader 값이 먼저 정해져야."""
        assert self._client is not None
        vals: dict[str, float] = {}
        for k, name in enumerate(b.names):
            if k < len(joints):
                vals[name] = float(joints[k])  # 팔 관절 (TcpState.joints 순서)
        for k, name in enumerate(b.names):  # 1-pass: 비-mimic 그리퍼 leader
            if name not in vals and name not in b.mimic:
                vals[name] = b.lower[k] + grip_frac * (b.upper[k] - b.lower[k])
        for name, (leader, mult, off) in b.mimic.items():  # 2-pass: follower
            if leader in vals:
                vals[name] = mult * vals[leader] + off
        for k, j in enumerate(b.movable):
            p.resetJointState(
                b.body, j, vals[b.names[k]], physicsClientId=self._client
            )

    def in_collision(
        self,
        joints_a: list[float],
        joints_b: list[float],
        *,
        grip_a: float = 1.0,
        grip_b: float = 1.0,
        margin_m: float | None = None,
    ) -> bool:
        """구성 쌍의 링크 간 최소 거리 < margin 이면 True (충돌 위험).

        grip_* = 그리퍼 개구 fraction (0=닫힘, 1=벌림 — 기본 1.0 = 최대 점유
        보수). margin_m = 이 판정만의 여유 override (핸드오프 근접 국면 등).
        """
        margin = self.margin_m if margin_m is None else margin_m
        with self._lock:
            self._ensure_init()
            assert self._a is not None and self._b is not None
            self._set_config(self._a, joints_a, grip_a)
            self._set_config(self._b, joints_b, grip_b)
            pts = p.getClosestPoints(
                bodyA=self._a.body,
                bodyB=self._b.body,
                distance=margin,
                physicsClientId=self._client,
            )
            return len(pts) > 0

    def path_in_collision(
        self,
        path_a: list[list[float]],
        joints_b: list[float],
        *,
        grip_a: float = 1.0,
        grip_b: float = 1.0,
        margin_m: float | None = None,
    ) -> bool:
        """a 의 관절 경로(waypoint 열, 사이 lerp 표본) vs b 고정 구성.

        실행부(MoveJ/MoveL)의 실제 궤적과 동일하진 않다 — 관절 보간 근사
        (MoveJ 등가). MoveL 구간은 호출부가 IK 해 열을 path_a 로 넘길 것.
        """
        if not path_a:
            return False
        prev = path_a[0]
        if self.in_collision(
            prev, joints_b, grip_a=grip_a, grip_b=grip_b, margin_m=margin_m
        ):
            return True
        for nxt in path_a[1:]:
            qa, qb = np.asarray(prev, float), np.asarray(nxt, float)
            n = max(1, int(math.ceil(float(np.max(np.abs(qb - qa))) / _PATH_STEP_RAD)))
            for k in range(1, n + 1):
                q = [float(v) for v in qa + (qb - qa) * (k / n)]
                if self.in_collision(
                    q, joints_b, grip_a=grip_a, grip_b=grip_b, margin_m=margin_m
                ):
                    return True
            prev = nxt
        return False
