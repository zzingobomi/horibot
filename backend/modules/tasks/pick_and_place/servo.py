"""closed-loop(look-then-move) 파지의 순수 계산 — 하드웨어/wire 0, pytest 대상.

설계 정본 = [docs/closed_loop_grasp_handoff.md] 의 사고 가이드를 이 로봇의 실측으로
채운 것. 루프 정의 (§1 — 아래가 코드 SSOT):

- **measure**: 정지 상태에서 `DETECT_ORIENTED` 1회 = 그 순간의 TCP snapshot 과 쌍인
  base-frame 관측 (`OrientedDetection` — 윗면 band centroid / base_z / height /
  grasp_yaw / footprint / points). 실패 = found=False 또는 빈 candidates (데이터).
- **command**: `MoveL` (물체 근처 직선, 자세 고정). 목표는 **관측한 그 tick 의 TCP
  를 기준으로 한 상대 오차**에서 계산 — 측정·명령이 같은 자세의 FK 오차를 공유해
  common-mode 로 상쇄된다 (eye-in-hand 의 존재 이유, handoff §3).
- **loop rate**: 이산 look-then-move. 검출(GDINO+SAM, PC)이 tick 당 ~1s 급이라
  연속 velocity servo 는 무의미하고, **이동 중 검출은 카메라 frame ↔ TCP snapshot
  쌍이 깨진다** (detector 가 검출 시점 TCP 를 따로 읽음) — 정지 측정이 계약.
- **좌표계**: 전부 base frame (`T_base_*`). 상쇄는 "그 자세에서 잰 상대 목표" 로 성립.
- **수렴**: rung(standoff 사다리) 별 lateral 오차(접근축 수직 성분) ≤ eps → 하강.
- **commit**: 최종 rung 의 마지막 관측으로 잔여 접근을 blind MoveL (그 뒤 depth
  근접 한계/그리퍼 가림으로 측정 신뢰 불가 — handoff §4).

실측 근거 (scripts/grasp_verify/closed_loop_feasibility.py, 2026-07-15 실물 세션):
- base 관측 편차는 카메라 거리에 비례 (r=0.95): 14-17cm 에서 5-12mm, 31-33cm 에서
  ~40mm → 사다리를 내려갈수록 측정이 좋아진다 = loop 성립.
- cam-frame centroid 산출 자체는 건강 (기하 거리와 최대 5mm 차).
- mask 오검출로 455mm 튄 뷰가 실데이터에 존재 → tick gate (매치 반경/도약/점수)
  는 선택이 아니라 필수.
- hand_eye: 카메라가 TCP 후방 (-77,-9,-65)mm, 광축이 TCP 를 ~5° 오차로 응시 →
  TCP-파지점 거리 s 에서 카메라-물체 거리 ≈ |(s+77, 9, 65)|mm. s=5cm 이면
  ≈14.3cm = 검증된 최적 측정 대역. **파지 직전까지 물체가 시야에 있다.**
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from modules.detector.contract import OrientedDetection

from .geometry import (
    _FIXED_JAW_CLEAR_M,
    _TCP_TO_FIXED_JAW_M,
    _TILTS_DEG,
    Quat,
    Vec3,
)

# 조 축 선택 — OBB 긴 변(grasp_yaw)에 수직(짧은 변을 무는 것)이 1순위, 긴 변
# 방향이 2순위 (짧은 쪽이 조 개구에 들어갈 확률이 높다 — 관측 footprint 기준,
# 모양 가정 아님). 폭 자체는 매 tick 점군에서 재측정 (width_along).
_JAW_YAW_OFFSETS_DEG = (90.0, 0.0)


@dataclass(frozen=True, slots=True)
class ServoConfig:
    """closed-loop 파라미터 SSOT — 전부 실물 첫 런 데이터로 튜닝하는 knob.

    standoffs: TCP→파지점 접근축 후방 거리 사다리 (m). 카메라-물체 거리로는
      각각 ≈ |(s+0.077, 0.009, 0.065)| = 17.1/14.3cm — 둘 다 실물에서 편차
      5-12mm 로 최적이었던 측정 대역 (14.1~17.4cm). 12cm 이상 standoff 는
      **실 URDF IK 에서 자세 고정 사다리가 전멸** (test_motion servo ladder
      sim — SO-101 은 높은 standoff 에서 파지 자세를 못 세움, top-down 사각
      클래스와 동일) → 2단으로 확정.
    eps_descend_m: rung 별 lateral 수렴 임계 — 이하면 다음 rung 하강 (마지막
      rung 에서는 commit). 측정 정확도의 거리비례에 맞춰 점감.
    capture_max_m: 수렴 실패/관측 소실 시 "그래도 잡힐" 상한 — 조 개구(~65mm)
      대비 2.5cm 큐브의 편측 여유 ~2cm 에서 보수적으로 절반.
    """

    standoffs: tuple[float, ...] = (0.08, 0.05)
    eps_descend_m: tuple[float, ...] = (0.010, 0.005)
    corrections_per_rung: int = 3
    capture_max_m: float = 0.012
    max_ticks: int = 15
    miss_max: int = 2  # 연속 관측 실패 허용 (드롭 1회 hold — handoff §2 empty 해석)
    match_radius_m: float = 0.05
    jump_max_m: float = 0.03  # 직전 채택 관측 대비 위치 도약 상한 (mask 오검출 gate)
    min_points: int = 50  # 점군 최소 (depth 붕괴/가림 gate)
    fuse_last_k: int = 4  # 기하(z/폭) 융합에 쓰는 최근 채택 관측 수
    grip_depth_frac: float = 0.5  # 파지 z = base_z + height·frac (clamp 아래)
    grip_depth_min_m: float = 0.006
    grip_depth_max_m: float = 0.018
    close_attempts: int = 2  # close 후 EMPTY 재시도 상한 (재관측부터)
    withdraw_standoff_m: float = 0.08  # 파지 후 접근축 역방향 후퇴 거리
    settle_s: float = 0.4  # 이동 후 카메라 정착 (검출 품질)


@dataclass(frozen=True, slots=True)
class GraspFamily:
    """servo 내내 고정되는 파지 자세 가족 — 자세를 고정해야 common-mode 상쇄와
    카메라 시점 일관성이 유지된다 (rung 간 이동은 접근축 직선 + lateral 보정만)."""

    label: str
    quat: Quat
    approach: Vec3  # 단위 벡터 (base) — 진입 방향
    jaw_axis: Vec3  # 단위 벡터 (base) — 조 이동 축
    tilt_deg: int


@dataclass(slots=True)
class GateResult:
    obs: OrientedDetection | None
    reason: str  # 채택 시 "", 기각 시 사람이 읽을 사유 (trace 에 그대로)


@dataclass(slots=True)
class TickDecision:
    """tick 하나의 판정 결과 — steps 의 servo 루프가 이걸 보고 모션을 명령한다."""

    action: str  # "correct" | "descend" | "commit" | "hold" | "abort"
    reason: str
    lateral_m: float = 0.0


def grasp_families(obs: OrientedDetection) -> list[GraspFamily]:
    """coarse 관측 → 파지 자세 후보 가족 (선호순) — 도달 판정은 motion resolve 몫.

    조 축 = OBB yaw 기준 2방향(짧은 변 우선) × flip(단일 가동 조 lateral 방향)
    × tilt 사다리(수직부터 — geometry._TILTS_DEG 재사용). 회전 구성은 open-loop
    plan_grasp 와 동일 규약 (tool x=접근, y=조 축, z=x×y).
    """
    down = np.array([0.0, 0.0, -1.0])
    out: list[GraspFamily] = []
    for tilt_deg in _TILTS_DEG:
        for yaw_off_deg in _JAW_YAW_OFFSETS_DEG:
            jaw_yaw = obs.grasp_yaw + math.radians(yaw_off_deg)
            for flip in (1.0, -1.0):
                y = np.array(
                    [math.cos(jaw_yaw), math.sin(jaw_yaw), 0.0]
                ) * flip
                approach = Rotation.from_rotvec(
                    y * math.radians(tilt_deg)
                ).apply(down)
                rot_m = np.column_stack([approach, y, np.cross(approach, y)])
                qx, qy, qz, qw = (
                    float(v) for v in Rotation.from_matrix(rot_m).as_quat()
                )
                out.append(
                    GraspFamily(
                        label=(
                            f"jaw{'∥short' if yaw_off_deg else '∥long'} "
                            f"tilt={tilt_deg:+d} flip={'+' if flip > 0 else '-'}"
                        ),
                        quat=(qx, qy, qz, qw),
                        approach=(
                            float(approach[0]), float(approach[1]),
                            float(approach[2]),
                        ),
                        jaw_axis=(float(y[0]), float(y[1]), float(y[2])),
                        tilt_deg=tilt_deg,
                    )
                )
    return out


def width_along(
    points: list[Vec3] | None, axis: Vec3, fallback_m: float
) -> float:
    """점군의 조 축 방향 폭 (5–95 percentile) — lateral 오프셋의 관측 근거.

    점군 없으면 fallback (coarse footprint). 모양 가정 없음 — 관측 그대로.
    """
    if not points or len(points) < 8:
        return fallback_m
    p = np.asarray(points, dtype=float)
    proj = p @ np.asarray(axis, dtype=float)
    return float(np.percentile(proj, 95) - np.percentile(proj, 5))


def lateral_offset(width_m: float) -> float:
    """단일 가동 조 보정 — 고정 조 안쪽 면이 [폭/2 + 여유] 에 오는 조 축 횡이동.
    open-loop 파지와 동일 규약 (geometry.plan_grasp)."""
    return width_m / 2.0 + _FIXED_JAW_CLEAR_M - _TCP_TO_FIXED_JAW_M


def grasp_point(
    latest: OrientedDetection, fused: OrientedDetection, cfg: ServoConfig
) -> Vec3:
    """파지 지점 (물체 좌표, TCP 아님) — XY 는 **최신 관측** (common-mode 상쇄는
    최신 자세의 측정에만 성립), z 는 융합 기하 (base_z/height 는 여러 close 관측이
    안정적 — 단일 뷰 height 과소 보완).

    z = base_z + clamp(height·frac) 을 [바닥+4mm, 윗면−2mm] 로 재clamp — height
    과소/과대가 파지 깊이를 바닥 충돌이나 헛집기로 밀지 않게.
    """
    depth = min(
        max(fused.height * cfg.grip_depth_frac, cfg.grip_depth_min_m),
        cfg.grip_depth_max_m,
    )
    z = fused.base_z + depth
    z = max(z, fused.base_z + 0.004)
    z = min(z, latest.position[2] - 0.002)
    return (latest.position[0], latest.position[1], z)


def grasp_tcp(point: Vec3, family: GraspFamily, lateral_m: float) -> Vec3:
    """파지 지점 → TCP 목표 (조 축 lateral 오프셋 적용 — plan_grasp 동일 규약)."""
    rot = Rotation.from_quat(family.quat)
    off = rot.apply([0.0, lateral_m, 0.0])
    return (
        float(point[0] + off[0]),
        float(point[1] + off[1]),
        float(point[2] + off[2]),
    )


def standoff(tcp: Vec3, family: GraspFamily, s_m: float) -> Vec3:
    """파지 TCP 에서 접근축 후방 s_m 의 standoff TCP."""
    a = family.approach
    return (tcp[0] - a[0] * s_m, tcp[1] - a[1] * s_m, tcp[2] - a[2] * s_m)


def split_error(delta: Vec3, family: GraspFamily) -> tuple[float, float]:
    """오차 벡터 → (lateral 크기, axial 성분) — 접근축 기준 분해.

    lateral 이 수렴 판정 대상 (axial 은 rung 하강이 자연 흡수).
    """
    d = np.asarray(delta, dtype=float)
    a = np.asarray(family.approach, dtype=float)
    axial = float(d @ a)
    lat = d - axial * a
    return float(np.linalg.norm(lat)), axial


def gate_observation(
    candidates: list[OrientedDetection],
    expected_xy: Vec3,
    last_accepted: OrientedDetection | None,
    cfg: ServoConfig,
) -> GateResult:
    """tick 관측 게이트 — 실데이터의 실패 클래스가 근거 (docstring 상단):

    ① 매치: 기대 위치 XY 반경 안 최근접 후보 (prompt 가 다른 물체를 잡는 것 차단)
    ② 도약: 직전 채택 대비 jump_max 초과 = mask 오검출 의심 (실데이터 455mm 사례)
    ③ 점군: min_points 미만 = depth 붕괴/가림 (근접 한계 신호)
    기각 사유는 문자열로 — trace/로그에 그대로 남아 사후분석 가능.
    """
    if not candidates:
        return GateResult(None, "검출 0건")
    best: OrientedDetection | None = None
    best_d = cfg.match_radius_m
    for c in candidates:
        d = math.hypot(
            c.position[0] - expected_xy[0], c.position[1] - expected_xy[1]
        )
        if d <= best_d:
            best, best_d = c, d
    if best is None:
        near = min(
            math.hypot(
                c.position[0] - expected_xy[0], c.position[1] - expected_xy[1]
            )
            for c in candidates
        )
        return GateResult(
            None,
            f"매치 실패 — 기대 위치 반경 {cfg.match_radius_m * 1000:.0f}mm 밖 "
            f"(최근접 {near * 1000:.0f}mm)",
        )
    if last_accepted is not None:
        jump = math.dist(best.position, last_accepted.position)
        if jump > cfg.jump_max_m:
            return GateResult(
                None,
                f"위치 도약 {jump * 1000:.0f}mm > {cfg.jump_max_m * 1000:.0f}mm "
                "(mask 오검출 의심)",
            )
    n = len(best.points or [])
    if n < cfg.min_points:
        return GateResult(
            None, f"점군 부족 {n} < {cfg.min_points} (depth 붕괴/가림 의심)"
        )
    return GateResult(best, "")


@dataclass(slots=True)
class ServoState:
    """rung/보정/miss 카운터 — tick 판정(decide_tick)의 입력이자 출력.

    steps 의 루프가 소유하고 decide_tick 이 갱신한다 (순수 함수 유지를 위해
    상태 전이도 여기 명시적 — 숨은 전역 없음).
    """

    rung: int = 0
    corrections: int = 0
    misses: int = 0
    ticks: int = 0
    last_lateral_m: float = math.inf
    error_history_mm: list[float] = field(default_factory=list)


def decide_tick(
    state: ServoState,
    gate: GateResult,
    lateral_m: float | None,
    cfg: ServoConfig,
) -> TickDecision:
    """tick 판정 — 게이트 결과 + lateral 오차로 다음 행동을 정한다 (모션 명령은
    호출부 steps 몫 — 여기는 순수 상태 전이).

    handoff §2 표의 구현 (각 행 → 분기, "감지 못 하면 크래시/무한대기" 금지):
    - 관측 실패 단발 → hold (모션 0, 재측정)
    - 관측 실패 연속(miss_max): 이미 가까이(rung≥1) + 직전 오차가 capture 안
      → 직전 관측으로 commit (blind) / 아니면 abort (사유 포함)
    - 수렴 안 함 (rung 당 보정 상한): capture 안이면 하강 강행 (더 가까운 측정이
      더 정확 — 실측 r=0.95), 밖이면 abort (발진/오차 정체)
    - tick 상한 → abort (전체 timeout — "언젠가 수렴한다" 가정 금지)

    lateral_m: 이번 tick 의 lateral 오차 (게이트 기각이면 None). 호출부가
    "관측한 그 tick 의 TCP" 기준으로 분해해 넘긴다 (common-mode 상쇄 성립점).
    """
    state.ticks += 1
    if state.ticks > cfg.max_ticks:
        return TickDecision(
            action="abort",
            reason=f"tick 상한 {cfg.max_ticks} 초과 — 수렴 실패 "
            f"(오차 이력 mm: {[round(e, 1) for e in state.error_history_mm]})",
        )

    if gate.obs is None or lateral_m is None:
        state.misses += 1
        if state.misses < cfg.miss_max:
            return TickDecision(
                action="hold", reason=f"관측 기각 ({gate.reason}) — 재측정"
            )
        if state.rung >= 1 and state.last_lateral_m <= cfg.capture_max_m:
            return TickDecision(
                action="commit",
                reason=f"관측 연속 소실 ({gate.reason}) — 직전 수렴 관측"
                f"(lateral {state.last_lateral_m * 1000:.1f}mm)으로 commit",
            )
        return TickDecision(
            action="abort",
            reason=f"관측 연속 {state.misses}회 소실 (rung {state.rung}, "
            f"마지막 사유: {gate.reason}) — 물체 위치/가림 확인 후 다시 실행하세요",
        )

    state.misses = 0
    state.last_lateral_m = lateral_m
    state.error_history_mm.append(lateral_m * 1000.0)
    last_rung = len(cfg.standoffs) - 1

    if lateral_m <= cfg.eps_descend_m[state.rung]:
        if state.rung == last_rung:
            return TickDecision(
                action="commit",
                reason=f"수렴 (lateral {lateral_m * 1000:.1f}mm ≤ "
                f"{cfg.eps_descend_m[state.rung] * 1000:.1f}mm, 최종 rung)",
                lateral_m=lateral_m,
            )
        state.rung += 1
        state.corrections = 0
        return TickDecision(
            action="descend",
            reason=f"수렴 → rung {state.rung} (standoff "
            f"{cfg.standoffs[state.rung] * 1000:.0f}mm) 하강",
            lateral_m=lateral_m,
        )

    state.corrections += 1
    if state.corrections > cfg.corrections_per_rung:
        if lateral_m <= cfg.capture_max_m:
            if state.rung == last_rung:
                return TickDecision(
                    action="commit",
                    reason=f"보정 상한 — capture 여유 안 "
                    f"(lateral {lateral_m * 1000:.1f}mm ≤ "
                    f"{cfg.capture_max_m * 1000:.0f}mm), 최종 rung 에서 commit",
                    lateral_m=lateral_m,
                )
            state.rung += 1
            state.corrections = 0
            return TickDecision(
                action="descend",
                reason=f"보정 상한 — capture 여유 안, 더 가까운 측정으로 하강 "
                f"(lateral {lateral_m * 1000:.1f}mm)",
                lateral_m=lateral_m,
            )
        return TickDecision(
            action="abort",
            reason=f"rung {state.rung} 수렴 실패 — 보정 "
            f"{cfg.corrections_per_rung}회 후에도 lateral "
            f"{lateral_m * 1000:.1f}mm > capture "
            f"{cfg.capture_max_m * 1000:.0f}mm "
            f"(오차 이력 mm: {[round(e, 1) for e in state.error_history_mm]}) — "
            "발진/오차 정체. 캘/물체 상태 확인 후 다시 실행하세요",
        )
    return TickDecision(
        action="correct",
        reason=f"lateral {lateral_m * 1000:.1f}mm > "
        f"{cfg.eps_descend_m[state.rung] * 1000:.1f}mm — rung 유지 보정 "
        f"({state.corrections}/{cfg.corrections_per_rung})",
        lateral_m=lateral_m,
    )
