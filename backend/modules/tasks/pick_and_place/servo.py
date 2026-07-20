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
from collections.abc import Sequence
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

# 조 축 yaw 후보 (§11 대수술, 2026-07-20) — **절대 격자 + 면정렬 우선 정렬**.
#
# 폐지된 옛 설계 (OBB 앵커 + aspect 문턱 + 2단 확장)의 사고 사슬:
#   ① yaw 를 노이즈 낀 OBB grasp_yaw 에 묶음 (near-square 는 뷰마다 랜덤 —
#      07-17 같은 큐브 두 뷰가 yaw 84° 차로 전멸/채택 갈림)
#   ② 탈출구(yaw_free 확장)를 또 노이즈 낀 스칼라(aspect 1.25 문턱)로 gate —
#      07-20 둥근 큐브가 관측 노이즈로 aspect 1.397 → 확장 침묵 미실행 → 전멸
#      (해 있는 확장 3가족을 시도조차 안 함 — ik_yaw_free_audit 실측)
# → 이산화 복권의 클래스 자체를 제거: yaw 는 절대 0..180° 격자 전체가 항상
#   후보 (물리 필터는 "그 yaw 방향 관측 폭 ≤ 개구" — plan 의 width 게이트),
#   순서만 물체 면 정렬(grasp_yaw mod 90) 근접순. 해석적 IK(수 ms/가족)라
#   후보 수 증가는 전멸 CT 에만 선형 (~수 s), 채택 경로는 선호순 조기 종료.
_YAW_GRID_DEG = 15.0  # 도달 yaw 밴드 실측 30~40° 폭 → 밴드당 2~3 샘플


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
    # 낙하/밀림 재시도 첫 재획득의 매치 반경 — 물체가 움직인 걸 아는 국면이라
    # 넓힌다 (실물: 낙하 큐브 10cm 굴러감 → 5cm 반경이 재획득 거부 → abort).
    reacquire_radius_m: float = 0.15
    jump_max_m: float = 0.03  # 직전 채택 관측 대비 위치 도약 상한 (mask 오검출 gate)
    # 직전 채택 대비 윗면 z 도약 상한 — top-앵커 파지 z 의 안전 gate. 정지 물체의
    # 윗면이 tick 사이 cm 급으로 움직이는 건 물리 불가 = 조 끝/가림 depth 오염
    # (2026-07-17 실물: top +2cm 점프 → 파지 목표 허공 → 이동 IK 거부로 중단).
    z_jump_max_m: float = 0.01
    # 점군 최소 (depth 붕괴/가림 gate). 50→30 (2026-07-17 저녁 실물): detector
    # body_points 소스 청소 후엔 **진짜 물체 점만** 세어진다 — raw 시절 문턱
    # 50 이 건강한 2cm 큐브 top-view(원거리 r≈0.32, 청소 후 49점)를 연속
    # 기각해 소실 중단시킨 실사고. 진짜 붕괴/가림은 수~십수 점 급이라 30 도
    # 잡는다 (기존 회귀 테스트 = 10점 기각 유지).
    min_points: int = 30
    # tick 관측 score 하한 — plan(_PICK_SCORE_MIN)과 동일 근거의 servo 판.
    # 열화 관측(부분 뷰)이 **첫 앵커**가 되면 이후 정상 관측이 z 도약으로
    # 연속 기각된다 (2026-07-17 13:53 실물: tick1 score 0.43·top z 16mm 낮은
    # 관측이 앵커 → 정상 0.83 관측 2연속 기각 → 소실 중단).
    min_score: float = 0.45
    fuse_last_k: int = 4  # 기하(z/폭) 융합에 쓰는 최근 채택 관측 수
    # 파지 z = 윗면(top band centroid z) − grip_below_top_m — **윗면 앵커**.
    # base_z 앵커 폐기 (2026-07-16 실물: 단일 top-view 의 base_z 는 실제 바닥이
    # 아니라 보이는 band 하단 ≈ 윗면 — 25mm 큐브에서 +25mm 계통 오차 → 파지 z 가
    # 윗면 −2mm nip → close 가 물체를 튕겨냄 2연속). 윗면은 카메라가 매 tick
    # 직접 보는 유일한 면이라 유일하게 믿을 수 있는 z 앵커.
    grip_below_top_m: float = 0.010
    # 융합 height 신뢰 문턱 — 옆면을 본 뷰가 섞여야 height 가 실측 (이상이면
    # "바닥 위 4mm" 하한 guard 활성). 미만이면 height 는 band 두께일 뿐.
    height_credible_m: float = 0.015
    # 바닥 위 파지 z 하한 여유 — grip_below_top 이 물체 높이보다 깊으면 (납작한
    # 물체) 파지 z 가 테이블을 뚫는다. height 가 신뢰 밖일 때도 바닥(floor_z,
    # plan 의 클러스터 min base_z)은 항상 안다 → z ≥ floor + 이 여유로 clamp.
    floor_clear_m: float = 0.004
    close_attempts: int = 2  # close 후 EMPTY 재시도 상한 (재관측부터)
    withdraw_standoff_m: float = 0.08  # 파지 후 접근축 역방향 후퇴 거리
    settle_s: float = 0.4  # 이동 후 카메라 정착 (검출 품질)
    # 접촉 인접 이동(commit blind/touch-up/withdraw) 속도 배율 — 전역 상한
    # 10cm/s 그대로는 접촉 직전 관성/진동이 크다 (2026-07-17: close 판정 통과
    # 후 withdraw 중 흘림). 0.25 → ~2.5cm/s (산업 최종접근 관례 1~2cm/s 대).
    gentle_speed_scale: float = 0.25
    # 파지 TCP 접근축 추가 전진 — 물체를 조 **끝이 아니라 안쪽**에 물리기.
    # 2026-07-17 영상(test2.mp4): 조 끝 점접촉 파지가 정적으로는 버티는데(판정
    # HELD) 감속 withdraw 시작 직후 무게중심 토크로 회전하며 이탈(cam-out).
    # 접촉선을 CoM 에 가깝게 = 조 면으로 깊이 무는 게 물리 처방 (크기 무관).
    engage_m: float = 0.006
    # withdraw 후 gap 유지율 하한 — close 시점 gap 대비 (절대 아님 = 물체 크기
    # 무관). 미만 = 물체가 조 안에서 끝쪽으로 미끄러짐 (2026-07-17 실물: close
    # gap 210 → withdraw 후 36 = 17% — 매달려는 있지만 이송 불가 파지) →
    # 내려놓고 재시도. 공중 open 금지 (튀어 도망감).
    slip_retention: float = 0.6
    # ── commit 2단 하강 (2026-07-17 스틱션 release 스침 대응) ─────────────
    # 42런 comp z 잔차 분석: sag 모델(active) 통과 후에도 같은 자리 인접 tick 간
    # ±5~13mm 널뜀(부호 반전 −5.7 포함) = 불연속 stick-slip/유격 — comp 가
    # stall 국면에서 배운 +z 선보상이 blind 하강 중 release 되면 과보상으로
    # 조 끝이 바닥을 긁는다. 대응 = 물체 위 midstop 에서 하강방향 재안착 후
    # **그 순간의 FK 실측 잔차**로 마지막 구간을 재앵커 (stale comp 대체).
    commit_midstop_m: float = 0.020  # 접근축 후방 중간 정지 거리 (0 = 기능 off)
    # midstop 재안착 왕복 폭 — 후방 +dither 갔다가 되-내려와 기어열을 하강쪽
    # 플랭크에 앉힌다 (unidirectional approach 관례). 0 = 왕복 생략.
    commit_dither_m: float = 0.003
    commit_settle_s: float = 0.15  # 재안착 후 FK 안정 대기
    commit_residual_max_m: float = 0.02  # 재앵커 잔차 clamp (오염 실측 방어)
    # 하강 프로파일 샘플링 (진단 전용 — 제어 무관): 하강 이동 중 20Hz 로
    # FK z + 관절 load 를 trace 에 남긴다 → release 가 **언제** 났는지 /
    # 바닥 접촉 load 지문이 데이터로 남는다. ≤0 = off.
    descent_sample_hz: float = 20.0
    # 바닥 접촉 의심 arm load 문턱 (진단 플래그 전용) — 실물 특성화 런으로
    # 튜닝할 것 (초기값은 자유이동 load << HELD 부하 ~300 사이 추정치).
    descent_load_suspect_raw: int = 150


@dataclass(frozen=True, slots=True)
class GraspFamily:
    """servo 내내 고정되는 파지 자세 가족 — 자세를 고정해야 common-mode 상쇄와
    카메라 시점 일관성이 유지된다 (rung 간 이동은 접근축 직선 + lateral 보정만)."""

    label: str
    quat: Quat
    approach: Vec3  # 단위 벡터 (base) — 진입 방향
    jaw_axis: Vec3  # 단위 벡터 (base) — 조 이동 축
    tilt_deg: int
    flip: float = 1.0  # 단일 가동 조 lateral 방향 (±1) — refit 변형 매칭 키


@dataclass(slots=True)
class GateResult:
    obs: OrientedDetection | None
    reason: str  # 채택 시 "", 기각 시 사람이 읽을 사유 (trace 에 그대로)
    # 도약/z 도약으로 기각된 **품질 통과** 후보 (score·점군은 통과) — 재앵커
    # 판정 입력 (TrackState.consider_reanchor). 다른 기각 클래스는 None
    # (저품질 관측으로 재앵커하면 안 됨).
    rejected: OrientedDetection | None = None


@dataclass(slots=True)
class TickDecision:
    """tick 하나의 판정 결과 — steps 의 servo 루프가 이걸 보고 모션을 명령한다."""

    action: str  # "correct" | "descend" | "commit" | "hold" | "abort"
    reason: str
    lateral_m: float = 0.0


def _face_align_dist_deg(yaw_deg: float, grasp_yaw_rad: float) -> float:
    """yaw(도) 가 물체 면 정렬각(grasp_yaw mod 90°)에서 얼마나 먼가 (0~45)."""
    gy = math.degrees(grasp_yaw_rad)
    d = (yaw_deg - gy) % 90.0
    return min(d, 90.0 - d)


def grasp_families(obs: OrientedDetection) -> list[GraspFamily]:
    """coarse 관측 → 파지 자세 후보 가족 (선호순) — 도달 판정은 motion resolve 몫.

    조 축 yaw = **절대 격자(_YAW_GRID_DEG) + 면 정렬각 2개**(grasp_yaw, +90°) —
    OBB 앵커/aspect 문턱/2단 확장 폐지 (상단 _YAW_GRID_DEG 사고 사슬 주석).
    "이 yaw 로 물 수 있나"의 물리(그 방향 관측 폭 ≤ 개구)는 호출자(plan)의
    width 게이트 몫 — 여기는 자세 후보만.

    순서 = 선호: tilt 사다리(수직부터, geometry._TILTS_DEG) → 면 정렬 근접
    yaw → flip. 회전 구성은 open-loop plan_grasp 와 동일 규약
    (tool x=접근, y=조 축, z=x×y).
    """
    down = np.array([0.0, 0.0, -1.0])
    # yaw 후보: 면 정렬각 2개 + 절대 격자, 면 정렬 근접순. 근접 중복(<반 격자)
    # 은 격자 쪽을 제거 — 면 정렬각이 그 밴드의 대표.
    exact = [math.degrees(obs.grasp_yaw) % 180.0,
             (math.degrees(obs.grasp_yaw) + 90.0) % 180.0]
    yaws = list(exact)
    for g in np.arange(0.0, 180.0, _YAW_GRID_DEG):
        if all(min(abs(g - e) % 180.0, 180.0 - abs(g - e) % 180.0)
               >= _YAW_GRID_DEG / 2.0 for e in exact):
            yaws.append(float(g))
    # 1차 = 면 정렬 근접 (물체 옆면에 조가 수직), 2차 = 짧은 변 물기 우선
    # (= grasp_yaw+90 근접 — 짧은 쪽이 개구에 들어갈 확률이 높다, 옛
    # jaw∥short 1순위 계약 보존).
    short_yaw = (math.degrees(obs.grasp_yaw) + 90.0) % 180.0

    def _dist180(a: float, b: float) -> float:
        d = abs(a - b) % 180.0
        return min(d, 180.0 - d)

    yaws.sort(key=lambda y: (
        _face_align_dist_deg(y, obs.grasp_yaw), _dist180(y, short_yaw)
    ))

    out: list[GraspFamily] = []
    for tilt_deg in _TILTS_DEG:
        for yaw_deg in yaws:
            jaw_yaw = math.radians(yaw_deg)
            for flip in (1.0, -1.0):
                y = np.array(
                    [math.cos(jaw_yaw), math.sin(jaw_yaw), 0.0]
                ) * flip
                approach = Rotation.from_rotvec(
                    y * math.radians(tilt_deg)
                ).apply(down)
                rot_m = np.column_stack(
                    [approach, y, np.cross(approach, y)]
                )
                qx, qy, qz, qw = (
                    float(v) for v in Rotation.from_matrix(rot_m).as_quat()
                )
                out.append(
                    GraspFamily(
                        label=(
                            f"jaw@{yaw_deg:.0f}° tilt={tilt_deg:+d} "
                            f"flip={'+' if flip > 0 else '-'}"
                        ),
                        quat=(qx, qy, qz, qw),
                        approach=(
                            float(approach[0]), float(approach[1]),
                            float(approach[2]),
                        ),
                        jaw_axis=(float(y[0]), float(y[1]), float(y[2])),
                        tilt_deg=tilt_deg,
                        flip=flip,
                    )
                )
    return out


def refit_family(
    fam: GraspFamily, obs: OrientedDetection, min_delta_deg: float = 10.0
) -> GraspFamily | None:
    """재획득 관측의 yaw 로 **같은 변형**(tilt × flip)의 면 정렬 가족 재유도.

    가족은 plan coarse 의 yaw 로 고정되는데(자세 고정 = common-mode 상쇄 계약),
    물체가 밀리거나 튕기며 **회전**하면 옛 각도로 닫게 된다 (2026-07-17 실물:
    1차 close 튕김으로 86°→-27° 회전 → 2차가 24° 스큐로 닫아 재튕김). 물체가
    움직인 걸 아는 국면(재시도 재획득)에서만 호출 — yaw 차(mod 90)가 문턱
    미만이면 None (기존 유지, 마구 돌리지 않는다).

    §11 이후: 같은 (tilt, flip) 의 **면 정렬 최우선** yaw 가족 = 새 관측 기준
    그 변형의 1순위 (grasp_families 정렬 계약).
    """
    cand = next(
        (f for f in grasp_families(obs)
         if f.tilt_deg == fam.tilt_deg and f.flip == fam.flip),
        None,
    )
    if cand is None:
        return None
    a0 = math.atan2(fam.jaw_axis[1], fam.jaw_axis[0])
    a1 = math.atan2(cand.jaw_axis[1], cand.jaw_axis[0])
    delta = abs((math.degrees(a1 - a0) + 90.0) % 180.0 - 90.0)
    if delta < min_delta_deg:
        return None
    return cand


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
    latest: OrientedDetection,
    fused: OrientedDetection,
    cfg: ServoConfig,
    floor_z: float | None = None,
) -> Vec3:
    """파지 지점 (물체 좌표, TCP 아님) — XY 는 **최신 관측** (common-mode 상쇄는
    최신 자세의 측정에만 성립), z 는 **윗면 앵커** (top band centroid z 아래로
    grip_below_top_m).

    base_z 앵커 폐기 근거 = ServoConfig.grip_below_top_m 주석 (단일 top-view 의
    base_z ≈ 윗면 — nip 튕김 실사고). 하한 guard 두 겹 (모양 가정 없이 관측만):
    ① 융합 height 신뢰 가능(옆면 뷰 포함) → 관측 바닥 +4mm 위.
    ② floor_z(plan 의 클러스터 min base_z = 실 바닥 추정) → 바닥 +floor_clear 위
       — height 를 못 믿는 납작한 물체(< height_credible)에서 grip_below_top 이
       테이블을 뚫는 것을 막는 유일한 데이터 (물체 크기 무관 안전망).
    상한은 윗면 −2mm (헛집기 방지) — 하한들과 충돌하면 상한이 이긴다 (얕은
    물체는 윗쪽 sliver 파지가 물리적으로 유일한 선택).
    """
    top = latest.position[2]
    z = top - cfg.grip_below_top_m
    if fused.height >= cfg.height_credible_m:
        z = max(z, top - fused.height + 0.004)
    if floor_z is not None:
        z = max(z, floor_z + cfg.floor_clear_m)
    z = min(z, top - 0.002)
    return (latest.position[0], latest.position[1], z)


def grasp_tcp(
    point: Vec3, family: GraspFamily, lateral_m: float, engage_m: float = 0.0
) -> Vec3:
    """파지 지점 → TCP 목표 — tool frame 오프셋 2개 적용:
    y(조 축) = lateral (단일 가동 조 보정, plan_grasp 동일 규약),
    x(접근축) = engage (조 끝이 아니라 안쪽에 물리는 전진 — cfg.engage_m 주석).
    """
    rot = Rotation.from_quat(family.quat)
    off = rot.apply([engage_m, lateral_m, 0.0])
    return (
        float(point[0] + off[0]),
        float(point[1] + off[1]),
        float(point[2] + off[2]),
    )


def standoff(tcp: Vec3, family: GraspFamily, s_m: float) -> Vec3:
    """파지 TCP 에서 접근축 후방 s_m 의 standoff TCP."""
    a = family.approach
    return (tcp[0] - a[0] * s_m, tcp[1] - a[1] * s_m, tcp[2] - a[2] * s_m)


def midstop_sequence(g_tcp: Vec3, family: GraspFamily, cfg: ServoConfig) -> list[Vec3]:
    """commit 2단 하강의 중간 정지 시퀀스 — 마지막 원소 도달 후 FK 실측.

    [midstop, midstop+dither(후방), midstop] — 마지막 이동이 **하강(접근) 방향**
    이 되도록 왕복해 기어열을 착지 때와 같은 플랭크에 앉힌다 (재안착 없이 재면
    stall 국면 잔차를 재서 release 시 또 과보상). dither=0 이면 왕복 생략,
    midstop=0 이면 기능 자체 off (빈 시퀀스 — 호출부는 단발 하강)."""
    if cfg.commit_midstop_m <= 0.0:
        return []
    mid = standoff(g_tcp, family, cfg.commit_midstop_m)
    if cfg.commit_dither_m <= 0.0:
        return [mid]
    back = standoff(g_tcp, family, cfg.commit_midstop_m + cfg.commit_dither_m)
    return [mid, back, mid]


def reanchor(
    g_tcp: Vec3, cmd1: Vec3, measured: Sequence[float], max_m: float
) -> tuple[Vec3, Vec3]:
    """midstop 실측 잔차로 최종 하강 명령 재앵커 → (resid, cmd2).

    resid = 방금 그 자세에서 잰 플랜트 미달 (cmd1 − 실측 FK, 축별 ±max_m clamp
    — 오염 실측 방어). cmd2 = g_tcp + resid: release 됐으면 resid≈0 → 과보상
    없음(스침 소멸), 안 풀렸으면 resid ≈ 기존 comp → 오늘과 동일 동작."""
    resid = np.clip(
        np.asarray(cmd1, dtype=float) - np.asarray(measured, dtype=float),
        -max_m, max_m,
    )
    cmd2 = (
        float(g_tcp[0] + resid[0]),
        float(g_tcp[1] + resid[1]),
        float(g_tcp[2] + resid[2]),
    )
    return (float(resid[0]), float(resid[1]), float(resid[2])), cmd2


def descent_suspect(
    samples: list[dict], gripper_index: int, load_thr_raw: int
) -> bool:
    """하강 프로파일에서 바닥 접촉 의심 — arm 관절 load 스파이크 (진단 전용).

    gripper load 는 close 국면 신호라 제외. 문턱은 ServoConfig 주석대로 실물
    특성화로 튜닝 (플래그가 제어를 바꾸지 않으므로 오판은 노이즈일 뿐)."""
    for s in samples:
        loads = s.get("loads")
        if not loads:
            continue
        arm = [v for i, v in enumerate(loads) if i != gripper_index]
        if arm and max(abs(int(v)) for v in arm) >= load_thr_raw:
            return True
    return False


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
    match_radius_m: float | None = None,
) -> GateResult:
    """tick 관측 게이트 — 실데이터의 실패 클래스가 근거 (docstring 상단):

    ① 매치: 기대 위치 XY 반경 안 최근접 후보 (prompt 가 다른 물체를 잡는 것 차단)
    ② 도약: 직전 채택 대비 jump_max 초과 = mask 오검출 의심 (실데이터 455mm 사례)
    ③ 점군: min_points 미만 = depth 붕괴/가림 (근접 한계 신호)
    기각 사유는 문자열로 — trace/로그에 그대로 남아 사후분석 가능.

    match_radius_m: 기본 cfg 값 override — 낙하 재획득처럼 "물체가 움직였음을
    아는" 국면은 넓혀서 부른다 (2026-07-17 실물: 떨어진 큐브가 10cm 굴러갔는데
    5cm 반경이 재획득을 거부 → abort).
    """
    if not candidates:
        return GateResult(None, "검출 0건")
    radius = match_radius_m if match_radius_m is not None else cfg.match_radius_m
    best: OrientedDetection | None = None
    best_d = radius
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
            f"매치 실패 — 기대 위치 반경 {radius * 1000:.0f}mm 밖 "
            f"(최근접 {near * 1000:.0f}mm)",
        )
    # 품질 게이트(score/점군)를 도약 게이트보다 먼저 — 열화 관측은 앵커도
    # 재앵커 후보도 될 수 없다 (2026-07-17 13:53: score 0.43 부분 뷰가 앵커가
    # 되어 정상 관측을 연속 기각).
    if best.score < cfg.min_score:
        return GateResult(
            None,
            f"저신뢰 관측 score {best.score:.2f} < {cfg.min_score:.2f} "
            "(부분 뷰/오검출 의심)",
        )
    n = len(best.points or [])
    if n < cfg.min_points:
        return GateResult(
            None, f"점군 부족 {n} < {cfg.min_points} (depth 붕괴/가림 의심)"
        )
    if last_accepted is not None:
        jump = math.dist(best.position, last_accepted.position)
        if jump > cfg.jump_max_m:
            return GateResult(
                None,
                f"위치 도약 {jump * 1000:.0f}mm > {cfg.jump_max_m * 1000:.0f}mm "
                "(mask 오검출 의심)",
                rejected=best,
            )
        dz = abs(best.position[2] - last_accepted.position[2])
        if dz > cfg.z_jump_max_m:
            # 윗면 z 는 파지 z 의 앵커 — 정지 물체에서 tick 간 cm 급 z 변동은
            # 물리 불가 (조 끝/가림 depth 오염). XY 도약보다 좁게 gate.
            return GateResult(
                None,
                f"윗면 z 도약 {dz * 1000:.0f}mm > "
                f"{cfg.z_jump_max_m * 1000:.0f}mm (조/가림 depth 오염 의심)",
                rejected=best,
            )
    return GateResult(best, "")


@dataclass(slots=True)
class PlantComp:
    """명령-실측 잔차 보상 (feedforward) — "명령한 절대 pose ≠ 도달 pose"
    (backlash/부하 sag). 무보상 절대 재명령은 정상상태 오차가 영원히 남는다
    (2026-07-16 실물: 관측·목표 안정인데 lateral 8~12mm 정체 → capture 턱걸이
    commit → 헛집기 2연속). 직전 명령 − 실측 FK 를 다음 명령에 가산 — 상수
    오프셋은 1스텝 소거. clamp 는 오검출/이상 실측의 폭주 방지.

    사용 규약: 매 정지 관측에서 observe(실측 TCP) → 이동 목표는 apply(target)
    → **실제로 실행된** 명령만 commanded(cmd) (거부된 이동은 기록하지 않는다).
    """

    max_m: float = 0.03
    _comp: np.ndarray = field(default_factory=lambda: np.zeros(3))
    _last_cmd: Vec3 | None = None

    def observe(self, measured: Sequence[float]) -> None:
        """플랜트 잔차 갱신 — 검출과 무관 (직전 명령 vs 실측 FK 만 필요)."""
        if self._last_cmd is None:
            return
        self._comp = np.clip(
            np.asarray(self._last_cmd, dtype=float)
            - np.asarray(measured, dtype=float),
            -self.max_m, self.max_m,
        )

    def apply(self, target: Vec3) -> Vec3:
        return (
            float(target[0] + self._comp[0]),
            float(target[1] + self._comp[1]),
            float(target[2] + self._comp[2]),
        )

    def commanded(self, cmd: Vec3) -> None:
        self._last_cmd = cmd

    @property
    def mm(self) -> list[float]:
        """trace 기록용 (mm, 반올림)."""
        return [round(float(v) * 1000, 1) for v in self._comp]


@dataclass(slots=True)
class TrackState:
    """관측 추적 + 파지 기하 상태 — servo_pick 루프가 소유 (2026-07-17 리팩토링:
    루프 지역변수 산개 → 응집. 각 필드/전이의 실물 사고 근거는 메서드 주석).

    ServoState(tick/rung 카운터, decide_tick 입력)와 별개 — 이쪽은 "무엇을
    어디서 어떻게 잡을지"의 추적 상태.
    """

    fam: GraspFamily
    expected_xy: Vec3
    g_tcp: Vec3
    g_point: Vec3  # 물체측 파지점 (마커 표시용 — update_grasp 가 매 채택마다 갱신)
    lateral: float
    fallback_width_m: float  # coarse footprint — 점군 없는 tick 의 폭 fallback
    floor_z: float | None
    last: OrientedDetection | None = None
    accepted: list[OrientedDetection] = field(default_factory=list)
    widths: list[float] = field(default_factory=list)
    move_fails: int = 0
    close_attempts: int = 0
    reacquiring: bool = False
    # 직전 tick 의 도약-기각 관측 (품질 통과분만 — GateResult.rejected) —
    # 재앵커 판정의 비교 기준.
    last_rejected: OrientedDetection | None = None

    def note_accept(self, obs: OrientedDetection) -> None:
        """채택 관측 반영 — gate 기준/기대 위치 갱신 (융합은 호출부: 이력이
        갱신된 뒤 최근 k 개로 융합해야 해서 순서 의존)."""
        self.last = obs
        self.accepted.append(obs)
        self.expected_xy = obs.position
        self.last_rejected = None

    def consider_reanchor(
        self, rejected: OrientedDetection | None, cfg: ServoConfig
    ) -> OrientedDetection | None:
        """연속 도약-기각 2건이 상호 일관하면 그 관측 채택(재앵커) — 반환값이
        새 앵커, 아니면 None.

        나쁜 앵커가 좋은 관측 스트림을 기각하는 역전 차단 (2026-07-17 13:53
        실물: tick1 열화 관측이 앵커 → 정상 관측(0.83) 2연속 'z 도약' 기각 →
        소실 중단 — 기각된 둘은 서로 1.4mm/0mm 일치였다). 일관 문턱 = 도약
        게이트의 절반 (jump/2, z_jump/2 — 정상 연속 관측의 실측 일치는 mm 급).
        위험 인지: 정지 상태의 조/가림 오염도 2연속 일관할 수 있음 — 그래서
        score·점군 게이트를 통과한 기각만 후보(GateResult.rejected 계약)이고,
        이후 이동 IK 거부 관용·close 파지 판정이 최종 안전망."""
        if rejected is None:
            self.last_rejected = None
            return None
        prev, self.last_rejected = self.last_rejected, rejected
        if prev is None:
            return None
        dxy = math.hypot(
            rejected.position[0] - prev.position[0],
            rejected.position[1] - prev.position[1],
        )
        dz = abs(rejected.position[2] - prev.position[2])
        if dxy <= cfg.jump_max_m / 2 and dz <= cfg.z_jump_max_m / 2:
            self.last_rejected = None
            return rejected
        return None

    def update_grasp(
        self, obs: OrientedDetection, fused: OrientedDetection, cfg: ServoConfig
    ) -> None:
        """파지 기하 갱신 — 폭은 채택 관측별 측정의 **중앙값** (단일 뷰 depth
        번짐 outlier 가 폭을 부풀려 lateral_offset 을 밀어낸 실사고: 실물
        20mm 가 det 33mm)."""
        self.widths.append(
            width_along(obs.points, self.fam.jaw_axis, self.fallback_width_m)
        )
        width = float(np.median(self.widths))
        self.lateral = lateral_offset(width)
        self.g_point = grasp_point(obs, fused, cfg, self.floor_z)
        self.g_tcp = grasp_tcp(self.g_point, self.fam, self.lateral, cfg.engage_m)

    def distrust_last(self) -> None:
        """이동 거부 시 — 허공 목표를 만든 최신 관측을 이력에서 빼고 gate
        기준(last)도 리셋해 다음 tick 이 깨끗이 재관측 (2026-07-17: 조/가림
        depth 오염 관측 → IK 거부가 옳은데 태스크 전체가 죽던 사고)."""
        if self.accepted:
            self.accepted.pop()
        self.last = self.accepted[-1] if self.accepted else None

    def reset_for_retry(self) -> None:
        """낙하/밀림 재시도 — 관측 이력·폭 리셋 (물체가 밀렸을 수 있음, 도약
        gate 가 밀린 물체를 오검출로 오판하지 않게) + 재획득 반경 확대.
        expected_xy 는 직전 파지 지점 유지 (최선의 추정)."""
        self.last = None
        self.accepted = []
        self.widths = []
        self.reacquiring = True


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
