"""handover 시나리오 step 들 — omx(giver)가 **자기 eye-in-hand 웹캠으로** 큐브를
보고 집어 공중에 제시하면, so101(receiver)이 **재검출**해 omx 가 문 면이 아닌
**직교 면**을 받아 상자에 적치.

⚠ **2026-07-23 전면 재배선 + 2026-07-26 펜→큐브 전환, 실물 미검증** (설계 근거 =
docs/omx_handover_prep.md). 동그란 펜은 omx 평행조로 안정 파지가 어려워 폐기
(사용자 지시 — cube.py docstring):
  A. omx 가 본다 — 계산된 nadir 관측 자세 + DETECT_PLANAR (mono ray∩z=table).
  B. omx 파지 계획 — top-down 전용(5축 도달성 §5.1) + 큐브 **중심** 파지 + 조 축
     = 검출 yaw 기준 면 정렬 후보(도달성이 채택). Z 사다리(depth 없어 크기 가정).
  C. omx 집기 — 파지 해로 move_j 스윙인 → close (top-down 은 책상면 전용 관절
     리밋이라 위에서 수직 접근·refine 불가 — omx=best-effort, 정밀은 so101).
  D. omx 제시 — 티칭 폐기, **계산**: 랑데부(두 workcell ROI 교집합) + 큐브는
     대칭이라 방위 제약 없음 → yaw×tilt 격자 열거, 도달성/충돌이 채택.
  E. so101 수취 — omx TCP FK 짐작 폐기, **closed-loop 재검출** (공중 대역, omx
     제시 위치/자세가 매번 달라 필수) + **위/아래 면 수직 조축 파지**(_RECV_FAMILY,
     스윕 2026-07-26) + refine 1 tick + 수취 순서 불변식(so101 held 뒤에만 omx
     open) + cross-robot 충돌 게이트 + 벽(뒤) 게이트.

실물 첫 런 전 확인 필수 가정 (omx_handover_prep.md §7 미지수):
  ① omx tcp/그리퍼 물리 조립이 URDF 규약(tool x=approach, y=jaw)과 일치 (§5.2).
  ② _OMX_TABLE_Z_M — omx base 가 책상 위 전제 (다르면 관측/파지 z 전체 시프트).
  ③ 파지 Z — omx depth 없음 → 2cm 큐브 중심 = 바닥+1cm 가정 (사용자 지시
     2026-07-26). 집는 위치가 이상하면 chosen_dz 로그로 보정 (_CUBE_PICK_DZ_LADDER).
  ④ omx held 판정 — gap 5%/load 80 은 so101 Feetech 실측값. omx=Dynamixel XL330
     은 load 스케일이 달라 **미검증** (§5.4) — 실물 전 gripper_characterize.py.

설계 원칙 (pick_and_place 계승): 계획(모션 0 resolve) 먼저·판정 해 == 실행 해 /
실패는 사유+다음 행동 명시 (침묵 fallback 금지 — refine 실패의 coarse 진행도
로그+trace 에 남긴다) / 수취 순서 불변식은 회귀 테스트 잠금.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    SnapshotBundleRequest,
)
from modules.detector.contract import (
    DetectOrientedResponse,
    DetectPlanarRequest,
    DetectRequest,
    Detector,
    OrientedDetection,
)
from modules.motion.contract import (
    JointTarget,
    Motion,
    MoveJRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    PoseTarget,
    ResolveReachableRequest,
    ResolveReachableResponse,
    TcpPose,
    TcpSnapshotRequest,
    TcpState,
)
from modules.motor.contract import (
    JointState,
    Motor,
    ReadStateRequest,
    SetGripperRequest,
    SetGripperResponse,
    SetTorqueRequest,
    SetTorqueResponse,
)
from modules.shared_config.contract import (
    SharedConfig,
    SnapshotWorkcellRequest,
    WorkcellBundle,
    WorkcellRoi,
)
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import (
    DetectionNotFound,
    GraspFailed,
    NoReachableGrasp,
    TaskError,
)
from modules.tasks.core.step import step
from modules.waypoint.contract import (
    GetWaypointByNameRequest,
    GetWaypointByNameResponse,
    ListGroupMembersByNameRequest,
    ListGroupMembersByNameResponse,
    Waypoint,
    WaypointRecord,
)

from . import cube, frames
from .collision import BasePose, CrossRobotChecker
from .cube import CubeGrasp
from .frames import robot_to_world, world_to_robot
from .trace import HandoverTrace

logger = logging.getLogger(__name__)

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

# ─── 상수 (노브 SSOT — 실물 첫 런 데이터로 튜닝, 전부 미검증 기본값) ────
#
# knob_snapshot() 이 이 블록 전체를 trace summary 에 각인한다 — 실물 런 결과와
# 노브 값이 항상 한 파일에 붙어 다니게 (task.md §4 노브 SSOT 규약).

_SEARCH_GROUP = "search"  # so101 적치 검출 스윕 waypoint 그룹 (pick_and_place 공유)
_SEARCH_SETTLE_S = 0.6
_TOP_K = 3
_GRIPPER_SETTLE_S = 4.0  # close 완료 대기 (pick_and_place 와 동일 근거)
# held 판정 load 하한 — ⚠ so101(Feetech STS3215) 실측값. omx(Dynamixel XL330)
# 는 load 단위가 달라 무의미할 수 있음 (§5.4 — 얇은 펜은 gap≈닫힘이라 load 가
# 유일 판별자). 실물 전 scripts/gripper_characterize.py 로 재도출.
_HELD_LOAD_MIN_RAW = 80

# ── A. omx 관측 (mono z=0) ──
# omx base frame 의 테이블 평면 z — **1회 설정/측정 앵커** (omx 는 depth 가 없어
# 스스로 못 잼, omx_handover_prep.md §4 횡단 전제). base 가 책상 위 설치 전제
# = 0.0. 실물 첫 런에서 자/블록로 실측 보정.
_OMX_TABLE_Z_M = 0.0
_OMX_OBSERVE_CAM_H_M = 0.25  # nadir 카메라 높이 — §8-1 계산 (25cm = 도달영역 100% 커버)
# 카메라 optical-roll 후보 (deg) — 관측은 이미지 방위 무관이라 자유 DOF. 선호
# 90° = 넓은 화각 축(H 94°)을 omx 도달영역 넓은 축(좌우 44cm)에 정렬 (§8-1).
_OMX_OBSERVE_PSI_DEG = (90.0, 60.0, 120.0, 30.0, 150.0, 0.0, -90.0)
_OBSERVE_SETTLE_S = 0.6
# 검출 신뢰 게이트 — 큐브 footprint 기하 (mono 는 score 만으론 약함). 2cm 큐브라
# 두 변 모두 ~2cm 대역 + 종횡비 1 근처 (펜류 길쭉함 컷).
_SCORE_MIN = 0.45
_CUBE_EDGE_MIN_M = 0.010  # footprint 짧은 변 하한
_CUBE_EDGE_MAX_M = 0.045  # footprint 긴 변 상한 (mono 번짐 여유 포함)
_CUBE_ASPECT_MAX = 2.2  # 긴 변/짧은 변 상한 (초과 = 펜류, 큐브 아님)

# ── B. omx 파지 계획 (큐브 중심 top-down) ──
# 큐브는 대칭 — 조 축은 검출 yaw 기준 면 정렬(0/90) 우선 + 대각(45/135) 폴백,
# 도달성이 채택 (square footprint 의 grasp_yaw 는 노이지).
_CUBE_PICK_YAW_OFFSETS_DEG = (0.0, 90.0, 45.0, 135.0)
# footprint 평균 clamp (mono 번짐 방어) — 크기 로깅용, Z 는 아래 사다리.
_CUBE_SIZE_MIN_M = 0.012
_CUBE_SIZE_MAX_M = 0.030
# 파지 Z 사다리 (table 위 dz, 첫 도달+바닥클리어 채택). ⚠ 실물(2026-07-26):
# TCP=바닥+1cm 로 하니 **그리퍼가 바닥을 훑었다** — omx URDF 상 물리 손끝이
# TCP(link5 x 9.19cm)보다 ~11mm 아래로 뻗고(probe omx_pick_z_probe), 게다가
# **손끝에 붙인 골무(finger cot)는 URDF 에 없어** 그 두께만큼 더 내려간다.
# → floor 게이트 = URDF 여유(_CUBE_PICK_FLOOR_CLEAR_M) + 골무 오프셋
# (_GRIPPER_TIP_EXTRA_M) 를 요구해 그만큼 TCP 를 올린다. 2cm 큐브라 TCP 가
# 높아도(≤22mm) 긴 손가락이 옆면 아래쪽까지 문다. resolve 가 실 손가락 메시로
# 걸러 사다리 중 가장 낮게 클리어하는 높이를 채택 — chosen_dz(trace)로 확인.
_CUBE_PICK_DZ_LADDER = (0.014, 0.016, 0.018, 0.020, 0.022, 0.024)
# floor 게이트 = table + (실 손끝 목표 여유 + 골무 연장). URDF 손끝은 TCP−11mm
# 이라 14mm 면 3mm 뜨지만, 얇은 골무(~2mm)가 그걸 거의 먹어 "거의 훑음"이었다
# (2026-07-26 실물, 둘 다 재시작 후). → 실 손끝이 눈에 띄게 뜨게 5mm 목표.
_CUBE_PICK_FLOOR_CLEAR_M = 0.01  # 실 손끝이 바닥에서 뜰 목표 여유 (조금 더 위로, 2026-07-26)
# 골무 등 URDF 미모델 손끝 연장 — floor 게이트에 가산 (URDF 는 골무를 모름).
# ⚠ **실물 튜닝 노브**: 여전히 긁으면 ↑ / 큐브 위를 잡아 미끄러지면 ↓
# (chosen_dz 로그 확인). 사용자 실측 "얼마 안 튀어나옴" → 2mm.
_GRIPPER_TIP_EXTRA_M = 0.002

# ── C. 수취 refine 보정 게이트 (so101 수취측 so_refine 사용) ──
_REFINE_JUMP_MAX_M = 0.03  # 겨냥점 보정 상한 — 초과 = 관측 오염 의심 (계획값 유지)

# ── D. 제시 (랑데부 계산) ──
# ⚠ 랑데부 = **so101 도달 robust 실측 지점** (2026-07-26 실물+probe). so101 공중
# 도달이 razor-thin(어디서든 192개 중 3~5개)이라, 랑데부를 omx/전방x/반경으로
# 고르면 실 큐브(omx 그립으로 ~3cm 어긋남)에서 도달 0 이 된다(실물 확인). robust
# probe: (x0.16~0.20, y0.26, z0.28)만 [so101 도달(±3cm 버팀)+omx 제시+비가림+충돌
# 여유15mm] 전부 통과 → 그 지점을 prefer_point 로 직접 겨냥 (z0.32 는 그 지점서
# omx 제시 도달 X 라 제외). 실 큐브는 여전히 재검출로 잡음(closed-loop) — 이건
# omx 가 큐브를 가져다 놓을 목표점.
_PRESENT_Z_WORLD = (0.28, 0.30, 0.26)  # 파지점(TCP)의 world z 후보 (선호순)
_RENDEZVOUS_PREFER_XY = (0.18, 0.26)  # so101 도달 robust 지점 (world xy, 실측)
_PRESENT_LIMIT = 8  # 랑데부 후보 상한 (resolve+충돌 게이트 시도 수)
# 제시 자세족 파라미터 — **일반 grasp 샘플러**(_grasp_family): e=접근고도(0수평..
# 90수직하향) / az=방위 스텝 / r=조축 roll. 큐브는 대칭이라 임의 tool 방위가 유효,
# 스윕이 이 샘플러로 제시 도달 자세를 전역에서 찾음 (옛 접선+spin 은 악수 높이에서
# 전멸했음 — 높이가 바뀌면 도달 다양체가 통째로 달라진다).
_PRESENT_FAMILY_E = (0.0, 30.0, 60.0, 90.0)
_PRESENT_FAMILY_AZ_STEP = 30.0
_PRESENT_FAMILY_R = (0.0, 90.0)

# ── E. so101 수취 ──
# ⚠ **수취는 "위/아래 면 수직 조축 파지"가 정답** (2026-07-26 전수 스윕 + 사용자
# 실측 재현). omx 는 큐브 옆면을 물고, so101 은 위/아래면을 수직 조축으로 받는다
# (= 자연스러운 "다른 면", 두 그리퍼 회피). 옛 _grasp_quat family 는 조축을 항상
# **수평**으로만 만들어 이 파지를 못 냈고 → 악수 높이 실 중심에서 도달 0개였다.
# 그래서 수취도 **일반 grasp 샘플러**(_grasp_family) 로 열거, 수직 조축(r90) 우선.
# ⚠ 관측 사다리 — omx 가림 회피가 지배 (2026-07-26 실물: omx 그리퍼가 큐브를
# 가려 검출 실패 + 높은 관측 자세라 J2 떨림). occlusion probe(scratchpad
# so101_observe_occlusion_probe): **저각(elev 30°) + so101 측 방위만 비가림**,
# 높은 각(40/55)은 omx 에 전부 가림 + 더 뻗어 J2 부하↑. 이 밴드로 좁혀 resolve
# 가 뭘 고르든 비가림 (밴드 전체가 clear 실측). ⚠ 큐브 위치가 크게 바뀌면 가림
# 기하도 바뀌니 재-probe.
_RECV_OBS_DIST_M = (0.18, 0.15)  # 재검출 카메라-큐브 거리 (probe: 0.18 최적)
_RECV_OBS_ELEV_DEG = (30.0, 25.0)  # 저각만 비가림 (40/55 는 omx 가림 + J2 부하↑)
_RECV_OBS_AZOFF_DEG = (20.0, 0.0, 40.0, -20.0)  # so101 측, +20 이 비가림+최소 J2
_RECV_OBS_PSI_DEG = (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)
_RECV_MATCH_RADIUS_M = 0.08  # 제시 계획점 대비 재검출 매치 반경
_RECV_Z_BAND_M = 0.06  # 공중 대역 — 제시 z ± 이 값 (테이블 대역 게이트의 개방판)
# 공중 큐브는 작고 omx 그리퍼 옆이라 GDINO 신뢰도가 낮다 — 2026-07-26 실물:
# 위치 정확(계획점 3cm)·z 1mm·점군 116 인 진짜 검출인데 score 0.327 로 0.35 문턱에
# 걸림. 위치/z/반경(8cm)/점군(≥20) 게이트가 오검출 방어를 하므로 score 는 낮춰도
# 안전. → 0.25 (0.327 에 여유 + 다음 런 변동 대비).
_RECV_SCORE_MIN = 0.25
_RECV_MIN_POINTS = 20  # 공중 물체 점군 하한 (큐브는 펜보다 조밀 — 가림만 주의)
# 수취 자세족 파라미터 — 일반 grasp 샘플러. r90(수직 조축=위/아래면)을 앞에 둬
# 선호(스윕: 도달·비관통 해가 전부 수직 조축). resolve 가 순서대로 첫 도달 채택.
_RECV_FAMILY_E = (0.0, 30.0, 60.0, 90.0)
_RECV_FAMILY_AZ_STEP = 30.0
_RECV_FAMILY_R = (90.0, 45.0, 0.0, 135.0)
_RECV_PRE_CLEAR_M = 0.07
# 랑데부 반경 선호 — prefer_point(위 _RENDEZVOUS_PREFER_XY) 폴백용만 남김.
# ⚠ 반경만으론 so101-robust 지점을 못 겨냥해서(같은 r 딴 y 채택) prefer_point 로
# 대체됨 (2026-07-26 실물: r0.24 겨냥이 실 큐브 도달 0 이었음).
_RENDEZVOUS_R_SO_M = 0.30
_RECV_WITHDRAW_M = 0.08
_RECV_COLLISION_RETRY = 5
# 핸드오프 근접 국면 margin — 악수 높이(z~0.32)에선 두 팔이 벌어져 스윕 여유가
# 16~22mm (크로스캘 σ_t ~8mm 의 2배↑ = 편안). margin 8mm 로 게이트 (σ 수준 —
# 실물 첫 수취에서 재확인). z 를 낮추면 여유가 급감(음수)하니 높이 유지가 핵심.
_RECV_COLLISION_MARGIN_M = 0.008
_OMX_HOLD_GRIP_FRAC = 0.2  # 충돌 형상 — 큐브 든 omx 조 개구 (거의 닫힘)
# 벽(뒤) 침범 게이트 — 팔 링크 world x 하한. 이보다 뒤로 넘어가는 IK 해는 기각
# (사용자: 뒤는 벽). so101 원점 기준, omx base x≈0.034. 스윕: 앞쪽·악수높이
# 채택해는 링크 min-x ≈ -0.007 로 안전. 실 테이블 벽 위치 보고 조정.
_WALL_MIN_X_M = -0.03

# 접촉 인접 이동 감속 (흉터 2 — 접촉 인접 이동이 물체를 흘림/이젝션)
_GENTLE_SPEED_SCALE = 0.25

# 적치 (pick_and_place plan_place 슬림판 — 상자 위 open-loop, v1 유지).
_PLACE_TILTS_DEG = (0, 30, -30, 45, -45)
_PLACE_YAW_OFFSETS_DEG = (0.0, 90.0, 180.0, 270.0)
_PLACE_DROP_CLEAR_M = 0.005
_PLACE_PRE_CLEAR_M = 0.06
_BASE_Z_MAX_M = 0.08  # 적치 spot 대역 (테이블)

# 기준 자세: 툴 x(approach)→base -z (수직 하향), y(조 축)→base +y — so101
# URDF tcp 규약 (pick_and_place geometry._TOPDOWN 동일. omx 도 동일 — §5.2
# 구조 확정, 물리 조립 일치는 실물 미지수 가정 ①).
_TOPDOWN = Rotation.from_matrix(
    np.column_stack([[0, 0, -1], [0, 1, 0], [1, 0, 0]]))


def knob_snapshot() -> dict[str, float | tuple]:
    """노브 블록 스냅샷 — trace summary 각인용 (값과 결과가 한 파일에)."""
    g = globals()
    return {
        k: g[k]
        for k in sorted(g)
        if k.startswith("_") and k.isupper() and isinstance(g[k], (int, float, tuple))
    }


async def _emit(trace: HandoverTrace | None, record: dict) -> None:
    """trace 기록 (없으면 no-op) — 관측이 실행을 죽이지 않게 예외는 삼키고 로깅."""
    if trace is None:
        return
    try:
        await asyncio.to_thread(trace.emit, record)
    except Exception:
        logger.exception("handover trace 기록 실패 (실행 영향 없음)")


def _grasp_quat(yaw: float, tilt_deg: float) -> Quat:
    """yaw(조 축 방위) × tilt(조 축 둘레 기울임) → TCP quat — pick_and_place
    회전 구성과 동일 규약 (tool x=approach, y=jaw). tilt=0 이면 tool z 의
    world 방위각 = yaw (펜 축 정렬에 사용 — tool z ∥ 펜 노출 방향 규약)."""
    rot = (
        Rotation.from_euler("z", yaw)
        * _TOPDOWN
        * Rotation.from_euler("y", math.radians(tilt_deg))
    )
    qx, qy, qz, qw = (float(v) for v in rot.as_quat())
    return (qx, qy, qz, qw)


def _approach_of(yaw: float, tilt_deg: float) -> Vec3:
    rot = (
        Rotation.from_euler("z", yaw)
        * _TOPDOWN
        * Rotation.from_euler("y", math.radians(tilt_deg))
    )
    a = rot.apply([1.0, 0.0, 0.0])
    return (float(a[0]), float(a[1]), float(a[2]))


def _grasp_family(
    e_list: tuple[float, ...],
    az_step_deg: float,
    r_list: tuple[float, ...],
) -> list[tuple[str, Quat, Vec3]]:
    """일반 grasp 자세 샘플러 → [(라벨, quat, 접근 단위벡터 tool x)].

    e=접근 고도(0=수평 접근 .. 90=수직 하향), az=접근 방위, r=조 축 roll. r 로
    **수직 조축(위/아래 면)·수평 조축(옆면) 둘 다** 나온다 — 이게 _grasp_quat
    (top-down tilt, 조축 항상 수평)이 못 내던 자세족이고, 핸드오프 스윕
    (2026-07-26)이 이 샘플러로 so101 위/아래면 수취·omx 제시를 전역에서 찾았다.
    tool x=접근(a), tool y=조축(jaw), tool z=a×jaw. quat 은 robot frame."""
    out: list[tuple[str, Quat, Vec3]] = []
    for e_deg in e_list:
        e = math.radians(e_deg)
        for az_deg in np.arange(0.0, 360.0, az_step_deg):
            az = math.radians(float(az_deg))
            a = np.array(
                [math.cos(e) * math.cos(az), math.cos(e)
                 * math.sin(az), -math.sin(e)]
            )
            a = a / np.linalg.norm(a)
            ref = np.cross([0.0, 0.0, 1.0], a)
            if np.linalg.norm(ref) < 1e-6:  # 접근이 수직 → ref 임의 수평
                ref = np.array([1.0, 0.0, 0.0])
            ref = ref / np.linalg.norm(ref)
            for r_deg in r_list:
                jaw = Rotation.from_rotvec(a * math.radians(r_deg)).apply(ref)
                jaw = jaw / np.linalg.norm(jaw)
                z = np.cross(a, jaw)
                q = Rotation.from_matrix(
                    np.column_stack([a, jaw, z])).as_quat()
                out.append(
                    (
                        f"e{e_deg:.0f}/az{az_deg:.0f}/r{r_deg:.0f}",
                        (float(q[0]), float(q[1]), float(q[2]), float(q[3])),
                        (float(a[0]), float(a[1]), float(a[2])),
                    )
                )
    return out


# 제시/수취 일반 자세족 (모듈 로드 시 1회 — tcp 위치 무관, 방위만). 스윕 검증.
_PRESENT_FAMILY = _grasp_family(
    _PRESENT_FAMILY_E, _PRESENT_FAMILY_AZ_STEP, _PRESENT_FAMILY_R
)
_RECV_FAMILY = _grasp_family(
    _RECV_FAMILY_E, _RECV_FAMILY_AZ_STEP, _RECV_FAMILY_R)


# ─── 0. 자산/설정 fail-fast (모션 0 시점) ─────────────────────────────


@step(title="waypoint 조회")
async def named_waypoint(
    ctx: TaskContext, robot_id: str, name: str, teach_hint: str
) -> WaypointRecord:
    res = await ctx.call(
        Waypoint.Service.GET_WAYPOINT_BY_NAME,
        GetWaypointByNameRequest(robot_id=robot_id, name=name),
        GetWaypointByNameResponse,
    )
    if res.waypoint is None:
        raise TaskError(
            f"'{name}' waypoint 없음 (robot={robot_id}) — {teach_hint}")
    return res.waypoint


@step(title="workcell 조회")
async def load_workcells(
    ctx: TaskContext, so101: str, omx: str
) -> tuple[WorkcellRoi, WorkcellRoi]:
    """양쪽 workcell ROI — 랑데부(공통 워크스페이스)와 omx 관측 겨냥의 SSOT.
    미설정은 모션 0 시점 명시 실패 (instance.yaml `workcell:` 블록이 앵커)."""
    bundle = await ctx.call(
        SharedConfig.Service.SNAPSHOT_WORKCELL,
        SnapshotWorkcellRequest(),
        WorkcellBundle,
    )
    roi_so = bundle.robots.get(so101)
    roi_omx = bundle.robots.get(omx)
    missing = [r for r, roi in (
        (so101, roi_so), (omx, roi_omx)) if roi is None]
    if missing:
        raise TaskError(
            f"workcell ROI 미설정: {missing} — robot/instances/<id>/instance.yaml "
            "에 workcell: 블록을 설정한 뒤 다시 실행하세요 (랑데부/관측 겨냥 앵커)"
        )
    assert roi_so is not None and roi_omx is not None
    return roi_so, roi_omx


@step(title="hand_eye 조회")
async def load_hand_eye(ctx: TaskContext, robot_id: str) -> np.ndarray:
    """T_tcp←cam (4×4) — 관측 자세 역산(T_tcp = T_cam · X⁻¹)에 필요.
    없으면 모션 0 시점 명시 실패 (침묵 identity 금지 — 실사고 전례 클래스)."""
    bundle = await ctx.call(
        Calibration.Service.SNAPSHOT_BUNDLE,
        SnapshotBundleRequest(robot_id=robot_id),
        CalibrationBundle,
    )
    if bundle.hand_eye is None:
        raise TaskError(
            f"{robot_id} hand_eye 캘 없음 — 캘 완료 후 다시 실행하세요 "
            "(관측 자세 계산에 필수, 침묵 identity 금지)"
        )
    x = np.eye(4)
    x[:3, :3] = np.array(
        bundle.hand_eye.result_data.R_cam2gripper, dtype=float)
    x[:3, 3] = np.array(bundle.hand_eye.result_data.t_cam2gripper, dtype=float).reshape(
        3
    )
    return x


# ─── A. omx 가 펜을 본다 (계산된 관측 자세 + mono z=0 검출) ──────────


def _camera_pose_groups(
    c: np.ndarray,
    z_axis: np.ndarray,
    psi_candidates_deg: tuple[float, ...],
    t_tcp_cam: np.ndarray,
) -> tuple[list[list[TcpPose]], list[float]]:
    """카메라 (위치 c, optical z) + roll ψ 후보 → TCP pose 그룹들.

    관측은 이미지 방위 무관 → optical-roll 이 자유 DOF. ψ 마다 T_base_cam 을
    만들고 T_base_tcp = T_base_cam · X⁻¹ 로 역산 (plan_search_poses.py 계열의
    hand-eye 역변환 — 흉터 14 의 이식). 도달 판정은 resolve 몫.
    """
    z = z_axis / np.linalg.norm(z_axis)
    # 기준 x = 수평 ⟂ z (z 가 수직에 가까우면 world x 사용)
    horiz = np.cross(z, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(horiz) < 1e-6:
        x0 = np.array([1.0, 0.0, 0.0])
    else:
        x0 = horiz / np.linalg.norm(horiz)
    y0 = np.cross(z, x0)
    x_inv = np.linalg.inv(t_tcp_cam)
    groups: list[list[TcpPose]] = []
    metas: list[float] = []
    for psi_deg in psi_candidates_deg:
        psi = math.radians(psi_deg)
        x = math.cos(psi) * x0 + math.sin(psi) * y0
        y = np.cross(z, x)
        t_base_cam = np.eye(4)
        t_base_cam[:3, :3] = np.column_stack([x, y, z])
        t_base_cam[:3, 3] = c
        t_base_tcp = t_base_cam @ x_inv
        q = Rotation.from_matrix(t_base_tcp[:3, :3]).as_quat()
        groups.append(
            [
                TcpPose(
                    position=(
                        float(t_base_tcp[0, 3]),
                        float(t_base_tcp[1, 3]),
                        float(t_base_tcp[2, 3]),
                    ),
                    quaternion=(float(q[0]), float(q[1]),
                                float(q[2]), float(q[3])),
                )
            ]
        )
        metas.append(psi_deg)
    return groups, metas


@step(title="omx 관측 자세 계획")
async def plan_omx_observe(
    ctx: TaskContext,
    omx: str,
    roi_omx: WorkcellRoi,
    t_tcp_cam: np.ndarray,
    trace: HandoverTrace | None = None,
) -> list[float]:
    """nadir(수직하향) 카메라 @ 도달영역 centroid 위 — §8-1 계산 확정 포즈의
    런타임판 (table_z/hand_eye 를 실 설정에서 읽으므로 오프라인 수치 하드코딩
    안 함). roll ψ 격자 → 첫 도달 그룹 채택."""
    look_x = (roi_omx.x_min + roi_omx.x_max) / 2.0
    look_y = (roi_omx.y_min + roi_omx.y_max) / 2.0
    c = np.array([look_x, look_y, _OMX_TABLE_Z_M + _OMX_OBSERVE_CAM_H_M])
    groups, metas = _camera_pose_groups(
        c, np.array([0.0, 0.0, -1.0]), _OMX_OBSERVE_PSI_DEG, t_tcp_cam
    )
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=groups),
        ResolveReachableResponse,
        robot_id=omx,
    )
    await _emit(
        trace,
        {
            "phase": "observe",
            "event": "plan_omx_observe",
            "look": [look_x, look_y],
            "cam_h": _OMX_OBSERVE_CAM_H_M,
            "psi_candidates": list(metas),
            "index": res.index,
            "group_failures": res.group_failures,
        },
    )
    if res.index < 0:
        raise NoReachableGrasp(
            f"omx 관측 자세 후보 {len(groups)}개 전멸 — {res.message}. "
            "workcell ROI/카메라 높이(_OMX_OBSERVE_CAM_H_M) 조정 후 다시 실행하세요"
        )
    logger.info(
        "plan_omx_observe: ψ=%.0f° 채택, look=(%.3f,%.3f) h=%.2f",
        metas[res.index],
        look_x,
        look_y,
        _OMX_OBSERVE_CAM_H_M,
    )
    return res.solutions[0]


def _trusted_cube_candidates(
    cands: list[OrientedDetection],
) -> list[OrientedDetection]:
    """큐브 신뢰 게이트 — score + 기하(두 변 대역 + 종횡비 상한). mono 는 depth
    게이트가 없으므로 기하가 오검출 컷의 주력 (셀 밖 컷은 detector ROI 가 상류
    담당). footprint = (긴 변, 짧은 변)."""
    return [
        c
        for c in cands
        if c.score >= _SCORE_MIN
        and _CUBE_EDGE_MIN_M <= c.footprint[1]
        and c.footprint[0] <= _CUBE_EDGE_MAX_M
        and c.footprint[0] <= _CUBE_ASPECT_MAX * max(c.footprint[1], 1e-6)
    ]


@step(title="omx 관측·검출")
async def omx_observe_detect(
    ctx: TaskContext,
    omx: str,
    prompt: str,
    observe_joints: list[float],
    trace: HandoverTrace | None = None,
) -> OrientedDetection:
    """관측 자세 이동 → 정지 → DETECT_PLANAR (mono ray∩z=table). 신뢰 컷 후
    최고 score. 0건 = 명시 실패 (사유 + 다음 행동)."""
    await _move_j(ctx, omx, joints=observe_joints)
    await asyncio.sleep(_OBSERVE_SETTLE_S)
    res = await ctx.call(
        Detector.Service.DETECT_PLANAR,
        DetectPlanarRequest(
            robot_id=omx, plane_z=_OMX_TABLE_Z_M, prompts=[
                prompt], top_k=_TOP_K
        ),
        DetectOrientedResponse,
    )
    await _emit(
        trace,
        {
            "phase": "observe",
            "event": "detect_planar",
            "prompt": prompt,
            "plane_z": _OMX_TABLE_Z_M,
            "candidates": [
                {
                    "position": list(c.position),
                    "score": c.score,
                    "yaw_deg": round(math.degrees(c.grasp_yaw), 1),
                    "footprint_mm": [round(v * 1000) for v in c.footprint],
                    "points": len(c.points or []),
                }
                for c in res.candidates
            ],
        },
    )
    trusted = _trusted_cube_candidates(res.candidates)
    if not trusted:
        raise DetectionNotFound(
            prompt,
            candidates=len(res.candidates),
            reason=(
                f"신뢰 컷 미달 (score≥{_SCORE_MIN}, 변 "
                f"{_CUBE_EDGE_MIN_M * 1000:.0f}~{_CUBE_EDGE_MAX_M * 1000:.0f}mm, "
                f"종횡비≤{_CUBE_ASPECT_MAX}) — 큐브 배치/조명/"
                f"table_z({_OMX_TABLE_Z_M}) 확인 후 다시 실행하세요"
            ),
        )
    best = max(trusted, key=lambda c: c.score)
    logger.info(
        "omx_observe_detect: '%s' 채택 — center=(%.3f,%.3f) yaw=%.1f° "
        "len=%.0fmm w=%.0fmm score=%.2f",
        prompt,
        best.position[0],
        best.position[1],
        math.degrees(best.grasp_yaw),
        best.footprint[0] * 1000,
        best.footprint[1] * 1000,
        best.score,
    )
    return best


# ─── B. omx 파지 계획 (top-down + J5 roll) ────────────────────────────


def plan_cube_grasp_from(det: OrientedDetection, base_omx: BasePose) -> CubeGrasp:
    """검출 → 큐브 파지 기하 (omx frame). 순수 계산 — step 아님 (모션 0).
    큐브는 대칭이라 짧음 실패가 없다 (펜과 달리 항상 파지 가능)."""
    return cube.plan_cube_grasp(
        (det.position[0], det.position[1]),
        det.grasp_yaw,
        (det.footprint[0], det.footprint[1]),
        yaw_offsets_deg=_CUBE_PICK_YAW_OFFSETS_DEG,
        size_min_m=_CUBE_SIZE_MIN_M,
        size_max_m=_CUBE_SIZE_MAX_M,
    )


@dataclass(frozen=True, slots=True)
class CubePick:
    """omx 큐브 파지 계획 산출 — 실행(집기)과 제시(present)가 공유.

    jaw_yaw_omx: 채택 조 축 방위 (omx frame, rad) — 어느 두 면을 물었나. 제시/
    수취의 "직교 면" 판정 기준(단, 수취는 omx 의 **현재 TCP quat** 에서 실측하는
    게 정본 — 이 값은 로깅/계획 추적용)."""

    sols: list[list[float]]  # [grasp] 관절해 (단일 — top-down pre/lift 폐기)
    quat: Quat
    grasp_omx: Vec3  # 파지점 (omx frame — z = table + 파지 dz)
    jaw_yaw_omx: float
    size_m: float
    chosen_dz: float  # 채택 파지 Z (table 위, 실물 보정 데이터)


@step(title="omx 집기 계획")
async def plan_omx_pick_cube(
    ctx: TaskContext,
    omx: str,
    grasp: CubeGrasp,
    trace: HandoverTrace | None = None,
) -> CubePick:
    """큐브 중심 파지점(책상면 top-down)만 resolve. top-down 은 관절 리밋상 책상
    근처에서만 도달한다(§5.1 은 DOF 수 얘기 — 실측 도달 밴드는 z≈table). 위에서
    수직 접근하는 pre / 수직 lift 를 top-down 으로 강제하면 IK 전멸이라 폐기했다:
    접근은 관측 자세에서 파지 해로 move_j 스윙인, 리프트는 제시 단계가 도달 가능한
    자세로 수행 (omx=best-effort, 정밀은 so101).

    조 축 후보(면 정렬 우선) × Z 사다리(낮은→높은, 첫 도달 채택). omx 는 depth
    가 없어 높이를 못 재므로 파지 Z 는 큐브 크기 가정(2cm → 중심 바닥+1cm)이
    앵커 — 사다리는 도달/바닥클리어를 위한 소폭 탐색(chosen_dz 로 실물 보정)."""
    gx, gy = grasp.center_xy
    z_ladder = [_OMX_TABLE_Z_M + dz for dz in _CUBE_PICK_DZ_LADDER]
    groups: list[list[TcpPose]] = []
    metas: list[tuple[Quat, float, float]] = []  # (quat, jaw_yaw, gz)
    for yaw in grasp.yaw_candidates:  # 면 정렬(0/90) 우선, 대각 폴백
        quat = _grasp_quat(yaw, 0)
        for gz in z_ladder:  # 선호 dz(1cm) 우선
            groups.append([TcpPose(position=(gx, gy, gz), quaternion=quat)])
            metas.append((quat, yaw, gz))
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=groups, floor_z=_OMX_TABLE_Z_M + _CUBE_PICK_FLOOR_CLEAR_M
        ),
        ResolveReachableResponse,
        robot_id=omx,
    )
    await _emit(
        trace,
        {
            "phase": "pick",
            "event": "plan_omx_pick_cube",
            "grasp_xy": [gx, gy],
            "z_ladder": z_ladder,
            "size_m": grasp.size_m,
            "yaw_candidates_deg": [round(math.degrees(y), 1) for y in grasp.yaw_candidates],
            "index": res.index,
            "chosen_dz": (metas[res.index][2] - _OMX_TABLE_Z_M) if res.index >= 0 else None,
            "chosen_jaw_deg": (
                round(math.degrees(metas[res.index][1]),
                      1) if res.index >= 0 else None
            ),
            "group_failures": res.group_failures,
        },
    )
    if res.index < 0:
        raise NoReachableGrasp(
            f"omx top-down 큐브 파지 후보 {len(groups)}개 전멸 — {res.message} "
            f"(그룹별: {res.group_failures}). 큐브를 omx 도달영역 중심 쪽으로 "
            "옮긴 후 다시 실행하세요"
        )
    quat, jaw_yaw, g_z = metas[res.index]
    g = (gx, gy, g_z)
    dz = g_z - _OMX_TABLE_Z_M
    logger.info(
        "plan_omx_pick_cube: 조축=%.0f° 채택 — grasp(omx)=(%.3f,%.3f,%.3f) "
        "파지dz=%.0fmm 큐브=%.0fmm",
        math.degrees(jaw_yaw),
        g[0],
        g[1],
        g[2],
        dz * 1000,
        grasp.size_m * 1000,
    )
    return CubePick(
        sols=res.solutions,
        quat=quat,
        grasp_omx=g,
        jaw_yaw_omx=jaw_yaw,
        size_m=grasp.size_m,
        chosen_dz=dz,
    )


# ─── C. omx 집기 (move_j 스윙인 — look-then-move 폐기) ─────────────────


@step(title="omx 집기")
async def omx_pick_cube(
    ctx: TaskContext,
    omx: str,
    plan: CubePick,
    trace: HandoverTrace | None = None,
) -> None:
    """관측 자세 → 파지 해로 바로 move_j(속도 cap 으로 부드럽게) → close → 판정.
    top-down 은 책상면에서만 도달하므로 위에서의 수직 접근(pre)·수직 lift 는 불가 —
    스윙인으로 간다. 리프트/제시는 다음 단계(plan_omx_present)가 도달 가능한 자세로
    수행. refine(look-then-move) 폐기 — omx=best-effort, 정밀은 so101 수취가 흡수."""
    await _move_j(ctx, omx, joints=plan.sols[0])
    await set_gripper(ctx, omx, open_=False)
    await verify_grasp(ctx, omx, phase="omx close 직후", trace=trace)


# ─── D. omx 제시 (랑데부 계산 — 티칭 폐기) ────────────────────────────


@dataclass(frozen=True, slots=True)
class PresentPlan:
    sols: list[list[float]]  # [제시 자세] 관절해
    quat: Quat
    h_world: Vec3  # 큐브 중심 목표 (world) — so101 재검출 겨냥점 (≈ 제시 TCP)


def _present_orientations() -> list[tuple[str, Quat]]:
    """제시 자세 후보 (선호순) — (라벨, quat(omx frame)). 일반 grasp 샘플러
    (_PRESENT_FAMILY) 그대로 — 큐브 대칭이라 임의 tool 방위가 유효, 도달성/충돌이
    채택. (tcp 위치 무관 — 방위만 열거.)"""
    return [(label, q) for label, q, _a in _PRESENT_FAMILY]


@step(title="제시 계획")
async def plan_omx_present(
    ctx: TaskContext,
    omx: str,
    roi_so: WorkcellRoi,
    roi_omx: WorkcellRoi,
    base_omx: BasePose,
    so101_joints: list[float],
    checker: CrossRobotChecker | None,
    trace: HandoverTrace | None = None,
) -> PresentPlan:
    """랑데부 후보(workcell ROI 교집합, 흉터 5 예방)를 **TCP 위치**로 순회 —
    각 점에서 자세족(_present_orientations)을 resolve (그룹 순서 = 선호) 하고
    채택안을 cross-robot 충돌 게이트. 첫 통과 채택, 전멸 = 명시 실패.

    H(so101 재검출 겨냥점) = 제시 TCP = 큐브 중심 (omx 가 중심을 물었으므로 큐브가
    TCP 에 매달림. so101 은 어차피 재검출로 실 위치를 잡으니 겨냥점은 근사면 충분).
    큐브 조 축(so101 직교 기준)은 수취 시 omx 현재 TCP quat 에서 실측 (여기서 안
    실음 — 계획 자세와 실 자세가 같아 동치이나 실측이 정본)."""
    cands = frames.rendezvous_candidates(
        roi_so,
        roi_omx,
        base_omx,
        _PRESENT_Z_WORLD,
        limit=_PRESENT_LIMIT,
        prefer_point=_RENDEZVOUS_PREFER_XY,  # so101 도달 robust 지점 직접 겨냥
    )
    if not cands:
        raise TaskError(
            "두 팔 공통 워크스페이스(workcell ROI 교집합)가 비어 있음 — "
            "instance.yaml workcell 값/_PRESENT_Z_WORLD 를 확인하세요"
        )
    rejects: list[str] = []
    orients = _present_orientations()
    omx_tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=omx
    )
    for tcp_w in cands:
        tcp_omx = world_to_robot(tcp_w, base_omx)
        groups = [[TcpPose(position=tcp_omx, quaternion=q)]
                  for _l, q in orients]
        alive = list(range(len(groups)))
        for _attempt in range(_RECV_COLLISION_RETRY):
            res = await ctx.call(
                Motion.Service.RESOLVE_REACHABLE,
                ResolveReachableRequest(groups=[groups[i] for i in alive]),
                ResolveReachableResponse,
                robot_id=omx,
            )
            if res.index < 0:
                rejects.append(f"tcp={tcp_w}: 자세족 전멸 ({res.message})")
                break
            gi = alive[res.index]
            label, quat = orients[gi]
            # 벽(뒤) 침범 — omx 링크가 베이스 뒤로 넘어가면 기각 (side="b")
            if checker is not None and _behind_wall(checker, "b", res.solutions[0]):
                rejects.append(f"tcp={tcp_w}/{label}: omx 벽(뒤) 침범")
                alive.remove(gi)
                if not alive:
                    break
                continue
            if checker is not None and _omx_path_collides(
                checker,
                so101_joints,
                [list(omx_tcp.joints), res.solutions[0]],
            ):
                rejects.append(f"tcp={tcp_w}/{label}: so101 충돌 위험")
                alive.remove(gi)
                if not alive:
                    break
                continue
            h_world = (tcp_w[0], tcp_w[1], tcp_w[2])
            await _emit(
                trace,
                {
                    "phase": "present",
                    "event": "plan_omx_present",
                    "tcp_world": list(tcp_w),
                    "orientation": label,
                    "h_world": list(h_world),
                    "rejects": rejects,
                },
            )
            logger.info(
                "plan_omx_present: tcp=(%.3f,%.3f,%.3f) %s 채택 (기각 %d) — "
                "H=(%.3f,%.3f,%.3f)",
                tcp_w[0],
                tcp_w[1],
                tcp_w[2],
                label,
                len(rejects),
                h_world[0],
                h_world[1],
                h_world[2],
            )
            return PresentPlan(sols=res.solutions, quat=quat, h_world=h_world)
    await _emit(
        trace,
        {
            "phase": "present",
            "event": "plan_omx_present_exhausted",
            "rejects": rejects,
        },
    )
    raise NoReachableGrasp(
        f"제시 후보 {len(cands)}개 전멸 — {rejects}. workcell 교집합/제시 높이"
        "(_PRESENT_Z_WORLD) 조정 후 다시 실행하세요"
    )


def _behind_wall(
    checker: CrossRobotChecker, side: str, joints: list[float]
) -> bool:
    """robot(side='a'=so101 / 'b'=omx) 링크가 벽(뒤, world x < _WALL_MIN_X_M)로
    넘어가는 IK 해인가 (사용자: 로봇 뒤는 벽). resolve 는 벽을 모르므로 채택
    해를 여기서 후처리 기각 (충돌 게이트 alive-loop 과 동형)."""
    grip = 1.0 if side == "a" else _OMX_HOLD_GRIP_FRAC
    return checker.min_link_world_x(side, joints, grip=grip) < _WALL_MIN_X_M


def _omx_path_collides(
    checker: CrossRobotChecker,
    so101_joints: list[float],
    omx_path: list[list[float]],
) -> bool:
    """omx 관절 경로 vs so101 고정 구성 — checker 는 (a=so101, b=omx) 로 생성돼
    path_in_collision 이 a 경로만 받으므로 b 경로는 표본을 직접 돈다."""
    prev = omx_path[0]
    if checker.in_collision(so101_joints, prev, grip_b=_OMX_HOLD_GRIP_FRAC):
        return True
    for nxt in omx_path[1:]:
        qa, qb = np.asarray(prev, float), np.asarray(nxt, float)
        n = max(1, int(math.ceil(float(np.max(np.abs(qb - qa))) / math.radians(6.0))))
        for k in range(1, n + 1):
            q = [float(v) for v in qa + (qb - qa) * (k / n)]
            if checker.in_collision(so101_joints, q, grip_b=_OMX_HOLD_GRIP_FRAC):
                return True
        prev = nxt
    return False


@step(title="omx 내밀기")
async def omx_present(
    ctx: TaskContext,
    omx: str,
    present: PresentPlan,
    trace: HandoverTrace | None = None,
) -> None:
    """물체를 든 채 계산된 제시 자세로 (관절해 그대로) + held 재확인."""
    logger.info(
        "omx_present → H_world=(%.3f,%.3f,%.3f)",
        present.h_world[0],
        present.h_world[1],
        present.h_world[2],
    )
    await _move_j(ctx, omx, joints=present.sols[0])
    await verify_grasp(ctx, omx, phase="제시 자세 도달", trace=trace)


# ─── E. so101 수취 (재검출 + refine — FK 짐작 폐기) ───────────────────


@step(title="수취 관측 자세")
async def plan_so_observe(
    ctx: TaskContext,
    so101: str,
    t_tcp_cam: np.ndarray,
    h_world: Vec3,
    trace: HandoverTrace | None = None,
) -> list[float]:
    """제시점 H 를 D405 검증 대역 거리에서 내려다보는 카메라 pose 역산 —
    (방위 오프셋 × 고도 × 거리 × roll ψ) 사다리 resolve (so101 공중 도달이
    좁아 단일 기하는 전멸 실측 — 노브 블록 주석). FK/계획값은 **관측
    유도용으로만** — 파지는 재검출."""
    az0 = math.atan2(h_world[1], h_world[0])
    groups: list[list[TcpPose]] = []
    metas: list[tuple[float, float, float, float]] = []
    for az_off in _RECV_OBS_AZOFF_DEG:
        for elev_deg in _RECV_OBS_ELEV_DEG:
            for dist in _RECV_OBS_DIST_M:
                az = az0 + math.radians(az_off)
                elev = math.radians(elev_deg)
                c = np.array(
                    [
                        h_world[0] - math.cos(az) * dist * math.cos(elev),
                        h_world[1] - math.sin(az) * dist * math.cos(elev),
                        h_world[2] + dist * math.sin(elev),
                    ]
                )
                g, m = _camera_pose_groups(
                    c,
                    np.asarray(h_world, dtype=float) - c,
                    _RECV_OBS_PSI_DEG,
                    t_tcp_cam,
                )
                groups.extend(g)
                metas.extend((az_off, elev_deg, dist, psi) for psi in m)
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=groups),
        ResolveReachableResponse,
        robot_id=so101,
    )
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "plan_so_observe",
            "h_world": list(h_world),
            "index": res.index,
            "meta": metas[res.index] if res.index >= 0 else None,
            "n_groups": len(groups),
        },
    )
    if res.index < 0:
        raise NoReachableGrasp(
            f"so101 수취 관측 자세 전멸 ({len(groups)}개) — {res.message}. "
            "제시 높이/거리 노브(_PRESENT_Z_WORLD/_RECV_OBS_*) 조정 후 다시 "
            "실행하세요"
        )
    logger.info(
        "plan_so_observe: az_off=%.0f° elev=%.0f° dist=%.2f ψ=%.0f° 채택",
        *metas[res.index],
    )
    return res.solutions[0]


def _match_aerial(
    cands: list[OrientedDetection], h_world: Vec3
) -> OrientedDetection | None:
    """공중 펜 매치 — 제시 계획점 반경 + 공중 z 대역 + score/점군 게이트.
    (테이블 대역 게이트의 개방판 — base_z 가 아니라 position z 로 판정: 공중
    물체의 base_z 는 '보이는 band 하단'이라 물리 바닥이 아니다.)"""
    trusted = [
        c
        for c in cands
        if c.score >= _RECV_SCORE_MIN
        and len(c.points or []) >= _RECV_MIN_POINTS
        and abs(c.position[2] - h_world[2]) <= _RECV_Z_BAND_M
        and math.hypot(c.position[0] - h_world[0], c.position[1] - h_world[1])
        <= _RECV_MATCH_RADIUS_M
    ]
    return max(trusted, key=lambda c: c.score) if trusted else None


@step(title="수취 재검출")
async def so_redetect(
    ctx: TaskContext,
    so101: str,
    prompt: str,
    observe_joints: list[float],
    h_world: Vec3,
    trace: HandoverTrace | None = None,
) -> OrientedDetection:
    """관측 자세 이동 → 공중의 제시된 펜 재검출. 실패 = 명시 실패 (FK 로
    후퇴하지 않는다 — §8-4: 정적 계산 ~1–2cm 자세의존 오차가 so101 이
    closed-loop 로 간 이유 그 자체)."""
    await _move_j(ctx, so101, joints=observe_joints)
    await asyncio.sleep(_OBSERVE_SETTLE_S)
    res = await ctx.call(
        Detector.Service.DETECT_ORIENTED,
        DetectRequest(robot_id=so101, prompts=[prompt], top_k=_TOP_K),
        DetectOrientedResponse,
    )
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "so_redetect",
            "h_world": list(h_world),
            "candidates": [
                {
                    "position": list(c.position),
                    "score": c.score,
                    "points": len(c.points or []),
                    "yaw_deg": round(math.degrees(c.grasp_yaw), 1),
                }
                for c in res.candidates
            ],
        },
    )
    best = _match_aerial(res.candidates, h_world)
    if best is None:
        raise DetectionNotFound(
            prompt,
            candidates=len(res.candidates),
            reason=(
                f"공중 재검출 매치 실패 (계획점 {h_world} 반경 "
                f"{_RECV_MATCH_RADIUS_M * 1000:.0f}mm · z±"
                f"{_RECV_Z_BAND_M * 1000:.0f}mm · score≥{_RECV_SCORE_MIN} · "
                f"점군≥{_RECV_MIN_POINTS}) — 제시 자세/조명 확인 후 다시 "
                "실행하세요 (공중 큐브 검출은 실물 미검증 — 가림 주의)"
            ),
        )
    return best


@dataclass(frozen=True, slots=True)
class ReceivePlan:
    sols: list[list[float]]  # [pre, grasp] 관절해
    quat: Quat
    target: Vec3  # 파지 겨냥점 (world) = 큐브 중심
    omx_joints: list[float]


@step(title="수취 계획")
async def plan_receive(
    ctx: TaskContext,
    so101: str,
    omx: str,
    det: OrientedDetection,
    base_omx: BasePose,
    checker: CrossRobotChecker | None,
    trace: HandoverTrace | None = None,
) -> ReceivePlan:
    """재검출 기반 수취 계획 — **일반 grasp 자세족**(_RECV_FAMILY, 수직 조축=위/아래
    면 우선) resolve + **충돌 게이트** + **벽(뒤) 게이트**. 채택 그룹이 충돌/벽이면
    빼고 재-resolve (상한 소진 = 명시 실패).

    겨냥점 = 큐브 중심(재검출 position). so101 은 omx 가 문 옆면이 아닌 **위/아래
    면을 수직 조축으로** 받는다 (스윕 2026-07-26: 도달·비관통 해가 전부 수직 조축).
    "다른 면"의 물리적 보증은 충돌 게이트(두 그리퍼 여유)가 담당."""
    omx_tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=omx
    )
    omx_joints = list(omx_tcp.joints)
    target = (det.position[0], det.position[1], det.position[2])
    groups: list[list[TcpPose]] = []
    metas: list[Quat] = []
    for _label, quat, a in _RECV_FAMILY:  # 수직 조축(위/아래면) 우선 순서
        pre = (
            target[0] - a[0] * _RECV_PRE_CLEAR_M,
            target[1] - a[1] * _RECV_PRE_CLEAR_M,
            target[2] - a[2] * _RECV_PRE_CLEAR_M,
        )
        groups.append(
            [
                TcpPose(position=pre, quaternion=quat),
                TcpPose(position=target, quaternion=quat),
            ]
        )
        metas.append(quat)
    alive = list(range(len(groups)))
    for attempt in range(_RECV_COLLISION_RETRY):
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(groups=[groups[i]
                                    for i in alive], linear=True),
            ResolveReachableResponse,
            robot_id=so101,
        )
        if res.index < 0:
            raise NoReachableGrasp(
                f"수취 접근 후보 전멸 ({len(alive)}개) — {res.message}. "
                "제시 높이(_PRESENT_Z_WORLD)를 조정 후 다시 실행하세요"
            )
        gi = alive[res.index]
        # 벽(뒤) — so101 grasp 해가 베이스 뒤로 넘어가면 기각 (side="a")
        if checker is not None and _behind_wall(checker, "a", res.solutions[-1]):
            logger.warning("plan_receive: 그룹 %d 채택안 so101 벽(뒤) 침범 — 제외", gi)
            alive.remove(gi)
            if not alive:
                break
            continue
        if checker is None or not checker.path_in_collision(
            res.solutions,
            omx_joints,
            grip_a=1.0,  # 접근은 조를 벌린 채 (실 충돌 형상)
            grip_b=_OMX_HOLD_GRIP_FRAC,
            margin_m=_RECV_COLLISION_MARGIN_M,
        ):
            await _emit(
                trace,
                {
                    "phase": "receive",
                    "event": "plan_receive",
                    "target": list(target),
                    "group": gi,
                    "orientation": _RECV_FAMILY[gi][0],
                    "attempt": attempt,
                },
            )
            return ReceivePlan(
                sols=res.solutions,
                quat=metas[gi],
                target=target,
                omx_joints=omx_joints,
            )
        logger.warning(
            "plan_receive: 그룹 %d 채택안이 omx 와 충돌 위험 (margin %.0fmm) — "
            "제외 후 재시도 %d/%d",
            gi,
            _RECV_COLLISION_MARGIN_M * 1000,
            attempt + 1,
            _RECV_COLLISION_RETRY,
        )
        alive.remove(gi)
        if not alive:
            break
    raise NoReachableGrasp(
        "수취 접근 전부 omx 와 충돌/벽 위험 — 제시 자세를 두 로봇이 더 벌어지게 "
        "조정(_PRESENT_Z_WORLD/랑데부)한 후 다시 실행하세요"
    )


@step(title="수취 보정")
async def so_refine(
    ctx: TaskContext,
    so101: str,
    prompt: str,
    plan: ReceivePlan,
    trace: HandoverTrace | None = None,
) -> Vec3:
    """pre 도달 후 재검출 1 tick — 겨냥점 갱신 (look-then-move 최소형: 측정
    자세와 실행 자세가 가까워 common-mode 상쇄). 실패 = 계획 겨냥점 유지
    (로그+trace — 침묵 금지)."""
    await asyncio.sleep(_OBSERVE_SETTLE_S)
    res = await ctx.call(
        Detector.Service.DETECT_ORIENTED,
        DetectRequest(robot_id=so101, prompts=[prompt], top_k=_TOP_K),
        DetectOrientedResponse,
    )
    best = _match_aerial(res.candidates, plan.target)
    if best is None:
        reason = "수취 refine 재검출 실패 — 계획 겨냥점으로 진행"
        logger.warning("so_refine: %s", reason)
        await _emit(
            trace, {"phase": "receive",
                    "event": "refine_miss", "reason": reason}
        )
        return plan.target
    updated = (best.position[0], best.position[1], best.position[2])  # 큐브 중심
    jump = math.dist(updated, plan.target)
    if jump > _REFINE_JUMP_MAX_M:
        reason = (
            f"수취 refine 도약 {jump * 1000:.0f}mm > "
            f"{_REFINE_JUMP_MAX_M * 1000:.0f}mm — 계획 겨냥점 유지"
        )
        logger.warning("so_refine: %s", reason)
        await _emit(
            trace, {"phase": "receive",
                    "event": "refine_rejected", "reason": reason}
        )
        return plan.target
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "refine",
            "target": list(updated),
            "jump_mm": round(jump * 1000, 1),
        },
    )
    return updated


@step(title="수취")
async def receive(
    ctx: TaskContext,
    so101: str,
    omx: str,
    plan: ReceivePlan,
    prompt: str,
    trace: HandoverTrace | None = None,
) -> None:
    """so101 접근(pre 관절해) → refine 1 tick → 진입(감속) → close →
    **held 확인 후에만** omx open → so101 이탈(감속).

    수취 순서 불변식 (모듈 docstring): so101 판정 전 omx 를 열면 물체 낙하 —
    회귀 테스트가 호출 순서를 잠근다."""
    await _move_j(ctx, so101, joints=plan.sols[0])
    target = await so_refine(ctx, so101, prompt, plan, trace)
    await _move_l(
        ctx,
        so101,
        position=target,
        quaternion=plan.quat,
        speed_scale=_GENTLE_SPEED_SCALE,
    )
    await set_gripper(ctx, so101, open_=False)
    await verify_grasp(ctx, so101, phase="수취 close 직후", trace=trace)
    # so101 확보 확인 완료 — 이제 giver 가 놓는다
    await set_gripper(ctx, omx, open_=True)
    a = _approach_of_quat(plan.quat)
    withdraw = (
        target[0] - a[0] * _RECV_WITHDRAW_M,
        target[1] - a[1] * _RECV_WITHDRAW_M,
        target[2] - a[2] * _RECV_WITHDRAW_M,
    )
    await _move_l(
        ctx,
        so101,
        position=withdraw,
        quaternion=plan.quat,
        speed_scale=_GENTLE_SPEED_SCALE,
    )
    await verify_grasp(ctx, so101, phase="수취 이탈 후", trace=trace)


@step(title="omx 복귀")
async def omx_retreat(
    ctx: TaskContext,
    omx: str,
    so101: str,
    home_omx: WaypointRecord,
    checker: CrossRobotChecker | None,
) -> None:
    """omx home 복귀 — 복귀 관절 경로를 so101 현재 구성과 충돌 검사. 충돌
    위험이면 **정지 유지 + 명시 실패** (so101 이 물체를 들고 있으므로 omx 가
    멈추는 쪽이 안전). so101 은 파지 상태(닫힘)라 grip_a 를 좁혀 실 형상으로."""
    if checker is not None:
        so_tcp = await ctx.call(
            Motion.Service.TCP_SNAPSHOT,
            TcpSnapshotRequest(),
            TcpState,
            robot_id=so101,
        )
        omx_tcp = await ctx.call(
            Motion.Service.TCP_SNAPSHOT,
            TcpSnapshotRequest(),
            TcpState,
            robot_id=omx,
        )
        if _so_static_path_collides(
            checker,
            list(so_tcp.joints),
            [list(omx_tcp.joints), list(home_omx.joint_values)],
        ):
            raise TaskError(
                "omx 복귀 경로가 so101 과 충돌 위험 — omx 정지 유지. so101 을 "
                "먼저 적치/이탈시킨 뒤 omx 를 수동 복귀하세요"
            )
    await _move_j(ctx, omx, joints=home_omx.joint_values)


def _so_static_path_collides(
    checker: CrossRobotChecker,
    so101_joints: list[float],
    omx_path: list[list[float]],
) -> bool:
    """omx 복귀 경로 vs so101(파지 중 = 조 닫힘) — b 경로 표본 검사."""
    prev = omx_path[0]
    if checker.in_collision(so101_joints, prev, grip_a=0.2, grip_b=1.0):
        return True
    for nxt in omx_path[1:]:
        qa, qb = np.asarray(prev, float), np.asarray(nxt, float)
        n = max(1, int(math.ceil(float(np.max(np.abs(qb - qa))) / math.radians(6.0))))
        for k in range(1, n + 1):
            q = [float(v) for v in qa + (qb - qa) * (k / n)]
            if checker.in_collision(so101_joints, q, grip_a=0.2, grip_b=1.0):
                return True
        prev = nxt
    return False


def _approach_of_quat(quat: Quat) -> Vec3:
    a = Rotation.from_quat(quat).apply([1.0, 0.0, 0.0])
    return (float(a[0]), float(a[1]), float(a[2]))


# ─── 적치 (pick_and_place 슬림판 — open-loop, v1 유지) ────────────────


@step(title="검출")
async def detect(ctx: TaskContext, so101: str, prompt: str) -> list[OrientedDetection]:
    """search 그룹 자세 전부 순회 → 후보 누적 (so101 카메라 — 적치 spot 검출)."""
    members = await ctx.call(
        Waypoint.Service.LIST_GROUP_MEMBERS_BY_NAME,
        ListGroupMembersByNameRequest(robot_id=so101, name=_SEARCH_GROUP),
        ListGroupMembersByNameResponse,
    )
    if not members.found:
        raise TaskError(
            f"'{_SEARCH_GROUP}' waypoint 그룹 없음 (robot={so101}) — 검색 자세를 "
            "티칭해 그룹으로 저장한 뒤 다시 실행하세요"
        )
    if not members.waypoints:
        raise TaskError(f"'{_SEARCH_GROUP}' 그룹이 비어있음 (robot={so101})")
    t0 = time.monotonic()
    cands: list[OrientedDetection] = []
    for wp in members.waypoints:
        await _move_j(ctx, so101, joints=wp.joint_values)
        await asyncio.sleep(_SEARCH_SETTLE_S)
        res = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=so101, prompts=[prompt], top_k=_TOP_K),
            DetectOrientedResponse,
        )
        cands.extend(res.candidates)
    logger.info(
        "detect(%s): %d 자세 → 후보 %d (%.1fs)",
        prompt,
        len(members.waypoints),
        len(cands),
        time.monotonic() - t0,
    )
    return cands


@step(title="적치")
async def place_into(
    ctx: TaskContext,
    so101: str,
    prompt: str,
    held_height_m: float,
    home_so: WaypointRecord,
) -> None:
    """상자 검출 → [pre, place] resolve → 접근/삽입/release/후퇴.

    pick_and_place plan_place 의 슬림판 (정렬 4 yaw × tilt 5 — 폴백 자유 yaw
    가족은 생략, 필요해지면 그대로 이식). 후퇴는 pre 관절해 MoveJ (07-17
    retreat 실행 IK 실사고 회피 — 계획 해 재사용)."""
    spots = await detect(ctx, so101, prompt)
    ranked = sorted(
        (s for s in spots if -0.04 <= s.base_z <= _BASE_Z_MAX_M),
        key=lambda s: s.score,
        reverse=True,
    )
    if not ranked:
        raise TaskError(
            f"'{prompt}' 적치 대상 검출 0건 (타당 대역) — 상자 배치 확인 후 "
            "다시 실행하세요"
        )
    for spot in ranked:
        place_z = spot.position[2] + held_height_m * 0.5 + _PLACE_DROP_CLEAR_M
        groups: list[list[TcpPose]] = []
        metas: list[tuple[Quat, Vec3, Vec3]] = []
        for tilt in _PLACE_TILTS_DEG:
            for off in _PLACE_YAW_OFFSETS_DEG:
                yaw = spot.grasp_yaw + math.radians(off)
                quat = _grasp_quat(yaw, tilt)
                a = _approach_of(yaw, tilt)
                place = (spot.position[0], spot.position[1], place_z)
                pre = (
                    place[0] - a[0] * _PLACE_PRE_CLEAR_M,
                    place[1] - a[1] * _PLACE_PRE_CLEAR_M,
                    place[2] - a[2] * _PLACE_PRE_CLEAR_M,
                )
                groups.append(
                    [
                        TcpPose(position=pre, quaternion=quat),
                        TcpPose(position=place, quaternion=quat),
                    ]
                )
                metas.append((quat, place, pre))
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=groups, floor_z=spot.base_z - 0.005, linear=True
            ),
            ResolveReachableResponse,
            robot_id=so101,
        )
        if res.index < 0:
            logger.info(
                "place_into: spot score=%.2f 전멸 — 다음 spot (%s)",
                spot.score,
                res.message,
            )
            continue
        quat, place, _pre = metas[res.index]
        await _move_j(ctx, so101, joints=res.solutions[0])
        await _move_l(ctx, so101, position=place, quaternion=quat)
        await verify_grasp(ctx, so101, phase="적치 직전")
        await set_gripper(ctx, so101, open_=True)
        try:
            await _move_l(ctx, so101, position=_pre, quaternion=quat)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            logger.warning("place 후퇴 MoveL 실패 (%s) — pre 관절해 MoveJ 폴백", e)
            await _move_j(ctx, so101, joints=res.solutions[0])
        await _move_j(ctx, so101, joints=home_so.joint_values)
        return
    raise NoReachableGrasp(
        f"적치 spot {len(ranked)}건 전부 도달 불가 — 상자를 so101 쪽으로 "
        "옮긴 뒤 다시 실행하세요"
    )


# ─── 공용 primitive (pick_and_place 계승 — 계약 동일) ────────────────


@step(title="home 경유")
async def go_home(ctx: TaskContext, robot_id: str, home: WaypointRecord) -> None:
    logger.info("go_home robot=%s → '%s'", robot_id, home.name)
    await _move_j(ctx, robot_id, joints=home.joint_values)


@step(title="토크 on")
async def enable_torque(ctx: TaskContext, robot_id: str) -> None:
    """참여 로봇 토크 enable — 모션 전 필수. Dynamixel(omx)은 전원 on 시 torque
    off 가 기본이라(dynamixel.py open() 은 안 켬) 안 켜면 MoveJ 를 받아도 limp,
    엔코더 FK 로 mono 투영까지 어긋난다 (2026-07-24 실물 확인). Feetech(so101)는
    기본 on 이라 재확인일 뿐 무해. 프론트 토크 토글 없이 헤드리스 실행하는
    task 는 자기 참여 robot 을 스스로 enable 해야 self-contained."""
    logger.info("enable_torque robot=%s", robot_id)
    await ctx.call(
        Motor.Service.SET_TORQUE,
        SetTorqueRequest(enabled=True),
        SetTorqueResponse,
        robot_id=robot_id,
    )


@step(title="그리퍼")
async def set_gripper(ctx: TaskContext, robot_id: str, *, open_: bool) -> None:
    spec = ctx.spec(robot_id)
    raw = spec.gripper_open_raw if open_ else spec.gripper_close_raw
    logger.info(
        "gripper robot=%s → %s (raw=%d)",
        robot_id,
        "OPEN" if open_ else "CLOSE",
        raw,
    )
    await ctx.call(
        Motor.Service.SET_GRIPPER,
        SetGripperRequest(position_raw=raw),
        SetGripperResponse,
        robot_id=robot_id,
    )
    await asyncio.sleep(_GRIPPER_SETTLE_S)


@step(title="파지 확인")
async def verify_grasp(
    ctx: TaskContext,
    robot_id: str,
    *,
    phase: str,
    trace: HandoverTrace | None = None,
) -> None:
    """gap OR load 판정 (pick_and_place _gripper_holding 동일 규약) — 미달이면
    GraspFailed. 판정 근거 전부 로깅+trace (실물 임계 튜닝 데이터 — 특히 omx
    Dynamixel load 스케일은 미검증 §5.4, 이 원값이 재특성화의 1차 소스)."""
    spec = ctx.spec(robot_id)
    state = await ctx.call(
        Motor.Service.READ_STATE, ReadStateRequest(), JointState, robot_id=robot_id
    )
    gi = spec.gripper_index
    achieved = state.positions_raw[gi]
    load = (
        state.loads_raw[gi]
        if state.loads_raw is not None and gi < len(state.loads_raw)
        else None
    )
    margin = abs(spec.gripper_held_threshold_raw - spec.gripper_close_raw)
    gap = abs(achieved - spec.gripper_close_raw)
    held = gap > margin or (load is not None and load >= _HELD_LOAD_MIN_RAW)
    logger.info(
        "verify_grasp[%s] robot=%s achieved=%d (close=%d thr=%d load=%s) → %s",
        phase,
        robot_id,
        achieved,
        spec.gripper_close_raw,
        spec.gripper_held_threshold_raw,
        load,
        "HELD" if held else "EMPTY",
    )
    await _emit(
        trace,
        {
            "phase": "grasp",
            "event": "verify_grasp",
            "robot_id": robot_id,
            "grasp_phase": phase,
            "achieved_raw": achieved,
            "close_raw": spec.gripper_close_raw,
            "gap": gap,
            "load_raw": load,
            "held": held,
        },
    )
    if not held:
        raise GraspFailed(
            phase=phase,
            achieved_raw=achieved,
            close_raw=spec.gripper_close_raw,
            load_raw=load,
        )


async def _move_j(ctx: TaskContext, robot_id: str, *, joints: list[float]) -> None:
    await ctx.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=JointTarget(kind="joint", joints=joints)),
        MoveJResponse,
        robot_id=robot_id,
    )


async def _move_l(
    ctx: TaskContext,
    robot_id: str,
    *,
    position: Vec3,
    quaternion: Quat | None = None,
    speed_scale: float = 1.0,
) -> None:
    await ctx.call(
        Motion.Service.MOVE_L,
        MoveLRequest(
            target=PoseTarget(kind="pose", position=position,
                              quaternion=quaternion),
            speed_scale=speed_scale,
        ),
        MoveLResponse,
        robot_id=robot_id,
    )


# steps 표면에 frame 변환 재노출 (소비자/테스트 호환 — 정의는 frames.py)
__all__ = [
    "world_to_robot",
    "robot_to_world",
]
