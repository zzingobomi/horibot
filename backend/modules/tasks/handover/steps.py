"""handover 시나리오 step 들 — omx(giver)가 **자기 eye-in-hand 웹캠으로** 봉을
보고 한쪽 끝을 집어 **수직으로 세워 늘어뜨려** 제시하면, so101(receiver)이
**재검출**해 아래로 늘어진 노출부를 받아 상자에 적치.

⚠ **2026-07-27 큐브→봉(8×2×2cm) 전환, 실물 미검증** (설계 근거 = block.py
docstring + scripts/handover_block_probe.py 결합 스윕. 2cm 큐브는 두 그리퍼가
같은 점에 모여 도달↔가림 정면충돌 — omx_handover_realtest_handoff.md §T.3):
  A. omx 가 본다 — 계산된 nadir 관측 자세 + DETECT_PLANAR (mono ray∩봉 윗면
     평면 z=table+단면 — 2026-07-27 실물: z=table 투영은 2cm 윗면의 원근 확대
     +측면 mask 유입으로 footprint 109mm(실물 80) 과대 = 파지점 오염).
  B. omx 파지 계획 — top-down 전용(5축 도달성 §5.1) + 봉 **한쪽 끝** 파지
     (양 끝 동등 후보 — 축대칭, 도달성이 채택) + tool z ∥ 봉 축(노출 방향 u).
     Z 사다리(depth 없어 크기 가정 — 봉 단면 2cm = 큐브와 동일 사다리).
  C. omx 집기 — 파지 해로 move_j 스윙인 → close (top-down 은 책상면 전용 관절
     리밋이라 위에서 수직 접근·refine 불가 — omx=best-effort, 정밀은 so101).
  D. omx 제시 — **hang(z↑) 매달기 제시** (2026-07-27 밤 개정 — 사용자 토크오프
     데모 실측 2개): pick 이 tool z ∥ −u 로 물었으므로 랑데부 TCP 에서 tool z ↑
     로 젖히면 봉이 그리퍼 아래로 수직 매달림 (중력 모멘트 0, 방위 대칭이라
     조준 불요). 봉-수직 기하는 옛 B/down 과 동일하나 **J5=0 손목 중립** —
     B/down(tool z ↓)은 J5=±180 뒤집기가 유일해라 웹캠 USB 케이블이 감겼다
     (실물 1차 런). so101 파지점 E = TCP − (0,0,tcp_to_e). ⚠ 자세는 omx 5DOF
     **도달 다양체 위에서 구성** (tool x = 팔 평면 방위 α 의 수평 radial —
     임의 방위 열거는 5DOF 에서 measure-zero 라 전멸한다. probe 1차 교훈).
  E. so101 수취 (2026-07-30 전면 재설계 — "IK 전멸=종료" 폐기): closed-loop
     재검출 → **봉 실측**(measure_bar — 점군 PCA 축/자유단. 2026-07-29 근인:
     봉이 omx 조 안에서 84°/70° 돌았는데 계획축 가정으로 전멸, 검출은 정확했다)
     → **잡기 조건 그리드**(_grasp_orients — 조 닫힘축 ⊥ 실측축만 요구, 옛
     "tool z ∥ 축 정확 일치"는 과잉 제약으로 해를 0~7개로 죽였다) → 전멸 시
     **협상**(find_receive_shift 가 "잡을 수 있는 가장 가까운 봉 위치" δ 역산
     → omx_nudge 재배치 → 재관측 재계획, 상한 _NEGOTIATE_MAX) + servo 수렴
     + 수취 순서 불변식(so101 held 뒤에만 omx open) + 충돌/벽 게이트.

실물 첫 런 전 확인 필수 가정 (omx_handover_prep.md §7 미지수 + 봉 신규):
  ① omx tcp/그리퍼 물리 조립이 URDF 규약(tool x=approach, y=jaw)과 일치 (§5.2).
  ② _OMX_TABLE_Z_M — omx base 가 책상 위 전제 (다르면 관측/파지 z 전체 시프트).
  ③ 파지 Z — omx depth 없음 → 봉 단면 2cm 가정. 집는 위치가 이상하면
     chosen_dz 로그로 보정 (_PICK_DZ_LADDER).
  ④ omx held 판정 — 2026-07-27 실물 1차: 빈손 close 가 achieved=1977 에서 스톨
     (옛 limit_min 1800 이 물리 하드스톱 너머 → gap 177 로 **빈손 HELD 오판**
     + 항시 스트레인 load −499). → motors.yaml limit_min 을 실측 스톨로 재설정
     (1975) + load 판정은 abs() (Dynamixel Present_Load 는 방향 부호). 2cm 봉
     물림 시 achieved/load 실측은 아직 없음 — 다음 런 verify_grasp trace 확인.
  ⑤ 픽(봉 수평)→제시(봉 수직) 재배향 스윙 중 봉 끝이 책상을 스칠 가능성 —
     move_j 관절 보간 경로는 봉을 모델링하지 않는다. 첫 런 stop_before_receive
     로 눈 확인 (긁으면 제시 TCP z 사다리를 올리거나 경유 자세 추가).

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

from . import block, frames
from .block import BlockGrasp
from .collision import BasePose, CrossRobotChecker
from .frames import robot_to_world, world_to_robot
from .trace import HandoverTrace

logger = logging.getLogger(__name__)

Vec2 = tuple[float, float]
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
# 유일 판별자). 판정은 **abs(load)** — Dynamixel Present_Load 는 2B signed
# (부호=방향, 2026-07-27 실물: 닫힘 방향 스톨이 −499). 실물 물림 raw 는
# scripts/gripper_characterize.py 로 재도출.
_HELD_LOAD_MIN_RAW = 80

# ── A. omx 관측 (mono — 투영 평면은 봉 윗면 z=table+단면) ──
# omx base frame 의 테이블 평면 z — **1회 설정/측정 앵커** (omx 는 depth 가 없어
# 스스로 못 잼, omx_handover_prep.md §4 횡단 전제). base 가 책상 위 설치 전제
# = 0.0. 실물 첫 런에서 자/블록로 실측 보정.
_OMX_TABLE_Z_M = 0.0
_OMX_OBSERVE_CAM_H_M = 0.25  # nadir 카메라 높이 — §8-1 계산 (25cm = 도달영역 100% 커버)
# 카메라 optical-roll 후보 (deg) — 관측은 이미지 방위 무관이라 자유 DOF. 선호
# 90° = 넓은 화각 축(H 94°)을 omx 도달영역 넓은 축(좌우 44cm)에 정렬 (§8-1).
_OMX_OBSERVE_PSI_DEG = (90.0, 60.0, 120.0, 30.0, 150.0, 0.0, -90.0)
_OBSERVE_SETTLE_S = 0.6
# 검출 신뢰 게이트 — 봉 footprint 기하 (mono 는 score 만으론 약함). 8×2cm
# 각봉이라 긴 변 5~12cm 대역 + 짧은 변 상한 + 종횡비 하한 (정사각/큐브류 컷).
_SCORE_MIN = 0.45
_BLOCK_LEN_MIN_M = 0.050  # footprint 긴 변 하한 (8cm 봉 — mono 과소 여유)
_BLOCK_LEN_MAX_M = 0.120  # footprint 긴 변 상한 (mono 번짐 여유 포함)
# footprint 짧은 변 상한. ⚠ 2026-07-28 실물 4회 실측 = 29 / 33 / 33 / **35** mm
# (실물 단면 20mm — mono 투영이 mask 번짐+측면 유입으로 항상 +9~15mm 부풀린다).
# 옛 35mm 는 그 분포 위에 걸터앉아 있어 4번째 런에서 score 0.82·점군 421 의
# **정상 검출을 컷했다**. 실측 분포 + 여유로 45mm. 정사각 blob(큐브/흰 그리퍼)
# 컷은 종횡비 하한(_BLOCK_ASPECT_MIN)이 담당한다 — 긴 변 하한 50mm 와 겹쳐
# 45mm 짧은 변이면 종횡비 1.1 로 자동 기각이라 상한을 올려도 방어가 안 뚫린다.
_BLOCK_WIDTH_MAX_M = 0.045
_BLOCK_ASPECT_MIN = 2.0  # 긴 변/짧은 변 하한 (미달 = 봉 아님 — 큐브류 컷)

# ── B. omx 파지 계획 (봉 한쪽 끝 top-down, tool z ∥ 봉 축) ──
# 봉 기하 (block.py plan_block_grasp 인자 — 8×2×2cm 주황 각봉 기준):
# ⚠ 봉 스펙은 **known 노브** (검출값 아님) — 2026-07-27 실물: mono 검출 길이가
# mask 번짐+측면 유입으로 109mm(실물 80) 과대 → "검출 tip 에서 20%" 파지점이
# 실물 끝 ~7mm 지점 = 헛집음. 파지 Z 의 "단면 2cm 가정"과 동형으로 길이도
# 물체 스펙을 앵커, 검출 footprint 는 신뢰 게이트 전용.
_BLOCK_LEN_M = 0.080  # known 봉 길이 — 파지점/노출(E) 기하의 앵커
_BLOCK_CROSS_M = 0.020  # known 봉 단면 — 검출 투영 평면(윗면) + 파지 Z 가정
# 손목(J5) 자연해 상한 — **케이블 안전 불변식** (2026-07-27 실물: J5 ±180
# 뒤집기 해가 웹캠 USB 케이블을 감음). pick/present 채택안이 이걸 넘으면 기각
# — 뒤집힌 픽은 제시 hang 에서 봉이 위로 솟아 수취도 조용히 깨진다.
# 92°: 수평 제시(omx 접선, elev 0)의 해석해가 J5=정확히 90.000° 라 90.0 경계에
# 두면 polish 부동소수 잡음으로 채택이 복불복이 된다 (2026-07-29 sim). 2° 여유는
# 케이블 안전과 무관 (실측 안전 J5 80°, 감김 사고는 ±180°).
_WRIST_NATURAL_MAX_RAD = math.radians(92.0)
# 파지점 = 잡는 끝에서 30% (2.4cm). 20%(1.6cm)는 검출 중심의 축방향 바이어스
# 예산이 없었다 — mono mask 가 윗면+측면을 물면 한쪽으로만 ~2cm 번져 중심이
# ~1cm 밀리고 (22:31 실물: footprint 108/102mm vs 실물 80), 파지점이 실물 끝
# ~0.5cm 지점 = 2mm 얕은 물림 (gap 12). 30% 면 바이어스 1.4cm 에도 접촉 ≥1cm.
_PICK_RETRIES = 2  # 집기 실패(GraspFailed) 시 재관측부터 재시도 — abort 금지
_BLOCK_GRASP_FRAC = 0.30
_OMX_JAW_ALONG_M = 0.020  # omx 조가 봉 축 방향으로 차지하는 폭 (실물 실측 대상)
# so101 파지점 E = 노출 세그먼트의 조-쪽 끝에서 65% 지점 (probe 기준치 —
# 두 그리퍼 이격 ~2.5cm 확보 + 끝 여백 1.9cm 로 so101 조 접촉폭 커버)
_BLOCK_EXPOSED_FRAC = 0.65
_SO_MIN_GRASP_M = 0.015  # so101 조 최소 파지 길이
_EXPOSED_MARGIN_M = 0.010  # 노출 최소 margin (짧음 명시 실패 문턱에 가산)
# 파지 Z 사다리 (table 위 dz, 첫 도달+바닥클리어 채택). ⚠ 실물(2026-07-26,
# 큐브에서 실측 — 봉도 단면 2cm 로 동일): TCP=바닥+1cm 로 하니 **그리퍼가
# 바닥을 훑었다** — omx URDF 상 물리 손끝이 TCP(link5 x 9.19cm)보다 ~11mm
# 아래로 뻗고(probe omx_pick_z_probe), 게다가 **손끝에 붙인 골무(finger cot)는
# URDF 에 없어** 그 두께만큼 더 내려간다. → floor 게이트 = URDF 여유
# (_PICK_FLOOR_CLEAR_M) + 골무 오프셋(_GRIPPER_TIP_EXTRA_M) 를 요구해 그만큼
# TCP 를 올린다. 단면 2cm 라 TCP 가 높아도(≤24mm) 긴 손가락이 옆면 아래쪽까지
# 문다. resolve 가 실 손가락 메시로 걸러 사다리 중 가장 낮게 클리어하는 높이를
# 채택 — chosen_dz(trace)로 확인.
_PICK_DZ_LADDER = (0.014, 0.016, 0.018, 0.020, 0.022, 0.024)
# floor 게이트 = table + (실 손끝 목표 여유 + 골무 연장). URDF 손끝은 TCP−11mm
# 이라 14mm 면 3mm 뜨지만, 얇은 골무(~2mm)가 그걸 거의 먹어 "거의 훑음"이었다
# (2026-07-26 실물, 둘 다 재시작 후). → 실 손끝이 눈에 띄게 뜨게 5mm 목표.
_PICK_FLOOR_CLEAR_M = 0.01  # 실 손끝이 바닥에서 뜰 목표 여유 (조금 더 위로, 2026-07-26)
# 골무 등 URDF 미모델 손끝 연장 — floor 게이트에 가산 (URDF 는 골무를 모름).
# ⚠ **실물 튜닝 노브**: 여전히 긁으면 ↑ / 큐브 위를 잡아 미끄러지면 ↓
# (chosen_dz 로그 확인). 사용자 실측 "얼마 안 튀어나옴" → 2mm.
_GRIPPER_TIP_EXTRA_M = 0.002

# ── C. 수취 refine 보정 게이트 (so101 수취측 so_refine 사용) ──
_REFINE_JUMP_MAX_M = 0.03  # 겨냥점 보정 상한 — 초과 = 관측 오염 의심 (계획값 유지)

# ── D. 제시 (랑데부 계산) ──
# ⚠ 랑데부 = **결합 스윕 robust 밴드** (scripts/handover_layout_tune.py — omx
# hang 제시 도달 + 벽 게이트 + so101 수취 도달 + 충돌 클리어 + 노출부 비가림
# 전 게이트 결합). 배치를 물리적으로 바꾸면 이 두 노브가 통째로 무효 —
# layout_tune 재실행으로 재산출한다 (스크립트 docstring 절차).
#
# 2026-07-28 밤: **사용자 토크오프 실측 앵커**로 교체. probe 스윕은 so101
# workcell ROI 격자 위에서만 도는데 그 ROI 가 실제 도달보다 좁아(y_min -0.157)
# 성립하는 수평 자세를 아예 못 봤다 (260/260 이 매달기로 나옴). ROI 를 문서화된
# 도달 하한으로 정정(instance.yaml)하고, 선호점은 **실물에서 두 팔로 직접 만든
# 자세**를 쓴다: omx TCP world (0.126, -0.274, 0.204), 봉 축 (-0.96,-0.064,0.274)
# = **omx 접선과 0.3° / elev +15.9°** (2026-07-29 정정: 옛 주석은 이걸 so101
# 접선 "-t" 로 오귀속했고 그 프레임으로 후보를 만들어 수평 전멸 — 근인).
# 그 자세를 실 URDF·캘로 검증한 값 = 링크 여유 58.9mm(게이트 8mm) / 노출부
# 시선 완전 클리어 / J5 80°. 선호점은 **정렬 seed 일 뿐** — 도달·손목·벽·
# 충돌·가림·수취결합 게이트가 채택을 결정한다.
_PRESENT_Z_WORLD = (0.20, 0.22, 0.18)  # omx TCP 의 world z 후보 (선호순, 실측 0.204)
_RENDEZVOUS_PREFER_XY = (0.126, -0.274)  # 실측 제시 TCP xy
# 후보 상한 — 8 은 재제시(world_offset) 재계획이 도달 교집합에 못 미침 (offset
# 40mm 실측에서 생존 후보(hang)가 prefer 에서 ~11cm 밖: probe 2026-07-29).
# 정상 경로(offset 0)는 앞 후보에서 조기 채택돼 비용 무변 — 상한은 전멸 판정
# 비용만 키운다.
_PRESENT_LIMIT = 48
# E-ROI 사전필터 여유 — ROI 는 도달 percentile 박스(휴리스틱)라 경계 밖에도
# IK 해가 산다. 진짜 판정은 수취 결합 probe — 사전필터가 그보다 먼저 죽이지
# 않게 slack 을 준다 (2026-07-29 사용자 지시: ROI 로 후보 죽이지 말 것).
_E_ROI_SLACK_M = 0.05
# 노출 방향 w 의 elevation 후보 (deg, 노출 끝이 위로 들리는 쪽 +). 선호순.
# ⚠ 2026-07-28 사용자 토크오프 실측 = **+15.9°, omx 접선** (수평 제시). 옛
# 매달기(수직 −90°)는 omx 그리퍼가 봉 위에 있어 so101 시야를 구조적으로 막았다
# (관측 128자세 전부 가림 실측) — 수평이면 그리퍼가 봉 **끝**에 있어 가림이 없다.
# 매달기 폴백은 −90 사다리 항목이 아니라 _present_w_candidates 가 마지막에
# 단일 "hang" 후보로 붙인다 (옛 "family 마다 −90" 은 −t 매달기가 +t 수평보다
# 먼저 시도되는 순서 버그 + 동일 후보 4중복 — 2026-07-29 전멸 근인 중 하나).
# 참고: omx 접선족의 J5 해석해 = 90°±elev (2026-07-29 sim FK 실증) — +15/+30 은
# 손목 게이트(92°) 위라 실제로는 0/−15 가 채택된다. 게이트를 넘는 항목을 남겨둔
# 이유 = +t(반대 접선) 분기에선 부호가 뒤집혀 통과할 수 있어서 (게이트가 결정).
_PRESENT_W_ELEV_DEG = (15.0, 0.0, 30.0, -15.0)
# jaw 를 수직에 붙일 수 있는 최소 여유 — |ẑ − (ẑ·z)z| 가 이보다 작으면 축이
# 수직에 너무 가까워(매달기) 수직 jaw 가 정의되지 않는다 → hang 구성으로 폴백.
_JAW_VERTICAL_MIN = 0.20

# ── E. so101 수취 ──
# ⚠ **수취는 수직 조축 (tool z ∥ 봉 축 = 수직)** — hang 제시로 봉 축이
# 수직임을 계획이 안다 (검출 yaw 불요 — 수직 봉의 평면 OBB yaw 는 무의미).
# 수평 접근(tool x 수평) spin 사다리를 base→E 방위 근접순으로 정렬 — so101
# 쪽에서 진입하는 해 선호 (omx 를 감아 도는 해 회피). 큐브 시대 실측
# "도달·비관통 해가 전부 수직 조축" 과 정합 (2026-07-26 스윕).
# ⚠ 관측 사다리 — omx 가림 회피가 지배 (2026-07-26 실물: omx 그리퍼가 물체를
# 가려 검출 실패 + 높은 관측 자세라 J2 떨림). **저각(elev 25~30°) + so101 측
# 방위** 밴드 (큐브 시대 occlusion probe). hang 은 omx 그리퍼/카메라가 E 위
# ~4.5cm 라 저각 측면 시선과 구조적으로 안 겹침 — block_probe rayTest 23/23
# 비가림. ⚠ 봉 위치가 크게 바뀌면 재-probe.
_RECV_OBS_DIST_M = (0.18, 0.15)  # 재검출 카메라-봉 거리 (probe: 0.18 최적)
_RECV_OBS_ELEV_DEG = (30.0, 25.0)  # 저각만 비가림 (40/55 는 omx 가림 + J2 부하↑)
_RECV_OBS_AZOFF_DEG = (20.0, 0.0, 40.0, -20.0)  # so101 측, +20 이 비가림+최소 J2
_RECV_OBS_PSI_DEG = (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)
# 관측 자세 **도달 검증** 임계 (계획 카메라 pose 대비 실측 FK). ⚠ 2026-07-28
# 실물: so101 이 계획과 위치 14cm·광축 40° 다른 자세에서 검출을 시도했고 봉은
# 광축 86° 밖(프레임 부재) → GDINO 가 갈색 책상을 "orange block" 으로 물었다.
# 검출 실패로 6초 뒤 엉뚱한 사유가 나오는 대신 **여기서 사유를 특정해 실패**한다
# (침묵 금지 — 관측이 틀렸는지 검출이 틀렸는지 trace 로 구분되게).
_OBS_POS_TOL_M = 0.03
_OBS_AXIS_TOL_DEG = 12.0
# 노출부 시선 표본 수 — 이 점들 **전부** 안 가리는 관측 자세만 채택 (가림 게이트).
# handover_layout_tune 게이트 ⑥ 와 같은 판정을 런타임으로 승격 (collision.
# sight_blocked docstring — 2026-07-28 실물: 가려서 반쪽만 본 검출이 겨냥점을
# 2.8cm 밀어 수취 IK 를 전멸시켰다).
_SIGHT_SAMPLES = 9
_RECV_MATCH_RADIUS_M = 0.08  # 제시 계획점 대비 재검출 매치 반경
# 공중 대역 — 제시 z ± 이 값. ⚠ 2026-07-28 60→30mm: 겨냥점 z 가 aerial_target
# (노출부 중간, 실측 오차 0.4mm)으로 정확해졌으므로 옛 관대한 밴드는 오검출
# 통과창일 뿐이다 (60mm 는 48.6mm 오차를 침묵 통과시켜 수취 IK 를 전멸시켰다).
_RECV_Z_BAND_M = 0.03
# 공중 물체는 GDINO 신뢰도가 낮다 — 2026-07-26 실물(큐브): 위치 정확·z 1mm·
# 점군 116 인 진짜 검출인데 score 0.327 로 0.35 문턱에 걸림. 위치/z/반경(8cm)/
# 점군(≥20) 게이트가 오검출 방어를 하므로 score 는 낮춰도 안전. → 0.25.
# 봉은 크고(8cm) 주황색이라 큐브보다 유리할 것 — 실물 첫 런 trace 로 확인.
_RECV_SCORE_MIN = 0.25
_RECV_MIN_POINTS = 20  # 공중 물체 점군 하한
# 수취 자세족 — **잡기 조건 그리드** (_grasp_orients, 2026-07-30 전면 교체).
# ⚠ 옛 "tool z ∥ ±봉축 정확 일치 spin 사다리"(_recv_orients)는 물리가 요구하지
# 않는 제약이었다 — 잡는 데 필요한 건 **조 닫힘축(tool y) ⊥ 봉축** 뿐. 정확족은
# 어제 실패 지점 실측에서 0~7해, 조건족은 10~17해 (full_diagnosis.py 2026-07-30
# — "위치 통과 64/64 인데 자세 IK 전멸"의 근인은 관절 한계에 걸리는 과잉 자세
# 제약). y⊥축을 정확히 유지한 채 남는 2 자유도를 격자로 전개한다:
#   φ = 조 닫힘축의 봉축 둘레 방위 (옛 spin 등가)
#   ψ = 접근 roll (tool z 의 봉축 이탈각) — ψ=0/180 이 옛 z∥±축 족과 동일
_RECV_SPIN_STEP_DEG = 45.0  # φ 스텝
# ψ 사다리 (선호순) — 0/180(옛 축 정렬족, 조 물림 깊이 최대) 먼저, 대각 →
# 축 평행 접근(90/270, 봉 끝으로 진입) 마지막. 게이트(충돌/벽/직선)는 동일 적용.
_RECV_APPROACH_ROLL_DEG = (0.0, 180.0, 45.0, 315.0, 135.0, 225.0, 90.0, 270.0)
# 실측 봉축 품질 게이트 — 점군 주축 span 이 이보다 짧으면 축 추정 불신 (봉
# 노출 46mm 전제 — 반쪽 가림이어도 25mm 는 나온다), 계획축 폴백 (trace 명시).
_BAR_AXIS_MIN_SPAN_M = 0.025
# 실측 봉축 vs 계획축 이탈 경고 문턱 — 초과 = "봉이 조 안에서 돌았다" 지문
# (2026-07-29 실물: 84°/70° — 미믹 약파지 + 재배향 중력토크. 검출은 정확했다).
_BAR_AXIS_DEV_WARN_DEG = 25.0
# 파지 겨냥점 = 실측 자유단에서 축 방향 안쪽 inset (조 접촉폭 확보 + 끝 여백)
_GRASP_TIP_INSET_M = 0.015
# 수취 협상 (2026-07-30 신설) — plan_receive 전멸 시 "so101 이 잡을 수 있는
# 가장 가까운 봉 위치"를 역산해 omx 가 봉을 옮겨 재제시. 전멸=종료 대신 양팔
# 반복 조정 (research: handover pose optimization / capability map 계열).
_NEGOTIATE_MAX = 2  # 협상 왕복 상한 (nudge → 재관측 → 재계획)
_NEGOTIATE_STEP_M = 0.02  # 이동 제안 격자 스텝
_NEGOTIATE_RANGE_M = 0.04  # 이동 제안 최대 반경 (omx 재배치 가용 밴드 내)
# 접근 여유 **사다리** (큰 것 우선 — 긴 standoff 가 정렬/refine 에 유리).
# ⚠ 2026-07-28 실물 실패 재현으로 확정 (debug/handover/20260727_230417):
# 수취 자세족은 도달영역 전체에서 사실상 **단일해**(`zdown/spin0`)인데, 7cm 뒤
# pre 지점이 특이점 근처라 pre→target 직선 **2.0cm 지점에서 관절 141° 구성
# 플립** → 그 하나까지 기각 → 전멸(NoReachableGrasp). 같은 프레임에서 여유를
# 5cm 로 줄이면 통과한다(재현 확인). 단일값 교체 대신 사다리 — 7cm 이 되는
# 지점에서는 7cm 을 쓰고, 특이점에 걸릴 때만 짧아진다 (채택값은 trace
# `pre_clear_m` + plan_receive 로그).
_RECV_PRE_CLEAR_LADDER = (0.07, 0.05, 0.04, 0.03)
_RECV_WITHDRAW_M = 0.08
_RECV_COLLISION_RETRY = 5
# 핸드오프 근접 국면 margin — 매달기 기하는 두 그리퍼가 봉 축으로 ~2.5cm 이격
# (probe 충돌 게이트 23/23 통과). margin 8mm 게이트 (크로스캘 σ_t ~8mm 수준 —
# 실물 첫 수취에서 재확인).
_RECV_COLLISION_MARGIN_M = 0.008
_OMX_HOLD_GRIP_FRAC = 0.2  # 충돌 형상 — 봉 든 omx 조 개구 (거의 닫힘)
# 재제시 보정 (look-then-move, 2026-07-29 실물 근인): omx 가 "여기 들고 있다"
# (FK+base_pose)와 so101 눈 실측이 축 수직으로 40~43mm 어긋난다 (21:11 런
# offset_mm=40.3 실측 확정. 07-27 옛 배치 크로스캘 상태에서도 34mm — §4.8
# 미지수②. 출처 = omx FK/base_pose/조 안 물림 합성, 캘로 못 잡는 성분 포함.
# 크로스캘은 사용자 결정으로 기각). so101 수취 IK 밴드는 1~2해 razor-thin 이라
# 겨냥점만 옮기면 전멸 → 실측 오차를 world_offset 으로 **제시를 전면 재계획**
# (plan_omx_present — so101 쪽 게이트를 실물 좌표로 평가) 해 봉이 "so101 이
# 실제로 받을 수 있는 곳"에 실제로 가게 한다. 출처 불문 상쇄 — PnP
# look-then-move 와 같은 클래스. ⚠ 같은 자세 단일 평행이동(1차 구현)은 omx
# 가용 밴드를 벗어나 IK 전멸했다 (21:11 런) — 재계획이 정석.
_REPRESENT_PERP_MIN_M = 0.010  # 이하 = 수취 servo 가 흡수 (22:06 런: 잔차 17mm
# 로 상한 소진 뒤 수취가 허공 — 15mm 는 마지막 조준 예산을 다 먹었다)
# 보정 상한 = **1회** (2026-07-29 23:13 실물): 그립이 약하면 omx 가 움직일
# 때마다 봉이 조 안에서 돌아 잔차가 수렴하지 않는다 — 재제시 반복은 목표를
# 쫓아다니는 역효과 + 재계획 그라인딩만 반복. 1회로 봉을 수취 밴드 근처에
# 옮기고, 그 뒤 omx 는 **정지** — 움직이지 않는 봉은 plan_receive 보정 +
# 수취 servo(so101 쪽)가 조준한다 (측정과 파지 사이에 봉이 안 움직이는 유일한
# 국면이 "omx 정지 후"다).
_REPRESENT_MAX = 1
# 수취 servo (PnP servo.py 이식, 2026-07-29 22:06 근인): 재검출 한 번 + refine
# 한 번 믿고 open-loop 돌진 → 허공 (연속 두 측정이 12.5mm 어긋남 = 산포 cm 급,
# 점군 52개). PnP 실측 원칙 그대로 — pre(standoff)에서 **수렴할 때까지**
# look-then-move 반복 후에만 진입. pre 3cm 의 카메라-봉 거리 ≈ |(30+77,9,65)|
# ≈ 12.9cm = PnP 검증 최적 측정 대역 (14±3cm, 편차 5-12mm).
_RECV_SERVO_MAX = 3  # pre 에서의 측정-보정 tick 상한
_RECV_SERVO_EPS_M = 0.005  # lateral 수렴 임계 — 이하면 진입 (2cm 봉 편측 여유)
# 재계획 hang 우선 문턱 — offset 이 이보다 크면 수평(접선) 교집합이 빈다는 게
# probe 실증 (40mm: omx IK 432 + 수취 88 기각, ROI 무관·hang 만 생존. §T.9.3).
# 수평 pass 를 먼저 돌면 기각 폭포로 재계획이 ~190s (22:49 실물) — hang pass
# 를 먼저 돌려 수 초에 채택하고, 수평은 그 뒤 폴백으로 유지 (완전성 보존).
_REPRESENT_HANG_FIRST_M = 0.025
# so101 수취 E 밴드 중심 (world xy) — 방향 자유도 히트맵 실측으로 갱신
# (2026-07-30 full_diagnosis.py: x 0.25~0.35 / y −0.30~−0.15 밴드가 방향 도달
# 7~11% 로 최상, 어제 수평 겨냥점 (0.06,−0.24)는 1~5% 최악 급 — 옛 값 (0.22,
# −0.235)도 밴드 안이나 피크를 직접 겨냥). hang pass 후보를 이 점 기준
# 근접순으로 돌려 수취 probe 낭비를 제거. **순서일 뿐 게이트 아님** — 완전성
# 보존. 배치 변경 시 full_diagnosis 히트맵 재실행으로 갱신.
_RECV_SWEET_XY = (0.26, -0.26)
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


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("영벡터 정규화")
    return v / n


def _present_quat_axis(w: Vec3, alpha: float) -> Quat:
    """제시 quat (omx frame) — **노출 방향 w** 를 만드는 자세. 축 일반형.

    pick 이 tool z ∥ **−u**(노출 반대)로 물었으므로 노출 방향 = −tool z 다.
    따라서 봉을 방향 w 로 내밀려면 **tool z = −w**.

    남는 1 자유도(z 둘레 roll)는 **jaw(tool y) 를 수직에 최대한 붙여서** 정한다:
    ① 조가 봉을 위/아래로 물어 중력 모멘트가 조를 비틀지 않는다 ② so101 수취
    자세족과 같은 규약이 된다 (2026-07-28 사용자 토크오프 실측에서 두 로봇 조가
    모두 수직 — omx 18.5° / so101 9.7° off).

    w 가 수직에 가까우면(=매달기) 수직 투영이 퇴화하므로 옛 hang 구성으로 —
    tool x = 팔 평면 방위 α 의 수평 radial (R=Rz(α), **θ=0/J5=0 손목 중립**).
    ⚠ B/down(tool z 정확히 ↓)은 ZYYYX 상 J5=±180 뒤집기가 유일해라 웹캠 USB
    케이블이 감겼다 (2026-07-27 실물) — 그래서 매달기는 tool z ↑ 로 젖힌다.

    ⚠ omx 5DOF(ZYYYX) 도달 다양체: 임의 방위를 요구하면 measure-zero 라 전멸
    한다 (probe 1차 교훈). 이 구성(jaw 수직)이 다양체 위에 놓이는 수평 w 는
    **omx 접선족뿐** — w 후보 생성이 그 족만 내놓는다 (_present_w_candidates,
    2026-07-29 근인 수정). 접선 w 에서 jaw tilt = |elevation| 로 실측(18.5°)과
    정합, J5 해석해 = 90°±elev (sim FK 실증)."""
    z = -_unit(np.asarray(w, dtype=float))
    up = np.array([0.0, 0.0, 1.0])
    y_raw = up - float(np.dot(up, z)) * z
    if float(np.linalg.norm(y_raw)) < _JAW_VERTICAL_MIN:
        # 매달기(축 ≈ 수직) — jaw 를 수직에 붙일 수 없다. 옛 hang 구성.
        x = np.array([math.cos(alpha), math.sin(alpha), 0.0])
        y = np.cross(z, x)
    else:
        y = _unit(y_raw)
        x = np.cross(y, z)
    q = Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _present_w_candidates(
    tcp_w: Vec3, base_omx: BasePose
) -> list[tuple[str, Vec3]]:
    """노출 방향 w 후보 (선호순) — (라벨, world 단위벡터).

    ⚠ 접선은 **omx base 기준** (2026-07-29 근인 수정). omx 5DOF(ZYYYX) 다양체
    에서 jaw-수직 quat(_present_quat_axis)이 성립하는 수평 w 는 omx 팔 평면에
    수직인 방향 = **omx 접선**뿐이다 (tool x 가 팔 평면에 갇힘 — 해석 IK
    analytic_zyyyx 의 M=Ry·Rx 분해와 동치). 옛 코드는 접선을 so101(world 원점)
    기준으로 계산했고, 2026-07-28 직각 재배치 후 두 접선이 랑데부 지점에서
    20~30° 어긋나 수평 후보 전원이 자세 IK 전멸 → 매달기 폴백 → 관측 전멸
    (20260729_054607 trace). 사용자 토크오프 실측 봉 축 (-0.96,-0.064,0.274)
    이 omx 접선과 0.3° — 실측족 = omx 접선족이라는 물리 근거.

    선호순: so101(world 원점)에서 **멀어지는 쪽** 접선(-t, 실측 방향 — 노출
    끝이 omx 그리퍼 반대편이라 so101 접근/시선과 안 겹침) → 반대쪽(+t) ×
    elevation 사다리. radial 족 삭제 — 다양체상 roll 이 기하에 종속이라 jaw 가
    수평으로 강제돼 jaw-수직 quat 은 IK 전멸 (2026-07-29 sim). 매달기(hang)는
    마지막 **단일** 폴백 (family 별 -90 중복 + 순서 버그 제거)."""
    r = np.array([tcp_w[0] - base_omx.x, tcp_w[1] - base_omx.y, 0.0])
    if float(np.linalg.norm(r)) < 1e-9:
        r = np.array([1.0, 0.0, 0.0])
    r = _unit(r)
    t = np.array([-r[1], r[0], 0.0])  # omx radial 의 world z 둘레 +90°
    r_so = np.array([tcp_w[0], tcp_w[1], 0.0])
    if float(np.linalg.norm(r_so)) < 1e-9:
        r_so = np.array([1.0, 0.0, 0.0])
    r_so = _unit(r_so)
    away = -t if float(np.dot(-t, r_so)) < float(np.dot(t, r_so)) else t
    out: list[tuple[str, Vec3]] = []
    for name, base in (("-t", away), ("+t", -away)):
        for elev in _PRESENT_W_ELEV_DEG:
            e = math.radians(elev)
            v = _unit(base * math.cos(e) + np.array([0.0, 0.0, math.sin(e)]))
            out.append(
                (f"{name}/elev{elev:+.0f}", (float(v[0]), float(v[1]), float(v[2])))
            )
    out.append(("hang", (0.0, 0.0, -1.0)))
    return out


def _grasp_orients(e: Vec3, axis: Vec3) -> list[tuple[str, Quat, Vec3]]:
    """수취 자세 후보 (선호순) — (라벨, quat(so101=world frame), 접근 tool x).

    **잡기 조건 그리드** (2026-07-30 전면 교체): 봉을 잡는 데 물리가 요구하는
    제약은 **조 닫힘축(tool y) ⊥ 봉 축** 하나뿐이다. 옛 _recv_orients 는 여기에
    "tool z ∥ ±봉축 정확 일치"를 추가로 강제했는데, 그 과잉 제약이 어제 실패
    지점 실측에서 해를 0~7개로 죽였다 (조건족은 10~17개 — full_diagnosis.py,
    죽는 메커니즘은 관절 한계). y⊥축을 정확히 유지한 채 남는 2 자유도를 격자
    전개한다:
      φ (_RECV_SPIN_STEP_DEG): 조 닫힘축의 봉축 둘레 방위 — 옛 spin 등가.
      ψ (_RECV_APPROACH_ROLL_DEG): 접근 roll — tool z 가 봉축에서 벗어나는 각.
        ψ=0/180 이 옛 z∥±축 족과 정확히 동일 (superset — 옛 해는 전부 포함).
    선호순 = 접근(tool x)이 base→E 방위(so101 쪽 진입)에 가까운 순 → ψ 사다리
    순 — omx 를 감아 도는 해 회피는 유지, 물림 깊이 최대(축 정렬) 우선.
    """
    a = _unit(np.asarray(axis, dtype=float))
    # 기준 조 방위 u1 = base→E 방위의 축 수직 성분 (퇴화 시 수직/x 폴백)
    r = np.array([e[0], e[1], 0.0])
    if float(np.linalg.norm(r)) < 1e-9:
        r = np.array([1.0, 0.0, 0.0])
    r = _unit(r)
    u1_raw = r - float(np.dot(r, a)) * a
    if float(np.linalg.norm(u1_raw)) < 1e-6:
        alt = np.array([0.0, 0.0, 1.0])
        u1_raw = alt - float(np.dot(alt, a)) * a
        if float(np.linalg.norm(u1_raw)) < 1e-6:
            u1_raw = np.array([1.0, 0.0, 0.0])
    u1 = _unit(u1_raw)
    u2 = np.cross(a, u1)
    phis = [s * _RECV_SPIN_STEP_DEG for s in
            range(int(360.0 / _RECV_SPIN_STEP_DEG))]
    entries: list[tuple[float, int, str, Quat, Vec3]] = []
    for phi in phis:
        p = math.radians(phi)
        y = _unit(u1 * math.cos(p) + u2 * math.sin(p))  # 조 닫힘축 ⊥ a (정확)
        n = np.cross(a, y)
        for pi, psi in enumerate(_RECV_APPROACH_ROLL_DEG):
            s = math.radians(psi)
            z = _unit(a * math.cos(s) + n * math.sin(s))
            x = np.cross(y, z)
            q = Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()
            # 선호 1순위: 접근이 base→E 진입 방위에 가까움 (옛 규약 유지)
            align = -float(np.dot(x, r))
            entries.append(
                (
                    align,
                    pi,
                    f"jaw{phi:.0f}/roll{psi:.0f}",
                    (float(q[0]), float(q[1]), float(q[2]), float(q[3])),
                    (float(x[0]), float(x[1]), float(x[2])),
                )
            )
    entries.sort(key=lambda t: (t[0], t[1]))
    return [(label, q, x) for _a, _p, label, q, x in entries]


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


def _trusted_block_candidates(
    cands: list[OrientedDetection],
) -> list[OrientedDetection]:
    """봉 신뢰 게이트 — score + 기하(긴 변 대역 + 짧은 변 상한 + 종횡비 하한).
    mono 는 depth 게이트가 없으므로 기하가 오검출 컷의 주력 (셀 밖 컷은
    detector ROI 가 상류 담당). footprint = (긴 변, 짧은 변). 종횡비 하한이
    큐브류/그리퍼 오검출(정사각 blob) 컷 — 봉은 4:1 이라 확실히 길쭉하다."""
    return [
        c
        for c in cands
        if c.score >= _SCORE_MIN
        and _BLOCK_LEN_MIN_M <= c.footprint[0] <= _BLOCK_LEN_MAX_M
        and c.footprint[1] <= _BLOCK_WIDTH_MAX_M
        and c.footprint[0] >= _BLOCK_ASPECT_MIN * max(c.footprint[1], 1e-6)
    ]


@step(title="omx 관측·검출")
async def omx_observe_detect(
    ctx: TaskContext,
    omx: str,
    prompt: str,
    observe_joints: list[float],
    trace: HandoverTrace | None = None,
) -> OrientedDetection:
    """관측 자세 이동 → 정지 → DETECT_PLANAR (mono ray∩봉 **윗면** 평면
    z=table+단면). z=table 투영은 윗면(z=2cm)이 원근으로 ~9% 확대 + 측면 mask
    유입까지 겹쳐 footprint 과대 (2026-07-27 실물 109mm vs 80) — 윗면 평면
    투영이 중심/크기 왜곡의 뿌리 수정. 신뢰 컷 후 최고 score. 0건 = 명시 실패
    (사유 + 다음 행동)."""
    await _move_j(ctx, omx, joints=observe_joints)
    await asyncio.sleep(_OBSERVE_SETTLE_S)
    plane_z = _OMX_TABLE_Z_M + _BLOCK_CROSS_M  # 봉 윗면
    res = await ctx.call(
        Detector.Service.DETECT_PLANAR,
        DetectPlanarRequest(
            robot_id=omx, plane_z=plane_z, prompts=[prompt], top_k=_TOP_K
        ),
        DetectOrientedResponse,
    )
    await _emit(
        trace,
        {
            "phase": "observe",
            "event": "detect_planar",
            "prompt": prompt,
            "plane_z": plane_z,
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
    trusted = _trusted_block_candidates(res.candidates)
    if not trusted:
        raise DetectionNotFound(
            prompt,
            candidates=len(res.candidates),
            reason=(
                f"신뢰 컷 미달 (score≥{_SCORE_MIN}, 긴 변 "
                f"{_BLOCK_LEN_MIN_M * 1000:.0f}~{_BLOCK_LEN_MAX_M * 1000:.0f}mm, "
                f"짧은 변≤{_BLOCK_WIDTH_MAX_M * 1000:.0f}mm, "
                f"종횡비≥{_BLOCK_ASPECT_MIN}) — 봉 배치/조명/"
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


# ─── B. omx 파지 계획 (top-down + J5 roll, tool z ∥ 봉 축) ────────────


def plan_block_grasp_from(det: OrientedDetection, base_omx: BasePose) -> BlockGrasp:
    """검출 → 봉 파지 기하 (omx frame). 순수 계산 — step 아님 (모션 0).
    봉이 짧으면 block.plan_block_grasp 가 **명시 실패** (침묵 진행 금지).

    길이는 검출값이 아니라 known 스펙(_BLOCK_LEN_M) — 검출 center+yaw 만 소비
    (2026-07-27 실물 헛집음의 뿌리: 검출 길이 과대 → 파지점이 실물 끝으로 밀림.
    상세 = _BLOCK_LEN_M 노브 주석). 검출 footprint 는 신뢰 게이트가 이미 소비."""
    return block.plan_block_grasp(
        (det.position[0], det.position[1]),
        det.grasp_yaw,
        # 길이·단면 **둘 다** known 앵커 — 검출 footprint 는 신뢰 게이트 전용
        # (2026-07-28: 짧은 변도 mono 가 +9~15mm 부풀린다. 지금은 단면이 파지
        # 기하에 안 쓰이지만, 오염된 값을 계획에 흘리는 통로 자체를 막는다)
        (_BLOCK_LEN_M, _BLOCK_CROSS_M),
        grasp_frac=_BLOCK_GRASP_FRAC,
        jaw_along_m=_OMX_JAW_ALONG_M,
        exposed_frac=_BLOCK_EXPOSED_FRAC,
        min_exposed_m=_SO_MIN_GRASP_M + _EXPOSED_MARGIN_M,
        len_min_m=_BLOCK_LEN_MIN_M,
        len_max_m=_BLOCK_LEN_MAX_M,
    )


@dataclass(frozen=True, slots=True)
class BlockPick:
    """omx 봉 파지 계획 산출 — 실행(집기)과 제시(present)가 공유.

    tool z ∥ **−u**(노출 반대) 규약 — 제시 hang 이 tool z 를 하늘로 젖히면
    노출부(긴 자유부)가 자동으로 아래를 향한다 (J5=0 손목 중립, 2026-07-27)."""

    sols: list[list[float]]  # [grasp] 관절해 (단일 — top-down pre/lift 폐기)
    quat: Quat
    grasp_omx: Vec3  # 파지점 (omx frame — z = table + 파지 dz)
    u_omx: Vec2  # 노출 방향 (omx frame XY 단위벡터 — 로깅/계획 추적용)
    geom: BlockGrasp
    chosen_dz: float  # 채택 파지 Z (table 위, 실물 보정 데이터)


@step(title="omx 집기 계획")
async def plan_omx_pick_block(
    ctx: TaskContext,
    omx: str,
    grasp: BlockGrasp,
    trace: HandoverTrace | None = None,
) -> BlockPick:
    """봉 끝 파지점(책상면 top-down)만 resolve. top-down 은 관절 리밋상 책상
    근처에서만 도달한다(§5.1 은 DOF 수 얘기 — 실측 도달 밴드는 z≈table). 위에서
    수직 접근하는 pre / 수직 lift 를 top-down 으로 강제하면 IK 전멸이라 폐기했다:
    접근은 관측 자세에서 파지 해로 move_j 스윙인, 리프트는 제시 단계가 도달 가능한
    자세로 수행 (omx=best-effort, 정밀은 so101).

    양 끝 후보(축대칭 — 도달성이 채택) × Z 사다리(낮은→높은, 첫 도달 채택).
    tool z ∥ **−u** (노출 반대 — loop 안 주석 참조: 자연손목에선 먼 끝이 채택돼
    제시 hang(z↑)에서 봉이 아래로 매달린다. 2026-07-27 케이블 감김 수정).
    omx 는 depth 가 없어 높이를 못 재므로 파지 Z 는 단면 2cm 가정이 앵커 —
    사다리는 도달/바닥클리어를 위한 소폭 탐색(chosen_dz 로 실물 보정)."""
    z_ladder = [_OMX_TABLE_Z_M + dz for dz in _PICK_DZ_LADDER]
    # 자연손목 끝 먼저 — tool z ∥ −u 라 노출(u)이 base 쪽인 끝(dot(u,g)<0 =
    # 대체로 먼 끝)이 J5≈0. 반대 끝은 J5≈±180 뒤집힌 해로도 도달은 하므로
    # (offline probe 2026-07-27) 순서+아래 손목 게이트 둘 다 필요.
    ends = sorted(
        grasp.ends, key=lambda eu: eu[1][0] * eu[0][0] + eu[1][1] * eu[0][1]
    )
    groups: list[list[TcpPose]] = []
    # (quat, grasp_xy, u, gz)
    metas: list[tuple[Quat, Vec2, Vec2, float]] = []
    for (gx, gy), u in ends:  # 양 끝 후보 (자연손목 우선 정렬)
        # tool z ∥ **−u** (노출 반대) — 2026-07-27 실물+사용자 데모: tool z ∥ u 는
        # 자연손목(J5=0) top-down 에서 노출부가 팔 반대(+radial)로만 가능해
        # 가까운 끝을 물었고, 제시(매달기)가 tool z ↓ = J5 ±180 뒤집기를 강제
        # (USB 케이블 감김). −u 규약이면 **먼 끝**이 자연해가 되고 제시는
        # tool z ↑(_present_quat_hang, J5=0)로 봉이 저절로 아래 매달린다.
        yaw = math.atan2(-u[1], -u[0])
        quat = _grasp_quat(yaw, 0)
        for gz in z_ladder:
            groups.append([TcpPose(position=(gx, gy, gz), quaternion=quat)])
            metas.append((quat, (gx, gy), u, gz))
    floor_z = _OMX_TABLE_Z_M + _PICK_FLOOR_CLEAR_M + _GRIPPER_TIP_EXTRA_M
    # 손목 게이트 alive-loop — 뒤집힌(J5 |>90°|) 채택안은 기각하고 그 그룹을
    # 빼고 재-resolve (수취 충돌 게이트와 동형). 뒤집힌 픽이 통과하면 제시
    # hang 에서 봉이 위로 솟아 수취가 조용히 깨진다 + 케이블 감김.
    alive = list(range(len(groups)))
    chosen = -1
    res = None
    wrist_rejects: list[int] = []
    while alive:
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=[groups[i] for i in alive],
                # floor 게이트 = 여유 + 골무(URDF 미모델) — ⚠ 2026-07-27 골무 항
                # 배선 (그전엔 주석만 선언된 죽은 노브 — 돌려도 무효였다. +2mm
                # 만큼 07-26 실측 대비 게이트 상승: 긁힘 방지 쪽 보수)
                floor_z=floor_z,
            ),
            ResolveReachableResponse,
            robot_id=omx,
        )
        if res.index < 0:
            break
        gi = alive[res.index]
        if abs(res.solutions[0][-1]) > _WRIST_NATURAL_MAX_RAD:
            logger.warning(
                "plan_omx_pick_block: 그룹 %d 채택안 손목 뒤집힘 (J5=%.0f°) — 기각",
                gi,
                math.degrees(res.solutions[0][-1]),
            )
            wrist_rejects.append(gi)
            alive.remove(gi)
            continue
        chosen = gi
        break
    await _emit(
        trace,
        {
            "phase": "pick",
            "event": "plan_omx_pick_block",
            "ends": [[list(g), list(u)] for g, u in ends],
            "z_ladder": z_ladder,
            "length_m": grasp.length_m,
            "width_m": grasp.width_m,
            "exposed_len_m": grasp.exposed_len_m,
            "index": chosen,
            "chosen_dz": (metas[chosen][3] - _OMX_TABLE_Z_M) if chosen >= 0 else None,
            "chosen_u": list(metas[chosen][2]) if chosen >= 0 else None,
            "wrist_rejects": wrist_rejects,
            "group_failures": res.group_failures if res is not None else [],
        },
    )
    if chosen < 0 or res is None:
        raise NoReachableGrasp(
            f"omx top-down 봉 끝 파지 후보 {len(groups)}개 전멸 — "
            f"{res.message if res is not None else ''} (손목 뒤집힘 기각 "
            f"{len(wrist_rejects)}개 포함). 봉을 omx 도달영역 중심 쪽으로 "
            "옮긴 후 다시 실행하세요"
        )
    quat, g_xy, u, g_z = metas[chosen]
    g = (g_xy[0], g_xy[1], g_z)
    dz = g_z - _OMX_TABLE_Z_M
    logger.info(
        "plan_omx_pick_block: u=(%.2f,%.2f) 채택 — grasp(omx)=(%.3f,%.3f,%.3f) "
        "파지dz=%.0fmm 봉 %.0f×%.0fmm 노출=%.0fmm",
        u[0],
        u[1],
        g[0],
        g[1],
        g[2],
        dz * 1000,
        grasp.length_m * 1000,
        grasp.width_m * 1000,
        grasp.exposed_len_m * 1000,
    )
    return BlockPick(
        sols=res.solutions,
        quat=quat,
        grasp_omx=g,
        u_omx=u,
        geom=grasp,
        chosen_dz=dz,
    )


# ─── C. omx 집기 (move_j 스윙인 — look-then-move 폐기) ─────────────────


@step(title="omx 집기")
async def omx_pick_block(
    ctx: TaskContext,
    omx: str,
    plan: BlockPick,
    trace: HandoverTrace | None = None,
) -> None:
    """관측 자세 → 파지 해로 바로 move_j(속도 cap 으로 부드럽게) → close → 판정.
    top-down 은 책상면에서만 도달하므로 위에서의 수직 접근(pre)·수직 lift 는 불가 —
    스윙인으로 간다. 리프트/제시는 다음 단계(plan_omx_present)가 도달 가능한 자세로
    수행. refine(look-then-move) 폐기 — omx=best-effort, 정밀은 so101 수취가 흡수."""
    await _move_j(ctx, omx, joints=plan.sols[0])
    await set_gripper(ctx, omx, open_=False)
    await verify_grasp(
        ctx, omx, phase="omx close 직후", trace=trace, advisory=True
    )


# ─── D. omx 제시 (랑데부 계산 — 티칭 폐기) ────────────────────────────


@dataclass(frozen=True, slots=True)
class PresentPlan:
    sols: list[list[float]]  # [제시 자세] 관절해
    quat: Quat
    h_world: Vec3  # so101 파지점 E 의 **실물 추정** = FK E + world_offset
    tcp_world: Vec3  # 제시 TCP (world, **FK 좌표**) — 명령이 사는 공간
    w: Vec3  # **노출 방향** (world 단위) — 봉 축. 하류 전체가 이걸 소비한다
    label: str  # 채택된 w 후보 라벨 (예 "-t/elev+15") — 실물 원인분석용
    # 세계모델 오차 (실물 − FK, so101 재검출 실측) — 재제시 보정이 누적 기입.
    # so101 쪽 소비자(수취 probe/관측 겨냥/매치)는 FK+offset(실물)을 봐야 한다.
    world_offset: Vec3 = (0.0, 0.0, 0.0)


def _present_tcp_real(present: PresentPlan) -> Vec3:
    """제시 TCP 의 실물 추정 (world) = FK + world_offset — so101 쪽 소비자용."""
    return (
        present.tcp_world[0] + present.world_offset[0],
        present.tcp_world[1] + present.world_offset[1],
        present.tcp_world[2] + present.world_offset[2],
    )


async def _receive_probe(
    ctx: TaskContext,
    so101: str,
    e_world: Vec3,
    w: Vec3,
    omx_present_sol: list[float],
    checker: CrossRobotChecker | None,
) -> bool:
    """제시 후보의 **수취 결합 판정** (모션 0) — 이 E/w 에서 so101 수취
    [pre, grasp] 해가 사는지 채택 전에 확인한다.

    2026-07-29 전멸 근인 ②의 예방: 제시가 자기 게이트만 보고 채택돼 수취
    전멸(NoReachableGrasp)이 관측 이동 실행 후에야 드러났다. 2026-07-30 잡기
    조건 그리드 전환에 맞춰 probe 도 같은 족의 **선호순 앞 12개 × 여유 2단**
    만 본다 (비용 옛 6×4 와 동일, 방위 커버리지는 ψ 확장으로 더 넓게).
    실제 수취는 plan_receive 가 재검출 겨냥점으로 전 그리드를 다시 돈다.

    벽/충돌 게이트는 채택 해에만 건다 (plan_receive 의 alive-loop 축약판,
    재시도 3회) — probe 는 존재 증명이 목적이라 완전 열거는 안 한다."""
    orients = _grasp_orients(e_world, w)[:12]
    groups: list[list[TcpPose]] = []
    for _label, quat, a in orients:
        for clear in _RECV_PRE_CLEAR_LADDER[1::2]:  # (0.05, 0.03) — 존재 증명용
            pre = (
                e_world[0] - a[0] * clear,
                e_world[1] - a[1] * clear,
                e_world[2] - a[2] * clear,
            )
            groups.append(
                [
                    TcpPose(position=pre, quaternion=quat),
                    TcpPose(position=e_world, quaternion=quat),
                ]
            )
    alive = list(range(len(groups)))
    for _ in range(3):
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=[groups[i] for i in alive], linear=True
            ),
            ResolveReachableResponse,
            robot_id=so101,
        )
        if res.index < 0:
            return False
        gi = alive[res.index]
        if checker is None:
            return True
        if not _behind_wall(checker, "a", res.solutions[-1]) and (
            not checker.path_in_collision(
                res.solutions,
                omx_present_sol,
                grip_a=1.0,
                grip_b=_OMX_HOLD_GRIP_FRAC,
                margin_m=_RECV_COLLISION_MARGIN_M,
            )
        ):
            return True
        alive.remove(gi)
        if not alive:
            return False
    return False


@step(title="제시 계획")
async def plan_omx_present(
    ctx: TaskContext,
    omx: str,
    so101: str,
    roi_so: WorkcellRoi,
    roi_omx: WorkcellRoi,
    base_omx: BasePose,
    pick: BlockPick,
    so101_joints: list[float],
    checker: CrossRobotChecker | None,
    trace: HandoverTrace | None = None,
    world_offset: Vec3 = (0.0, 0.0, 0.0),
) -> PresentPlan:
    """랑데부 후보(workcell ROI 교집합, 흉터 5 예방)를 **TCP 위치**로 순회하고,
    각 점에서 **노출 방향 w 후보**(_present_w_candidates — omx 접선 × elevation
    + hang 폴백)를 선호순으로 resolve → 채택안을 손목/벽/cross-robot 충돌 +
    **수취 결합**(_receive_probe) 게이트. 첫 통과 채택, 전멸 = 명시 실패.

    ⚠ world_offset (재제시 보정, 2026-07-29): so101 재검출이 실측한 세계모델
    오차 (실물 = FK + offset, 21:11 런 실측 40mm). omx IK/명령은 FK 공간
    그대로, **so101 쪽 게이트(E-ROI/수취 probe)와 h_world 는 실물(FK+offset)
    로 평가** — 그래야 "so101 이 실제로 받을 수 있는 곳에 봉이 실제로 가는"
    FK 목표가 뽑힌다. 1차 구현(같은 자세 단일 평행이동)은 omx 가용 밴드를
    벗어나 IK 전멸 — 전면 재계획이 정석 (후보 스윕/게이트 전부 재사용).

    E(so101 파지점 = 재검출 겨냥점) = **TCP + w·tcp_to_e** — 봉이 방향 w 로
    뻗어 있으므로 파지점은 그 축 위다 (block.py tcp_to_e_m 은 축 방향 스칼라
    거리). 매달기(w=아래)면 옛 `TCP − (0,0,tcp_to_e)` 와 같아진다.
    E 가 so101 ROI 밖인 조합은 기각 (축 오프셋만큼 두 팔 유효 대역이 어긋난다).

    ⚠ 2026-07-28 수평 제시로 전환: 매달기는 omx 그리퍼가 봉 **위**에 있어
    so101 관측 시야를 구조적으로 막았다 (관측 128자세 전부 가림 실측). 수평이면
    그리퍼가 봉 **끝**에 있어 가림이 사라진다. 사용자 토크오프 실측(omx 접선,
    elev +15.9°, 두 팔 링크 여유 58.9mm, 시선 클리어)이 이 족의 근거다.

    ⚠ 수취 결합 게이트 (2026-07-29): 제시 자기 게이트만 보고 채택하면 수취
    전멸이 관측 이동까지 실행된 뒤에야 드러난다 — sim 스윕에서 수취 해는
    랑데부 지역 전체에 1~2/16 뿐이라, 제시만 통과하는 후보가 다수다. offline
    probe(handover_block_probe)의 결합 판정을 계획 시점으로 승격."""
    cands = frames.rendezvous_candidates(
        roi_so,
        roi_omx,
        base_omx,
        _PRESENT_Z_WORLD,
        limit=_PRESENT_LIMIT,
        prefer_point=_RENDEZVOUS_PREFER_XY,  # probe robust 밴드 중심 직접 겨냥
    )
    if not cands:
        raise TaskError(
            "두 팔 공통 워크스페이스(workcell ROI 교집합)가 비어 있음 — "
            "instance.yaml workcell 값/_PRESENT_Z_WORLD 를 확인하세요"
        )
    rejects: list[str] = []
    omx_tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=omx
    )
    # 수평(접선) 후보를 **전 랑데부 후보에서 소진한 뒤에만** hang 폴백 — hang 을
    # 후보별 사다리 끝에 두면 앞 랑데부의 hang 이 뒤 랑데부의 수평을 가린다
    # (2026-07-29 sim: c0 hang 이 c2 수평을 가려 관측-전멸 자세가 채택됐다).
    #
    # ⚠ resolve 는 **후보(tcp)당 1콜 배치** + 게이트 기각 시 alive-loop —
    # (tcp × w) 쌍마다 1콜(옛 코드)은 재계획 스윕에서 왕복 수백 회 = 분 단위
    # 지연이었다 (2026-07-29 22:06 실물: 재계획 1회 ~140s). 채택 순서는 불변
    # (후보-major, 후보 안 w 선호순 = 배치 그룹 순서, hang 은 마지막 pass).
    async def _try_candidate(
        tcp_w: Vec3, hang_pass: bool
    ) -> PresentPlan | None:
        tcp_omx = world_to_robot(tcp_w, base_omx)
        alpha = math.atan2(tcp_omx[1], tcp_omx[0])
        entries: list[tuple[str, Vec3, Vec3, Quat]] = []
        for label, w in _present_w_candidates(tcp_w, base_omx):
            if (label == "hang") != hang_pass:
                continue
            # E 실물 추정 = FK E + world_offset — so101 쪽 게이트는 이 값으로
            e_world = (
                tcp_w[0] + w[0] * pick.geom.tcp_to_e_m + world_offset[0],
                tcp_w[1] + w[1] * pick.geom.tcp_to_e_m + world_offset[1],
                tcp_w[2] + w[2] * pick.geom.tcp_to_e_m + world_offset[2],
            )
            if not (
                roi_so.x_min - _E_ROI_SLACK_M <= e_world[0]
                <= roi_so.x_max + _E_ROI_SLACK_M
                and roi_so.y_min - _E_ROI_SLACK_M <= e_world[1]
                <= roi_so.y_max + _E_ROI_SLACK_M
                and roi_so.z_min - _E_ROI_SLACK_M <= e_world[2]
                <= roi_so.z_max + _E_ROI_SLACK_M
            ):
                rejects.append(f"tcp={tcp_w} w={label}: E={e_world} so101 ROI 밖")
                continue
            # w 는 world 정의 — omx frame 회전 (base yaw 만, 평행이동 무관)
            quat = _present_quat_axis(
                frames.world_dir_to_robot(w, base_omx), alpha
            )
            entries.append((label, w, e_world, quat))
        alive = list(range(len(entries)))
        while alive:
            res = await ctx.call(
                Motion.Service.RESOLVE_REACHABLE,
                ResolveReachableRequest(
                    groups=[
                        [TcpPose(position=tcp_omx, quaternion=entries[i][3])]
                        for i in alive
                    ]
                ),
                ResolveReachableResponse,
                robot_id=omx,
            )
            if res.index < 0:
                rejects.append(
                    f"tcp={tcp_w}: 잔여 w {len(alive)}종 도달 불가 ({res.message})"
                )
                return None
            gi = alive[res.index]
            label, w, e_world, quat = entries[gi]
            # 손목 뒤집힘 기각 — 케이블 안전 불변식 (_WRIST_NATURAL_MAX_RAD)
            if abs(res.solutions[0][-1]) > _WRIST_NATURAL_MAX_RAD:
                rejects.append(
                    f"tcp={tcp_w} w={label}: 손목 뒤집힘 해 "
                    f"(J5={math.degrees(res.solutions[0][-1]):.0f}°)"
                )
                alive.remove(gi)
                continue
            # 벽(뒤) 침범 — omx 링크가 베이스 뒤로 넘어가면 기각 (side="b")
            if checker is not None and _behind_wall(
                checker, "b", res.solutions[0]
            ):
                rejects.append(f"tcp={tcp_w} w={label}: omx 벽(뒤) 침범")
                alive.remove(gi)
                continue
            if checker is not None and _omx_path_collides(
                checker,
                so101_joints,
                [list(omx_tcp.joints), res.solutions[0]],
            ):
                rejects.append(f"tcp={tcp_w} w={label}: so101 충돌 위험")
                alive.remove(gi)
                continue
            if not await _receive_probe(
                ctx, so101, e_world, w, res.solutions[0], checker
            ):
                rejects.append(f"tcp={tcp_w} w={label}: 수취 결합 probe 전멸")
                alive.remove(gi)
                continue
            return _adopted_plan(
                tcp_w, label, w, e_world, quat, res.solutions, alpha
            )
        return None

    async def _adopted_emit(plan: PresentPlan, alpha: float) -> None:
        await _emit(
            trace,
            {
                "phase": "present",
                "event": "plan_omx_present",
                "tcp_world": list(plan.tcp_world),
                "orientation": f"axis({plan.label})",
                "w_world": [round(v, 4) for v in plan.w],
                "alpha_deg": round(math.degrees(alpha), 1),
                "h_world": list(plan.h_world),
                "tcp_to_e_m": pick.geom.tcp_to_e_m,
                "j5_deg": round(math.degrees(plan.sols[0][-1]), 1),
                "world_offset_mm": [
                    round(float(v) * 1000, 1) for v in world_offset
                ],
                "rejects": rejects,
            },
        )

    def _adopted_plan(
        tcp_w: Vec3, label: str, w: Vec3, e_world: Vec3, quat: Quat,
        sols: list[list[float]], alpha: float,
    ) -> PresentPlan:
        logger.info(
            "plan_omx_present: tcp=(%.3f,%.3f,%.3f) w=%s 채택 (기각 %d) — "
            "E=(%.3f,%.3f,%.3f) J5=%.0f°",
            tcp_w[0], tcp_w[1], tcp_w[2], label, len(rejects),
            e_world[0], e_world[1], e_world[2],
            math.degrees(sols[0][-1]),
        )
        return PresentPlan(
            sols=sols, quat=quat, h_world=e_world, tcp_world=tcp_w,
            w=w, label=label, world_offset=world_offset,
        )

    # offset 이 크면 hang pass 먼저 (_REPRESENT_HANG_FIRST_M 주석 — 수평은
    # 교집합이 비어 기각 폭포만 만든다. 폴백으로는 유지 = 완전성 보존)
    off_norm = float(np.linalg.norm(np.asarray(world_offset)))
    passes = (
        (True, False) if off_norm > _REPRESENT_HANG_FIRST_M else (False, True)
    )
    for hang_pass in passes:
        ordered = cands
        if hang_pass:
            # hang 의 E_real xy = cand xy + offset (후보 무관 상수 이동) —
            # 수취 밴드 중심 근접순이 죽은 probe 를 안 태운다 (_RECV_SWEET_XY)
            ordered = sorted(
                cands,
                key=lambda t: math.hypot(
                    t[0] + world_offset[0] - _RECV_SWEET_XY[0],
                    t[1] + world_offset[1] - _RECV_SWEET_XY[1],
                ),
            )
        for tcp_w in ordered:
            plan = await _try_candidate(tcp_w, hang_pass)
            if plan is not None:
                tcp_omx = world_to_robot(tcp_w, base_omx)
                await _adopted_emit(
                    plan, math.atan2(tcp_omx[1], tcp_omx[0])
                )
                return plan
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
    await verify_grasp(
        ctx, omx, phase="제시 자세 도달", trace=trace, advisory=True
    )


# ─── E. so101 수취 (재검출 + refine — FK 짐작 폐기) ───────────────────


def _camera_pose_of(tcp: TcpState, t_tcp_cam: np.ndarray) -> tuple[Vec3, Vec3]:
    """TCP 상태(FK 실측) + hand_eye → (카메라 위치, 광축) world.

    `_camera_pose_groups` 의 역방향 (그쪽은 T_tcp = T_cam · X⁻¹, 여기는
    T_cam = T_tcp · X) — 같은 X 를 쓰므로 계획값과 직접 비교 가능하다."""
    t_base_tcp = np.eye(4)
    t_base_tcp[:3, :3] = Rotation.from_quat(tcp.quaternion).as_matrix()
    t_base_tcp[:3, 3] = np.asarray(tcp.position, dtype=float)
    t_base_cam = t_base_tcp @ t_tcp_cam
    p, a = t_base_cam[:3, 3], t_base_cam[:3, 2]
    return (
        (float(p[0]), float(p[1]), float(p[2])),
        (float(a[0]), float(a[1]), float(a[2])),
    )


def _axis_error_deg(a: Vec3, b: Vec3) -> float:
    """두 단위(에 준하는) 방향 벡터 사이 각 (deg)."""
    va, vb = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    va, vb = va / np.linalg.norm(va), vb / np.linalg.norm(vb)
    return float(math.degrees(math.acos(float(np.clip(np.dot(va, vb), -1.0, 1.0)))))


def sight_targets(tcp_world: Vec3, w: Vec3, geom: BlockGrasp) -> list[Vec3]:
    """봉 **노출부**의 시선 표본 (world) — 조 끝에서 봉 자유단까지 축 방향 등간격.

    제시 TCP 에서 노출 방향 w 로: 조가 차지하는 절반(jaw/2)을 지나 exposed_len
    만큼. 매달기(w=(0,0,−1))면 옛 "TCP 아래로 수직 표본" 과 같아진다."""
    a = np.asarray(w, dtype=float)
    start = np.asarray(tcp_world, dtype=float) + a * (_OMX_JAW_ALONG_M / 2.0)
    n = max(2, _SIGHT_SAMPLES)
    return [
        tuple(  # type: ignore[misc]
            float(v) for v in start + a * (geom.exposed_len_m * k / (n - 1))
        )
        for k in range(n)
    ]


@dataclass(frozen=True, slots=True)
class ObservePlan:
    """수취 관측 자세 계획 — 관절해 + **계획 카메라 pose**.

    카메라 pose 를 들고 다니는 이유: 이동 후 실측 FK 로 **도달 검증**을 해야
    한다 (2026-07-28 실물 — _OBS_POS_TOL_M 주석). 관측이 어긋난 걸 모르고
    검출을 시도하면 실패 사유가 "검출 실패"로 위장된다."""

    joints: list[float]
    cam_pos: Vec3  # 계획 카메라 위치 (world)
    cam_axis: Vec3  # 계획 광축 (world, H 를 향함)


@step(title="수취 관측 자세")
async def plan_so_observe(
    ctx: TaskContext,
    so101: str,
    omx: str,
    t_tcp_cam: np.ndarray,
    present: PresentPlan,
    geom: BlockGrasp,
    checker: CrossRobotChecker | None = None,
    trace: HandoverTrace | None = None,
) -> ObservePlan:
    """제시점 H 를 D405 검증 대역 거리에서 내려다보는 카메라 pose 역산 —
    (방위 오프셋 × 고도 × 거리 × roll ψ) 사다리 resolve (so101 공중 도달이
    좁아 단일 기하는 전멸 실측 — 노브 블록 주석). FK/계획값은 **관측
    유도용으로만** — 파지는 재검출.

    ⚠ **cross-robot 충돌 게이트** (2026-07-28 신설): 이 이동은 omx 가 봉을 들고
    제시 자세로 서 있는 사이에 so101 이 그 코앞(관측 거리 15~18cm)으로 들어가는
    국면인데, 그전까지 **무검사**였다 (게이트가 present/receive/retreat 3곳에만
    있었다 — 실제로 가장 가까워지는 이동 하나가 빠져 있었던 것). 채택안이
    충돌이면 빼고 재-resolve (plan_receive 와 동형 alive-loop).

    ⚠ **시선 가림 게이트** (2026-07-28 신설, 같은 alive-loop): 도달만 보고 1순위
    자세를 채택하면 omx 손목이 봉을 가로지르는 방향을 고를 수 있다 (실물: 봉이
    두 조각으로 갈려 아래 조각만 검출 → 겨냥점 2.8cm 하향 → 수취 IK 전멸).
    offline probe 가 "128 중 44 비가림" 으로 세던 판정을 런타임 게이트로 승격."""
    h_world = present.h_world
    az0 = math.atan2(h_world[1], h_world[0])
    groups: list[list[TcpPose]] = []
    metas: list[tuple[float, float, float, float]] = []
    cams: list[tuple[Vec3, Vec3]] = []  # (계획 카메라 위치, 광축) — groups 와 평행
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
                axis = np.asarray(h_world, dtype=float) - c
                axis = axis / np.linalg.norm(axis)
                g, m = _camera_pose_groups(
                    c, axis, _RECV_OBS_PSI_DEG, t_tcp_cam
                )
                groups.extend(g)
                metas.extend((az_off, elev_deg, dist, psi) for psi in m)
                cams.extend(
                    (
                        (float(c[0]), float(c[1]), float(c[2])),
                        (float(axis[0]), float(axis[1]), float(axis[2])),
                    )
                    for _ in m
                )
    # 충돌 판정 상대 = omx 의 **실제** 구성 (제시 자세로 봉을 들고 서 있다)
    omx_joints: list[float] = []
    so_joints: list[float] = []
    if checker is not None:
        omx_tcp = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=omx
        )
        so_tcp = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=so101
        )
        omx_joints, so_joints = list(omx_tcp.joints), list(so_tcp.joints)
    # 시선 표본은 봉의 **실물 추정** 위치로 (FK + world_offset — 재제시 보정)
    targets = sight_targets(_present_tcp_real(present), present.w, geom)
    alive = list(range(len(groups)))
    chosen = -1
    res = None
    collision_rejects: list[int] = []
    sight_rejects: list[int] = []
    while alive:
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(groups=[groups[i] for i in alive]),
            ResolveReachableResponse,
            robot_id=so101,
        )
        if res.index < 0:
            break
        gi = alive[res.index]
        if checker is not None and checker.path_in_collision(
            [so_joints, res.solutions[0]],
            omx_joints,
            grip_a=1.0,
            grip_b=_OMX_HOLD_GRIP_FRAC,
            margin_m=_RECV_COLLISION_MARGIN_M,
        ):
            logger.warning(
                "plan_so_observe: 그룹 %d 채택안 omx 충돌 위험 — 기각", gi
            )
            collision_rejects.append(gi)
            alive.remove(gi)
            continue
        if checker is not None and checker.sight_blocked(
            res.solutions[0],
            omx_joints,
            cams[gi][0],
            targets,
            grip_a=1.0,
            grip_b=_OMX_HOLD_GRIP_FRAC,
        ):
            logger.warning(
                "plan_so_observe: 그룹 %d (az_off=%.0f° elev=%.0f° dist=%.2f "
                "ψ=%.0f°) 노출부 시선 가림 — 기각",
                gi,
                *metas[gi],
            )
            sight_rejects.append(gi)
            alive.remove(gi)
            continue
        chosen = gi
        break
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "plan_so_observe",
            "h_world": list(h_world),
            "index": chosen,
            "meta": metas[chosen] if chosen >= 0 else None,
            "n_groups": len(groups),
            "collision_rejects": collision_rejects,
            "sight_rejects": sight_rejects,
            "sight_targets": [list(t) for t in targets],
            # 실물 원인분석용 — 명령 관절과 계획 카메라 pose 를 남긴다 (도달
            # 검증이 이 값과 실측 FK 를 비교한다. 2026-07-28)
            "joints": [round(v, 5) for v in res.solutions[0]]
            if chosen >= 0 and res is not None
            else None,
            "cam_pos": list(cams[chosen][0]) if chosen >= 0 else None,
            "cam_axis": [round(v, 4) for v in cams[chosen][1]]
            if chosen >= 0
            else None,
        },
    )
    if chosen < 0 or res is None:
        raise NoReachableGrasp(
            f"so101 수취 관측 자세 전멸 ({len(groups)}개 — omx 충돌 기각 "
            f"{len(collision_rejects)}개, 시선 가림 기각 {len(sight_rejects)}개) — "
            f"{res.message if res is not None else ''}. 가림 기각이 대부분이면 "
            "관측 방위 사다리(_RECV_OBS_AZOFF_DEG)를 넓히거나 제시 높이"
            "(_PRESENT_Z_WORLD)를 조정하세요"
        )
    logger.info(
        "plan_so_observe: az_off=%.0f° elev=%.0f° dist=%.2f ψ=%.0f° 채택 "
        "(충돌 기각 %d, 가림 기각 %d) — 계획 카메라 (%.3f,%.3f,%.3f)",
        *metas[chosen],
        len(collision_rejects),
        len(sight_rejects),
        *cams[chosen][0],
    )
    return ObservePlan(
        joints=res.solutions[0],
        cam_pos=cams[chosen][0],
        cam_axis=cams[chosen][1],
    )


def detection_centroid(det: OrientedDetection) -> Vec3:
    """검출 물체의 **점군 centroid** (world) — 축 무관 위치 추정.

    ⚠ 축 일반화(2026-07-28): 옛 `aerial_target` 은 `base_z + height/2` 로 z 대역
    중간을 썼는데 그건 **봉이 수직일 때만** 의미가 있다 (수평 봉의 z 대역은
    단면 2cm 라 중간값이 축 위 위치를 못 준다). 점군 centroid 는 자세와 무관하고,
    가림으로 반쪽만 보여 centroid 가 축 방향으로 밀리는 오차는 하류에서
    **축 성분을 앵커로 덮어** 무해화한다 (_axis_corrected_target)."""
    pts = det.points or []
    if not pts:
        return (det.position[0], det.position[1], det.position[2])
    c = np.mean(np.asarray(pts, dtype=float), axis=0)
    return (float(c[0]), float(c[1]), float(c[2]))


def _axis_split(v: Vec3, axis: Vec3) -> tuple[float, Vec3]:
    """벡터를 축 성분(스칼라)과 축에 수직인 성분(벡터)으로 분해."""
    a = _unit(np.asarray(axis, dtype=float))
    vv = np.asarray(v, dtype=float)
    along = float(np.dot(vv, a))
    perp = vv - along * a
    return along, (float(perp[0]), float(perp[1]), float(perp[2]))


def _match_aerial(
    cands: list[OrientedDetection], tcp_world: Vec3, w: Vec3, geom: BlockGrasp
) -> OrientedDetection | None:
    """공중 봉 매치 — **계획 봉 세그먼트와의 3D 정합** + score/점군 게이트.

    옛 게이트(계획점 xy 반경 + z 대역)는 봉이 수직일 때만 맞는 판정이었다. 축
    일반형: centroid 를 (축 방향 s, 축 수직 거리 d) 로 분해해
      · d ≤ _RECV_MATCH_RADIUS_M     (봉 축에서 얼마나 벗어났나)
      · −margin ≤ s ≤ exposed_len+margin  (노출 세그먼트 안인가)
    로 본다. 축 방향은 가림에 따라 밀리는 값이라 관대하게, **축 수직 거리는
    엄격하게** — 수직 오차가 곧 수취 IK 를 죽이는 성분이다 (2026-07-28 실물:
    관대한 z 밴드 60mm 가 48.6mm 오차를 침묵 통과시켜 전멸)."""
    base = np.asarray(tcp_world, dtype=float) + np.asarray(
        w, dtype=float
    ) * (_OMX_JAW_ALONG_M / 2.0)
    lo = -_RECV_Z_BAND_M
    hi = geom.exposed_len_m + _RECV_Z_BAND_M
    trusted: list[OrientedDetection] = []
    for c in cands:
        if c.score < _RECV_SCORE_MIN or len(c.points or []) < _RECV_MIN_POINTS:
            continue
        d = np.asarray(detection_centroid(c), dtype=float) - base
        s, perp = _axis_split((float(d[0]), float(d[1]), float(d[2])), w)
        if float(np.linalg.norm(perp)) > _RECV_MATCH_RADIUS_M:
            continue
        if not (lo <= s <= hi):
            continue
        trusted.append(c)
    return max(trusted, key=lambda c: c.score) if trusted else None


@step(title="수취 재검출")
async def so_redetect(
    ctx: TaskContext,
    so101: str,
    prompt: str,
    observe: ObservePlan,
    present: PresentPlan,
    geom: BlockGrasp,
    t_tcp_cam: np.ndarray,
    trace: HandoverTrace | None = None,
) -> OrientedDetection:
    """관측 자세 이동 → **도달 검증** → 공중의 제시된 봉 재검출. 실패 = 명시
    실패 (FK 로 후퇴하지 않는다 — §8-4: 정적 계산 ~1–2cm 자세의존 오차가
    so101 이 closed-loop 로 간 이유 그 자체).

    ⚠ 도달 검증 (2026-07-28 실물): 이동 후 실측 FK 카메라 pose 를 계획과 대조해
    임계(_OBS_POS_TOL_M / _OBS_AXIS_TOL_DEG) 초과면 **검출 전에** 실패한다.
    그날 so101 은 계획과 위치 14cm·광축 40° 다른 자세에 있었고 봉은 광축 86°
    밖이었는데, 실패는 6초 뒤 "검출 실패"로 나타나 원인이 위장됐다."""
    h_world = present.h_world
    await _move_j(ctx, so101, joints=observe.joints)
    await asyncio.sleep(_OBSERVE_SETTLE_S)
    tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=so101
    )
    cam_pos, cam_axis = _camera_pose_of(tcp, t_tcp_cam)
    pos_err = float(
        np.linalg.norm(np.asarray(cam_pos) - np.asarray(observe.cam_pos))
    )
    axis_err = _axis_error_deg(cam_axis, observe.cam_axis)
    target_off = _axis_error_deg(
        cam_axis,
        (
            h_world[0] - cam_pos[0],
            h_world[1] - cam_pos[1],
            h_world[2] - cam_pos[2],
        ),
    )
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "so_observe_reached",
            "cam_pos_planned": list(observe.cam_pos),
            "cam_pos_actual": list(cam_pos),
            "cam_axis_planned": [round(v, 4) for v in observe.cam_axis],
            "cam_axis_actual": [round(v, 4) for v in cam_axis],
            "pos_err_mm": round(pos_err * 1000, 1),
            "axis_err_deg": round(axis_err, 1),
            # 겨냥점이 광축에서 몇 도 벗어났나 — FOV(±43°) 밖이면 프레임 부재
            "target_off_axis_deg": round(target_off, 1),
            "joints_commanded": [round(v, 5) for v in observe.joints],
            "joints_actual": [round(v, 5) for v in tcp.joints],
        },
    )
    if pos_err > _OBS_POS_TOL_M or axis_err > _OBS_AXIS_TOL_DEG:
        raise TaskError(
            f"so101 이 계획한 수취 관측 자세에 도달하지 못했습니다 — 카메라 위치 "
            f"오차 {pos_err * 1000:.0f}mm (허용 {_OBS_POS_TOL_M * 1000:.0f}) · "
            f"광축 오차 {axis_err:.1f}° (허용 {_OBS_AXIS_TOL_DEG:.0f}) · 겨냥점이 "
            f"광축에서 {target_off:.0f}° 벗어남. 검출을 시도하지 않고 멈췄습니다 "
            "(그대로 가면 '검출 실패'로 위장됩니다). trace 의 joints_commanded vs "
            "joints_actual 을 비교하세요 — 어긋나면 모터/토크·MoveJ 실행 문제, "
            "같으면 관측 pose→관절 변환(hand_eye/IK) 문제입니다"
        )
    res = await ctx.call(
        Detector.Service.DETECT_ORIENTED,
        DetectRequest(
            robot_id=so101, prompts=[prompt], top_k=_TOP_K,
            # 노출부(자유단) 군집 선호 — 조 안쪽 조각 채택 방지. 매달기에선
            # 자유단이 아래라 "bottom" 이 맞고, 수평 제시에선 축 성분을 하류가
            # 앵커로 덮으므로(_axis_corrected) 어느 조각이 와도 무해하다.
            body_select="bottom",
        ),
        DetectOrientedResponse,
    )
    # 매치/세그먼트 기준 = 봉 실물 추정 (FK + world_offset — 재제시 보정)
    tcp_real = _present_tcp_real(present)
    seg_base = np.asarray(tcp_real, dtype=float) + np.asarray(
        present.w, dtype=float
    ) * (_OMX_JAW_ALONG_M / 2.0)
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "so_redetect",
            "h_world": list(h_world),
            "w_world": [round(v, 4) for v in present.w],
            "candidates": [
                {
                    "position": list(c.position),
                    "centroid": [round(v, 4) for v in detection_centroid(c)],
                    # 계획 봉 세그먼트 기준 (축 방향 s, 축 수직 거리) — 매치
                    # 게이트가 보는 값 그대로 (실패 시 어느 쪽이 컸는지 즉시)
                    "seg_along_mm": round(
                        _axis_split(
                            tuple(  # type: ignore[arg-type]
                                float(v)
                                for v in np.asarray(detection_centroid(c)) - seg_base
                            ),
                            present.w,
                        )[0] * 1000,
                        1,
                    ),
                    "seg_perp_mm": round(
                        float(
                            np.linalg.norm(
                                _axis_split(
                                    tuple(  # type: ignore[arg-type]
                                        float(v)
                                        for v in np.asarray(detection_centroid(c))
                                        - seg_base
                                    ),
                                    present.w,
                                )[1]
                            )
                        ) * 1000,
                        1,
                    ),
                    "score": c.score,
                    "points": len(c.points or []),
                    "yaw_deg": round(math.degrees(c.grasp_yaw), 1),
                }
                for c in res.candidates
            ],
        },
    )
    best = _match_aerial(res.candidates, tcp_real, present.w, geom)
    if best is None:
        raise DetectionNotFound(
            prompt,
            candidates=len(res.candidates),
            reason=(
                f"공중 재검출 매치 실패 — 계획 봉 세그먼트(TCP {present.tcp_world} "
                f"방향 {present.label}) 기준 축 수직 거리 ≤"
                f"{_RECV_MATCH_RADIUS_M * 1000:.0f}mm · 축 방향 −"
                f"{_RECV_Z_BAND_M * 1000:.0f}~{(geom.exposed_len_m + _RECV_Z_BAND_M) * 1000:.0f}mm"
                f" · score≥{_RECV_SCORE_MIN} · 점군≥{_RECV_MIN_POINTS}. trace 의 "
                "seg_along_mm / seg_perp_mm 로 어느 게이트가 걸렸는지 보세요 "
                "(제시 자세/조명/가림 확인 후 다시 실행)"
            ),
        )
    return best


def represent_offset(det: OrientedDetection, h_ref: Vec3, w: Vec3) -> Vec3:
    """실측 봉 위치의 **축 수직·수평 이탈** p (world) — 재제시 보정량의 원천.

    p = perp(검출 centroid − h_ref) 의 **수평 성분** (z=0). h_ref = 현 제시
    계획의 h_world (= FK E + 누적 world_offset — **실물 추정 좌표**라
    systematic 오차가 이미 빠져 있어 잔차만 남는다. FK 좌표 대비로 재면 오차가
    매번 다시 보여 수렴 불가). 새 누적 offset = present.world_offset + p →
    plan_omx_present 재계획 입력.

    버리는 성분 둘 다 "아는 기하는 앵커, 검출은 보정" 클래스:
    · 축 방향 — 가림에 밀리는 값 + 봉 위 어디를 잡아도 무방 (§4.8).
    · **z — 마스크 가장자리 depth 번짐이 centroid 를 아래로 끌어내린다**
      (2026-07-29 21:11 실측: centroid z 0.187 = 봉 바닥면보다 3mm 아래,
      점군 하한 0.167 = 책상 방향 번짐. 봉은 강체로 omx TCP 에 물려 있어
      z 는 FK 가 훨씬 정확). 실물 런에서 수취가 일관되게 z 로 긁으면 이
      가정(그립 sag 무시)을 재검토."""
    _along, perp = _axis_split(
        tuple(  # type: ignore[arg-type]
            float(v)
            for v in np.asarray(detection_centroid(det), dtype=float)
            - np.asarray(h_ref, dtype=float)
        ),
        w,
    )
    return (perp[0], perp[1], 0.0)


@dataclass(frozen=True, slots=True)
class MeasuredBar:
    """so101 재검출 점군으로 **실측한** 봉 — 수취 계획의 기준 (계획 가정 아님).

    2026-07-29 근인: 봉이 omx 조 안에서 ~90° 돌았는데 (미믹 약파지 + 재배향
    중력토크 — 점군 주축 vs 계획축 84°/70° 실측) 수취가 계획축 w 기준 자세족
    +겨냥점을 만들어 전멸했다. 검출은 돌아간 봉을 정확히 보고 있었다 (마스크
    분홍 99%, 주축 span 85mm = 봉 전체). → "눈을 믿는다": 축·파지점 전부 실측.
    """

    axis: Vec3  # 실측 봉 축 (world 단위, **자유단 방향** — omx TCP 반대쪽)
    target: Vec3  # so101 파지 겨냥점 = 실측 자유단에서 inset 안쪽
    span_m: float  # 점군 주축 span (5~95 백분위) — 품질 지표
    n_points: int
    axis_dev_deg: float  # 계획축(present.w) 대비 이탈각 — "조 안 회전" 지문
    fallback: bool  # True = 점군 빈약 → 계획축+FK 앵커 폴백 (trace 사유)


@step(title="봉 실측")
async def measure_bar(
    ctx: TaskContext,
    omx: str,
    base_omx: BasePose,
    det: OrientedDetection,
    present: PresentPlan,
    geom: BlockGrasp,
    trace: HandoverTrace | None = None,
) -> MeasuredBar:
    """재검출 점군 PCA 로 봉 축/자유단/파지점 실측 (world).

    - 축 방향: 주축 부호를 "omx TCP 에서 멀어지는 쪽 = 자유단" 으로 고정.
    - 끝점: 주축 투영의 5/95 백분위 (마스크 가장자리 depth 번짐 완화).
    - 파지점: 자유단에서 _GRASP_TIP_INSET_M 안쪽 (조 접촉폭 + 끝 여백).
    - 품질 게이트: 점군 수/span 미달 → **계획축+FK 앵커 폴백** (옛 plan_receive
      규약 그대로 — 축성분 앵커, 축수직 xy 만 검출 보정). 폴백 사유는 trace.
    - sanity: 파지점-omx TCP 거리가 봉 길이 밖 → 명시 실패 (검출이 봉이 아님
      또는 FK/base_pose 대붕괴 — 그대로 가면 허공/충돌).
    """
    omx_tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=omx
    )
    # 실물 추정 좌표 = **실측 FK**(settle/sag 반영, 명령값 아님) + world_offset
    # — so101 눈 실측과 같은 공간 (옛 plan_receive 앵커 규약 계승)
    fk_w = robot_to_world(omx_tcp.position, base_omx)
    tcp_real = (
        fk_w[0] + present.world_offset[0],
        fk_w[1] + present.world_offset[1],
        fk_w[2] + present.world_offset[2],
    )
    w_plan = np.asarray(present.w, dtype=float)
    pts = np.asarray(det.points or [], dtype=float)
    cen = np.asarray(detection_centroid(det), dtype=float)

    fallback_reason: str | None = None
    axis = w_plan
    span = 0.0
    lo = hi = 0.0  # 주축 투영 5/95 백분위 (실측 성공 시에만 의미)
    near_free: tuple[np.ndarray, np.ndarray] | None = None
    if len(pts) < _RECV_MIN_POINTS:
        fallback_reason = f"점군 {len(pts)} < {_RECV_MIN_POINTS}"
    else:
        c = pts - pts.mean(axis=0)
        _w, v = np.linalg.eigh(c.T @ c / len(pts))
        main = v[:, int(np.argmax(_w))]
        proj = c @ main
        lo, hi = float(np.percentile(proj, 5)), float(np.percentile(proj, 95))
        span = hi - lo
        if span < _BAR_AXIS_MIN_SPAN_M:
            fallback_reason = (
                f"주축 span {span * 1000:.0f}mm < "
                f"{_BAR_AXIS_MIN_SPAN_M * 1000:.0f}mm"
            )
        else:
            # 자유단 방향 고정 — centroid 는 노출부 위라 omx TCP 반대쪽이 +
            if float(np.dot(main, cen - np.asarray(tcp_real))) < 0:
                main, lo, hi = -main, -hi, -lo
            axis = main

    if fallback_reason is not None:
        # 옛 규약 폴백: 축성분은 FK 앵커, 축수직 xy 만 검출 보정
        anchor = np.asarray(tcp_real, dtype=float) + w_plan * geom.tcp_to_e_m
        along, perp_raw = _axis_split(
            tuple(float(v) for v in cen - anchor),  # type: ignore[arg-type]
            present.w,
        )
        target = (
            float(anchor[0] + perp_raw[0]),
            float(anchor[1] + perp_raw[1]),
            float(anchor[2]),  # z 도 앵커 (depth 번짐)
        )
        logger.warning(
            "measure_bar: 축 실측 불가 (%s) — 계획축+FK 앵커 폴백 "
            "(축방향 밀림 %.0fmm 은 앵커로 덮음)", fallback_reason, along * 1000,
        )
        meas = MeasuredBar(
            axis=tuple(float(v) for v in w_plan),  # type: ignore[arg-type]
            target=target, span_m=span, n_points=len(pts),
            axis_dev_deg=0.0, fallback=True,
        )
    else:
        mean = pts.mean(axis=0)
        free_end = mean + axis * hi
        near_end = mean + axis * lo
        tgt = free_end - axis * _GRASP_TIP_INSET_M
        dev = _axis_error_deg(
            tuple(float(v) for v in axis),  # type: ignore[arg-type]
            present.w,
        )
        if dev > _BAR_AXIS_DEV_WARN_DEG:
            logger.warning(
                "measure_bar: 실측 봉축이 계획축에서 %.0f° 이탈 — 봉이 omx 조 "
                "안에서 돌았습니다 (미믹 약파지 지문). 실측축 기준으로 수취 진행",
                dev,
            )
        meas = MeasuredBar(
            axis=tuple(float(v) for v in axis),  # type: ignore[arg-type]
            target=(float(tgt[0]), float(tgt[1]), float(tgt[2])),
            span_m=span, n_points=len(pts), axis_dev_deg=dev, fallback=False,
        )
        near_free = (near_end, free_end)  # trace 용
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "measure_bar",
            # ── 실물 원인분석 전량 (2026-07-29 "anchor 미기록" 재발 방지) ──
            "omx_tcp_fk_world": [round(v, 4) for v in fk_w],
            "omx_joints": [round(v, 5) for v in omx_tcp.joints],
            "tcp_real_world": [round(v, 4) for v in tcp_real],
            "world_offset_mm": [
                round(v * 1000, 1) for v in present.world_offset
            ],
            "w_plan": [round(v, 4) for v in present.w],
            "centroid": [round(v, 4) for v in cen] if len(pts) else None,
            "n_points": len(pts),
            "axis_meas": [round(v, 4) for v in meas.axis],
            "axis_span_mm": round(span * 1000, 1),
            "axis_dev_deg": round(meas.axis_dev_deg, 1),
            "endpoints": [
                [round(float(x), 4) for x in p] for p in near_free
            ] if near_free is not None else None,
            "target": [round(v, 4) for v in meas.target],
            "fallback": fallback_reason,
        },
    )
    # sanity — 봉은 omx TCP 에 물린 강체다: 파지점이 봉 길이 밖 = 오검출/FK 붕괴
    d_tcp = math.dist(meas.target, tcp_real)
    if not (0.015 <= d_tcp <= geom.length_m + 0.08):
        raise TaskError(
            f"실측 파지점이 omx TCP 에서 {d_tcp * 100:.1f}cm — 봉 길이"
            f"({geom.length_m * 100:.0f}cm) 기준 비정상. 재검출이 봉이 아닌 것을 "
            "물었거나 omx FK/base_pose 가 크게 어긋났습니다. trace 의 "
            "measure_bar(centroid/tcp_real_world)와 detect 이미지를 대조하세요"
        )
    return meas


@dataclass(frozen=True, slots=True)
class ReceivePlan:
    sols: list[list[float]]  # [pre, grasp] 관절해
    quat: Quat
    target: Vec3  # 파지 겨냥점 (world) = 재검출 노출부 중심 (≈E)
    omx_joints: list[float]
    pre_clear_m: float  # 채택된 접근 여유 (사다리 — 실물 보정 데이터)
    axis: Vec3 = (0.0, 0.0, -1.0)  # 봉 축 (실측 우선) — refine 의 축수직 분해 기준


@step(title="수취 계획")
async def plan_receive(
    ctx: TaskContext,
    so101: str,
    omx: str,
    meas: MeasuredBar,
    present: PresentPlan,
    geom: BlockGrasp,
    checker: CrossRobotChecker | None,
    trace: HandoverTrace | None = None,
) -> ReceivePlan:
    """실측 봉(measure_bar) 기반 수취 계획 — **잡기 조건 그리드**(_grasp_orients
    — 조 닫힘축 ⊥ 실측 봉축, 2 자유도 전개) resolve + **충돌 게이트** + **벽(뒤)
    게이트**. 채택 그룹이 충돌/벽이면 빼고 재-resolve (상한 소진 = 명시 실패).

    2026-07-30 재설계 (사용자 지시 — "정확 자세 목록 전멸=종료" 폐기):
    · 축/겨냥점 = **실측** (measure_bar) — 봉이 조 안에서 돌아도 돌아간 그대로
      잡는다 (2026-07-29 근인: 계획축 가정이 실물과 84°/70° 어긋나 전멸).
    · 자세족 = 잡기 조건만 (과잉 제약 제거 — full_diagnosis.py: 정확족 0~7해
      지점에서 조건족 10~17해).
    · 전멸 = 종료가 아니라 **협상** — 호출자(module)가 find_receive_shift 로
      "잡을 수 있는 가장 가까운 봉 위치"를 역산해 omx 재배치 후 재시도.
    · 실패 시에도 trace 에 입력/후보별 사유 전량 (anchor 미기록 재발 방지)."""
    omx_tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=omx
    )
    omx_joints = list(omx_tcp.joints)
    target = meas.target
    orients = _grasp_orients(target, meas.axis)
    groups: list[list[TcpPose]] = []
    metas: list[tuple[str, Quat, float, Vec3]] = []  # (라벨, quat, 여유, 접근)
    # 자세-major × 접근 여유 사다리 — 자세 품질이 standoff 길이보다 우선이라
    # 선호 자세를 전 여유에서 먼저 소진한다 (_RECV_PRE_CLEAR_LADDER 주석).
    for label, quat, a in orients:  # 접근 = base→E 방위 근접순 (so101 쪽 진입)
        for clear in _RECV_PRE_CLEAR_LADDER:
            pre = (
                target[0] - a[0] * clear,
                target[1] - a[1] * clear,
                target[2] - a[2] * clear,
            )
            groups.append(
                [
                    TcpPose(position=pre, quaternion=quat),
                    TcpPose(position=target, quaternion=quat),
                ]
            )
            metas.append((label, quat, clear, a))
    logger.info(
        "plan_receive: 겨냥점 (%.3f,%.3f,%.3f) 실측축 (%.2f,%.2f,%.2f) "
        "(계획축 이탈 %.0f°%s) — 후보 %d (자세 %d × 여유 %d)",
        target[0], target[1], target[2],
        meas.axis[0], meas.axis[1], meas.axis[2],
        meas.axis_dev_deg,
        ", 폴백" if meas.fallback else "",
        len(groups), len(orients), len(_RECV_PRE_CLEAR_LADDER),
    )
    alive = list(range(len(groups)))
    gate_rejects: list[str] = []  # 충돌/벽 기각 이력 (trace)
    res = None
    for attempt in range(_RECV_COLLISION_RETRY):
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(groups=[groups[i]
                                    for i in alive], linear=True),
            ResolveReachableResponse,
            robot_id=so101,
        )
        if res.index < 0:
            break
        gi = alive[res.index]
        # 벽(뒤) — so101 grasp 해가 베이스 뒤로 넘어가면 기각 (side="a")
        if checker is not None and _behind_wall(checker, "a", res.solutions[-1]):
            gate_rejects.append(f"{metas[gi][0]}/clr{metas[gi][2] * 1000:.0f}: 벽(뒤)")
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
                    "axis": [round(v, 4) for v in meas.axis],
                    "axis_dev_deg": round(meas.axis_dev_deg, 1),
                    "group": gi,
                    "orientation": metas[gi][0],
                    "pre_clear_m": metas[gi][2],
                    "attempt": attempt,
                    "gate_rejects": gate_rejects,
                    "omx_joints": [round(v, 5) for v in omx_joints],
                    "sol_pre": [round(v, 5) for v in res.solutions[0]],
                    "sol_grasp": [round(v, 5) for v in res.solutions[-1]],
                },
            )
            logger.info(
                "plan_receive: %s 채택 (접근 여유 %.0fmm — 사다리 %s)",
                metas[gi][0],
                metas[gi][2] * 1000,
                [round(c * 1000) for c in _RECV_PRE_CLEAR_LADDER],
            )
            return ReceivePlan(
                sols=res.solutions,
                quat=metas[gi][1],
                target=target,
                omx_joints=omx_joints,
                pre_clear_m=metas[gi][2],
                axis=meas.axis,
            )
        gate_rejects.append(
            f"{metas[gi][0]}/clr{metas[gi][2] * 1000:.0f}: omx 충돌"
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
    # 전멸 — 원인분석 전량을 trace 에 (2026-07-29 "sim 7 vs 실물 0" 미스터리의
    # 직접 재발 방지: 실패 시점의 겨냥점/축/솔버 사유가 파일로 남는다)
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "plan_receive_dead",
            "target": list(target),
            "axis": [round(v, 4) for v in meas.axis],
            "axis_dev_deg": round(meas.axis_dev_deg, 1),
            "axis_fallback": meas.fallback,
            "n_groups": len(groups),
            "n_alive_last": len(alive),
            "gate_rejects": gate_rejects,
            "resolve_message": res.message if res is not None else None,
            "group_failures": list(
                zip(
                    [f"{m[0]}/clr{m[2] * 1000:.0f}" for m in
                     (metas[i] for i in alive)],
                    res.group_failures,
                )
            )[:24] if res is not None and res.group_failures else None,
            "omx_joints": [round(v, 5) for v in omx_joints],
        },
    )
    raise NoReachableGrasp(
        f"수취 후보 전멸 (잡기조건 그리드 {len(groups)}개"
        f"{', 게이트 기각 ' + str(len(gate_rejects)) if gate_rejects else ''}) — "
        f"{res.message if res is not None else '게이트 전멸'}. trace 의 "
        "plan_receive_dead(겨냥점/실측축/후보별 사유)를 확인하세요 — 협상 상한 "
        "소진 시 이 오류가 최종 보고됩니다"
    )


@step(title="협상 — 이동 제안")
async def find_receive_shift(
    ctx: TaskContext,
    so101: str,
    meas: MeasuredBar,
    trace: HandoverTrace | None = None,
) -> Vec3 | None:
    """수취 전멸 시 so101 의 되받기: "봉이 δ 만큼 옮겨지면 잡을 수 있다".

    2026-07-30 신설 (사용자 지시 — 전멸=종료 폐기, 양팔 협상): 현 겨냥점 주변
    격자(δ, |δ|≤_NEGOTIATE_RANGE_M)를 작은 이동 우선으로 훑어, **잡기 조건
    그리드 축약본**(선호 24자세 × 여유 2단)이 사는 첫 δ 를 반환. None = 주변
    전부 죽음 (호출자가 명시 실패). 히트맵 스위트밴드(_RECV_SWEET_XY) 방향을
    동률 우선 — 방향 자유도가 좋은 쪽으로 옮긴다 (full_diagnosis.py 실측).

    충돌/벽 게이트는 여기서 안 건다 — δ 채택 후 재관측→재계획(plan_receive)이
    풀 게이트로 다시 판정한다 (여긴 존재 증명만)."""
    steps_1d = [0.0]
    k = 1
    while k * _NEGOTIATE_STEP_M <= _NEGOTIATE_RANGE_M + 1e-9:
        steps_1d += [k * _NEGOTIATE_STEP_M, -k * _NEGOTIATE_STEP_M]
        k += 1
    deltas = [
        (dx, dy, dz)
        for dx in steps_1d
        for dy in steps_1d
        for dz in (0.0, _NEGOTIATE_STEP_M, -_NEGOTIATE_STEP_M)
        if (dx, dy, dz) != (0.0, 0.0, 0.0)
        and math.sqrt(dx * dx + dy * dy + dz * dz) <= _NEGOTIATE_RANGE_M + 1e-9
    ]
    deltas.sort(
        key=lambda d: (
            math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2),
            math.hypot(
                meas.target[0] + d[0] - _RECV_SWEET_XY[0],
                meas.target[1] + d[1] - _RECV_SWEET_XY[1],
            ),
        )
    )
    tried: list[dict] = []
    for delta in deltas:
        t = (
            meas.target[0] + delta[0],
            meas.target[1] + delta[1],
            meas.target[2] + delta[2],
        )
        groups = []
        for _label, quat, a in _grasp_orients(t, meas.axis)[:24]:
            for clear in _RECV_PRE_CLEAR_LADDER[1::2]:  # (0.05, 0.03)
                pre = (t[0] - a[0] * clear, t[1] - a[1] * clear,
                       t[2] - a[2] * clear)
                groups.append([
                    TcpPose(position=pre, quaternion=quat),
                    TcpPose(position=t, quaternion=quat),
                ])
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(groups=groups, linear=True),
            ResolveReachableResponse,
            robot_id=so101,
        )
        tried.append({
            "delta_mm": [round(v * 1000) for v in delta],
            "ok": res.index >= 0,
        })
        if res.index >= 0:
            logger.info(
                "find_receive_shift: δ=(%+.0f,%+.0f,%+.0f)mm 에서 수취 해 존재 "
                "— omx 재배치 제안 (%d개 지점 탐색)",
                delta[0] * 1000, delta[1] * 1000, delta[2] * 1000, len(tried),
            )
            await _emit(
                trace,
                {
                    "phase": "receive",
                    "event": "negotiate_shift",
                    "delta_mm": [round(v * 1000) for v in delta],
                    "target_from": list(meas.target),
                    "tried": tried,
                },
            )
            return delta
    await _emit(
        trace,
        {
            "phase": "receive",
            "event": "negotiate_shift_dead",
            "target_from": list(meas.target),
            "tried": tried,
        },
    )
    logger.warning(
        "find_receive_shift: ±%.0fcm 격자 %d개 지점 전부 수취 해 없음",
        _NEGOTIATE_RANGE_M * 100, len(tried),
    )
    return None


@step(title="omx 재배치 (협상)")
async def omx_nudge(
    ctx: TaskContext,
    omx: str,
    so101: str,
    base_omx: BasePose,
    present: PresentPlan,
    delta: Vec3,
    checker: CrossRobotChecker | None,
    trace: HandoverTrace | None = None,
) -> PresentPlan:
    """협상 응답: omx 가 봉을 δ(world) 만큼 평행이동 — 자세(축) 불변.

    so101 되받기(find_receive_shift)가 "여기면 잡는다" 한 지점으로 봉을 옮긴다.
    자세가 그대로라 실측축도 그대로 유효 — 이동 후 재관측→재실측이 잔차를
    다시 잡는다. δ 도달 불가면 δ/2 폴백 1회 (부분 접근도 다음 협상 라운드의
    시작점을 개선), 그것도 죽으면 명시 실패."""
    so_tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
        robot_id=so101,
    )
    omx_tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
        robot_id=omx,
    )
    for frac in (1.0, 0.5):
        d = (delta[0] * frac, delta[1] * frac, delta[2] * frac)
        new_tcp_w = (
            present.tcp_world[0] + d[0],
            present.tcp_world[1] + d[1],
            present.tcp_world[2] + d[2],
        )
        tcp_omx = world_to_robot(new_tcp_w, base_omx)
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=[[TcpPose(position=tcp_omx, quaternion=present.quat)]]
            ),
            ResolveReachableResponse,
            robot_id=omx,
        )
        reason = None
        if res.index < 0:
            reason = f"omx IK 없음 ({res.message})"
        elif abs(res.solutions[0][-1]) > _WRIST_NATURAL_MAX_RAD:
            reason = f"손목 뒤집힘 (J5={math.degrees(res.solutions[0][-1]):.0f}°)"
        elif checker is not None and _behind_wall(checker, "b", res.solutions[0]):
            reason = "omx 벽(뒤) 침범"
        elif checker is not None and _omx_path_collides(
            checker, list(so_tcp.joints),
            [list(omx_tcp.joints), res.solutions[0]],
        ):
            reason = "so101 충돌 위험"
        await _emit(
            trace,
            {
                "phase": "present",
                "event": "omx_nudge",
                "delta_mm": [round(v * 1000) for v in d],
                "tcp_world": list(new_tcp_w),
                "ok": reason is None,
                "reason": reason,
            },
        )
        if reason is not None:
            logger.warning(
                "omx_nudge: δ×%.1f=(%+.0f,%+.0f,%+.0f)mm 기각 — %s",
                frac, d[0] * 1000, d[1] * 1000, d[2] * 1000, reason,
            )
            continue
        await _move_j(ctx, omx, joints=res.solutions[0])
        await verify_grasp(
            ctx, omx, phase="협상 재배치 도달", trace=trace, advisory=True
        )
        return PresentPlan(
            sols=res.solutions,
            quat=present.quat,
            h_world=(
                present.h_world[0] + d[0],
                present.h_world[1] + d[1],
                present.h_world[2] + d[2],
            ),
            tcp_world=new_tcp_w,
            w=present.w,
            label=f"{present.label}+nudge",
            world_offset=present.world_offset,
        )
    raise NoReachableGrasp(
        f"협상 재배치 실패 — omx 가 δ=({delta[0] * 1000:+.0f},"
        f"{delta[1] * 1000:+.0f},{delta[2] * 1000:+.0f})mm (및 절반) 이동 불가. "
        "trace 의 omx_nudge 기각 사유를 확인하세요 (봉/랑데부를 물리적으로 "
        "옮긴 후 다시 실행이 차선)"
    )


@step(title="수취 보정")
async def so_refine(
    ctx: TaskContext,
    so101: str,
    prompt: str,
    plan: ReceivePlan,
    present: PresentPlan,
    geom: BlockGrasp,
    trace: HandoverTrace | None = None,
) -> Vec3:
    """pre 도달 후 재검출 1 tick — 겨냥점 갱신 (look-then-move 최소형: 측정
    자세와 실행 자세가 가까워 common-mode 상쇄). 실패 = 계획 겨냥점 유지
    (로그+trace — 침묵 금지).

    갱신도 **축 수직 성분만** — plan.target 의 축 위 위치는 실측 자유단(또는
    FK 앵커 폴백)에서 온 값이라 검출 centroid 의 축 방향 밀림(가림 의존)으로
    덮지 않는다. 분해 축 = **plan.axis (실측 봉축)** — 계획축이 아니라 봉이
    실제로 뻗은 방향 기준 (2026-07-30 measure_bar 규약)."""
    await asyncio.sleep(_OBSERVE_SETTLE_S)
    res = await ctx.call(
        Detector.Service.DETECT_ORIENTED,
        DetectRequest(
            robot_id=so101, prompts=[prompt], top_k=_TOP_K,
            body_select="bottom",  # 노출부(자유단) 군집 선호
        ),
        DetectOrientedResponse,
    )
    best = _match_aerial(res.candidates, _present_tcp_real(present), present.w, geom)
    if best is None:
        reason = "수취 refine 재검출 실패 — 계획 겨냥점으로 진행"
        logger.warning("so_refine: %s", reason)
        await _emit(
            trace, {"phase": "receive",
                    "event": "refine_miss", "reason": reason}
        )
        return plan.target
    cen = np.asarray(detection_centroid(best), dtype=float)
    _along, perp = _axis_split(
        tuple(  # type: ignore[arg-type]
            float(v) for v in cen - np.asarray(plan.target, dtype=float)
        ),
        plan.axis,
    )
    updated = (
        plan.target[0] + perp[0],
        plan.target[1] + perp[1],
        plan.target[2],  # z 앵커 — 검출 z 는 번짐 오염 (plan_receive 동일 규약)
    )
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
    present: PresentPlan,
    geom: BlockGrasp,
    trace: HandoverTrace | None = None,
) -> None:
    """so101 접근(pre 관절해) → **pre 에서 look-then-move servo** (수렴까지
    측정-보정 반복, PnP servo 이식) → 진입(감속) → close →
    **held 확인 후에만** omx open → so101 이탈(감속).

    ⚠ refine 1 tick + open-loop 진입(옛 코드)은 2026-07-29 22:06 실물에서
    허공을 물었다 — 연속 두 측정이 12.5mm 어긋나는(점군 52개) 산포를 한 번
    보고 믿은 것. servo 규약 (PnP servo.py 동일): 정지 측정 → lateral 오차
    ≤ eps 면 진입, 아니면 **보정된 pre 로 이동 후 재측정** (측정·명령이 같은
    자세 근방 = eye-in-hand common-mode 상쇄). 상한 소진 = 마지막 측정으로
    진입 (경고 — 침묵 금지).

    수취 순서 불변식 (모듈 docstring): so101 판정 전 omx 를 열면 물체 낙하 —
    회귀 테스트가 호출 순서를 잠근다."""
    await _move_j(ctx, so101, joints=plan.sols[0])
    a = _approach_of_quat(plan.quat)
    target = plan.target
    lateral = float("inf")
    for _tick in range(_RECV_SERVO_MAX):
        updated = await so_refine(
            ctx, so101, prompt,
            ReceivePlan(
                sols=plan.sols, quat=plan.quat, target=target,
                omx_joints=plan.omx_joints, pre_clear_m=plan.pre_clear_m,
                axis=plan.axis,
            ),
            present, geom, trace,
        )
        lateral = math.dist(updated, target)
        target = updated
        if lateral <= _RECV_SERVO_EPS_M:
            break
        # 보정된 pre 로 재정렬 (standoff 유지 — 다음 측정은 보정된 자세에서)
        pre = (
            target[0] - a[0] * plan.pre_clear_m,
            target[1] - a[1] * plan.pre_clear_m,
            target[2] - a[2] * plan.pre_clear_m,
        )
        await _move_l(
            ctx, so101, position=pre, quaternion=plan.quat,
            speed_scale=_GENTLE_SPEED_SCALE,
        )
    else:
        logger.warning(
            "receive: servo %d tick 에도 lateral 미수렴 (마지막 %.0fmm) — "
            "마지막 측정으로 진입", _RECV_SERVO_MAX, lateral * 1000,
        )
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
    advisory: bool = False,
) -> None:
    """gap OR load 판정 (pick_and_place _gripper_holding 동일 규약) — 미달이면
    GraspFailed. 판정 근거 전부 로깅+trace (실물 임계 튜닝 데이터 — 특히 omx
    Dynamixel load 스케일은 미검증 §5.4, 이 원값이 재특성화의 1차 소스).

    advisory=True — **판정을 경고로 강등** (2026-07-29 사용자 지시): omx 그리퍼
    미믹 기어가 어긋나면(하드웨어) raw↔개구 매핑이 밀려 gap 판정이 헛발.
    EMPTY 여도 raise 하지 않고 경고+trace 만 남기고 진행. 기어 정비/재특성화
    후 되돌릴 것 — 판정 원값은 계속 trace 에 쌓인다."""
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
    # abs — Dynamixel Present_Load 는 부호=방향 (omx 닫힘 스톨 −499, 2026-07-27)
    held = gap > margin or (load is not None and abs(load) >= _HELD_LOAD_MIN_RAW)
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
        if advisory:
            logger.warning(
                "verify_grasp[%s] robot=%s EMPTY 판정이지만 advisory — 진행 "
                "(미믹 기어 어긋남으로 raw 판정 신뢰 불가, 2026-07-29)",
                phase,
                robot_id,
            )
            return
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
