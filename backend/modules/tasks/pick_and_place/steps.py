"""pick_and_place @step 함수들 — closed-loop(servo) 파지 판 (2026-07-16 재설계).

**집기 = closed-loop look-then-move** (docs/closed_loop_grasp_handoff.md 구현,
순수 계산·실측 근거·상태 전이 = servo.py, trace = servo_trace.py):

    찾기(search 스윕, coarse) → 계획(자세 가족 + standoff 사다리 resolve, 모션 0)
    → servo 루프 (rung 마다: 정지 관측 → tick gate → 상대 오차 보정 MoveL → 수렴
    시 하강) → commit (마지막 관측으로 blind 진입) → close → 파지 판정 (재시도 1)
    → 후퇴 → 판정 → home

옛 open-loop 파지 (멀티뷰 융합 → 표면 antipodal → 일괄 실행) 는 **대체됨** —
팔 절대정확도(자세의존 ~1-2cm) ≈ 큐브(2.5cm) 라 구조적으로 실패했다 (2026-07-15
post-mortem, 성공 0). antipodal/plan_grasp 코드는 grasp_verify 진단 스크립트가
소비하므로 geometry/antipodal.py 에 남아 있다 (production 소비자는 이 파일에서
제거). **놓기는 open-loop 유지** — 적치 대상(상자)이 크고 넓어 1-2cm 오차가
치명적이지 않다 (실측 도달 오차 12.8mm < 상자 여유).

handoff §2 실패 표 대비 구현 현황 (정직):
- 구현: 처음부터 못 봄 / 단발 드롭 vs 연속 소실 / mask 오검출(도약 gate) / depth
  붕괴(점군 gate) / 수렴 실패·발진(보정 상한) / 전체 timeout(tick 상한) / servo
  이동 IK 거부(MoveJ 폴백 후 실패) / close 후 EMPTY(재시도 상한) / 이송 중 놓침.
- 미구현 (알고 넘어감): FOV 부분 이탈(잘림) 전용 감지 — 응답에 이미지 크기가
  없어 bbox 경계 판정 불가. 점군 부족/도약 gate 가 간접 커버, 실물 데이터에서
  전용 gate 필요성이 보이면 계약 확장.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from modules.detector.contract import (
    DetectOrientedResponse,
    DetectRequest,
    Detector,
    FuseOrientedRequest,
    FuseOrientedResponse,
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
)
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import (
    DetectionNotFound,
    GraspFailed,
    NoReachableGrasp,
    ServoFailed,
    TaskError,
)
from modules.tasks.core.step import step
from modules.waypoint.contract import (
    ListGroupMembersRequest,
    ListGroupMembersResponse,
    ListGroupsRequest,
    ListGroupsResponse,
    ListWaypointsRequest,
    ListWaypointsResponse,
    Waypoint,
    WaypointRecord,
)

from . import geometry, servo
from .antipodal import _JAW_OPEN_MAX_M
from .geometry import PlaceCandidate, Quat, Vec3
from .servo_trace import ServoTrace

logger = logging.getLogger(__name__)

# gripper 이동 완료 대기 — SET_GRIPPER 는 단발 goal 이라 완료 통지가 없다.
# 2026-07-17 close 30°/s 감속(타격 발사 방지)에 맞춰 1.2→4.0: open↔close 풀
# 스트로크 ~110° ≈ 3.7s. 짧으면 verify 가 닫히는 도중 raw 를 읽어 오판.
_GRIPPER_SETTLE_S = 4.0
_TOP_K = 5

# 검색 자세 그룹 — 사용자가 티칭한 "search" waypoint 그룹 (robot 별). 이 자세들을
# 모두 돌며 관측한다 (coarse 찾기 전용 — 파지 정밀도는 servo 루프 몫).
_SEARCH_GROUP = "search"
_SEARCH_SETTLE_S = 0.3  # MoveJ 후 카메라 흔들림 정착 대기 (검출 품질)

# 경유 자세 waypoint — 긴 이동(관측→접근)이 관절 공간으로 여기를 거친다. 티칭 필수.
_HOME_WAYPOINT = "home"

# 바닥 충돌 게이트 평면을 검출 base_z 보다 살짝 내리는 버퍼 — 게이트는 cm-급
# 육안 충돌용, mm-급 여유는 geometry clearance 상수 책임.
_FLOOR_GATE_MARGIN_M = 0.005

# 같은 물체 판정 반경 — 융합 결과에서 타깃 군집을 찾을 때 (base frame 정렬 전제).
_VIEW_MATCH_RADIUS_M = 0.05
# 이웃 장애물 수집 반경 — 이 안의 다른 검출 군집 점군을 계획 resolve 의 충돌
# 게이트에 장애물로 넣는다 (이보다 먼 물체는 접근과 무관).
_NEIGHBOR_RADIUS_M = 0.15

# servo 파라미터 SSOT (servo.ServoConfig docstring — 실물 첫 런 데이터로 튜닝).
_SERVO_CFG = servo.ServoConfig()

# 집기 계획이 resolve 를 시도하는 검출 후보 수 상한 — 오염 뷰 하나에 태스크가
# 죽지 않게 다음 후보로 넘어가되, 전멸 뷰당 resolve ~40s 라 폭주 방지 상한.
# 건강한 뷰는 첫 budget 에서 조기 성공 (수 초).
_PLAN_TRY_MAX = 4

# 후보 시도 순서용 base_z 문턱 — base_z 는 뷰마다 다른 걸 잰다 (top-view =
# 보이는 band 하단 ≈ 윗면 / 옆면 뷰 = 실제 바닥 근처. 2026-07-16 실물 확인).
# top-view 는 obs XY 가 윗면 centroid = 파지 XY 그 자체라 계획 진입이 깨끗
# → 먼저 시도. 낮은 base_z 뷰는 기하 정보(실 바닥)는 좋지만 위치에 뷰별 FK
# 오차가 섞여 후순위. 기각 아님 — 최종 심판은 resolve.
_BASE_Z_PLAUSIBLE_MIN_M = -0.01
# 위쪽 상한 — flying-pixel 트레일이 검출을 공중으로 들어올린 오염의 방어선
# (1차 방어 = detector _body_z_band 질량 앵커, 2026-07-17 실물: 오염 spot
# base_z=+0.156~0.175 가 score 상위로 resolve ~55s×2 낭비 / 큐브 +0.044 →
# servo 허공 목표). 적치로 상자 위에 놓인 물체(base_z ≈ 상자 top ~0.03)는
# 정상 통과해야 하므로 gross floater 만 거르는 성긴 상한.
_BASE_Z_PLAUSIBLE_MAX_M = 0.08
# 적치 spot 하한이 pick(-0.01)보다 깊은 이유: 적치 상자는 크고 멀티뷰 바닥이
# 실제로 -0.02 대까지 관측된다 (2026-07-17 로그: 실상자 -0.007~-0.024) —
# pick 문턱을 그대로 쓰면 건강한 상자 뷰 전부가 후순위로 밀린다.
_PLACE_BASE_Z_MIN_M = -0.04

# pick 후보 score 하한 — 이 밑은 실행 후보에서 **제외** (후순위가 아니라 컷).
# 도달성 우선 순회가 저신뢰 오검출까지 내려가면 엉뚱한 물체를 집으러 간다
# (2026-07-17 실물: 진짜 큐브(0.76/0.49) 전멸 → score 0.31 오검출(로봇 옆
# 흰 어댑터) 채택 → 사용자 STOP). 실측 분리: 진짜 큐브 min 0.49 / 오검출 max
# 0.44. 전 후보 미달이면 명시 실패 — 오동작보다 정직한 실패가 낫다.
# **pick 전용** — place spot 은 진짜 상자가 score 0.34 로 채택된 실측(06:18)이
# 있어 하한을 걸면 정상 run 이 죽는다 (적치는 오검출 비용도 낮음).
_PICK_SCORE_MIN = 0.45
# pick 후보 폭 상한 — 조가 물리적으로 못 무는 물체는 score 와 무관하게 후보가
# 아니다 (2026-07-17 실물: 손에 든 큐브 전멸 후 score 0.68 짜리 footprint
# 116mm blob 을 "small cube" 로 채택, lateral 47mm 계획으로 "완전 다른 데"
# 주행 — antipodal 쌍 필터(_JAW_OPEN_MAX_M)는 쓰레기 점군 안 우연 쌍으로
# 우회됨). 문턱 = 조 개구 SSOT + 관측 번짐 여유 (실측: 실물 20mm 가 depth
# 번짐으로 33mm 로 측정된 전례 — 진짜 큐브를 컷하지 않게 +15mm).
_PICK_WIDTH_BLEED_M = 0.015
_PICK_MAX_WIDTH_M = _JAW_OPEN_MAX_M + _PICK_WIDTH_BLEED_M

# (명령-실측 잔차 보상은 servo.PlantComp 로 이동 — 근거/사용 규약은 그 docstring)


# ─── scenario 골격: 계획(모션 0 판정) → servo 집기 → 놓기 ────────────
#
# 순서 규약 (2026-07-13): 물리 파지 **전에** 집기·놓기 도달성을 모두 검증한다 —
# 놓을 곳이 도달 불가면 아무것도 집기 전에 실패 (쥔 채 멈춤 corrupt 방지).
# 놓기 계획의 held 기하는 coarse 관측 (단일 뷰 height 과소 가능 — release 가
# 수 mm 낮아질 수 있으나 상자 삽입은 관대. 정밀화는 실물 데이터 후 판단).


@dataclass(frozen=True, slots=True)
class ServoPlan:
    """plan_pick 산출 — servo 루프의 시작 조건 (전부 coarse 관측 기준 초기값).

    rung0_joints: resolve 가 반환한 첫 standoff 의 IK 해 (실행부 재계산 없음).
    grasp_point0/grasp_tcp0: coarse 기준 초기 파지 지점/TCP — 루프가 매 tick
    관측으로 갱신하므로 이 값은 진입용 + 마커 표시용.
    """

    coarse: OrientedDetection
    family: servo.GraspFamily
    rung0_joints: list[float]
    grasp_point0: Vec3
    grasp_tcp0: Vec3
    lateral0: float
    # 실 바닥 추정 (클러스터 min base_z − margin) — servo 루프의 파지 z 하한
    # guard (납작한 물체에서 grip_below_top 이 테이블을 뚫는 것 방지).
    floor_z: float | None = None


def servo_ladder_groups(
    coarse: OrientedDetection,
    cfg: servo.ServoConfig,
    floor_z: float | None = None,
    *,
    yaw_free: bool = False,
) -> tuple[list[list[TcpPose]], list[tuple[servo.GraspFamily, Vec3, Vec3, float]]]:
    """coarse 관측 → resolve 후보 그룹 ([standoff 사다리…, 파지] × 가족) + 메타.

    plan_pick 과 sim 게이트 테스트(test_motion — 실 URDF IK 로 이 그룹이 진짜
    풀리는지)가 공유하는 그룹 구성 SSOT. yaw_free=True 면 근사 정사각 물체의
    확장 yaw 가족이 기존 블록 **뒤에** 붙는다 (2단 resolve — 호출부 슬라이스)."""
    families = servo.grasp_families(coarse, yaw_free=yaw_free)
    groups: list[list[TcpPose]] = []
    metas: list[tuple[servo.GraspFamily, Vec3, Vec3, float]] = []
    g_point0 = servo.grasp_point(coarse, coarse, cfg, floor_z)
    for fam in families:
        width = servo.width_along(
            coarse.points, fam.jaw_axis, fallback_m=coarse.footprint[1]
        )
        lateral = servo.lateral_offset(width)
        g_tcp0 = servo.grasp_tcp(g_point0, fam, lateral, cfg.engage_m)
        poses = [
            TcpPose(
                position=servo.standoff(g_tcp0, fam, s), quaternion=fam.quat
            )
            for s in cfg.standoffs
        ]
        poses.append(TcpPose(position=g_tcp0, quaternion=fam.quat))
        groups.append(poses)
        metas.append((fam, g_point0, g_tcp0, lateral))
    return groups, metas


@step(title="집기 계획")
async def plan_pick(
    ctx: TaskContext, robot_id: str, prompt: str, home: WaypointRecord
) -> ServoPlan:
    """찾기(coarse) + servo 접근 계획 (모션 0) → ServoPlan.

    **도달성 우선 선택 (2026-07-16)**: score 1등에 커밋하지 않는다 — 스윕 뷰 간
    검출 위치는 FK 계통 오차로 1.5~3.3cm 어긋나며 (detector FUSE_ORIENTED
    docstring), 오염 뷰(예: base_z 가 테이블 아래로 3cm 눌린 관측)는 resolve 가
    정당하게 전멸시킨다. score 1등이 그 오염 뷰면 태스크 전체가 죽는 실사고
    (2026-07-16: 같은 큐브의 건강 뷰 4가족 통과, score 1등 오염 뷰 전멸).
    → score 내림차순으로 후보마다 resolve, 첫 성공 채택 (plan_place 2026-07-14
    와 동일 원칙). 전 후보 전멸 = 후보별 사유 포함 명시 실패 (맹목 파지 금지).

    자세 가족(조 축 2 × flip 2 × tilt 13)마다 [standoff 사다리…, 파지] 를 한
    그룹으로 resolve — 게이트: 끝점 IK + 바닥 + 그리퍼(벌림)↔**이웃** 점군
    충돌 + home→rung0 관절 경로 + 사다리 구간 직선(linear). 파지 대상 자신의
    점군은 장애물이 아니다 (engage 겹침은 의도 — resolve 호출부 주석).
    """
    cands = await detect(ctx, robot_id, prompt)
    if not cands:
        raise DetectionNotFound(prompt, candidates=0, reason="검출 0건")
    cfg = _SERVO_CFG
    # 신뢰 컷 2종 — 저신뢰 score / 조 개구 초과 폭. 어느 쪽이든 순회 폴백
    # 대상조차 아님 (상수 주석 — 엉뚱한 물체 파지 실사고 2건). 컷은 침묵하지
    # 않는다.
    oversize = [c for c in cands if c.footprint[1] > _PICK_MAX_WIDTH_M]
    if oversize:
        logger.info(
            "plan_pick(%s): 개구 초과 후보 %d개 제외 (짧은 변 %s > %.0fmm — "
            "조가 못 무는 크기)",
            prompt, len(oversize),
            [f"{c.footprint[1] * 1000:.0f}mm" for c in oversize],
            _PICK_MAX_WIDTH_M * 1000,
        )
    sized = [c for c in cands if c.footprint[1] <= _PICK_MAX_WIDTH_M]
    trusted = [c for c in sized if c.score >= _PICK_SCORE_MIN]
    if len(trusted) < len(sized):
        logger.info(
            "plan_pick(%s): 저신뢰 후보 %d개 제외 (score < %.2f — 오검출 방지)",
            prompt, len(sized) - len(trusted), _PICK_SCORE_MIN,
        )
    if not trusted:
        raise DetectionNotFound(
            prompt,
            candidates=len(cands),
            reason=(
                f"검출 {len(cands)}건 전부 신뢰 컷 미달 (score < "
                f"{_PICK_SCORE_MIN} 저신뢰 {len(sized) - len(trusted)}건 / "
                f"조 개구 {_PICK_MAX_WIDTH_M * 1000:.0f}mm 초과 "
                f"{len(oversize)}건). 물체 위치/조명 확인 후 다시 실행하세요"
            ),
        )
    # 물리 타당(base_z 가 설치면 근방 대역 안) 후보 먼저, 그 안에서 score
    # 내림차순 — 불가능 기하에 resolve ~40s 를 먼저 태우지 않는다 (2026-07-16:
    # score 1등이 base_z=-0.021 오염 뷰라 건강 뷰 도달인데 1분 소모+전멸 보고 /
    # 2026-07-17: 공중 부양 오염 뷰 — 상한 추가).
    ordered = sorted(
        trusted,
        key=lambda c: (
            c.base_z < _BASE_Z_PLAUSIBLE_MIN_M
            or c.base_z > _BASE_Z_PLAUSIBLE_MAX_M,
            -c.score,
        ),
    )[:_PLAN_TRY_MAX]
    failures: list[str] = []
    for rank, coarse in enumerate(ordered):
        neighbors = _neighbor_points(cands, coarse)
        # 바닥 평면 = 같은 물체를 본 뷰들(클러스터) base_z 의 최솟값 — 단일
        # top-view 의 base_z 는 바닥이 아니라 ≈윗면이라, 그걸 floor 로 쓰면
        # 윗면 근처 가짜 바닥이 생겨 깊은 파지가 계획에서 전멸한다. 옆면을 본
        # 뷰의 base_z 가 실 바닥에 가장 가깝다 (min 이 그 뷰를 고른다).
        cluster_base = [
            c.base_z for c in cands
            if _xy_dist(c.position, coarse.position) <= _VIEW_MATCH_RADIUS_M
        ]
        floor_z = min(cluster_base) - _FLOOR_GATE_MARGIN_M
        # 2단 resolve (2026-07-17 저녁 — 채택 속도와 커버리지의 분리): 1단 =
        # 기존 축 52가족 (빠른 채택 경로 — 잘 잡히는 위치는 여기서 수 초),
        # 전멸 시에만 2단 = 확장 yaw 가족 (근사 정사각 물체의 yaw 자유 —
        # OBB yaw 노이즈 복권 탈출. servo.grasp_families yaw_free docstring).
        full_groups, full_metas = servo_ladder_groups(
            coarse, cfg, floor_z, yaw_free=True
        )
        base_groups, base_metas = servo_ladder_groups(coarse, cfg, floor_z)
        base_n = len(base_groups)
        stages: list[tuple[str, list, list]] = [
            ("기존", base_groups, base_metas)
        ]
        if len(full_groups) > base_n:
            stages.append(("확장", full_groups[base_n:], full_metas[base_n:]))

        stage_fails: list[str] = []
        adopted = None
        for stage, groups, metas in stages:
            t0 = time.monotonic()
            # 장애물 = **이웃 점군만** — 파지 대상 자신의 점군은 넣지 않는다
            # (2026-07-17 오후 실사고): engage(조를 물체 쪽으로 밀어넣어 물기)
            # 설계상 grasp 자세의 조↔대상 겹침은 **의도된 것**인데, 자기
            # 점군을 장애물로 검사하면 그 겹침이 기각된다 (실측 침투 -3.6~
            # -9.6mm = engage 겹침 — 기각/통과를 가른 건 "카메라가 어느 면을
            # 봤나"). 대상 보호 = antipodal 구성 + servo + 파지 판정
            # (MoveIt ACM 의 조작 대상 충돌 허용 등가).
            res = await ctx.call(
                Motion.Service.RESOLVE_REACHABLE,
                ResolveReachableRequest(
                    groups=groups,
                    floor_z=floor_z,
                    linear=True,
                    obstacle_points=list(neighbors),
                    gripper_open=True,
                    path_from=list(home.joint_values),
                ),
                ResolveReachableResponse,
                robot_id=robot_id,
            )
            resolve_s = time.monotonic() - t0
            if res.index >= 0:
                adopted = (stage, groups, metas, res, resolve_s)
                break
            stage_fails.append(
                f"{stage} {len(groups)}가족 전멸 ({resolve_s:.1f}s) — "
                f"{res.message}"
            )
            if stage == "기존" and len(stages) > 1:
                logger.info(
                    "plan_pick(%s): 후보%d 기존 축 전멸 — 확장 yaw %d가족 "
                    "재시도 (근사 정사각 — OBB yaw 노이즈 복권 탈출)",
                    prompt, rank, len(full_groups) - base_n,
                )
        if adopted is None:
            msg = (
                f"후보{rank}(score {coarse.score:.2f} "
                f"pos={_fmt(coarse.position)} base_z={coarse.base_z:+.3f}): "
                + _join_msgs(stage_fails)
            )
            failures.append(msg)
            logger.info("plan_pick(%s): %s — 다음 후보 시도", prompt, msg)
            continue
        stage, groups, metas, res, resolve_s = adopted
        fam, g_point0, g_tcp0, lateral = metas[res.index]
        logger.info(
            "plan_pick(%s): 후보%d/%d 채택 (score %.2f) — %s 가족 %d/%d %s, "
            "grasp0=%s lateral=%.1fmm (resolve %.1fs)",
            prompt, rank, len(ordered), coarse.score, stage, res.index,
            len(groups), fam.label, _fmt(g_tcp0), lateral * 1000.0, resolve_s,
        )
        return ServoPlan(
            coarse=coarse,
            family=fam,
            rung0_joints=res.solutions[0],
            grasp_point0=g_point0,
            grasp_tcp0=g_tcp0,
            lateral0=lateral,
            floor_z=floor_z,
        )
    raise NoReachableGrasp(
        f"servo 접근 — 검출 후보 {len(ordered)}개 전부 전멸:\n  "
        + "\n  ".join(failures)
    )


@step(title="놓기 계획")
async def plan_place(
    ctx: TaskContext,
    robot_id: str,
    prompt: str,
    *,
    held: OrientedDetection,
    lateral: float,
    home: WaypointRecord,
) -> tuple[PlaceCandidate, list[float]]:
    """검출 + 적치 후보 게이트 판정 (모션 0) → (적치 후보, pre 관절해).

    **도달성 우선 선택 (2026-07-14)**: 점수 1등에 무조건 커밋하지 않는다 — spot
    을 점수순으로 돌며 팔이 실제로 닿는 첫 spot 채택. spot 마다 yaw 두 가족 순차
    (① 상자 방위 정렬 ② 전멸 시 자유 — 삐딱하게라도 놓는 게 task 실패보다 낫다).
    held/lateral 은 coarse 관측·계획 lateral (servo 확정값과 수 mm 차 가능 —
    상자 적치는 관대)."""
    spots = await detect(ctx, robot_id, prompt)
    if not spots:
        raise TaskError(
            f"'{prompt}' 적치 대상 검출 0건 — 물체 배치/조명 확인 후 다시 "
            "실행하세요"
        )
    # 물리 타당(base_z 대역 안) spot 먼저 — plan_pick 과 같은 원칙 (기각 아님,
    # 최종 심판은 resolve). 2026-07-17 실물: 공중 부양 오염 spot(base_z=+0.156
    # ~0.175)이 score 상위를 차지해 spot 당 정렬+자유 resolve ~55s 를 먼저
    # 태우고, 최악 런은 plan_place 에만 3.5분 소모 후 실패.
    ranked = sorted(
        spots,
        key=lambda s: (
            s.base_z < _PLACE_BASE_Z_MIN_M
            or s.base_z > _BASE_Z_PLAUSIBLE_MAX_M,
            -s.score,
        ),
    )
    for spot in ranked:
        for family, pplan in (
            ("정렬", geometry.plan_place(spot, held=held, lateral=lateral)),
            ("자유", geometry.plan_place_free(spot, held=held, lateral=lateral)),
        ):
            got = await resolve_place(
                ctx, robot_id, pplan,
                floor_z=spot.base_z - _FLOOR_GATE_MARGIN_M,
                home=home,
            )
            if got is not None:
                idx, sols = got
                logger.info(
                    "plan_place(%s): spot 채택 score=%.2f base_z=%.3fm "
                    "pos=(%.3f,%.3f) — %s yaw %s (후보 %d건 중)",
                    prompt, spot.score, spot.base_z, spot.position[0],
                    spot.position[1], pplan[idx].label, family, len(ranked),
                )
                return pplan[idx], sols[0]
            logger.info(
                "plan_place(%s): spot score=%.2f pos=(%.3f,%.3f) %s yaw %d후보 "
                "전멸 — %s", prompt, spot.score, spot.position[0],
                spot.position[1], family, len(pplan),
                "자유 yaw 폴백" if family == "정렬" else "다음 spot",
            )
    raise NoReachableGrasp(
        f"놓을 자리 도달 불가 — '{prompt}' 후보 {len(ranked)}건 모두 팔이 닿지 "
        "않습니다 (정렬+자유 yaw 전부 시도 — workspace 밖이거나 주변이 막힘). "
        "상자를 로봇 쪽으로 옮기거나 주변 장애물을 치운 뒤 다시 실행하세요"
    )


# ─── servo 집기 (closed-loop 본체) ──────────────────────────────────


@step(title="servo 집기")
async def servo_pick(
    ctx: TaskContext,
    robot_id: str,
    plan: ServoPlan,
    prompt: str,
    home: WaypointRecord,
    on_grasp: Callable[[Vec3], None] | None = None,
) -> None:
    """closed-loop 파지 실행 — home 경유 → rung0 진입 → tick 루프 → commit →
    close → 판정(재시도) → 후퇴 → 판정 → home.

    루프 계약 (servo.py docstring = SSOT):
    - 관측은 **정지 상태** 에서만 (이동 완료 → settle → DETECT_ORIENTED).
    - 명령은 관측한 그 tick 의 TCP 기준 상대 목표 → common-mode FK 상쇄.
    - 모든 실패에 정의된 동작 (decide_tick) — 크래시/무한대기 없음, 사유는
      ServoFailed 메시지 + trace 에 남는다.
    - trace: 매 tick JSONL + 종료 summary (debug/servo_pick/<ts>/ —
      실패 재구성이 하드웨어 없이 가능해야 한다는 요구의 구현).
    - on_grasp: 채택 관측이 파지점을 갱신할 때마다 호출 (호출자=module 이
      마커 스트림 재발행 — 계획 시점 마커가 실행 내내 고정 표시되던 UI 구멍,
      2026-07-17 사용자 리포트).
    """
    cfg = _SERVO_CFG
    trace = ServoTrace(prompt, robot_id)
    state = servo.ServoState()  # tick/rung 카운터 — decide_tick 의 입력
    run = servo.TrackState(  # 관측 추적 + 파지 기하 (전이 근거 = servo.py 주석)
        fam=plan.family,
        expected_xy=plan.coarse.position,
        g_tcp=plan.grasp_tcp0,
        g_point=plan.grasp_point0,
        lateral=plan.lateral0,
        fallback_width_m=plan.coarse.footprint[1],
        floor_z=plan.floor_z,
    )
    comp = servo.PlantComp()  # 명령-실측 잔차 보상 (근거 = servo.py docstring)
    tcp: TcpState | None = None
    summary: dict = {"result": "unknown", "family": run.fam.label}

    try:
        await go_home(ctx, robot_id, home)
        await _move_j_joints(ctx, robot_id, plan.rung0_joints)
        await open_gripper(ctx, robot_id)

        while True:  # attempt 루프 (close 후 EMPTY 재시도)
            committed = False
            while not committed:  # tick 루프
                await asyncio.sleep(cfg.settle_s)
                det = await ctx.call(
                    Detector.Service.DETECT_ORIENTED,
                    DetectRequest(robot_id=robot_id, prompt=prompt, top_k=_TOP_K),
                    DetectOrientedResponse,
                )
                tcp = await ctx.call(
                    Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
                    robot_id=robot_id,
                )
                comp.observe(tcp.position)
                gate = servo.gate_observation(
                    det.candidates, run.expected_xy, run.last, cfg,
                    match_radius_m=(
                        cfg.reacquire_radius_m if run.reacquiring else None
                    ),
                )
                if gate.obs is None:
                    # 재앵커 — 연속 도약-기각 2건이 상호 일관하면 그쪽이
                    # 다수결 진실 (servo.TrackState.consider_reanchor 주석).
                    re = run.consider_reanchor(gate.rejected, cfg)
                    if re is not None:
                        logger.info(
                            "servo: 재앵커 — 연속 기각 2건 상호 일관 (%s)",
                            gate.reason,
                        )
                        await _trace_emit(trace, {
                            "phase": "reanchor",
                            "tick": state.ticks,
                            "reason": gate.reason,
                            "obs": _obs_record(re),
                        })
                        gate = servo.GateResult(re, "")
                lateral_err: float | None = None
                axial_err = 0.0
                fused = None
                if gate.obs is not None:
                    if run.reacquiring:
                        # 물체가 움직인 뒤 첫 재획득 — 회전했을 수 있으니 조
                        # 각도(가족 yaw)도 새 관측으로 재유도 (servo.refit_family
                        # docstring — 옛 각도 스큐 close 가 재튕김 만든 실사고)
                        new_fam = servo.refit_family(run.fam, gate.obs)
                        if new_fam is not None:
                            logger.info(
                                "servo: 재획득 yaw 재유도 — obs yaw %.1f° 로 "
                                "가족 회전 (%s)",
                                math.degrees(gate.obs.grasp_yaw),
                                run.fam.label,
                            )
                            await _trace_emit(trace, {
                                "phase": "refit",
                                "tick": state.ticks,
                                "action": "family_refit",
                                "obs_yaw_deg": round(
                                    math.degrees(gate.obs.grasp_yaw), 1
                                ),
                            })
                            run.fam = new_fam
                    run.reacquiring = False  # 재획득 완료 — 반경 원복
                    run.note_accept(gate.obs)
                    fused = await _fuse_recent(
                        ctx, run.accepted[-cfg.fuse_last_k:], gate.obs
                    )
                    run.update_grasp(gate.obs, fused, cfg)
                    _notify_grasp(on_grasp, run.g_point)  # 마커 재발행
                    target_so = servo.standoff(
                        run.g_tcp, run.fam, cfg.standoffs[state.rung]
                    )
                    delta = (
                        target_so[0] - tcp.position[0],
                        target_so[1] - tcp.position[1],
                        target_so[2] - tcp.position[2],
                    )
                    lateral_err, axial_err = servo.split_error(delta, run.fam)

                decision = servo.decide_tick(state, gate, lateral_err, cfg)
                logger.info(
                    "servo tick %d rung=%d(%.0fmm) 관측=%s lat=%s ax=%.1fmm → "
                    "%s (%s)",
                    state.ticks, state.rung,
                    cfg.standoffs[state.rung] * 1000.0,
                    "채택" if gate.obs is not None else f"기각[{gate.reason}]",
                    f"{lateral_err * 1000:.1f}mm" if lateral_err is not None
                    else "-",
                    axial_err * 1000.0, decision.action, decision.reason,
                )
                await _trace_emit(trace, {
                    "phase": "tick",
                    "tick": state.ticks,
                    "rung": state.rung,
                    "standoff_m": cfg.standoffs[state.rung],
                    "gate_reason": gate.reason,
                    "observation": _obs_record(gate.obs),
                    "fused": _obs_record(fused),
                    "candidates_n": len(det.candidates),
                    "lateral_mm": (
                        round(lateral_err * 1000, 2)
                        if lateral_err is not None else None
                    ),
                    "axial_mm": round(axial_err * 1000, 2),
                    "grasp_tcp": [round(v, 4) for v in run.g_tcp],
                    "lateral_offset_mm": round(run.lateral * 1000, 2),
                    "comp_mm": comp.mm,
                    "tcp_position": [round(v, 4) for v in tcp.position],
                    "tcp_joints": [round(v, 4) for v in tcp.joints],
                    "action": decision.action,
                    "reason": decision.reason,
                })

                if decision.action == "hold":
                    continue
                if decision.action == "abort":
                    raise ServoFailed(decision.reason, ticks=state.ticks)
                if decision.action in ("correct", "descend"):
                    cmd = comp.apply(servo.standoff(
                        run.g_tcp, run.fam, cfg.standoffs[state.rung]
                    ))
                    try:
                        await _servo_move(
                            ctx, robot_id, cmd, run.fam.quat, trace
                        )
                    except ServoFailed:
                        # 오염 관측이 만든 허공 목표는 IK 거부가 옳다 — 관측
                        # 불신 + 재관측으로 계속 (2026-07-17 실물: 거부 1회에
                        # 태스크 전체 중단 사고). 연속 2회 = 재관측도 같은
                        # 목표를 재생산 = **관측이 진실인데 가족이 그 자리서
                        # 불가** (07-17 저녁 실물: 헛집음이 큐브를 3cm 밀어
                        # r≈0.32 경계로 — 재유도 가족 standoff 가 IK 밖, 정확한
                        # 관측 2연속을 '불신'만 하다 사망) → 그 위치 기준 가족
                        # 재-resolve 로 계속, 그것도 전멸이면 진짜 경계 → abort.
                        run.move_fails += 1
                        if run.move_fails >= 2:
                            if await _replan_family(
                                ctx, robot_id, run, state, cfg, tcp, trace
                            ):
                                run.move_fails = 0
                                continue
                            raise
                        run.distrust_last()
                        logger.warning(
                            "servo 이동 거부 (%d/2) — 관측 불신, 재관측으로 "
                            "계속: cmd=%s", run.move_fails, _fmt(cmd),
                        )
                        await _trace_emit(trace, {
                            "phase": "move",
                            "action": "rejected_hold",
                            "tick": state.ticks,
                            "move_fails": run.move_fails,
                            "cmd": [round(v, 4) for v in cmd],
                        })
                        continue
                    run.move_fails = 0
                    comp.commanded(cmd)
                    continue
                committed = True  # commit

            # ── commit: 마지막 관측으로 blind 최종 접근 (handoff §4) ──
            blind_m = (
                math.dist(run.g_tcp, tuple(tcp.position))
                if tcp is not None else cfg.standoffs[state.rung]
            )
            logger.info(
                "servo commit: rung=%d blind=%.1fmm grasp_tcp=%s (%s)",
                state.rung, blind_m * 1000.0, _fmt(run.g_tcp), run.fam.label,
            )
            grasp_cmd = comp.apply(run.g_tcp)
            await _trace_emit(trace, {
                "phase": "commit",
                "tick": state.ticks,
                "rung": state.rung,
                "blind_mm": round(blind_m * 1000, 1),
                "grasp_tcp": [round(v, 4) for v in run.g_tcp],
                "comp_mm": comp.mm,
                "cmd": [round(v, 4) for v in grasp_cmd],
                "action": "commit",
                "reason": f"blind {blind_m * 1000:.1f}mm 최종 접근",
            })
            await _move_l(
                ctx, robot_id, grasp_cmd, run.fam.quat,
                speed_scale=cfg.gentle_speed_scale,
            )
            comp.commanded(grasp_cmd)
            await _touch_up(ctx, robot_id, run, grasp_cmd, comp, cfg,
                            trace, state.ticks)
            await _log_reached_tcp(
                ctx, robot_id, expected=run.g_tcp, phase="servo grasp 도달"
            )
            await close_gripper(ctx, robot_id)
            try:
                grip = await verify_grasp(
                    ctx, robot_id, phase="close 직후",
                    grasp_label=run.fam.label,
                )
                await _trace_emit(trace, {
                    "phase": "close", "tick": state.ticks, "action": "held",
                    **grip,
                })
                # ── 후퇴 + 슬립 재판정 — 이송 자격 검증 (attempt 루프 안:
                # 놓침/슬립 모두 재시도 대상). 후퇴 감속 — 잡은 직후 가속이
                # 얕은 파지를 흔들어 빼는 실사고 (2026-07-17).
                try:
                    await _move_l(
                        ctx, robot_id,
                        servo.standoff(
                            run.g_tcp, run.fam, cfg.withdraw_standoff_m
                        ),
                        run.fam.quat,
                        speed_scale=cfg.gentle_speed_scale,
                    )
                except asyncio.CancelledError:
                    raise
                except Exception as e_w:
                    # **쥔 이후의 이동 실패는 task 를 죽일 수 없다** (2026-07-17
                    # 저녁 실물: HELD·부하 320 직후 withdraw 사전검증 거부 →
                    # 쥔 채 사망. withdraw 목표는 servo 가 보정한 새 위치
                    # 기준이라 계획이 검증한 적 없는 자세 — 경계에서 자세 IK
                    # 가 안 풀릴 수 있다). 폴백 = 계획이 증명한 rung0 관절해
                    # (IK 재풀이 0 — retreat/재플랜과 동일 원칙: 알려진 해가
                    # 있으면 그 해로).
                    logger.warning(
                        "withdraw MoveL 실패 (%s) — 계획 rung0 관절해 MoveJ "
                        "폴백 (쥔 채 계속)", e_w,
                    )
                    await _trace_emit(trace, {
                        "phase": "withdraw",
                        "action": "fallback_rung0",
                        "tick": state.ticks,
                        "reason": str(e_w),
                    })
                    await _move_j_joints(ctx, robot_id, plan.rung0_joints)
                grip2 = await verify_grasp(
                    ctx, robot_id, phase="withdraw 후",
                    grasp_label=run.fam.label,
                )
                # 슬립 = close 시점 gap 대비 유지율 미달 (상대 비교 — 물체
                # 크기 무관). **실패 아님 — 경고 + 계속** (2026-07-17 실물:
                # 공중에 뜨는 순간 자중이 조 곡면을 타고 끝 홈까지 밀리는
                # 결정적 현상 — gap 216→35 두 번 재현, 끝 홈에서 load 288 로
                # 안정적으로 매달림. 내려놓고 재시도해봤자 똑같이 미끄러지고,
                # 조 끝 파지도 이송·상자 적치는 된다. 근본 = 조 마찰 패드
                # [하드웨어]). verify(load OR gap)가 "물고 있음"은 이미 보장.
                slip = (
                    grip2["gap_raw"]
                    < cfg.slip_retention * grip["gap_raw"]
                )
                await _trace_emit(trace, {
                    "phase": "withdraw", "tick": state.ticks,
                    "action": "slip" if slip else "held",
                    "close_gap_raw": grip["gap_raw"], **grip2,
                })
                if slip:
                    logger.warning(
                        "servo: withdraw 슬립 (gap %d→%d, load %s) — 조 끝 "
                        "파지로 이송 계속. 반복되면 조 접촉면 마찰 패드 권장",
                        grip["gap_raw"], grip2["gap_raw"], grip2["load_raw"],
                    )
            except GraspFailed as e:
                run.close_attempts += 1
                await _trace_emit(trace, {
                    "phase": "grasp_retry",
                    "tick": state.ticks,
                    "action": "retry",
                    "reason": str(e),
                    "attempt": run.close_attempts,
                })
                if run.close_attempts >= cfg.close_attempts:
                    raise
                logger.warning(
                    "servo: 파지 실패(%s) — 재시도 %d/%d (후퇴, 재관측)",
                    e, run.close_attempts, cfg.close_attempts,
                )
                await open_gripper(ctx, robot_id)
                await _retreat_for_retry(ctx, robot_id, run, comp, cfg)
                state = servo.ServoState(rung=1)
                continue
            break  # 파지 + 이송 자격 통과

        await go_home(ctx, robot_id, home)
        summary.update({
            "result": "success",
            "close_attempts": run.close_attempts + 1,
            "final_grasp_tcp": [round(v, 4) for v in run.g_tcp],
            "error_history_mm": [
                round(e, 1) for e in state.error_history_mm
            ],
        })
    except BaseException as e:
        summary.update({
            "result": "cancelled" if isinstance(e, asyncio.CancelledError)
            else "failed",
            "error": f"{type(e).__name__}: {e}",
            "close_attempts": run.close_attempts,
            "error_history_mm": [
                round(er, 1) for er in state.error_history_mm
            ],
        })
        raise
    finally:
        try:
            await asyncio.to_thread(trace.finish, summary)
            logger.info("servo trace: %s (%s)", trace.dir, summary["result"])
        except Exception:
            logger.exception("servo trace summary 기록 실패")


async def _replan_family(
    ctx: TaskContext,
    robot_id: str,
    run: servo.TrackState,
    state: servo.ServoState,
    cfg: servo.ServoConfig,
    tcp: TcpState | None,
    trace: ServoTrace,
) -> bool:
    """이동 거부 연속 시 최후 수단 — 현 관측(run.last) 위치 기준 가족 전체
    재-resolve, 통과 가족으로 교체 + rung0 재진입 (plan_pick 채택과 동일 경로).

    전제: 거부가 2연속이면 재관측이 같은 목표를 재생산한 것 = 관측은 진실이고
    **기존 가족이 그 자리에서 IK 불가** (2026-07-17 저녁 실물: 헛집음이 큐브를
    3cm 밀어 경계로 — yaw 재유도만으로는 같은 tilt 가족의 standoff 가 계속
    도달 밖). 전멸/관측 없음 = False — 호출부가 기존 명시 실패 경로 유지."""
    if run.last is None:
        return False
    t0 = time.monotonic()
    # plan_pick 과 동일한 2단 — 1단 기존 축(빠름), 전멸 시 확장 yaw.
    full_groups, full_metas = servo_ladder_groups(
        run.last, cfg, run.floor_z, yaw_free=True
    )
    base_groups, base_metas = servo_ladder_groups(run.last, cfg, run.floor_z)
    base_n = len(base_groups)
    stages: list[tuple[list, list]] = [(base_groups, base_metas)]
    if len(full_groups) > base_n:
        stages.append((full_groups[base_n:], full_metas[base_n:]))
    adopted: tuple[ResolveReachableResponse, list, int] | None = None
    last_msg = ""
    for groups, metas in stages:
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=groups,
                floor_z=run.floor_z,
                linear=True,
                path_from=list(tcp.joints) if tcp is not None else None,
            ),
            ResolveReachableResponse,
            robot_id=robot_id,
        )
        if res.index >= 0:
            adopted = (res, metas, len(groups))
            break
        last_msg = res.message
    if adopted is None:
        logger.warning(
            "servo 재플랜: 가족 %d개(확장 포함) 전멸 (%.1fs) — 물체가 도달 "
            "범위 밖으로 밀려난 것으로 판단: %s",
            len(full_groups), time.monotonic() - t0, last_msg,
        )
        return False
    res, metas, n_groups = adopted
    fam, g_point0, g_tcp0, lateral = metas[res.index]
    logger.info(
        "servo 재플랜: 가족 %d/%d 채택 (%s) — 밀린 물체 위치 기준 rung0 재진입 "
        "(%.1fs)", res.index, n_groups, fam.label, time.monotonic() - t0,
    )
    await _trace_emit(trace, {
        "phase": "replan",
        "tick": state.ticks,
        "action": "refamily",
        "family": fam.label,
        "grasp_tcp": [round(v, 4) for v in g_tcp0],
    })
    run.fam = fam
    run.g_point = g_point0
    run.g_tcp = g_tcp0
    run.lateral = lateral
    run.widths.clear()  # 조 축이 바뀌었을 수 있음 — 폭 이력 무효
    state.rung = 0
    state.corrections = 0
    state.misses = 0
    await _move_j_joints(ctx, robot_id, res.solutions[0])
    return True


def _join_msgs(parts: list[str], sep: str = " / ") -> str:
    """실패 사유 조립 — 명명 헬퍼인 이유: 프리뷰 정적 인덱서가 문자열 리터럴
    `.join` 호출을 `<동적>` 노이즈 행으로 잡는다 (step 트리 오염 방지)."""
    return sep.join(parts)


def _notify_grasp(cb: Callable[[Vec3], None] | None, p: Vec3) -> None:
    """파지점 갱신 통지 — 모듈 레벨 간접호출인 이유: 시나리오 프리뷰의 정적
    인덱서가 파라미터 직접 호출(`on_grasp(...)`)을 못 풀어 step 트리에 `<동적>`
    노이즈 행을 만든다 (2026-07-17 preview 계약 테스트로 포착). 마커 통지는
    step 이 아니므로 트리 미표시가 올바른 의미론."""
    if cb is not None:
        cb(p)


async def _trace_emit(trace: ServoTrace, record: dict) -> None:
    """trace tick 기록 — blocking 파일 I/O 는 to_thread, 실패는 로깅만 (관측이
    실행을 죽이면 안 됨)."""
    try:
        await asyncio.to_thread(trace.emit, record)
    except Exception:
        logger.exception("servo trace 기록 실패 (실행 영향 없음)")


def _obs_record(obs: OrientedDetection | None) -> dict | None:
    """trace 용 관측 요약 — 원시 depth/mask 는 detector 덤프에 (timestamp 교차참조)."""
    if obs is None:
        return None
    return {
        "position": [round(v, 4) for v in obs.position],
        "base_z": round(obs.base_z, 4),
        "height_mm": round(obs.height * 1000, 1),
        "score": round(obs.score, 3),
        "grasp_yaw_deg": round(math.degrees(obs.grasp_yaw), 1),
        "footprint_mm": [round(v * 1000, 1) for v in obs.footprint],
        "points_n": len(obs.points or []),
    }


async def _fuse_recent(
    ctx: TaskContext, recent: list[OrientedDetection], latest: OrientedDetection
) -> OrientedDetection:
    """최근 채택 관측 융합 → 타깃 군집 (z/height/폭 안정화). 융합 불가/군집 없음
    이면 latest 그대로 (침묵 아님 — 로그)."""
    if len(recent) < 2:
        return latest
    res = await ctx.call(
        Detector.Service.FUSE_ORIENTED,
        FuseOrientedRequest(candidates=list(recent)),
        FuseOrientedResponse,
    )
    near = _nearest_within(res.candidates, latest.position, _VIEW_MATCH_RADIUS_M)
    if near is None:
        logger.info("servo: 융합 군집 없음 (%d 관측) — 최신 관측 단독 사용",
                    len(recent))
        return latest
    return near


async def _touch_up(
    ctx: TaskContext,
    robot_id: str,
    run: servo.TrackState,
    grasp_cmd: Vec3,
    comp: servo.PlantComp,
    cfg: servo.ServoConfig,
    trace: ServoTrace,
    tick: int,
) -> None:
    """blind 도달 touch-up — 카메라가 가려진 마지막 ~5cm 는 comp 측정 시점
    (standoff)과 자세·부하가 달라 잔차가 남는다 (2026-07-16 실물: lateral
    1.1mm 수렴인데 EMPTY — 잔여 미달이 조 끝 nip/밀어냄). 관측 불가 구간이라
    FK(엔코더) 잔차로 재보정, 상한 2회."""
    for _ in range(2):
        snap = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
            robot_id=robot_id,
        )
        resid = np.asarray(run.g_tcp) - np.asarray(snap.position)
        resid_norm = float(np.linalg.norm(resid))
        if resid_norm <= 0.003:
            break
        grasp_cmd = (
            float(grasp_cmd[0] + resid[0]),
            float(grasp_cmd[1] + resid[1]),
            float(grasp_cmd[2] + resid[2]),
        )
        logger.info(
            "servo touch-up: FK 잔차 %.1fmm → 재보정 %s",
            resid_norm * 1000.0, _fmt(grasp_cmd),
        )
        await _trace_emit(trace, {
            "phase": "touchup",
            "tick": tick,
            "resid_mm": [round(float(v) * 1000, 1) for v in resid],
            "cmd": [round(v, 4) for v in grasp_cmd],
        })
        await _move_l(
            ctx, robot_id, grasp_cmd, run.fam.quat,
            speed_scale=cfg.gentle_speed_scale,
        )
        comp.commanded(grasp_cmd)


async def _retreat_for_retry(
    ctx: TaskContext,
    robot_id: str,
    run: servo.TrackState,
    comp: servo.PlantComp,
    cfg: servo.ServoConfig,
) -> None:
    """파지 재시도 후퇴 — rung1 standoff 로 물러나 재관측 준비. 관측 이력
    리셋 + 재획득 반경 확대는 run.reset_for_retry (근거 = servo.py 주석)."""
    cmd = comp.apply(servo.standoff(run.g_tcp, run.fam, cfg.standoffs[1]))
    await _move_l(  # 물체 옆에서 시작하는 후퇴 — 감속 (재밀침 방지)
        ctx, robot_id, cmd, run.fam.quat,
        speed_scale=cfg.gentle_speed_scale,
    )
    comp.commanded(cmd)
    run.reset_for_retry()


async def _servo_move(
    ctx: TaskContext,
    robot_id: str,
    position: Vec3,
    quat: Quat,
    trace: ServoTrace,
) -> None:
    """servo 보정/하강 이동 — MoveL(직선, 자세 고정) 우선, 거부 시 MoveJ 폴백
    1회 (관절 보간 — 목표 동일. 짧은 구간이라 스윙 미미), 둘 다 실패 = ServoFailed.

    실패를 침묵으로 넘기면 루프가 "명령한 증분은 항상 실행된다" 를 가정하게 된다
    (handoff §2 표) — 여기서 명시적으로 끊는다.
    """
    try:
        await _move_l(ctx, robot_id, position, quat)
        return
    except asyncio.CancelledError:
        raise
    except Exception as e_l:
        logger.warning(
            "servo 이동 MoveL 거부 (%s) — MoveJ 폴백: %s", _fmt(position), e_l
        )
        await _trace_emit(trace, {
            "phase": "move",
            "action": "movel_rejected",
            "reason": str(e_l),
            "target": [round(v, 4) for v in position],
        })
        try:
            await ctx.call(
                Motion.Service.MOVE_J,
                MoveJRequest(
                    target=PoseTarget(
                        kind="pose", position=position, quaternion=quat
                    )
                ),
                MoveJResponse,
                robot_id=robot_id,
            )
            return
        except asyncio.CancelledError:
            raise
        except Exception as e_j:
            raise ServoFailed(
                f"servo 이동 실패 — MoveL({e_l}) / MoveJ 폴백({e_j}). "
                "목표가 workspace 경계일 수 있습니다 — 물체를 로봇 쪽으로 "
                "옮긴 뒤 다시 실행하세요"
            ) from e_j


# ─── 찾기 (search 스윕 — coarse 전용) ─────────────────────────────────


@step(title="검출")
async def detect(
    ctx: TaskContext, robot_id: str, prompt: str
) -> list[OrientedDetection]:
    """search 그룹 자세를 **전부** 돌며 검출 → 후보 **누적** (첫 자세에서 안 멈춤).

    단일 시점 검출은 가림/시야/각도로 놓치거나 오검출한다 — 여러 관측 자세를 다
    돌아 모으면 강건. **선택은 안 함** — select_target_by_score 가 누적 전체에서.
    스윕 관측은 멀리서라 FK 오차가 크다 (실측: 카메라 31-33cm 에서 ~40mm) —
    coarse 위치 전용, 파지 정밀도는 servo 루프가 close 관측으로 잡는다.
    """
    t0 = time.monotonic()
    members = await _search_waypoints(ctx, robot_id)
    candidates: list[OrientedDetection] = []
    for wp in members:
        await _move_j_joints(ctx, robot_id, wp.joint_values)
        await asyncio.sleep(_SEARCH_SETTLE_S)  # MoveJ 후 카메라 정착 (검출 품질)
        res = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=robot_id, prompt=prompt, top_k=_TOP_K),
            DetectOrientedResponse,
        )
        if res.candidates:
            candidates.extend(res.candidates)
    logger.info(
        "detect(%s): search '%s' %d 자세 → 후보 누적 %d (%.1fs)",
        prompt, _SEARCH_GROUP, len(members), len(candidates),
        time.monotonic() - t0,
    )
    for i, c in enumerate(candidates):
        logger.info(
            "  후보%d: score=%.2f height(단일뷰)=%.1fcm base_z(물체바닥)=%.3fm "
            "top=%.3fm pos=(%.3f,%.3f)",
            i, c.score, c.height * 100.0, c.base_z, c.position[2],
            c.position[0], c.position[1],
        )
    return candidates


async def _search_waypoints(
    ctx: TaskContext, robot_id: str
) -> list[WaypointRecord]:
    """search 그룹 멤버(티칭 순서). 그룹 없음/빔 = 명시적 실패 (침묵 단일-뷰 폴백
    금지 — 사용자가 관측 자세를 티칭해야 multi-view 검색이 성립)."""
    groups = await ctx.call(
        Waypoint.Service.LIST_GROUPS,
        ListGroupsRequest(robot_id=robot_id),
        ListGroupsResponse,
    )
    grp = next((g for g in groups.groups if g.name == _SEARCH_GROUP), None)
    if grp is None or grp.id is None:
        raise TaskError(
            f"'{_SEARCH_GROUP}' waypoint 그룹 없음 (robot={robot_id}) — 검색 자세를 "
            "티칭해 '검색' 그룹으로 묶은 뒤 다시 실행하세요"
        )
    members = await ctx.call(
        Waypoint.Service.LIST_GROUP_MEMBERS,
        ListGroupMembersRequest(group_row_id=grp.id),
        ListGroupMembersResponse,
    )
    if not members.waypoints:
        raise TaskError(
            f"'{_SEARCH_GROUP}' 그룹이 비어있음 (robot={robot_id}) — 검색 자세를 "
            "이 그룹에 추가하세요"
        )
    return members.waypoints


async def _move_j_joints(
    ctx: TaskContext, robot_id: str, joints: list[float]
) -> None:
    """관절값으로 MoveJ (waypoint joint_values 그대로 — WaypointPanel 이동과 동일)."""
    await ctx.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=JointTarget(kind="joint", joints=list(joints))),
        MoveJResponse,
        robot_id=robot_id,
    )


def _xy_dist(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _nearest_within(
    cands: list[OrientedDetection], anchor: Vec3, radius_m: float
) -> OrientedDetection | None:
    """anchor 와 XY 거리 radius_m 안의 최근접 후보 (없으면 None)."""
    best, best_d = None, radius_m
    for c in cands:
        d = _xy_dist(c.position, anchor)
        if d <= best_d:
            best, best_d = c, d
    return best


def _neighbor_points(
    cands: list[OrientedDetection], coarse: OrientedDetection
) -> list[Vec3]:
    """타깃 아닌 이웃 후보의 점군 — 계획 resolve 충돌 게이트의 장애물.

    같은 prompt 로 잡힌 다른 물체 군집 (매치 반경 밖 ~ _NEIGHBOR_RADIUS_M 안).
    다른 prompt 의 물체는 지금 관측 채널이 없다 — 미관측 장애물은 실물 몫."""
    out: list[Vec3] = []
    for c in cands:
        d = _xy_dist(c.position, coarse.position)
        if d <= _VIEW_MATCH_RADIUS_M or d > _NEIGHBOR_RADIUS_M:
            continue
        out.extend(c.points or [])
    return out


@step(title="home 자세 조회")
async def home_waypoint(ctx: TaskContext, robot_id: str) -> WaypointRecord:
    """'home' waypoint 조회 (긴 이동 경유 자세). 없음 = 명시적 실패 — 모션 0
    시점(계획 전)에 걸리도록 시나리오 맨 앞에서 호출한다."""
    res = await ctx.call(
        Waypoint.Service.LIST,
        ListWaypointsRequest(robot_id=robot_id),
        ListWaypointsResponse,
    )
    wp = next((w for w in res.waypoints if w.name == _HOME_WAYPOINT), None)
    if wp is None:
        raise TaskError(
            f"'{_HOME_WAYPOINT}' waypoint 없음 (robot={robot_id}) — 픽↔플레이스"
            " 사이 경유할 안전 자세를 티칭해 'home' 으로 저장한 뒤 다시 실행하세요"
        )
    return wp


@step(title="적치 후보 선별")
async def resolve_place(
    ctx: TaskContext,
    robot_id: str,
    plan: list[PlaceCandidate],
    *,
    floor_z: float,
    home: WaypointRecord,
) -> tuple[int, list[list[float]]] | None:
    """한 spot 의 적치 후보 게이트 판정 (위치→자세→바닥→home→pre 관절 경로→
    pre↔place 직선) — 모션 0. 닿는 그룹 있으면 (index, solutions), 없으면 None.

    None = 이 spot 은 도달 불가 (부정 데이터 — 호출부가 다음 spot 으로 폴백).
    최종 실패 판정(모든 spot 소진)은 호출부 plan_place 가 raise."""
    t0 = time.monotonic()
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=geometry.place_ik_groups(plan),
            floor_z=floor_z,
            linear=True,
            path_from=list(home.joint_values),
        ),
        ResolveReachableResponse,
        robot_id=robot_id,
    )
    resolve_s = time.monotonic() - t0
    if res.index < 0:
        logger.info("resolve_place: 도달 불가 (%.1fs) — %s", resolve_s, res.message)
        return None
    logger.info(
        "resolve_place: group %d — %s (%.1fs)",
        res.index, plan[res.index].label, resolve_s,
    )
    return res.index, res.solutions


# ─── 놓기 실행 (open-loop 유지 — 상자 적치는 오차 관대) ───────────────


@step(title="놓기 실행")
async def execute_place(
    ctx: TaskContext,
    robot_id: str,
    c: PlaceCandidate,
    pre_joints: list[float],
    home: WaypointRecord,
) -> None:
    """계획된 적치 후보로 실제 적치 — 접근 → 삽입 → 내려놓기 → 후퇴 → home.

    home 에서 시작 (servo_pick 이 home 으로 끝남). 종료도 home — 다음 run 의
    시작 자세가 일정하고 카메라 시야에서 팔이 빠진다."""
    await pre_place(ctx, robot_id, pre_joints)
    await insert(ctx, robot_id, c)
    # 파지 판정: 내려놓기 직전에도 물고 있나 (이송 중 놓쳤으면 여기서 실패 —
    # 빈 손으로 release 하는 허위 성공 방지).
    await verify_grasp(ctx, robot_id, phase="적치 직전", grasp_label=c.label)
    await release(ctx, robot_id)
    await retreat(ctx, robot_id, c, pre_joints)
    await go_home(ctx, robot_id, home)
    # 마무리: 그리퍼 닫아 정리 자세 (열린 조가 대기 중 걸리적/충돌 표면 —
    # 사용자 요청 2026-07-17)
    await close_gripper(ctx, robot_id)


# ─── primitives ────────────


@step(title="home 경유")
async def go_home(ctx: TaskContext, robot_id: str, home: WaypointRecord) -> None:
    logger.info("go_home robot=%s → '%s'", robot_id, home.name)
    await _move_j_joints(ctx, robot_id, home.joint_values)


@step(title="적치 접근")
async def pre_place(
    ctx: TaskContext, robot_id: str, pre_joints: list[float]
) -> None:
    logger.info("pre_place robot=%s → joints=%s", robot_id, _fmt_joints(pre_joints))
    await _move_j_joints(ctx, robot_id, pre_joints)


@step(title="삽입")
async def insert(ctx: TaskContext, robot_id: str, c: PlaceCandidate) -> None:
    logger.info("insert robot=%s → place %s", robot_id, _fmt(c.place))
    await _move_l(ctx, robot_id, c.place, c.quat)


@step(title="적치 후퇴")
async def retreat(
    ctx: TaskContext,
    robot_id: str,
    c: PlaceCandidate,
    pre_joints: list[float],
) -> None:
    """place → pre 직선 후퇴. MoveL 실패 시 계획 관절해(pre_joints) MoveJ 폴백.

    2026-07-17 실물: 사전 검증 통과한 retreat MoveL 이 실행 중 끝점(=pre)에서
    IK 실패 — pre 는 resolve 가 관절해까지 증명했고 20초 전 MoveJ 로 실제 도달한
    자세다 (좁은 basin — 실행 seed 연쇄가 다른 branch 로 흘러 못 푸는 복권).
    알려진 해가 있는데 재풀이 실패로, **적치까지 성공한** task 를 죽이지
    않는다. 이 시점 물체는 이미 release 됨 + 폴백 시작 관절은 insert 로 온 구성
    근방이라 관절 보간 스윕 위험 낮음. (정석은 insert 궤적 역재생 — motion 단
    지원 필요, 2026-07-17 분석 §근본3. 폴백은 그 전까지의 안전망.)
    """
    logger.info("retreat robot=%s → pre %s", robot_id, _fmt(c.pre))
    try:
        await _move_l(ctx, robot_id, c.pre, c.quat)
    except asyncio.CancelledError:
        raise
    except Exception as e:
        logger.warning("retreat MoveL 실패 (%s) — 계획 관절해(pre) MoveJ 폴백", e)
        await _move_j_joints(ctx, robot_id, pre_joints)


@step(title="그리퍼 열기")
async def open_gripper(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=True)


@step(title="그리퍼 닫기")
async def close_gripper(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=False)


@step(title="내려놓기")
async def release(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=True)


# ─── 파지 판정 (물었나/놓쳤나) ────────────────────────────


# held 판정 부하 하한 — gap 이 작아도 (얇은 물체 / 슬립 후 조 끝 sliver 물림)
# 부하가 물체를 누르고 있으면 물림. 2026-07-17 실측 (so101 STS3215): 빈손 close
# = goal 도달이라 load 56~64 / sliver 물림 load 296 / 정상 물림 300~368 —
# 150 은 빈손×2 마진. ⚠ Feetech raw 기준 — 타 벤더(OMX Dynamixel) 부하 스케일
# 검증 전 (활성 robot 은 so101 뿐).
_HELD_LOAD_MIN_RAW = 150


def _gripper_holding(
    achieved_raw: int, load_raw: int | None, spec  # noqa: ANN001 — TaskRobotSpec
) -> bool:
    """물었나 판정 (벤더 무관). 신호 2개의 OR:

    ① gap = |achieved − close| > held margin (resolve.py, 5% range) — close 명령
      했는데 물체가 막아 완전히 못 닫힘 = 물림.
    ② 부하 ≥ _HELD_LOAD_MIN_RAW — 얇은 물체/슬립 sliver 는 gap 이 margin 아래로
      내려가지만 (2026-07-17 실물: gap 36 인데 실제로 물고 있었음 — 절대 gap
      문턱의 구조적 한계) 물체를 누르는 부하는 남는다. 빈손 close 는 goal 도달로
      부하가 낮아 (56~64) 구분된다.
    """
    margin = abs(spec.gripper_held_threshold_raw - spec.gripper_close_raw)
    gap = abs(achieved_raw - spec.gripper_close_raw)
    if gap > margin:
        return True
    return load_raw is not None and load_raw >= _HELD_LOAD_MIN_RAW


@step(title="파지 확인")
async def verify_grasp(
    ctx: TaskContext, robot_id: str, *, phase: str, grasp_label: str = ""
) -> dict:
    """실제 그리퍼 도달 위치로 물림 판정 — 빈 파지/놓침이면 GraspFailed raise.

    단일 시점·단일 신호의 허점(못 잡았는데 잡음/잡았다 놓침)을 줄이려 servo/place
    가 여러 시점(close 직후·withdraw 후·적치 직전)에서 이걸 부른다. 판정 근거(도달
    raw / close / threshold / load / 계획 폭)를 **전부 로깅** → 실패 시 원인분석 +
    실물 임계값 튜닝 데이터. fail-closed: 물림 확신 못 하면 실패로 기운다."""
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
    held = _gripper_holding(achieved, load, spec)
    logger.info(
        "verify_grasp[%s] robot=%s grip achieved=%d (close=%d open=%d held_thr=%d "
        "load=%s) 계획폭=%s → %s",
        phase, robot_id, achieved, spec.gripper_close_raw, spec.gripper_open_raw,
        spec.gripper_held_threshold_raw, load, grasp_label or "?",
        "HELD" if held else "EMPTY",
    )
    if not held:
        raise GraspFailed(
            phase=phase,
            achieved_raw=achieved,
            close_raw=spec.gripper_close_raw,
            load_raw=load,
        )
    # 성공 판정의 근거도 반환 — trace 에 남겨 임계 튜닝 데이터 (실패만 기록하면
    # "잡았을 때 raw/부하 분포"를 영영 못 본다. 2026-07-17 문턱 오판 진단 교훈).
    return {
        "achieved_raw": achieved,
        "gap_raw": abs(achieved - spec.gripper_close_raw),
        "load_raw": load,
    }


async def _log_reached_tcp(
    ctx: TaskContext, robot_id: str, *, expected: Vec3, phase: str
) -> None:
    """도달 TCP snapshot 로깅 — 계획 vs 실제 위치 오차. 실패 시 "arm 이 목표에
    도달했나"를 "기하가 틀렸나"와 분리하는 진단 신호 (침묵 X, 서비스 실패해도
    파지 흐름은 계속 — 로깅은 부수)."""
    try:
        tcp = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
            robot_id=robot_id,
        )
    except Exception as e:  # 로깅 실패가 파지를 막지 않게
        logger.warning("_log_reached_tcp[%s] TCP snapshot 실패: %s", phase, e)
        return
    a = tcp.position
    dx, dy, dz = a[0] - expected[0], a[1] - expected[1], a[2] - expected[2]
    err_mm = math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0
    logger.info(
        "reached[%s] robot=%s 계획=(%.3f,%.3f,%.3f) 도달=(%.3f,%.3f,%.3f) "
        "오차=%.1fmm",
        phase, robot_id, expected[0], expected[1], expected[2],
        a[0], a[1], a[2], err_mm,
    )


# ─── internal helpers ──


async def _move_l(
    ctx: TaskContext,
    robot_id: str,
    position: Vec3,
    quaternion: Quat,
    *,
    speed_scale: float = 1.0,
) -> None:
    await ctx.call(
        Motion.Service.MOVE_L,
        MoveLRequest(
            target=PoseTarget(kind="pose", position=position, quaternion=quaternion),
            speed_scale=speed_scale,
        ),
        MoveLResponse,
        robot_id=robot_id,
    )


async def _set_gripper(ctx: TaskContext, robot_id: str, *, open_: bool) -> None:
    spec = ctx.spec(robot_id)
    raw = spec.gripper_open_raw if open_ else spec.gripper_close_raw
    logger.info(
        "gripper robot=%s → %s (raw=%d)", robot_id, "OPEN" if open_ else "CLOSE", raw
    )
    await ctx.call(
        Motor.Service.SET_GRIPPER,
        SetGripperRequest(position_raw=raw),
        SetGripperResponse,
        robot_id=robot_id,
    )
    await asyncio.sleep(_GRIPPER_SETTLE_S)


def _fmt(pos: Vec3) -> str:
    return f"({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})"


def _fmt_joints(joints: list[float]) -> str:
    return "[" + ",".join(f"{j:.3f}" for j in joints) + "]"
