"""접근·관측 — 파지/적치 계획을 '멀리서(스윕)'가 아니라 '가까이서' 세우기 위한
앞단 (2026-07-21 재구조, docs/pnp_scenario_rework.md §3.2 "가족 이사").

스윕 관측은 카메라 31-33cm 라 ~40mm 오차 → yaw 를 못 믿어 312 가족으로 헤지(느림
+ 겨우 닿는 자세 채택 위험). 물체 위 **관측 자세**(standoff ~8cm = 카메라 ~14cm
= servo.py 실측 최적 대역, base 관측 편차 5-12mm)로 팔을 가져가 다시 관측하면
정확도가 좋아진다 — 그 정확 관측으로 plan_pick/plan_place 가 돈다.

**servo 는 안 건드린다** — 이 step 은 servo *앞에서* 더 좋은 입력을 만들 뿐.
관측 자세 선택 = **도달 편함 우선(수직 강제 X)** — 수직 강제가 IK 전멸의 원인이었다
(§5-2). servo_ladder 와 동일하게 잔차선호 resolve(motion._RESOLVE_RESIDUAL_GOOD_MM)
로 "가장 잘 닿는" look-pose 를 고른다. 각 tilt 당 대표 yaw 1개만(관측엔 조 방향이
무관) — resolve 후보를 소수로.

실패는 침묵하지 않되 치명적이지 않다: 관측 자세 도달 불가/close 관측 0 이면
**coarse 관측으로 폴백**(경고 로그 + 계획은 예전처럼 멀리서). 회귀 아님 — 07-19
까지 돌던 경로로 degrade.
"""

from __future__ import annotations

import asyncio
import logging
import time

from modules.detector.contract import (
    DetectOrientedResponse,
    DetectRequest,
    Detector,
    FuseOrientedRequest,
    FuseOrientedResponse,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    ResolveReachableRequest,
    ResolveReachableResponse,
    TcpPose,
)
from modules.tasks.core.context import TaskContext
from modules.tasks.core.step import step
from modules.waypoint.contract import WaypointRecord

from .. import servo
from . import primitives
from .primitives import (
    _TOP_K,
    _VIEW_MATCH_RADIUS_M,
    _move_j_joints,
    _nearest_within,
    _xy_dist,
    go_home,
)

logger = logging.getLogger(__name__)

# look-pose TCP standoff — 접근축 후방 거리. 카메라가 TCP 후방 ~77mm 라 이 값이
# 8cm 면 카메라-물체 ~14cm = servo.py 실측 최적 관측 대역(편차 5-12mm).
_OBSERVE_STANDOFF_M = 0.08
_OBSERVE_SETTLE_S = 0.3  # MoveJ 후 카메라 진동 정착 (검출 품질)
# 관측 프레임 수. 1 = 가까이서 한 번(카메라 14cm 편차 5-12mm 로 이미 coarse 대비
# 충분). 다중프레임 융합(노이즈↓)은 실물 첫 런 데이터로 이득 보이면 올린다 —
# servo tick 루프가 하강 중 재관측·refit 로 위치는 이미 계속 보정. ⚠ 실물 튜닝점.
_OBSERVE_FRAMES = 1


def _look_families(coarse: OrientedDetection) -> list[servo.GraspFamily]:
    """관측 look-pose 후보 = tilt 당 대표(면정렬 최우선) 가족 1개.

    관측엔 조 방향(yaw)·flip 이 무관하니 파지 격자(312)를 다 풀 필요 없다 —
    tilt 사다리별 대표만(수직부터 선호순). '가장 잘 닿는' 것은 resolve 잔차선호."""
    seen_tilt: set[int] = set()
    out: list[servo.GraspFamily] = []
    for fam in servo.grasp_families(coarse):
        if fam.flip != 1.0 or fam.tilt_deg in seen_tilt:
            continue
        seen_tilt.add(fam.tilt_deg)
        out.append(fam)
    return out


@step(title="접근·관측")
async def approach_observe(
    ctx: TaskContext,
    robot_id: str,
    coarse_cands: list[OrientedDetection],
    prompt: str,
    home: WaypointRecord,
) -> tuple[list[OrientedDetection], list[float], bool]:
    """coarse 후보 → 최고 score 대상 위 관측 자세로 이동 → 정확 관측.

    반환 = (계획용 후보 리스트, 관측 자세 joints, **close: 정확 관측 성공 여부**).
    계획 리스트 = [정확 관측, *coarse 이웃] (이웃은 plan 의 장애물/바닥 문맥 유지 —
    타깃만 정확본으로 교체). 관측 실패(자세 도달 불가/close 후보 0) 시 (coarse_cands,
    joints, **False**) = 폴백(멀리서 계획, yaw 격자 유지). close=True 면 호출부가
    관측 yaw 를 믿어 파지 yaw 격자를 끈다 (plan_pick trust_yaw)."""
    if not coarse_cands:
        return coarse_cands, list(home.joint_values), False  # plan_pick 이 0 처리
    cfg = primitives._SERVO_CFG
    target = max(coarse_cands, key=lambda c: c.score)
    point = servo.grasp_point(target, target, cfg, None)
    fams = _look_families(target)
    groups = [
        [
            TcpPose(
                position=servo.standoff(
                    servo.grasp_tcp(point, f, 0.0), f, _OBSERVE_STANDOFF_M
                ),
                quaternion=f.quat,
            )
        ]
        for f in fams
    ]
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=groups, path_from=list(home.joint_values)),
        ResolveReachableResponse,
        robot_id=robot_id,
    )
    if res.index < 0:
        logger.warning(
            "approach_observe(%s): 관측 자세 도달 불가 (%d후보 전멸: %s) — coarse "
            "관측 유지, 계획은 멀리서 + yaw 격자 (폴백)", prompt, len(groups), res.message,
        )
        return coarse_cands, list(home.joint_values), False
    look_joints = res.solutions[0]
    await go_home(ctx, robot_id, home)
    await _move_j_joints(ctx, robot_id, look_joints)
    await asyncio.sleep(_OBSERVE_SETTLE_S)

    t0 = time.monotonic()
    seen: list[OrientedDetection] = []
    for _ in range(_OBSERVE_FRAMES):
        det = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=robot_id, prompts=[prompt], top_k=_TOP_K),
            DetectOrientedResponse,
        )
        near = _nearest_within(det.candidates, target.position, _VIEW_MATCH_RADIUS_M)
        if near is not None:
            seen.append(near)
    if not seen:
        logger.warning(
            "approach_observe(%s): close 관측 0프레임 (타깃 근방 후보 없음) — "
            "coarse 유지 + yaw 격자 (폴백)", prompt,
        )
        return coarse_cands, look_joints, False

    accurate = await _fuse(ctx, seen, target.position)
    logger.info(
        "approach_observe(%s): close 관측 %d/%d프레임 → pos=(%.3f,%.3f) base_z=%.3f "
        "(coarse=(%.3f,%.3f), %.1fs)",
        prompt, len(seen), _OBSERVE_FRAMES,
        accurate.position[0], accurate.position[1], accurate.base_z,
        target.position[0], target.position[1], time.monotonic() - t0,
    )
    # 타깃 클러스터는 정확본으로 교체, 이웃(다른 물체)은 문맥 유지.
    neighbors = [
        c for c in coarse_cands
        if _xy_dist(c.position, accurate.position) > _VIEW_MATCH_RADIUS_M
    ]
    return [accurate, *neighbors], look_joints, True


async def _fuse(
    ctx: TaskContext, seen: list[OrientedDetection], anchor: tuple[float, float, float]
) -> OrientedDetection:
    """관측 프레임 융합 → 타깃 군집. 2프레임 미만/군집 없음이면 최신 단독(침묵 X)."""
    if len(seen) < 2:
        return seen[-1]
    res = await ctx.call(
        Detector.Service.FUSE_ORIENTED,
        FuseOrientedRequest(candidates=list(seen)),
        FuseOrientedResponse,
    )
    near = _nearest_within(res.candidates, anchor, _VIEW_MATCH_RADIUS_M)
    if near is None:
        logger.info("approach_observe: 융합 군집 없음 (%d프레임) — 최신 단독", len(seen))
        return seen[-1]
    return near
