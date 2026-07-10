"""단팔 pick-and-place — task #1 (§17.1 첫 task, §17.5 recipe).

§17.1③ "거의 DSL 없이 평범한 async 함수" — 검출 순회/후보 누적은 generic ForEach(=
Orchestration, rule-of-three defer)가 아니라 **task-local domain step (SearchWaypoint
Group) 안의 평범한 파이썬 loop**. Day-1(MoveJ/MoveTCP/Gripper/VerifyGrasp)은 steps.py
재사용, 여기 정의하는 domain step 은 이 task 로컬 (§17.1 Domain = task 로컬 → rule of
three 승격 전엔 shared 로 안 올림).

recipe (§17.5, 옛 backend/modules/task/tasks/pick_and_place.py 흐름 포팅):
  open → SearchWaypointGroup(pick) → SelectTarget(pick) → [place 검색도] →
  GraspPolicy → pre_grasp/grasp/close/verify/lift/verify → [PlacePolicy → place seq] → home

옛 first-match-break(search_and_detect) 대체 = **Waypoint Group 전 자세 순회 + 후보
누적 → SelectTarget** (여러 자세 관측 → 오검출 강건, §17.5). base_z/height 기하 prior
+ 스코어링 임계는 실물 tuning (§17.5 "정확도 = 집 하드웨어").
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass

from modules.detector.contract import (
    DetectRequest,
    DetectResponse,
    Detection,
    Detector,
)
from modules.motion.contract import (
    Motion,
    MoveJRequest,
    MoveJResponse,
)
from modules.waypoint.contract import (
    ListGroupMembersRequest,
    ListGroupMembersResponse,
    ListGroupsRequest,
    ListGroupsResponse,
    Waypoint,
)

from ..schema import Position3, SlotOr
from ..step import Step, StepContext, TaskSpec
from ..steps import Gripper, MoveJ, MoveToPose, VerifyGrasp, Wait

logger = logging.getLogger(__name__)

PRE_GRASP_DZ = 0.06  # grasp 위 hover (m)
LIFT_DZ = 0.08  # 파지 후 들어올림 (grasp 기준)
PLACE_HOVER_DZ = 0.05  # place 위 hover
HOME_WAYPOINT = "home"  # 종료 복귀 자세 (사용자 티칭 waypoint)
# tcp≠파지점(단일 가동 jaw 그리퍼) 보정 — tool frame (x=접근축 / -y=고정 jaw 쪽, m).
# 그리퍼 상수(큐브 무관). MoveToPose 가 tcp 대신 tcp+PINCH_OFFSET 을 target 에 맞춤
# → 큐브가 파지 중심(고정 jaw 쪽)에 놓임.
# ⚠ 값은 **rough URDF 추정** (AABB 기반, 접촉면 정밀도 부족) — 실물에서 미스 방향
#   보고 튜닝할 것 (옆으로 밀리면 y 크기↑, 반대면 부호). 정밀히는 실측:
#   큐브 문 자세(토크오프)에서 tcp pose vs 큐브 base → tool frame 차이.
PINCH_OFFSET: tuple[float, float, float] | None = (0.0, -0.015, 0.0)


# ─── 기하 prior (§17.5 ②) ────────────────────────────────────────────


@dataclass(frozen=True)
class GeometricPrior:
    """base_z / height 예상 범위 (§17.5 ② "예상 범위 밖 reject"). None = 필터 안 함.

    임계값 자체는 **실물 tuning** (§17.5 "스코어링 = 집 하드웨어") — 구조(필터 훅)만
    지금, 실 데이터 보며 범위 채움. confidence 무관 기하 구분 (흰 큐브 vs 바닥 흰 천)."""

    base_z_range: tuple[float, float] | None = None
    height_range: tuple[float, float] | None = None

    def accepts(self, det: Detection) -> bool:
        if self.base_z_range is not None:
            lo, hi = self.base_z_range
            if not lo <= det.base_z <= hi:
                return False
        if self.height_range is not None:
            lo, hi = self.height_range
            if not lo <= det.height <= hi:
                return False
        return True


# ─── task-local domain step ──────────────────────────────────────────


@dataclass(kw_only=True)
class SearchWaypointGroup(Step[list[Detection]]):
    """Waypoint Group 의 각 자세로 이동하며 Detector Top-K 검출 → 후보 누적 (§17.5).

    옛 search_and_detect(첫 검출 break) 대체 — 첫 자세에서 안 멈추고 group 전 자세를
    돌며 후보를 모은다. 이 step 안의 평범한 loop (§17.1③, generic ForEach 아님). 최종
    선택은 SelectTarget. group 은 robot-agnostic (robot_id=req body, §2.7), MoveJ 는
    robot-scoped (ctx.call 키 라우팅)."""

    prompt: SlotOr[str]
    group: str
    top_k: int = 5
    settle_sec: float = 0.3

    async def execute(self, ctx: StepContext) -> list[Detection]:
        prompt = ctx.resolve(self.prompt)
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError(f"SearchWaypointGroup.prompt 비어있음 [{self.label}]")
        prompt = prompt.strip()

        # 1. group 이름 → row id → 멤버 (순서). 둘 다 robot-agnostic.
        groups = await ctx.runtime.call(
            Waypoint.Service.LIST_GROUPS,
            ListGroupsRequest(robot_id=ctx.robot_id),
            ListGroupsResponse,
        )
        grp = next((g for g in groups.groups if g.name == self.group), None)
        if grp is None or grp.id is None:
            raise RuntimeError(
                f"SearchWaypointGroup: group '{self.group}' 없음 "
                f"(robot={ctx.robot_id}) — 티칭 필요 [{self.label}]"
            )
        members = await ctx.runtime.call(
            Waypoint.Service.LIST_GROUP_MEMBERS,
            ListGroupMembersRequest(group_row_id=grp.id),
            ListGroupMembersResponse,
        )
        if not members.waypoints:
            raise RuntimeError(
                f"SearchWaypointGroup: group '{self.group}' 비어있음 [{self.label}]"
            )

        # 2. 각 자세: MoveJ → settle → Detect Top-K → 후보 누적 (평범한 loop).
        candidates: list[Detection] = []
        for wp in members.waypoints:
            mv = await ctx.call(
                Motion.Service.MOVE_J,
                MoveJRequest(target_joints=wp.joint_values),
                MoveJResponse,
                timeout=60.0,
            )
            if not mv.accepted:
                raise RuntimeError(
                    f"SearchWaypointGroup: MoveJ '{wp.name}' 거부: {mv.message}"
                )
            await asyncio.sleep(self.settle_sec)
            det = await ctx.runtime.call(
                Detector.Service.DETECT,
                DetectRequest(robot_id=ctx.robot_id, prompt=prompt, top_k=self.top_k),
                DetectResponse,
                timeout=60.0,
            )
            if det.found:
                candidates.extend(det.candidates)
        logger.info(
            "SearchWaypointGroup '%s' group=%s: %d 자세 → 후보 %d  [%s]",
            prompt, self.group, len(members.waypoints), len(candidates), self.label,
        )
        return candidates


@dataclass(kw_only=True)
class SelectTarget(Step[Detection]):
    """누적 후보 중 기하 prior 통과 + 최고 score 1개 (§17.5 SelectTarget).

    prior 미지정 시 range 필터 없이 최고 score (회귀 없음 — 옛 first-match 와 달리 전
    자세 후보에서 최선). 통과 후보 0 이면 fail (의도적, 오검출 넘어감 방지)."""

    candidates: SlotOr[list[Detection]]
    prompt: SlotOr[str] = ""
    priors: GeometricPrior | None = None

    async def execute(self, ctx: StepContext) -> Detection:
        cands = ctx.resolve(self.candidates)
        if not isinstance(cands, list) or not cands:
            raise RuntimeError(f"SelectTarget: 후보 없음 [{self.label}]")
        prior = self.priors or GeometricPrior()
        passed = [c for c in cands if prior.accepts(c)]
        if not passed:
            raise RuntimeError(
                f"SelectTarget: 기하 prior 통과 후보 없음 "
                f"(후보 {len(cands)}) [{self.label}]"
            )
        best = max(passed, key=lambda c: c.score)
        logger.info(
            "SelectTarget: 후보 %d → prior통과 %d → best score=%.2f "
            "base_z=%.3f h=%.3f  [%s]",
            len(cands), len(passed), best.score, best.base_z, best.height,
            self.label,
        )
        return best


@dataclass(kw_only=True)
class GraspPolicy(Step[Position3]):
    """Detection → 옆면 grasp Position3 (§17.5 "순수 계산", v1 포팅).

    grasp_z = base_z + height * grasp_ratio (0.5 = 옆면 중간). x/y 는 검출 중심."""

    target: SlotOr[Detection]
    grasp_ratio: float = 0.5

    async def execute(self, ctx: StepContext) -> Position3:
        det = ctx.resolve(self.target)
        if not isinstance(det, Detection):
            raise TypeError(
                f"GraspPolicy.target: Detection 기대, {type(det).__name__}"
            )
        grasp_z = det.base_z + det.height * self.grasp_ratio
        out = Position3(x=det.position[0], y=det.position[1], z=grasp_z)
        logger.info(
            "GraspPolicy base_z=%.3f height=%.3f → grasp_z=%.3f  [%s]",
            det.base_z, det.height, grasp_z, self.label,
        )
        return out


@dataclass(kw_only=True)
class PlacePolicy(Step[Position3]):
    """place 객체 윗면 + drop_clearance — 공중에서 안 떨구게 (§17.5 순수 계산, v1 포팅)."""

    target: SlotOr[Detection]
    drop_clearance: float = 0.010

    async def execute(self, ctx: StepContext) -> Position3:
        det = ctx.resolve(self.target)
        if not isinstance(det, Detection):
            raise TypeError(
                f"PlacePolicy.target: Detection 기대, {type(det).__name__}"
            )
        place_z = det.position[2] + self.drop_clearance
        out = Position3(x=det.position[0], y=det.position[1], z=place_z)
        logger.info(
            "PlacePolicy: top_z=%.3f → place_z=%.3f (clearance=%.3f)  [%s]",
            det.position[2], place_z, self.drop_clearance, self.label,
        )
        return out


# ─── task factory ────────────────────────────────────────────────────


def create_pick_and_place_task(
    pick_object: str,
    place_object: str | None = None,
    *,
    search_group: str = "search",
) -> TaskSpec:
    """pick_object 를 집어 place_object(있으면) 에 둔다. LLM 이 파싱한 (pick, place) 주입.

    pick/place 둘 다 검출 대상 — search_group 의 자세들을 순회하며 후보 누적 후 선택.
    """
    desc = (
        f"'{pick_object}' 집어서 '{place_object}' 에 두기"
        if place_object
        else f"'{pick_object}' 집기"
    )

    search_pick = SearchWaypointGroup(
        prompt=pick_object, group=search_group, label=f"search_pick:{pick_object}"
    )
    select_pick = SelectTarget(
        candidates=search_pick.out, prompt=pick_object, label="select_pick"
    )
    grasp = GraspPolicy(target=select_pick.out, label="grasp_policy")

    steps: list[Step] = [
        Gripper(action="open", label="open_gripper"),
        search_pick,
        select_pick,
    ]

    # place 도 팔로 가리기 전에 미리 검출 (v1 순서).
    select_place: SelectTarget | None = None
    if place_object:
        search_place = SearchWaypointGroup(
            prompt=place_object, group=search_group,
            label=f"search_place:{place_object}",
        )
        select_place = SelectTarget(
            candidates=search_place.out, prompt=place_object, label="select_place"
        )
        steps += [search_place, select_place]

    steps += [
        grasp,
        # 관절 공간 config 전이(MoveToPose = pose→IK→MoveJ). Cartesian 직선(MoveL)은
        # SO-101 workspace 에서 "자세 고정한 채 이동" 이 높이마다 변하는 도달 자세를
        # 못 덮어 IK 실패 — 관절 이동으로 전환(2026-07-07). 위치는 어디든 도달하고
        # 자세는 경로 따라 자유. approach=위 config, grasp=파지 config.
        MoveToPose(
            target=grasp.out,
            offset=Position3(x=0.0, y=0.0, z=PRE_GRASP_DZ),
            label="pre_grasp",
        ),
        MoveToPose(target=grasp.out, tool_offset=PINCH_OFFSET, label="grasp"),
        Gripper(action="close", label="close_gripper"),
        VerifyGrasp(label="verify_grasp"),
        Wait(duration_sec=0.5, label="grip_settle"),
        # lift = 월드 Z 상승이 아니라 "위 config 로 복귀" → 큐브 자연 상승 (관절 이동).
        MoveToPose(
            target=grasp.out,
            offset=Position3(x=0.0, y=0.0, z=LIFT_DZ),
            label="lift",
        ),
        VerifyGrasp(label="verify_after_lift"),
    ]

    if select_place is not None:
        place_xyz = PlacePolicy(target=select_place.out, label="place_policy")
        steps += [
            place_xyz,
            MoveToPose(
                target=place_xyz.out,
                offset=Position3(x=0.0, y=0.0, z=PLACE_HOVER_DZ),
                label="pre_place",
            ),
            MoveToPose(target=place_xyz.out, tool_offset=PINCH_OFFSET, label="place"),
            VerifyGrasp(label="verify_before_release"),
            Gripper(action="open", label="release"),
            Wait(duration_sec=0.3, label="release_settle"),
            MoveToPose(
                target=place_xyz.out,
                offset=Position3(x=0.0, y=0.0, z=PLACE_HOVER_DZ),
                label="post_place_retreat",
            ),
        ]

    steps.append(MoveJ(waypoint=HOME_WAYPOINT, label="return_home"))
    return TaskSpec(name="pick_and_place", steps=steps, description=desc)
