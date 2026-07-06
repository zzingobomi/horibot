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
from scipy.spatial.transform import Rotation

from modules.motion.contract import (
    Motion,
    MoveJRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    TcpSnapshotRequest,
    TcpState,
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
from ..steps import Gripper, MoveJ, MoveTCP, VerifyGrasp, Wait

logger = logging.getLogger(__name__)

APPROACH_STANDOFF = 0.06  # 접근축 뒤 standoff (m) — 옛 PRE_GRASP_DZ(수직 hover) 대체
LIFT_DZ = 0.08  # 파지 후 들어올림 (grasp 기준)
PLACE_STANDOFF = 0.05  # place 접근 standoff
HOME_WAYPOINT = "home"  # 종료 복귀 자세 (사용자 티칭 waypoint)


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


@dataclass(kw_only=True)
class ApproachAlongTool(Step[None]):
    """접근축(현재 tcp x = 그리퍼 pointing 방향)을 따라 standoff → target 진입.

    수직 하강이 기울어진 그리퍼 몸통으로 큐브를 옆으로 밀던 실패(2026-07-07) fix —
    손가락이 가리키는 선을 따라 미끄러져 들어가 정면 도달. 자세는 경로 내내 고정
    (MoveL target_quaternion). **접근각 = 직전 자세(search 마지막 waypoint)** —
    더 가파른 접근을 원하면 search 자세를 가파르게 티칭하면 됨.

    수직 top-down 강제는 기각: SO-101 기구 한계로 접근축 수직은 z≤0.05m 에서만
    reachable (FK 20만 샘플 조사, 2026-07-07) — hover 높이에선 물리적으로 불가."""

    target: SlotOr[Position3]
    standoff: float = APPROACH_STANDOFF

    async def execute(self, ctx: StepContext) -> None:
        tgt = ctx.resolve(self.target)
        if not isinstance(tgt, Position3):
            raise TypeError(
                f"ApproachAlongTool.target: Position3 기대, {type(tgt).__name__}"
            )
        snap = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState
        )
        quat = snap.quaternion
        adir = Rotation.from_quat(list(quat)).apply([1.0, 0.0, 0.0])
        pre = (
            tgt.x - float(adir[0]) * self.standoff,
            tgt.y - float(adir[1]) * self.standoff,
            tgt.z - float(adir[2]) * self.standoff,
        )
        logger.info(
            "ApproachAlongTool: standoff %.3f %.3f %.3f → target %.3f %.3f %.3f "
            "(adir=%.2f %.2f %.2f)  [%s]",
            *pre, tgt.x, tgt.y, tgt.z, *adir, self.label,
        )
        for phase, p in (("standoff", pre), ("target", (tgt.x, tgt.y, tgt.z))):
            res = await ctx.call(
                Motion.Service.MOVE_L,
                MoveLRequest(target_position=p, target_quaternion=quat),
                MoveLResponse,
                timeout=60.0,
            )
            if not res.accepted:
                raise RuntimeError(
                    f"ApproachAlongTool {phase} MoveL 거부: {res.message} [{self.label}]"
                )


@dataclass(kw_only=True)
class RetreatAlongTool(Step[None]):
    """현재 접근축 반대 방향으로 standoff 후퇴 (자세 고정) — release 후 이탈용.

    수직 상승은 기울어진 그리퍼가 상자 테두리를 칠 수 있음 — 들어온 선 그대로 후퇴."""

    standoff: float = PLACE_STANDOFF

    async def execute(self, ctx: StepContext) -> None:
        snap = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState
        )
        quat = snap.quaternion
        adir = Rotation.from_quat(list(quat)).apply([1.0, 0.0, 0.0])
        back = (
            snap.position[0] - float(adir[0]) * self.standoff,
            snap.position[1] - float(adir[1]) * self.standoff,
            snap.position[2] - float(adir[2]) * self.standoff,
        )
        res = await ctx.call(
            Motion.Service.MOVE_L,
            MoveLRequest(target_position=back, target_quaternion=quat),
            MoveLResponse,
            timeout=60.0,
        )
        if not res.accepted:
            raise RuntimeError(f"RetreatAlongTool MoveL 거부: {res.message} [{self.label}]")


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
        # 접근축 진입 — 기울어진 그리퍼의 수직 하강이 큐브를 밀던 실패 fix.
        ApproachAlongTool(
            target=grasp.out, standoff=APPROACH_STANDOFF, label="approach_grasp"
        ),
        Gripper(action="close", label="close_gripper"),
        VerifyGrasp(label="verify_grasp"),
        Wait(duration_sec=0.5, label="grip_settle"),
        # lift — 자세 고정(hold) 수직 상승 (파지 후 자세 흔들림 방지).
        MoveTCP(
            target=grasp.out,
            offset=Position3(x=0.0, y=0.0, z=LIFT_DZ),
            orientation="hold",
            label="lift",
        ),
        VerifyGrasp(label="verify_after_lift"),
    ]

    if select_place is not None:
        place_xyz = PlacePolicy(target=select_place.out, label="place_policy")
        steps += [
            place_xyz,
            ApproachAlongTool(
                target=place_xyz.out, standoff=PLACE_STANDOFF, label="approach_place"
            ),
            VerifyGrasp(label="verify_before_release"),
            Gripper(action="open", label="release"),
            Wait(duration_sec=0.3, label="release_settle"),
            # 들어온 선 그대로 후퇴 — 수직 상승은 그리퍼가 상자 테두리를 칠 수 있음.
            RetreatAlongTool(standoff=PLACE_STANDOFF, label="retreat_place"),
        ]

    steps.append(MoveJ(waypoint=HOME_WAYPOINT, label="return_home"))
    return TaskSpec(name="pick_and_place", steps=steps, description=desc)
