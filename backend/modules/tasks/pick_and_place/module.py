"""Pick & Place task 모듈 — so101 단팔. 검출(OBB) → 도달성 선별 → 파지 → (선택) 적치.

task 모듈 표준형 (첫 구현 = 레퍼런스): 평범한 모듈 + TaskRunner/TaskContextFactory
부품 조합. wire 핸들러는 runner 위임 one-liner, 도메인 로직은 _scenario (위에서
아래로 읽힘) + geometry.py 순수 함수. 취소/게이트/진행 발행/실패 처리는 runner 가 —
시나리오는 실패 시 raise 만 하면 FAILED + 사유 + 모터 정지가 자동.

새 task 모듈을 만들 때 이 파일 구조를 그대로 따라하면 됨 (contract.py + module.py
+ 순수 함수 파일 + TASK_INFO 등록).
"""

from __future__ import annotations

import logging

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.tasks.core.context import TaskContext, TaskContextFactory
from modules.tasks.core.metadata import TaskMetadata, register_task
from modules.tasks.core.runner import TaskRunner
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.core.contract import TaskState, TaskStepResult, TaskTrace

from . import geometry
from .contract import (
    ControlRequest,
    ControlResponse,
    PickAndPlace,
    RunRequest,
    RunResponse,
    RunToRequest,
    ToggleBreakpointRequest,
)

logger = logging.getLogger(__name__)

# GET /tasks 노출 — param 스펙은 RunRequest 에서 자동 파생 (hand-sync 없음).
TASK_INFO = register_task(
    TaskMetadata(
        name="pick_and_place",
        robots=["so101_6dof_0"],
        description="prompt 로 지정한 물체를 집어 올리고, place 대상 위에 내려놓는다",
        run=PickAndPlace.Service.RUN,
        params_model=RunRequest,
    )
)


@publishes(
    (PickAndPlace.Stream.STATE, TaskState),
    (PickAndPlace.Stream.TRACE, TaskTrace),
    (PickAndPlace.Stream.STEP_RESULT, TaskStepResult),
)
class PickAndPlaceModule:
    def __init__(
        self, runtime: ModuleRuntime, robots: dict[str, TaskRobotSpec]
    ) -> None:
        self.runtime = runtime
        # 프레임워크 부품 (조합 — 상속/자동배선 없음):
        # runner = 실행 생명주기 (robot 은 id 만 앎), contexts = 도메인 접근 (spec 보유).
        self.task = TaskRunner(runtime, streams=PickAndPlace.Stream)
        self.contexts = TaskContextFactory(runtime, robots)

    async def start(self) -> None:
        logger.info("PickAndPlaceModule start")

    async def stop(self) -> None:
        self.task.cancel()  # host 종료 시 활성 run 정리 (없으면 no-op 결과)

    # ─── wire (전부 runner 위임 — 트리거 어댑터) ─────────────────────

    @service(PickAndPlace.Service.RUN)
    async def run(self, req: RunRequest) -> RunResponse:
        r = self.task.start(
            self._scenario,
            ctx=self.contexts.create(),
            robot_ids=list(TASK_INFO.robots),
            task_name=TASK_INFO.name,
            pick_object=req.pick_object,
            place_object=req.place_object,
        )
        return RunResponse(accepted=r.accepted, message=r.message)

    @service(PickAndPlace.Service.STOP)
    async def stop_run(self, req: ControlRequest) -> ControlResponse:
        r = self.task.cancel()  # in-flight 모션 대기도 즉시 끊김 + Motion.STOP
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.PAUSE)
    async def pause(self, req: ControlRequest) -> ControlResponse:
        r = self.task.pause()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.RESUME)
    async def resume(self, req: ControlRequest) -> ControlResponse:
        r = self.task.resume()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.STEP_ONCE)
    async def step_once(self, req: ControlRequest) -> ControlResponse:
        r = self.task.step_once()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.RUN_TO)
    async def run_to(self, req: RunToRequest) -> ControlResponse:
        r = self.task.run_to(req.label)
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.TOGGLE_BREAKPOINT)
    async def toggle_breakpoint(self, req: ToggleBreakpointRequest) -> ControlResponse:
        r = self.task.toggle_breakpoint(req.label)
        return ControlResponse(ok=r.ok, message=r.message)

    # ─── 시나리오 (도메인 로직 — 실패는 raise, 나머지는 프레임워크) ──

    async def _scenario(
        self, ctx: TaskContext, pick_object: str, place_object: str = ""
    ) -> None:
        so101 = ctx.robot("so101_6dof_0")

        # 집을 것 찾기 — 후보 판단(prior/선택)과 접근 계획은 순수 함수 (geometry.py)
        cands = await so101.detect_oriented(pick_object, top_k=5, label="detect_pick")
        target = geometry.select_pick_target(cands, prompt=pick_object)
        plan = geometry.plan_grasp(target)
        idx = await so101.select_reachable(
            geometry.grasp_ik_groups(plan), label="select_grasp"
        )
        best = plan[idx]
        logger.info("approach 확정: %s", best.label)
        ctx.record("grasp", best)  # 3D 씬에 파지 계획 마커

        # 접근 → 파지 → 들어올림 (2026-07-09 실기 검증 시퀀스)
        await so101.move_j_pose(best.pre, best.quat, label="pre_grasp")
        await so101.gripper("open")  # 하강 전에 활짝 (가동 조가 반대편 안 치게)
        await so101.move_l(best.grasp, best.quat, label="descend")  # 자세 고정 하강
        await so101.gripper("close")  # 가동 조가 물체를 고정 조까지 밀며 클램프
        await so101.move_l(best.pre, best.quat, label="lift")

        if not place_object:
            return  # pick 만 — 든 채 종료 (놓을 곳 미지정)

        # 놓을 곳 찾기 → 위에 적치
        spots = await so101.detect_oriented(place_object, top_k=5, label="detect_place")
        spot = geometry.select_pick_target(spots, prompt=place_object)
        pplan = geometry.plan_place(spot, held=target, lateral=best.lateral)
        pidx = await so101.select_reachable(
            geometry.place_ik_groups(pplan), label="select_place"
        )
        drop = pplan[pidx]
        ctx.record("place", drop)

        await so101.move_j_pose(drop.pre, drop.quat, label="pre_place")
        await so101.move_l(drop.place, drop.quat, label="lower")
        await so101.gripper("open")  # release
        await so101.move_l(drop.pre, drop.quat, label="retreat")
