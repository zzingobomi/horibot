from __future__ import annotations

import asyncio
import logging
import math
import time

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.tasks.core.context import TaskContext, TaskContextFactory
from modules.tasks.core.contract import (
    ControlRequest,
    ControlResponse,
    PreviewRequest,
    PreviewResponse,
    RunResponse,
    RunToRequest,
    TaskState,
    TaskTrace,
    ToggleBreakpointRequest,
    TraceEntry,
)
from modules.tasks.core.preview import build_preview
from modules.tasks.core.runner import RunState, TaskRunner
from modules.tasks.core.spec import TaskRobotSpec

from . import servo, steps
from .contract import (
    ListRobotsRequest,
    ListRobotsResponse,
    PickAndPlace,
    RunRequest,
    TaskMarker,
    TaskMarkers,
)

logger = logging.getLogger(__name__)


@publishes(
    (PickAndPlace.Stream.STATE, TaskState),
    (PickAndPlace.Stream.TRACE, TaskTrace),
    (PickAndPlace.Stream.MARKERS, TaskMarkers),
)
class PickAndPlaceModule:
    TASK_ROBOTS = ("so101_6dof_0",)

    task = TaskRunner()

    def __init__(
        self,
        runtime: ModuleRuntime,
        robots: dict[str, TaskRobotSpec] | None = None,
    ) -> None:
        self.runtime = runtime
        self.contexts = TaskContextFactory(runtime, robots)
        self._seq = {"state": 0, "trace": 0, "markers": 0}

    async def stop(self) -> None:
        self.task.cancel()

    # ─── Services ──

    @service(PickAndPlace.Service.RUN)
    async def run(self, req: RunRequest) -> RunResponse:
        r = self.task.start(
            self.scenario,
            ctx=self.contexts.create(),
            robot_ids=list(self.TASK_ROBOTS),
            task_name="pick_and_place",
            pick_object=req.pick_object,
            place_object=req.place_object,
        )
        return RunResponse(accepted=r.accepted, message=r.message)

    @service(PickAndPlace.Service.STOP)
    async def stop_run(self, req: ControlRequest) -> ControlResponse:
        r = self.task.cancel()
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
        r = self.task.run_to(req.name)
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.TOGGLE_BREAKPOINT)
    async def toggle_breakpoint(self, req: ToggleBreakpointRequest) -> ControlResponse:
        r = self.task.toggle_breakpoint(req.name)
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.LIST_ROBOTS)
    async def list_robots(self, req: ListRobotsRequest) -> ListRobotsResponse:
        return ListRobotsResponse(robot_ids=list(self.TASK_ROBOTS))

    @service(PickAndPlace.Service.PREVIEW)
    async def preview(self, req: PreviewRequest) -> PreviewResponse:
        entries = await asyncio.to_thread(build_preview, self.scenario)
        return PreviewResponse(entries=entries)

    # ─── Publishing ────

    @task.on_state
    def _publish_state(self, s: RunState) -> None:
        for robot_id in s.robot_ids or self.TASK_ROBOTS:
            self.runtime.publish(
                PickAndPlace.Stream.STATE,
                TaskState(
                    robot_id=robot_id,
                    seq=self._next_seq("state"),
                    timestamp_unix=time.time(),
                    status=s.status,
                    task_name=s.task_name,
                    current_name=s.current_name,
                    current_title=s.current_title,
                    error=s.error,
                    breakpoints=list(s.breakpoints),
                ),
            )

    @task.on_trace
    def _publish_trace(self, s: RunState, entries: list[TraceEntry]) -> None:
        for robot_id in s.robot_ids:
            self.runtime.publish(
                PickAndPlace.Stream.TRACE,
                TaskTrace(
                    robot_id=robot_id,
                    seq=self._next_seq("trace"),
                    timestamp_unix=time.time(),
                    task_name=s.task_name,
                    entries=list(entries),
                ),
            )

    def _publish_markers(self, robot_id: str, markers: list[TaskMarker]) -> None:
        self.runtime.publish(
            PickAndPlace.Stream.MARKERS,
            TaskMarkers(
                robot_id=robot_id,
                seq=self._next_seq("markers"),
                timestamp_unix=time.time(),
                markers=list(markers),
            ),
        )

    def _next_seq(self, stream: str) -> int:
        seq = self._seq[stream]
        self._seq[stream] = seq + 1
        return seq

    # ─── Task Scenarios ─────────

    async def scenario(
        self,
        ctx: TaskContext,
        pick_object: str,
        place_object: str = "",
    ) -> None:
        so101 = self.TASK_ROBOTS[0]

        home = await steps.home_waypoint(ctx, so101)

        # search 자세 순회하며 detect
        prompts = [pick_object]
        if place_object and place_object != pick_object:
            prompts.append(place_object)
        found = await steps.detect(ctx, so101, prompts)

        # 2) 접근·관측 + 계획 (2026-07-21 재구조) — **관측(팔 이동)은 상자 먼저 →
        # 물건 마지막**: 물건을 마지막에 봐야 관측이 최신이고, 물건 본 뒤엔 계획
        # (plan_* = 모션 0, 팔 안 움직임)만 하다 **바로 집으러** 간다 (상자로 왕복
        # 없음). 놓기 계획은 집기와 독립(상자 정중앙 위, 물건 폭/높이 무시 —
        # geometry TODO)이라 먼저 세운다. 집기·놓기 도달성을 **둘 다 집기 전에**
        # 검증 — 놓을 곳 도달 불가면 아무것도 안 집는다 (쥔 채 멈춤 corrupt 방지).
        drop, drop_pre, place_marker = None, None, None
        if place_object:
            # 상자 빈손 관측 (§3.3 — 든 물건이 상자 가리는 것 회피, 집기 전 관측)
            # → 놓기 계획 (coarse 스윕 노이즈 대신 정확 관측).
            place_cands, _, _ = await steps.approach_observe(
                ctx, so101, found.get(place_object, []), place_object, home
            )
            drop, drop_pre = await steps.plan_place(
                ctx,
                so101,
                place_object,
                home=home,
                spots=place_cands,
            )
            place_marker = TaskMarker(
                label="place",
                position=drop.place,
                # 적치 진입 방향 = pre→place (수직 삽입 축), 자세 = 계획 quat.
                # 조 축은 place 마커엔 생략 (release 는 조 방향이 관심사 아님).
                approach=_unit_dir(drop.pre, drop.place),
                quaternion=drop.quat,
            )

        # 물건은 **마지막에** 관측 → 파지 계획 (그 뒤엔 이동 없이 바로 집으러).
        # 가까이 정확 관측 성공(pick_close) 시 관측 yaw 를 믿어 파지 yaw 격자를
        # 끈다 (trust_yaw → 가족 312→~52, 전멸 CT 6배↓). 폴백이면 격자 유지.
        pick_cands, _, pick_close = await steps.approach_observe(
            ctx, so101, found.get(pick_object, []), pick_object, home
        )
        plan = await steps.plan_pick(
            ctx,
            so101,
            pick_object,
            home,
            pick_cands,
            trust_yaw=pick_close,
        )

        # 마커: 파지[0] + 적치[1] (on_grasp 가 [0] 만 실시간 갱신, [1:] 유지).
        markers = [_grasp_marker(plan.grasp_point0, plan.family)]
        if place_marker is not None:
            markers.append(place_marker)
        # 계획 확정 시점에 마커 표시 (파지·적치 지점을 실행 전 미리 보여줌).
        self._publish_markers(so101, markers)

        # servo 가 파지점을 갱신할 때마다 마커 재발행 — 계획 시점 마커가 실행
        # 내내 고정 표시되던 UI 구멍 (2026-07-17 사용자 리포트. 스트림은
        # latest-wins 라 매 채택 발행이 곧 실시간 표시). fam 도 함께 — refit/
        # 재플랜으로 파지 방향이 바뀌면 화살표·조 축 바가 실시간 따라간다.
        def on_grasp(p: tuple[float, float, float], fam: servo.GraspFamily) -> None:
            self._publish_markers(
                so101,
                [
                    _grasp_marker(p, fam),
                    *markers[1:],  # place 마커 유지 (계획값 — 적치는 open-loop)
                ],
            )

        # 3) 실행 — 집기 = closed-loop servo (물체 근처에서 관측→보정 루프,
        # steps/servo.py 정본), 놓기 = open-loop (상자 적치는 오차 관대).
        await steps.servo_pick(ctx, so101, plan, pick_object, home, on_grasp)
        if drop is not None and drop_pre is not None:
            await steps.execute_place(ctx, so101, drop, drop_pre, home)


def _grasp_marker(p: tuple[float, float, float], fam: servo.GraspFamily) -> TaskMarker:
    """파지 마커 — 위치 + 가족 방향(approach/jaw_axis/quat) 동봉.

    계획 확정·servo 채택 갱신이 같은 구성을 쓴다 — "어느 면을 어느 방향으로
    무는지"의 시각화 소스는 항상 servo.GraspFamily (contract.TaskMarker)."""
    return TaskMarker(
        label="grasp",
        position=p,
        approach=fam.approach,
        jaw_axis=fam.jaw_axis,
        quaternion=fam.quat,
    )


def _unit_dir(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float] | None:
    """a→b 단위벡터 — 퇴화(0 길이)면 None (마커는 방향 표시 생략)."""
    d = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    n = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    if n < 1e-9:
        return None
    return (d[0] / n, d[1] / n, d[2] / n)
