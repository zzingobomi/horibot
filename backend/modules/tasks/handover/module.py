"""HandoverModule — omx(giver)가 **자기 웹캠으로 보고** 봉(8×2×2cm 주황 각봉)의
한쪽 끝을 집어 **수직으로 세워 늘어뜨려** 제시하면, so101(receiver)이
**재검출**해 아래로 늘어진 노출부를 받아 (선택) 상자 적치.

pick_and_place 표준형 복제 (task.md §3). ⚠ 2026-07-27 큐브→봉 전환 — **실물
미검증** (설계 근거/가정/미지수 = block.py + steps.py docstring + probe
scripts/handover_block_probe.py. 2cm 큐브는 도달↔가림 정면충돌로 폐기 —
docs/omx_handover_realtest_handoff.md §T). frontend = /tasks/handover 페이지
(stop_before_receive 토글 포함) 또는 터미널:
    uv run --no-sync python scripts/run_task.py srv/handover/run \
        --param "pick_object=orange block" --param "place_object=white box"
    # omx 집기+제시만 먼저 눈으로 볼 때: --param "stop_before_receive=true"

관측성: run 마다 debug/handover/<ts>/{trace.jsonl, summary.json} — 봉 실물
데이터 0 인 첫 런이 그 데이터만으로 원인분석 가능해야 한다 (§6).
"""

from __future__ import annotations

import asyncio
import logging
import time

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.tasks.core.context import TaskContext, TaskContextFactory
from modules.tasks.core.errors import GraspFailed
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

from . import steps
from .collision import BasePose, CrossRobotChecker
from .contract import (
    Handover,
    ListRobotsRequest,
    ListRobotsResponse,
    RunRequest,
    TaskMarker,
    TaskMarkers,
)
from .frames import robot_to_world
from .publish import MarkerPublisher
from .trace import HandoverTrace

logger = logging.getLogger(__name__)


@publishes(
    (Handover.Stream.STATE, TaskState),
    (Handover.Stream.TRACE, TaskTrace),
    (Handover.Stream.MARKERS, TaskMarkers),
)
class HandoverModule:
    # (receiver, giver) — 순서는 표시용일 뿐, 역할은 시나리오 상수가 SSOT
    TASK_ROBOTS = ("so101_6dof_0", "omx_f_0")

    task = TaskRunner()

    def __init__(
        self,
        runtime: ModuleRuntime,
        robots: dict[str, TaskRobotSpec] | None = None,
        omx_base_pose: BasePose | None = None,
        checker: CrossRobotChecker | None = None,
    ) -> None:
        self.runtime = runtime
        self.contexts = TaskContextFactory(runtime, robots)
        # 크로스캘 base_pose (robots.yaml SSOT — apps/resolve.py 가 투영).
        # None = 배선 누락 — 시나리오 진입 시 fail-fast (침묵 identity 금지:
        # hand_eye identity fallback 사고 전례와 같은 클래스).
        self._omx_base_pose = omx_base_pose
        # cross-robot 충돌 체커 — None 이면 수취/복귀 충돌 게이트 생략 (경고).
        self._checker = checker
        self._seq = {"state": 0, "trace": 0, "markers": 0}

    async def stop(self) -> None:
        self.task.cancel()

    # ─── Services ──

    @service(Handover.Service.RUN)
    async def run(self, req: RunRequest) -> RunResponse:
        r = self.task.start(
            self.scenario,
            ctx=self.contexts.create(),
            robot_ids=list(self.TASK_ROBOTS),
            task_name="handover",
            pick_object=req.pick_object,
            place_object=req.place_object,
            stop_before_receive=req.stop_before_receive,
        )
        return RunResponse(accepted=r.accepted, message=r.message)

    @service(Handover.Service.STOP)
    async def stop_run(self, req: ControlRequest) -> ControlResponse:
        r = self.task.cancel()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(Handover.Service.PAUSE)
    async def pause(self, req: ControlRequest) -> ControlResponse:
        r = self.task.pause()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(Handover.Service.RESUME)
    async def resume(self, req: ControlRequest) -> ControlResponse:
        r = self.task.resume()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(Handover.Service.STEP_ONCE)
    async def step_once(self, req: ControlRequest) -> ControlResponse:
        r = self.task.step_once()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(Handover.Service.RUN_TO)
    async def run_to(self, req: RunToRequest) -> ControlResponse:
        r = self.task.run_to(req.name)
        return ControlResponse(ok=r.ok, message=r.message)

    @service(Handover.Service.TOGGLE_BREAKPOINT)
    async def toggle_breakpoint(self, req: ToggleBreakpointRequest) -> ControlResponse:
        r = self.task.toggle_breakpoint(req.name)
        return ControlResponse(ok=r.ok, message=r.message)

    @service(Handover.Service.LIST_ROBOTS)
    async def list_robots(self, req: ListRobotsRequest) -> ListRobotsResponse:
        return ListRobotsResponse(robot_ids=list(self.TASK_ROBOTS))

    @service(Handover.Service.PREVIEW)
    async def preview(self, req: PreviewRequest) -> PreviewResponse:
        entries = await asyncio.to_thread(build_preview, self.scenario)
        return PreviewResponse(entries=entries)

    # ─── Publishing ────

    @task.on_state
    def _publish_state(self, s: RunState) -> None:
        for robot_id in s.robot_ids or self.TASK_ROBOTS:
            self.runtime.publish(
                Handover.Stream.STATE,
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
                Handover.Stream.TRACE,
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
            Handover.Stream.MARKERS,
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

    # ─── Scenario ─────────

    async def scenario(
        self,
        ctx: TaskContext,
        pick_object: str,
        place_object: str = "",
        stop_before_receive: bool = False,
    ) -> None:
        so101, omx = self.TASK_ROBOTS
        marks = MarkerPublisher(self._publish_markers, giver=omx, receiver=so101)
        if self._omx_base_pose is None:
            raise RuntimeError(
                "omx base_pose 미배선 — robots.yaml base_pose → apps/resolve.py "
                "handover deps 투영 확인 (침묵 identity 금지)"
            )
        base_omx = self._omx_base_pose
        # 로컬 별칭 — self._checker 직접 메서드 호출은 checker=None(mock)일 때
        # preview 정적 해석이 <동적> 구멍으로 표시한다 (지역 객체 호출은 무시 규약)
        checker = self._checker
        if checker is None:
            logger.warning(
                "cross-robot 충돌 체커 미배선 — 제시/수취/복귀 충돌 게이트 생략 "
                "(mock 테스트 전용 상태. 실물 실행 전 배선 필수)"
            )
        else:
            # 이전 run 의 재제시 보정 offset 이 남아 있으면 안 된다 (run 시작
            # 시점엔 오차 미실측 — 모델 원 base_pose 로 리셋)
            checker.set_base_offset_b((0.0, 0.0, 0.0))
        trace = HandoverTrace(pick_object)
        status, error = "success", None
        try:
            # 0) 자산/설정 fail-fast (모션 0 시점) — 티칭은 home 뿐 (제시는 계산)
            home_so = await steps.named_waypoint(
                ctx, so101, "home", "so101 안전 경유 자세를 'home' 으로 저장하세요"
            )
            home_omx = await steps.named_waypoint(
                ctx, omx, "home", "omx 안전 경유 자세를 'home' 으로 저장하세요"
            )
            roi_so, roi_omx = await steps.load_workcells(ctx, so101, omx)
            t_tcp_cam_omx = await steps.load_hand_eye(ctx, omx)
            t_tcp_cam_so = await steps.load_hand_eye(ctx, so101)

            # 1) 시작 자세 — 토크 enable 먼저 (omx Dynamixel 전원 on 시 off 기본,
            # 헤드리스 실행엔 프론트 토크 토글이 없어 limp 로 시작. 2026-07-24 실물)
            await steps.enable_torque(ctx, so101)
            await steps.enable_torque(ctx, omx)
            await steps.go_home(ctx, so101, home_so)
            await steps.go_home(ctx, omx, home_omx)
            await steps.set_gripper(ctx, omx, open_=True)

            # 2~4) A-C. 관측→검출→계획→집기 — 얕은 물림/헛집기(GraspFailed)는
            # run abort 대신 **그리퍼 열고 재관측부터 재시도** (검출 mask 의
            # 축방향 번짐 바이어스가 뷰마다 다르므로 재관측이 재롤. 22:31 실물:
            # 봉 끝 2mm 얕은 물림 → 게이트 fail → 런 전체 죽음은 과잉).
            pick: steps.BlockPick | None = None
            for attempt in range(steps._PICK_RETRIES + 1):
                obs_joints = await steps.plan_omx_observe(
                    ctx, omx, roi_omx, t_tcp_cam_omx, trace
                )
                det = await steps.omx_observe_detect(
                    ctx, omx, pick_object, obs_joints, trace
                )
                grasp = steps.plan_block_grasp_from(det, base_omx)
                pick = await steps.plan_omx_pick_block(ctx, omx, grasp, trace)
                g_world = robot_to_world(pick.grasp_omx, base_omx)
                marks.show_grasp(g_world)
                try:
                    await steps.omx_pick_block(ctx, omx, pick, trace)
                    break
                except GraspFailed:
                    if attempt >= steps._PICK_RETRIES:
                        raise
                    logger.warning(
                        "omx 집기 실패 (%d/%d) — 그리퍼 열고 재관측부터 재시도",
                        attempt + 1,
                        steps._PICK_RETRIES,
                    )
                    await steps.set_gripper(ctx, omx, open_=True)
            assert pick is not None  # 루프는 break(성공) 아니면 raise 로만 탈출

            # 5) D. 제시 — omx 접선 수평 (hang 은 폴백), so101 은 home 에 있음.
            # 수취 결합 게이트가 so101 수취 IK 존재까지 확인하고 채택한다.
            present = await steps.plan_omx_present(
                ctx, omx, so101, roi_so, roi_omx, base_omx, pick,
                list(home_so.joint_values), self._checker, trace,
            )
            await steps.omx_present(ctx, omx, present, trace)
            marks.show_handover(present.h_world)

            # omx 집기+제시까지만 (테스트용) — so101 수취 진입 전 성공 종료.
            # omx 는 큐브를 든 채로 남는다 (파지/제시 자세를 눈으로 검증하도록).
            # ⚠ 사용자 지시(2026-07-26): 실물에서 omx 집기+제시가 어떻게 나오는지
            # 먼저 눈으로 확인해야 하므로 이 게이트를 아직 지우지 않는다.
            if stop_before_receive:
                logger.info("stop_before_receive=True — omx 제시까지만 하고 종료")
                return

            # 6) E. so101 수취 — 재검출 → 재제시 보정 → 계획(충돌 게이트) →
            # refine → 불변식 실행. 그리퍼는 **관측 전에** 연다 — 닫힌 조가
            # eye-in-hand 시야 정중앙(=봉 자리)을 가려 재검출을 조각냈다
            # (2026-07-29 실물 20:43 런, 펜 시절 §2 교훈의 so101 재발).
            await steps.set_gripper(ctx, so101, open_=True)
            so_obs = await steps.plan_so_observe(
                ctx, so101, omx, t_tcp_cam_so, present, pick.geom,
                self._checker, trace,
            )
            det2 = await steps.so_redetect(
                ctx, so101, pick_object, so_obs, present, pick.geom,
                t_tcp_cam_so, trace,
            )
            # 6b) 재제시 보정 루프 (look-then-move) — 실측 이탈이 크면 오차를
            # world_offset 으로 **제시를 전면 재계획** (so101 쪽 게이트를 실물
            # 좌표로 평가 — steps._REPRESENT_* 노브 주석). 같은 자세 단일
            # 평행이동은 omx 가용 밴드 이탈로 IK 전멸 (2026-07-29 21:11 실물).
            # 기준은 항상 현 계획의 h_world (= FK+offset 실물 추정) — 잔차만
            # 남아 수렴한다.
            for _ in range(steps._REPRESENT_MAX):
                off = steps.represent_offset(det2, present.h_world, present.w)
                off_norm = float(
                    (off[0] ** 2 + off[1] ** 2 + off[2] ** 2) ** 0.5
                )
                if off_norm <= steps._REPRESENT_PERP_MIN_M:
                    break
                total = (
                    present.world_offset[0] + off[0],
                    present.world_offset[1] + off[1],
                    present.world_offset[2] + off[2],
                )
                # 충돌/시선/벽 판정의 omx 몸체도 실물 위치로 (margin 실물화)
                if checker is not None:
                    checker.set_base_offset_b(total)
                present = await steps.plan_omx_present(
                    ctx, omx, so101, roi_so, roi_omx, base_omx, pick,
                    list(so_obs.joints), self._checker, trace,
                    world_offset=total,
                )
                await steps.omx_present(ctx, omx, present, trace)
                marks.show_handover(present.h_world)
                so_obs = await steps.plan_so_observe(
                    ctx, so101, omx, t_tcp_cam_so, present, pick.geom,
                    self._checker, trace,
                )
                det2 = await steps.so_redetect(
                    ctx, so101, pick_object, so_obs, present, pick.geom,
                    t_tcp_cam_so, trace,
                )
            else:
                # 상한 소진 — 침묵 금지: 잔차 초과면 남기고 진행 (plan_receive
                # 가 검출 기준으로 겨냥하므로 성립할 수 있고, 죽으면 명시 실패)
                off = steps.represent_offset(det2, present.h_world, present.w)
                residual = float(
                    (off[0] ** 2 + off[1] ** 2 + off[2] ** 2) ** 0.5
                )
                if residual > steps._REPRESENT_PERP_MIN_M:
                    logger.warning(
                        "재제시 보정 %d회 후에도 잔차 %.0fmm — 그대로 수취 시도",
                        steps._REPRESENT_MAX,
                        residual * 1000,
                    )
            plan = await steps.plan_receive(
                ctx, so101, omx, det2, base_omx, present, pick.geom,
                self._checker, trace,
            )
            await steps.receive(
                ctx, so101, omx, plan, pick_object, present, pick.geom, trace
            )
            await steps.omx_retreat(ctx, omx, so101, home_omx, self._checker)

            # 7) 적치 (선택) — 비우면 든 채 home (사용자 인계. 계약 주석)
            if place_object:
                # 봉은 검출 height(mono/공중 과소) 대신 계획 기하로 — 적치
                # 자세(tilt)에 따라 봉이 기울 수 있어 전체 길이를 보수 상한으로
                held_h = pick.geom.length_m
                await steps.place_into(ctx, so101, place_object, held_h, home_so)
            else:
                await steps.go_home(ctx, so101, home_so)
        except BaseException as e:
            status, error = "failed", f"{type(e).__name__}: {e}"
            raise
        finally:
            # 실패해도 반드시 — 첫 실물 런의 진단은 이 파일이 전부다 (§6)
            try:
                await asyncio.to_thread(trace.finish, {
                    "status": status,
                    "error": error,
                    "knobs": steps.knob_snapshot(),
                })
            except Exception:
                logger.exception("handover trace summary 기록 실패")
