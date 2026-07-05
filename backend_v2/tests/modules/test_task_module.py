"""TaskModule runner + 디버거 단위테스트 (§17.4 "runner+디버거 e2e 부터").

fake runtime 으로 publish 캡처. 도메인 step 0개 trivial task (Wait+NoOp) — 실
하드웨어/motion 없이 runner 순차 실행 + step_result + 디버거(breakpoint/pause/resume)
검증. 의미(뒤집으면 회귀): SUCCESS 미도달 / breakpoint 무시 / tree 미발행 /
미등록 task accept.
"""

from __future__ import annotations

import asyncio
from typing import cast

from pydantic import BaseModel

from modules.task.contract import RunRequest, TaskState, TaskStatus
from modules.task.module import TaskModule
from modules.task.runner import TaskRunner
from modules.task.tasks import build_task

_ROBOT = "so101_6dof_0"


class _FakeRuntime:
    """publish 캡처. noop task 는 서비스 call 안 함 (도메인 step 0개)."""

    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):  # noqa: ANN001, ANN201
        raise AssertionError("noop task 는 서비스 call 안 함")


def _states(rt: _FakeRuntime) -> list[TaskState]:
    return [cast(TaskState, e) for k, e in rt.published if k.endswith("/state")]


def _last_state(rt: _FakeRuntime) -> TaskState | None:
    s = _states(rt)
    return s[-1] if s else None


async def test_runner_runs_trivial_task_to_success():
    rt = _FakeRuntime()
    runner = TaskRunner(rt, _ROBOT)
    spec = build_task("noop", {})
    assert runner.run(spec) is True
    assert runner._handle is not None
    await runner._handle

    final = _last_state(rt)
    assert final is not None and final.status == TaskStatus.SUCCESS
    # 모든 step COMPLETED
    assert final.step_statuses and all(
        v == "completed" for v in final.step_statuses.values()
    )
    # step 마다 step_result 1개
    results = [e for k, e in rt.published if k.endswith("/step_result")]
    assert len(results) == len(spec.steps)


async def test_runner_breakpoint_pauses_then_resumes():
    rt = _FakeRuntime()
    runner = TaskRunner(rt, _ROBOT)
    spec = build_task("noop", {})
    bp = spec.steps[1]  # noop_b
    assert runner.toggle_breakpoint(bp.id) is True
    runner.run(spec)

    # PAUSED 도달까지 event loop 양보하며 poll (breakpoint 앞에서 멈춤).
    for _ in range(500):
        await asyncio.sleep(0)
        st = _last_state(rt)
        if st is not None and st.status == TaskStatus.PAUSED:
            break

    st = _last_state(rt)
    assert st is not None and st.status == TaskStatus.PAUSED
    assert st.current_step_id == bp.id
    assert st.step_statuses[spec.steps[0].id] == "completed"  # 앞 step 은 끝
    assert st.step_statuses[bp.id] == "pending"  # breakpoint step 은 아직

    assert runner.resume() is True
    assert runner._handle is not None
    await runner._handle
    end = _last_state(rt)
    assert end is not None and end.status == TaskStatus.SUCCESS


async def test_module_run_publishes_tree_and_runs():
    rt = _FakeRuntime()
    mod = TaskModule(rt, {})
    res = await mod.run(RunRequest(robot_id=_ROBOT, task_name="noop"))
    assert res.accepted

    trees = [e for k, e in rt.published if k.endswith("/tree")]
    assert len(trees) == 1
    tree = trees[0]
    assert tree.task_name == "noop"  # type: ignore[attr-defined]
    assert len(tree.steps) == 3  # type: ignore[attr-defined]

    runner = mod._runners[_ROBOT]
    assert runner._handle is not None
    await runner._handle
    assert (s := _last_state(rt)) is not None and s.status == TaskStatus.SUCCESS


async def test_module_run_unknown_task_rejected():
    rt = _FakeRuntime()
    mod = TaskModule(rt, {})
    res = await mod.run(RunRequest(robot_id=_ROBOT, task_name="nope"))
    assert not res.accepted
    assert "nope" in res.message
