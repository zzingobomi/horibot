from __future__ import annotations

import asyncio
import threading
import time
from enum import StrEnum

import pytest
from pydantic import BaseModel

from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport


_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}


@pytest.fixture
def transport():
    t = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    yield t
    t.close()


@pytest.fixture
async def runtime(transport: ZenohTransport):
    rt = Runtime(transport)
    yield rt
    await rt.stop()


# ─── stop() 격리 — 한 모듈 실패가 나머지 shutdown 막지 않음 ──


async def test_stop_isolates_module_failures(transport: ZenohTransport):
    flag = {"stopped": False}

    class _FlagStop:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        def stop(self) -> None:
            flag["stopped"] = True

    class _BoomStop:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        def stop(self) -> None:
            raise RuntimeError("의도된 stop 실패")

    rt = Runtime(transport)
    # add 순서 [Flag, Boom] → stop 은 reversed → Boom 먼저 throw, Flag 가 그 뒤에도 실행돼야
    rt.add_module(_FlagStop)
    rt.add_module(_BoomStop)
    await rt.start()
    # Boom.stop 이 throw 해도 stop() 자체는 raise 안 하고 Flag.stop 까지 도달
    await rt.stop()
    assert flag["stopped"], "앞 모듈 stop 실패가 뒤 모듈 stop 을 막음 (격리 실패)"


# ─── start() 중간 실패 — 이미 start 된 모듈 rollback ─────────


async def test_start_failure_stops_already_started_modules(
    transport: ZenohTransport,
):
    """boot 중간 실패 시 이미 start 된 모듈의 stop 이 불려야 함.

    방치하면 앞 모듈의 worker thread/task 가 좀비로 남아 프로세스 종료를 막는다
    (실 사고 2026-07-07: 유령 backend 의 port 점유 → bridge start 실패 →
    rollback 없이 방치 → pytest 프로세스 hang)."""
    events: list[str] = []

    class _OkModule:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        def start(self) -> None:
            events.append("ok:start")

        def stop(self) -> None:
            events.append("ok:stop")

    class _BoomStart:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        def start(self) -> None:
            raise RuntimeError("의도된 start 실패")

        def stop(self) -> None:
            # start 못 한 모듈의 stop 은 불리면 안 됨
            events.append("boom:stop")

    rt = Runtime(transport)
    rt.add_module(_OkModule)
    rt.add_module(_BoomStart)
    with pytest.raises(RuntimeError, match="의도된 start 실패"):
        await rt.start()
    assert events == ["ok:start", "ok:stop"], (
        f"start 실패 rollback 이 이미 start 된 모듈만 역순 stop 해야 함: {events}"
    )
    # rollback 후 재시작 가능해야 (started flag 원복)
    await rt.stop()  # no-op (_started guard)


# ─── Test fixtures — wire keys + domain class ────────────────


class EchoServiceKey(StrEnum):
    ECHO = "srv/runtime_test/echo"
    DOUBLE = "srv/runtime_test/double"


class MotionServiceKey(StrEnum):
    MOVE_L = "srv/motion/{robot_id}/move_l"


class GreetEventTopic(StrEnum):
    GREETED = "event/runtime_test/greeted"


class JoinedEventTopic(StrEnum):
    JOINED = "event/runtime_test/{robot_id}/joined"


class EchoRequest(BaseModel):
    message: str


class EchoResponse(BaseModel):
    echoed: str


class DoubleRequest(BaseModel):
    value: int


class DoubleResponse(BaseModel):
    doubled: int


class MoveLRequest(BaseModel):
    x: float


class MoveLResponse(BaseModel):
    ok: bool


class GreetEvent(BaseModel):
    name: str


class JoinedEvent(BaseModel):
    robot_id: str
    name: str


# ─── 2. 두 Module + service call ────────────────────────────


async def test_two_modules_service_call(runtime: Runtime):
    class EchoModule:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @service(EchoServiceKey.ECHO)
        def echo(self, req: EchoRequest) -> EchoResponse:
            return EchoResponse(echoed=f"got:{req.message}")

    class CallerModule:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            self.result: EchoResponse | None = None

        async def do_call(self):
            self.result = await self.runtime.call(
                EchoServiceKey.ECHO, EchoRequest(message="hi"), EchoResponse
            )

    echo = runtime.add_module(EchoModule)
    caller = runtime.add_module(CallerModule)
    await runtime.start()

    await caller.do_call()
    assert isinstance(caller.result, EchoResponse)
    assert caller.result.echoed == "got:hi"
    _ = echo  # avoid unused warning


# ─── 3. publish → @subscriber callback 도달 ─────────────────


async def test_publish_subscriber_round_trip(runtime: Runtime):
    received: list[GreetEvent] = []
    done = threading.Event()

    class PublisherMod:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        def emit(self, name: str):
            self.runtime.publish(GreetEventTopic.GREETED,
                                 GreetEvent(name=name))

    class SubscriberMod:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(GreetEventTopic.GREETED)
        def on_greet(self, event: GreetEvent) -> None:
            received.append(event)
            done.set()

    publisher = runtime.add_module(PublisherMod)
    _sub = runtime.add_module(SubscriberMod)
    await runtime.start()

    publisher.emit("alice")
    assert done.wait(timeout=2.0)
    assert received == [GreetEvent(name="alice")]
    _ = _sub


# ─── 4. Module A start() 가 Module B service 호출 ──────────


async def test_module_start_calls_other_service(runtime: Runtime):
    """phase 2 (register) 가 phase 3 (start) 이전 완료 박힘 검증."""

    class TargetModule:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            self.called = False

        @service(EchoServiceKey.DOUBLE)
        def double(self, req: DoubleRequest) -> DoubleResponse:
            self.called = True
            return DoubleResponse(doubled=req.value * 2)

    class CallerStartupModule:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            self.result: DoubleResponse | None = None

        async def start(self):
            # phase 3 시점 — DOUBLE service 가 register 박혀있음
            self.result = await self.runtime.call(
                EchoServiceKey.DOUBLE, DoubleRequest(value=21), DoubleResponse
            )

    target = runtime.add_module(TargetModule)
    caller = runtime.add_module(CallerStartupModule)
    await runtime.start()

    assert target.called
    assert isinstance(caller.result, DoubleResponse)
    assert caller.result.doubled == 42


# ─── 5. robot-scoped service register — self.robot_id substitute ──


async def test_robot_scoped_service_substitutes_self_robot_id(
    runtime: Runtime,
):
    class MotorModule:
        def __init__(self, runtime: ModuleRuntime, robot_id: str):
            self.runtime = runtime
            self.robot_id = robot_id
            self.moved_to: float | None = None

        @service(MotionServiceKey.MOVE_L)
        def move_l(self, req: MoveLRequest) -> MoveLResponse:
            self.moved_to = req.x
            return MoveLResponse(ok=True)

    omx = runtime.add_module(MotorModule, robot_id="omx_f_0")
    so101 = runtime.add_module(MotorModule, robot_id="so101_0")
    await runtime.start()

    # caller — robot_id= 로 substitute
    res_omx = await runtime.module_runtime.call(
        MotionServiceKey.MOVE_L, MoveLRequest(x=1.0), MoveLResponse,
        robot_id="omx_f_0",
    )
    res_so = await runtime.module_runtime.call(
        MotionServiceKey.MOVE_L, MoveLRequest(x=2.0), MoveLResponse,
        robot_id="so101_0",
    )
    assert isinstance(res_omx, MoveLResponse) and res_omx.ok
    assert isinstance(res_so, MoveLResponse) and res_so.ok
    # 각 robot 인스턴스 로 dispatch 박힘
    assert omx.moved_to == 1.0
    assert so101.moved_to == 2.0


# ─── 6. caller 가 robot_id 안 박으면 fail ──────────────────


async def test_robot_scoped_call_without_robot_id_raises(runtime: Runtime):
    class MotorModule:
        def __init__(self, runtime: ModuleRuntime, robot_id: str):
            self.runtime = runtime
            self.robot_id = robot_id

        @service(MotionServiceKey.MOVE_L)
        def move_l(self, req: MoveLRequest) -> MoveLResponse:
            return MoveLResponse(ok=True)

    runtime.add_module(MotorModule, robot_id="omx_f_0")
    await runtime.start()

    with pytest.raises(ValueError, match="robot_id"):
        await runtime.module_runtime.call(
            MotionServiceKey.MOVE_L, MoveLRequest(x=1.0), MoveLResponse,
        )


# ─── 7. robot-scoped event publish — event.robot_id substitute ──


async def test_robot_scoped_event_publish_and_subscribe(runtime: Runtime):
    received: list[JoinedEvent] = []
    done = threading.Event()

    class PublisherMod:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        def emit(self, robot_id: str, name: str):
            self.runtime.publish(
                JoinedEventTopic.JOINED,
                JoinedEvent(robot_id=robot_id, name=name),
            )

    class SubscriberMod:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        # wildcard substitute (transport detail)
        @subscriber(JoinedEventTopic.JOINED)
        def on_joined(self, event: JoinedEvent) -> None:
            received.append(event)
            done.set()

    publisher = runtime.add_module(PublisherMod)
    runtime.add_module(SubscriberMod)
    await runtime.start()

    publisher.emit(robot_id="omx_f_0", name="alice")
    assert done.wait(timeout=2.0)
    assert received == [JoinedEvent(robot_id="omx_f_0", name="alice")]


# ─── 8. Module 에 robot_id 없는데 robot-scoped @service 박으면 fail ──


async def test_robot_scoped_service_without_robot_id_module_fails(
    transport: ZenohTransport,
):
    class BadModule:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            # robot_id parameter 안 박힘

        @service(MotionServiceKey.MOVE_L)
        def move_l(self, req: MoveLRequest) -> MoveLResponse:
            return MoveLResponse(ok=True)

    rt = Runtime(transport)
    rt.add_module(BadModule)
    with pytest.raises(ValueError, match="robot_id"):
        await rt.start()
    await rt.stop()


# ─── 9. add_module 시 missing dep → TypeError ────────────


async def test_add_module_missing_dep_raises(transport: ZenohTransport):
    class NeedsDep:
        def __init__(self, runtime: ModuleRuntime, repo: object):
            self.runtime = runtime
            self.repo = repo

    rt = Runtime(transport)
    with pytest.raises(TypeError, match="repo"):
        rt.add_module(NeedsDep)
    await rt.stop()


# ─── 10. add_module 시 user dep (repo) inject ────────────


async def test_add_module_injects_user_deps(transport: ZenohTransport):
    class FakeRepo:
        pass

    class UsesRepo:
        def __init__(self, runtime: ModuleRuntime, repo: FakeRepo):
            self.runtime = runtime
            self.repo = repo

    rt = Runtime(transport)
    repo = FakeRepo()
    mod = rt.add_module(UsesRepo, repo=repo)
    assert mod.repo is repo
    await rt.start()
    await rt.stop()


# ─── 11. Module 의 sync start/stop ─────────────────


async def test_module_sync_start_stop(transport: ZenohTransport):
    log: list[str] = []

    class SyncLifecycle:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        def start(self):
            log.append("start")

        def stop(self):
            log.append("stop")

    rt = Runtime(transport)
    rt.add_module(SyncLifecycle)
    await rt.start()
    await rt.stop()
    assert log == ["start", "stop"]

# ─── 12. Module 의 async start/stop ────────────────────────────


async def test_module_async_start_stop(transport: ZenohTransport):
    log: list[str] = []

    class AsyncLifecycle:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        async def start(self):
            await asyncio.sleep(0)
            log.append("start")

        async def stop(self):
            await asyncio.sleep(0)
            log.append("stop")

    rt = Runtime(transport)
    rt.add_module(AsyncLifecycle)
    await rt.start()
    await rt.stop()
    assert log == ["start", "stop"]


# ─── 13. add_module dep key 오타 fail-fast ─────────────────────


def test_add_module_rejects_unknown_dep_key(transport: ZenohTransport):
    """생성자에 없는 dep key 는 조용히 버리지 않고 TypeError.

    조용히 버리면 그 파라미터에 default 가 생기는 순간 resolve 오타가
    silent 오배선 (default 로 대체) 이 됨 — 부팅 시점 거부가 안전.
    """

    class NeedsDriver:
        def __init__(self, runtime: ModuleRuntime, driver: object):
            self.runtime = runtime
            self.driver = driver

    rt = Runtime(transport)
    with pytest.raises(TypeError, match="drivr"):
        rt.add_module(NeedsDriver, drivr=object())  # 'driver' 오타
