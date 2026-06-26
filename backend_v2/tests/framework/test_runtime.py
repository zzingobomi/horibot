"""tests/framework/test_runtime.py — Step 3 검증 (§11 Step 3).

검증 자세 (§11 Step 3):
1. 빈 Module + ZenohTransport runtime start → stop 정상
2. 두 Module + service call 정상 (method reference round-trip)
3. Module A publish → Module B @subscriber callback 도달
4. Module A 의 start() 가 Module B 의 service 호출 — register 가 먼저 끝났는지 검증
5. robot-scoped service register 시 self.robot_id 자세 substitute
6. caller 의 robot_id= 자세 substitute round-trip
"""

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


# ─── 1. 빈 Module + runtime start/stop ──────────────────────


async def test_empty_runtime_start_stop(transport: ZenohTransport):
    rt = Runtime(transport)
    await rt.start()
    await rt.stop()


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
                EchoModule.echo, EchoRequest(message="hi")
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
            self.runtime.publish(GreetEventTopic.GREETED, GreetEvent(name=name))

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
    """phase 2 (register) 자세 phase 3 (start) 이전 완료 박힘 검증."""

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
            # phase 3 시점 — TargetModule.double 자세 register 박혀있음
            self.result = await self.runtime.call(
                TargetModule.double, DoubleRequest(value=21)
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

    # caller — robot_id= 자세 substitute
    res_omx = await runtime.module_runtime.call(
        MotorModule.move_l, MoveLRequest(x=1.0), robot_id="omx_f_0"
    )
    res_so = await runtime.module_runtime.call(
        MotorModule.move_l, MoveLRequest(x=2.0), robot_id="so101_0"
    )
    assert isinstance(res_omx, MoveLResponse) and res_omx.ok
    assert isinstance(res_so, MoveLResponse) and res_so.ok
    # 각 robot 인스턴스 자세 dispatch 박힘
    assert omx.moved_to == 1.0
    assert so101.moved_to == 2.0


# ─── 6. caller 자세 robot_id 안 박으면 fail ──────────────────


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
            MotorModule.move_l, MoveLRequest(x=1.0)
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

        @subscriber(JoinedEventTopic.JOINED)            # wildcard substitute (transport detail)
        def on_joined(self, event: JoinedEvent) -> None:
            received.append(event)
            done.set()

    publisher = runtime.add_module(PublisherMod)
    runtime.add_module(SubscriberMod)
    await runtime.start()

    publisher.emit(robot_id="omx_f_0", name="alice")
    assert done.wait(timeout=2.0)
    assert received == [JoinedEvent(robot_id="omx_f_0", name="alice")]


# ─── 8. Module 자세 robot_id 없는데 robot-scoped @service 박으면 fail ──


async def test_robot_scoped_service_without_robot_id_module_fails(
    transport: ZenohTransport,
):
    class BadModule:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            # robot_id 자세 안 박힘

        @service(MotionServiceKey.MOVE_L)
        def move_l(self, req: MoveLRequest) -> MoveLResponse:
            return MoveLResponse(ok=True)

    rt = Runtime(transport)
    rt.add_module(BadModule)
    with pytest.raises(ValueError, match="robot_id"):
        await rt.start()
    await rt.stop()


# ─── 9. add_module 자세 missing dep — TypeError ────────────


async def test_add_module_missing_dep_raises(transport: ZenohTransport):
    class NeedsDep:
        def __init__(self, runtime: ModuleRuntime, repo: object):
            self.runtime = runtime
            self.repo = repo

    rt = Runtime(transport)
    with pytest.raises(TypeError, match="repo"):
        rt.add_module(NeedsDep)
    await rt.stop()


# ─── 10. add_module 자세 dep inject — repo 자세 ────────────


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


# ─── 11. Module 자세 sync start/stop 자세 ─────────────────


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


# ─── 12. async start/stop 자세 ────────────────────────────


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
