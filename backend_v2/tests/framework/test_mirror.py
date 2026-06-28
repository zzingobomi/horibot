from __future__ import annotations

import asyncio
import os
import subprocess
import sys
import time
from enum import StrEnum
from pathlib import Path

import pytest
from pydantic import BaseModel

from framework.contract.mirror import Mirror, MirrorState, NotReady, discover_mirrors
from framework.contract.publisher import encode_event
from framework.contract.service import service
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


# ─── Contract — nested class + payload ────────────────


class Calib:
    """robot-agnostic Calibration contract (test fixture)."""

    class Service(StrEnum):
        SNAPSHOT_BUNDLE = "srv/calibration/snapshot_bundle"
        ACTIVATE = "srv/calibration/activate"

    class Event(StrEnum):
        ACTIVATED = "event/calibration/activated"


class CalibRobot:
    """robot-scoped Calibration contract (test fixture)."""

    class Service(StrEnum):
        SNAPSHOT_BUNDLE = "srv/calibration/{robot_id}/snapshot_bundle"

    class Event(StrEnum):
        ACTIVATED = "event/calibration/{robot_id}/activated"


class SnapshotRequest(BaseModel):
    robot_id: str | None = None


class CalibrationBundle(BaseModel):
    bundle_id: int
    hand_eye: list[float]
    joint_offsets: list[float]


class CalibrationActivated(BaseModel):
    bundle_id: int


class CalibrationActivatedRobot(BaseModel):
    robot_id: str
    bundle_id: int


class ActivateRequest(BaseModel):
    bundle_id: int


class ActivateResponse(BaseModel):
    ok: bool


# ─── MirrorState 단위 — NotReady / value / is_ready ──


def test_mirror_state_not_ready_raises():
    state: MirrorState[CalibrationBundle] = MirrorState(
        spec=Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        ).spec
    )
    assert state.is_ready is False
    with pytest.raises(NotReady):
        _ = state.value


def test_mirror_state_set_then_value():
    state: MirrorState[CalibrationBundle] = MirrorState(
        spec=Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        ).spec
    )
    bundle = CalibrationBundle(bundle_id=7, hand_eye=[1.0], joint_offsets=[0.1])
    state._set(bundle)
    assert state.is_ready is True
    assert state.value.bundle_id == 7
    assert state.value.hand_eye == [1.0]


# ─── Mirror descriptor — per-instance state ─────


def test_mirror_descriptor_per_instance_state():
    class Reader:
        cal: Mirror[CalibrationBundle] = Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        )

    a = Reader()
    b = Reader()
    # 두 instance 의 state 는 별개 object
    assert a.cal is not b.cal
    # 같은 instance 의 두 access 는 같은 state object (cached)
    assert a.cal is a.cal


def test_discover_mirrors_finds_descriptor():
    class Reader:
        cal: Mirror[CalibrationBundle] = Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        )

    reader = Reader()
    mirrors = discover_mirrors(reader)
    assert len(mirrors) == 1
    name, state = mirrors[0]
    assert name == "cal"
    assert state.spec.snapshot_service == "srv/calibration/snapshot_bundle"


# ─── Runtime 통합 — same-process snapshot + event refetch ──


async def test_mirror_initial_snapshot_fetch(runtime: Runtime):
    """Phase 3a — Owner 떠 있으면 Reader 의 Mirror cache 즉시 채워짐."""

    class Owner:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            self._bundle = CalibrationBundle(
                bundle_id=1, hand_eye=[0.0, 1.0], joint_offsets=[0.01, 0.02],
            )

        @service(Calib.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
            return self._bundle

    class Reader:
        cal: Mirror[CalibrationBundle] = Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        )

        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

    runtime.add_module(Owner)
    reader = runtime.add_module(Reader)
    await runtime.start()

    assert reader.cal.is_ready
    assert reader.cal.value.bundle_id == 1
    assert reader.cal.value.hand_eye == [0.0, 1.0]


async def test_mirror_event_triggers_refetch(runtime: Runtime):
    """Owner 가 새 값으로 변경 + event publish → Reader cache 갱신."""

    class Owner:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            self._bundle = CalibrationBundle(
                bundle_id=1, hand_eye=[0.0], joint_offsets=[0.0],
            )

        @service(Calib.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
            return self._bundle

        @service(Calib.Service.ACTIVATE)
        def activate(self, req: ActivateRequest) -> ActivateResponse:
            self._bundle = CalibrationBundle(
                bundle_id=req.bundle_id,
                hand_eye=[float(req.bundle_id)],
                joint_offsets=[float(req.bundle_id) * 0.1],
            )
            self.runtime.publish(
                Calib.Event.ACTIVATED,
                CalibrationActivated(bundle_id=req.bundle_id),
            )
            return ActivateResponse(ok=True)

    class Reader:
        cal: Mirror[CalibrationBundle] = Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        )

        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

    runtime.add_module(Owner)
    reader = runtime.add_module(Reader)
    await runtime.start()

    assert reader.cal.value.bundle_id == 1

    # Owner activate → event publish → Reader refetch
    await runtime.module_runtime.call(
        Calib.Service.ACTIVATE, ActivateRequest(bundle_id=42), ActivateResponse,
    )

    # event 처리 + refetch 까지 짧은 대기 (callback thread → asyncio loop)
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if reader.cal.value.bundle_id == 42:
            break
        await asyncio.sleep(0.05)

    assert reader.cal.value.bundle_id == 42
    assert reader.cal.value.hand_eye == [42.0]


async def test_mirror_owner_not_up_initially(transport: ZenohTransport):
    """Owner 안 떠 있는 상태에서 Reader start — fail 안 함 (non-blocking).
    snapshot service 없으니 is_ready=False 유지 검증."""

    class Reader:
        cal: Mirror[CalibrationBundle] = Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        )

        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

    rt = Runtime(transport)
    rt.mirror_snapshot_timeout = 0.3  # 짧게
    reader = rt.add_module(Reader)
    await rt.start()  # Owner 없어도 fail X
    try:
        assert reader.cal.is_ready is False
        with pytest.raises(NotReady):
            _ = reader.cal.value
    finally:
        await rt.stop()


# ─── robot-scoped Mirror ─────────────────────────────


async def test_mirror_robot_scoped_snapshot(runtime: Runtime):
    """robot-scoped Mirror — snapshot_req factory 가 self.robot_id 사용."""

    class Owner:
        def __init__(self, runtime: ModuleRuntime, robot_id: str):
            self.runtime = runtime
            self.robot_id = robot_id
            self._bundle = CalibrationBundle(
                bundle_id=int(robot_id[-1]),
                hand_eye=[float(robot_id[-1])],
                joint_offsets=[],
            )

        @service(CalibRobot.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
            return self._bundle

    class Reader:
        cal: Mirror[CalibrationBundle] = Mirror(
            snapshot_service=CalibRobot.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(robot_id=self.robot_id),
            change_topic=CalibRobot.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivatedRobot,
        )

        def __init__(self, runtime: ModuleRuntime, robot_id: str):
            self.runtime = runtime
            self.robot_id = robot_id

    runtime.add_module(Owner, robot_id="r1")
    runtime.add_module(Owner, robot_id="r2")
    reader1 = runtime.add_module(Reader, robot_id="r1")
    reader2 = runtime.add_module(Reader, robot_id="r2")
    await runtime.start()

    # 각 Reader 가 자기 robot 의 Owner 에서 snapshot 가져옴
    assert reader1.cal.value.bundle_id == 1
    assert reader2.cal.value.bundle_id == 2


async def test_mirror_robot_scoped_event_filter(runtime: Runtime):
    """robot-scoped Mirror — 다른 robot 의 event 무시."""

    class Owner:
        def __init__(self, runtime: ModuleRuntime, robot_id: str):
            self.runtime = runtime
            self.robot_id = robot_id
            self._bundle = CalibrationBundle(
                bundle_id=1, hand_eye=[1.0], joint_offsets=[],
            )

        @service(CalibRobot.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
            return self._bundle

        def update(self, new_id: int) -> None:
            self._bundle = CalibrationBundle(
                bundle_id=new_id,
                hand_eye=[float(new_id)],
                joint_offsets=[],
            )
            self.runtime.publish(
                CalibRobot.Event.ACTIVATED,
                CalibrationActivatedRobot(robot_id=self.robot_id, bundle_id=new_id),
            )

    class Reader:
        cal: Mirror[CalibrationBundle] = Mirror(
            snapshot_service=CalibRobot.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(robot_id=self.robot_id),
            change_topic=CalibRobot.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivatedRobot,
        )

        def __init__(self, runtime: ModuleRuntime, robot_id: str):
            self.runtime = runtime
            self.robot_id = robot_id

    owner1 = runtime.add_module(Owner, robot_id="r1")
    owner2 = runtime.add_module(Owner, robot_id="r2")
    reader1 = runtime.add_module(Reader, robot_id="r1")
    reader2 = runtime.add_module(Reader, robot_id="r2")
    await runtime.start()

    assert reader1.cal.value.bundle_id == 1
    assert reader2.cal.value.bundle_id == 1

    # owner2 만 update — reader2 만 변해야 함
    owner2.update(99)

    deadline = time.time() + 2.0
    while time.time() < deadline:
        if reader2.cal.value.bundle_id == 99:
            break
        await asyncio.sleep(0.05)

    assert reader2.cal.value.bundle_id == 99
    assert reader1.cal.value.bundle_id == 1  # filter 박힘 — 변 X
    _ = owner1  # unused warning 회피


# ─── Mirror invariant — race / thread safety ────────


async def test_mirror_event_during_init_not_lost(runtime: Runtime):
    """M5.1 / M5.4 — INITIALIZING 중 받은 event 가 snapshot 후 fresh refetch trigger.

    spec §3.3.1 의 result invariant — Mirror 는 READY 이전 event 를 잃지 않음.
    구현 = subscribe-before-snapshot + event 받으면 무조건 refetch (callback thread
    → asyncio loop schedule).

    race window:
      T=0    Reader Runtime.start() 시작 (Phase 2 subscribe register + Phase 3a snapshot fetch)
      T=0    Owner snapshot handler 가 0.5s sleep — race window
      T=0.2  Owner update + publish event (bundle_id=2)
             → subscriber callback 이 refetch task schedule
      T=0.5  첫 snapshot fetch 완료 → cache = bundle_id=1 (stale)
      T=0.5+ scheduled refetch 실행 → cache = bundle_id=2 (fresh)

    검증: 결국 cache.bundle_id == 2 (READY 이전 박힌 event 가 안 잃힘).
    """
    snapshot_delay = 0.5  # snapshot fetch race window

    class Owner:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            self._bundle = CalibrationBundle(
                bundle_id=1, hand_eye=[1.0], joint_offsets=[],
            )

        @service(Calib.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
            time.sleep(snapshot_delay)  # 별도 thread (asyncio.to_thread) — loop block X
            return self._bundle

        def update_and_publish(self, new_id: int) -> None:
            self._bundle = CalibrationBundle(
                bundle_id=new_id, hand_eye=[float(new_id)], joint_offsets=[],
            )
            self.runtime.publish(
                Calib.Event.ACTIVATED,
                CalibrationActivated(bundle_id=new_id),
            )

    class Reader:
        cal: Mirror[CalibrationBundle] = Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        )

        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

    owner = runtime.add_module(Owner)
    reader = runtime.add_module(Reader)

    async def emit_during_init():
        # snapshot fetch 가 진행 중인 시점 — race window 안에서 event 박음
        await asyncio.sleep(0.2)
        owner.update_and_publish(2)

    emit_task = asyncio.create_task(emit_during_init())
    await runtime.start()
    await emit_task

    # snapshot delay + refetch 까지 충분히 대기
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if reader.cal.is_ready and reader.cal.value.bundle_id == 2:
            break
        await asyncio.sleep(0.05)

    assert reader.cal.is_ready, "Mirror 가 ready 안 됨"
    assert reader.cal.value.bundle_id == 2, (
        f"INITIALIZING 중 박힌 event 가 잃힘 — cache = {reader.cal.value.bundle_id}"
    )


def test_mirror_concurrent_read_write_no_partial_state():
    """M6.1 — 동시 read/write 자체 partial state 안 보임 (RLock 검증).

    spec §3.3.2 — Mirror update 가 event callback thread, .value access 가 다른
    thread. 두 access 사이 race 가 partial state (cache 만 새값 / initialized 만
    옛값) 보이면 안 됨. 현재 구현 = RLock.

    검증: 한 thread = writer (1000 회 _set), 다른 thread = reader (1000 회 .value).
    reader 가 본 value 의 invariant — `bundle_id` 와 `hand_eye[0]` 가 일치 (한
    bundle 안 두 field 는 같은 write 의 결과). partial 박히면 두 값 mismatch.
    """
    import threading as _threading

    state: MirrorState[CalibrationBundle] = MirrorState(
        spec=Mirror(
            snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
            snapshot_req=lambda self: SnapshotRequest(),
            change_topic=Calib.Event.ACTIVATED,
            value_cls=CalibrationBundle,
            change_event_cls=CalibrationActivated,
        ).spec
    )
    initial = CalibrationBundle(bundle_id=0, hand_eye=[0.0], joint_offsets=[])
    state._set(initial)

    iterations = 1000
    errors: list[str] = []
    stop = _threading.Event()

    def writer():
        for i in range(1, iterations + 1):
            if stop.is_set():
                return
            bundle = CalibrationBundle(
                bundle_id=i,
                hand_eye=[float(i)],
                joint_offsets=[],
            )
            state._set(bundle)

    def reader():
        for _ in range(iterations):
            if stop.is_set():
                return
            try:
                v = state.value
            except NotReady as e:
                errors.append(f"NotReady (writer 가 init 이미 박았는데): {e}")
                stop.set()
                return
            # invariant — 한 bundle 안 bundle_id 와 hand_eye[0] 는 같은 write 결과
            if v.hand_eye[0] != float(v.bundle_id):
                errors.append(
                    f"partial state — bundle_id={v.bundle_id}, "
                    f"hand_eye[0]={v.hand_eye[0]}"
                )
                stop.set()
                return

    t1 = _threading.Thread(target=writer)
    t2 = _threading.Thread(target=reader)
    t1.start()
    t2.start()
    t1.join(timeout=10.0)
    t2.join(timeout=10.0)

    assert not errors, f"thread race 에서 partial state 발견: {errors[:3]}"
    # 두 thread 다 정상 종료
    assert not t1.is_alive() and not t2.is_alive()


# ─── cross-process ──────────────────────────────────


_OWNER_SCRIPT = """\
import asyncio
import os
import sys
import time
from enum import StrEnum
from pathlib import Path

sys.path.insert(0, os.environ["BACKEND_V2_PATH"])

from pydantic import BaseModel

from framework.contract.service import service
from framework.runtime.app import Runtime
from framework.runtime.api import ModuleRuntime
from infra.transport.zenoh import ZenohTransport


class Calib:
    class Service(StrEnum):
        SNAPSHOT_BUNDLE = "srv/calibration/snapshot_bundle"

    class Event(StrEnum):
        ACTIVATED = "event/calibration/activated"


class SnapshotRequest(BaseModel):
    robot_id: str | None = None


class CalibrationBundle(BaseModel):
    bundle_id: int
    hand_eye: list[float]
    joint_offsets: list[float]


class CalibrationActivated(BaseModel):
    bundle_id: int


class Owner:
    def __init__(self, runtime: ModuleRuntime):
        self.runtime = runtime
        self._bundle = CalibrationBundle(bundle_id=7, hand_eye=[7.0], joint_offsets=[])
        self._update_count = 0

    @service(Calib.Service.SNAPSHOT_BUNDLE)
    def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
        return self._bundle

    def start(self):
        # background — 1초 후 새 값 publish (Reader 가 받는지 검증용)
        asyncio.create_task(self._update_later())

    async def _update_later(self):
        await asyncio.sleep(1.0)
        self._bundle = CalibrationBundle(
            bundle_id=42, hand_eye=[42.0], joint_offsets=[]
        )
        self.runtime.publish(
            Calib.Event.ACTIVATED, CalibrationActivated(bundle_id=42)
        )


async def main():
    cfg = {
        "mode": "peer",
        "scouting": {"multicast": {"enabled": False}},
        "connect": [os.environ["ZENOH_ENDPOINT"]],
    }
    transport = ZenohTransport(cfg)
    rt = Runtime(transport)
    rt.add_module(Owner)
    await rt.start()

    deadline = time.time() + 8.0
    ready = Path(os.environ["READY_FILE"])
    ready.write_text("ok")
    while time.time() < deadline:
        if Path(os.environ["DONE_FILE"]).exists():
            break
        await asyncio.sleep(0.1)
    await rt.stop()


asyncio.run(main())
"""


def test_mirror_cross_process(tmp_path: Path):
    """Reader (parent process) ↔ Owner (subprocess) — Zenoh between-session."""
    endpoint = "tcp/127.0.0.1:17448"

    ready_file = tmp_path / "ready"
    done_file = tmp_path / "done"
    script_path = tmp_path / "owner.py"
    script_path.write_text(_OWNER_SCRIPT, encoding="utf-8")

    backend_v2_path = str(Path(__file__).resolve().parents[2])
    venv_python = Path(backend_v2_path) / ".venv" / "Scripts" / "python.exe"
    if not venv_python.is_file():
        venv_python = Path(backend_v2_path) / ".venv" / "bin" / "python"
    python_exe = str(venv_python) if venv_python.is_file() else sys.executable

    env = os.environ.copy()
    env["BACKEND_V2_PATH"] = backend_v2_path
    env["ZENOH_ENDPOINT"] = endpoint
    env["READY_FILE"] = str(ready_file)
    env["DONE_FILE"] = str(done_file)
    env["PYTHONIOENCODING"] = "utf-8"

    # parent transport 먼저 — subprocess 가 connect 할 endpoint
    parent_cfg = {
        "mode": "peer",
        "scouting": {"multicast": {"enabled": False}},
        "listen": [endpoint],
    }
    parent_transport = ZenohTransport(parent_cfg)
    time.sleep(0.5)  # listen 안정

    proc = subprocess.Popen(
        [python_exe, str(script_path)],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    try:
        # Owner subprocess ready 대기
        deadline = time.time() + 10.0
        while time.time() < deadline:
            if ready_file.exists():
                break
            time.sleep(0.1)
        assert ready_file.exists(), "Owner subprocess 안 떠옴"

        # Zenoh discovery + service register 가시화 안정
        time.sleep(1.0)

        class Reader:
            cal: Mirror[CalibrationBundle] = Mirror(
                snapshot_service=Calib.Service.SNAPSHOT_BUNDLE,
                snapshot_req=lambda self: SnapshotRequest(),
                change_topic=Calib.Event.ACTIVATED,
                value_cls=CalibrationBundle,
                change_event_cls=CalibrationActivated,
            )

            def __init__(self, runtime: ModuleRuntime):
                self.runtime = runtime

        async def reader_flow():
            rt = Runtime(parent_transport)
            rt.mirror_snapshot_timeout = 5.0  # cross-process 충분 timeout
            reader = rt.add_module(Reader)
            await rt.start()
            try:
                assert reader.cal.is_ready, "initial snapshot fetch 실패"
                # initial bundle_id == 7 (또는 owner update 가 빨라서 이미 42)
                assert reader.cal.value.bundle_id in (7, 42)

                # Owner 가 1초 후 update — 받을 때까지 대기
                deadline2 = time.time() + 5.0
                while time.time() < deadline2:
                    if reader.cal.value.bundle_id == 42:
                        break
                    await asyncio.sleep(0.1)
                assert reader.cal.value.bundle_id == 42
            finally:
                await rt.stop()

        asyncio.run(reader_flow())
        done_file.write_text("ok")

        proc.wait(timeout=10)
    finally:
        parent_transport.close()
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)
        # encode_event silence unused warning
        _ = encode_event
