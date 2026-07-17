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
    """robot-scoped Mirror — 다른 robot 의 event 무시.

    값 비교만으론 vacuous (owner1 값이 안 변해 refetch 가 새도 1 이 나옴 —
    2026-07-17 발견). 그래서 snapshot 호출 수 스파이로 "타 robot event 에
    refetch 자체가 없다" 를 잠근다."""

    class Owner:
        def __init__(self, runtime: ModuleRuntime, robot_id: str):
            self.runtime = runtime
            self.robot_id = robot_id
            self.snapshot_calls = 0
            self._bundle = CalibrationBundle(
                bundle_id=1, hand_eye=[1.0], joint_offsets=[],
            )

        @service(CalibRobot.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
            self.snapshot_calls += 1
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
    owner1_boot_calls = owner1.snapshot_calls  # reader1 초기 snapshot 만

    # owner2 만 update — reader2 만 변해야 함
    owner2.update(99)

    deadline = time.time() + 2.0
    while time.time() < deadline:
        if reader2.cal.value.bundle_id == 99:
            break
        await asyncio.sleep(0.05)

    assert reader2.cal.value.bundle_id == 99
    assert reader1.cal.value.bundle_id == 1
    # 필터의 실체 — r2 activation 이 reader1 의 refetch 를 유발하지 않는다
    assert owner1.snapshot_calls == owner1_boot_calls, (
        "타 robot event 에 reader1 이 refetch — robot-scoped 필터 붕괴"
    )


# ─── Mirror invariant — race / thread safety ────────


@pytest.mark.sim  # race window 재현에 실 대기 0.5s+α — fast loop 제외
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


# M6.1 동시 read/write partial-state 테스트는 삭제 (2026-07-17 정리) — 현 구현이
# 참조 스왑(_set = 원자적 교체)이라 CPython GIL 아래서 락을 빼도 못 깨지는
# vacuous 검증이었고 (2×1000 thread, join 10s) 비용만 컸다. in-place 변형으로
# 리팩토링하려면 그때 동시성 테스트를 실검출력 있게 재설계할 것.


# ─── cross-process ──────────────────────────────────


_OWNER_SCRIPT = """\
import asyncio
import os
import sys
import time
from enum import StrEnum
from pathlib import Path

sys.path.insert(0, os.environ["BACKEND_PATH"])

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


@pytest.mark.sim  # subprocess 인터프리터 스폰 + 실 tcp (~수 초) — fast loop 제외
def test_mirror_cross_process(tmp_path: Path):
    """Reader (parent process) ↔ Owner (subprocess) — Zenoh between-session."""
    endpoint = "tcp/127.0.0.1:17448"

    ready_file = tmp_path / "ready"
    done_file = tmp_path / "done"
    script_path = tmp_path / "owner.py"
    script_path.write_text(_OWNER_SCRIPT, encoding="utf-8")

    backend_path = str(Path(__file__).resolve().parents[2])
    venv_python = Path(backend_path) / ".venv" / "Scripts" / "python.exe"
    if not venv_python.is_file():
        venv_python = Path(backend_path) / ".venv" / "bin" / "python"
    python_exe = str(venv_python) if venv_python.is_file() else sys.executable

    env = os.environ.copy()
    env["BACKEND_PATH"] = backend_path
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


# ─── liveliness 수렴 — 부팅 순서 해방 (2026-07-07 근본 수정) ─────


@pytest.mark.sim  # 2세션 tcp + 수렴 대기 (~수 초) — fast loop 제외 (full 은 필수)
async def test_mirror_converges_when_owner_boots_later():
    """THE 회귀 테스트 — Reader 먼저 부팅 (owner 없음, NotReady) → Owner 가
    **나중에** 부팅 → change event 하나 없이 liveliness alive 만으로 mirror 수렴.

    이 테스트가 red 면 "PC 늦게 켜면 motion 이 무보정으로 영원히 돈다" 재발.
    """
    ep = "tcp/127.0.0.1:17562"
    t_reader = ZenohTransport({**_LOCAL_CFG, "listen": [ep]})
    t_owner: ZenohTransport | None = None
    rt_owner: Runtime | None = None

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

    class Owner:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            self._bundle = CalibrationBundle(
                bundle_id=7, hand_eye=[7.0], joint_offsets=[0.7]
            )

        @service(Calib.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
            return self._bundle

    rt_reader = Runtime(t_reader)
    rt_reader.mirror_snapshot_timeout = 0.3  # 초기 fetch 빠른 실패
    reader = rt_reader.add_module(Reader)
    try:
        await rt_reader.start()
        assert reader.cal.is_ready is False, "owner 없는데 ready — 테스트 전제 붕괴"

        # Owner 늦은 부팅 (별도 zenoh 세션 = 분산 프로세스 등가)
        t_owner = ZenohTransport({**_LOCAL_CFG, "connect": [ep]})
        rt_owner = Runtime(t_owner)
        rt_owner.add_module(Owner)
        await rt_owner.start()

        # event publish 없음 — liveliness 만으로 수렴해야 함
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if reader.cal.is_ready:
                break
            await asyncio.sleep(0.05)

        assert reader.cal.is_ready, "owner 늦은 부팅 후 mirror 미수렴 (liveliness 경로 죽음)"
        assert reader.cal.value.bundle_id == 7
    finally:
        await rt_reader.stop()
        if rt_owner is not None:
            await rt_owner.stop()
        t_reader.close()
        if t_owner is not None:
            t_owner.close()


async def test_mirror_on_change_transition_contract(runtime: Runtime):
    """on_change 발화 계약 3종:
    ① 최초 도착 → (None, v) 1회
    ② 값 변경 → (v, v') 1회
    ③ 같은 값 refetch (event 만 발행, 값 무변) → 발화 X
    """

    class Owner:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime
            self._bundle = CalibrationBundle(
                bundle_id=1, hand_eye=[1.0], joint_offsets=[]
            )

        @service(Calib.Service.SNAPSHOT_BUNDLE)
        def snapshot(self, req: SnapshotRequest) -> CalibrationBundle:
            return self._bundle

        def change(self, new_id: int) -> None:
            self._bundle = CalibrationBundle(
                bundle_id=new_id, hand_eye=[float(new_id)], joint_offsets=[]
            )
            self.runtime.publish(
                Calib.Event.ACTIVATED, CalibrationActivated(bundle_id=new_id)
            )

        def touch(self) -> None:
            # 값 안 바꾸고 event 만 — refetch 는 일어나되 on_change 는 X
            self.runtime.publish(
                Calib.Event.ACTIVATED,
                CalibrationActivated(bundle_id=self._bundle.bundle_id),
            )

    calls: list[tuple[int | None, int]] = []

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

        @cal.on_change
        async def _on_cal_change(
            self, old: CalibrationBundle | None, new: CalibrationBundle
        ) -> None:
            calls.append((old.bundle_id if old else None, new.bundle_id))

    owner = runtime.add_module(Owner)
    runtime.add_module(Reader)
    await runtime.start()

    # ① 최초 도착 (initial snapshot)
    assert calls == [(None, 1)], f"최초 도착 발화 어긋남: {calls}"

    # ② 값 변경
    owner.change(42)
    deadline = time.time() + 2.0
    while time.time() < deadline and (None, 1) == calls[-1]:
        await asyncio.sleep(0.05)
    deadline = time.time() + 2.0
    while time.time() < deadline and len(calls) < 2:
        await asyncio.sleep(0.05)
    assert calls == [(None, 1), (1, 42)], f"값 변경 발화 어긋남: {calls}"

    # ③ 같은 값 refetch — event 만 발행 → 발화 X
    owner.touch()
    await asyncio.sleep(0.5)  # refetch 여유
    assert calls == [(None, 1), (1, 42)], f"동일값 refetch 가 발화함: {calls}"
