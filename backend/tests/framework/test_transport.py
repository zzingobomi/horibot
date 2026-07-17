from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from framework.transport.protocol import RemoteError
from infra.transport.zenoh import ZenohTransport


# multicast 격리 — test 간 LAN 누출 X.
_LOCAL_CFG = {
    "mode": "peer",
    "scouting": {"multicast": {"enabled": False}},
}


# ─── Fixture ────────────────────────────────────────

@pytest.fixture
def transport():
    t = ZenohTransport(_LOCAL_CFG)
    # zenoh internal setup 안정 대기
    time.sleep(0.05)
    yield t
    t.close()


# ─── same-session publish/subscribe ─────────────────

def test_publish_subscribe_same_session(transport: ZenohTransport):
    received: list[bytes] = []
    done = threading.Event()

    def on_message(payload: bytes) -> None:
        received.append(payload)
        done.set()

    handle = transport.subscribe("test/echo", on_message)
    try:
        time.sleep(0.1)  # subscriber register 안정 대기
        transport.publish("test/echo", b"hello")
        assert done.wait(timeout=2.0), "subscriber callback 미수신"
        assert received == [b"hello"]
    finally:
        handle.undeclare()


def test_subscribe_callback_exception_swallowed(transport: ZenohTransport):
    """callback exception 은 transport 가 swallow + log — publisher 영향 0."""
    good_received = threading.Event()
    bad_called = threading.Event()

    def bad_callback(payload: bytes) -> None:
        bad_called.set()
        raise RuntimeError("intentional")

    def good_callback(payload: bytes) -> None:
        good_received.set()

    h1 = transport.subscribe("test/swallow", bad_callback)
    h2 = transport.subscribe("test/swallow", good_callback)
    try:
        time.sleep(0.1)
        transport.publish("test/swallow", b"x")
        assert bad_called.wait(timeout=2.0)
        assert good_received.wait(timeout=2.0), (
            "bad callback raise 가 good callback 막으면 안 됨"
        )
    finally:
        h1.undeclare()
        h2.undeclare()


# ─── same-session register_service + call ───────────

async def test_service_call_same_session(transport: ZenohTransport):
    def echo_handler(req: bytes) -> bytes:
        return b"echo:" + req

    handle = transport.register_service("test/svc/echo", echo_handler)
    try:
        time.sleep(0.1)
        res = await transport.call("test/svc/echo", b"ping", timeout=2.0)
        assert res == b"echo:ping"
    finally:
        handle.undeclare()


async def test_service_handler_exception_propagates(transport: ZenohTransport):
    """handler exception → caller RemoteError(type, message) — spec §3.1."""

    class NotFound(Exception):
        pass

    def handler(req: bytes) -> bytes:
        raise NotFound("result 없음")

    handle = transport.register_service("test/svc/err", handler)
    try:
        time.sleep(0.1)
        with pytest.raises(RemoteError) as ei:
            await transport.call("test/svc/err", b"", timeout=2.0)
        assert ei.value.type_name == "NotFound"
        assert "result 없음" in ei.value.message
    finally:
        handle.undeclare()


async def test_service_call_timeout(transport: ZenohTransport):
    """register 안 된 key → TimeoutError."""
    with pytest.raises(TimeoutError):
        await transport.call("test/svc/nonexistent", b"", timeout=0.3)


# ─── cross-process publish/subscribe ────────────────

_SUBSCRIBER_SCRIPT = """\
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.environ["BACKEND_PATH"])

from infra.transport.zenoh import ZenohTransport

cfg = {
    "mode": "peer",
    "scouting": {"multicast": {"enabled": False}},
    "connect": [os.environ["ZENOH_ENDPOINT"]],
}
t = ZenohTransport(cfg)

out_file = Path(os.environ["OUT_FILE"])

def cb(payload):
    out_file.write_bytes(payload)

h = t.subscribe(os.environ["KEY"], cb)

# parent publish 수신까지 wait — out_file 생성 = 수신 완료
deadline = time.time() + 10.0
while time.time() < deadline:
    if out_file.exists():
        break
    time.sleep(0.05)

h.undeclare()
t.close()
sys.exit(0 if out_file.exists() else 1)
"""


@pytest.mark.sim  # subprocess + 실 tcp 세션 (~수 초) — fast loop 제외
def test_publish_subscribe_cross_process(tmp_path: Path):
    """다른 process 의 subscriber 가 publish 받음 — Zenoh between-session network."""
    endpoint = "tcp/127.0.0.1:17447"
    parent_cfg = {
        "mode": "peer",
        "scouting": {"multicast": {"enabled": False}},
        "listen": [endpoint],
    }
    parent = ZenohTransport(parent_cfg)

    out_file = tmp_path / "received.bin"
    script_path = tmp_path / "subscriber.py"
    script_path.write_text(_SUBSCRIBER_SCRIPT, encoding="utf-8")

    backend_path = str(Path(__file__).resolve().parents[2])

    env = os.environ.copy()
    env["BACKEND_PATH"] = backend_path
    env["ZENOH_ENDPOINT"] = endpoint
    env["OUT_FILE"] = str(out_file)
    env["KEY"] = "test/xproc"
    env["PYTHONIOENCODING"] = "utf-8"

    proc = subprocess.Popen(
        [sys.executable, str(script_path)],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        # child setup (zenoh open + connect + subscribe) 대기
        time.sleep(2.0)
        parent.publish("test/xproc", b"cross-process-hello")
        # child 가 파일 쓸 때까지 polling
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if out_file.exists():
                break
            time.sleep(0.1)
        rc = proc.wait(timeout=10.0)
        stderr = proc.stderr.read().decode(
            "utf-8", errors="replace") if proc.stderr else ""
        assert out_file.exists(), f"child 미수신 (rc={rc}, stderr={stderr})"
        assert out_file.read_bytes() == b"cross-process-hello"
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        parent.close()


# ─── liveliness — presence 관측 (분산 부팅 순서 해방의 L1) ─────────

def _wait_until(pred, timeout: float = 3.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(0.05)
    return False


@pytest.mark.sim  # 2세션 tcp + 전이 폴링 (~수 초) — fast loop 제외
def test_liveliness_presence_lifecycle():
    """4 전이 검증 (2026-07-07 probe 회귀 잠금):
    ① 구독 전 이미 살아있는 token → history 로 즉시 alive (부팅 순서 무관의 핵심)
    ② undeclare → gone
    ③ 재선언 → 다시 alive (owner 재시작 감지)
    ④ owner 세션 close → gone (크래시 등가 — 커스텀 이벤트가 못 주는 것)
    """
    ep = "tcp/127.0.0.1:17561"
    owner = ZenohTransport({**_LOCAL_CFG, "listen": [ep]})
    consumer = ZenohTransport({**_LOCAL_CFG, "connect": [ep]})
    events: list[tuple[str, bool]] = []
    sub = None
    try:
        # ① 구독 "전에" token 선언 — history 수신 검증
        token = owner.declare_liveliness("srv/test/live_svc")
        time.sleep(0.3)  # peer 연결 안정

        sub = consumer.subscribe_liveliness(
            "srv/test/**", lambda k, alive: events.append((k, alive))
        )
        assert _wait_until(lambda: ("srv/test/live_svc", True) in events), (
            f"사전 존재 token 의 history alive 미수신: {events}"
        )

        # ② undeclare → gone
        token.undeclare()
        assert _wait_until(lambda: ("srv/test/live_svc", False) in events), (
            f"undeclare gone 미수신: {events}"
        )

        # ③ 재선언 → 다시 alive (owner 재시작 시나리오)
        owner.declare_liveliness("srv/test/live_svc")
        assert _wait_until(lambda: events.count(("srv/test/live_svc", True)) >= 2), (
            f"재선언 alive 미수신: {events}"
        )

        # ④ owner 세션 close → gone (크래시 등가)
        owner.close()
        assert _wait_until(
            lambda: events.count(("srv/test/live_svc", False)) >= 2, timeout=5.0
        ), f"세션 close gone 미수신: {events}"
    finally:
        if sub is not None:
            sub.undeclare()
        consumer.close()
        # owner 는 ④에서 이미 close — 재호출 오류 무시
        try:
            owner.close()
        except Exception:
            pass
