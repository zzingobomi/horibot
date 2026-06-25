"""tests/framework/test_transport.py — Step 1 검증 (§11).

검증 두 case:
1. ZenohTransport.publish(...) → 같은 session 안 subscriber callback (same-session in-routing)
2. ZenohTransport.publish(...) → 다른 process subscriber callback (cross-process subprocess)

추가 surface — register_service / call / handler exception → RemoteError / timeout → TimeoutError.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from framework.transport.protocol import RemoteError, Transport
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


# ─── Transport(Protocol) 구조적 만족 ─────────────────

def test_zenoh_transport_satisfies_protocol(transport: ZenohTransport):
    assert isinstance(transport, Transport)


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

sys.path.insert(0, os.environ["BACKEND_V2_PATH"])

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

    backend_v2_path = str(Path(__file__).resolve().parents[2])

    env = os.environ.copy()
    env["BACKEND_V2_PATH"] = backend_v2_path
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
        stderr = proc.stderr.read().decode("utf-8", errors="replace") if proc.stderr else ""
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
