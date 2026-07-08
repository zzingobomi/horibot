"""Dev Console 검증 — GET /dev(자체완결 페이지) + POST /dev/invoke(서비스 호출).

개발용 콘솔은 프론트 없이 브라우저로 임의 서비스를 두드리는 dev 작업대. invoke 는
request/reply 라 HTTP 로 매핑(transport.call). 검증:
- dev_console 플래그 게이트 (off → 404)
- invoke 라운드트립 (JSON in → transport.call → JSON out)
- robot-scoped 키 robot_id 치환 (+ 누락 시 400)
- 서비스 예외 → {ok:false, error} 릴레이
서비스는 test_bridge_ws 와 동일하게 같은 ZenohTransport 로 세운다(intra-session).
"""

from __future__ import annotations

import time
from pathlib import Path

import httpx
import msgspec
import pytest

from apps.config import DeploymentConfig, DriverMode, ModuleEntry, load_robots
from apps.resolve import resolve_host_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.bridge.module import BridgeModule

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}


def _deploy(dev_console: bool) -> DeploymentConfig:
    return DeploymentConfig(
        driver_mode=DriverMode.MOCK,
        modules=[ModuleEntry(name="bridge")],
        bridge_port=0,  # ephemeral
        dev_console=dev_console,
    )


async def _bridge(dev_console: bool):
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    runtime = Runtime(transport)
    deps = resolve_host_deps("bridge", load_robots(), _deploy(dev_console))
    bridge_mod = runtime.add_module(BridgeModule, host="127.0.0.1", **deps)
    await runtime.start()
    return transport, runtime, f"http://127.0.0.1:{bridge_mod.port}"


@pytest.fixture
async def dev_bridge():
    transport, runtime, url = await _bridge(dev_console=True)
    yield transport, url
    await runtime.stop()
    transport.close()


# ─── 게이트 ──────────────────────────────────────────────────────


async def test_dev_console_off_returns_404():
    transport, runtime, url = await _bridge(dev_console=False)
    try:
        async with httpx.AsyncClient() as c:
            assert (await c.get(f"{url}/dev")).status_code == 404
            r = await c.post(f"{url}/dev/invoke", json={"key": "srv/x/y"})
            assert r.status_code == 404
    finally:
        await runtime.stop()
        transport.close()


async def test_dev_console_page_served(dev_bridge):
    _transport, url = dev_bridge
    async with httpx.AsyncClient() as c:
        res = await c.get(f"{url}/dev")
    assert res.status_code == 200
    assert "Dev Console" in res.text
    assert "/dev/invoke" in res.text  # 자체완결 JS 가 invoke 엔드포인트 호출


# ─── invoke 라운드트립 ───────────────────────────────────────────


async def test_invoke_round_trip(dev_bridge):
    transport, url = dev_bridge
    key = "srv/test/echo"

    def handler(req_bytes: bytes) -> bytes:
        req = msgspec.msgpack.decode(req_bytes)  # {timestamp, data}
        return msgspec.msgpack.encode(
            {"timestamp": time.time(), "data": {"echo": req["data"]}}
        )

    svc = transport.register_service(key, handler)
    try:
        async with httpx.AsyncClient() as c:
            res = await c.post(
                f"{url}/dev/invoke", json={"key": key, "data": {"x": 1}}
            )
    finally:
        svc.undeclare()

    assert res.status_code == 200
    body = res.json()
    assert body["ok"] is True
    assert body["data"] == {"echo": {"x": 1}}


async def test_invoke_robot_scoped_substitutes_robot_id(dev_bridge):
    transport, url = dev_bridge
    # robot_id 치환 후의 실 키에 핸들러 등록
    svc = transport.register_service(
        "srv/test/r1/move",
        lambda _b: msgspec.msgpack.encode(
            {"timestamp": time.time(), "data": {"moved": "r1"}}
        ),
    )
    try:
        async with httpx.AsyncClient() as c:
            ok = await c.post(
                f"{url}/dev/invoke",
                json={"key": "srv/test/{robot_id}/move", "robot_id": "r1", "data": {}},
            )
            missing = await c.post(
                f"{url}/dev/invoke",
                json={"key": "srv/test/{robot_id}/move", "data": {}},
            )
    finally:
        svc.undeclare()

    assert ok.json()["data"] == {"moved": "r1"}
    assert missing.status_code == 400  # robot-scoped 인데 robot_id 없음


async def test_invoke_relays_service_error(dev_bridge):
    transport, url = dev_bridge
    key = "srv/test/boom"

    def handler(_b: bytes) -> bytes:
        raise ValueError("의도된 실패")

    svc = transport.register_service(key, handler)
    try:
        async with httpx.AsyncClient() as c:
            res = await c.post(f"{url}/dev/invoke", json={"key": key, "data": {}})
    finally:
        svc.undeclare()

    body = res.json()
    assert body["ok"] is False
    assert body["error"]["type"] == "ValueError"
    assert "의도된 실패" in body["error"]["message"]


async def test_invoke_timeout_when_no_service(dev_bridge):
    _transport, url = dev_bridge
    async with httpx.AsyncClient() as c:
        res = await c.post(
            f"{url}/dev/invoke",
            json={"key": "srv/test/nonexistent", "data": {}, "timeout_s": 0.3},
        )
    body = res.json()
    assert body["ok"] is False
    assert body["error"]["type"] == "TimeoutError"
