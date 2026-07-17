"""Bridge module C1a 검증 — HTTP helper (`/robots` / `/hosts`) + boot 배선.

검증:
- registry / resolve_host_deps (RobotConfig → RobotInfo 변환)
- build_runtime 의 host-level (robot-agnostic) 분기
- add_module 의 raw transport 주입 + FastAPI lifecycle
- 실 uvicorn 기동 + httpx GET (relay-only — domain logic 0)

mock.yaml 은 안 건드림 (motor/camera e2e 테스트를 uvicorn 포트에 안 묶이게).
"""

from __future__ import annotations

import asyncio
import socket
import time

from pathlib import Path
from typing import cast

import httpx
import pytest

from apps.config import DeploymentConfig, DriverMode, ModuleEntry, load_robots
from apps.main import build_runtime
from apps.resolve import resolve_host_deps
from framework.runtime.app import Runtime
from framework.transport.protocol import RawTransport
from infra.transport.zenoh import ZenohTransport
from modules.bridge.module import BridgeModule

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}


def _robots() -> dict:
    return load_robots()


def _mock_bridge_deploy() -> DeploymentConfig:
    return DeploymentConfig(
        driver_mode=DriverMode.MOCK,
        modules=[ModuleEntry(name="bridge")],  # host-level (robots 비움)
        bridge_port=0,  # ephemeral — 다른 backend/테스트와 포트 충돌 원천 차단
    )


# ─── 배선 ────────────────────────────────────────────────────────


def test_resolve_host_deps_bridge_returns_robot_info():
    deps = resolve_host_deps("bridge", _robots(), _mock_bridge_deploy())
    infos = {r.id: r for r in deps["robots"]}
    # 2026-07-09 omx_f_0 재활성화 — 두 robot 동등 열거 (기본 로봇 개념 없음).
    assert set(infos) == {"so101_6dof_0", "omx_f_0"}
    assert infos["so101_6dof_0"].type == "so101_6dof"
    assert "rgbd" in infos["so101_6dof_0"].capabilities
    # RobotConfig → RobotInfo 변환 — base_pose(크로스캘)가 변환을 통과하는지만
    # 본다. 좌표 literal 은 재캘 때마다 바뀌는 robots.yaml 미러라 잠그지 않는다.
    assert infos["omx_f_0"].base_pose != infos["so101_6dof_0"].base_pose
    assert infos["omx_f_0"].type == "omx_f"
    assert "rgbd" not in infos["omx_f_0"].capabilities  # UVC color-only


def test_resolve_bridge_excludes_disabled_robots():
    # robots.yaml spec — enabled=false robot 은 런타임이 무시 (노출 X).
    robots = load_robots()
    robots["omx_f_0"] = robots["omx_f_0"].model_copy(update={"enabled": False})
    deps = resolve_host_deps("bridge", robots, _mock_bridge_deploy())
    ids = {r.id for r in deps["robots"]}
    assert ids == {"so101_6dof_0"}


def test_build_runtime_wires_host_level_bridge():
    # host-level 분기 (resolve_host_deps + add_module) — start 안 함 (uvicorn X)
    transport = ZenohTransport(_LOCAL_CFG)
    try:
        runtime = build_runtime(_mock_bridge_deploy(), _robots(), transport)
        assert any(isinstance(m, BridgeModule) for m in runtime._modules)
    finally:
        transport.close()


# ─── start — 포트 점유 시 fast-fail ──────────────────────────────


class _UnusedTransport:
    """RawTransport 스텁 — start 경로는 transport 를 안 건드림."""

    def call(self, key: str, payload: bytes, timeout: float = 5.0) -> bytes:
        raise NotImplementedError

    def publish(self, key: str, payload: bytes) -> None:
        raise NotImplementedError

    def subscribe(self, key: str, callback) -> object:  # noqa: ANN001
        raise NotImplementedError


async def test_start_port_conflict_raises_clear_error():
    """점유된 포트 → uvicorn 의 sys.exit(1) (SystemExit — 이벤트 루프째 무너뜨려
    caller 의 rollback/teardown 전부 스킵, pytest hang) 이 아니라 명확한
    RuntimeError 로 즉시 실패해야 함 (실 사고 2026-07-07: 유령 backend 의 :8000)."""
    blocker = socket.create_server(("127.0.0.1", 0))
    port = blocker.getsockname()[1]
    bridge = BridgeModule(
        transport=cast(RawTransport, _UnusedTransport()),
        robots=[],
        host="127.0.0.1",
        port=port,
    )
    try:
        with pytest.raises(RuntimeError, match="bind 실패"):
            await bridge.start()
    finally:
        blocker.close()


# ─── e2e — 실 uvicorn + HTTP ─────────────────────────────────────


@pytest.fixture
async def bridge_url():
    transport = ZenohTransport(_LOCAL_CFG)
    runtime = Runtime(transport)
    deps = resolve_host_deps("bridge", _robots(), _mock_bridge_deploy())
    bridge = runtime.add_module(BridgeModule, host="127.0.0.1", **deps)
    await runtime.start()  # uvicorn 기동 + started 까지 대기 (port=0 → 실 포트 갱신)
    yield f"http://127.0.0.1:{bridge.port}"
    await runtime.stop()
    transport.close()


async def test_get_robots(bridge_url: str):
    async with httpx.AsyncClient() as client:
        res = await client.get(f"{bridge_url}/robots")
    assert res.status_code == 200
    body = res.json()
    ids = {r["id"] for r in body["robots"]}
    # enabled robot 전부 노출 (2026-07-09 omx_f_0 재활성화 — N=2).
    assert ids == {"so101_6dof_0", "omx_f_0"}


async def test_static_robot_mount_serves_urdf(bridge_url: str):
    # frontend urdf-loader 가 받을 robot URDF 를 /robot 으로 서빙
    async with httpx.AsyncClient() as client:
        res = await client.get(
            f"{bridge_url}/robot/so101_6dof/urdf/so101_6dof.urdf"
        )
    assert res.status_code == 200
    assert "<robot" in res.text


async def test_get_hosts_fan_in_and_staleness():
    """GET /hosts = host_monitor fan-in 집계. 여러 host 가 한 키로 발행 → bridge 가
    payload.host 로 demux (§3.4.1). stale(오래된 timestamp) → offline 파생."""
    from framework.contract.publisher import encode_event
    from modules.host_monitor.contract import HostMetrics, HostMonitor

    transport = ZenohTransport(_LOCAL_CFG)
    runtime = Runtime(transport)
    deps = resolve_host_deps("bridge", _robots(), _mock_bridge_deploy())
    bridge = runtime.add_module(BridgeModule, host="127.0.0.1", **deps)
    await runtime.start()
    url = f"http://127.0.0.1:{bridge.port}"
    try:
        now = time.time()
        samples = [
            HostMetrics(host="pc", seq=0, timestamp_unix=now,
                        cpu_percent=12.0, mem_percent=40.0),
            HostMetrics(host="pi_hori1", seq=0, timestamp_unix=now,
                        cpu_percent=5.0, mem_percent=30.0),
            HostMetrics(host="pi_hori2", seq=0, timestamp_unix=now - 100.0,
                        cpu_percent=0.0, mem_percent=0.0),  # stale → offline
        ]
        for m in samples:
            transport.publish(str(HostMonitor.Stream.METRICS), encode_event(m))

        hosts: dict = {}
        for _ in range(60):
            async with httpx.AsyncClient() as client:
                res = await client.get(f"{url}/hosts")
            hosts = {h["host"]: h for h in res.json()["hosts"]}
            if len(hosts) >= 3:
                break
            await asyncio.sleep(0.05)

        # 여러 publisher 가 한 키로 → 구독자 하나가 전부 fan-in
        assert set(hosts) == {"pc", "pi_hori1", "pi_hori2"}
        assert hosts["pc"]["online"] is True
        assert hosts["pc"]["cpu_percent"] == 12.0
        assert hosts["pi_hori1"]["online"] is True
        # stale = offline (침묵 아님: age_s 로 사유 표시)
        assert hosts["pi_hori2"]["online"] is False
        assert hosts["pi_hori2"]["age_s"] > 50
    finally:
        await runtime.stop()
        transport.close()
