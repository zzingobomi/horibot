"""Bridge module C1a 검증 — HTTP helper (`/robots` / `/system`) + boot 배선.

검증:
- registry / resolve_host_deps (RobotConfig → RobotInfo 변환)
- build_runtime 의 host-level (robot-agnostic) 분기
- add_module 의 raw transport 주입 + FastAPI lifecycle
- 실 uvicorn 기동 + httpx GET (relay-only — domain logic 0)

mock.yaml 은 안 건드림 (motor/camera e2e 테스트를 uvicorn 포트에 안 묶이게).
"""

from __future__ import annotations

import socket

from pathlib import Path
from typing import cast

import httpx
import pytest

from apps.config import DeploymentConfig, DriverMode, ModuleEntry, load_robots
from apps.main import build_runtime
from apps.registry import load_module_class
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


def test_registry_has_bridge():
    assert load_module_class("bridge") is BridgeModule


def test_resolve_host_deps_bridge_returns_robot_info():
    deps = resolve_host_deps("bridge", _robots(), _mock_bridge_deploy())
    infos = {r.id: r for r in deps["robots"]}
    # 2026-07-09 omx_f_0 재활성화 — 두 robot 동등 열거 (기본 로봇 개념 없음).
    assert set(infos) == {"so101_6dof_0", "omx_f_0"}
    assert infos["so101_6dof_0"].type == "so101_6dof"
    assert "rgbd" in infos["so101_6dof_0"].capabilities
    # RobotConfig → RobotInfo 변환 — 크로스캘 확정값 (so101=원점 anchor, 2026-07-11)
    assert infos["so101_6dof_0"].base_pose.x == 0.0
    assert infos["omx_f_0"].base_pose.x == 0.0342
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


async def test_get_system(bridge_url: str):
    async with httpx.AsyncClient() as client:
        res = await client.get(f"{bridge_url}/system")
    assert res.status_code == 200
    body = res.json()
    assert "cpu_percent" in body
    assert "mem_percent" in body
