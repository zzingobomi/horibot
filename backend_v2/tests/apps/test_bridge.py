"""Bridge module C1a 검증 — HTTP helper (`/robots` / `/system`) + boot 배선.

검증:
- registry / resolve_host_deps (RobotConfig → RobotInfo 변환)
- build_runtime 의 host-level (robot-agnostic) 분기
- add_module 의 raw transport 주입 + FastAPI lifecycle
- 실 uvicorn 기동 + httpx GET (relay-only — domain logic 0)

mock.yaml 은 안 건드림 (motor/camera e2e 테스트를 uvicorn 포트에 안 묶이게).
"""

from __future__ import annotations

from pathlib import Path

import httpx
import pytest

from apps.config import DeploymentConfig, DriverMode, ModuleEntry, load_robots
from apps.main import build_runtime
from apps.registry import MODULE_REGISTRY
from apps.resolve import resolve_host_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.bridge.module import BridgeModule

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_TEST_PORT = 8077  # mock.yaml 의 8000 과 분리 — 포트 충돌/flakiness 회피


def _robots() -> dict:
    return load_robots()


def _mock_bridge_deploy() -> DeploymentConfig:
    return DeploymentConfig(
        driver_mode=DriverMode.MOCK,
        modules=[ModuleEntry(name="bridge")],  # host-level (robots 비움)
    )


# ─── 배선 ────────────────────────────────────────────────────────


def test_registry_has_bridge():
    assert MODULE_REGISTRY["bridge"] is BridgeModule


def test_resolve_host_deps_bridge_returns_robot_info():
    deps = resolve_host_deps(BridgeModule, _robots(), _mock_bridge_deploy())
    infos = {r.id: r for r in deps["robots"]}
    assert set(infos) == {"so101_6dof_0", "omx_f_0"}
    assert infos["so101_6dof_0"].type == "so101_6dof"
    assert "rgbd" in infos["so101_6dof_0"].capabilities
    assert infos["so101_6dof_0"].base_pose.x == 0.4  # RobotConfig → RobotInfo 변환


def test_build_runtime_wires_host_level_bridge():
    # host-level 분기 (resolve_host_deps + add_module) — start 안 함 (uvicorn X)
    transport = ZenohTransport(_LOCAL_CFG)
    try:
        runtime = build_runtime(_mock_bridge_deploy(), _robots(), transport)
        assert any(isinstance(m, BridgeModule) for m in runtime._modules)
    finally:
        transport.close()


# ─── e2e — 실 uvicorn + HTTP ─────────────────────────────────────


@pytest.fixture
async def bridge_url():
    transport = ZenohTransport(_LOCAL_CFG)
    runtime = Runtime(transport)
    deps = resolve_host_deps(BridgeModule, _robots(), _mock_bridge_deploy())
    runtime.add_module(BridgeModule, port=_TEST_PORT, host="127.0.0.1", **deps)
    await runtime.start()  # uvicorn 기동 + started 까지 대기
    yield f"http://127.0.0.1:{_TEST_PORT}"
    await runtime.stop()
    transport.close()


async def test_get_robots(bridge_url: str):
    async with httpx.AsyncClient() as client:
        res = await client.get(f"{bridge_url}/robots")
    assert res.status_code == 200
    body = res.json()
    ids = {r["id"] for r in body["robots"]}
    assert ids == {"so101_6dof_0", "omx_f_0"}
    assert body["default"] in ids


async def test_get_system(bridge_url: str):
    async with httpx.AsyncClient() as client:
        res = await client.get(f"{bridge_url}/system")
    assert res.status_code == 200
    body = res.json()
    assert "cpu_percent" in body
    assert "mem_percent" in body
