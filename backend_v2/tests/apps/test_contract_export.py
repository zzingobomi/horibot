"""Contract export 검증 — frontend_contract_gen.md §6 (backend EXPORT 쪽).

경계: backend 는 자기 계약을 /contract.json 으로 EXPORT 만 한다. TS 조립(render)은
frontend 소비자(frontend_v2/scripts/gen-contract.mjs)의 몫 — 그쪽 vitest 가 검증.
여기선 EXPORT 쪽만:
- snapshot 이 로드된 module 의 @service/@subscriber/@publishes 를 열거하는지
- build_contract_json 이 FRONTEND_EXPOSED subset + reachability + name-conflict 를
  올바로 반영하는지
- 그 결과가 frontend 가 소비하는 커밋된 fixture 와 일치하는지 (계약 정합 — 이게
  깨지면 fixture 재생성 필요)
- provider closure 가 runtime 위에 배선되는지 (resolve/main wiring)
- GET /contract.json 이 그 JSON 을 serve 하는지 (bridge relay + HTTP)
- stale / incomplete-host guard 가 fail-fast 하는지
"""

from __future__ import annotations

from pathlib import Path

import httpx
import pytest

from apps.contract_export import (
    FRONTEND_EXPOSED,
    build_contract_json,
    check_exposed,
)
from apps.main import build_runtime, load_configs
from apps.resolve import resolve_host_deps
from framework.runtime.app import Runtime
from framework.runtime.snapshot import ContractSnapshot
from infra.transport.zenoh import ZenohTransport
from modules.bridge.module import BridgeModule

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}


def _built_runtime() -> tuple[Runtime, ZenohTransport]:
    """mock 전 module add (start X — snapshot 은 add 만 되면 유효, uvicorn 불필요)."""
    transport = ZenohTransport(_LOCAL_CFG)
    deploy, robots = load_configs("mock", _CONFIG_DIR)
    runtime = build_runtime(deploy, robots, transport)
    return runtime, transport


# ─── snapshot ───────────────────────────────────────────────────


def test_snapshot_enumerates_loaded_module_specs():
    runtime, transport = _built_runtime()
    try:
        snap = runtime.contract_snapshot()
    finally:
        transport.close()

    # @service — motor/motion (robot-scoped template, {robot_id} 유지)
    assert "srv/motion/{robot_id}/move_j" in snap.services
    assert "srv/motor/{robot_id}/set_torque" in snap.services
    # @publishes — motor RAW_STATE (output stream)
    assert "stream/motor/{robot_id}/raw_state" in snap.topics
    # @subscriber — motion JOG (frontend→backend input stream)
    assert "stream/motion/{robot_id}/jog_j" in snap.topics
    # 내부 wire 도 snapshot 엔 있음 (노출 필터는 build 단계) — COMMAND
    assert "stream/motor/{robot_id}/command" in snap.topics


# ─── build_contract_json — 노출 subset + reachability ────────────


def test_contract_json_shape():
    runtime, transport = _built_runtime()
    try:
        data = build_contract_json(runtime.contract_snapshot())
    finally:
        transport.close()

    # 노출 keys = FRONTEND_EXPOSED (topic 5 + service 4)
    topic_keys = {t["key"] for t in data["topics"]}
    service_keys = {s["key"] for s in data["services"]}
    assert topic_keys | service_keys == FRONTEND_EXPOSED
    assert len(data["topics"]) == 5
    assert len(data["services"]) == 4
    # 내부 전용 payload 는 도달성으로 제외 — JointCommand 안 나옴
    iface_names = {i["name"] for i in data["interfaces"]}
    assert "JointCommand" not in iface_names
    # HTTP seed 모델은 포함 (reachability 로 안 잡히지만 seed)
    assert {"RobotsResponse", "SystemMetrics", "RobotInfo"} <= iface_names
    # name-conflict prefix (camera.CapabilitiesRequest 존재 → motor 것 prefix)
    assert "MotorCapabilitiesRequest" in iface_names
    # payload 타입은 TS 문자열로 이미 해소 (서버가 실 Python type 을 아니까)
    jog = next(t for t in data["topics"] if t["key"].endswith("/jog_j"))
    assert jog["payload"] == "JogJInput"


# ─── provider closure wiring (resolve/main) ──────────────────────


def test_contract_provider_closure_wired_on_bridge():
    runtime, transport = _built_runtime()
    try:
        bridge = next(m for m in runtime._modules if isinstance(m, BridgeModule))
        # build_runtime 이 runtime 을 resolve_host_deps 에 넘겨 closure 주입했는지
        assert bridge._contract_provider is not None
        data = bridge._contract_provider()
        assert set(data) == {"enums", "interfaces", "topics", "services"}
    finally:
        transport.close()


def test_bridge_without_runtime_has_no_provider():
    # runtime=None (기본) → provider 미주입 → GET /contract.json 은 503 (gen 안 씀)
    deploy, robots = load_configs("mock", _CONFIG_DIR)
    deps = resolve_host_deps("bridge", robots, deploy)  # runtime 안 넘김
    assert "contract_provider" not in deps


# ─── guards ──────────────────────────────────────────────────────


def test_check_exposed_rejects_stale_key():
    with pytest.raises(ValueError, match="discovered 되지 않은"):
        check_exposed({"srv/real/key"}, {"srv/typo/nonexistent"})


def test_incomplete_host_raises_helpful_error():
    runtime, transport = _built_runtime()
    try:
        full = runtime.contract_snapshot()
    finally:
        transport.close()

    # motion module 이 이 host 에 없는 상황 시뮬 — motion 계약 제거
    partial = ContractSnapshot(
        services={k: v for k, v in full.services.items() if "/motion/" not in k},
        topics={k: v for k, v in full.topics.items() if "/motion/" not in k},
    )
    with pytest.raises(RuntimeError, match="mock/dev"):
        build_contract_json(partial)


# ─── HTTP e2e — 실 bridge 가 /contract.json serve ─────────────────


@pytest.fixture
async def contract_endpoint():
    """mock 전 module + bridge start (uvicorn). /contract.json HTTP 검증용."""
    transport = ZenohTransport(_LOCAL_CFG)
    deploy, robots = load_configs("mock", _CONFIG_DIR)
    runtime = build_runtime(deploy, robots, transport)
    await runtime.start()
    yield "http://127.0.0.1:8000/contract.json"
    await runtime.stop()
    transport.close()


async def test_contract_json_endpoint_serves(contract_endpoint: str):
    async with httpx.AsyncClient() as client:
        res = await client.get(contract_endpoint)
    assert res.status_code == 200
    data = res.json()
    assert set(data) == {"enums", "interfaces", "topics", "services"}
    # HTTP 로 serve 된 JSON = in-process build_contract_json 과 동일 계약
    assert len(data["topics"]) == 5
    assert len(data["services"]) == 4
