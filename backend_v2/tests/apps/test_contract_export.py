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
    build_contract_graph,
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

    # 노출 keys = FRONTEND_EXPOSED (topic 6 + service 4)
    topic_keys = {t["key"] for t in data["topics"]}
    service_keys = {s["key"] for s in data["services"]}
    assert topic_keys | service_keys == FRONTEND_EXPOSED
    assert len(data["topics"]) == 11  # +scene3d CLOUD +scan BUILD_PROGRESS
    assert len(data["services"]) == 24  # +scene3d SET_STREAM +scan 9 서비스
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
    assert len(data["topics"]) == 11  # +scene3d CLOUD +scan BUILD_PROGRESS
    assert len(data["services"]) == 24  # +scene3d SET_STREAM +scan 9 서비스


# ─── build_contract_graph — unfiltered attribution + wiring (§5.2) ─


def _built_graph() -> dict:
    runtime, transport = _built_runtime()
    try:
        return build_contract_graph(
            runtime.module_contracts(), runtime.contract_snapshot()
        )
    finally:
        transport.close()


def test_graph_nodes_are_contentful_modules_only():
    graph = _built_graph()
    ids = {m["id"] for m in graph["modules"]}
    # contract 있는 module — bridge (relay, contract 0) 는 node 제외
    assert ids == {
        "MotorDriverModule",
        "MotionModule",
        "CameraDriverModule",
        "CameraDecodedModule",
        "CalibrationModule",
        "Scene3DModule",
        "ScanModule",
    }
    assert "BridgeModule" not in ids
    by_id = {m["id"]: m for m in graph["modules"]}
    assert by_id["MotorDriverModule"]["domain"] == "motor"
    assert by_id["MotionModule"]["domain"] == "motion"
    assert by_id["CameraDriverModule"]["domain"] == "camera"
    assert all(m["robot_scoped"] for m in graph["modules"])


def test_graph_edges_direction_and_no_service_edges():
    graph = _built_graph()
    edges = {(e["source"], e["target"], e["key"]) for e in graph["edges"]}
    # Motor → Motion (raw_state 는 Motor 가 publish, Motion 이 subscribe)
    assert (
        "MotorDriverModule",
        "MotionModule",
        "stream/motor/{robot_id}/raw_state",
    ) in edges
    # Motion → Motor (command 는 Motion 이 publish, Motor 가 subscribe) — 반대 방향
    assert (
        "MotionModule",
        "MotorDriverModule",
        "stream/motor/{robot_id}/command",
    ) in edges
    # CameraDriver → CameraDecoded (jpeg / depth_raw)
    assert (
        "CameraDriverModule",
        "CameraDecodedModule",
        "stream/camera/{robot_id}/jpeg",
    ) in edges
    # backend publisher 없는 stream (JOG_J = frontend→backend) 은 엣지 없음
    assert all("/jog_j" not in e["key"] for e in graph["edges"])
    # service 는 엣지가 아니라 owner node 속성 (§2 caller 한계)
    assert all(e["category"] in ("stream", "event") for e in graph["edges"])
    assert all(not e["key"].startswith("srv/") for e in graph["edges"])


def test_graph_is_unfiltered():
    graph = _built_graph()
    # 내부 wire (Motion→Motor COMMAND) 도 keys 에 포함 — build_contract_json 은 제외
    cmd = "stream/motor/{robot_id}/command"
    assert cmd in graph["keys"]
    assert graph["keys"][cmd]["category"] == "stream"
    assert graph["keys"][cmd]["payload"] == "JointCommand"
    # 그 payload model 도 models 에 (build_contract_json 은 reachability 로 제외했음)
    assert "JointCommand" in graph["models"]
    assert graph["models"]["JointCommand"]["positions_raw"] == "number[]"
    # service key 는 owner node 속성 — keys 엔 req/res 로
    move_j = "srv/motion/{robot_id}/move_j"
    assert graph["keys"][move_j]["category"] == "service"
    assert "req" in graph["keys"][move_j] and "res" in graph["keys"][move_j]


def test_graph_name_conflict_resolution():
    graph = _built_graph()
    # motor.CapabilitiesRequest ↔ camera.CapabilitiesRequest 충돌 → 둘 다 prefix
    assert "MotorCapabilitiesRequest" in graph["models"]
    assert "CameraCapabilitiesRequest" in graph["models"]
    assert "CapabilitiesRequest" not in graph["models"]
    # 그리고 keys 의 service req 이름이 resolved 이름을 참조 (models key 와 매칭)
    cap = graph["keys"]["srv/motor/{robot_id}/capabilities"]
    assert cap["req"] == "MotorCapabilitiesRequest"


def test_graph_provider_closure_wired_on_bridge():
    runtime, transport = _built_runtime()
    try:
        bridge = next(m for m in runtime._modules if isinstance(m, BridgeModule))
        assert bridge._graph_provider is not None
        data = bridge._graph_provider()
        assert set(data) == {"modules", "keys", "models", "edges"}
    finally:
        transport.close()


def test_graph_shows_declared_universe_not_running_fleet():
    """회귀 잡음 — 2026-07-01 사건: PC 배치 (camera_decoded + bridge 만) 자리
    `/contract/graph` 가 CameraDecodedModule 1개만 보였음. 원인: graph provider 가
    `runtime.module_contracts()` (자기 프로세스 module 만) 를 봤음.

    contract_graph_viewer.md §1 = 개발자 뷰어, §4 = unfiltered 전 module 의 전 계약.
    즉 declared universe (MODULE_REGISTRY) 를 그려야 함. 이 assert 는 PC 배치 시뮬
    (camera_decoded + bridge 만 add) 에서도 motor/motion/camera 다 나오는지 검증.
    뒤집으면 그 회귀 즉시 잡힘."""
    from apps.config import DeploymentConfig, DriverMode, ModuleEntry
    from apps.contract_export import build_static_contract_graph

    # PC 배치 시뮬 — camera_decoded + bridge 만 있는 minimal runtime
    pc_deploy = DeploymentConfig(
        driver_mode=DriverMode.MOCK,
        zenoh=_LOCAL_CFG,
        modules=[
            ModuleEntry(name="camera_decoded", robots=["so101_6dof_0"]),
            ModuleEntry(name="bridge", robots=[]),
        ],
    )
    _, robots = load_configs("mock", _CONFIG_DIR)
    transport = ZenohTransport(_LOCAL_CFG)
    try:
        runtime = build_runtime(pc_deploy, robots, transport)
        # runtime 은 module 2개만 — 하지만 static graph 는 registry 전체 봄
        assert len(runtime._modules) == 2

        # static graph — MODULE_REGISTRY 전체 introspect
        graph = build_static_contract_graph()
        ids = {m["id"] for m in graph["modules"]}
        # PC 프로세스에 실제 로드 안 된 Motor/Motion/CameraDriver 도 나와야 함
        assert "MotorDriverModule" in ids, (
            "graph 가 running fleet 만 봄 — declared universe 시각화 실패"
        )
        assert "MotionModule" in ids
        assert "CameraDriverModule" in ids
        assert "CameraDecodedModule" in ids
        # Bridge (contract 0 relay) 는 여전히 filter
        assert "BridgeModule" not in ids
    finally:
        transport.close()


@pytest.fixture
async def graph_endpoint():
    transport = ZenohTransport(_LOCAL_CFG)
    deploy, robots = load_configs("mock", _CONFIG_DIR)
    runtime = build_runtime(deploy, robots, transport)
    await runtime.start()
    yield "http://127.0.0.1:8000/contract/graph"
    await runtime.stop()
    transport.close()


async def test_contract_graph_endpoint_serves(graph_endpoint: str):
    async with httpx.AsyncClient() as client:
        res = await client.get(graph_endpoint)
    assert res.status_code == 200
    data = res.json()
    assert set(data) == {"modules", "keys", "models", "edges"}
    assert {m["id"] for m in data["modules"]} == {
        "MotorDriverModule",
        "MotionModule",
        "CameraDriverModule",
        "CameraDecodedModule",
        "CalibrationModule",
        "Scene3DModule",
        "ScanModule",
    }
    assert len(data["edges"]) >= 4
