"""분산 sim E2E — calibration capture flow 가 3 프로세스 분산 위에서 동작.

3 프로세스 localhost:
- pc_sim          — calibration / storage / scene3d / task / detector /
                    reconstruction + bridge. 카메라 / 모터 USB 직결 없음.
- pi_motor_sim    — mock_motor + motion + robot=so101_6dof_0.
- pi_camera_sim   — mock_camera (CALIB_SIM_BOARD=1) + robot=so101_6dof_0.

Zenoh 가 토픽 / 서비스를 프로세스 경계 넘어 라우팅. test 는 같은 flow (intrinsic
+ handeye capture + finalize) 가 분산 토폴로지 위에서 깨지지 않는지 검증.

LAN 격리: host_pc_sim listen `tcp/127.0.0.1:7447`, 다른 두 sim 이 거기로 connect.
multicast OFF. test process 의 zenoh peer 도 같은 endpoint 로 connect →
localhost loopback 만, 같은 LAN 의 실 robot pi backend 와 격리.

cross-process 의존 검증:
- calibration_node (pc_sim) ← MOTOR_STATE_JOINT publish (pi_motor_sim) cache 채움
- calibration_node (pc_sim) ← CAMERA_STREAM_RAW + CAMERA_DEPTH_FRAME (pi_camera_sim)
- calibration_node → motion_node (pi_motor_sim) MoveJ 서비스 호출
- finalize 후 storage (pc_sim) 의 run status = ready_for_analysis 조회

backend 3 process boot (~30s) + 27×2 자세 × ~2s = ~110s + buffer = ~3분.
"""

from __future__ import annotations

import json
import subprocess
import time
from collections.abc import Iterator
from typing import Any

import pytest

try:
    import zenoh
except ImportError:
    pytest.skip("zenoh 미설치", allow_module_level=True)

import requests

from tests.conftest import isolated_zenoh_config, spawn_backend

BRIDGE_URL = "http://localhost:8000"
ROBOT_ID = "so101_6dof_0"

GET_TCP_SVC = f"horibot/{ROBOT_ID}/motion/srv/get_tcp"
MOVE_J_SVC = f"horibot/{ROBOT_ID}/motion/srv/move_j"
SET_STREAM_SVC = f"horibot/{ROBOT_ID}/scene3d/srv/set_stream"
INTR_START = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/start"
INTR_CAPTURE = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/capture"
INTR_SAVE = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/save"
HE_START = f"horibot/{ROBOT_ID}/calib/srv/handeye/start"
HE_CAPTURE = f"horibot/{ROBOT_ID}/calib/srv/handeye/capture"
HE_FINALIZE = f"horibot/{ROBOT_ID}/calib/srv/handeye/finalize"
HE_RESET = f"horibot/{ROBOT_ID}/calib/srv/handeye/reset"
LIST_RUNS_SVC = "horibot/storage/srv/calibration/list_runs"


def _call(session, key: str, data: dict, timeout: float = 10.0) -> dict:
    payload = json.dumps({"timestamp": time.time(), "data": data}).encode()
    for r in session.get(key, payload=payload, timeout=timeout):
        if r.ok is not None:
            return json.loads(r.ok.payload.to_bytes())
        return {"success": False, "message": "err reply", "data": None}
    return {"success": False, "message": "no reply", "data": None}


def _move_to(session, degs: list[float], settle: float = 2.5) -> None:
    joints = [{"id": i + 1, "degree": float(d)} for i, d in enumerate(degs)]
    _call(session, MOVE_J_SVC, {"joints": joints})
    time.sleep(settle)


# ─── Fixtures ────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def distributed_backend() -> Iterator[None]:
    """3 프로세스 분산 토폴로지 — localhost multi-process.

    Start 순서: motor / camera 먼저 (가벼움) → pc 마지막 (storage init + bridge
    boot 무거움). 마지막 pc 가 bridge HTTP 응답해야 ready 판정.

    Process 정리: 모든 proc terminate/wait/kill 3단 + conftest atexit (자기가 띄운
    pid 만 정리, 사용자 production 안 건드림).
    """
    procs = [
        spawn_backend("pi_motor_sim"),
        spawn_backend("pi_camera_sim", env_extra={"CALIB_SIM_BOARD": "1"}),
        spawn_backend("pc_sim"),
    ]
    deadline = time.time() + 150
    ready = False
    failure: str | None = None
    while time.time() < deadline:
        for p, label in zip(procs, ("pi_motor_sim", "pi_camera_sim", "pc_sim")):
            if p.poll() is not None:
                failure = f"{label} 조기 종료 code={p.returncode}"
                break
        if failure:
            break
        try:
            r = requests.get(f"{BRIDGE_URL}/openapi.json", timeout=1)
            if r.status_code == 200:
                ready = True
                break
        except Exception:
            pass
        time.sleep(1)

    if failure:
        for p in procs:
            try:
                p.kill()
                p.wait(timeout=3)
            except Exception:
                pass
        raise RuntimeError(failure)
    if not ready:
        for p in procs:
            try:
                p.kill()
                p.wait(timeout=3)
            except Exception:
                pass
        raise RuntimeError("분산 backend boot timeout (150s)")
    # 3 peer 자동 발견 안정 (localhost TCP — 멀티캐스트 안 쓰지만 안전 margin).
    time.sleep(5)

    yield

    for p in procs:
        try:
            p.terminate()
        except Exception:
            pass
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()


@pytest.fixture(scope="module")
def zsession(distributed_backend) -> Iterator[Any]:
    """test process zenoh peer — isolated config + cross-process cache wait."""
    session = zenoh.open(isolated_zenoh_config())
    time.sleep(2)
    # motion (pi_motor_sim) 이 cross-process 로 reachable 할 때까지.
    deadline = time.time() + 30
    while time.time() < deadline:
        if _call(session, GET_TCP_SVC, {}, timeout=2.0).get("success"):
            break
        time.sleep(0.5)
    else:
        session.close()
        raise RuntimeError("분산: motion (pi_motor_sim) 미발견 (30s)")
    yield session
    session.close()


# ─── Tests ───────────────────────────────────────────────────────────


def test_distributed_intrinsic(zsession):
    """Intrinsic — pc_sim 의 calibration_node 가 pi_camera_sim 의 frame +
    pi_motor_sim 의 motor state 를 cross-process 로 받아 capture / save.
    """
    from modules.calibration.sim_board import SIM_CALIB_POSES_DEG

    assert _call(zsession, INTR_START, {}).get("success")

    pose_set = []
    for base in SIM_CALIB_POSES_DEG:
        pose_set.append(base)
        pose_set.append([base[0] + 8.0] + list(base[1:]))
        pose_set.append([base[0] - 8.0] + list(base[1:]))

    detected = 0
    for degs in pose_set:
        _move_to(zsession, degs, settle=2.5)
        res = _call(zsession, INTR_CAPTURE, {})
        if res.get("success") and res.get("data", {}).get("detected"):
            detected += 1
    assert detected >= 5, f"분산 intrinsic 검출 {detected}/{len(pose_set)} — 최소 5"

    save = _call(zsession, INTR_SAVE, {})
    assert save.get("success"), f"분산 intrinsic save 실패: {save}"
    assert save["data"]["rms_error"] < 2.0


def test_distributed_handeye_capture_finalize(zsession):
    """Handeye — depth_frame 이 pi_camera_sim 에서 pc_sim 의 calibration_node
    까지 cross-process 도착, capture row + blob 이 pc_sim storage 에 영속.
    """
    from modules.calibration.sim_board import SIM_CALIB_POSES_DEG

    _call(zsession, HE_RESET, {})

    stream = _call(zsession, SET_STREAM_SVC, {"enabled": True})
    assert stream.get("success"), f"분산 depth stream ON 실패: {stream}"
    time.sleep(1.5)

    try:
        start = _call(zsession, HE_START, {})
        assert start.get("success"), f"분산 HE_START 실패: {start}"
        run_id = start["data"]["run_id"]
        assert run_id > 0

        pose_set = []
        for base in SIM_CALIB_POSES_DEG:
            pose_set.append(base)
            pose_set.append([base[0] + 8.0] + list(base[1:]))

        detected = 0
        for degs in pose_set:
            _move_to(zsession, degs, settle=2.5)
            res = _call(zsession, HE_CAPTURE, {})
            if res.get("success") and res.get("data", {}).get("detected"):
                detected += 1
        assert detected >= 3, f"분산 handeye 검출 {detected}/{len(pose_set)} — 최소 3"

        fin = _call(zsession, HE_FINALIZE, {})
        assert fin.get("success"), f"분산 finalize 실패: {fin}"
        assert fin["data"]["run_id"] == run_id

        # storage (pc_sim) 의 run status 검증 — cross-process RDB / ObjectStore.
        runs = _call(zsession, LIST_RUNS_SVC, {"robot_id": ROBOT_ID, "limit": 50})
        assert runs.get("success"), f"분산 list_runs 실패: {runs}"
        handeye_run = next(
            (r for r in runs["data"]["runs"] if r["run"]["id"] == run_id), None
        )
        assert handeye_run is not None, "분산 finalize 한 run 못 찾음"
        assert handeye_run["run"]["status"] == "ready_for_analysis"
    finally:
        _call(zsession, SET_STREAM_SVC, {"enabled": False})
