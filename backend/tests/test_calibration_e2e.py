"""E2E — host_mock + ChArUco eye-in-hand 시뮬 캘리브레이션 capture flow 검증.

mock_camera 가 CALIB_SIM_BOARD=1 일 때 로봇 joint → board_in_cam 계산해 ChArUco
보드 렌더. 실 코드 경로 (FrameCache → board.detect → solvePnP → storage append)
를 브라우저 / 실 하드웨어 없이 e2e 검증.

새 design (calibration_ux_rewrite — capture-only):
- backend = 자세 자유 capture + storage 저장만. BA / σ / commit 다 offline 스크립트
  (`backend/scripts/calibrate_offline.py`).
- HE_FINALIZE = run status → `ready_for_analysis`. test 는 finalize 까지만 검증.

LAN 격리: host_mock yaml + conftest.isolated_zenoh_config 가 multicast OFF +
localhost TCP only. 같은 LAN 의 실 robot pi backend 가 떠있어도 reach X.

Process 정리: spawn_backend (venv python 직접) + fixture finalize 3단 + atexit.

검증:
1. intrinsic — start → 9 자세 capture → save (rms 합리).
2. handeye — depth stream ON → start → 9 자세 capture → finalize → run status
   == ready_for_analysis.

backend boot ~15s + 9×2 자세 × 2 phase ≈ 60s = 약 90s.
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

# Robot-scoped services.
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

# Global storage service (robot_id 는 payload).
LIST_RUNS_SVC = "horibot/storage/srv/calibration/list_runs"


# ─── Helpers ─────────────────────────────────────────────────────────


def _call(session, key: str, data: dict, timeout: float = 8.0) -> dict:
    """BaseNode.call_service 와 동일 wire — `{timestamp, data}` → `{success, message, data}`."""
    payload = json.dumps({"timestamp": time.time(), "data": data}).encode()
    for r in session.get(key, payload=payload, timeout=timeout):
        if r.ok is not None:
            return json.loads(r.ok.payload.to_bytes())
        return {"success": False, "message": "err reply", "data": None}
    return {"success": False, "message": "no reply", "data": None}


def _move_to(session, degs: list[float], settle: float = 4.0) -> None:
    """MoveJ + 자세 도달 + mock camera ChArUco 렌더 반영 대기.

    settle 4.0s — SO-101 6dof + 큰 angle (예: J3 -80°) jump 가 mock_motor 의
    100Hz 보간으로 ~3s 걸릴 수 있어 안전 margin. 짧으면 trajectory 종점 전에
    capture 돌아 board 가 시야 밖.
    """
    joints = [{"id": i + 1, "degree": float(d)} for i, d in enumerate(degs)]
    _call(session, MOVE_J_SVC, {"joints": joints})
    time.sleep(settle)


# ─── Fixtures ────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def sim_backend() -> Iterator[None]:
    """host_mock + CALIB_SIM_BOARD=1 subprocess.

    Process 안전망: spawn_backend (venv python 직접) → finalize 시 terminate/wait/kill
    3단 → conftest atexit 가 KeyboardInterrupt / 누락 시 safety net.
    LAN 격리: host_mock yaml 의 zenoh = multicast OFF + localhost TCP.
    """
    proc = spawn_backend("mock", env_extra={"CALIB_SIM_BOARD": "1"})
    deadline = time.time() + 120
    ready = False
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"backend 조기 종료 code={proc.returncode}")
        try:
            r = requests.get(f"{BRIDGE_URL}/openapi.json", timeout=1)
            if r.status_code == 200:
                ready = True
                break
        except Exception:
            pass
        time.sleep(1)
    if not ready:
        proc.kill()
        proc.wait()
        raise RuntimeError("backend boot timeout (120s)")
    # peer 연결 + calibration_node 부팅 background (storage fetch + cache push).
    time.sleep(3)

    yield

    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


@pytest.fixture(scope="module")
def zsession(sim_backend) -> Iterator[Any]:
    """test process zenoh peer — 격리 config + motion cache 채워질 때까지 wait."""
    session = zenoh.open(isolated_zenoh_config())
    time.sleep(1)
    deadline = time.time() + 30
    while time.time() < deadline:
        if _call(session, GET_TCP_SVC, {}, timeout=2.0).get("success"):
            break
        time.sleep(0.5)
    else:
        session.close()
        raise RuntimeError("motion cache 미준비 (30s)")
    yield session
    session.close()


# ─── Tests ───────────────────────────────────────────────────────────


def test_intrinsic_pipeline(zsession):
    """Intrinsic — start → 9 자세 capture → save.

    ChArUco 렌더는 self-consistent 이므로 rms 는 매우 작게 나옴 (< 2.0 px).
    `detected >= 5` 는 일부 자세에서 보드가 시야 밖일 수 있어 여유.
    """
    from modules.calibration.sim_board import SIM_CALIB_POSES_DEG

    assert _call(zsession, INTR_START, {}).get("success")

    # 9 base 자세 + J1 ±8° 변형 = 27 자세 후보. intrinsic 은 OpenCV
    # calibrateCamera 가 5+ 자세 요구 — base 만 (9) 으로는 시야 밖 자세 빠지면
    # 미달. 변형 추가로 안전 margin.
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

    # flow 검증 — 최소 5 자세 (OpenCV calibrateCamera 요구치).
    assert detected >= 5, f"intrinsic 검출 {detected}/{len(pose_set)} — 최소 5"

    save = _call(zsession, INTR_SAVE, {})
    assert save.get("success"), f"intrinsic save 실패: {save}"
    rms = save["data"]["rms_error"]
    assert rms < 2.0, f"intrinsic rms={rms} 너무 큼"


def test_handeye_capture_finalize(zsession):
    """Handeye — depth stream ON → start → 9 자세 capture → finalize.

    HE_CAPTURE 가 fresh depth_frame 요구 → SCENE3D_SET_STREAM enable=True 로
    mock_camera 8Hz depth publish 활성. test_intrinsic_pipeline 이 먼저 돌아
    intrinsic 이 in-memory 에 있어야 HE_START 성공 (calibration_node.start
    검사: `st.intrinsic.result is None` reject).
    """
    from modules.calibration.sim_board import SIM_CALIB_POSES_DEG

    # 직전 fail / 잔여 in_progress 세션 정리.
    _call(zsession, HE_RESET, {})

    stream = _call(zsession, SET_STREAM_SVC, {"enabled": True})
    assert stream.get("success"), f"depth stream ON 실패: {stream}"
    # mock_camera 8Hz publish 첫 frame 대기.
    time.sleep(1.0)

    try:
        start = _call(zsession, HE_START, {})
        assert start.get("success"), f"HE_START 실패: {start}"
        run_id = start["data"]["run_id"]
        assert run_id > 0

        detected = 0
        for degs in SIM_CALIB_POSES_DEG:
            _move_to(zsession, degs)
            res = _call(zsession, HE_CAPTURE, {})
            if res.get("success") and res["data"].get("detected"):
                detected += 1
        # flow 검증 — 최소 3 자세. depth_frame fresh + ChArUco 검출 cascade 통과.
        assert detected >= 3, f"handeye 검출 {detected}/9 — 최소 3"

        fin = _call(zsession, HE_FINALIZE, {})
        assert fin.get("success"), f"finalize 실패: {fin}"
        assert fin["data"]["run_id"] == run_id
        assert fin["data"]["pose_count"] == detected

        # 검증: run status → ready_for_analysis (offline 분석 대기).
        runs = _call(
            zsession, LIST_RUNS_SVC, {"robot_id": ROBOT_ID, "limit": 50}
        )
        assert runs.get("success"), f"list_runs 실패: {runs}"
        handeye_run = next(
            (
                r
                for r in runs["data"]["runs"]
                if r["run"]["id"] == run_id
            ),
            None,
        )
        assert handeye_run is not None, "finalize 한 run 못 찾음"
        assert handeye_run["run"]["status"] == "ready_for_analysis"
        assert handeye_run["run"]["kind"] == "hand_eye"
    finally:
        _call(zsession, SET_STREAM_SVC, {"enabled": False})
