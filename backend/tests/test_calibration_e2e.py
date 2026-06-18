"""E2E — host_mock + ChArUco eye-in-hand 시뮬로 캘리브레이션 전 파이프라인 headless 검증.

mock 카메라가 로봇 joint 로부터 board_in_cam 을 계산해 ChArUco 렌더 (CALIB_SIM_BOARD=1)
→ 실 캘 코드 경로 (FrameCache → board.detect → solvePnP → 저장 → BA → gating →
observability → commit) 를 브라우저/실 하드웨어 없이 검증.

검증 자리 (docs/handeye_ux_solver_v3_plan.md §8):
- intrinsic 캘: 자세 순회 capture → save → rms 산출
- handeye 캘: START → capture×N (board 검출) → compute (σ) → commit (success)
- CALIB_HANDEYE_PARAM_OBSERVABILITY 토픽 수신 (MVP2 staged gating 결과)

NOTE: zenoh peer scout (멀티캐스트) 의존 — CI 멀티캐스트 X 면 skip. localhost OK.
boot + 자세 순회로 ~2-3분. 느린 test.
"""

from __future__ import annotations

import json
import os
import subprocess
import time
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest

try:
    import zenoh
except ImportError:
    pytest.skip("zenoh 미설치", allow_module_level=True)

import requests

from modules.calibration.sim_board import SIM_CALIB_POSES_DEG

BACKEND_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_URL = "http://localhost:8000"
ROBOT_ID = "so101_6dof_0"

CAM_STATUS = f"horibot/{ROBOT_ID}/camera/state/status"
MOVE_J_SVC = f"horibot/{ROBOT_ID}/motion/srv/move_j"
GET_TCP_SVC = f"horibot/{ROBOT_ID}/motion/srv/get_tcp"
INTR_START = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/start"
INTR_CAPTURE = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/capture"
INTR_SAVE = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/save"
HE_START = f"horibot/{ROBOT_ID}/calib/srv/handeye/start"
HE_CAPTURE = f"horibot/{ROBOT_ID}/calib/srv/handeye/capture"
HE_COMPUTE = f"horibot/{ROBOT_ID}/calib/srv/handeye/compute"
HE_COMMIT = f"horibot/{ROBOT_ID}/calib/srv/handeye/commit"
HE_RESET = f"horibot/{ROBOT_ID}/calib/srv/handeye/reset"
PARAM_OBS_TOPIC = f"horibot/{ROBOT_ID}/calib/state/handeye_param_observability"
# Rollback = DB history (global storage 서비스, robot_id 는 payload).
LIST_RUNS = "horibot/storage/srv/calibration/list_runs"
ACTIVATE = "horibot/storage/srv/calibration/activate"


def _kill_stale() -> None:
    try:
        out = subprocess.check_output(["netstat", "-ano"], text=True, stderr=subprocess.DEVNULL)
    except Exception:
        return
    for line in out.splitlines():
        if ":8000" in line and "LISTENING" in line:
            pid = line.split()[-1]
            subprocess.run(["taskkill", "/PID", pid, "/F"], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


@pytest.fixture(scope="module")
def sim_backend() -> Iterator[None]:
    """host_mock + CALIB_SIM_BOARD=1 subprocess."""
    _kill_stale()
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["CALIB_SIM_BOARD"] = "1"  # mock 카메라 ChArUco 시뮬 ON
    proc = subprocess.Popen(
        ["uv", "run", "--active", "python", "main.py", "--host", "mock"],
        cwd=str(BACKEND_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
    )
    deadline = time.time() + 120
    ready = False
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"backend 조기 종료 code={proc.returncode}")
        try:
            if requests.get(f"{BRIDGE_URL}/openapi.json", timeout=1).status_code == 200:
                ready = True
                break
        except Exception:
            pass
        time.sleep(1)
    if not ready:
        proc.kill()
        proc.wait()
        raise RuntimeError("backend boot timeout")
    time.sleep(3)
    yield
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    _kill_stale()


def _call(session, key: str, data: dict, timeout: float = 8.0) -> dict:
    payload = json.dumps({"timestamp": time.time(), "data": data}).encode()
    for r in session.get(key, payload=payload, timeout=timeout):
        if r.ok is not None:
            return json.loads(r.ok.payload.to_bytes())
        return {"success": False, "message": "err", "data": None}
    return {"success": False, "message": "no reply", "data": None}


@pytest.fixture(scope="module")
def zsession(sim_backend) -> Iterator[Any]:
    session = zenoh.open(zenoh.Config())
    time.sleep(2)
    # 카메라 + 모터 cache 준비 대기.
    deadline = time.time() + 15
    while time.time() < deadline:
        if _call(session, GET_TCP_SVC, {}, timeout=2.0).get("success"):
            break
        time.sleep(0.3)
    else:
        session.close()
        raise RuntimeError("motion/motor cache 미준비")
    yield session
    session.close()


def _move_to(session, degs: list[float], settle: float = 2.2) -> None:
    """MoveJ → 자세 도달 + 카메라 렌더 반영 대기."""
    joints = [{"id": i + 1, "degree": float(d)} for i, d in enumerate(degs)]
    _call(session, MOVE_J_SVC, {"joints": joints})
    time.sleep(settle)


class _Collector:
    def __init__(self, session, topic):
        self.msgs: list[dict] = []
        self._sub = session.declare_subscriber(topic, self._on)

    def _on(self, sample):
        try:
            self.msgs.append(json.loads(sample.payload.to_bytes()))
        except Exception:
            pass

    def close(self):
        self._sub.undeclare()


def test_full_calibration_pipeline(zsession):
    """intrinsic → handeye capture → compute → commit 전 파이프라인 + observability."""
    # 깨끗한 시작.
    _call(zsession, HE_RESET, {})

    # ─── 1. Intrinsic 캘 (sim 보드 자세 순회) ───
    assert _call(zsession, INTR_START, {}).get("success")
    intr_detected = 0
    for degs in SIM_CALIB_POSES_DEG:
        _move_to(zsession, degs)
        res = _call(zsession, INTR_CAPTURE, {})
        if res.get("success") and res.get("data", {}).get("detected"):
            intr_detected += 1
    assert intr_detected >= 5, f"intrinsic board 검출 부족: {intr_detected}"
    save = _call(zsession, INTR_SAVE, {})
    assert save.get("success"), f"intrinsic save 실패: {save}"
    rms = save["data"]["rms_error"]
    assert rms < 2.0, f"intrinsic rms 너무 큼: {rms}"

    # ─── 2. Hand-Eye 캘 ───
    obs_collector = _Collector(zsession, PARAM_OBS_TOPIC)
    try:
        assert _call(zsession, HE_START, {}).get("success")
        he_detected = 0
        for degs in SIM_CALIB_POSES_DEG:
            _move_to(zsession, degs)
            res = _call(zsession, HE_CAPTURE, {})
            if res.get("success") and res.get("data", {}).get("detected"):
                he_detected += 1
        assert he_detected >= 4, f"handeye board 검출 부족: {he_detected}"

        # ─── 3. Compute (σ + per-param observability) ───
        comp = _call(zsession, HE_COMPUTE, {"mode": "physical_sag"}, timeout=90.0)
        assert comp.get("success"), f"compute 실패: {comp}"
        data = comp["data"]
        assert data.get("sigma_rot_deg") is not None
        assert data.get("param_observability") is not None, "observability 미산출"
        po = data["param_observability"]
        assert "scores" in po and "unlocked" in po

        # ─── 4. observability 토픽 수신 확인 ───
        time.sleep(1.0)
        assert len(obs_collector.msgs) >= 1, "param_observability 토픽 미수신"
        last = obs_collector.msgs[-1]
        assert "handeye_rot" in last["scores"]

        # ─── 5. Commit (DB finalize + activate) ───
        commit = _call(zsession, HE_COMMIT, {}, timeout=15.0)
        assert commit.get("success"), f"commit 실패: {commit}"
    finally:
        obs_collector.close()

    # ─── 6. Rollback (DB history) — run 목록 조회 → 결과 재활성 ───
    # 옛 npz backup 아니라 DB history 기반 롤백 검증 (CalibrationHistoryPanel ACTIVATE).
    runs_res = _call(zsession, LIST_RUNS, {"robot_id": ROBOT_ID})
    assert runs_res.get("success"), f"list_runs 실패: {runs_res}"
    runs = runs_res["data"]["runs"]
    assert len(runs) >= 1, "commit 한 run 이 DB history 에 없음"
    # 활성화할 result id 1개 추출 (롤백 = 과거 run 의 result 재활성).
    result_id = None
    for run in runs:
        for r in run.get("results", []):
            if r.get("id") is not None:
                result_id = r["id"]
                break
        if result_id is not None:
            break
    assert result_id is not None, "history run 에 result 없음"
    act = _call(zsession, ACTIVATE, {"result_id": result_id})
    assert act.get("success"), f"롤백(ACTIVATE) 실패: {act}"
