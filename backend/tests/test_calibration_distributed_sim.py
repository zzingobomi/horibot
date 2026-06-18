"""분산(sim) E2E — calibration / motor / camera 가 *별도 프로세스* 일 때 전 파이프라인.

3 프로세스 (host_pc_sim = calibration+bridge, host_pi_motor_sim = mock_motor+motion,
host_pi_camera_sim = mock_camera) 를 localhost 에 띄우고 Zenoh 멀티캐스트로 서로
발견. single-process headless e2e (test_calibration_e2e) 와 동일 캘 흐름을 *분산
토폴로지* 위에서 검증 — Zenoh 가 motor cmd / camera frame / calib service 를 프로세스
경계 넘어 투명 라우팅하는지 (CLAUDE.md 분산 토폴로지) + 내 캘 변경이 분산에서 무회귀.

CALIB_SIM_BOARD=1 은 camera 프로세스(mock_camera)에 줘서 ChArUco eye-in-hand 렌더.

NOTE: zenoh 멀티캐스트 + 3 heavy 프로세스 (pc_sim 은 torch/detector 로드). ~3분.
CI 멀티캐스트 X 또는 저사양이면 skip.
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

GET_TCP = f"horibot/{ROBOT_ID}/motion/srv/get_tcp"
MOVE_J = f"horibot/{ROBOT_ID}/motion/srv/move_j"
INTR_START = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/start"
INTR_CAPTURE = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/capture"
INTR_SAVE = f"horibot/{ROBOT_ID}/calib/srv/intrinsic/save"
HE_START = f"horibot/{ROBOT_ID}/calib/srv/handeye/start"
HE_CAPTURE = f"horibot/{ROBOT_ID}/calib/srv/handeye/capture"
HE_COMPUTE = f"horibot/{ROBOT_ID}/calib/srv/handeye/compute"
HE_COMMIT = f"horibot/{ROBOT_ID}/calib/srv/handeye/commit"
HE_RESET = f"horibot/{ROBOT_ID}/calib/srv/handeye/reset"


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


def _spawn(host: str, extra_env: dict[str, str] | None = None) -> subprocess.Popen:
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    if extra_env:
        env.update(extra_env)
    return subprocess.Popen(
        ["uv", "run", "--active", "python", "main.py", "--host", host],
        cwd=str(BACKEND_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
    )


@pytest.fixture(scope="module")
def distributed_backend() -> Iterator[None]:
    """3 프로세스 분산 토폴로지 (motor/camera 먼저 — 가벼움, pc 마지막 — 무거움)."""
    _kill_stale()
    procs = [
        _spawn("pi_motor_sim"),
        _spawn("pi_camera_sim", {"CALIB_SIM_BOARD": "1"}),
        _spawn("pc_sim"),
    ]
    deadline = time.time() + 150
    ready = False
    while time.time() < deadline:
        for p in procs:
            if p.poll() is not None:
                for q in procs:
                    q.kill()
                raise RuntimeError(f"분산 프로세스 조기 종료 code={p.returncode}")
        try:
            if requests.get(f"{BRIDGE_URL}/openapi.json", timeout=1).status_code == 200:
                ready = True
                break
        except Exception:
            pass
        time.sleep(1)
    if not ready:
        for p in procs:
            p.kill()
        raise RuntimeError("분산 backend boot timeout")
    time.sleep(5)  # peer discovery 안정 (3 peer 멀티캐스트 scout)
    yield
    for p in procs:
        p.terminate()
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    _kill_stale()


def _call(session, key: str, data: dict, timeout: float = 10.0) -> dict:
    payload = json.dumps({"timestamp": time.time(), "data": data}).encode()
    for r in session.get(key, payload=payload, timeout=timeout):
        if r.ok is not None:
            return json.loads(r.ok.payload.to_bytes())
        return {"success": False, "message": "err", "data": None}
    return {"success": False, "message": "no reply", "data": None}


@pytest.fixture(scope="module")
def zsession(distributed_backend) -> Iterator[Any]:
    session = zenoh.open(zenoh.Config())
    time.sleep(2)
    # motion (pi_motor_sim) 가 프로세스 경계 넘어 reachable 할 때까지.
    deadline = time.time() + 20
    while time.time() < deadline:
        if _call(session, GET_TCP, {}, timeout=2.0).get("success"):
            break
        time.sleep(0.3)
    else:
        session.close()
        raise RuntimeError("분산: motion (pi_motor_sim) 미발견")
    yield session
    session.close()


def _move(session, degs: list[float]) -> None:
    joints = [{"id": i + 1, "degree": float(d)} for i, d in enumerate(degs)]
    _call(session, MOVE_J, {"joints": joints})
    time.sleep(2.2)


def test_distributed_calibration_pipeline(zsession):
    """3-프로세스 분산: 모터/카메라/캘이 별 프로세스인데 캘 전 파이프라인 동작."""
    _call(zsession, HE_RESET, {})

    # intrinsic (camera = pi_camera_sim, calibration = pc_sim — 프로세스 경계 넘음)
    assert _call(zsession, INTR_START, {}).get("success")
    intr = 0
    for degs in SIM_CALIB_POSES_DEG:
        _move(zsession, degs)
        r = _call(zsession, INTR_CAPTURE, {})
        if r.get("success") and r.get("data", {}).get("detected"):
            intr += 1
    assert intr >= 5, f"분산 intrinsic 검출 부족: {intr}"
    assert _call(zsession, INTR_SAVE, {}).get("success"), "분산 intrinsic save 실패"

    # handeye
    assert _call(zsession, HE_START, {}).get("success")
    he = 0
    for degs in SIM_CALIB_POSES_DEG:
        _move(zsession, degs)
        r = _call(zsession, HE_CAPTURE, {})
        if r.get("success") and r.get("data", {}).get("detected"):
            he += 1
    assert he >= 4, f"분산 handeye 검출 부족: {he}"

    comp = _call(zsession, HE_COMPUTE, {"mode": "physical_sag"}, timeout=90.0)
    assert comp.get("success"), f"분산 compute 실패: {comp}"
    assert comp["data"].get("param_observability") is not None

    assert _call(zsession, HE_COMMIT, {}, timeout=15.0).get("success"), "분산 commit 실패"
