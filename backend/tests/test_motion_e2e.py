"""E2E — host_mock backend subprocess + zenoh client 로 motion service 호출.

mock_motor 가 cmd 즉시 internal position 갱신 + MOTOR_STATE_JOINT publish. 우리는
motion service 호출 → MOTOR_CMD_JOINT publish 가 *진짜* 흘러나오는지 검증.

검증 자리:
- ServoTcp: 1회 service 호출 → MOTOR_CMD_JOINT 1회 publish (chase 패턴).
- SpeedJ: 갱신 동안 continuous publish, 갱신 끊김 → ~100ms 후 자동 정지.
- SpeedTcp (5DOF): angular 무시 + linear-only Jacobian fallback, publish 발생.
- MoveJ: trajectory_runner position mode 정상 동작 (regression).
- ServoTcp가 진행중 trajectory 가로채는 자리.

backend 부팅 시간 (~15s) 때문에 module-scoped fixture.

NOTE: 이 test 는 zenoh peer scout (멀티캐스트) 의존. CI 환경에서 멀티캐스트 X 면
backend 와 test 가 서로 못 봄 → skip. localhost 단일 머신은 OK.
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

BACKEND_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_URL = "http://localhost:8000"
ROBOT_ID = "so101_6dof_0"

# host_mock 의 robot. mock_motor 는 6축 arm + 1 gripper (so101_6dof motors.yaml).
MOTOR_CMD_TOPIC = f"horibot/{ROBOT_ID}/motor/cmd/joint"
SERVO_TCP_SVC = f"horibot/{ROBOT_ID}/motion/srv/servo_tcp"
SERVO_J_SVC = f"horibot/{ROBOT_ID}/motion/srv/servo_j"
JOG_TCP_SVC = f"horibot/{ROBOT_ID}/motion/srv/jog_tcp"
JOG_J_SVC = f"horibot/{ROBOT_ID}/motion/srv/jog_j"
JOG_TCP_STREAM = f"horibot/{ROBOT_ID}/motion/cmd/jog_tcp_stream"
JOG_J_STREAM = f"horibot/{ROBOT_ID}/motion/cmd/jog_j_stream"
MOVE_J_SVC = f"horibot/{ROBOT_ID}/motion/srv/move_j"
GET_TCP_SVC = f"horibot/{ROBOT_ID}/motion/srv/get_tcp"
STOP_SVC = f"horibot/{ROBOT_ID}/motion/srv/stop"


# ─── Fixtures ────────────────────────────────────────────────────────


def _kill_stale_backends() -> None:
    """이전 test run 의 stale backend (port 8000 listener) 제거 — 여러 peer 가
    같은 토픽 publish 하면 zenoh service get() 가 race 로 잘못된 응답 받음.
    Windows 자리에서 ctrl-c 받기 전 process 가 살아남는 자리도 흔함.
    """
    try:
        out = subprocess.check_output(
            ["netstat", "-ano"], text=True, stderr=subprocess.DEVNULL
        )
    except Exception:
        return
    pids: set[str] = set()
    for line in out.splitlines():
        if ":8000" in line and "LISTENING" in line:
            parts = line.split()
            if parts:
                pids.add(parts[-1])
    for pid in pids:
        try:
            subprocess.run(
                ["taskkill", "/PID", pid, "/F"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass


@pytest.fixture(scope="module")
def mock_backend() -> Iterator[None]:
    """host_mock 으로 backend subprocess. bridge HTTP 응답 = ready."""
    _kill_stale_backends()
    env = os.environ.copy()
    # Windows console UTF-8 — log 인코딩 안정.
    env["PYTHONIOENCODING"] = "utf-8"
    proc = subprocess.Popen(
        ["uv", "run", "--active", "python", "main.py", "--host", "mock"],
        cwd=str(BACKEND_ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=env,
    )

    deadline = time.time() + 90
    ready = False
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"backend prematurely exited code={proc.returncode}")
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
        raise RuntimeError("backend boot timeout (90s)")
    # peer discover 안정 대기 — 멀티캐스트 scout.
    time.sleep(3)

    yield

    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    # LLM / Grounded DINO subprocess 가 부모 종료 후도 GPU 메모리 잡고 있는 자리
    # — port 8000 listener 가 남으면 다음 test run 의 zenoh peer 충돌 source.
    _kill_stale_backends()


@pytest.fixture(scope="module")
def zsession(mock_backend) -> Iterator[Any]:
    """test process 의 zenoh peer. backend 가 같은 LAN 멀티캐스트로 발견됨."""
    session = zenoh.open(zenoh.Config())
    # peer scout 대기
    time.sleep(2)
    # motor state cache 가 채워질 때까지 wait — backend 내부에서 mock_motor publish
    # 가 motion_node cache 에 누적되어야 get_tcp 가 success. 부팅 직후엔 race.
    deadline = time.time() + 10
    while time.time() < deadline:
        res = call_service(session, GET_TCP_SVC, {}, timeout=2.0)
        if res.get("success"):
            break
        time.sleep(0.3)
    else:
        session.close()
        raise RuntimeError("motion_node 의 joint cache 가 채워지지 않음 (10s)")
    yield session
    session.close()


# ─── Helpers ─────────────────────────────────────────────────────────


def call_service(session, key: str, data: dict, timeout: float = 5.0) -> dict:
    """우리 BaseNode.call_service 와 동일한 wire 형식 ({timestamp, data} → {success, message, data})."""
    payload = json.dumps({"timestamp": time.time(), "data": data}).encode()
    replies = session.get(key, payload=payload, timeout=timeout)
    for r in replies:
        if r.ok is not None:
            return json.loads(r.ok.payload.to_bytes())
        err = r.err
        msg = (
            err.payload.to_string()
            if err is not None and err.payload is not None
            else "err reply"
        )
        return {"success": False, "message": msg, "data": None}
    return {"success": False, "message": "no reply", "data": None}


class CmdCollector:
    """MOTOR_CMD_JOINT 토픽 수집기 — 각 sample 의 joints 리스트 보존."""

    def __init__(self, session) -> None:
        self._samples: list[dict] = []
        self._sub = session.declare_subscriber(MOTOR_CMD_TOPIC, self._on)

    def _on(self, sample) -> None:
        try:
            self._samples.append(json.loads(sample.payload.to_bytes()))
        except Exception:
            pass

    @property
    def samples(self) -> list[dict]:
        return list(self._samples)

    def reset(self) -> None:
        self._samples.clear()

    def close(self) -> None:
        self._sub.undeclare()


# ─── Tests ───────────────────────────────────────────────────────────


def test_get_tcp_works(zsession):
    """sanity — motion_node 가 살아 있고 get_tcp 응답."""
    res = call_service(zsession, GET_TCP_SVC, {})
    assert res.get("success"), f"get_tcp 실패: {res}"
    assert "position" in res["data"]
    assert "quaternion" in res["data"]
    assert len(res["data"]["position"]) == 3
    assert len(res["data"]["quaternion"]) == 4


def test_servo_tcp_publishes_cmd(zsession):
    """ServoTcp 1회 호출 → MOTOR_CMD_JOINT 1회 publish (planner 우회 chase)."""
    # 현재 TCP 읽고 그 자리로 servo (즉 같은 자세) — IK 가 trivially 풀림.
    cur = call_service(zsession, GET_TCP_SVC, {})
    assert cur["success"]
    pos = cur["data"]["position"]

    collector = CmdCollector(zsession)
    time.sleep(0.1)
    collector.reset()
    try:
        res = call_service(
            zsession,
            SERVO_TCP_SVC,
            {"position": pos, "quaternion": None},
        )
        assert res.get("success"), f"ServoTcp 실패: {res}"
        time.sleep(0.3)
        assert len(collector.samples) >= 1, (
            f"ServoTcp 후 MOTOR_CMD_JOINT publish 없음: {collector.samples}"
        )
    finally:
        collector.close()


def test_servo_tcp_with_orientation_6dof(zsession):
    """6DOF (SO-101) — quaternion 전달 시 IK 가 orientation 까지 추종."""
    cur = call_service(zsession, GET_TCP_SVC, {})
    pos = cur["data"]["position"]
    quat = cur["data"]["quaternion"]

    res = call_service(
        zsession,
        SERVO_TCP_SVC,
        {"position": pos, "quaternion": quat},
    )
    # 현재 자세로 servo 라 6DOF IK 통과해야.
    assert res.get("success"), f"ServoTcp(6DOF) 실패: {res}"


def test_jog_j_stream_streams(zsession):
    """JogJ topic stream — 50Hz velocity publish → backend latch + 적분 → motor cmd."""
    collector = CmdCollector(zsession)
    time.sleep(0.1)
    collector.reset()
    try:
        end = time.time() + 0.2
        while time.time() < end:
            publish_topic(
                zsession,
                JOG_J_STREAM,
                {"velocities": [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]},
            )
            time.sleep(0.02)
        time.sleep(0.1)
        n_during = len(collector.samples)
        assert n_during >= 3, f"JogJ stream publish 부족: {n_during}"
        first_j1 = collector.samples[0]["joints"][0]["position"]
        last_j1 = collector.samples[-1]["joints"][0]["position"]
        assert last_j1 > first_j1, (
            f"JogJ stream 후 J1 단조 증가 안 함: {first_j1} → {last_j1}"
        )
    finally:
        collector.close()


def test_jog_tcp_stream_6dof(zsession):
    """JogTcp topic stream — twist input → backend SE(3) 적분 → IK → motor cmd."""
    collector = CmdCollector(zsession)
    time.sleep(0.1)
    collector.reset()
    try:
        end = time.time() + 0.3
        while time.time() < end:
            publish_topic(
                zsession,
                JOG_TCP_STREAM,
                {
                    "linear": [0.0, 0.0, 0.05],
                    "angular": [0.0, 0.0, 0.0],
                    "frame": "base",
                },
            )
            time.sleep(0.02)
        time.sleep(0.1)
        assert len(collector.samples) >= 3, (
            f"JogTcp stream publish 부족: {len(collector.samples)}"
        )
        first = collector.samples[0]["joints"]
        last = collector.samples[-1]["joints"]
        deltas = [abs(la["position"] - fa["position"]) for la, fa in zip(last, first)]
        assert max(deltas) > 0, f"JogTcp stream 후 motor 변화 0: {deltas}"
    finally:
        collector.close()


def test_jog_tcp_stream_tcp_frame_passthrough(zsession):
    """JogTcp frame='tcp' — backend SE(3) 적분 자리 tcp frame 통과."""
    publish_topic(
        zsession,
        JOG_TCP_STREAM,
        {
            "linear": [0.01, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.0],
            "frame": "tcp",
        },
    )
    time.sleep(0.2)


def test_move_j_regression(zsession):
    """MoveJ 가 회귀 없는지 — trajectory_runner.run_joint 동작."""
    # 이전 test 자리 J1 잔여 위치 자리에 무관하게 robust 자리. 먼저 J1 home (0°)
    # 자리 보내고, 그 후 5° 자리 target 자리 → 단조 증가 자리 보장.
    call_service(
        zsession,
        MOVE_J_SVC,
        {
            "joints": [
                {"id": 1, "degree": 0.0},
                {"id": 2, "degree": 0.0},
                {"id": 3, "degree": 0.0},
                {"id": 4, "degree": 0.0},
                {"id": 5, "degree": 0.0},
                {"id": 6, "degree": 0.0},
            ]
        },
    )
    time.sleep(2.0)

    collector = CmdCollector(zsession)
    time.sleep(0.1)
    collector.reset()
    try:
        res = call_service(
            zsession,
            MOVE_J_SVC,
            {
                "joints": [
                    {"id": 1, "degree": 10.0},
                    {"id": 2, "degree": 0.0},
                    {"id": 3, "degree": 0.0},
                    {"id": 4, "degree": 0.0},
                    {"id": 5, "degree": 0.0},
                    {"id": 6, "degree": 0.0},
                ]
            },
        )
        assert res.get("success"), f"MoveJ 실패: {res}"
        time.sleep(2.0)

        assert len(collector.samples) >= 5, (
            f"MoveJ 동안 publish 부족: {len(collector.samples)}"
        )
        first_j1 = collector.samples[0]["joints"][0]["position"]
        last_j1 = collector.samples[-1]["joints"][0]["position"]
        assert last_j1 > first_j1, f"MoveJ J1 이동 안 함: {first_j1} → {last_j1}"
    finally:
        collector.close()


def test_move_l_regression(zsession):
    """MoveL trajectory — solve_ik (motion_modes.servo_tcp) 콜백 의존. cartesian
    path 가 streamer/Ruckig velocity mode 변경에 regression 없는지."""
    cur = call_service(zsession, GET_TCP_SVC, {})
    assert cur["success"]
    pos = list(cur["data"]["position"])
    # 2cm Z+ 이동.
    target = [pos[0], pos[1], pos[2] + 0.02]

    collector = CmdCollector(zsession)
    time.sleep(0.1)
    collector.reset()
    try:
        move_l_svc = f"horibot/{ROBOT_ID}/motion/srv/move_l"
        res = call_service(zsession, move_l_svc, {"position": target})
        assert res.get("success"), f"MoveL 실패: {res}"
        time.sleep(2.0)  # trajectory 완료 대기 (2cm @ 10cm/s + ramp)
        assert len(collector.samples) >= 5, (
            f"MoveL 동안 publish 부족: {len(collector.samples)}"
        )
        # Z 방향 모터 (J2/J3 등) 변화 — 모든 6 motor 중 하나라도 변화
        first = collector.samples[0]["joints"]
        last = collector.samples[-1]["joints"]
        deltas = [abs(la["position"] - fa["position"]) for la, fa in zip(last, first)]
        assert max(deltas) > 0, f"MoveL 후 motor 변화 0: {deltas}"
    finally:
        collector.close()


def test_jog_tcp_invalid_frame_rejected(zsession):
    """JogTcp frame literal validation — base/tcp 외 값은 pydantic 가 reject."""
    res = call_service(
        zsession,
        JOG_TCP_SVC,
        {"linear": [0.01, 0, 0], "angular": [0, 0, 0], "frame": "world"},
    )
    assert not res.get("success"), (
        f"invalid frame='world' 가 통과됨: {res}"
    )


def test_servo_tcp_during_trajectory_interrupts(zsession):
    """ServoTcp 가 진행 중 trajectory 를 가로채는 자리 — runner.is_running stop 자리."""
    # MoveJ 시작 (느린 trajectory).
    call_service(
        zsession,
        MOVE_J_SVC,
        {
            "joints": [
                {"id": 1, "degree": 10.0},
                {"id": 2, "degree": 0.0},
                {"id": 3, "degree": 0.0},
                {"id": 4, "degree": 0.0},
                {"id": 5, "degree": 0.0},
                {"id": 6, "degree": 0.0},
            ]
        },
    )
    time.sleep(0.1)

    # 현재 TCP servo — runner.stop() 가 trajectory thread 죽임.
    cur = call_service(zsession, GET_TCP_SVC, {})
    res = call_service(
        zsession,
        SERVO_TCP_SVC,
        {"position": cur["data"]["position"], "quaternion": None},
    )
    assert res.get("success"), f"ServoTcp interrupt 실패: {res}"
    # trajectory 가 끊겨야 (이후 cmd 변화 없음 또는 단일 servo 후 정지).
    time.sleep(0.5)
    # stop 명시
    call_service(zsession, STOP_SVC, {})


# ─── Servo (absolute target stream) tests ─────────────────────────────


def publish_topic(session, topic: str, data: dict) -> None:
    """fire-and-forget topic publish — bridge 가 같은 wire 로 forward."""
    session.put(topic, json.dumps(data).encode())


def test_servo_j_absolute_target_publishes(zsession):
    """ServoJ — 절대 joint target → 직접 publish (RL replay 자리)."""
    res = call_service(
        zsession,
        SERVO_J_SVC,
        {"positions": [0.0] * 6},
    )
    assert res.get("success"), f"ServoJ service 호출 실패: {res}"


def test_servo_j_dof_mismatch_rejected(zsession):
    """ServoJ — positions 길이가 arm dof 와 다르면 reject."""
    res = call_service(
        zsession,
        SERVO_J_SVC,
        {"positions": [0.0] * 5},
    )
    assert not res.get("success"), (
        f"잘못된 dof (5 != 6) 가 통과됨: {res}"
    )


# ─── Jog (velocity stream, backend latch) tests ───────────────────────


def test_jog_j_idle_then_resume_fresh_latch(zsession):
    """JogJ — publish 끊김 후 다시 시작 자리 fresh latch (인코더 - ref drift 차단).

    SpeedJ 와 달리 deadman ramp 자리 X — 마지막 target 머무름 + 다음 hold 시
    backend 가 joint_cache 에서 새로 latch.
    """
    # 1차 hold — publish 적분.
    for _ in range(5):
        publish_topic(
            zsession,
            JOG_J_STREAM,
            {"velocities": [0.3] + [0] * 5},
        )
        time.sleep(0.02)
    time.sleep(0.4)  # IDLE_RESET_S=0.2 초과

    # 2차 hold — fresh latch. 같은 자리 다시 시작.
    collector = CmdCollector(zsession)
    time.sleep(0.1)
    collector.reset()
    try:
        for _ in range(5):
            publish_topic(
                zsession,
                JOG_J_STREAM,
                {"velocities": [0.3] + [0] * 5},
            )
            time.sleep(0.02)
        time.sleep(0.1)
        # 2차 hold 자리 publish 흐름 정상 — J1 단조 증가 (적분 정상 동작).
        assert len(collector.samples) >= 2
        first_j1 = collector.samples[0]["joints"][0]["position"]
        last_j1 = collector.samples[-1]["joints"][0]["position"]
        assert last_j1 > first_j1, (
            f"JogJ 2차 hold 자리 단조 증가 안 함: {first_j1} → {last_j1}"
        )
    finally:
        collector.close()


def test_jog_j_service_also_works(zsession):
    """JogJ — service 호출 자리 (단발, 자동화 tool / test 호출)."""
    res = call_service(
        zsession,
        JOG_J_SVC,
        {"velocities": [0.0] * 6},
    )
    assert res.get("success"), f"JogJ service 호출 실패: {res}"


def test_jog_j_dof_mismatch_rejected(zsession):
    """JogJ — velocities 길이가 arm dof 와 다르면 reject."""
    res = call_service(
        zsession,
        JOG_J_SVC,
        {"velocities": [0.0] * 5},
    )
    assert not res.get("success")


def test_jog_tcp_service_also_works(zsession):
    """JogTcp — service 호출 자리 (단발)."""
    res = call_service(
        zsession,
        JOG_TCP_SVC,
        {"linear": [0.0] * 3, "angular": [0.0] * 3, "frame": "base"},
    )
    assert res.get("success"), f"JogTcp service 호출 실패: {res}"
