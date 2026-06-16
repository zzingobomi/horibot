"""TrajectoryRunner 의 velocity streamer + ServoTcpCommand 동작 in-process 검증.

motion_taxonomy.md Phase 1 의 SpeedJ / SpeedTcp / ServoTcp 가 mock motor 없이도
*runner 단독* 으로 정상 동작하는지. e2e (zenoh service 호출) 는 별도 자리.

검증 자리:
- SpeedJ: set 호출 시 publish_cmd 가 호출됨 + target velocity 추종.
- SpeedJ timeout: 더 이상 갱신 X → jerk-limited 감속 후 자연 종료.
- SpeedTcp: tcp_twist_to_joint_vel 콜백 호출됨.
- ServoTcpCommand: 직접 publish_cmd 호출 (planner 우회).
"""

from __future__ import annotations

import time

import pytest

from modules.kinematics.motion_commands import ServoTcpCommand
from modules.kinematics.trajectory_runner import (
    TRAJ_DT,
    VELOCITY_INPUT_TIMEOUT,
    TrajectoryRunner,
)


def _make_runner(n: int = 5) -> tuple[TrajectoryRunner, list[list[float]], list[tuple]]:
    """단독 TrajectoryRunner. publish_cmd / state / IK 콜백 모두 stub.

    solve_ik stub = 받은 angle 그대로 반환 (cartesian path 자리 fallback).
    tcp_twist_to_joint_vel = linear[0] 을 J1 velocity 로 매핑 (단순 식별 검증).
    """
    cmds: list[list[float]] = []
    states: list[tuple] = []

    def pub_cmd(a: list[float]) -> None:
        cmds.append(list(a))

    def pub_state(s, p: float) -> None:
        states.append((s, p))

    def get_angles() -> list[float]:
        return [0.0] * n

    def twist_to_vel(
        lin: list[float], ang: list[float], joint: list[float], frame: str
    ) -> list[float] | None:
        # linear[0] = J1 vel, 나머지 0. frame 무관 (stub).
        return [lin[0]] + [0.0] * (n - 1)

    def solve_ik(pos, angles):
        # cartesian path stub — 받은 자세 그대로.
        return list(angles)

    runner = TrajectoryRunner(
        n_arm=n,
        joint_max_velocity=[2.0] * n,
        joint_max_acceleration=[10.0] * n,
        joint_max_jerk=[200.0] * n,
        cartesian_max_velocity=0.1,
        cartesian_max_acceleration=0.5,
        cartesian_max_jerk=5.0,
        release_profile=lambda: True,
        restore_profile=lambda: True,
        publish_cmd=pub_cmd,
        publish_state=pub_state,
        solve_ik=solve_ik,
        get_joint_angles=get_angles,
        tcp_twist_to_joint_vel=twist_to_vel,
    )
    return runner, cmds, states


def test_speed_j_publishes_and_tracks_target():
    """SpeedJ — target velocity 0.5 rad/s 추종 시 J1 position 단조 증가."""
    runner, cmds, _ = _make_runner()
    try:
        # 50ms 동안 0.5 rad/s 갱신 (50Hz × ~2 step)
        end = time.time() + 0.05
        while time.time() < end:
            runner.set_speed_joint([0.5, 0.0, 0.0, 0.0, 0.0])
            time.sleep(0.005)
        time.sleep(0.05)  # publish 더 받기

        assert len(cmds) >= 2, f"publish_cmd 가 충분히 호출 안 됨: {len(cmds)}"
        j1_first = cmds[0][0]
        j1_last = cmds[-1][0]
        assert j1_last > j1_first, (
            f"J1 position 증가 안 함: first={j1_first} last={j1_last}"
        )
        # 다른 joint 는 0 근처에 유지 (target=0)
        for cmd in cmds:
            for j in cmd[1:]:
                assert abs(j) < 0.01, f"비-target joint 가 0 이탈: {cmd}"
    finally:
        runner.stop()


def test_speed_j_timeout_decelerates_and_terminates():
    """SpeedJ — 갱신 끊김 → 100ms timeout 후 target=0 → idle grace 후 자연 종료."""
    runner, cmds, states = _make_runner()
    try:
        runner.set_speed_joint([0.5, 0.0, 0.0, 0.0, 0.0])
        time.sleep(0.05)  # 잠시 추종

        # 더 이상 set X. timeout (100ms) + idle grace (500ms) + 감속 margin.
        time.sleep(VELOCITY_INPUT_TIMEOUT + 1.0)

        assert not runner.is_running, "streamer 가 자연 종료 안 됨"
        # 종료 직전 cmd 가 변하지 않아야 (target=0 + 실 velocity=0)
        assert len(cmds) >= 4
        # 마지막 2 cmd 사이 J1 변동 < 1e-3 (이미 정지)
        delta = abs(cmds[-1][0] - cmds[-2][0])
        assert delta < 1e-3, f"종료 시점에 아직 움직임: delta={delta}"
    finally:
        runner.stop()


def test_speed_tcp_invokes_twist_callback():
    """SpeedTcp — tcp_twist_to_joint_vel stub (linear[0]→J1) 가 적용되어
    J1 position 이 linear vx 방향으로 움직임."""
    runner, cmds, _ = _make_runner()
    try:
        end = time.time() + 0.05
        while time.time() < end:
            runner.set_speed_tcp(
                linear=[0.5, 0.0, 0.0],
                angular=[0.0, 0.0, 0.0],
                frame="base",
            )
            time.sleep(0.005)
        time.sleep(0.05)

        assert len(cmds) >= 2
        # twist_to_vel stub 가 linear[0]=0.5 → J1 vel=0.5
        assert cmds[-1][0] > cmds[0][0], (
            f"SpeedTcp J1 추종 실패: {cmds[0][0]} → {cmds[-1][0]}"
        )
    finally:
        runner.stop()


def test_speed_j_dof_mismatch_raises():
    """SpeedJ velocities 길이 ≠ n_arm 이면 ValueError."""
    runner, _, _ = _make_runner(n=5)
    try:
        with pytest.raises(ValueError, match="length|길이"):
            runner.set_speed_joint([0.5, 0.0])  # 5축인데 2개만
    finally:
        runner.stop()


def test_servo_tcp_command_publishes_directly():
    """ServoTcpCommand — runner trajectory 없이 publish_cmd 직접 호출 (chase)."""
    cmds: list[list[float]] = []
    solve_calls: list[tuple] = []

    def pub_cmd(a: list[float]) -> None:
        cmds.append(list(a))

    def solve_servo(position, quaternion, angles):
        solve_calls.append((tuple(position), quaternion, tuple(angles)))
        # stub IK = current angles 에 position[0] 더한 J1.
        result = list(angles)
        result[0] += position[0]
        return result

    cmd = ServoTcpCommand(solve_servo, pub_cmd)
    runner, _, _ = _make_runner()
    try:
        req = {"data": {"position": [0.1, 0.0, 0.0], "quaternion": None}}
        cmd.execute(req, [0.0] * 5, [0.0, 0.0, 0.0], runner)

        assert len(solve_calls) == 1
        assert len(cmds) == 1
        # stub IK 가 J1 에 0.1 더함
        assert cmds[0][0] == pytest.approx(0.1, abs=1e-9)
    finally:
        runner.stop()


def test_servo_tcp_with_quaternion_passed_through():
    """ServoTcp 의 quaternion 필드 — solve 콜백에 그대로 전달 (6DOF)."""
    solve_calls: list[tuple] = []

    def pub_cmd(a):
        pass

    def solve_servo(position, quaternion, angles):
        solve_calls.append((tuple(position), quaternion, tuple(angles)))
        return list(angles)

    cmd = ServoTcpCommand(solve_servo, pub_cmd)
    runner, _, _ = _make_runner()
    try:
        req = {
            "data": {
                "position": [0.1, 0.0, 0.0],
                "quaternion": [0.0, 0.0, 0.0, 1.0],
            }
        }
        cmd.execute(req, [0.0] * 5, [0.0, 0.0, 0.0], runner)
        assert solve_calls[0][1] == (0.0, 0.0, 0.0, 1.0)
    finally:
        runner.stop()


def test_servo_tcp_ik_failure_raises():
    """ServoTcp — IK 가 None 반환 시 ValueError."""

    def pub_cmd(a):
        pass

    def solve_servo(position, quaternion, angles):
        return None  # IK 수렴 실패 시뮬

    cmd = ServoTcpCommand(solve_servo, pub_cmd)
    runner, _, _ = _make_runner()
    try:
        req = {"data": {"position": [0.1, 0.0, 0.0], "quaternion": None}}
        with pytest.raises(ValueError, match="IK"):
            cmd.execute(req, [0.0] * 5, [0.0, 0.0, 0.0], runner)
    finally:
        runner.stop()


def test_traj_dt_constant_sanity():
    """50Hz 보장 — TRAJ_DT 가 0.02 라고 가정한 다른 자리 (gamepad poll, server timeout) 안전."""
    assert TRAJ_DT == pytest.approx(0.02, abs=1e-6)
    assert VELOCITY_INPUT_TIMEOUT >= 5 * TRAJ_DT, (
        "timeout 이 너무 짧음 — 매 step 갱신 가능한 마진 필요"
    )
