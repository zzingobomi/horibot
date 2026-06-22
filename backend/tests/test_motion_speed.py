"""TrajectoryRunner + Servo / Jog command 동작 in-process 검증.

motion_taxonomy.md 4 계층 자리 (Move / Servo / Jog / Task) 의 Servo / Jog 자리.

검증 자리:
- ServoTcpCommand: 절대 target → 직접 IK + publish (planner 우회).
- ServoJCommand: 절대 joint target → 직접 publish (IK 불요).
- JogJCommand: velocity input + backend latched ref + dt 적분.
- JogTcpCommand: twist input + backend latched ref + SE(3) 적분 + IK + publish.
- IDLE_RESET_S 후 fresh latch 자리 (인코더 - ref 누적 drift 차단).
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from modules.kinematics.motion_commands import (
    JogJCommand,
    JogTcpCommand,
    ServoJCommand,
    ServoTcpCommand,
)
from modules.kinematics.trajectory_runner import (
    TRAJ_DT,
    TrajectoryRunner,
)


def _make_runner(n: int = 5) -> tuple[TrajectoryRunner, list[list[float]], list[tuple]]:
    """단독 TrajectoryRunner. publish_cmd / state / IK 콜백 모두 stub."""
    cmds: list[list[float]] = []
    states: list[tuple] = []

    def pub_cmd(a: list[float]) -> None:
        cmds.append(list(a))

    def pub_state(s, p: float) -> None:
        states.append((s, p))

    def get_angles() -> list[float]:
        return [0.0] * n

    def solve_ik(pos, angles):
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
    )
    return runner, cmds, states


# ─── ServoTcp / ServoJ — 절대 target chase ──────────────────────────


def test_servo_tcp_command_publishes_directly():
    """ServoTcpCommand — 절대 target → planner 우회 IK + publish."""
    cmds: list[list[float]] = []
    solve_calls: list[tuple] = []

    def pub_cmd(a: list[float]) -> None:
        cmds.append(list(a))

    def solve_servo(position, quaternion, angles):
        solve_calls.append((tuple(position), quaternion, tuple(angles)))
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
        assert cmds[0][0] == pytest.approx(0.1, abs=1e-9)
    finally:
        runner.stop()


def test_servo_tcp_with_quaternion_passed_through():
    """ServoTcp — quaternion 자리 6DOF 자리 그대로 IK 콜백 전달."""
    solve_calls: list[tuple] = []

    def solve_servo(position, quaternion, angles):
        solve_calls.append((tuple(position), quaternion, tuple(angles)))
        return list(angles)

    cmd = ServoTcpCommand(solve_servo, lambda a: None)
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
    """ServoTcp — IK None 자리 ValueError."""

    def solve_servo(position, quaternion, angles):
        return None

    cmd = ServoTcpCommand(solve_servo, lambda a: None)
    runner, _, _ = _make_runner()
    try:
        req = {"data": {"position": [0.1, 0.0, 0.0], "quaternion": None}}
        with pytest.raises(ValueError, match="IK"):
            cmd.execute(req, [0.0] * 5, [0.0, 0.0, 0.0], runner)
    finally:
        runner.stop()


def test_servo_j_command_publishes_directly():
    """ServoJCommand — 절대 joint target 직접 publish (IK 불요)."""
    cmds: list[list[float]] = []

    def pub_cmd(a):
        cmds.append(list(a))

    cmd = ServoJCommand(pub_cmd, n_arm=6)
    runner, _, _ = _make_runner(n=6)
    try:
        req = {"data": {"positions": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]}}
        cmd.execute(req, [], [], runner)
        assert len(cmds) == 1
        assert cmds[0] == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    finally:
        runner.stop()


def test_servo_j_command_validates_dof():
    """ServoJCommand — positions 길이 != arm dof 자리 validate 실패."""
    cmd = ServoJCommand(lambda a: None, n_arm=6)
    assert cmd.validate({"data": {"positions": [0.0] * 5}}) is not None
    assert cmd.validate({"data": {"positions": [0.0] * 6}}) is None
    assert cmd.validate({"data": {}}) is not None


# ─── JogJ — velocity input + backend latch + 적분 ──────────────────


def test_jog_j_first_publish_latches_from_cache():
    """JogJCommand 첫 publish 자리 joint_cache 의 현재 URDF rad 으로 latch.

    velocity 입력 자리 영향 X (첫 publish 자리 fresh latch = 현재 위치).
    """
    cmds: list[list[float]] = []

    def pub_cmd(a):
        cmds.append(list(a))

    current_state = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    cmd = JogJCommand(pub_cmd, lambda: list(current_state), n_arm=6)
    runner, _, _ = _make_runner(n=6)
    try:
        cmd.execute({"data": {"velocities": [1.0] * 6}}, [], [], runner)
        assert len(cmds) == 1
        assert cmds[0] == pytest.approx(current_state)
    finally:
        runner.stop()


def test_jog_j_command_validates_dof():
    """JogJCommand — velocities 길이 검증."""
    cmd = JogJCommand(lambda a: None, lambda: [0.0] * 6, n_arm=6)
    assert cmd.validate({"data": {"velocities": [0.0] * 5}}) is not None
    assert cmd.validate({"data": {"velocities": [0.0] * 6}}) is None
    assert cmd.validate({"data": {}}) is not None


def test_jog_j_streaming_integrates_velocity():
    """50Hz publish loop — backend 실 dt 적분 → J1 단조 증가."""
    cmds: list[list[float]] = []

    def pub_cmd(a):
        cmds.append(list(a))

    cmd = JogJCommand(pub_cmd, lambda: [0.0] * 6, n_arm=6)
    runner, _, _ = _make_runner(n=6)
    try:
        for _ in range(10):
            cmd.execute(
                {"data": {"velocities": [0.5, 0, 0, 0, 0, 0]}}, [], [], runner
            )
            time.sleep(0.02)
        assert len(cmds) == 10
        assert cmds[0] == pytest.approx([0.0] * 6, abs=1e-9)
        for i in range(9):
            step = cmds[i + 1][0] - cmds[i][0]
            assert step > 0, f"cycle {i}→{i+1} step {step} not increasing"
            assert 0.005 < step < 0.04, f"cycle {i}→{i+1} step {step} out of range"
        for c in cmds:
            assert c[1:] == pytest.approx([0.0] * 5, abs=1e-9)
    finally:
        runner.stop()


def test_jog_j_idle_reset_fresh_latches():
    """IDLE_RESET_S 보다 publish 끊긴 후 → joint_cache fresh latch."""
    cmds: list[list[float]] = []
    cache_state = {"value": [0.0] * 5}

    cmd = JogJCommand(
        lambda a: cmds.append(list(a)),
        lambda: list(cache_state["value"]),
        n_arm=5,
    )
    runner, _, _ = _make_runner(n=5)
    try:
        for _ in range(5):
            cmd.execute(
                {"data": {"velocities": [0.5, 0, 0, 0, 0]}}, [], [], runner
            )
            time.sleep(0.02)
        last_first_session = cmds[-1][0]
        cache_state["value"] = [last_first_session, 0, 0, 0, 0]

        time.sleep(0.3)  # IDLE_RESET_S=0.2s 초과

        cmd.execute({"data": {"velocities": [0.5] + [0] * 4}}, [], [], runner)
        # fresh latch — cache_state 값 그대로.
        assert cmds[-1][0] == pytest.approx(last_first_session, abs=1e-9)
    finally:
        runner.stop()


def test_jog_j_raises_when_cache_empty():
    """JogJ — joint_cache 비어있으면 fresh latch 실패."""
    cmd = JogJCommand(lambda a: None, lambda: None, n_arm=5)
    runner, _, _ = _make_runner(n=5)
    try:
        with pytest.raises(ValueError, match="joint_cache"):
            cmd.execute(
                {"data": {"velocities": [0.0] * 5}}, [], [], runner
            )
    finally:
        runner.stop()


# ─── JogTcp — twist input + backend latch + SE(3) 적분 + IK ────────


def test_jog_tcp_first_publish_latches_from_fk():
    """JogTcpCommand 첫 publish 자리 fk → fresh latch + IK + publish."""
    cmds: list[list[float]] = []
    solve_calls: list[tuple] = []
    fk_pos = np.array([0.3, 0.0, 0.4])
    fk_quat = np.array([0.0, 0.0, 0.0, 1.0])

    def fk(angles):
        return fk_pos, fk_quat

    def solve_servo(position, quaternion, angles):
        solve_calls.append((tuple(position), tuple(quaternion), tuple(angles)))
        return list(angles)

    cmd = JogTcpCommand(
        solve_servo,
        lambda a: cmds.append(list(a)),
        lambda: [0.0] * 6,
        fk,
    )
    runner, _, _ = _make_runner(n=6)
    try:
        req = {"data": {"linear": [0.0] * 3, "angular": [0.0] * 3, "frame": "base"}}
        cmd.execute(req, [], [], runner)
        assert len(solve_calls) == 1
        # fresh latch + IK = 현재 자세 (= fk_pos, fk_quat).
        assert solve_calls[0][0] == pytest.approx((0.3, 0.0, 0.4))
        assert solve_calls[0][1] == pytest.approx((0.0, 0.0, 0.0, 1.0))
        assert len(cmds) == 1
    finally:
        runner.stop()


def test_jog_tcp_linear_base_frame_integrates():
    """JogTcp linear (base frame) — 실 dt 자리 ref pos 적분, IK 호출."""
    cmds: list[list[float]] = []
    solve_calls: list[tuple] = []

    def solve_servo(position, quaternion, angles):
        solve_calls.append(tuple(position))
        return list(angles)

    cmd = JogTcpCommand(
        solve_servo,
        lambda a: cmds.append(list(a)),
        lambda: [0.0] * 6,
        lambda a: (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    )
    runner, _, _ = _make_runner(n=6)
    try:
        # 10 cycle × Z velocity 0.05 m/s × ~20ms → Z position 단조 증가
        for _ in range(10):
            cmd.execute(
                {
                    "data": {
                        "linear": [0.0, 0.0, 0.05],
                        "angular": [0.0] * 3,
                        "frame": "base",
                    }
                },
                [], [], runner,
            )
            time.sleep(0.02)
        # 첫 publish 자리 (fresh latch) z=0, 이후 적분.
        assert solve_calls[0][2] == pytest.approx(0.0)
        for i in range(9):
            step = solve_calls[i + 1][2] - solve_calls[i][2]
            assert step > 0
            assert 0.0003 < step < 0.003  # 0.05 m/s × ~20ms = 1mm
    finally:
        runner.stop()


def test_jog_tcp_validates_input():
    """JogTcp — linear/angular 누락 또는 frame 잘못 자리 reject."""
    cmd = JogTcpCommand(
        lambda *a: [0.0],
        lambda a: None,
        lambda: [0.0] * 6,
        lambda a: (np.zeros(3), np.array([0, 0, 0, 1])),
    )
    assert cmd.validate({"data": {}}) is not None
    assert cmd.validate({"data": {"linear": [0, 0, 0]}}) is not None
    assert cmd.validate(
        {"data": {"linear": [0, 0, 0], "angular": [0, 0, 0], "frame": "world"}}
    ) is not None
    assert cmd.validate(
        {"data": {"linear": [0, 0, 0], "angular": [0, 0, 0]}}
    ) is None  # frame default base


def test_jog_tcp_ik_failure_rolls_back_ref():
    """JogTcp IK None 자리 ref 자리 *적분 전 값* 유지 (reach 한계 누적 X)."""
    fk_pos = np.array([0.0, 0.0, 0.0])
    fk_quat = np.array([0.0, 0.0, 0.0, 1.0])

    # 첫 호출은 IK success (fresh latch), 둘째 호출은 IK None.
    call_count = [0]

    def solve_servo(position, quaternion, angles):
        call_count[0] += 1
        if call_count[0] == 1:
            return list(angles)
        return None

    cmd = JogTcpCommand(
        solve_servo,
        lambda a: None,
        lambda: [0.0] * 6,
        lambda a: (fk_pos, fk_quat),
    )
    runner, _, _ = _make_runner(n=6)
    try:
        # 첫 publish — fresh latch.
        cmd.execute(
            {"data": {"linear": [0.05, 0, 0], "angular": [0, 0, 0], "frame": "base"}},
            [], [], runner,
        )
        ref_before = cmd._last_pos.copy()
        time.sleep(0.02)

        # 둘째 publish — IK 실패 → rollback.
        with pytest.raises(ValueError, match="IK"):
            cmd.execute(
                {"data": {"linear": [0.05, 0, 0], "angular": [0, 0, 0], "frame": "base"}},
                [], [], runner,
            )
        # ref 자리 적분 전 값 유지.
        np.testing.assert_allclose(cmd._last_pos, ref_before, atol=1e-9)
    finally:
        runner.stop()


def test_traj_dt_constant_sanity():
    """50Hz 보장 — TRAJ_DT 가 0.02 라고 가정한 다른 자리 (gamepad poll 등) 안전."""
    assert TRAJ_DT == pytest.approx(0.02, abs=1e-6)
