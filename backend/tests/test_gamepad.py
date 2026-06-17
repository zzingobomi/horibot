"""GamepadNode 매핑 / capability gate / deadman 단위 검증.

zenoh session 만 띄움 (backend X). _publish_tcp_jog / _publish_joint_jog 의 인자
를 monkey-patch 된 call_service 로 캡처, twist / velocity 매핑 정확성 검증.

검증 자리:
- capability gate: enabled+gamepad 인 robot 정확히 1개 → target_robot_id 잡힘.
- deadman LT: LT < threshold → motion publish X.
- TCP mode 의 stick → twist 매핑 (linear/angular 부호 + frame).
- Joint mode 의 stick → velocity 매핑 (per joint).
- mode toggle (Back) / frame toggle (Start) / 토크 / 그리퍼 토글.
"""

from __future__ import annotations

import pytest


from core.transport.zenoh_session import ZenohSession


@pytest.fixture(scope="module", autouse=True)
def _zenoh_session():
    """GamepadNode → ApplicationNode → BaseNode → ZenohSession.get() 의존.
    test process 의 임시 peer session 1회 init.
    """
    try:
        ZenohSession.get()
    except Exception:
        ZenohSession.init({"mode": "peer", "connect": []})
    yield


# Import 는 fixture 보다 늦게 — RobotRegistry import 가 robots.yaml load.
from modules.gamepad.driver import GamepadState  # noqa: E402
from modules.gamepad import mapper as M  # noqa: E402
from nodes.application.gamepad_node import (  # noqa: E402
    DEADMAN_THRESHOLD,
    JOINT_VEL_MAX,
    TCP_ANGULAR_MAX,
    TCP_LINEAR_MAX,
    GamepadNode,
)


class FakeCalls:
    """call_service / publish 호출 인자 캡처. (key|topic, data) 쌍 자리."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, object]] = []

    def call_service(self, key: str, data, res_cls, timeout: float = 5.0):
        self.calls.append((key, data))

        class _Res:
            success = True
            message = "ok"
            data = None

        return _Res()

    def publish(self, topic: str, data) -> None:
        self.calls.append((topic, data))


@pytest.fixture
def node(monkeypatch):
    n = GamepadNode()
    # robots.yaml 에 so101_6dof_0.capabilities=[move, calibrate, gamepad] 이 박혀
    # 있으니 capability gate 가 그것을 잡았어야.
    assert n._target_robot_id == "so101_6dof_0", (
        f"capability gate: {n._target_robot_id}"
    )

    fake = FakeCalls()
    monkeypatch.setattr(n, "call_service", fake.call_service)
    monkeypatch.setattr(n, "publish", fake.publish)
    return n, fake


def _state(
    *,
    lt: float = 0.0,
    rt: float = 0.0,
    left_x: float = 0.0,
    left_y: float = 0.0,
    right_x: float = 0.0,
    right_y: float = 0.0,
    hat: tuple[int, int] = (0, 0),
    pressed: set[int] | None = None,
    held: set[int] | None = None,
) -> GamepadState:
    return GamepadState(
        connected=True,
        left_x=left_x,
        left_y=left_y,
        right_x=right_x,
        right_y=right_y,
        lt=lt,
        rt=rt,
        buttons_pressed=pressed or set(),
        buttons_held=held or set(),
        hat=hat,
    )


# ─── Capability gate ─────────────────────────────────────────────────


def test_capability_gate_picks_gamepad_robot(node):
    n, _ = node
    assert n._target_robot_id == "so101_6dof_0"
    # arm_cfgs 길이 = SO-101 의 6 arm.
    assert n._n_arm == 6


# ─── Deadman ─────────────────────────────────────────────────────────


def test_deadman_lt_below_threshold_no_motion(node):
    n, fake = node
    # LT 안 누름 + 스틱 가득 — motion publish X.
    n._handle_motion(_state(lt=0.0, left_x=1.0))
    assert fake.calls == []
    # threshold 직전.
    n._handle_motion(_state(lt=DEADMAN_THRESHOLD - 0.01, left_x=1.0))
    assert fake.calls == []


# ─── TCP jog mode ────────────────────────────────────────────────────


def test_tcp_jog_left_stick_maps_to_linear_xy(node):
    """왼스틱 X → linear +X, 왼스틱 Y (up = -) → linear +Y."""
    n, fake = node
    n._mode = "tcp"
    n._publish_tcp_jog(_state(left_x=1.0, left_y=-1.0))
    assert len(fake.calls) == 1
    key, req = fake.calls[0]
    assert "jog_tcp_stream" in key
    # stick 가득 → linear = (max, max, 0), angular = (0, 0, 0)
    assert req.linear == pytest.approx([TCP_LINEAR_MAX, TCP_LINEAR_MAX, 0.0])
    assert req.angular == pytest.approx([0.0, 0.0, 0.0])
    assert req.frame == "base"


def test_tcp_jog_d_pad_maps_to_z(node):
    """D-Pad ↑ → linear +Z, ↓ → -Z."""
    n, fake = node
    n._mode = "tcp"
    n._publish_tcp_jog(_state(hat=(0, 1)))
    assert fake.calls[-1][1].linear[2] == pytest.approx(TCP_LINEAR_MAX)
    n._publish_tcp_jog(_state(hat=(0, -1)))
    assert fake.calls[-1][1].linear[2] == pytest.approx(-TCP_LINEAR_MAX)


def test_tcp_jog_right_stick_maps_to_pitch_yaw(node):
    """오른스틱 X → +Wz (yaw), Y (up = -) → +Wx (pitch)."""
    n, fake = node
    n._mode = "tcp"
    n._publish_tcp_jog(_state(right_x=1.0, right_y=-1.0))
    req = fake.calls[-1][1]
    assert req.angular[0] == pytest.approx(TCP_ANGULAR_MAX)  # Wx
    assert req.angular[1] == pytest.approx(0.0)
    assert req.angular[2] == pytest.approx(TCP_ANGULAR_MAX)  # Wz


def test_tcp_jog_lb_rb_maps_to_roll(node):
    """LB → -Wy, RB → +Wy."""
    n, fake = node
    n._mode = "tcp"
    n._publish_tcp_jog(_state(held={M.BTN_RB}))
    assert fake.calls[-1][1].angular[1] == pytest.approx(TCP_ANGULAR_MAX)
    n._publish_tcp_jog(_state(held={M.BTN_LB}))
    assert fake.calls[-1][1].angular[1] == pytest.approx(-TCP_ANGULAR_MAX)


def test_tcp_jog_frame_toggle(node):
    """Start 버튼 → frame base ↔ tcp 토글. TCP mode 자리만."""
    n, fake = node
    n._mode = "tcp"
    assert n._frame == "base"
    n._handle_buttons(_state(pressed={M.BTN_START}))
    assert n._frame == "tcp"
    n._handle_buttons(_state(pressed={M.BTN_START}))
    assert n._frame == "base"
    # joint mode 면 Start 가 frame 안 토글.
    n._mode = "joint"
    n._handle_buttons(_state(pressed={M.BTN_START}))
    assert n._frame == "base"


# ─── Joint jog mode ──────────────────────────────────────────────────


def test_joint_jog_stick_mapping(node):
    """6DOF: 왼X→J1, -왼Y→J2, 오X→J3, -오Y→J4, RB-LB→J5, D-Pad↑/↓→J6."""
    n, fake = node
    n._mode = "joint"
    n._publish_joint_jog(
        _state(
            left_x=1.0, left_y=-1.0,
            right_x=1.0, right_y=-1.0,
            held={M.BTN_RB},
            hat=(0, 1),
        )
    )
    req = fake.calls[-1][1]
    assert "jog_j_stream" in fake.calls[-1][0]
    assert len(req.velocities) == 6  # SO-101 6DOF
    # 모두 +JOINT_VEL_MAX 방향.
    expected = [JOINT_VEL_MAX] * 6
    for got, exp in zip(req.velocities, expected):
        assert got == pytest.approx(exp), f"{req.velocities}"


def test_joint_jog_dof_matches_arm_count(node):
    """velocities 길이 = robot 의 arm dof — server SpeedJ schema 와 일치."""
    n, fake = node
    n._mode = "joint"
    n._publish_joint_jog(_state(left_x=0.5))
    assert len(fake.calls[-1][1].velocities) == n._n_arm


# ─── Mode toggle ─────────────────────────────────────────────────────


def test_back_toggles_mode(node):
    """Back 버튼 → TCP ↔ Joint."""
    n, _ = node
    assert n._mode == "tcp"
    n._handle_buttons(_state(pressed={M.BTN_BACK}))
    assert n._mode == "joint"
    n._handle_buttons(_state(pressed={M.BTN_BACK}))
    assert n._mode == "tcp"


# ─── Discrete actions ────────────────────────────────────────────────


def test_x_toggles_torque(node):
    n, fake = node
    initial = n._torque_on
    n._handle_buttons(_state(pressed={M.BTN_X}))
    assert n._torque_on == (not initial)
    # MOTOR_ENABLE 호출됐는지.
    assert any("motor/srv/enable" in c[0] for c in fake.calls)


def test_a_toggles_gripper(node):
    n, fake = node
    n._handle_buttons(_state(pressed={M.BTN_A}))
    assert any("motor/srv/gripper" in c[0] for c in fake.calls)


def test_b_triggers_calib_capture(node):
    n, fake = node
    n._handle_buttons(_state(pressed={M.BTN_B}))
    assert any("calib/srv/handeye/capture" in c[0] for c in fake.calls)


def test_y_triggers_home(node):
    n, fake = node
    n._handle_buttons(_state(pressed={M.BTN_Y}))
    # MoveJ 호출 with all degree=0.
    move_j_calls = [c for c in fake.calls if "motion/srv/move_j" in c[0]]
    assert len(move_j_calls) == 1
    req = move_j_calls[0][1]
    assert len(req.joints) == n._n_arm
    for j in req.joints:
        assert j.degree == 0.0
