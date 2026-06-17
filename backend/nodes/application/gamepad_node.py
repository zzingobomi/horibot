"""Mini pendant — SO-101 6DOF jog. JogTcp / JogJ topic stream publish.

motion_taxonomy.md Phase 1 의 gamepad enabling. 산업 펜던트 컨벤션:
- 6DOF twist (3 linear + 3 angular) → cartesian jog
- joint velocity 벡터 → joint jog
- enabling switch (deadman, ISO 10218-1) → LT trigger hold

매핑 (8BitDo Ultimate 2C):

  공통:
    X         = 토크 토글
    Y         = 홈 (MoveJ to 0)
    A         = 그리퍼 토글
    B         = 캘 캡처 (CALIB_HANDEYE_CAPTURE) — 캘 모드 자리
    Back      = mode 토글 (TCP ↔ Joint jog)
    Start     = frame 토글 (base ↔ tcp) — TCP mode 자리만
    LT (hold) = deadman. 안 누름 = motion publish X (server timeout 자동 정지)

  TCP jog mode (cartesian twist):
    왼스틱 X       → linear  +X (좌우)
    왼스틱 Y       → linear  +Y (전후, stick up = +Y)
    D-Pad 좌우     → linear  +X  (1m/s scale 보조)
    D-Pad 상하     → linear  +Z
    오른스틱 X     → angular +Wz (yaw)
    오른스틱 Y     → angular +Wx (pitch, stick up = +Wx)
    LB / RB        → angular -Wy / +Wy (roll)

  Joint jog mode (joint velocity vector):
    왼스틱 X       → J1
    왼스틱 Y       → J2
    오른스틱 X     → J3
    오른스틱 Y     → J4
    LB / RB        → J5- / J5+
    D-Pad 상하     → J6

Capability gate — robots.yaml::capabilities 에 "gamepad" 있는 enabled robot 정확히 1개.
N>1 이면 RuntimeError. 0 이면 start() no-op.
"""

from __future__ import annotations

import logging
import threading
import time

from core.transport.application_node import ApplicationNode
from core.transport.messages.base import EmptyData
from core.transport.messages.calibration import HandeyeCaptureRes
from core.transport.messages.motion import (
    JogJReq,
    JogTcpReq,
    JointDegree,
    MoveJReq,
)
from core.transport.messages.motor import (
    MotorEnableReq,
    MotorEnableRes,
    MotorGripperReq,
)
from core.transport.topic_map import Service, Topic
from modules.gamepad import mapper as M
from modules.gamepad.driver import GamepadDriver, GamepadState
from modules.motor.motor_config import load_motor_layout

logger = logging.getLogger(__name__)

POLL_HZ = 50
POLL_DT = 1.0 / POLL_HZ

# Velocity scale — 펜던트 max 입력 (스틱 1.0 또는 트리거 1.0) 시의 robot 속도.
# 산업 펜던트 jog 안전 권장치 정도. 실 운용 시 ergonomics 보고 motion.yaml SSOT 화 검토.
TCP_LINEAR_MAX = 0.08   # m/s
TCP_ANGULAR_MAX = 0.8   # rad/s
JOINT_VEL_MAX = 0.6     # rad/s (per joint)

# Deadman 임계. LT 트리거가 이 이상이어야 motion publish (LT 0..1 normalized).
DEADMAN_THRESHOLD = 0.3

GAMEPAD_CAPABILITY = "gamepad"


class GamepadNode(ApplicationNode):
    """Mini pendant — capability='gamepad' robot 1개에 SpeedTcp / SpeedJ 발행."""

    def __init__(self) -> None:
        super().__init__("gamepad_node")

        self._driver = GamepadDriver()
        self._last_connected: bool = False

        # ─── capability gate ─────────────────────────────────────
        # enabled + gamepad capability 정확히 1개. 0 = disable, 2+ = fail-fast.
        candidates = [
            c for c in self._registry.enabled_robots()
            if GAMEPAD_CAPABILITY in c.capabilities
        ]
        if len(candidates) > 1:
            raise RuntimeError(
                f"GamepadNode: capability='gamepad' robot 이 2개 이상 "
                f"({[c.robot_id for c in candidates]}). "
                f"robots.yaml 의 capabilities 에서 1개로 줄여야 함."
            )
        self._target_robot_id: str | None = (
            candidates[0].robot_id if candidates else None
        )

        # robot 별 arm_cfgs — joint 개수 / id 조회용.
        self._arm_cfgs = (
            load_motor_layout(self._target_robot_id).arm
            if self._target_robot_id else []
        )
        self._n_arm = len(self._arm_cfgs)

        # 모드 / frame / 토글 상태.
        self._mode: str = "tcp"  # "tcp" | "joint"
        self._frame: str = "base"  # "base" | "tcp"
        self._torque_on = True
        self._gripper_open = False

        # Lifecycle
        self._running = False
        self._thread: threading.Thread | None = None

    # ─── Lifecycle ────────────────────────────────────────────────

    def start(self) -> None:
        super().start()
        if self._target_robot_id is None:
            self.log("info", "GamepadNode: capability='gamepad' robot 없음 — 비활성")
            return
        self.log("info", f"GamepadNode: target robot = {self._target_robot_id}")
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="gamepad_poll"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        super().stop()

    # ─── Topic key helper (target robot scoped) ───────────────────

    def _t(self, template: str) -> str:
        """target robot 의 robot_id 로 template expand. None 자리는 호출 X."""
        assert self._target_robot_id is not None
        return template.format(robot_id=self._target_robot_id)

    # ─── Main loop ────────────────────────────────────────────────

    def _loop(self) -> None:
        self._driver.init()
        try:
            while self._running:
                t0 = time.monotonic()

                state = self._driver.poll()
                if state.connected != self._last_connected:
                    self.log(
                        "info",
                        "조이스틱 연결됨" if state.connected else "조이스틱 연결 해제",
                    )
                    self._last_connected = state.connected

                if state.connected:
                    self._handle_buttons(state)
                    if state.lt >= DEADMAN_THRESHOLD:
                        self._handle_motion(state)
                    # deadman released — motion publish X. server side 100ms timeout
                    # 으로 streamer 가 jerk-limited 자동 정지.

                elapsed = time.monotonic() - t0
                sleep_t = POLL_DT - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        finally:
            self._driver.quit()

    # ─── Buttons ──────────────────────────────────────────────────

    def _handle_buttons(self, state: GamepadState) -> None:
        pressed = state.buttons_pressed

        if M.BTN_X in pressed:
            self._toggle_torque()
        if M.BTN_Y in pressed:
            self._go_home()
        if M.BTN_A in pressed:
            self._toggle_gripper()
        if M.BTN_B in pressed:
            self._calib_capture()
        if M.BTN_BACK in pressed:
            self._mode = "joint" if self._mode == "tcp" else "tcp"
            self.log("info", f"jog 모드 → {self._mode.upper()}")
        if M.BTN_START in pressed and self._mode == "tcp":
            self._frame = "tcp" if self._frame == "base" else "base"
            self.log("info", f"TCP jog frame → {self._frame.upper()}")

    # ─── Motion (LT held) ─────────────────────────────────────────

    def _handle_motion(self, state: GamepadState) -> None:
        # defensive deadman — `_loop` 가 이미 검사하지만 외부 caller (test/직접 호출)
        # 자리에서도 안전. LT 안 누름 = 무조건 motion publish X.
        if state.lt < DEADMAN_THRESHOLD:
            return
        if self._mode == "tcp":
            self._publish_tcp_jog(state)
        else:
            self._publish_joint_jog(state)

    def _publish_tcp_jog(self, state: GamepadState) -> None:
        # Linear:
        #   왼스틱 X         = +X
        #   왼스틱 Y (up = -) = +Y  (stick up → +Y)
        #   D-Pad ←/→        = +X 보조
        #   D-Pad ↑/↓        = +Z
        hat_x, hat_y = state.hat
        vx = state.left_x * TCP_LINEAR_MAX + float(hat_x) * TCP_LINEAR_MAX
        vy = -state.left_y * TCP_LINEAR_MAX
        vz = float(hat_y) * TCP_LINEAR_MAX
        # clamp
        vx = max(-TCP_LINEAR_MAX, min(TCP_LINEAR_MAX, vx))

        # Angular:
        #   오른스틱 X = +Wz (yaw)
        #   오른스틱 Y = +Wx (pitch, stick up → +Wx)
        #   RB / LB     = +Wy / -Wy (roll)
        wz = state.right_x * TCP_ANGULAR_MAX
        wx = -state.right_y * TCP_ANGULAR_MAX
        wy = 0.0
        if M.BTN_RB in state.buttons_held:
            wy += TCP_ANGULAR_MAX
        if M.BTN_LB in state.buttons_held:
            wy -= TCP_ANGULAR_MAX

        # JogTcp topic stream — backend 가 latched ref + SE(3) 적분 + IK 자리.
        # 50Hz service RTT 회피 위해 fire-and-forget topic publish.
        self.publish(
            self._t(Topic.MOTION_JOG_TCP_STREAM),
            JogTcpReq(
                linear=[vx, vy, vz],
                angular=[wx, wy, wz],
                frame=self._frame,  # type: ignore[arg-type]
            ),
        )

    def _publish_joint_jog(self, state: GamepadState) -> None:
        # joint 매핑 (위에서부터 1-base):
        #   J1 = 왼스틱 X
        #   J2 = -왼스틱 Y (stick up = +J2)
        #   J3 = 오른스틱 X
        #   J4 = -오른스틱 Y
        #   J5 = RB - LB
        #   J6 = D-Pad ↑/↓
        hat_x, hat_y = state.hat
        rb = 1.0 if M.BTN_RB in state.buttons_held else 0.0
        lb = 1.0 if M.BTN_LB in state.buttons_held else 0.0

        joint_axes: list[float] = [
            state.left_x,
            -state.left_y,
            state.right_x,
            -state.right_y,
            rb - lb,
            float(hat_y),
        ]
        # robot dof 만큼만 (5DOF 면 처음 5개).
        velocities = [a * JOINT_VEL_MAX for a in joint_axes[: self._n_arm]]

        self.publish(
            self._t(Topic.MOTION_JOG_J_STREAM),
            JogJReq(velocities=velocities),
        )

    # ─── Discrete actions ─────────────────────────────────────────

    def _toggle_torque(self) -> None:
        self._torque_on = not self._torque_on
        res = self.call_service(
            self._t(Service.MOTOR_ENABLE),
            MotorEnableReq(enable=self._torque_on),
            MotorEnableRes,
        )
        if res.success:
            self.log("info", f"토크 {'ON' if self._torque_on else 'OFF'}")
        else:
            self._torque_on = not self._torque_on  # 롤백
            logger.warning(
                f"토크 토글 실패: {res.message}"
            )

    def _go_home(self) -> None:
        res = self.call_service(
            self._t(Service.MOTION_MOVE_J),
            MoveJReq(
                joints=[
                    JointDegree(id=cfg.id, degree=0.0)
                    for cfg in self._arm_cfgs
                ]
            ),
            EmptyData,
        )
        if res.success:
            self.log("info", "홈 이동")
        else:
            logger.warning(f"홈 이동 실패: {res.message}")

    def _toggle_gripper(self) -> None:
        self._gripper_open = not self._gripper_open
        res = self.call_service(
            self._t(Service.MOTOR_GRIPPER),
            MotorGripperReq(
                action="open" if self._gripper_open else "close"
            ),
            EmptyData,
        )
        if res.success:
            self.log("info", f"그리퍼 {'열기' if self._gripper_open else '닫기'}")
        else:
            self._gripper_open = not self._gripper_open  # 롤백
            logger.warning(f"그리퍼 실패: {res.message}")

    def _calib_capture(self) -> None:
        res = self.call_service(
            self._t(Service.CALIB_HANDEYE_CAPTURE),
            EmptyData(),
            HandeyeCaptureRes,
        )
        if res.success:
            self.log("info", "Hand-Eye 캡처")
        else:
            logger.debug(f"캡처 실패: {res.message}")
