"""
입력 매핑:
  D-Pad X/Y       → TCP X/Y  (1mm/step, 버튼 repeat)
  Right Stick X/Y → TCP X/Y  (아날로그, 기울기 비례 속도)
  LB / RB         → TCP Z-/Z+ (버튼 repeat)
  LT / RT         → TCP Z-/Z+ (아날로그)
  X               → 토크 ON/OFF
  Y               → 홈 이동
  B               → TODO: 캡처
"""

import time
import threading
import logging

from core.common import GRIPPER_ID
from core.transport.base_node import BaseNode
from core.transport.topic_map import Service
from core.transport.messages.base import EmptyData
from core.transport.messages.motion import (
    JointDegree,
    MotionTcpPose,
    MoveJReq,
    MoveTcpReq,
)
from core.transport.messages.motor import (
    MotorEnableReq,
    MotorEnableRes,
    MotorGripperReq,
)
from modules.motor.motor_config import load_motor_config
from modules.gamepad.driver import GamepadDriver, GamepadState
from modules.gamepad import mapper as M

logger = logging.getLogger(__name__)

logger = logging.getLogger(__name__)

# ─── 버튼 repeat 타이밍 ───────────────────────────────────────────────────────
REPEAT_INITIAL_DELAY = 0.4
REPEAT_INTERVAL = 0.02  # 반복 간격 (초) ≈ 50Hz

# ─── 이동 스텝 ────────────────────────────────────────────────────────────────
DPAD_STEP = 0.001  # D-Pad 1회 이동 (1mm)
ANALOG_MAX = 0.002  # 스틱/트리거 최대 속도 (m/tick, 50Hz 기준)

POLL_HZ = 50


class ButtonRepeater:
    def __init__(
        self,
        initial_delay: float = REPEAT_INITIAL_DELAY,
        interval: float = REPEAT_INTERVAL,
    ) -> None:
        self._initial_delay = initial_delay
        self._interval = interval
        self._press_time: float | None = None
        self._next_repeat: float | None = None

    def update(self, is_held: bool, now: float) -> bool:
        if not is_held:
            self._press_time = None
            self._next_repeat = None
            return False

        if self._press_time is None:
            # 새로 눌림 → 즉시 1회
            self._press_time = now
            self._next_repeat = now + self._initial_delay
            return True

        # press_time != None 이면 _next_repeat 도 같이 set 됐음 — 동기 set.
        assert self._next_repeat is not None
        if now >= self._next_repeat:
            self._next_repeat = now + self._interval
            return True

        return False


class GamepadNode(BaseNode):
    def __init__(self, robot_id: str | None = None) -> None:
        # gamepad 는 global UI — robot-scoped service 호출 시 self.r() default
        # fallback (N=1). multi-robot 시 어떤 robot 을 조작할지 결정 필요.
        super().__init__("gamepad_node", robot_id=robot_id)

        self._driver = GamepadDriver()
        self._torque_on = True
        self._last_connected: bool = False

        _, self._motor_cfgs = load_motor_config(robot_id)
        self._arm_cfgs = [m for m in self._motor_cfgs if m.id != GRIPPER_ID]
        self._tcp_position: list[float] | None = None
        self._gripper_open = False

        # D-Pad 4방향 repeater
        self._rep_hat_right = ButtonRepeater()
        self._rep_hat_left = ButtonRepeater()
        self._rep_hat_up = ButtonRepeater()
        self._rep_hat_down = ButtonRepeater()

        # LB / RB repeater
        self._rep_lb = ButtonRepeater()
        self._rep_rb = ButtonRepeater()

        self._running = False
        self._thread: threading.Thread | None = None

    # ─── Lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="gamepad_poll"
        )
        self._thread.start()
        logger.info("GamepadNode 시작")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("GamepadNode 종료")

    # ─── Main loop ────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        self._driver.init()

        try:
            while self._running:
                t0 = time.monotonic()
                now = t0

                state = self._driver.poll()

                if state.connected != self._last_connected:
                    if state.connected:
                        self.log("info", "조이스틱 연결됨")
                        self._sync_tcp()
                    else:
                        self.log("info", "조이스틱 연결 해제")
                        self._tcp_position = None
                    self._last_connected = state.connected

                if state.connected:
                    self._handle_buttons(state, now)
                    self._handle_movement(state, now)

                elapsed = time.monotonic() - t0
                sleep_t = (1.0 / POLL_HZ) - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        finally:
            self._driver.quit()

    def _sync_tcp(self) -> bool:
        res = self.call_service(
            self.r(Service.MOTION_GET_TCP), EmptyData(), MotionTcpPose
        )
        if res.success and res.data is not None and res.data.position:
            self._tcp_position = list(res.data.position)
            return True
        logger.debug("TCP 동기화 실패")
        self._tcp_position = None
        return False

    # ─── Button handling ──────────────────────────────────────────────────────

    def _handle_buttons(self, state: GamepadState, now: float) -> None:
        pressed = state.buttons_pressed

        # X: 토크 ON/OFF
        if M.BTN_X in pressed:
            self._torque_on = not self._torque_on
            res = self.call_service(
                self.r(Service.MOTOR_ENABLE),
                MotorEnableReq(enable=self._torque_on),
                MotorEnableRes,
            )
            if res.success:
                self.log("info", f"토크 {'ON' if self._torque_on else 'OFF'}")
                self._sync_tcp()
            else:
                logger.warning(
                    f"토크 {'ON' if self._torque_on else 'OFF'} 실패: {res.message}")
                self._torque_on = not self._torque_on

        # Y: 홈 이동
        if M.BTN_Y in pressed:
            self._go_home()

        # A: 그리퍼 open/close
        if M.BTN_A in pressed:
            self._toggle_gripper()

        # B: 캡처 (TODO)
        if M.BTN_B in pressed:
            logger.debug("캡처 버튼 — TODO")

    # ─── Movement ─────────────────────────────────────────────────────────────

    def _handle_movement(self, state: GamepadState, now: float) -> None:
        dx, dy, dz = 0.0, 0.0, 0.0

        # ── D-Pad (버튼 repeat) ───────────────────────────────────────────────
        hat_x, hat_y = state.hat

        if self._rep_hat_right.update(hat_x > 0, now):
            dy -= DPAD_STEP
        if self._rep_hat_left.update(hat_x < 0, now):
            dy += DPAD_STEP
        if self._rep_hat_up.update(hat_y > 0, now):
            dx += DPAD_STEP
        if self._rep_hat_down.update(hat_y < 0, now):
            dx -= DPAD_STEP

        # ── Right Stick (아날로그 XY) ─────────────────────────────────────────
        dx += -state.right_y * ANALOG_MAX
        dy += -state.right_x * ANALOG_MAX

        # ── LB/RB (버튼 repeat Z) ─────────────────────────────────────────────
        if self._rep_rb.update(M.BTN_RB in state.buttons_held, now):
            dz += DPAD_STEP
        if self._rep_lb.update(M.BTN_LB in state.buttons_held, now):
            dz -= DPAD_STEP

        # ── LT/RT (아날로그 Z) ────────────────────────────────────────────────
        if state.rt > M.TRIGGER_THRESHOLD:
            dz += (state.rt - M.TRIGGER_THRESHOLD) / \
                (1.0 - M.TRIGGER_THRESHOLD) * ANALOG_MAX
        if state.lt > M.TRIGGER_THRESHOLD:
            dz -= (state.lt - M.TRIGGER_THRESHOLD) / \
                (1.0 - M.TRIGGER_THRESHOLD) * ANALOG_MAX

        if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9:
            return

        self._move_tcp_delta(dx, dy, dz)

    # ─── TCP delta move ───────────────────────────────────────────────────────

    def _move_tcp_delta(self, dx: float, dy: float, dz: float) -> None:
        if self._tcp_position is None:
            if not self._sync_tcp():
                return
        # _sync_tcp 가 True 면 self._tcp_position 채워짐 — pyright 보강용 assert.
        assert self._tcp_position is not None

        target = [
            self._tcp_position[0] + dx,
            self._tcp_position[1] + dy,
            self._tcp_position[2] + dz,
        ]

        res = self.call_service(
            self.r(Service.MOTION_MOVE_TCP),
            MoveTcpReq(position=target),
            EmptyData,
        )

        if res.success:
            self._tcp_position = target
        else:
            logger.debug(f"move_tcp 실패: {res.message} — TCP 재동기화")
            self._sync_tcp()

    # ─── Home ─────────────────────────────────────────────────────────────────

    def _go_home(self) -> None:
        res = self.call_service(
            self.r(Service.MOTION_MOVE_J),
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
            self._tcp_position = None
        else:
            logger.warning(f"홈 이동 실패: {res.message}")

    # ─── Gripper toggle ────────────────────────────────────────────────────────

    def _toggle_gripper(self) -> None:
        self._gripper_open = not self._gripper_open
        res = self.call_service(
            self.r(Service.MOTOR_GRIPPER),
            MotorGripperReq(action="open" if self._gripper_open else "close"),
            EmptyData,
        )
        if res.success:
            self.log("info", f"그리퍼 {'열기' if self._gripper_open else '닫기'}")
        else:
            self._gripper_open = not self._gripper_open  # 실패 시 롤백
            logger.warning(f"그리퍼 실패: {res.message}")
