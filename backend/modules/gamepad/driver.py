"""Gamepad driver — SDL hot-plug 이벤트 기반.

부재 자리는 zero-cost (`pygame.event.get()` 빈 큐만 훑음). 런타임 hot-plug 은
SDL 가 OS 의 device-change 알림 (Windows WM_DEVICECHANGE / Linux udev /
macOS IOKit) 받아 `JOYDEVICEADDED` / `JOYDEVICEREMOVED` 이벤트를 큐잉 — 우리는
poll 마다 이벤트 큐만 비우면서 연결/해제 처리. polling 기반 `quit() + init()`
사이클은 사용 X (CPU/GIL 점유).

시작 시 이미 꽂혀 있는 자리도 동일 경로 — `pygame.joystick.init()` 이 SDL 에
기존 디바이스용 `JOYDEVICEADDED` 를 큐잉하므로 첫 poll 에서 연결 처리.
"""
import logging
from dataclasses import dataclass, field
import pygame

from modules.gamepad import mapper as M

logger = logging.getLogger(__name__)


@dataclass
class GamepadState:
    connected: bool = False

    # Axes
    left_x:  float = 0.0
    left_y:  float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    lt:      float = 0.0
    rt:      float = 0.0

    # Buttons (이번 poll에서 새로 눌린 것)
    buttons_pressed: set[int] = field(default_factory=set)
    # Buttons (현재 눌려있는 것)
    buttons_held: set[int] = field(default_factory=set)

    # D-Pad
    hat: tuple[int, int] = (0, 0)


class GamepadDriver:
    def __init__(self, deadzone: float = M.DEADZONE) -> None:
        self._deadzone = deadzone
        self._joystick: pygame.joystick.JoystickType | None = None
        self._prev_buttons: dict[int, bool] = {}
        self._initialized = False

    # ─── Public ────────────────────────────────────────────────────────────

    def init(self) -> None:
        # pygame.init() — SDL video subsystem 까지 켜는 게 Windows 의 hot-plug
        # 알림 (hidden message window 통한 WM_DEVICECHANGE) 에 필요. joystick
        # subsystem 만 켜도 대부분 동작하지만 플랫폼 의존 위험 회피 차원에서 full init.
        pygame.init()
        pygame.joystick.init()
        self._initialized = True
        logger.info("GamepadDriver 초기화 완료")

    def quit(self) -> None:
        self._release_joystick()
        if self._initialized:
            try:
                pygame.joystick.quit()
                pygame.quit()
            except Exception:
                pass
        self._initialized = False

    def poll(self) -> GamepadState:
        state = GamepadState()
        if not self._initialized:
            return state

        # SDL 이벤트 큐 소비. 미연결 자리에서 이 자리만 도는 게 전부 — device
        # enum / quit-init 사이클 X. event.get() 이 내부적으로 pump 까지 함.
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                self._handle_device_added(event.device_index)
            elif event.type == pygame.JOYDEVICEREMOVED:
                self._handle_device_removed(event.instance_id)

        if self._joystick is None:
            return state

        try:
            state.connected = True

            state.left_x = self._apply_deadzone(
                self._get_axis(M.AXIS_LEFT_X))
            state.left_y = self._apply_deadzone(
                self._get_axis(M.AXIS_LEFT_Y))
            state.right_x = self._apply_deadzone(
                self._get_axis(M.AXIS_RIGHT_X))
            state.right_y = self._apply_deadzone(
                self._get_axis(M.AXIS_RIGHT_Y))
            state.lt = self._normalize_trigger(self._get_axis(M.AXIS_LT))
            state.rt = self._normalize_trigger(self._get_axis(M.AXIS_RT))

            n = self._joystick.get_numbuttons()
            cur: dict[int, bool] = {
                i: bool(self._joystick.get_button(i)) for i in range(n)
            }
            state.buttons_held = {i for i, v in cur.items() if v}
            state.buttons_pressed = {
                i for i, v in cur.items()
                if v and not self._prev_buttons.get(i, False)
            }
            self._prev_buttons = cur

            if self._joystick.get_numhats() > 0:
                hat = self._joystick.get_hat(M.HAT_INDEX)
                state.hat = (int(hat[0]), int(hat[1]))

        except Exception as e:
            logger.warning(f"조이스틱 읽기 오류: {e}")
            self._release_joystick()

        return state

    # ─── Internal — SDL hot-plug 이벤트 핸들러 ──────────────────────────────

    def _handle_device_added(self, device_index: int) -> None:
        # mini pendant 컨벤션 — 동시에 1개. 이미 연결된 자리에서 추가 디바이스는 무시.
        if self._joystick is not None:
            logger.debug(
                f"JOYDEVICEADDED({device_index}) 무시 — 이미 패드 연결됨"
            )
            return
        try:
            joy = pygame.joystick.Joystick(device_index)
            joy.init()
            self._joystick = joy
            self._prev_buttons = {}
            logger.info(
                f"조이스틱 연결: {joy.get_name()} "
                f"(축 {joy.get_numaxes()}개, 버튼 {joy.get_numbuttons()}개, "
                f"햇 {joy.get_numhats()}개)"
            )
        except Exception as e:
            logger.warning(f"조이스틱 초기화 실패 (idx={device_index}): {e}")

    def _handle_device_removed(self, instance_id: int) -> None:
        if self._joystick is None:
            return
        try:
            if self._joystick.get_instance_id() == instance_id:
                logger.info("조이스틱 연결 해제")
                self._release_joystick()
        except Exception:
            # 객체 상태 corrupted 자리 — 그냥 release
            self._release_joystick()

    def _release_joystick(self) -> None:
        if self._joystick is not None:
            try:
                self._joystick.quit()
            except Exception:
                pass
            self._joystick = None
            self._prev_buttons = {}

    def _get_axis(self, index: int) -> float:
        if self._joystick is None:
            return 0.0
        try:
            if index < self._joystick.get_numaxes():
                return float(self._joystick.get_axis(index))
        except Exception:
            pass
        return 0.0

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self._deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self._deadzone) / (1.0 - self._deadzone)

    @staticmethod
    def _normalize_trigger(raw: float) -> float:
        return (raw + 1.0) / 2.0
