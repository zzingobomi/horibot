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

        pygame.event.pump()

        if self._joystick is None:
            self._try_connect()

        if self._joystick is None:
            return state

        if not self._joystick.get_init():
            logger.info("조이스틱 연결 해제 감지")
            self._release_joystick()
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

    # ─── Internal ─────────────────────────────────────────────────────────────

    def _try_connect(self) -> None:
        pygame.joystick.quit()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            return

        try:
            joy = pygame.joystick.Joystick(0)
            joy.init()
            self._joystick = joy
            self._prev_buttons = {}
            logger.info(
                f"조이스틱 연결: {joy.get_name()} "
                f"(축 {joy.get_numaxes()}개, 버튼 {joy.get_numbuttons()}개, 햇 {joy.get_numhats()}개)"
            )
        except Exception as e:
            logger.debug(f"조이스틱 초기화 실패: {e}")

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
