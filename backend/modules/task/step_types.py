from dataclasses import dataclass, field
from typing import Literal, Union

from core.gripper_setup import GripperSetup
from modules.kinematics.solver import Position3


@dataclass
class MoveTCPStep:
    position: Position3 | None = None
    position_key: str | None = None
    offset: Position3 = (0.0, 0.0, 0.0)
    label: str = ""
    type: Literal["move_tcp"] = field(
        default="move_tcp", init=False, repr=False)


@dataclass
class GripperStep:
    action: Literal["open", "close"] = "open"
    current: int = 200   # mA, 파지력 조정용
    label: str = ""
    type: Literal["gripper"] = field(default="gripper", init=False, repr=False)


@dataclass
class DetectStep:
    output_key: str = "detected_position"
    label: str = ""
    type: Literal["detect"] = field(default="detect", init=False, repr=False)


@dataclass
class GroundedDetectStep:
    """자연어 prompt → Grounding DINO → base frame position을 context에 저장."""

    prompt: str = ""
    output_key: str = "detected_position"
    label: str = ""
    type: Literal["grounded_detect"] = field(
        default="grounded_detect", init=False, repr=False
    )


@dataclass
class WaitStep:
    duration_sec: float = 0.5
    label: str = ""
    type: Literal["wait"] = field(default="wait", init=False, repr=False)


@dataclass
class HomeStep:
    label: str = "go_home"
    type: Literal["home"] = field(default="home", init=False, repr=False)


@dataclass
class SelfPlayStep:
    """Self-play attempt loop 전체를 1 step 으로 wrapping.

    실제 attempt loop / 3-stage 측정은 `SelfPlayRunner` 가 담당
    (step_executor 의 `_self_play` 핸들러). 결정 로그 #2 의 '전용 클래스' 정신을
    유지하면서 TaskRunner 의 pause/resume/state publish 인프라 재사용.

    grasp z 는 자동: detector 가 객체 height (bbox depth 분석으로 추정) 를
    응답하면 runner 가 height 보고 정책 결정 (얇은 객체→윗면 옆 / 큰 객체→옆면 중간).
    """

    prompt: str = "white calibration cube"
    max_attempts: int = 100
    log_dir: str = "robot/logs/self_play"
    gripper_setup: GripperSetup | None = None  # 객체별 override (None = default)
    label: str = "self_play"
    type: Literal["self_play"] = field(
        default="self_play", init=False, repr=False
    )


Step = Union[
    MoveTCPStep,
    GripperStep,
    DetectStep,
    GroundedDetectStep,
    WaitStep,
    HomeStep,
    SelfPlayStep,
]


@dataclass
class Task:
    name: str
    steps: list[Step]
    description: str = ""


@dataclass
class TaskContext:
    data: dict = field(default_factory=dict)

    def set(self, key: str, value: object) -> None:
        self.data[key] = value

    def get(self, key: str, default: object = None) -> object:
        return self.data.get(key, default)

    def has(self, key: str) -> bool:
        return key in self.data

    def clear(self) -> None:
        self.data.clear()
