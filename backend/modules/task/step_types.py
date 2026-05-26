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
    # close 직후 Present_Position 으로 잡힘 검증. 빈손이면 step fail → task fail.
    verify_grasp: bool = False
    label: str = ""
    type: Literal["gripper"] = field(default="gripper", init=False, repr=False)


@dataclass
class DetectStep:
    output_key: str = "detected_position"
    label: str = ""
    type: Literal["detect"] = field(default="detect", init=False, repr=False)


@dataclass
class GroundedDetectStep:
    """자연어 prompt → Grounding DINO → base frame position을 context에 저장.

    context에 저장되는 값:
      - output_key: position (객체 윗면 base xyz) [list[float] 3개]
      - output_key + "_meta": {"base_z": float, "height": float}
        후속 GraspPolicyStep이 height 기반으로 grasp_z를 계산할 때 사용.
    """

    prompt: str = ""
    output_key: str = "detected_position"
    label: str = ""
    type: Literal["grounded_detect"] = field(
        default="grounded_detect", init=False, repr=False
    )


@dataclass
class SearchAndDetectStep:
    """search pose 들 순회하며 grounded_detect 시도 — 첫 성공 시 break.

    robot_poses.yaml 의 search_* 자세 (lexical 정렬) 를 차례로 방문하면서 객체
    탐색. workspace 가 한 view 에 다 안 들어와도 안정적으로 객체 위치 확보.

    context 저장:
      - output_key: position (객체 윗면 base xyz)
      - output_key + "_meta": {"base_z", "height"}
    모든 pose 에서 fail → step fail (task 중단).
    """

    prompt: str = ""
    output_key: str = "detected_position"
    label: str = ""
    type: Literal["search_and_detect"] = field(
        default="search_and_detect", init=False, repr=False
    )


@dataclass
class PlacePolicyStep:
    """place 객체 detect 결과 → release 위치 계산.

    캐비넷/박스 같은 객체 위에 큐브를 *공중에서 떨구지 않고* 살짝 내려놓도록
    place 객체 윗면 z + drop_clearance 만큼 위에서 release.

    입력:
      - input_key: SearchAndDetectStep / GroundedDetectStep 의 output_key
        (position = 객체 윗면 base xyz)
    출력:
      - output_key: [x, y, z + drop_clearance]
    """

    input_key: str = "place_detected"
    output_key: str = "place_xyz"
    drop_clearance: float = 0.010  # 윗면 위 1cm
    label: str = ""
    type: Literal["place_policy"] = field(
        default="place_policy", init=False, repr=False
    )


@dataclass
class GraspPolicyStep:
    """객체 height 기반 grasp z 결정 정책 — 항상 옆면 그립.

    grasp_z = base_z + height * grasp_ratio (책상 + height 의 일정 비율).

    입력:
      - input_key: GroundedDetectStep의 output_key (position 들어있어야 함)
      - input_key + "_meta": {"base_z", "height"}
    출력:
      - output_key: [x, y, grasp_z] (MoveTCPStep의 position_key로 그대로 사용)
    """

    input_key: str = "detected_position"
    output_key: str = "grasp_xyz"
    grasp_ratio: float = 0.5       # height의 절반 (옆면 중간)
    label: str = ""
    type: Literal["grasp_policy"] = field(
        default="grasp_policy", init=False, repr=False
    )


@dataclass
class VerifyGraspStep:
    """현재 그리퍼 Present_Position 으로 잡힘 상태 확인.

    GripperStep(verify_grasp=True) 는 close *직후* 만 검증 → 이후 lift/place 중
    떨어진 경우 못 잡음. 이 step 을 lift/place 사이에 끼워서 중간 검증.
    """

    label: str = ""
    type: Literal["verify_grasp"] = field(
        default="verify_grasp", init=False, repr=False
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
    SearchAndDetectStep,
    GraspPolicyStep,
    PlacePolicyStep,
    VerifyGraspStep,
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
