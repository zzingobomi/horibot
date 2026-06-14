import sys
import yaml
from dataclasses import dataclass
from enum import StrEnum

from core.robot.robot_registry import RobotRegistry


class MotorKind(StrEnum):
    """모터 역할 — motors.yaml 의 `kind` 필드 SSOT.

    미래 확장 (tool_changer / linear_axis 등) 시 enum value 추가.
    """

    ARM = "arm"
    GRIPPER = "gripper"


@dataclass
class MotorConfig:
    id: int
    name: str
    model: str
    mode: str
    home: int
    limit_min: int
    limit_max: int
    reverse: bool
    kind: MotorKind = MotorKind.ARM
    pid_p: int | None = None
    pid_i: int | None = None
    pid_d: int | None = None


@dataclass
class PortConfig:
    windows: str
    linux: str

    def get(self) -> str:
        if sys.platform == "win32":
            return self.windows
        return self.linux


@dataclass
class ArmProfileConfig:
    """arm 모터들의 부드러운 motion profile baseline.

    motor_node start 시 적용 → slider/teleop slam 방지. TrajectoryRunner 가
    moveJ/L/C/P 진입 시 0,0 (= no cap, Ruckig 직접 명령) 으로 풀고 종료 시
    이 baseline 으로 복원.

    단위 — Feetech STS: velocity = steps/sec (1 step = 360°/4096),
           acceleration = 100·steps/sec². ex) 500/20 ≈ 44°/s, 176°/s² ramp.
    Dynamixel XL430: velocity = 0.229 rpm, acceleration = 214.577 rpm/s².
    """

    velocity: int
    acceleration: int


@dataclass
class MotorLayout:
    """robot 의 모터 layout — port + 전체 motors + arm/gripper 분류.

    `arm` / `gripper` 는 `motors` 위에서 `kind` 로 derive — 호출처가 매번
    `[m for m in motors if m.id != X]` 안 짜도 됨.

    invariant: gripper 는 정확히 1개. load 시 검증 (motors.yaml 에 kind=gripper
    행이 빠지거나 중복되면 fail-fast).
    """

    port: PortConfig
    motors: list[MotorConfig]
    arm_profile: ArmProfileConfig | None = None

    def __post_init__(self) -> None:
        grippers = [m for m in self.motors if m.kind == MotorKind.GRIPPER]
        if len(grippers) != 1:
            raise ValueError(
                f"motors.yaml: kind=gripper 가 정확히 1개여야 함 "
                f"(현재 {len(grippers)}개: {[m.name for m in grippers]})"
            )

    @property
    def arm(self) -> list[MotorConfig]:
        return [m for m in self.motors if m.kind != MotorKind.GRIPPER]

    @property
    def gripper(self) -> MotorConfig:
        return next(m for m in self.motors if m.kind == MotorKind.GRIPPER)


def load_motor_layout(robot_id: str | None = None) -> MotorLayout:
    """robot_type 의 motors.yaml + robot_id 의 instance.yaml 합쳐서 로드.

    type-level (motors 리스트, joint limit, gear ratio, PID, kind) → robot/<type>/motors.yaml
    instance-level (USB port, baud) → robot/instances/<robot_id>/instance.yaml
    multi_robot_architecture.md §5.1.2 split 기준.

    robot_id=None 이면 RobotRegistry().default() — Phase 1 single robot 편의.
    """
    cfg = RobotRegistry().default() if robot_id is None else RobotRegistry().get(robot_id)

    # type-level motors 로드
    with open(cfg.type_motors_yaml, "r", encoding="utf-8") as f:
        type_raw = yaml.safe_load(f)

    # instance-level port/baud 로드
    with open(cfg.instance_yaml, "r", encoding="utf-8") as f:
        inst_raw = yaml.safe_load(f)

    motor_inst = inst_raw["motor"]
    port = PortConfig(
        windows=motor_inst["port"]["windows"],
        linux=motor_inst["port"]["linux"],
    )

    arm_profile_raw = type_raw.get("arm_profile")
    arm_profile = (
        ArmProfileConfig(
            velocity=int(arm_profile_raw["velocity"]),
            acceleration=int(arm_profile_raw["acceleration"]),
        )
        if arm_profile_raw
        else None
    )

    motors: list[MotorConfig] = []
    for m in type_raw["motors"]:
        pid = m.get("pid") or {}
        motors.append(
            MotorConfig(
                id=m["id"],
                name=m["name"],
                model=m["model"],
                mode=m["mode"],
                home=m["home"],
                limit_min=m["limit"]["min"],
                limit_max=m["limit"]["max"],
                reverse=m.get("reverse", False),
                kind=MotorKind(m.get("kind", "arm")),
                pid_p=pid.get("p"),
                pid_i=pid.get("i"),
                pid_d=pid.get("d"),
            )
        )

    return MotorLayout(port=port, motors=motors, arm_profile=arm_profile)
