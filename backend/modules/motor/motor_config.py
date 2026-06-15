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
class MotorProfile:
    """모터 register 의 motion profile (slam guard, slider/teleop 안전망).

    physical 단위 (°/s, °/s²) — vendor 별 단위 차이는 MotorBackend adapter 가 흡수.
    OMX (Dynamixel X-series) / SO-101 (Feetech STS) 가 같은 dps 숫자로 같은 물리
    동작 — multi_robot_architecture.md §3.3 의 "wire protocol 차이 흡수" 가 단위
    레벨까지 확장된 자리.

    TrajectoryRunner 가 moveJ/L/C/P 진입 시 release_profile() (raw 0,0) 로 풀고
    종료 시 restore_profile() 로 모터마다 이 값을 복원 — 그 사이 Ruckig 가 직접
    명령 (motor cap 없이 trajectory shape 보존).
    """

    velocity_dps: float
    acceleration_dpss: float


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
    profile: MotorProfile | None = None
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
class MotorLayout:
    """robot 의 모터 layout — port + 전체 motors + arm/gripper 분류.

    `arm` / `gripper` 는 `motors` 위에서 `kind` 로 derive — 호출처가 매번
    `[m for m in motors if m.id != X]` 안 짜도 됨.

    invariant: gripper 는 정확히 1개. load 시 검증 (motors.yaml 에 kind=gripper
    행이 빠지거나 중복되면 fail-fast).
    """

    port: PortConfig
    motors: list[MotorConfig]

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

    type-level (motors 리스트, joint limit, gear ratio, PID, kind, profile) → robot/<type>/motors.yaml
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

    motors: list[MotorConfig] = []
    for m in type_raw["motors"]:
        pid = m.get("pid") or {}
        profile_raw = m.get("profile")
        profile = (
            MotorProfile(
                velocity_dps=float(profile_raw["velocity_dps"]),
                acceleration_dpss=float(profile_raw["acceleration_dpss"]),
            )
            if profile_raw
            else None
        )
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
                profile=profile,
                pid_p=pid.get("p"),
                pid_i=pid.get("i"),
                pid_d=pid.get("d"),
            )
        )

    return MotorLayout(port=port, motors=motors)
