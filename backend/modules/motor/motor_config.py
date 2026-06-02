import sys
import yaml
from dataclasses import dataclass
from pathlib import Path

from core.robot.robot_registry import RobotRegistry


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


def load_motor_config(
    robot_id: str | None = None,
) -> tuple[PortConfig, list[MotorConfig]]:
    """robot_type 의 motors.yaml + robot_id 의 instance.yaml 합쳐서 로드.

    type-level (motors 리스트, joint limit, gear ratio, PID 등) → robot/<type>/motors.yaml
    instance-level (USB port, baud) → robot/instances/<robot_id>/instance.yaml
    multi_robot_architecture.md §5.1.2 split 기준.

    robot_id=None 이면 RobotRegistry().default() — Phase 1 single robot.
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
                pid_p=pid.get("p"),
                pid_i=pid.get("i"),
                pid_d=pid.get("d"),
            )
        )

    return port, motors


def _legacy_load_motor_config_from_path(
    path: str | Path,
) -> tuple[PortConfig, list[MotorConfig]]:
    """legacy single-file motors.yaml loader. 더 이상 사용 X — RobotRegistry-based 사용.

    호환 위해 남겨두고 후속 정리에서 제거. 호출처가 직접 path 를 넘기던 경우만.
    """
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    port = PortConfig(
        windows=raw["port"]["windows"],
        linux=raw["port"]["linux"],
    )

    motors: list[MotorConfig] = []
    for m in raw["motors"]:
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
                pid_p=pid.get("p"),
                pid_i=pid.get("i"),
                pid_d=pid.get("d"),
            )
        )

    return port, motors
