import sys
import yaml
from pathlib import Path
from dataclasses import dataclass


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


def load_motor_config(path: str | Path | None = None) -> tuple[PortConfig, list[MotorConfig]]:
    if path is None:
        path = Path(__file__).parents[3] / "robot" / "config" / "motors.yaml"

    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    port = PortConfig(
        windows=raw["port"]["windows"],
        linux=raw["port"]["linux"],
    )

    motors = []
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
