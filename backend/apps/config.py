"""Config 모델 + 로더.

두 출처:
- **robot/ 트리** (top-level, v2 가 canonical 소유) — robot 데이터 SSOT.
  `robots.yaml`(registry) + `<type>/motors.yaml`(모터 레이아웃) +
  `instances/<id>/instance.yaml`(port/baud). 옛 RobotRegistry 코드는 안 씀 —
  v2 자체 loader (깨끗한 데이터만 재사용, 옛 아키텍처 X).
- **backend/config/deployments/** — v2 배포 토폴로지 (host→module, driver_mode).

calibration 도메인 파라미터 중:
- `sag_joint_motor_ids` 는 로봇 타입의 **물리 모델 사실** (어느 관절이 중력 sag 를
  받나) → `<type>/physical.yaml` 로 승격 (calibration 값이 아니라 robot physical
  model — Motion sag decorator + offline BA 공용). FkChain 도입과 함께 자리 잡음.
- `pose_recommend_strategy` / `wrist_roll_motor_id` 는 캘 전략이라 Calibration
  Module config 자리 (Step E 재배치) — lean read 에서 여전히 무시.
"""

from __future__ import annotations

import sys
from enum import StrEnum
from pathlib import Path

import yaml
from pydantic import BaseModel, Field

from modules.motor.contract import MotorKind
from modules.motor.layout import MotorSpec


_ROBOT_DIR = Path(__file__).resolve().parents[2] / "robot"


# ─── robot/ 트리 — robot 데이터 SSOT ────────────────────────────


class BasePose(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0


class JointMotionLimit(BaseModel):
    """Ruckig 한계 (rad/s, rad/s², rad/s³) — <type>/motion.yaml."""

    max_velocity: float
    max_acceleration: float
    max_jerk: float


class CartesianLimit(BaseModel):
    max_trans_vel: float = 0.1
    max_trans_acc: float = 0.3
    max_trans_jerk: float = 3.0


class RobotConfig(BaseModel):
    """robot 1개 — registry(robots.yaml) + 모터 레이아웃(motors.yaml) +
    instance(port/baud) 합본. v2 의 lean view (calib 파라미터 제외)."""

    id: str
    type: str
    enabled: bool = True
    capabilities: list[str] = Field(default_factory=list)
    motor_backend: str  # vendor — feetech / dynamixel
    camera_backend: str | None = None  # realsense / opencv
    base_pose: BasePose = Field(default_factory=BasePose)
    # instance-level (instances/<id>/instance.yaml, platform 별 port 해소)
    motor_port: str | None = None
    motor_baudrate: int | None = None
    # instance-level — UVC 카메라 cv2.VideoCapture 인덱스 (opencv backend 만 사용)
    camera_device_index: int | None = None
    # type-level (<type>/motors.yaml)
    motors: list[MotorSpec] = Field(default_factory=list)
    # type-level (<type>/motion.yaml) — joint name 별 Ruckig 한계 + cartesian
    motion_joint_limits: dict[str, JointMotionLimit] = Field(
        default_factory=dict)
    cartesian_limits: CartesianLimit = Field(default_factory=CartesianLimit)
    # type-level (<type>/physical.yaml) — robot physical model (모델링 선택만).
    # 중력 sag lumped-mass 모델을 적용할 관절 (motor id). Motion sag decorator +
    # offline BA 공용. 빈 list = sag 모델 없음.
    sag_joint_motor_ids: list[int] = Field(default_factory=list)


def _resolve_port(instance_raw: dict) -> tuple[str | None, int | None]:
    motor = instance_raw.get("motor") or {}
    port = motor.get("port") or {}
    key = "windows" if sys.platform.startswith("win") else "linux"
    return port.get(key), motor.get("baudrate")


def _load_motors(robot_type: str, robot_dir: Path) -> list[MotorSpec]:
    raw = yaml.safe_load(
        (robot_dir / robot_type / "motors.yaml").read_text(encoding="utf-8")
    )
    specs: list[MotorSpec] = []
    for m in raw.get("motors") or []:
        limit = m.get("limit") or {}
        profile = m.get("profile") or {}
        pid = m.get("pid") or {}
        kind = MotorKind(m["kind"]) if m.get("kind") else MotorKind.JOINT
        specs.append(
            MotorSpec(
                id=m["id"],
                name=m["name"],
                model=m["model"],
                kind=kind,
                home=m["home"],
                limit_min=limit["min"],
                limit_max=limit["max"],
                reverse=m.get("reverse", False),
                velocity_dps=profile.get("velocity_dps", 0.0),
                acceleration_dpss=profile.get("acceleration_dpss", 0.0),
                # pid 블록 없으면 None — Dynamixel(RAM) 만 driver 가 재적용,
                # Feetech(EEPROM) 는 yaml 자체가 pid 를 안 적음 (Wizard 1회).
                pid_p=pid.get("p"),
                pid_i=pid.get("i"),
                pid_d=pid.get("d"),
            )
        )
    return specs


def _load_motion(
    robot_type: str, robot_dir: Path
) -> tuple[dict[str, JointMotionLimit], CartesianLimit]:
    path = robot_dir / robot_type / "motion.yaml"
    if not path.exists():
        return {}, CartesianLimit()
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    joints = {
        name: JointMotionLimit(**lim)
        for name, lim in (raw.get("joint_limits") or {}).items()
    }
    cart = CartesianLimit(**(raw.get("cartesian_limits") or {}))
    return joints, cart


def _load_physical(robot_type: str, robot_dir: Path) -> list[int]:
    """<type>/physical.yaml 의 sag_joint_motor_ids (없으면 빈 list)."""
    path = robot_dir / robot_type / "physical.yaml"
    if not path.exists():
        return []
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return list(raw.get("sag_joint_motor_ids") or [])


def load_robots(robot_dir: Path = _ROBOT_DIR) -> dict[str, RobotConfig]:
    raw = yaml.safe_load(
        (robot_dir / "robots.yaml").read_text(encoding="utf-8"))
    robots: dict[str, RobotConfig] = {}
    for rid, body in (raw.get("robots") or {}).items():
        rtype = body["type"]
        port, baud = None, None
        camera_device_index = None
        inst_path = robot_dir / "instances" / rid / "instance.yaml"
        if inst_path.exists():
            inst_raw = yaml.safe_load(inst_path.read_text(encoding="utf-8"))
            port, baud = _resolve_port(inst_raw)
            camera_device_index = (inst_raw.get(
                "camera") or {}).get("device_index")
        motion_joints, cartesian = _load_motion(rtype, robot_dir)
        bp = body.get("base_pose") or {}
        robots[rid] = RobotConfig(
            id=rid,
            type=rtype,
            enabled=body.get("enabled", True),
            capabilities=list(body.get("capabilities") or []),
            motor_backend=body["motor_backend"],
            camera_backend=body.get("camera_backend"),
            base_pose=BasePose(**bp),
            motor_port=port,
            motor_baudrate=baud,
            camera_device_index=camera_device_index,
            motors=_load_motors(rtype, robot_dir),
            motion_joint_limits=motion_joints,
            cartesian_limits=cartesian,
            sag_joint_motor_ids=_load_physical(rtype, robot_dir),
        )
    return robots


# ─── deployment yaml (backend/config/deployments/) ───────────


class DriverMode(StrEnum):
    REAL = "real"
    MOCK = "mock"


class ModuleEntry(BaseModel):
    name: str
    robots: list[str] = Field(default_factory=list)


class DeploymentConfig(BaseModel):
    driver_mode: DriverMode = DriverMode.REAL
    zenoh: dict = Field(default_factory=dict)
    modules: list[ModuleEntry] = Field(default_factory=list)
    rdb_uri: str | None = None
    object_uri: str | None = None
    bridge_port: int = 8000
    dev_console: bool = False  # 개발용 콘솔
    # detector GDINO 합동 추론 (멀티 프롬프트 1-forward — pose 당 추론 N→1회).
    # 합동 쿼리는 단독과 score 분포가 다를 수 있어 기본 off — 켜기 전
    # scripts/compare_joint_prompt_scores.py 로 실물 덤프 분포 확인 (task 의
    # _PICK_SCORE_MIN 컷 마진과 비교). drivers/grounded_sam docstring 참조.
    detector_joint_inference: bool = False


def load_deployment(path: Path | str) -> DeploymentConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return DeploymentConfig.model_validate(raw)
