"""Robot motion planning 설정 loader.

robot/<robot_type>/motion.yaml 의 single source of truth — Ruckig (TrajectoryRunner)
가 moveJ / moveL / moveC / moveP 보간 시 사용하는 per-joint / cartesian 한계.

motors.yaml 의 `profile` (motor register slam guard) 와 다른 layer:
- motor profile = vendor register, slider/teleop 안전망 (TrajectoryRunner 가 푼다)
- motion limit = Ruckig 입력 (moveJ/L 도중의 진짜 속도)

산업 표준 (MoveIt joint_limits.yaml + Pilz cartesian_limits.yaml) 의 한 파일 통합
변형 — dict-by-joint-name 으로 5DOF/6DOF 추가 시 array length shift 없음.
multi_robot_architecture.md §3 의 "robot 무관 같은 코드 경로" 약속을 위해 필요.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class JointMotionLimit:
    """per-joint Ruckig 한계 (SI: rad/s, rad/s², rad/s³)."""

    max_velocity: float
    max_acceleration: float
    max_jerk: float


@dataclass(frozen=True)
class CartesianMotionLimit:
    """Cartesian (TCP) Ruckig 한계.

    저속 (<0.08 m/s) 에서 J3 P=1500 stick-slip chatter 회피용 최소 0.10 m/s 권장.
    """

    max_trans_vel: float
    max_trans_acc: float
    max_trans_jerk: float
    # 회전 한계는 5DOF position-only IK 시 미사용 — 6DOF 도착 시 활용.
    max_rot_vel: float = 1.57
    max_rot_acc: float = 4.0
    max_rot_jerk: float = 10.0


@dataclass(frozen=True)
class MotionConfig:
    """robot 1개의 motion planning 한계 — TrajectoryRunner ctor 입력 SSOT."""

    joint_limits: dict[str, JointMotionLimit]
    cartesian_limits: CartesianMotionLimit


def load_motion_config(motion_yaml_path: Path) -> MotionConfig:
    """robot/<type>/motion.yaml 로드. 파일 없거나 schema 어긋나면 ValueError."""
    if not motion_yaml_path.exists():
        raise FileNotFoundError(
            f"motion.yaml 없음: {motion_yaml_path}. robot 마다 motion.yaml 필수."
        )
    with open(motion_yaml_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ValueError(f"motion.yaml: top-level 이 dict 아님 ({motion_yaml_path})")

    joints_raw = raw.get("joint_limits")
    if not isinstance(joints_raw, dict) or not joints_raw:
        raise ValueError(
            f"motion.yaml: 'joint_limits' (dict by joint name) 누락 ({motion_yaml_path})"
        )
    joint_limits: dict[str, JointMotionLimit] = {}
    for name, entry in joints_raw.items():
        if not isinstance(entry, dict):
            raise ValueError(
                f"motion.yaml: joint_limits['{name}'] 가 dict 아님 ({motion_yaml_path})"
            )
        joint_limits[str(name)] = JointMotionLimit(
            max_velocity=float(entry["max_velocity"]),
            max_acceleration=float(entry["max_acceleration"]),
            max_jerk=float(entry["max_jerk"]),
        )

    cart_raw = raw.get("cartesian_limits")
    if not isinstance(cart_raw, dict):
        raise ValueError(
            f"motion.yaml: 'cartesian_limits' (dict) 누락 ({motion_yaml_path})"
        )
    cartesian = CartesianMotionLimit(
        max_trans_vel=float(cart_raw["max_trans_vel"]),
        max_trans_acc=float(cart_raw["max_trans_acc"]),
        max_trans_jerk=float(cart_raw["max_trans_jerk"]),
        max_rot_vel=float(cart_raw.get("max_rot_vel", 1.57)),
        max_rot_acc=float(cart_raw.get("max_rot_acc", 4.0)),
        max_rot_jerk=float(cart_raw.get("max_rot_jerk", 10.0)),
    )

    return MotionConfig(joint_limits=joint_limits, cartesian_limits=cartesian)
