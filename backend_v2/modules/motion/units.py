"""raw ↔ rad 변환 — Motion 책임 (§4: Motor raw → joint rad = Motion 안).

MotorSpec(home / reverse / limit) 기준. joint_offset(calibration) 은 D4 에서
Mirror[Bundle] 로 추가될 자리 — 지금은 home 만 zero.

motors.yaml 의 arm 순서 == URDF kinematic chain 순서 == command 순서 가정
(so101: joint1..joint6). 어긋나면 joint name 매핑 필요 (현재 order 기반).
"""

from __future__ import annotations

import math

from modules.motor.layout import MotorSpec

_TWO_PI = 2.0 * math.pi
_RAW_RANGE = 4095


def raw_to_rad(raw: int, spec: MotorSpec) -> float:
    rad = (raw - spec.home) / _RAW_RANGE * _TWO_PI
    return -rad if spec.reverse else rad


def rad_to_raw(rad: float, spec: MotorSpec) -> int:
    if spec.reverse:
        rad = -rad
    raw = round(rad / _TWO_PI * _RAW_RANGE + spec.home)
    return max(spec.limit_min, min(spec.limit_max, raw))


def joints_raw_to_rad(positions_raw: list[int], arm: list[MotorSpec]) -> list[float]:
    return [raw_to_rad(positions_raw[i], arm[i]) for i in range(len(arm))]


def joints_rad_to_raw(angles_rad: list[float], arm: list[MotorSpec]) -> list[int]:
    return [rad_to_raw(angles_rad[i], arm[i]) for i in range(len(arm))]
