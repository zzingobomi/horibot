"""MotorSpec — type-level 모터 1개 스펙 (robot/<type>/motors.yaml 한 줄).

Module SDK internal — driver 가 받아서 topology / raw 변환 / motion profile 에
사용. apps.config 가 motors.yaml 을 읽어 list[MotorSpec] 로 만들어 resolve_deps
가 driver 에 주입한다 (mock / feetech 공통).

backend_v2_modules.md §7.4 — topology 값 SSOT = driver self-declare. 그 driver
가 self-declare 하는 근거 데이터가 이 spec (robot type 의 정적 사실).
"""

from __future__ import annotations

from pydantic import BaseModel

from .contract import MotorKind


class MotorSpec(BaseModel):
    """모터 1개 — id / kind / raw 변환(home·limit) / motion profile."""

    id: int
    name: str
    model: str  # STS3215 / STS3250 / XM430 ...
    kind: MotorKind = MotorKind.JOINT
    home: int  # raw 중심 (0..4095)
    limit_min: int
    limit_max: int
    reverse: bool = False
    # motion slam-guard profile (Step D Motion 에서 사용). dps SSOT.
    velocity_dps: float
    acceleration_dpss: float
