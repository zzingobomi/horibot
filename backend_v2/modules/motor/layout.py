from __future__ import annotations

from pydantic import BaseModel

from .contract import MotorKind


class MotorSpec(BaseModel):
    id: int
    name: str
    model: str  # STS3215 / STS3250 / XM430 ...
    kind: MotorKind = MotorKind.JOINT
    home: int  # raw 중심 (0..4095)
    limit_min: int
    limit_max: int
    reverse: bool = False
    velocity_dps: float
    acceleration_dpss: float
    # Position PID (motors.yaml `pid` 블록). Dynamixel = RAM 이라 driver 가
    # connect 마다 재적용 (전원 cycle 시 소실). Feetech STS = EEPROM 이라
    # 비워둠 (Wizard 로 한 번 굽고 끝 — so101 motors.yaml 주석 참조).
    pid_p: int | None = None
    pid_i: int | None = None
    pid_d: int | None = None

    @property
    def initial_raw(self) -> int:
        """
        실제로 사용할 수 있는 초기 모터 위치(raw 값)를 반환한다.

        home은 '이론적인 영점 위치'지만,
        일부 조인트는 물리적인 limit 범위 밖에 있어 그대로 사용할 수 없다.

        예: home=2048(0도 기준)인데 limit이 [58, 1991]이면
        그대로 시작하면 물리적으로 도달 불가능한 자세가 된다.

        그래서 home 값을 limit 범위 안으로 clamp 해서
        안전하게 시작 가능한 초기 위치를 만든다.
        """
        return max(self.limit_min, min(self.limit_max, self.home))
