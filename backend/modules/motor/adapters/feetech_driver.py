"""FeetechDriver — `scservo_sdk` 기반 raw SDK wrap.

DynamixelDriver (adapters/dynamixel_driver.py) 와 동형 — 같은 method 시그니처 + 동작.
STS3215/3250 (protocol 0) 모터 sync read/write.

so101_6dof_plan.md §6-1 / §6-6 + LeRobot motors/feetech 검증.
Register map: LeRobot tables.py 검증 — Goal_Position=42(2byte), Present_Position=56(2byte),
Goal_Velocity=46(2byte), Acceleration=41(1byte), P/D/I=21/22/23(1byte EEPROM),
Torque_Enable=40, Lock=55, Operating_Mode=33(EEPROM, 0=position).

EEPROM write (PID / Limit / Mode 등) 시 Lock register unlock→write→lock 패턴 필수.
"""

from __future__ import annotations

import logging
import threading

from scservo_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncRead,
    GroupSyncWrite,
    COMM_SUCCESS,
    SCS_LOBYTE,
    SCS_HIBYTE,
)

from modules.motor.motor_config import MotorConfig

logger = logging.getLogger(__name__)

# ─── Control Table (STS3215 / STS3250 공통, LeRobot tables.py 검증) ──────────

# EEPROM (전원 사이클 유지)
ADDR_ID = 5
ADDR_BAUD_RATE = 6
ADDR_MIN_POSITION_LIMIT = 9       # 2byte
ADDR_MAX_POSITION_LIMIT = 11      # 2byte
ADDR_P_COEFFICIENT = 21           # 1byte
ADDR_D_COEFFICIENT = 22           # 1byte
ADDR_I_COEFFICIENT = 23           # 1byte
ADDR_OPERATING_MODE = 33          # 1byte (0=Position Servo)

# SRAM
ADDR_TORQUE_ENABLE = 40           # 1byte (0=off, 1=on)
ADDR_ACCELERATION = 41            # 1byte (= profile acceleration)
ADDR_GOAL_POSITION = 42           # 2byte
ADDR_GOAL_VELOCITY = 46           # 2byte (= profile velocity, max speed)
ADDR_LOCK = 55                    # 1byte (0=unlock, 1=lock) — EEPROM write 시 unlock 필요
ADDR_PRESENT_POSITION = 56        # 2byte (read-only)
ADDR_PRESENT_SPEED = 58           # 2byte signed
ADDR_PRESENT_LOAD = 60            # 2byte signed (current proxy)

# Lengths
LEN_GOAL_POSITION = 2
LEN_PRESENT_POSITION = 2
LEN_GOAL_VELOCITY = 2
LEN_ACCELERATION = 1
LEN_PRESENT_LOAD = 2

PROTOCOL_VERSION = 0              # STS series
BAUDRATE = 1_000_000              # 1Mbps default

LOCK_UNLOCK = 0
LOCK_LOCK = 1


class FeetechDriver:
    """`scservo_sdk` adapter — DynamixelDriver 와 동형 API.

    register length 차이 (Dynamixel 4byte vs STS 2byte) + protocol 0 + EEPROM Lock
    패턴 외에는 호출 시그니처 그대로.
    """

    def __init__(self, port: str, motors: list[MotorConfig]):
        self.port = port
        self.motors = {m.id: m for m in motors}
        self.motor_ids = [m.id for m in motors]

        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self._sync_write_goal: GroupSyncWrite | None = None
        self._sync_write_velocity: GroupSyncWrite | None = None
        self._sync_write_accel: GroupSyncWrite | None = None
        self._sync_read_position: GroupSyncRead | None = None
        self._sync_read_load: GroupSyncRead | None = None
        self._lock = threading.Lock()

    # ─── 연결 ────────────────────────────────────────────────

    def connect(self) -> bool:
        if not self.port_handler.openPort():
            logger.error(f"포트를 열 수 없습니다: {self.port}")
            return False

        if not self.port_handler.setBaudRate(BAUDRATE):
            logger.error(f"Baudrate 설정 실패: {BAUDRATE}")
            return False

        self._sync_write_goal = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_GOAL_POSITION, LEN_GOAL_POSITION,
        )
        self._sync_write_velocity = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY,
        )
        self._sync_write_accel = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_ACCELERATION, LEN_ACCELERATION,
        )
        self._sync_read_position = GroupSyncRead(
            self.port_handler, self.packet_handler,
            ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION,
        )
        for mid in self.motor_ids:
            self._sync_read_position.addParam(mid)

        self._sync_read_load = GroupSyncRead(
            self.port_handler, self.packet_handler,
            ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD,
        )
        for mid in self.motor_ids:
            self._sync_read_load.addParam(mid)

        logger.info(f"Feetech 연결 성공: {self.port}")
        return True

    def disconnect(self) -> None:
        self.torque_disable_all()
        self.port_handler.closePort()
        logger.info("Feetech 연결 종료")

    # ─── 토크 제어 ────────────────────────────────────────────

    def torque_enable(self, motor_id: int) -> None:
        self._write1(motor_id, ADDR_TORQUE_ENABLE, 1)

    def torque_disable(self, motor_id: int) -> None:
        self._write1(motor_id, ADDR_TORQUE_ENABLE, 0)

    def torque_enable_all(self) -> None:
        for mid in self.motor_ids:
            self.torque_enable(mid)

    def torque_disable_all(self) -> None:
        for mid in self.motor_ids:
            self.torque_disable(mid)

    # ─── 위치 제어 ────────────────────────────────────────────

    def set_goal_position(self, motor_id: int, position: int) -> None:
        pos = self._apply_limits(position, self.motors[motor_id])
        self._write2(motor_id, ADDR_GOAL_POSITION, pos)

    def set_goal_positions_sync(self, positions: dict[int, int]) -> None:
        assert self._sync_write_goal is not None
        with self._lock:
            for mid, pos in positions.items():
                pos = self._apply_limits(pos, self.motors[mid])
                param = self._int_to_2bytes(pos)
                self._sync_write_goal.addParam(mid, param)
            result = self._sync_write_goal.txPacket()
            self._sync_write_goal.clearParam()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncWrite(goal) 실패: {self.packet_handler.getTxRxResult(result)}")

    def get_present_positions(self) -> dict[int, int]:
        assert self._sync_read_position is not None
        with self._lock:
            result = self._sync_read_position.txRxPacket()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncRead 실패: {self.packet_handler.getTxRxResult(result)}")
            return {}
        positions = {}
        for mid in self.motor_ids:
            if self._sync_read_position.isAvailable(
                mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                positions[mid] = self._sync_read_position.getData(
                    mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                )
        return positions

    def get_present_velocity(self, motor_id: int) -> int:
        with self._lock:
            val, result, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_SPEED
            )
        return val if result == COMM_SUCCESS else 0

    def get_present_load(self, motor_id: int) -> int:
        with self._lock:
            val, result, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_LOAD
            )
        return val if result == COMM_SUCCESS else 0

    def get_present_loads(self) -> dict[int, int]:
        """전체 모터의 Present_Load sync read. STS = signed 2byte."""
        assert self._sync_read_load is not None
        with self._lock:
            result = self._sync_read_load.txRxPacket()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncRead(load) 실패: {self.packet_handler.getTxRxResult(result)}")
            return {}
        loads: dict[int, int] = {}
        for mid in self.motor_ids:
            if self._sync_read_load.isAvailable(
                mid, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD
            ):
                raw = self._sync_read_load.getData(
                    mid, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD
                )
                # 2byte signed 변환
                if raw >= 32768:
                    raw -= 65536
                loads[mid] = raw
        return loads

    # ─── Gripper (current-based) — STS 미지원 ──────────────

    def set_goal_current(self, motor_id: int, current: int) -> None:
        """STS = 단일 position loop, current control 모드 없음. no-op.

        DynamixelBackend.set_goal_current 와 인터페이스 통일 위해 메서드 존재.
        그리퍼 force control 은 별도 모드 (mode=5) 인 Dynamixel 과 달리
        STS 는 일반 position mode 로 운영 (so101_6dof_plan §6).
        """
        del motor_id, current  # unused

    # ─── Position PID gain (EEPROM, Lock unlock 필수) ──────

    def set_position_pid(
        self,
        motor_id: int,
        p: int | None = None,
        i: int | None = None,
        d: int | None = None,
    ) -> None:
        """STS PID 는 EEPROM 영역 (P=21, D=22, I=23, 각 1byte 0~255).
        Lock unlock → write → lock 패턴. None 값은 skip.
        caller (FeetechBackend.configure_pid) 가 read-first-then-write 로 EEPROM
        wear 방지.
        """
        if p is None and i is None and d is None:
            return
        self._write1(motor_id, ADDR_LOCK, LOCK_UNLOCK)
        try:
            if p is not None:
                self._write1(motor_id, ADDR_P_COEFFICIENT, int(p))
            if d is not None:
                self._write1(motor_id, ADDR_D_COEFFICIENT, int(d))
            if i is not None:
                self._write1(motor_id, ADDR_I_COEFFICIENT, int(i))
        finally:
            self._write1(motor_id, ADDR_LOCK, LOCK_LOCK)

    def read_position_pid(self, motor_id: int) -> tuple[int, int, int]:
        """현재 EEPROM PID (P, I, D). read-first-then-write 비교 용."""
        with self._lock:
            p_val, r1, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, ADDR_P_COEFFICIENT
            )
            d_val, r2, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, ADDR_D_COEFFICIENT
            )
            i_val, r3, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, ADDR_I_COEFFICIENT
            )
        return (
            p_val if r1 == COMM_SUCCESS else 0,
            i_val if r3 == COMM_SUCCESS else 0,
            d_val if r2 == COMM_SUCCESS else 0,
        )

    # ─── 프로파일 (Goal Velocity / Acceleration) ────────────

    def set_profile_velocity(self, motor_id: int, velocity: int) -> None:
        self._write2(motor_id, ADDR_GOAL_VELOCITY, velocity)

    def set_profile_acceleration(self, motor_id: int, acceleration: int) -> None:
        # STS Acceleration = 1byte (0~255)
        self._write1(
            motor_id, ADDR_ACCELERATION, min(255, max(0, acceleration))
        )

    def set_profile_velocities_sync(self, velocities: dict[int, int]) -> None:
        assert self._sync_write_velocity is not None
        with self._lock:
            for mid, vel in velocities.items():
                self._sync_write_velocity.addParam(mid, self._int_to_2bytes(vel))
            result = self._sync_write_velocity.txPacket()
            self._sync_write_velocity.clearParam()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncWrite(velocity) 실패: {self.packet_handler.getTxRxResult(result)}")

    def set_profile_accelerations_sync(
        self, accelerations: dict[int, int]
    ) -> None:
        assert self._sync_write_accel is not None
        with self._lock:
            for mid, acc in accelerations.items():
                acc_clamp = min(255, max(0, acc))
                self._sync_write_accel.addParam(mid, [acc_clamp])
            result = self._sync_write_accel.txPacket()
            self._sync_write_accel.clearParam()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncWrite(acceleration) 실패: {self.packet_handler.getTxRxResult(result)}")

    # ─── 재시작 — STS = software reboot 명령 없음 ────────────

    def reboot(self, motor_id: int) -> None:
        logger.warning(
            f"Feetech STS reboot — software 명령 미지원. "
            f"모터 {motor_id} 전원 cycle 필요"
        )

    # ─── Util ────────────────────────────────────────────────

    def _apply_limits(self, pos: int, cfg: MotorConfig) -> int:
        pos = max(cfg.limit_min, min(cfg.limit_max, pos))
        if cfg.reverse:
            center = (cfg.limit_min + cfg.limit_max) // 2
            pos = center - (pos - center)
        return pos

    @staticmethod
    def _int_to_2bytes(value: int) -> list[int]:
        return [SCS_LOBYTE(value), SCS_HIBYTE(value)]

    def _write1(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning(
                f"write1 실패 id={motor_id} addr={addr}: "
                f"{self.packet_handler.getTxRxResult(result)}"
            )

    def _write2(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning(
                f"write2 실패 id={motor_id} addr={addr}: "
                f"{self.packet_handler.getTxRxResult(result)}"
            )
