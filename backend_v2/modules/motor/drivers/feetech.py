"""FeetechBackend — `scservo_sdk` 기반 실 MotorBackend (STS3215/3250).

옛 backend/modules/motor/adapters/feetech_driver.py 의 faithful port.
register map / sync read·write / EEPROM Lock 패턴 그대로, v2 MotorBackend
Protocol 형태(list[int] 정렬 + capabilities/topology self-declare)로만 재구성.

Register map (LeRobot tables.py 검증): Goal_Position=42(2B), Present_Position=56(2B),
Goal_Velocity=46(2B), Acceleration=41(1B), Torque_Enable=40, Present_Speed=58(2B signed),
Present_Load=60(2B signed). PROTOCOL_VERSION=0.

검증: 작성/import/type 은 회사, 실 모터 동작은 집 SO-101.
"""

from __future__ import annotations

import logging
import threading

from scservo_sdk import (  # type: ignore[import-untyped]
    COMM_SUCCESS,
    SCS_HIBYTE,
    SCS_LOBYTE,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

from ..contract import MotorCapabilities, MotorCapability, MotorInfo, MotorKind, MotorTopology
from ..layout import MotorSpec

logger = logging.getLogger(__name__)

# ─── Control Table (STS3215/3250, LeRobot 검증) ──────────────
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_VELOCITY = 46
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60

LEN_GOAL_POSITION = 2
LEN_PRESENT_POSITION = 2
LEN_PRESENT_LOAD = 2

PROTOCOL_VERSION = 0


class FeetechBackend:
    """scservo_sdk adapter — v2 MotorBackend Protocol 만족. STS = position loop only
    (current control X), software reboot 미지원 (전원 cycle 필요)."""

    def __init__(
        self, motors: list[MotorSpec], port: str, baudrate: int = 1_000_000
    ) -> None:
        self._motors = list(motors)  # 순서 = read/write list 정렬 기준
        self._by_id = {m.id: m for m in self._motors}
        self._motor_ids = [m.id for m in self._motors]
        self._port = port
        self._baudrate = baudrate

        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(PROTOCOL_VERSION)
        self._sync_write_goal: GroupSyncWrite | None = None
        self._sync_read_position: GroupSyncRead | None = None
        self._sync_read_load: GroupSyncRead | None = None
        self._lock = threading.Lock()

        # 마지막 known position (sync read 실패 모터 fallback) — home 초기화
        self._positions: list[int] = [m.home for m in self._motors]

    # ── self-declare (§7.3) ──

    def capabilities(self) -> MotorCapabilities:
        # STS: torque toggle 만. reboot(software) / current control 미지원.
        return MotorCapabilities(flags={MotorCapability.TORQUE_TOGGLE})

    def topology(self) -> MotorTopology:
        return MotorTopology(
            motors=[MotorInfo(id=m.id, kind=m.kind) for m in self._motors]
        )

    # ── lifecycle ──

    def open(self) -> None:
        if not self._port_handler.openPort():
            raise RuntimeError(f"Feetech 포트 open 실패: {self._port}")
        if not self._port_handler.setBaudRate(self._baudrate):
            raise RuntimeError(f"Feetech baudrate 설정 실패: {self._baudrate}")

        self._sync_write_goal = GroupSyncWrite(
            self._port_handler, self._packet_handler,
            ADDR_GOAL_POSITION, LEN_GOAL_POSITION,
        )
        self._sync_read_position = GroupSyncRead(
            self._port_handler, self._packet_handler,
            ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION,
        )
        self._sync_read_load = GroupSyncRead(
            self._port_handler, self._packet_handler,
            ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD,
        )
        for mid in self._motor_ids:
            self._sync_read_position.addParam(mid)
            self._sync_read_load.addParam(mid)
        logger.info("Feetech 연결: %s @ %d", self._port, self._baudrate)

    def close(self) -> None:
        try:
            self.set_torque(False)
        finally:
            self._port_handler.closePort()
        logger.info("Feetech 연결 종료")

    # ── read (motors 순서 정렬 list) ──

    def read_positions(self) -> list[int]:
        assert self._sync_read_position is not None, "open() 후 호출"
        with self._lock:
            result = self._sync_read_position.txRxPacket()
        if result == COMM_SUCCESS:
            for i, m in enumerate(self._motors):
                if self._sync_read_position.isAvailable(
                    m.id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                ):
                    self._positions[i] = self._sync_read_position.getData(
                        m.id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    )
        else:
            logger.warning("Feetech SyncRead(position) 실패 — 직전 값 유지")
        return list(self._positions)

    def read_velocities(self) -> list[int] | None:
        # TODO: GroupSyncRead 로 묶기. 지금은 모터별 single read (옛 backend 동일).
        out: list[int] = []
        with self._lock:
            for mid in self._motor_ids:
                val, result, _ = self._packet_handler.read2ByteTxRx(
                    self._port_handler, mid, ADDR_PRESENT_SPEED
                )
                out.append(_to_signed16(val) if result == COMM_SUCCESS else 0)
        return out

    def read_loads(self) -> list[int] | None:
        assert self._sync_read_load is not None, "open() 후 호출"
        with self._lock:
            result = self._sync_read_load.txRxPacket()
        if result != COMM_SUCCESS:
            logger.warning("Feetech SyncRead(load) 실패")
            return None
        out: list[int] = []
        for m in self._motors:
            raw = 0
            if self._sync_read_load.isAvailable(
                m.id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD
            ):
                raw = _to_signed16(
                    self._sync_read_load.getData(
                        m.id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD
                    )
                )
            out.append(raw)
        return out

    # ── write ──

    def set_torque(self, enabled: bool) -> None:
        val = 1 if enabled else 0
        for mid in self._motor_ids:
            self._write1(mid, ADDR_TORQUE_ENABLE, val)

    def get_torque_enabled(self) -> bool:
        # STS TORQUE_ENABLE = RAM register, 전원 on 시 0. 모든 모터 동시 set 이라
        # 첫 모터만 read (bus overhead 최소). 실패 시 안전한 False fallback.
        if not self._motor_ids:
            return False
        with self._lock:
            val, result, _ = self._packet_handler.read1ByteTxRx(
                self._port_handler, self._motor_ids[0], ADDR_TORQUE_ENABLE
            )
        return result == COMM_SUCCESS and val == 1

    def reboot(self) -> None:
        # STS = software reboot 명령 없음. 전원 cycle 필요.
        logger.warning("Feetech STS reboot 미지원 — 전원 cycle 필요")

    def set_gripper(self, position_raw: int) -> None:
        gripper = next(
            (m for m in self._motors if m.kind == MotorKind.GRIPPER), None
        )
        if gripper is None:
            return
        self._write2(gripper.id, ADDR_GOAL_POSITION, self._clamp(position_raw, gripper))

    def write_positions(self, positions_raw: list[int]) -> None:
        assert self._sync_write_goal is not None, "open() 후 호출"
        with self._lock:
            for i, pos in enumerate(positions_raw):
                if i >= len(self._motors):
                    break
                clamped = self._clamp(pos, self._motors[i])
                self._sync_write_goal.addParam(
                    self._motors[i].id, [SCS_LOBYTE(clamped), SCS_HIBYTE(clamped)]
                )
            result = self._sync_write_goal.txPacket()
            self._sync_write_goal.clearParam()
        if result != COMM_SUCCESS:
            logger.warning("Feetech SyncWrite(goal) 실패")

    # ── util ──

    @staticmethod
    def _clamp(pos: int, m: MotorSpec) -> int:
        pos = max(m.limit_min, min(m.limit_max, pos))
        if m.reverse:
            center = (m.limit_min + m.limit_max) // 2
            pos = center - (pos - center)
        return pos

    def _write1(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self._packet_handler.write1ByteTxRx(
                self._port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning("Feetech write1 실패 id=%d addr=%d", motor_id, addr)

    def _write2(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self._packet_handler.write2ByteTxRx(
                self._port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning("Feetech write2 실패 id=%d addr=%d", motor_id, addr)


def _to_signed16(raw: int) -> int:
    return raw - 65536 if raw >= 32768 else raw
