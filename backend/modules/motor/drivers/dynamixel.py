"""DynamixelBackend — `dynamixel_sdk` 기반 실 MotorBackend (XL430 / XL330).

옛 backend/modules/motor/adapters/{dynamixel_driver,dynamixel_backend}.py 의
faithful port. register map / sync read·write / 4-byte goal 패턴 그대로,
v2 MotorBackend Protocol 형태(list[int] 정렬 + capabilities/topology self-declare)
로만 재구성. 자매 driver feetech.py 와 동형 구조.

Register map (X-series Protocol 2.0, XL430/XL330 공통):
  Position_D/I/P_Gain=80/82/84(2B RAM), Profile_Acceleration=108(4B),
  Profile_Velocity=112(4B), Goal_Position=116(4B), Present_Load=126(2B signed),
  Present_Velocity=128(4B signed), Present_Position=132(4B). PROTOCOL_VERSION=2.0.

vendor 장점을 driver 내부에서 활용 (Protocol 밖으로 안 새어나감):
- **PID = RAM** — 전원 cycle 마다 소실 → open()/reboot() 마다 motors.yaml pid
  재적용 (중력 부하 J2/J3 P=1500 등, v1 motor_node._apply_position_pid 계승).
  Feetech STS 는 PID 가 EEPROM 이라 반대로 안 건드림 (Wizard 1회 굽기).
- **profile: arm=0 / gripper=yaml dps** — v2 arm 은 전부 100Hz 스트리밍(Ruckig
  소유)이라 servo profile 이 개입하면 추종 지연으로 싸움 → 0(무제한) 명시.
  gripper 는 SET_GRIPPER 단발 goal 이라 servo 보간이 유일한 slam-guard →
  motors.yaml `profile` dps 적용. 두 vendor 공통 패턴 (feetech 도 동일 정책,
  레지스터/단위만 다름).
- **reboot 지원** (STS 는 없음) → REBOOT capability. reboot 은 RAM 리셋이므로
  PID + profile 재적용까지가 한 동작 (v1 "_srv_reboot 후 _apply_profiles" 계승).

검증: 작성/import/type 은 회사, 실 모터 동작은 집 OMX_F.
"""

from __future__ import annotations

import logging
import threading
import time

from dynamixel_sdk import (  # type: ignore[import-untyped]
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

from ..contract import (
    MotorCapabilities,
    MotorCapability,
    MotorInfo,
    MotorKind,
    MotorTopology,
)
from ..layout import MotorSpec

logger = logging.getLogger(__name__)

# ─── Control Table (XL430 / XL330, Protocol 2.0) ──────────────
ADDR_TORQUE_ENABLE = 64
ADDR_POSITION_D_GAIN = 80  # RAM — 전원 cycle 소실, open 마다 재적용
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4
LEN_PRESENT_LOAD = 2

PROTOCOL_VERSION = 2.0

# ─── Profile dps ↔ raw 변환 (X-series, v1 검증값) ──────────────
#   Profile_Velocity unit = 0.229 rev/min = 1.374 °/s per raw
#   Profile_Acceleration unit = 214.577 rev/min² = 21.46 °/s² per raw
_VEL_DPS_PER_RAW = 1.374
_ACC_DPSS_PER_RAW = 21.46

# reboot 후 모터 재기동 대기 — RAM 레지스터 write 가 유효해지는 시점.
_REBOOT_SETTLE_S = 0.3


class DynamixelBackend:
    """dynamixel_sdk adapter — v2 MotorBackend Protocol 만족. open/reboot 마다
    PID(RAM) + profile(arm 0 / gripper dps) 적용, software reboot 지원."""

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

        # 마지막 known position (sync read 실패 모터 fallback) — 유효 초기 자세
        # (home 영점을 limit 안으로 clamp). 실 모터는 첫 sync read 로 덮어씀.
        self._positions: list[int] = [m.initial_raw for m in self._motors]

    # ── self-declare (§7.3) ──

    def capabilities(self) -> MotorCapabilities:
        # XL: torque toggle + software reboot. current control 미지원 (position loop).
        return MotorCapabilities(
            flags={MotorCapability.TORQUE_TOGGLE, MotorCapability.REBOOT}
        )

    def topology(self) -> MotorTopology:
        return MotorTopology(
            motors=[MotorInfo(id=m.id, kind=m.kind) for m in self._motors]
        )

    # ── lifecycle ──

    def open(self) -> None:
        if not self._port_handler.openPort():
            raise RuntimeError(f"Dynamixel 포트 open 실패: {self._port}")
        if not self._port_handler.setBaudRate(self._baudrate):
            raise RuntimeError(f"Dynamixel baudrate 설정 실패: {self._baudrate}")

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

        self._apply_pid()
        self._apply_profiles()
        logger.info("Dynamixel 연결: %s @ %d", self._port, self._baudrate)

    def close(self) -> None:
        try:
            self.set_torque(False)
        finally:
            self._port_handler.closePort()
        logger.info("Dynamixel 연결 종료")

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
            logger.warning("Dynamixel SyncRead(position) 실패 — 직전 값 유지")
        return list(self._positions)

    def read_velocities(self) -> list[int] | None:
        # TODO: GroupSyncRead 로 묶기. 지금은 모터별 single read (옛 backend 동일).
        out: list[int] = []
        with self._lock:
            for mid in self._motor_ids:
                val, result, _ = self._packet_handler.read4ByteTxRx(
                    self._port_handler, mid, ADDR_PRESENT_VELOCITY
                )
                out.append(_to_signed32(val) if result == COMM_SUCCESS else 0)
        return out

    def read_loads(self) -> list[int] | None:
        assert self._sync_read_load is not None, "open() 후 호출"
        with self._lock:
            result = self._sync_read_load.txRxPacket()
        if result != COMM_SUCCESS:
            logger.warning("Dynamixel SyncRead(load) 실패")
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

    # ── driver control state ──

    def get_torque_enabled(self) -> bool:
        # TORQUE_ENABLE = RAM register, 전원 on 시 0. 모든 모터 동시 set 이라 첫
        # 모터만 read (bus overhead 최소). 실패 시 안전한 False fallback.
        if not self._motor_ids:
            return False
        with self._lock:
            val, result, _ = self._packet_handler.read1ByteTxRx(
                self._port_handler, self._motor_ids[0], ADDR_TORQUE_ENABLE
            )
        return result == COMM_SUCCESS and val == 1

    # ── write ──

    def set_torque(self, enabled: bool) -> None:
        val = 1 if enabled else 0
        for mid in self._motor_ids:
            self._write1(mid, ADDR_TORQUE_ENABLE, val)

    def reboot(self) -> None:
        with self._lock:
            for mid in self._motor_ids:
                self._packet_handler.reboot(self._port_handler, mid)
        logger.info("Dynamixel reboot: %s", self._motor_ids)
        # reboot = RAM 리셋 (PID default 복귀, profile=0) → 재적용까지가 한 동작.
        time.sleep(_REBOOT_SETTLE_S)
        self._apply_pid()
        self._apply_profiles()

    def set_gripper(self, position_raw: int) -> None:
        gripper = next(
            (m for m in self._motors if m.kind == MotorKind.GRIPPER), None
        )
        if gripper is None:
            return
        self._write4(
            gripper.id, ADDR_GOAL_POSITION, self._clamp(position_raw, gripper)
        )

    def write_positions(self, positions_raw: list[int]) -> None:
        assert self._sync_write_goal is not None, "open() 후 호출"
        with self._lock:
            for i, pos in enumerate(positions_raw):
                if i >= len(self._motors):
                    break
                clamped = self._clamp(pos, self._motors[i])
                self._sync_write_goal.addParam(
                    self._motors[i].id, _int_to_4bytes(clamped)
                )
            result = self._sync_write_goal.txPacket()
            self._sync_write_goal.clearParam()
        if result != COMM_SUCCESS:
            logger.warning("Dynamixel SyncWrite(goal) 실패")

    # ── PID / profile 적용 (open + reboot 공용) ──

    def _apply_pid(self) -> None:
        """motors.yaml `pid` 재적용 — RAM 이라 전원 cycle 마다 소실 (v1 계승).

        pid 블록 없는 모터는 skip (servo default 유지).
        """
        applied = 0
        for m in self._motors:
            if m.pid_p is None and m.pid_i is None and m.pid_d is None:
                continue
            if m.pid_d is not None:
                self._write2(m.id, ADDR_POSITION_D_GAIN, m.pid_d)
            if m.pid_i is not None:
                self._write2(m.id, ADDR_POSITION_I_GAIN, m.pid_i)
            if m.pid_p is not None:
                self._write2(m.id, ADDR_POSITION_P_GAIN, m.pid_p)
            applied += 1
        if applied:
            logger.info("Dynamixel PID 적용: %d개 (motors.yaml `pid`)", applied)

    def _apply_profiles(self) -> None:
        """profile 정책 — arm=0 (Ruckig 100Hz 소유, servo profile 개입 차단) /
        gripper=motors.yaml dps (SET_GRIPPER 단발 goal 의 slam-guard)."""
        for m in self._motors:
            if m.kind == MotorKind.GRIPPER:
                vel = max(0, round(m.velocity_dps / _VEL_DPS_PER_RAW))
                acc = max(0, round(m.acceleration_dpss / _ACC_DPSS_PER_RAW))
            else:
                vel, acc = 0, 0  # 0 = 무제한 (프로파일 없음)
            self._write4(m.id, ADDR_PROFILE_ACCELERATION, acc)
            self._write4(m.id, ADDR_PROFILE_VELOCITY, vel)
        logger.info(
            "Dynamixel profile 적용: arm=0(스트리밍 소유) / gripper=yaml dps"
        )

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
            logger.warning("Dynamixel write1 실패 id=%d addr=%d", motor_id, addr)

    def _write2(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self._packet_handler.write2ByteTxRx(
                self._port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning("Dynamixel write2 실패 id=%d addr=%d", motor_id, addr)

    def _write4(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self._packet_handler.write4ByteTxRx(
                self._port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning("Dynamixel write4 실패 id=%d addr=%d", motor_id, addr)


def _int_to_4bytes(value: int) -> list[int]:
    return [
        DXL_LOBYTE(DXL_LOWORD(value)),
        DXL_HIBYTE(DXL_LOWORD(value)),
        DXL_LOBYTE(DXL_HIWORD(value)),
        DXL_HIBYTE(DXL_HIWORD(value)),
    ]


def _to_signed16(raw: int) -> int:
    return raw - 65536 if raw >= 32768 else raw


def _to_signed32(raw: int) -> int:
    return raw - 4294967296 if raw >= 2147483648 else raw
