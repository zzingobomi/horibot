import logging
import threading
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    GroupSyncRead,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)

from modules.dynamixel.motor_config import MotorConfig

logger = logging.getLogger(__name__)

# ─── Control Table (XL430 / XL330 공통) ───────────────────────
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

LEN_POSITION_GAIN = 2
LEN_GOAL_CURRENT = 2
LEN_PROFILE_ACCEL = 4
LEN_PROFILE_VELOCITY = 4
LEN_GOAL_POSITION = 4
LEN_PRESENT_LOAD = 2
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_POSITION = 4

OPERATING_MODE_POSITION = 3
PROTOCOL_VERSION = 2.0
BAUDRATE = 1_000_000

# TODO: IO Thread + Queue 구조로 변경 검토하기


class DynamixelDriver:
    def __init__(self, port: str, motors: list[MotorConfig]):
        self.port = port
        self.motors = {m.id: m for m in motors}
        self.motor_ids = [m.id for m in motors]

        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self._sync_write_goal:        GroupSyncWrite | None = None
        self._sync_write_profile_vel: GroupSyncWrite | None = None
        self._sync_write_profile_acc: GroupSyncWrite | None = None
        self._sync_read_present:      GroupSyncRead | None = None
        self._sync_read_load:         GroupSyncRead | None = None
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
        self._sync_write_profile_vel = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY,
        )
        self._sync_write_profile_acc = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_PROFILE_ACCELERATION, LEN_PROFILE_ACCEL,
        )
        self._sync_read_present = GroupSyncRead(
            self.port_handler, self.packet_handler,
            ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION,
        )
        for mid in self.motor_ids:
            self._sync_read_present.addParam(mid)

        self._sync_read_load = GroupSyncRead(
            self.port_handler, self.packet_handler,
            ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD,
        )
        for mid in self.motor_ids:
            self._sync_read_load.addParam(mid)

        logger.info(f"Dynamixel 연결 성공: {self.port}")
        return True

    def disconnect(self) -> None:
        self.torque_disable_all()
        self.port_handler.closePort()
        logger.info("Dynamixel 연결 종료")

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
        self._write4(motor_id, ADDR_GOAL_POSITION, pos)

    def set_goal_positions_sync(self, positions: dict[int, int]) -> None:
        assert self._sync_write_goal is not None
        with self._lock:
            for mid, pos in positions.items():
                pos = self._apply_limits(pos, self.motors[mid])
                param = self._int_to_4bytes(pos)
                self._sync_write_goal.addParam(mid, param)
            result = self._sync_write_goal.txPacket()
            self._sync_write_goal.clearParam()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncWrite(goal) 실패: {self.packet_handler.getTxRxResult(result)}")

    def get_present_positions(self) -> dict[int, int]:
        assert self._sync_read_present is not None
        with self._lock:
            result = self._sync_read_present.txRxPacket()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncRead 실패: {self.packet_handler.getTxRxResult(result)}")
            return {}
        positions = {}
        for mid in self.motor_ids:
            if self._sync_read_present.isAvailable(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                positions[mid] = self._sync_read_present.getData(
                    mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                )
        return positions

    def get_present_velocity(self, motor_id: int) -> int:
        with self._lock:
            val, result, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_VELOCITY
            )
        return val if result == COMM_SUCCESS else 0

    def get_present_load(self, motor_id: int) -> int:
        with self._lock:
            val, result, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_LOAD
            )
        return val if result == COMM_SUCCESS else 0

    def get_present_loads(self) -> dict[int, int]:
        """모든 모터의 Present_Load 한 번에 group sync read.

        XL430 = ‰ (-1000~+1000, 토크 비율), XL330 = mA. 같은 레지스터(126)라
        한 번에 읽음. self-play contact spike 감지 등에서 baseline 대비 차이로 사용.
        반환: {motor_id: signed_load}.
        """
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

    # ─── Gripper ────────────────────────────────────────────

    def set_goal_current(self, motor_id: int, current: int) -> None:
        """Goal Current 설정.
        Current-based Position Control 모드(5)에서만 유효.
        current 단위: raw value (XL330 기준 1 ≈ 1mA)
        권장 파지력: 100~300 (물체 크기/무게에 따라 조정)
        """
        self._write2(motor_id, ADDR_GOAL_CURRENT, current)

    # ─── Position PID gain ──────────────────────────────────

    def set_position_pid(
        self,
        motor_id: int,
        p: int | None = None,
        i: int | None = None,
        d: int | None = None,
    ) -> None:
        if d is not None:
            self._write2(motor_id, ADDR_POSITION_D_GAIN, int(d))
        if i is not None:
            self._write2(motor_id, ADDR_POSITION_I_GAIN, int(i))
        if p is not None:
            self._write2(motor_id, ADDR_POSITION_P_GAIN, int(p))

    # ─── 프로파일 설정 ────────────────────────────────────────

    def set_profile_velocity(self, motor_id: int, velocity: int) -> None:
        self._write4(motor_id, ADDR_PROFILE_VELOCITY, velocity)

    def set_profile_acceleration(self, motor_id: int, acceleration: int) -> None:
        self._write4(motor_id, ADDR_PROFILE_ACCELERATION, acceleration)

    def set_profile_velocities_sync(self, velocities: dict[int, int]) -> None:
        assert self._sync_write_profile_vel is not None
        with self._lock:
            for mid, vel in velocities.items():
                self._sync_write_profile_vel.addParam(
                    mid, self._int_to_4bytes(vel))
            result = self._sync_write_profile_vel.txPacket()
            self._sync_write_profile_vel.clearParam()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncWrite(profile_vel) 실패: {self.packet_handler.getTxRxResult(result)}")

    def set_profile_accelerations_sync(self, accelerations: dict[int, int]) -> None:
        assert self._sync_write_profile_acc is not None
        with self._lock:
            for mid, acc in accelerations.items():
                self._sync_write_profile_acc.addParam(
                    mid, self._int_to_4bytes(acc))
            result = self._sync_write_profile_acc.txPacket()
            self._sync_write_profile_acc.clearParam()
        if result != COMM_SUCCESS:
            logger.warning(
                f"SyncWrite(profile_acc) 실패: {self.packet_handler.getTxRxResult(result)}")

    # ─── 재시작 ──────────────────────────────────────────────

    def reboot(self, motor_id: int) -> None:
        with self._lock:
            self.packet_handler.reboot(self.port_handler, motor_id)
        logger.info(f"모터 {motor_id} 재시작")

    # ─── Util ────────────────────────────────────────────────

    def _apply_limits(self, pos: int, cfg: MotorConfig) -> int:
        pos = max(cfg.limit_min, min(cfg.limit_max, pos))
        if cfg.reverse:
            center = (cfg.limit_min + cfg.limit_max) // 2
            pos = center - (pos - center)
        return pos

    @staticmethod
    def _int_to_4bytes(value: int) -> list[int]:
        return [
            DXL_LOBYTE(DXL_LOWORD(value)),
            DXL_HIBYTE(DXL_LOWORD(value)),
            DXL_LOBYTE(DXL_HIWORD(value)),
            DXL_HIBYTE(DXL_HIWORD(value)),
        ]

    def _write1(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning(
                f"write1 실패 id={motor_id}: {self.packet_handler.getTxRxResult(result)}")

    def _write2(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning(
                f"write2 실패 id={motor_id}: {self.packet_handler.getTxRxResult(result)}")

    def _write4(self, motor_id: int, addr: int, value: int) -> None:
        with self._lock:
            result, _ = self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, addr, value
            )
        if result != COMM_SUCCESS:
            logger.warning(
                f"write4 실패 id={motor_id}: {self.packet_handler.getTxRxResult(result)}")
