"""Mock MotorBackend — hardware-less 자리. mock.yaml + test 자리.

레이아웃(MotorSpec list)을 받아 합성 — real 과 동일 layout SSOT (motors.yaml).
"""

from __future__ import annotations

from ..contract import (
    MotorCapabilities,
    MotorCapability,
    MotorInfo,
    MotorKind,
    MotorTopology,
)
from ..layout import MotorSpec


class MockMotorBackend:
    """In-process mock — motors 레이아웃대로 합성 state, hardware 없이 동작."""

    def __init__(
        self,
        motors: list[MotorSpec],
        initial_positions_raw: list[int] | None = None,
    ) -> None:
        self._motors = list(motors)
        # 초기 position = 각 모터 유효 초기 자세 (home 영점을 limit 안으로 clamp —
        # joint3 처럼 영점 0° 가 limit 밖인 축도 물리적으로 유효한 자세로 시작).
        # initial_positions_raw 지정 시 그 자세로 부팅 (mock 전용 ready 자세 — home
        # 영점([0]*rad)이 IK-특이라 sim 데모/미리보기가 거기서 안 도는 것을 피함.
        # 실 driver 는 실물이 있는 자리에서 시작하므로 무관 — mock 배치만의 편의).
        if initial_positions_raw is not None and len(initial_positions_raw) != len(
            self._motors
        ):
            raise ValueError(
                f"initial_positions_raw 길이 {len(initial_positions_raw)} != "
                f"motors {len(self._motors)}"
            )
        self._positions: list[int] = (
            list(initial_positions_raw)
            if initial_positions_raw is not None
            else [m.initial_raw for m in self._motors]
        )
        self._torque_enabled = False
        self._gripper_index: int | None = (
            len(self._motors) - 1
            if self._motors and self._motors[-1].kind == MotorKind.GRIPPER
            else None
        )
        # 파지 시뮬 seam (테스트/특성화 전용, 기본 off = 실물 없는 mock 정상 동작).
        # 물체에 걸려 stall 한 그리퍼 readback 을 흉내 — set 하면 read_positions/
        # read_loads 가 명령값 대신 이 값을 그리퍼 자리에 돌려준다.
        self._gripper_fb_raw: int | None = None
        self._gripper_load_raw: int | None = None

    # ── self-declare ──

    def capabilities(self) -> MotorCapabilities:
        # mock = TORQUE_TOGGLE + REBOOT baseline (vendor 별 추가는 실 driver 자리)
        return MotorCapabilities(
            flags={MotorCapability.TORQUE_TOGGLE, MotorCapability.REBOOT},
        )

    def topology(self) -> MotorTopology:
        return MotorTopology(
            motors=[MotorInfo(id=m.id, kind=m.kind) for m in self._motors]
        )

    # ── lifecycle ──

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    # ── read ──

    def read_positions(self) -> list[int]:
        pos = list(self._positions)
        # 파지 시뮬: 물체에 걸려 stall 한 위치를 그리퍼 자리에 덮어씀 (명령값 무시).
        if self._gripper_fb_raw is not None and self._gripper_index is not None:
            pos[self._gripper_index] = self._gripper_fb_raw
        return pos

    def read_velocities(self) -> list[int] | None:
        return None  # mock — velocity 측정 없음

    def read_loads(self) -> list[int] | None:
        # 기본 None (mock 은 load 없음). 파지 시뮬에서만 그리퍼 부하 합성.
        if self._gripper_load_raw is None or self._gripper_index is None:
            return None
        loads = [0] * len(self._motors)
        loads[self._gripper_index] = self._gripper_load_raw
        return loads

    def get_torque_enabled(self) -> bool:
        return self._torque_enabled

    # ── write ──

    def set_torque(self, enabled: bool) -> None:
        self._torque_enabled = enabled

    def reboot(self) -> None:
        # mock — 실 reboot 없음
        pass

    def set_gripper(self, position_raw: int) -> None:
        # 마지막 motor 가 gripper kind 일 때만 (Topology 위 derived)
        if self._gripper_index is not None:
            self._positions[self._gripper_index] = position_raw

    # ── 파지 시뮬 seam (테스트/특성화 — 실물 stall/부하 흉내) ──

    def simulate_gripper_hold(
        self, *, position_raw: int | None, load_raw: int | None = None
    ) -> None:
        """그리퍼 readback 을 실물처럼 덮어씀. position_raw=None 이면 시뮬 해제
        (명령값을 그대로 반환 = 빈 파지·정상 mock). 물체 물림 = close 명령해도
        stall_raw 에서 멈춤 / 낙하 = 해제 후 다시 명령값(close_raw)."""
        self._gripper_fb_raw = position_raw
        self._gripper_load_raw = load_raw

    def write_positions(self, positions_raw: list[int]) -> None:
        # mock — 즉시 갱신 (실 motor latency 없음)
        for i, p in enumerate(positions_raw):
            if i < len(self._positions):
                self._positions[i] = p
