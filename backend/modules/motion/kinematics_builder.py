"""calibration bundle → 보정 kinematics 구성 — 소비자 공유 SSOT.

소비자 2 (공용 승격, 2026-07-07):
  - MotionModule (D4) — boot/live 적용 (Mirror 값)
  - ScanModule — TSDF build 시점 fresh bundle 로 저장된 raw 재계산
    ("scan 은 raw 로 저장, build 시 현재 캘로 재계산" 불변의 FK 절반)

적용 3종 (하나의 의미 — 여기와 offline BA 가 같은 모델):
  - link_offset  → URDF 자체 patch 후 factory 로드 (PyBullet 은 load 후 변경 불가)
  - sag          → SagCorrectedKinematics decorator (fk/ik 양방향, patched URDF 의
                   FkChain 으로 torque 계산 — BA 의 fk_with_sag 등가)
  - joint_offset → arm 순서 offsets 리스트 (units raw↔rad 변환 인자)

lifecycle 은 호출자 책임 — 여기선 **구성만** (initialize/close 안 함).
blocking (URDF 파싱/patch 파일 write) — async 컨텍스트에선 to_thread 로.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from modules.calibration.contract import CalibrationBundle
from modules.motor.layout import MotorSpec

from .fk_chain import FkChain
from .kinematics import Kinematics
from .sag_kinematics import SagCorrectedKinematics
from .urdf_patch import patch_urdf_link_offsets

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class CalibratedKinematics:
    """build_calibrated_kinematics 산출물 — kinematics 는 initialize 전."""

    kinematics: Kinematics
    joint_offsets: list[float] | None  # arm 순서 rad, None = joint_offset 없음
    applied: list[str]  # ["link_offset", "sag", "joint_offset"] 의 부분집합
    urdf_path: Path  # 실제 로드된 URDF (patched 면 원본과 다름)


def build_calibrated_kinematics(
    urdf_path: Path,
    robot_id: str,
    arm_specs: list[MotorSpec],
    bundle: CalibrationBundle | None,
    factory: Callable[[Path], Kinematics],
) -> CalibratedKinematics:
    """bundle 의 motion-관련 3종을 적용한 kinematics 구성. bundle=None = 무보정."""
    urdf = urdf_path
    applied: list[str] = []

    if bundle is not None and bundle.link_offset is not None:
        by_name: dict[str, tuple[list[float], list[float]]] = {}
        spec_by_id = {s.id: s for s in arm_specs}
        for entry in bundle.link_offset.result_data.offsets:
            spec = spec_by_id.get(entry.joint_id)
            if spec is None:
                logger.warning(
                    "link_offset joint_id=%d 가 arm 에 없음 — skip", entry.joint_id
                )
                continue
            by_name[spec.name] = (entry.trans_m, entry.rot_rad)
        if by_name:
            urdf = patch_urdf_link_offsets(urdf_path, robot_id, by_name)
            applied.append("link_offset")

    kin: Kinematics = factory(urdf)

    if bundle is not None and bundle.sag is not None:
        k_map = bundle.sag.result_data.k_rad_per_m
        arm_idx = {s.id: i for i, s in enumerate(arm_specs)}
        indices = [arm_idx[mid] for mid in k_map if mid in arm_idx]
        k_stiff = [k_map[mid] for mid in k_map if mid in arm_idx]
        if indices and any(abs(k) > 1e-12 for k in k_stiff):
            chain = FkChain(urdf, [s.name for s in arm_specs])
            kin = SagCorrectedKinematics(kin, chain, k_stiff, indices)
            applied.append("sag")

    joint_off: list[float] | None = None
    if bundle is not None and bundle.joint_offset is not None:
        off_map = bundle.joint_offset.result_data.offsets
        joint_off = [off_map.get(s.id, 0.0) for s in arm_specs]
        applied.append("joint_offset")

    return CalibratedKinematics(
        kinematics=kin,
        joint_offsets=joint_off,
        applied=applied,
        urdf_path=urdf,
    )
