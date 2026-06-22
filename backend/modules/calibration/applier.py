"""CalibrationApplier — stateless 도메인 반영 layer.

`CalibrationSnapshot` 을 받아 자기 process 의 runtime 객체에 apply:
- JointCoordinates / LinkCoordinates / SagCoordinates 의 in-memory state 갱신
- PybulletKinematics 의 first-time init (link_offsets 주입 + URDF patch + loadURDF)

본 클래스는 *stateless 함수 묶음* — instance state 없음. `CalibrationCache`
(데이터 layer) 와 분리:

    Storage  →  CalibrationCache (data layer, 부재면 cache.fetch_active)
                      ↓ snapshot
              CalibrationApplier (도메인 반영 layer, 본 파일)
                      ↓
              Coordinates 3종 / PybulletKinematics (각 process 싱글톤)

docs/storage_layer.md §7 원칙 5 — "런타임 calibration reload 는 범위 밖". 본
apply 는 *부팅 시 1회* 호출 가정. PybulletKinematics 가 이미 init 됐으면 re-init
시도 X (이미 initialized = no-op + log warning).

Multi-consumer 자리 — motion_node (모터 Pi) 가 첫 caller. so101 / omx 동시 운영 시
두 Pi 의 motion_node 가 각각 자기 process 에서 호출. detector / pointcloud /
task_node (PC) 는 현재 calibration_node 의 in-process push 로 통과 (Phase 4 자리에
Applier 사용으로 일원화 검토).
"""

from __future__ import annotations

import logging

from core.coords.joint_coordinates import JointCoordinates
from core.coords.link_coordinates import LinkCoordinates
from core.coords.sag_coordinates import SagCoordinates
from core.robot.robot_registry import RobotRegistry
from modules.calibration.calibration_cache import CalibrationSnapshot
from modules.kinematics.adapters.pybullet_kinematics import PybulletKinematics
from modules.kinematics.adapters.sag_corrected import SagCorrectedKinematics

logger = logging.getLogger(__name__)


class CalibrationApplier:
    """5종 캘 snapshot → 도메인 객체 반영. instance state 없음."""

    @staticmethod
    def apply(robot_id: str, snapshot: CalibrationSnapshot) -> None:
        """snapshot 을 자기 process 의 runtime 객체에 push.

        호출 순서 보장:
        1. Coordinates 3종 (joint/link/sag) — `set_offsets` 가 idempotent dict overwrite
        2. PybulletKinematics first-init — `apply_link_offsets` + `initialize`. 이미
           initialized 면 skip (런타임 reload 안 함, docs/storage_layer.md §7 원칙 5)
        3. SagCorrectedKinematics.reload_calibration — link/sag numpy array 캐시 갱신

        부팅 시 1회 호출 가정. 두 번째 호출 시 PyBullet 객체 재생성 X — Coordinates
        만 새 값으로 덮어씀.
        """
        JointCoordinates().set_offsets(robot_id, snapshot.joint_offsets)
        LinkCoordinates().set_offsets(robot_id, snapshot.link_offsets)
        SagCoordinates().set_offsets(robot_id, snapshot.sag_offsets)

        kinematics = RobotRegistry().get_kinematics(robot_id)
        # SagCorrectedKinematics decorator → ._inner → PybulletKinematics
        if isinstance(kinematics, SagCorrectedKinematics):
            inner = kinematics._inner  # type: ignore[attr-defined]
            if isinstance(inner, PybulletKinematics):
                if not inner._initialized:  # type: ignore[attr-defined]
                    inner.apply_link_offsets(snapshot.link_offsets)
                    inner.initialize()
                    logger.info(
                        "[%s] PybulletKinematics 첫 init 완료 (link_offsets %d joints)",
                        robot_id,
                        len(snapshot.link_offsets.offsets),
                    )
                else:
                    logger.debug(
                        "[%s] PybulletKinematics 이미 initialized — Coordinates 만 갱신",
                        robot_id,
                    )
            kinematics.reload_calibration()
