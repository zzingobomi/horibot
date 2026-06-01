"""자세 의존 중력 sag stiffness의 런타임 진입점 (SagOffsets 싱글톤 캐시).

[JointCoordinates](backend/core/joint_coordinates.py),
[LinkCoordinates](backend/core/link_coordinates.py)와 같은 싱글톤 + 디스크 캐시 패턴:
    - 디스크의 robot/calibration/sag_offsets.npz를 부팅 시 1회 load → 메모리 보관
    - snapshot() / commit_offsets() — 디스크 save + 메모리 reload
    - 분산 동기화는 git 처리 (.npz는 git 추적, 같은 commit = 같은 파일)

joint_offsets / link_offsets와 다른 점:
    - 값이 *scalar k* per joint (rad/(m·g_unit), lumped mass 가정)
    - 사용처가 *FK/IK 호출 시 angle 보정* — PybulletSolver가 fk/ik 메서드에서
      `apply_gravity_sag(angles, k_array)`로 입력 angle을 sag 적용 후 PyBullet 호출.
    - PyBullet URDF 재로드 불필요 (link_offsets와 달리). 즉 commit 후 restart 없이
      메모리 캐시만 갱신하면 다음 FK/IK 호출부터 자동 반영. 단, 분산 머신은 git
      pull + 재시작 필요.
    - **commit_offsets semantics: overwrite** (link_offsets와 동일 이유 —
      BA의 sag_k 도 absolute total 값). 자세한 분석은
      docs/accuracy_squeeze_plan.md §1.6 참조.
"""

from __future__ import annotations

import logging
import threading

from core.robot_registry import RobotRegistry
from modules.calibration import sag_offsets as sag_offsets_io
from modules.calibration.sag_offsets import SagOffsets

logger = logging.getLogger(__name__)


def _sag_offsets_path():
    return RobotRegistry().default().calibration_dir / "sag_offsets.npz"


class SagCoordinates:
    _instance: "SagCoordinates | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "SagCoordinates":
        if cls._instance is None:
            with cls._new_lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        self._cache_lock = threading.Lock()
        self._offsets: SagOffsets = sag_offsets_io.load(_sag_offsets_path())
        if not self._offsets.is_empty():
            ks = ", ".join(
                f"J{jid}={k:+.4f}"
                for jid, k in sorted(self._offsets.k_rad_per_m.items())
            )
            logger.info(f"sag_offsets 적용: {ks}")

    def snapshot(self) -> SagOffsets:
        with self._cache_lock:
            return SagOffsets(k_rad_per_m=dict(self._offsets.k_rad_per_m))

    def get_k(self, jid: int) -> float:
        with self._cache_lock:
            return self._offsets.get_k(jid)

    def commit_offsets(
        self,
        offsets: SagOffsets,
        method: str,
    ) -> SagOffsets:
        """COMMIT 시 atomic 갱신: 디스크 *overwrite* + 메모리 reload (PC 내부 한정).

        **Overwrite semantics** — `offsets`는 *absolute total* 값. 기존 disk값과
        가산하지 않고 그대로 덮어씀. 이유: BA의 sag_k 출력은 absolute 값이라
        cumulative 가산하면 누적 손상 (참조: accuracy_squeeze_plan §1.6).

        link_offsets와 달리 PybulletSolver 재시작 불필요 — sag는 매 FK/IK 호출
        시점에 메모리 캐시에서 읽으니까 다음 호출부터 자동 반영. 단 다른 머신은
        git pull + 재시작 (joint/link와 동일).
        """
        sag_offsets_io.save(_sag_offsets_path(), offsets, method=method)
        with self._cache_lock:
            self._offsets = SagOffsets(k_rad_per_m=dict(offsets.k_rad_per_m))
        return self.snapshot()
