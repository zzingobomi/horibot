"""Sag offset 캘리브레이션 파일 I/O.

Hand-Eye BA가 추정한 *자세 의존 중력 처짐* 보정 k (rad/(m·g_unit))를 저장/로드.
런타임의 SagCoordinates가 부팅 시 로드해 PybulletSolver의 FK/IK 호출에서 sag 적용
(`apply_gravity_sag` 참고).

현재 모델은 J2, J3에만 sag (DIY 5축에서 중력 부하 가장 큰 두 joint).
J1/J4/J5의 sag는 측정 noise 수준이라 모델 단순성 위해 제외 (검증:
[docs/diag_gravity_sag_physical.py](docs/diag_gravity_sag_physical.py)).

저장 포맷 (npz):
    joint_ids: int 배열 (보통 [2, 3] — 모터 id)
    sag_k_rad_per_m: (N,) float64 — joint별 stiffness 역수 (rad/(m·g_unit))
    method: 캘 방법 문자열

joint_offsets/link_offsets와 같은 cumulative delta 패턴 — merge_delta로 합산 후 save.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SagOffsets:
    """{joint_id: k_value} dict. 빈 dict면 sag 없음(= URDF 원본 그대로 동작)."""
    k_rad_per_m: dict[int, float] = field(default_factory=dict)

    def is_empty(self) -> bool:
        return not self.k_rad_per_m

    def get_k(self, jid: int) -> float:
        return float(self.k_rad_per_m.get(jid, 0.0))

    def as_array_for_joints(self, joint_ids: list[int]) -> np.ndarray:
        """주어진 joint id 순서로 k 값 배열 반환. 없으면 0.

        예: arm 5축 [1,2,3,4,5] → 길이 5 배열, J2/J3 위치에만 값 (나머지 0).
        BA의 sag_k_rad_per_m은 (2,) 배열이므로 호출 측에서 적절히 매핑.
        """
        return np.array([self.get_k(j) for j in joint_ids], dtype=np.float64)


def load(path: str | Path) -> SagOffsets:
    """파일 없거나 손상 시 비어있는 SagOffsets 반환."""
    path = Path(path)
    if not path.exists():
        return SagOffsets()
    try:
        data = np.load(str(path), allow_pickle=False)
        ids = data["joint_ids"].astype(int).tolist()
        ks = data["sag_k_rad_per_m"].astype(np.float64)
        return SagOffsets(
            k_rad_per_m={int(i): float(ks[k]) for k, i in enumerate(ids)},
        )
    except Exception as e:
        logger.warning(f"sag_offsets 로드 실패 ({path}): {e}")
        return SagOffsets()


def save(
    path: str | Path,
    offsets: SagOffsets,
    method: str = "BA(physical_sag)",
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ids = sorted(offsets.k_rad_per_m.keys())
    ks = np.array([offsets.get_k(i) for i in ids], dtype=np.float64)
    np.savez(
        str(path),
        joint_ids=np.array(ids, dtype=np.int32),
        sag_k_rad_per_m=ks,
        method=method,
    )
    logger.info(f"sag_offsets 저장: {path} (n={len(ids)})")


def merge_delta(
    existing: SagOffsets,
    delta: SagOffsets,
) -> SagOffsets:
    """기존 k에 delta cumulative 합산.

    sag k는 effective stiffness 역수라 가산이 직관적 — 매 라운드 BA가 잔여
    sag를 추가로 추정. 캡처 자세 분포가 좋으면 첫 commit 후 다음 delta는 ~0
    (수렴 신호).
    """
    ids = set(existing.k_rad_per_m.keys()) | set(delta.k_rad_per_m.keys())
    merged = SagOffsets()
    for jid in ids:
        merged.k_rad_per_m[jid] = existing.get_k(jid) + delta.get_k(jid)
    return merged
