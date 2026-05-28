"""Link offset 캘리브레이션 파일 I/O.

Hand-Eye BA가 추정한 link origin 보정(translation 3 + rotation 3 per joint)을 저장/로드.
런타임의 LinkCoordinates가 부팅 시 로드해 URDF의 <joint><origin xyz rpy/> 값에 patch.

저장 포맷 (npz):
    joint_ids: int 배열 (보통 [1,2,3,4,5] — 모터 id와 같은 순서)
    link_trans_m: (N, 3) float64 — joint i origin xyz에 더할 dx,dy,dz (meter)
    link_rot_rad: (N, 3) float64 — joint i origin rpy에 적용할 rotation vector
                                    (Rodrigues axis-angle, radian)
    method: 캘 방법 문자열

commit semantics: **overwrite** (joint_offsets와 다름).
    BA의 link_t 출력은 original URDF 기준 *absolute total* 값이라 cumulative 가산
    금지. LinkCoordinates.commit_offsets 가 save 직접 호출 (merge_delta 안 거침).
    merge_delta 유틸은 다른 용도(예: 수동 delta 가산 분석) 위해 남겨둠.
    이력: 과거 cumulative였으나 누적 손상 발견 → 2026-05-28 overwrite로 변경.
    참조: docs/accuracy_squeeze_plan.md §1.6.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class LinkOffsets:
    """{joint_id: (3,) ndarray} 두 dict 묶음. 빈 dict면 offset 없음(= URDF 원본)."""
    trans: dict[int, np.ndarray] = field(default_factory=dict)  # m
    rot: dict[int, np.ndarray] = field(default_factory=dict)    # rad rotvec

    def is_empty(self) -> bool:
        return not self.trans and not self.rot

    def get_trans(self, jid: int) -> np.ndarray:
        return self.trans.get(jid, np.zeros(3, dtype=np.float64))

    def get_rot(self, jid: int) -> np.ndarray:
        return self.rot.get(jid, np.zeros(3, dtype=np.float64))


def load(path: str | Path) -> LinkOffsets:
    """파일 없거나 손상 시 비어있는 LinkOffsets 반환."""
    path = Path(path)
    if not path.exists():
        return LinkOffsets()
    try:
        data = np.load(str(path), allow_pickle=False)
        ids = data["joint_ids"].astype(int).tolist()
        trans = data["link_trans_m"].astype(np.float64)
        rot = data["link_rot_rad"].astype(np.float64)
        return LinkOffsets(
            trans={int(i): trans[k].copy() for k, i in enumerate(ids)},
            rot={int(i): rot[k].copy() for k, i in enumerate(ids)},
        )
    except Exception as e:
        logger.warning(f"link_offsets 로드 실패 ({path}): {e}")
        return LinkOffsets()


def save(
    path: str | Path,
    offsets: LinkOffsets,
    method: str = "BA(extended)",
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ids = sorted(set(offsets.trans.keys()) | set(offsets.rot.keys()))
    trans = np.array([offsets.get_trans(i) for i in ids], dtype=np.float64)
    rot = np.array([offsets.get_rot(i) for i in ids], dtype=np.float64)
    np.savez(
        str(path),
        joint_ids=np.array(ids, dtype=np.int32),
        link_trans_m=trans,
        link_rot_rad=rot,
        method=method,
    )
    logger.info(f"link_offsets 저장: {path} (n={len(ids)})")


def merge_delta(
    existing: LinkOffsets,
    delta: LinkOffsets,
) -> LinkOffsets:
    """기존 offset에 delta cumulative 합산.

    회전은 small-angle 가정으로 rotvec 단순 가산 (각 < 5° 정도면 commutative 근사 OK).
    링크 캘 라운드마다 delta가 0에 가까워지면 가산 오차 무시 가능.
    """
    ids = set(existing.trans.keys()) | set(existing.rot.keys())
    ids |= set(delta.trans.keys()) | set(delta.rot.keys())
    merged = LinkOffsets()
    for jid in ids:
        merged.trans[jid] = existing.get_trans(jid) + delta.get_trans(jid)
        merged.rot[jid] = existing.get_rot(jid) + delta.get_rot(jid)
    return merged
