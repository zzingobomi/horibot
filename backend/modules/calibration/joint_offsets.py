"""Joint offset 캘리브레이션 파일 I/O.

Hand-Eye BA가 추정한 조인트 zero offset을 저장/로드. 런타임의
JointStateCache가 부팅 시 로드해 raw→rad 변환 결과에 더함.

저장 포맷 (npz):
    motor_ids: int 배열 (모터 id 순서, 보통 [1,2,3,4,5])
    offsets_rad: float 배열 (motor_ids와 같은 길이, 라디안)
    method: 캘 방법 문자열 (메타 정보)

BA 결과는 *delta* — 기존 파일이 있으면 cumulative하게 합산해 저장하는 책임은
호출 측 (보통 CalibrationNode의 commit 핸들러).
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


def load(path: str | Path) -> dict[int, float]:
    """{motor_id: offset_rad} 반환. 파일 없거나 손상 시 빈 dict."""
    path = Path(path)
    if not path.exists():
        return {}
    try:
        data = np.load(str(path), allow_pickle=False)
        ids = data["motor_ids"].astype(int).tolist()
        offsets = data["offsets_rad"].astype(float).tolist()
        return {int(i): float(o) for i, o in zip(ids, offsets)}
    except Exception as e:
        logger.warning(f"joint_offsets 로드 실패 ({path}): {e}")
        return {}


def save(
    path: str | Path,
    offsets: dict[int, float],
    method: str = "BA(huber)",
) -> None:
    """주어진 {motor_id: offset_rad}을 npz로 저장."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ids = sorted(offsets.keys())
    np.savez(
        str(path),
        motor_ids=np.array(ids, dtype=np.int32),
        offsets_rad=np.array([offsets[i] for i in ids], dtype=np.float64),
        method=method,
    )
    logger.info(f"joint_offsets 저장: {path} (n={len(ids)})")


def merge_delta(
    existing: dict[int, float],
    delta_by_id: dict[int, float],
) -> dict[int, float]:
    """기존 offset에 delta를 더한 cumulative dict 반환 (mutate 안 함)."""
    merged = dict(existing)
    for mid, delta in delta_by_id.items():
        merged[mid] = merged.get(mid, 0.0) + float(delta)
    return merged
