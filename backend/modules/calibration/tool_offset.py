"""Tool offset 캘리브레이션 파일 I/O.

URDF 의 tcp link (= hand_eye 캘 reference frame) 와 실제 그리퍼 끝점
(핑거 닫혔을 때 만나는 점) 사이 EE frame 오프셋. 두 frame 사이 차이가 캘 σ 및
종합 오차 (link/sag/joint 잔여) 의 합으로 모든 자세에서 일관 시프트를 만들면,
그 시프트를 단일 EE frame 벡터로 흡수해 motion 명령 진입 시점에 변환.

저장 포맷 (npz):
    trans_m:    (3,) float64 — EE frame 의 (실제 끝점 - URDF EE) translation (m)
    rot_rad:    (3,) float64 — EE frame 의 (실제 끝점 - URDF EE) rotation
                                (Rodrigues axis-angle, radian). small-angle 가정.
    method:     캘 방법 / 출처 문자열 (예: "manual", "BA(+ee)")

설계 분리 — LinkOffsets vs ToolOffset:
    - LinkOffsets ID 1~5: URDF joint origin xyz/rpy patch. urdf_patcher 가 patched
      URDF 에 적용. PyBullet IK / FK / 캘 모두 그 patched URDF 위에서 동작.
    - ToolOffset: URDF patch *안 함*. URDF EE 는 캘 reference frame 으로 *고정*.
      motion_node 의 cartesian service handler 가 명령/응답 변환에만 단방향 적용.
      이렇게 분리하지 않으면 detect 와 IK 가 같은 patched URDF 위에서 cancel out
      (2026-05-28 22:18 실측 확인).

이력:
    초기엔 LinkOffsets dict 의 ID=6 행을 *재해석* 해서 tool_offset 으로 사용했으나
    의미 충돌 (link patch vs motion 변환) 로 별도 산출물 신설 (2026-05-28).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class ToolOffset:
    """EE frame 의 (실제 끝점 - URDF EE) translation + rotation. 빈 상태면 0."""
    trans_m: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    rot_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )

    def is_empty(self) -> bool:
        return not (np.any(self.trans_m) or np.any(self.rot_rad))


def load(path: str | Path) -> ToolOffset:
    """파일 없거나 손상 시 비어있는 ToolOffset 반환."""
    path = Path(path)
    if not path.exists():
        return ToolOffset()
    try:
        data = np.load(str(path), allow_pickle=False)
        trans = data["trans_m"].astype(np.float64).reshape(3)
        rot = data["rot_rad"].astype(np.float64).reshape(3)
        return ToolOffset(trans_m=trans.copy(), rot_rad=rot.copy())
    except Exception as e:
        logger.warning(f"tool_offset 로드 실패 ({path}): {e}")
        return ToolOffset()


def save(
    path: str | Path,
    offset: ToolOffset,
    method: str = "manual",
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        str(path),
        trans_m=offset.trans_m.astype(np.float64).reshape(3),
        rot_rad=offset.rot_rad.astype(np.float64).reshape(3),
        method=method,
    )
    logger.info(
        f"tool_offset 저장: {path} (trans_mm={(offset.trans_m * 1000).round(2).tolist()})"
    )
