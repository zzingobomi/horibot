"""Named robot poses 로더.

robot/config/robot_poses.yaml 을 1회 읽어 메모리 캐시.
`load_pose(name)` 으로 자세별 lookup. MOTION_MOVE_J 페이로드의 `joints`
항목 그대로 사용 가능한 리스트 반환.
"""

from __future__ import annotations

import logging
from typing import TypedDict

import yaml

from core.robot_registry import RobotRegistry

logger = logging.getLogger(__name__)


class JointAngle(TypedDict):
    id: int
    degree: float


_cache: dict[str, list[JointAngle]] | None = None


def _load_all() -> dict[str, list[JointAngle]]:
    global _cache
    if _cache is not None:
        return _cache

    poses_path = RobotRegistry().default().robot_poses_yaml
    with open(poses_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    if not isinstance(raw, dict) or not raw:
        raise ValueError(f"robot_poses.yaml: 비어있거나 dict 아님 ({poses_path})")

    parsed: dict[str, list[JointAngle]] = {}
    for name, joints_raw in raw.items():
        if not isinstance(joints_raw, list) or not joints_raw:
            raise ValueError(f"robot_poses.yaml: '{name}' joints 리스트 아님")
        joints: list[JointAngle] = []
        for entry in joints_raw:
            if (
                not isinstance(entry, dict)
                or "id" not in entry
                or "degree" not in entry
            ):
                raise ValueError(f"robot_poses.yaml: '{name}' 잘못된 항목 {entry!r}")
            joints.append({"id": int(entry["id"]), "degree": float(entry["degree"])})
        parsed[str(name)] = joints

    _cache = parsed
    logger.info("robot_poses 로드: %s", ", ".join(parsed.keys()))
    return _cache


def load_pose(name: str) -> list[JointAngle]:
    """이름으로 자세 조회.

    Raises:
        KeyError: 해당 이름의 자세가 없을 때.
    """
    all_poses = _load_all()
    if name not in all_poses:
        raise KeyError(
            f"robot_poses.yaml 에 '{name}' 자세 없음. 사용 가능: {list(all_poses)}"
        )
    return all_poses[name]


def list_pose_names(prefix: str = "") -> list[str]:
    """prefix 로 시작하는 자세 이름 목록 (lexical 정렬).

    self-play search pose 순회용: `list_pose_names("search_")` 으로 사용 가능한
    search 자세 자동 수집. prefix=""면 모든 자세 반환.
    """
    all_poses = _load_all()
    return sorted(name for name in all_poses if name.startswith(prefix))
