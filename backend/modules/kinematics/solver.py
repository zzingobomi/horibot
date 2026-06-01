"""Backward-compat facade — `PybulletSolver()` 가 기존 14개 호출처와 동일 모양.

내부 구조는 multi_robot_architecture.md §3 의 Protocol + Decorator pattern:
    PybulletSolver() → CorrectedIKSolver(PybulletIKSolver(urdf_path), link, sag)

- ideal URDF 기구학: [`PybulletIKSolver`](adapters/pybullet_solver.py)
- sag 보정 (Decorator): [`CorrectedIKSolver`](corrected.py)
- 공통 인터페이스: [`IKSolver`](iksolver.py) Protocol

기존 callers 는 `PybulletSolver()` 그대로. robot_id 차원 도입 시 (후속 todo)
`RobotRegistry().get_iksolver(robot_id)` 같은 API 로 점진 이전 예정.
"""

from __future__ import annotations

import threading
from typing import TypeAlias

from core.link_coordinates import LinkCoordinates
from core.robot_registry import RobotRegistry
from core.sag_coordinates import SagCoordinates
from modules.kinematics.adapters.pybullet_solver import PybulletIKSolver
from modules.kinematics.corrected import CorrectedIKSolver

# 호환 type alias (기존 [solver.py](solver.py) 가 export 하던 것들)
Position3: TypeAlias = tuple[float, float, float]  # [x, y, z] 미터
Quaternion: TypeAlias = tuple[float, float, float, float]  # [x, y, z, w]
RotMatrix3x3: TypeAlias = list[list[float]]  # 3x3 회전 행렬


# Process-wide singleton 캐시 — 기존 `PybulletSolver()` singleton 호환.
_instance: CorrectedIKSolver | None = None
_lock = threading.Lock()


def PybulletSolver() -> CorrectedIKSolver:
    """RobotRegistry().default() 기준 robot 의 IK solver 반환.

    내부: `CorrectedIKSolver(PybulletIKSolver(...), link_coords, sag_coords)`.
    process 당 1 인스턴스 — 기존 singleton 동작 그대로.

    `IKSolver` Protocol 만족 — `.fk()` / `.ik()` / `.fk_to_matrix()` /
    `.joint_limits()` / `.self_collision()` + `.dof` / `.ee_link_name` property.
    추가로 `._reload_sag_cache()` 가 SagCoordinates COMMIT 후 캐시 재로드 (Decorator
    안에 있음).
    """
    global _instance
    if _instance is None:
        with _lock:
            if _instance is None:
                cfg = RobotRegistry().default()
                inner = PybulletIKSolver(cfg.urdf_path)
                _instance = CorrectedIKSolver(
                    inner, LinkCoordinates(), SagCoordinates()
                )
    return _instance
