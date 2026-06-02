"""Backward-compat facade — `PybulletSolver()` 가 기존 14개 호출처와 동일 모양.

내부적으로 RobotRegistry().get_iksolver(default_robot_id) 로 위임. multi-robot
에선 caller 가 `RobotRegistry().get_iksolver(robot_id)` 직접 호출 권장.

내부 구조 (multi_robot_architecture.md §3 의 Protocol + Decorator pattern):
    PybulletSolver() → RobotRegistry.get_iksolver(default)
                     → CorrectedIKSolver(PybulletIKSolver(urdf_path), link, sag)

- ideal URDF 기구학: [`PybulletIKSolver`](adapters/pybullet_solver.py)
- sag 보정 (Decorator): [`CorrectedIKSolver`](corrected.py)
- 공통 인터페이스: [`IKSolver`](iksolver.py) Protocol
"""

from __future__ import annotations

from typing import TypeAlias

from core.robot.robot_registry import RobotRegistry
from modules.kinematics.corrected import CorrectedIKSolver

# 호환 type alias (기존 [solver.py](solver.py) 가 export 하던 것들)
Position3: TypeAlias = tuple[float, float, float]  # [x, y, z] 미터
Quaternion: TypeAlias = tuple[float, float, float, float]  # [x, y, z, w]
RotMatrix3x3: TypeAlias = list[list[float]]  # 3x3 회전 행렬


def PybulletSolver() -> CorrectedIKSolver:
    """default robot 의 IK solver 반환 (RobotRegistry 캐시 경유).

    multi-robot 환경에선 caller 가 `RobotRegistry().get_iksolver(robot_id)` 로
    명시적 robot_id 전달 권장. 본 facade 는 기존 14개 호출처 호환용.

    `IKSolver` Protocol 만족 (CorrectedIKSolver 가 구현체) — `.fk()` / `.ik()` /
    `.fk_to_matrix()` / `.joint_limits()` / `.self_collision()` + `.dof` /
    `.ee_link_name` property. SagCoordinates COMMIT 후 캐시 재로드는 Decorator 의
    `_reload_caches()`.
    """
    solver = RobotRegistry().get_iksolver()
    # IKSolver Protocol 만 만족 (Pyright 위해 CorrectedIKSolver 로 narrow).
    assert isinstance(solver, CorrectedIKSolver)
    return solver
