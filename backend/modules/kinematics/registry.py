"""Default-robot Kinematics 접근자.

multi-robot 환경에선 caller 가 `RobotRegistry().get_kinematics(robot_id)` 로
명시적 robot_id 전달 권장. 본 함수는 N=1 / robot_id 모호 없는 자리에서 짧게.

내부 구조 (multi_robot_architecture.md §3 의 Protocol + 좌표계 어댑터):
    get_default_kinematics() → RobotRegistry.get_kinematics(default)
                             → SagCorrectedKinematics(PybulletKinematics(urdf), link, sag)

- ideal URDF 기구학: [`PybulletKinematics`](adapters/pybullet_kinematics.py)
- sag 보정 (좌표계 어댑터): [`SagCorrectedKinematics`](adapters/sag_corrected.py)
- 공통 인터페이스: [`Kinematics`](kinematics.py) Protocol
"""

from __future__ import annotations

from typing import TypeAlias

from core.robot.robot_registry import RobotRegistry
from modules.kinematics.adapters.sag_corrected import SagCorrectedKinematics

# 호환 type alias
Position3: TypeAlias = tuple[float, float, float]  # [x, y, z] 미터
Quaternion: TypeAlias = tuple[float, float, float, float]  # [x, y, z, w]
RotMatrix3x3: TypeAlias = list[list[float]]  # 3x3 회전 행렬


def get_default_kinematics() -> SagCorrectedKinematics:
    """default robot 의 Kinematics 반환 (RobotRegistry 캐시 경유).

    multi-robot 환경에선 caller 가 `RobotRegistry().get_kinematics(robot_id)` 로
    명시적 robot_id 전달 권장.

    `Kinematics` Protocol 만족 (SagCorrectedKinematics 가 구현체) — `.fk()` /
    `.ik()` / `.fk_to_matrix()` / `.joint_limits()` / `.self_collision()` +
    `.dof` / `.tcp_link_name` property. SagCoordinates COMMIT 후 캐시 재로드는
    어댑터의 `_reload_caches()`.
    """
    kin = RobotRegistry().get_kinematics()
    # Kinematics Protocol 만 만족 (Pyright 위해 SagCorrectedKinematics 로 narrow).
    assert isinstance(kin, SagCorrectedKinematics)
    return kin
