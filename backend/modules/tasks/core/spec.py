"""TaskRobotSpec — task 실행에 필요한 per-robot 물리 config (옛 modules/task/spec.py 이동).

RobotHandle.gripper 등이 사용. 물리값 추측 X — motors.yaml SSOT 를 apps/resolve.py 가
투영해 task 모듈 생성자에 dict[robot_id] 로 주입한다.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TaskRobotSpec:
    """한 robot 의 task 실행 물리 파라미터 (motors.yaml gripper 에서 투영)."""

    gripper_open_raw: int  # gripper limit.max (full open)
    gripper_close_raw: int  # gripper limit.min (full close)
    gripper_index: int  # positions_raw (motors.yaml 순) 내 gripper 위치
    # 잡힘 판정 raw threshold — 물체가 fingers 를 fully-close 위로 벌림. 하드웨어
    # tuning 값. 이 값 미만 = 빈손 (verify_grasp 계열 후속 primitive 용).
    gripper_held_threshold_raw: int
