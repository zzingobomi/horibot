"""TaskRobotSpec — task 실행에 필요한 per-robot 물리 config (host-level 주입).

task 는 robot-agnostic (host당 1, §2.7) 이나 gripper open/close raw 등 **per-robot
물리값**이 필요 → calibration/scan 처럼 resolve 가 motors.yaml 에서 투영해 dict[
robot_id] 로 주입. 물리값 추측 X — motors.yaml SSOT (CLAUDE.md 안전수치 규칙).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TaskRobotSpec:
    """한 robot 의 task 실행 물리 파라미터 (motors.yaml gripper 에서 투영)."""

    gripper_open_raw: int  # gripper limit.max (full open)
    gripper_close_raw: int  # gripper limit.min (full close)
    gripper_index: int  # positions_raw (motors.yaml 순) 내 gripper 위치
    # 잡힘 판정 raw threshold — 물체가 fingers 를 fully-close 위로 벌림. **하드웨어
    # tuning 값** (§17.5 "정확도 = 집 하드웨어"). resolve 가 close/open 사이 보수 default
    # 산출, 실물에서 조정. 이 값 미만 = 빈손 (VerifyGrasp).
    gripper_held_threshold_raw: int
