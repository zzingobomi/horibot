"""Gripper 셋업 — 객체별/task 별 gripper 동작 파라미터 override.

self-play 의 paper_cup vs cube 처럼 객체별로 close current / position / 잡힘
판정 threshold 가 다른 경우 (또는 다른 task 가 같은 목적으로 override 하고
싶을 때) 재사용. 모든 필드 optional — None 이면 호출 측이 자기 default 적용.

필드:
- close_current: gripper close 시 Goal Current (mA). 부드러운 객체일수록 낮춤
  (종이컵 찌그러짐 방지).
- open_position: open 명령의 raw position (0..4095, 클수록 더 열림). 큰 객체일
  수록 더 벌어진 값.
- close_position: close 명령의 raw position (작을수록 더 닫힘). 보통 default 면
  충분 (객체에 막혀 멈춤) — 안전/hardware 제약 시만 override.
- held_threshold: close 후 gripper Present_Position 이 이 값보다 크면 '잡힘',
  작으면 '빈손' 으로 판정. 객체 폭에 따라 다름 (큰 객체일수록 더 큰 값).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GripperSetup:
    """Gripper 동작 파라미터 override. None 필드는 호출측 default 사용."""

    close_current: int | None = None     # mA
    open_position: int | None = None     # raw 0..4095
    close_position: int | None = None    # raw 0..4095
    held_threshold: int | None = None    # raw 0..4095
