"""handover 마커 발행 — "프론트에 무엇을 표시하나"의 조립 규칙.

시나리오(module.py)는 show_grasp/show_handover 의도 호출만 남기고, TaskMarker
조립과 "어느 robot 스코프로 나가나"(giver=omx 파지 / receiver=so101 랑데부)는
여기가 소유한다. wire 봉투(seq/timestamp/TaskMarkers)는 module._publish_markers
몫 — 이 파일은 마커 리스트 조립까지 (pick_and_place/publish.py 동형).
"""

from __future__ import annotations

from typing import Callable

from .contract import TaskMarker

Vec3 = tuple[float, float, float]


class MarkerPublisher:
    """run 1회의 마커 표시 — 역할(giver/receiver)별 스코프 고정.

    handover 마커는 위치만 (방향 없음) — omx 파지는 mono z=0 검출이라 자세
    시각화 대상이 아니고, 랑데부는 지점 표시가 목적."""

    def __init__(
        self,
        publish: Callable[[str, list[TaskMarker]], None],
        *,
        giver: str,
        receiver: str,
    ) -> None:
        self._publish = publish
        self._giver = giver
        self._receiver = receiver

    def show_grasp(self, p: Vec3) -> None:
        """omx 파지점 (world) — giver 스코프."""
        self._publish(self._giver, [TaskMarker(label="grasp", position=p)])

    def show_handover(self, p: Vec3) -> None:
        """랑데부(제시) 지점 — receiver 스코프 (so101 이 받으러 갈 곳)."""
        self._publish(self._receiver, [TaskMarker(label="handover", position=p)])
