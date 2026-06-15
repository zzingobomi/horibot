"""CalibrationCache — runtime 소비자의 read-only in-memory state.

calibration_node 가 부팅 시 storage 에서 fetch 후 `set()` 으로 채움. 다른 소비자
(DetectorNode / bridge router / task_node / tsdf_builder) 는 `get()` 으로 read.

이 layer 의 목적:
- 소비자는 storage 모름 (docs/storage_layer.md §7 원칙 1)
- calibration_node 만 storage 앎 (원칙 2)
- push 방향 — calibration_node 가 부팅 시 + ACTIVATE invalidation 시 set
- read-only — 소비자가 set 호출 X

intrinsic + hand_eye 만 본 cache 에 — joint/link/sag offsets 는 Coordinates 가
자체 in-memory state 보유.
"""

from __future__ import annotations

import threading

from modules.calibration.loader import CalibrationData


class CalibrationCache:
    _instance: "CalibrationCache | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "CalibrationCache":
        if cls._instance is None:
            with cls._new_lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        self._lock = threading.Lock()
        self._by_robot: dict[str, CalibrationData] = {}
        # ready event 는 calibration_node 의 atomic push 완료 신호. consumer 의
        # hot path 가 wait_ready 로 partial state 회피.
        self._ready: dict[str, threading.Event] = {}

    def _event(self, robot_id: str) -> threading.Event:
        with self._lock:
            ev = self._ready.get(robot_id)
            if ev is None:
                ev = threading.Event()
                self._ready[robot_id] = ev
            return ev

    def set(self, robot_id: str, calib: CalibrationData) -> None:
        """calibration_node 가 atomic push 완료 직전 호출 + signal_ready 호출.

        intrinsic / hand_eye 외 다른 source (Coordinates 들) 도 같이 set 되어
        있는 상태에서 호출 — partial publish 금지 (docs/storage_layer.md §7).
        """
        with self._lock:
            self._by_robot[robot_id] = calib

    def signal_ready(self, robot_id: str) -> None:
        """5종 push 완료 후 호출. consumer hot path 의 wait_ready 가 풀림."""
        self._event(robot_id).set()

    def is_ready(self, robot_id: str) -> bool:
        return self._event(robot_id).is_set()

    def wait_ready(self, robot_id: str, timeout: float | None = None) -> bool:
        """consumer hot path — calibration_node 의 atomic push 완료까지 대기.

        timeout=None 이면 무한. False 반환 = timeout — caller 가 service 거부 결정.
        """
        return self._event(robot_id).wait(timeout)

    def get(self, robot_id: str) -> CalibrationData:
        """robot 에 아직 push 안 됐으면 empty CalibrationData — caller 가 is_ready
        체크해서 서비스 거부 / UI 경고.
        """
        with self._lock:
            return self._by_robot.get(robot_id) or CalibrationData()
