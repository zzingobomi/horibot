"""Relational DB store — Protocol + Phase 1 (캘 3 테이블) method.

다른 노드는 이 Protocol 직접 안 호출 — storage_node 가 Zenoh service handler
안에서만 사용. 구현체는 `adapters/` 의 SqliteStore / MemoryRdbStore 등.

Phase 2 진입 시 scans/meshes/task_runs method 가 *추가* (additive, 기존 method
변경 X) — docs/storage_layer.md §8.
"""

from __future__ import annotations

from typing import Protocol

from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationRunRecord,
)


class RdbStore(Protocol):
    """Relational DB store. Phase 1 = 캘 3 테이블."""

    # ─── 활성 result 조회 — 다른 노드의 hot path ─────────────
    def get_active_result(
        self, robot_id: str, kind: CalibrationKind
    ) -> CalibrationResultRecord | None:
        """같은 (robot_id, kind) 의 is_active=true row. 없으면 None."""
        ...

    # ─── History list ────────────────────────────────────────
    def list_results(
        self, robot_id: str, kind: CalibrationKind, limit: int = 100
    ) -> list[CalibrationResultRecord]:
        """`created_at DESC` 정렬. limit 최대 100 default."""
        ...

    def list_runs(
        self, robot_id: str, limit: int = 50
    ) -> list[tuple[CalibrationRunRecord, list[CalibrationResultRecord]]]:
        """Run 단위 history — frontend list/ACTIVATE 패널이 사용.

        `run.started_at DESC` 정렬. 각 tuple = (Run, 그 Run 의 모든 Result list).
        storage_layer.md Stage 4 design A — MLflow Model Registry 정합.
        """
        ...

    def get_result(self, result_id: int) -> CalibrationResultRecord | None: ...

    def get_run(self, run_id: int) -> CalibrationRunRecord | None: ...

    def list_captures(self, run_id: int) -> list[CalibrationCaptureRecord]:
        """`pose_index ASC` 정렬."""
        ...

    # ─── INSERT — Run + Result + Captures 한 transaction ─────
    def commit_calibration(
        self,
        run: CalibrationRunRecord,
        results: list[CalibrationResultRecord],
        captures: list[CalibrationCaptureRecord],
    ) -> tuple[int, list[int]]:
        """원자적 INSERT. results 의 `is_active` 는 모두 False 로 들어감.

        반환: (run_id, [result_id, ...]). caller 는 받은 id 들로 ACTIVATE 호출.

        invariant — captures 의 `run_id` 와 results 의 `run_id` 는 INSERT 시
        새로 발급된 run_id 로 자동 덮어씌움 (caller 의 임시 placeholder X).
        """
        ...

    # ─── ACTIVATE — atomic toggle ────────────────────────────
    def activate_result(self, result_id: int) -> CalibrationResultRecord:
        """대상 result.is_active=true + 같은 (robot_id, kind) 다른 row 들 false.

        한 transaction 안에서. 반환은 activate 된 result (caller 가 robot_id /
        kind 를 invalidation payload 에 쓰기 위해).

        존재하지 않는 id 면 KeyError.
        """
        ...
