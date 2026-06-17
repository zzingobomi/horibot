"""Relational DB store — Protocol + Phase 1 (캘 3 테이블) + Phase 2 (scan workflow).

다른 노드는 이 Protocol 직접 안 호출 — storage_node 가 Zenoh service handler
안에서만 사용. 구현체는 `adapters/` 의 SqliteStore / MemoryRdbStore 등.

Phase 2 추가 — scan_sessions / scans / reconstructions (append-only blob +
immutable metadata row, is_active 자리 X). docs/storage_layer.md §3 + §6.
"""

from __future__ import annotations

from typing import Protocol

from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationRunRecord,
)
from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)


class RdbStore(Protocol):
    """Relational DB store. Phase 1 = 캘 3 테이블 + Phase 2 = scan workflow 3 테이블."""

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

    # ─── Phase 2 — scan workflow ─────────────────────────────
    # scan_sessions / scans / reconstructions. append-only blob + immutable
    # metadata row 패턴. is_active / ACTIVATE / invalidation 자리 X — 캘 특유
    # 패턴 안 빌림 (storage_layer.md §3).

    # scan_sessions
    def insert_scan_session(self, record: ScanSessionRecord) -> int:
        """INSERT + return row_id. (robot_id, session_id) unique."""
        ...

    def get_scan_session(self, session_row_id: int) -> ScanSessionRecord | None: ...

    def find_scan_session_by_id(
        self, robot_id: str, session_id: str
    ) -> ScanSessionRecord | None:
        """human-readable session_id 자리 lookup. caller (storage_node) 의
        new_session 자리에서 idempotent 자리 확인 자리 사용 자리."""
        ...

    def list_scan_sessions(
        self, robot_id: str, limit: int = 100
    ) -> list[ScanSessionRecord]:
        """`created_at DESC` 정렬."""
        ...

    def delete_scan_session(self, session_row_id: int) -> None:
        """CASCADE — 자식 scans / reconstructions 자리 같이 삭제 (transaction).

        ObjectStore blob 자리는 caller (storage_node) 책임 — RDB row 자리 가져와서
        blob_key list → object_store.delete 자리 별도.
        """
        ...

    # scans
    def allocate_scan_id(self, session_row_id: int) -> int:
        """session 안 monotonic scan_id 발급. transaction lock 안에서 MAX+1.

        scan_id 자리 user-visible 자리 (scan #003 자리 UI). blob_key 자리 caller
        가 결정 자리 (받은 scan_id 자리 사용).
        """
        ...

    def insert_scan(self, record: ScanRecord) -> int:
        """INSERT + return row_id. record.scan_id 자리 caller 가 allocate_scan_id
        결과 자리 채움. (session_row_id, scan_id) unique."""
        ...

    def list_scans(self, session_row_id: int) -> list[ScanRecord]:
        """`scan_id ASC` 정렬."""
        ...

    def get_scan(self, scan_row_id: int) -> ScanRecord | None: ...

    def delete_scan(self, scan_row_id: int) -> None: ...

    # reconstructions
    def insert_reconstruction(self, record: ReconstructionRecord) -> int:
        """INSERT + return row_id."""
        ...

    def list_reconstructions(
        self, session_row_id: int
    ) -> list[ReconstructionRecord]:
        """`created_at DESC` 정렬."""
        ...

    def get_reconstruction(
        self, recon_row_id: int
    ) -> ReconstructionRecord | None: ...

    def delete_reconstruction(self, recon_row_id: int) -> None: ...
