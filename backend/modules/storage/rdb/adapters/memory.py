"""MemoryRdbStore — host_mock backend + 테스트용. 영속화 X (프로세스 종료 시 사라짐).

SqliteStore 의 contract 와 동일 — 같은 storage_node service handler 가 양쪽에서
동작. host yaml 의 rdb_uri 한 줄만 바꾸면 swap.
"""

from __future__ import annotations

import copy
import threading

from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationRunRecord,
)


class MemoryRdbStore:
    def __init__(self) -> None:
        self._runs: dict[int, CalibrationRunRecord] = {}
        self._results: dict[int, CalibrationResultRecord] = {}
        self._captures: dict[int, CalibrationCaptureRecord] = {}
        self._next_run_id = 1
        self._next_result_id = 1
        self._next_capture_id = 1
        self._lock = threading.Lock()

    # ─── Read ─────────────────────────────────────────────────

    def get_active_result(
        self, robot_id: str, kind: CalibrationKind
    ) -> CalibrationResultRecord | None:
        with self._lock:
            for r in self._results.values():
                if r.robot_id == robot_id and r.kind == kind and r.is_active:
                    return copy.deepcopy(r)
        return None

    def list_results(
        self, robot_id: str, kind: CalibrationKind, limit: int = 100
    ) -> list[CalibrationResultRecord]:
        with self._lock:
            matching = [
                r
                for r in self._results.values()
                if r.robot_id == robot_id and r.kind == kind
            ]
        matching.sort(key=lambda r: r.created_at, reverse=True)
        return [copy.deepcopy(r) for r in matching[:limit]]

    def get_result(self, result_id: int) -> CalibrationResultRecord | None:
        with self._lock:
            r = self._results.get(result_id)
            return copy.deepcopy(r) if r else None

    def get_run(self, run_id: int) -> CalibrationRunRecord | None:
        with self._lock:
            r = self._runs.get(run_id)
            return copy.deepcopy(r) if r else None

    def list_captures(self, run_id: int) -> list[CalibrationCaptureRecord]:
        with self._lock:
            matching = [c for c in self._captures.values() if c.run_id == run_id]
        matching.sort(key=lambda c: c.pose_index)
        return [copy.deepcopy(c) for c in matching]

    # ─── Write ────────────────────────────────────────────────

    def commit_calibration(
        self,
        run: CalibrationRunRecord,
        results: list[CalibrationResultRecord],
        captures: list[CalibrationCaptureRecord],
    ) -> tuple[int, list[int]]:
        with self._lock:
            run_id = self._next_run_id
            self._next_run_id += 1
            self._runs[run_id] = run.model_copy(update={"id": run_id})

            result_ids: list[int] = []
            for r in results:
                rid = self._next_result_id
                self._next_result_id += 1
                self._results[rid] = r.model_copy(
                    update={"id": rid, "run_id": run_id, "is_active": False}
                )
                result_ids.append(rid)

            for c in captures:
                cid = self._next_capture_id
                self._next_capture_id += 1
                self._captures[cid] = c.model_copy(
                    update={"id": cid, "run_id": run_id}
                )

        return run_id, result_ids

    def activate_result(self, result_id: int) -> CalibrationResultRecord:
        with self._lock:
            target = self._results.get(result_id)
            if target is None:
                raise KeyError(f"result_id={result_id} 없음")
            for r in list(self._results.values()):
                if (
                    r.robot_id == target.robot_id
                    and r.kind == target.kind
                    and r.is_active
                    and r.id != result_id
                ):
                    assert r.id is not None  # in-dict 보장
                    self._results[r.id] = r.model_copy(update={"is_active": False})
            self._results[result_id] = target.model_copy(update={"is_active": True})
            return copy.deepcopy(self._results[result_id])
