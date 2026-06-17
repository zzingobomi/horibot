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
from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)


class MemoryRdbStore:
    def __init__(self) -> None:
        self._runs: dict[int, CalibrationRunRecord] = {}
        self._results: dict[int, CalibrationResultRecord] = {}
        self._captures: dict[int, CalibrationCaptureRecord] = {}
        self._next_run_id = 1
        self._next_result_id = 1
        self._next_capture_id = 1
        # Phase 2 — scan workflow
        self._scan_sessions: dict[int, ScanSessionRecord] = {}
        self._scans: dict[int, ScanRecord] = {}
        self._reconstructions: dict[int, ReconstructionRecord] = {}
        self._next_scan_session_id = 1
        self._next_scan_id = 1
        self._next_reconstruction_id = 1
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

    def list_runs(
        self, robot_id: str, limit: int = 50
    ) -> list[tuple[CalibrationRunRecord, list[CalibrationResultRecord]]]:
        with self._lock:
            matching_runs = [
                r for r in self._runs.values() if r.robot_id == robot_id
            ]
            matching_runs.sort(key=lambda r: r.started_at, reverse=True)
            matching_runs = matching_runs[:limit]
            results_by_run: dict[int, list[CalibrationResultRecord]] = {}
            for r in self._results.values():
                if r.run_id in {run.id for run in matching_runs if run.id is not None}:
                    results_by_run.setdefault(r.run_id, []).append(copy.deepcopy(r))
        return [
            (
                copy.deepcopy(run),
                results_by_run.get(run.id, []) if run.id is not None else [],
            )
            for run in matching_runs
        ]

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

    # ─── Draft run / capture-as-you-go (Phase 1 확장) ────────────

    def new_calibration_run(self, run: CalibrationRunRecord) -> int:
        with self._lock:
            run_id = self._next_run_id
            self._next_run_id += 1
            self._runs[run_id] = run.model_copy(
                update={"id": run_id, "status": "in_progress"}
            )
            return run_id

    def append_calibration_capture(
        self, capture: CalibrationCaptureRecord
    ) -> int:
        with self._lock:
            cid = self._next_capture_id
            self._next_capture_id += 1
            self._captures[cid] = capture.model_copy(update={"id": cid})
            return cid

    def delete_last_capture(self, run_id: int) -> int | None:
        with self._lock:
            captures = [
                c for c in self._captures.values() if c.run_id == run_id
            ]
            if not captures:
                return None
            captures.sort(key=lambda c: c.pose_index, reverse=True)
            last = captures[0]
            assert last.id is not None
            self._captures.pop(last.id, None)
            return last.pose_index

    def get_in_progress_run(
        self, robot_id: str, kind: CalibrationKind
    ) -> tuple[CalibrationRunRecord, list[CalibrationCaptureRecord]] | None:
        with self._lock:
            matching = [
                r
                for r in self._runs.values()
                if r.robot_id == robot_id
                and r.kind == kind
                and r.status == "in_progress"
            ]
            if not matching:
                return None
            matching.sort(key=lambda r: r.started_at, reverse=True)
            run = matching[0]
            assert run.id is not None
            cap_list = [
                c for c in self._captures.values() if c.run_id == run.id
            ]
            cap_list.sort(key=lambda c: c.pose_index)
            return copy.deepcopy(run), [copy.deepcopy(c) for c in cap_list]

    def delete_calibration_run(self, run_id: int) -> None:
        with self._lock:
            self._runs.pop(run_id, None)
            cap_ids = [
                cid for cid, c in self._captures.items() if c.run_id == run_id
            ]
            for cid in cap_ids:
                self._captures.pop(cid, None)
            res_ids = [
                rid for rid, r in self._results.items() if r.run_id == run_id
            ]
            for rid in res_ids:
                self._results.pop(rid, None)

    def finalize_calibration_run(
        self,
        run_id: int,
        results: list[CalibrationResultRecord],
        capture_residuals: dict[int, tuple[float | None, float | None, float | None]]
        | None = None,
    ) -> list[int]:
        with self._lock:
            run = self._runs.get(run_id)
            if run is None or run.status != "in_progress":
                raise KeyError(f"in_progress run id={run_id} 없음 / 이미 종료")

            ended_at = results[0].created_at if results else run.started_at
            self._runs[run_id] = run.model_copy(
                update={"status": "success", "ended_at": ended_at}
            )

            if capture_residuals:
                for cid, c in list(self._captures.items()):
                    if c.run_id != run_id:
                        continue
                    r = capture_residuals.get(c.pose_index)
                    if r is None:
                        continue
                    rrot, rtrans, weight = r
                    self._captures[cid] = c.model_copy(
                        update={
                            "residual_rot": rrot,
                            "residual_trans": rtrans,
                            "weight": weight,
                        }
                    )

            result_ids: list[int] = []
            for r in results:
                rid = self._next_result_id
                self._next_result_id += 1
                self._results[rid] = r.model_copy(
                    update={"id": rid, "run_id": run_id, "is_active": False}
                )
                result_ids.append(rid)

            return result_ids

    # ─── Phase 2 — scan workflow ──────────────────────────────────

    # scan_sessions
    def insert_scan_session(self, record: ScanSessionRecord) -> int:
        with self._lock:
            # (robot_id, session_id) unique 자리 check
            for s in self._scan_sessions.values():
                if (
                    s.robot_id == record.robot_id
                    and s.session_id == record.session_id
                ):
                    raise ValueError(
                        f"scan_session (robot_id={record.robot_id}, "
                        f"session_id={record.session_id}) 이미 존재"
                    )
            row_id = self._next_scan_session_id
            self._next_scan_session_id += 1
            self._scan_sessions[row_id] = record.model_copy(update={"id": row_id})
            return row_id

    def get_scan_session(self, session_row_id: int) -> ScanSessionRecord | None:
        with self._lock:
            r = self._scan_sessions.get(session_row_id)
            return copy.deepcopy(r) if r else None

    def find_scan_session_by_id(
        self, robot_id: str, session_id: str
    ) -> ScanSessionRecord | None:
        with self._lock:
            for s in self._scan_sessions.values():
                if s.robot_id == robot_id and s.session_id == session_id:
                    return copy.deepcopy(s)
        return None

    def list_scan_sessions(
        self, robot_id: str, limit: int = 100
    ) -> list[ScanSessionRecord]:
        with self._lock:
            matching = [
                s for s in self._scan_sessions.values() if s.robot_id == robot_id
            ]
        matching.sort(key=lambda s: s.created_at, reverse=True)
        return [copy.deepcopy(s) for s in matching[:limit]]

    def delete_scan_session(self, session_row_id: int) -> None:
        with self._lock:
            self._scan_sessions.pop(session_row_id, None)
            # CASCADE — 자식 scans / reconstructions 같이 삭제
            scan_ids = [
                rid for rid, s in self._scans.items()
                if s.session_row_id == session_row_id
            ]
            for rid in scan_ids:
                self._scans.pop(rid, None)
            recon_ids = [
                rid for rid, r in self._reconstructions.items()
                if r.session_row_id == session_row_id
            ]
            for rid in recon_ids:
                self._reconstructions.pop(rid, None)

    # scans
    def allocate_scan_id(self, session_row_id: int) -> int:
        with self._lock:
            max_scan_id = max(
                (
                    s.scan_id
                    for s in self._scans.values()
                    if s.session_row_id == session_row_id
                ),
                default=0,
            )
            return max_scan_id + 1

    def insert_scan(self, record: ScanRecord) -> int:
        with self._lock:
            # (session_row_id, scan_id) unique 자리 check
            for s in self._scans.values():
                if (
                    s.session_row_id == record.session_row_id
                    and s.scan_id == record.scan_id
                ):
                    raise ValueError(
                        f"scan (session_row_id={record.session_row_id}, "
                        f"scan_id={record.scan_id}) 이미 존재"
                    )
            row_id = self._next_scan_id
            self._next_scan_id += 1
            self._scans[row_id] = record.model_copy(update={"id": row_id})
            return row_id

    def list_scans(self, session_row_id: int) -> list[ScanRecord]:
        with self._lock:
            matching = [
                s for s in self._scans.values()
                if s.session_row_id == session_row_id
            ]
        matching.sort(key=lambda s: s.scan_id)
        return [copy.deepcopy(s) for s in matching]

    def get_scan(self, scan_row_id: int) -> ScanRecord | None:
        with self._lock:
            r = self._scans.get(scan_row_id)
            return copy.deepcopy(r) if r else None

    def delete_scan(self, scan_row_id: int) -> None:
        with self._lock:
            self._scans.pop(scan_row_id, None)

    # reconstructions
    def insert_reconstruction(self, record: ReconstructionRecord) -> int:
        with self._lock:
            row_id = self._next_reconstruction_id
            self._next_reconstruction_id += 1
            self._reconstructions[row_id] = record.model_copy(
                update={"id": row_id}
            )
            return row_id

    def list_reconstructions(
        self, session_row_id: int
    ) -> list[ReconstructionRecord]:
        with self._lock:
            matching = [
                r for r in self._reconstructions.values()
                if r.session_row_id == session_row_id
            ]
        matching.sort(key=lambda r: r.created_at, reverse=True)
        return [copy.deepcopy(r) for r in matching]

    def get_reconstruction(
        self, recon_row_id: int
    ) -> ReconstructionRecord | None:
        with self._lock:
            r = self._reconstructions.get(recon_row_id)
            return copy.deepcopy(r) if r else None

    def delete_reconstruction(self, recon_row_id: int) -> None:
        with self._lock:
            self._reconstructions.pop(recon_row_id, None)

    # ─── Phase 1 (캘) 자리 ────────────────────────────────────────

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
