"""SqliteStore — host_dev / host_pc 의 실 RDB backend (Phase 1).

3 테이블 (calibration_runs / calibration_results / calibration_captures) +
UNIQUE INDEX (robot_id, kind) WHERE is_active=true. 부팅 시 CREATE TABLE IF
NOT EXISTS — 첫 부팅이면 새로 만들고, 기존 DB 면 그대로 유지.

스레드 안전: `check_same_thread=False` 로 connection 1개 공유 + lock 으로 직렬화.
storage_node 의 Zenoh queryable handler 는 다른 스레드에서 호출될 수 있음.
"""

from __future__ import annotations

import json
import logging
import sqlite3
import threading
from pathlib import Path

from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationResultRecordAdapter,
    CalibrationRunRecord,
)
from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)

logger = logging.getLogger(__name__)


_SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS calibration_runs (
    id               INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id         TEXT    NOT NULL,
    started_at       REAL    NOT NULL,
    ended_at         REAL,
    operator         TEXT,
    note             TEXT,
    algorithm        TEXT    NOT NULL,
    algorithm_params TEXT    NOT NULL DEFAULT '{}',
    status           TEXT    NOT NULL DEFAULT 'success'
);

CREATE TABLE IF NOT EXISTS calibration_results (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id      INTEGER NOT NULL REFERENCES calibration_runs(id),
    robot_id    TEXT    NOT NULL,
    kind        TEXT    NOT NULL,
    created_at  REAL    NOT NULL,
    is_active   INTEGER NOT NULL DEFAULT 0,
    sigma_rot   REAL,
    sigma_t     REAL,
    result_data TEXT    NOT NULL
);

-- per-kind active row 1개만 — UNIQUE partial index. ACTIVATE transaction 의
-- "다른 row deactivate + 대상 activate" 가 한 transaction 안에서 일관 보장.
CREATE UNIQUE INDEX IF NOT EXISTS idx_calibration_results_active
    ON calibration_results(robot_id, kind)
    WHERE is_active = 1;

CREATE INDEX IF NOT EXISTS idx_calibration_results_lookup
    ON calibration_results(robot_id, kind, created_at DESC);

CREATE TABLE IF NOT EXISTS calibration_captures (
    id             INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id         INTEGER NOT NULL REFERENCES calibration_runs(id),
    pose_index     INTEGER NOT NULL,
    joint_angles   TEXT    NOT NULL,
    board_in_cam   TEXT,
    residual_rot   REAL,
    residual_trans REAL,
    weight         REAL
);

CREATE INDEX IF NOT EXISTS idx_calibration_captures_run
    ON calibration_captures(run_id, pose_index);

-- ─── Phase 2 — scan workflow ──────────────────────────────────────
-- append-only blob (ObjectStore) + immutable metadata row. is_active 자리 X.
-- FK CASCADE — delete_scan_session 자리 자식 row 자동 삭제 (storage_layer.md §6).

CREATE TABLE IF NOT EXISTS scan_sessions (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id    TEXT    NOT NULL,
    session_id  TEXT    NOT NULL,
    created_at  REAL    NOT NULL,
    label       TEXT,
    note        TEXT,
    UNIQUE(robot_id, session_id)
);

CREATE INDEX IF NOT EXISTS idx_scan_sessions_lookup
    ON scan_sessions(robot_id, created_at DESC);

CREATE TABLE IF NOT EXISTS scans (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    session_row_id  INTEGER NOT NULL REFERENCES scan_sessions(id) ON DELETE CASCADE,
    robot_id        TEXT    NOT NULL,
    scan_id         INTEGER NOT NULL,
    created_at      REAL    NOT NULL,
    blob_key        TEXT    NOT NULL,
    num_frames      INTEGER NOT NULL,
    width           INTEGER NOT NULL,
    height          INTEGER NOT NULL,
    fx              REAL    NOT NULL,
    fy              REAL    NOT NULL,
    cx              REAL    NOT NULL,
    cy              REAL    NOT NULL,
    depth_scale     REAL    NOT NULL,
    motor_positions TEXT    NOT NULL,   -- JSON list[int]
    arm_motor_ids   TEXT    NOT NULL,   -- JSON list[int]
    UNIQUE(session_row_id, scan_id)
);

CREATE INDEX IF NOT EXISTS idx_scans_session
    ON scans(session_row_id, scan_id ASC);

CREATE TABLE IF NOT EXISTS reconstructions (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    session_row_id  INTEGER NOT NULL REFERENCES scan_sessions(id) ON DELETE CASCADE,
    robot_id        TEXT    NOT NULL,
    created_at      REAL    NOT NULL,
    blob_key        TEXT    NOT NULL,
    voxel_size      REAL    NOT NULL,
    sdf_trunc       REAL    NOT NULL,
    depth_trunc     REAL    NOT NULL,
    icp_max_dist    REAL    NOT NULL,
    n_scans         INTEGER NOT NULL,
    n_edges         INTEGER NOT NULL,
    vertex_count    INTEGER NOT NULL,
    triangle_count  INTEGER NOT NULL,
    elapsed         REAL    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_reconstructions_session
    ON reconstructions(session_row_id, created_at DESC);
"""


class SqliteStore:
    def __init__(self, path: Path):
        self._path = path
        # check_same_thread=False — Zenoh handler thread 들이 같은 connection 사용.
        # 자체 lock 으로 직렬화.
        self._conn = sqlite3.connect(
            str(path), check_same_thread=False, isolation_level=None
        )
        self._conn.execute("PRAGMA foreign_keys = ON")
        self._conn.execute("PRAGMA journal_mode = WAL")
        self._conn.row_factory = sqlite3.Row
        self._lock = threading.Lock()
        with self._lock:
            self._conn.executescript(_SCHEMA_SQL)
        logger.info("SqliteStore 초기화: %s", path)

    # ─── Read ─────────────────────────────────────────────────

    def get_active_result(
        self, robot_id: str, kind: CalibrationKind
    ) -> CalibrationResultRecord | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM calibration_results "
                "WHERE robot_id=? AND kind=? AND is_active=1",
                (robot_id, kind),
            ).fetchone()
        return _row_to_result(row) if row else None

    def list_results(
        self, robot_id: str, kind: CalibrationKind, limit: int = 100
    ) -> list[CalibrationResultRecord]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM calibration_results "
                "WHERE robot_id=? AND kind=? "
                "ORDER BY created_at DESC LIMIT ?",
                (robot_id, kind, limit),
            ).fetchall()
        return [_row_to_result(r) for r in rows]

    def list_runs(
        self, robot_id: str, limit: int = 50
    ) -> list[tuple[CalibrationRunRecord, list[CalibrationResultRecord]]]:
        with self._lock:
            run_rows = self._conn.execute(
                "SELECT * FROM calibration_runs "
                "WHERE robot_id=? "
                "ORDER BY started_at DESC LIMIT ?",
                (robot_id, limit),
            ).fetchall()
            runs = [_row_to_run(r) for r in run_rows]
            if not runs:
                return []
            # 한 번에 모든 Result fetch — N+1 query 회피.
            run_ids = [r.id for r in runs if r.id is not None]
            placeholders = ",".join("?" * len(run_ids))
            result_rows = self._conn.execute(
                f"SELECT * FROM calibration_results "
                f"WHERE run_id IN ({placeholders}) "
                f"ORDER BY created_at DESC",
                run_ids,
            ).fetchall()
        results_by_run: dict[int, list[CalibrationResultRecord]] = {
            rid: [] for rid in run_ids
        }
        for row in result_rows:
            result = _row_to_result(row)
            results_by_run.setdefault(row["run_id"], []).append(result)
        return [
            (run, results_by_run.get(run.id, []) if run.id is not None else [])
            for run in runs
        ]

    def get_result(self, result_id: int) -> CalibrationResultRecord | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM calibration_results WHERE id=?", (result_id,)
            ).fetchone()
        return _row_to_result(row) if row else None

    def get_run(self, run_id: int) -> CalibrationRunRecord | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM calibration_runs WHERE id=?", (run_id,)
            ).fetchone()
        return _row_to_run(row) if row else None

    def list_captures(self, run_id: int) -> list[CalibrationCaptureRecord]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM calibration_captures "
                "WHERE run_id=? ORDER BY pose_index ASC",
                (run_id,),
            ).fetchall()
        return [_row_to_capture(r) for r in rows]

    # ─── Write — atomic transaction ──────────────────────────

    def commit_calibration(
        self,
        run: CalibrationRunRecord,
        results: list[CalibrationResultRecord],
        captures: list[CalibrationCaptureRecord],
    ) -> tuple[int, list[int]]:
        with self._lock:
            try:
                self._conn.execute("BEGIN")
                cur = self._conn.execute(
                    "INSERT INTO calibration_runs "
                    "(robot_id, started_at, ended_at, operator, note, "
                    " algorithm, algorithm_params, status) "
                    "VALUES (?,?,?,?,?,?,?,?)",
                    (
                        run.robot_id,
                        run.started_at,
                        run.ended_at,
                        run.operator,
                        run.note,
                        run.algorithm,
                        json.dumps(run.algorithm_params),
                        run.status,
                    ),
                )
                run_id = int(cur.lastrowid or 0)

                result_ids: list[int] = []
                for r in results:
                    cur = self._conn.execute(
                        "INSERT INTO calibration_results "
                        "(run_id, robot_id, kind, created_at, is_active, "
                        " sigma_rot, sigma_t, result_data) "
                        "VALUES (?,?,?,?,?,?,?,?)",
                        (
                            run_id,
                            r.robot_id,
                            r.kind,
                            r.created_at,
                            0,  # COMMIT 시점 always is_active=false
                            r.sigma_rot,
                            r.sigma_t,
                            r.result_data.model_dump_json(),
                        ),
                    )
                    result_ids.append(int(cur.lastrowid or 0))

                for c in captures:
                    self._conn.execute(
                        "INSERT INTO calibration_captures "
                        "(run_id, pose_index, joint_angles, board_in_cam, "
                        " residual_rot, residual_trans, weight) "
                        "VALUES (?,?,?,?,?,?,?)",
                        (
                            run_id,
                            c.pose_index,
                            json.dumps(c.joint_angles),
                            json.dumps(c.board_in_cam)
                            if c.board_in_cam is not None
                            else None,
                            c.residual_rot,
                            c.residual_trans,
                            c.weight,
                        ),
                    )

                self._conn.execute("COMMIT")
            except Exception:
                self._conn.execute("ROLLBACK")
                raise

        return run_id, result_ids

    def close(self) -> None:
        """process 종료 시 connection 명시 close (Windows 에서 파일 lock 해제)."""
        with self._lock:
            self._conn.close()

    # ─── Phase 2 — scan workflow ──────────────────────────────────

    # scan_sessions
    def insert_scan_session(self, record: ScanSessionRecord) -> int:
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO scan_sessions "
                "(robot_id, session_id, created_at, label, note) "
                "VALUES (?,?,?,?,?)",
                (
                    record.robot_id,
                    record.session_id,
                    record.created_at,
                    record.label,
                    record.note,
                ),
            )
            return int(cur.lastrowid or 0)

    def get_scan_session(self, session_row_id: int) -> ScanSessionRecord | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM scan_sessions WHERE id=?", (session_row_id,)
            ).fetchone()
        return _row_to_scan_session(row) if row else None

    def find_scan_session_by_id(
        self, robot_id: str, session_id: str
    ) -> ScanSessionRecord | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM scan_sessions WHERE robot_id=? AND session_id=?",
                (robot_id, session_id),
            ).fetchone()
        return _row_to_scan_session(row) if row else None

    def list_scan_sessions(
        self, robot_id: str, limit: int = 100
    ) -> list[ScanSessionRecord]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM scan_sessions "
                "WHERE robot_id=? ORDER BY created_at DESC LIMIT ?",
                (robot_id, limit),
            ).fetchall()
        return [_row_to_scan_session(r) for r in rows]

    def delete_scan_session(self, session_row_id: int) -> None:
        # FK ON DELETE CASCADE — scans / reconstructions 자동 삭제.
        with self._lock:
            self._conn.execute(
                "DELETE FROM scan_sessions WHERE id=?", (session_row_id,)
            )

    # scans
    def allocate_scan_id(self, session_row_id: int) -> int:
        # transaction lock 안 MAX+1 — concurrent insert 시 race 차단.
        with self._lock:
            try:
                self._conn.execute("BEGIN")
                row = self._conn.execute(
                    "SELECT COALESCE(MAX(scan_id), 0) + 1 AS next_id "
                    "FROM scans WHERE session_row_id=?",
                    (session_row_id,),
                ).fetchone()
                next_id = int(row["next_id"])
                self._conn.execute("COMMIT")
                return next_id
            except Exception:
                self._conn.execute("ROLLBACK")
                raise

    def insert_scan(self, record: ScanRecord) -> int:
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO scans "
                "(session_row_id, robot_id, scan_id, created_at, blob_key, "
                " num_frames, width, height, fx, fy, cx, cy, depth_scale, "
                " motor_positions, arm_motor_ids) "
                "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    record.session_row_id,
                    record.robot_id,
                    record.scan_id,
                    record.created_at,
                    record.blob_key,
                    record.num_frames,
                    record.width,
                    record.height,
                    record.fx,
                    record.fy,
                    record.cx,
                    record.cy,
                    record.depth_scale,
                    json.dumps(record.motor_positions),
                    json.dumps(record.arm_motor_ids),
                ),
            )
            return int(cur.lastrowid or 0)

    def list_scans(self, session_row_id: int) -> list[ScanRecord]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM scans WHERE session_row_id=? "
                "ORDER BY scan_id ASC",
                (session_row_id,),
            ).fetchall()
        return [_row_to_scan(r) for r in rows]

    def get_scan(self, scan_row_id: int) -> ScanRecord | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM scans WHERE id=?", (scan_row_id,)
            ).fetchone()
        return _row_to_scan(row) if row else None

    def delete_scan(self, scan_row_id: int) -> None:
        with self._lock:
            self._conn.execute("DELETE FROM scans WHERE id=?", (scan_row_id,))

    # reconstructions
    def insert_reconstruction(self, record: ReconstructionRecord) -> int:
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO reconstructions "
                "(session_row_id, robot_id, created_at, blob_key, "
                " voxel_size, sdf_trunc, depth_trunc, icp_max_dist, "
                " n_scans, n_edges, vertex_count, triangle_count, elapsed) "
                "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    record.session_row_id,
                    record.robot_id,
                    record.created_at,
                    record.blob_key,
                    record.voxel_size,
                    record.sdf_trunc,
                    record.depth_trunc,
                    record.icp_max_dist,
                    record.n_scans,
                    record.n_edges,
                    record.vertex_count,
                    record.triangle_count,
                    record.elapsed,
                ),
            )
            return int(cur.lastrowid or 0)

    def list_reconstructions(
        self, session_row_id: int
    ) -> list[ReconstructionRecord]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM reconstructions WHERE session_row_id=? "
                "ORDER BY created_at DESC",
                (session_row_id,),
            ).fetchall()
        return [_row_to_reconstruction(r) for r in rows]

    def get_reconstruction(
        self, recon_row_id: int
    ) -> ReconstructionRecord | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM reconstructions WHERE id=?", (recon_row_id,)
            ).fetchone()
        return _row_to_reconstruction(row) if row else None

    def delete_reconstruction(self, recon_row_id: int) -> None:
        with self._lock:
            self._conn.execute(
                "DELETE FROM reconstructions WHERE id=?", (recon_row_id,)
            )

    # ─── Phase 1 (캘) 자리 ────────────────────────────────────────

    def activate_result(self, result_id: int) -> CalibrationResultRecord:
        with self._lock:
            try:
                self._conn.execute("BEGIN")
                row = self._conn.execute(
                    "SELECT * FROM calibration_results WHERE id=?", (result_id,)
                ).fetchone()
                if row is None:
                    self._conn.execute("ROLLBACK")
                    raise KeyError(f"result_id={result_id} 없음")

                # 같은 (robot_id, kind) 의 기존 active row 들 deactivate. UNIQUE
                # partial index 의 일관성 위해 deactivate 가 먼저, activate 가 나중.
                self._conn.execute(
                    "UPDATE calibration_results SET is_active=0 "
                    "WHERE robot_id=? AND kind=? AND is_active=1 AND id<>?",
                    (row["robot_id"], row["kind"], result_id),
                )
                self._conn.execute(
                    "UPDATE calibration_results SET is_active=1 WHERE id=?",
                    (result_id,),
                )
                self._conn.execute("COMMIT")

                updated = self._conn.execute(
                    "SELECT * FROM calibration_results WHERE id=?",
                    (result_id,),
                ).fetchone()
            except Exception:
                self._conn.execute("ROLLBACK")
                raise
        return _row_to_result(updated)


# ─── row → Pydantic record 변환 helper ───────────────────


def _row_to_run(row: sqlite3.Row) -> CalibrationRunRecord:
    return CalibrationRunRecord(
        id=row["id"],
        robot_id=row["robot_id"],
        started_at=row["started_at"],
        ended_at=row["ended_at"],
        operator=row["operator"],
        note=row["note"],
        algorithm=row["algorithm"],
        algorithm_params=json.loads(row["algorithm_params"] or "{}"),
        status=row["status"],
    )


def _row_to_result(row: sqlite3.Row) -> CalibrationResultRecord:
    # TypeAdapter 가 `kind` 보고 union arm 자동 선택 + result_data 를 알맞은
    # ResultData 모델로 validate. drift / 잘못된 (kind, result_data) 즉시 ValidationError.
    return CalibrationResultRecordAdapter.validate_python(
        {
            "id": row["id"],
            "run_id": row["run_id"],
            "robot_id": row["robot_id"],
            "kind": row["kind"],
            "created_at": row["created_at"],
            "is_active": bool(row["is_active"]),
            "sigma_rot": row["sigma_rot"],
            "sigma_t": row["sigma_t"],
            "result_data": json.loads(row["result_data"]),
        }
    )


def _row_to_capture(row: sqlite3.Row) -> CalibrationCaptureRecord:
    board = json.loads(row["board_in_cam"]) if row["board_in_cam"] else None
    return CalibrationCaptureRecord(
        id=row["id"],
        run_id=row["run_id"],
        pose_index=row["pose_index"],
        joint_angles=json.loads(row["joint_angles"]),
        board_in_cam=board,
        residual_rot=row["residual_rot"],
        residual_trans=row["residual_trans"],
        weight=row["weight"],
    )


# ─── Phase 2 row converters ──────────────────────────────────


def _row_to_scan_session(row: sqlite3.Row) -> ScanSessionRecord:
    return ScanSessionRecord(
        id=row["id"],
        robot_id=row["robot_id"],
        session_id=row["session_id"],
        created_at=row["created_at"],
        label=row["label"],
        note=row["note"],
    )


def _row_to_scan(row: sqlite3.Row) -> ScanRecord:
    return ScanRecord(
        id=row["id"],
        session_row_id=row["session_row_id"],
        robot_id=row["robot_id"],
        scan_id=row["scan_id"],
        created_at=row["created_at"],
        blob_key=row["blob_key"],
        num_frames=row["num_frames"],
        width=row["width"],
        height=row["height"],
        fx=row["fx"],
        fy=row["fy"],
        cx=row["cx"],
        cy=row["cy"],
        depth_scale=row["depth_scale"],
        motor_positions=json.loads(row["motor_positions"]),
        arm_motor_ids=json.loads(row["arm_motor_ids"]),
    )


def _row_to_reconstruction(row: sqlite3.Row) -> ReconstructionRecord:
    return ReconstructionRecord(
        id=row["id"],
        session_row_id=row["session_row_id"],
        robot_id=row["robot_id"],
        created_at=row["created_at"],
        blob_key=row["blob_key"],
        voxel_size=row["voxel_size"],
        sdf_trunc=row["sdf_trunc"],
        depth_trunc=row["depth_trunc"],
        icp_max_dist=row["icp_max_dist"],
        n_scans=row["n_scans"],
        n_edges=row["n_edges"],
        vertex_count=row["vertex_count"],
        triangle_count=row["triangle_count"],
        elapsed=row["elapsed"],
    )
