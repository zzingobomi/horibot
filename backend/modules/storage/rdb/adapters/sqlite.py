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
