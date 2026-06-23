"""일회용 — `horibot_bak.db` (옛 schema, float epoch) → `horibot.db` (새 schema, datetime).

개발 단계 자리 migration 파일 늘리지 않는 방향 — initial_schema.py 자체를
DateTime(timezone=True) 로 갱신 + 본 스크립트가 백업본 데이터를 변환해서 fresh DB
에 INSERT.

전제:
- target `horibot.db` 는 fresh schema (Alembic upgrade head 직후 빈 상태). 본
  스크립트는 데이터만 옮김 — schema 생성 X.
- source `horibot_bak.db` 는 옛 schema (float epoch). schema 자체는 호환
  (timestamp 컬럼만 type 다름).

순서 — FK 제약 자리:
  calibration_runs → calibration_captures → calibration_capture_artifacts → calibration_results
  scan_sessions → scans / reconstructions

CLI:
  uv run python scripts/migrate_bak_to_datetime.py
  uv run python scripts/migrate_bak_to_datetime.py --bak storage/horibot_bak.db --target storage/horibot.db
"""

from __future__ import annotations

import argparse
import logging
import sqlite3
import sys
from datetime import UTC, datetime
from pathlib import Path

logger = logging.getLogger(__name__)

BACKEND = Path(__file__).resolve().parents[1]

# (table, timestamp 컬럼 이름들) — initial_schema.py 와 정합.
_TIMESTAMP_COLUMNS: dict[str, tuple[str, ...]] = {
    "calibration_runs": ("started_at", "ended_at"),
    "calibration_results": ("created_at",),
    "calibration_capture_artifacts": ("created_at",),
    "scan_sessions": ("created_at",),
    "scans": ("created_at",),
    "reconstructions": ("created_at",),
    "calibration_captures": (),  # timestamp 컬럼 없음
}

# FK 의존 순서 — parent 가 앞.
_INSERT_ORDER = (
    "calibration_runs",
    "calibration_captures",
    "calibration_capture_artifacts",
    "calibration_results",
    "scan_sessions",
    "scans",
    "reconstructions",
)


def _epoch_to_iso(value: float | None) -> str | None:
    """float epoch seconds → ISO 8601 UTC string. SQLAlchemy DateTime(timezone=True) round-trip 호환."""
    if value is None:
        return None
    return datetime.fromtimestamp(value, tz=UTC).isoformat(sep=" ")


def _table_columns(con: sqlite3.Connection, table: str) -> list[str]:
    return [r[1] for r in con.execute(f"PRAGMA table_info({table})").fetchall()]


def _copy_table(
    bak: sqlite3.Connection,
    target: sqlite3.Connection,
    table: str,
    ts_columns: tuple[str, ...],
) -> int:
    """source 의 한 테이블 → target. source ∩ target 의 col 만 옮김 (schema drift 자리 source 쪽 *추가* col 자동 drop). timestamp col 자리 float → ISO 변환."""
    src_cols = _table_columns(bak, table)
    tgt_cols = _table_columns(target, table)
    common = [c for c in src_cols if c in tgt_cols]
    dropped = [c for c in src_cols if c not in tgt_cols]
    if dropped:
        logger.info("  %s: dropped source cols (target schema 자리 없음): %s",
                    table, ", ".join(dropped))

    cur_src = bak.execute(f"SELECT {','.join(common)} FROM {table}")
    rows = cur_src.fetchall()
    if not rows:
        logger.info("  %s: 0 rows", table)
        return 0

    ts_in_common = [c for c in ts_columns if c in common]
    ts_idx = [common.index(c) for c in ts_in_common]
    placeholders = ",".join(["?"] * len(common))
    insert_sql = (
        f"INSERT INTO {table} ({','.join(common)}) VALUES ({placeholders})"
    )

    converted_rows = []
    for row in rows:
        new_row = list(row)
        for i in ts_idx:
            new_row[i] = _epoch_to_iso(new_row[i])
        converted_rows.append(new_row)

    target.executemany(insert_sql, converted_rows)
    logger.info(
        "  %s: %d rows (ts cols: %s)",
        table,
        len(rows),
        ", ".join(ts_columns) if ts_columns else "—",
    )
    return len(rows)


def migrate(bak_path: Path, target_path: Path) -> dict[str, int]:
    if not bak_path.exists():
        raise FileNotFoundError(f"backup DB 없음: {bak_path}")
    if not target_path.exists():
        raise FileNotFoundError(
            f"target DB 없음: {target_path} — 먼저 `alembic upgrade head` 로 schema 생성"
        )

    bak = sqlite3.connect(str(bak_path))
    target = sqlite3.connect(str(target_path))
    target.execute("PRAGMA foreign_keys = ON")
    try:
        # target 비어 있는지 sanity — 데이터 덮어쓰기 사고 방지.
        for table in _INSERT_ORDER:
            n = target.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]
            if n > 0:
                raise RuntimeError(
                    f"target.{table} 에 이미 {n} rows — fresh DB 가 아님. "
                    f"target 비우고 다시 시도."
                )

        counts: dict[str, int] = {}
        for table in _INSERT_ORDER:
            ts_cols = _TIMESTAMP_COLUMNS[table]
            counts[table] = _copy_table(bak, target, table, ts_cols)

        target.commit()
        return counts
    except Exception:
        target.rollback()
        raise
    finally:
        bak.close()
        target.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bak", type=Path, default=BACKEND / "storage" / "horibot_bak.db",
        help="옛 schema 백업 DB",
    )
    parser.add_argument(
        "--target", type=Path, default=BACKEND / "storage" / "horibot.db",
        help="새 schema fresh DB (Alembic upgrade head 직후)",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname).1s] %(message)s",
    )
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except (AttributeError, Exception):
        pass

    logger.info("=== Migrate bak → fresh DB ===")
    logger.info("  bak    : %s", args.bak)
    logger.info("  target : %s", args.target)
    counts = migrate(args.bak, args.target)
    total = sum(counts.values())
    logger.info("=== 완료 — total %d rows ===", total)
    return 0


if __name__ == "__main__":
    sys.exit(main())
