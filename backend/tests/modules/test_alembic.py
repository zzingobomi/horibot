"""루트 Alembic 마이그레이션 검증 — upgrade head 가 ORM 과 일치하는 스키마를 만들고
partial unique 가 실제로 강제되는지 (create_all 이 아니라 진짜 migration 경로).
"""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

import pytest
from alembic import command
from alembic.config import Config
from sqlalchemy import inspect
from sqlalchemy.exc import IntegrityError

from infra.database.base import Base
from infra.database.sqlite import open_sqlite
from modules.calibration.persistence.orm import CalibrationResultOrm, CalibrationRunOrm

_ALEMBIC_INI = Path(__file__).resolve().parents[2] / "alembic.ini"
_CALIB_TABLES = {
    "calibration_runs",
    "calibration_results",
    "calibration_captures",
    "calibration_capture_artifacts",
}


def _upgrade_head(db_path: Path) -> None:
    cfg = Config(str(_ALEMBIC_INI))
    cfg.set_main_option("sqlalchemy.url", f"sqlite:///{db_path}")
    command.upgrade(cfg, "head")


def test_upgrade_head_creates_all_orm_tables(tmp_path: Path):
    db = tmp_path / "migrated.db"
    _upgrade_head(db)

    eng, _ = open_sqlite(db)
    names = set(inspect(eng).get_table_names())
    assert _CALIB_TABLES <= names
    assert "alembic_version" in names  # 단일 history version table


def test_migration_schema_matches_metadata(tmp_path: Path):
    """migration 이 만든 테이블 == ORM metadata (drift 없음)."""
    migrated = tmp_path / "migrated.db"
    _upgrade_head(migrated)
    created = tmp_path / "created.db"
    eng_c, _ = open_sqlite(created)
    Base.metadata.create_all(eng_c)
    eng_m, _ = open_sqlite(migrated)

    insp_m, insp_c = inspect(eng_m), inspect(eng_c)
    tbls_m = {t for t in insp_m.get_table_names() if t in _CALIB_TABLES}
    tbls_c = {t for t in insp_c.get_table_names() if t in _CALIB_TABLES}
    assert tbls_m == tbls_c
    for t in _CALIB_TABLES:
        assert {c["name"] for c in insp_m.get_columns(t)} == {
            c["name"] for c in insp_c.get_columns(t)
        }, f"{t} 컬럼 drift"
        assert {i["name"] for i in insp_m.get_indexes(t)} == {
            i["name"] for i in insp_c.get_indexes(t)
        }, f"{t} 인덱스 drift"


def test_partial_unique_active_enforced_on_migrated_db(tmp_path: Path):
    """migration 의 idx_calibration_results_active (partial unique) 가 실제로 강제."""
    db = tmp_path / "migrated.db"
    _upgrade_head(db)
    _, factory = open_sqlite(db)

    with factory() as s:
        run = CalibrationRunOrm(
            robot_id="r", started_at=datetime.now(UTC), algorithm="x",
            algorithm_params="{}", status="in_progress", kind="hand_eye",
        )
        s.add(run)
        s.commit()
        run_id = run.id

    def _mk_active() -> CalibrationResultOrm:
        return CalibrationResultOrm(
            run_id=run_id, robot_id="r", kind="hand_eye",
            created_at=datetime.now(UTC), is_active=True, result_data="{}",
        )

    with pytest.raises(IntegrityError):
        with factory() as s:
            s.add(_mk_active())
            s.add(_mk_active())  # 같은 (robot, kind) 두 active → partial unique 위반
            s.commit()
