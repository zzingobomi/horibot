"""Relational DB store — session-scoped repository aggregate (UnitOfWork).

`RdbStore.session()` context manager 가 transaction 경계. block 안에서 도메인
facade (`repos.calibration` / `repos.scan_workflow`) 가 같은 session 공유 →
multi-repo / multi-entity atomic transaction 자연. block exit 시 commit, exception
시 rollback (session_scope `__exit__`).

도메인 facade 내부는 Advanced Alchemy `SQLAlchemySyncRepository` entity sub-repo
컴포지션 + workflow 메서드. SQLAlchemy ORM 이 dialect 추상화 → Postgres 도입 시
같은 RdbStore + 다른 engine.
"""

from __future__ import annotations

import logging
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Iterator

from sqlalchemy import Engine

from modules.storage.rdb.base import session_scope
from modules.storage.rdb.repos.calibration import CalibrationRepo
from modules.storage.rdb.repos.scan_workflow import ScanWorkflowRepo

logger = logging.getLogger(__name__)


@dataclass
class RepoBundle:
    """한 transaction 안 도메인 facade 묶음."""

    calibration: CalibrationRepo
    scan_workflow: ScanWorkflowRepo


class RdbStore:
    """도메인 repository aggregate — session 단위 transaction 경계."""

    def __init__(self, engine: Engine) -> None:
        self._engine = engine
        logger.info("RdbStore 초기화: %s", engine.url)

    @contextmanager
    def session(self) -> Iterator[RepoBundle]:
        """transaction-scoped repo bundle.

        block 안 도메인 facade 들이 같은 session 공유 → multi-table atomic.
        block 정상 종료 시 commit, exception 시 rollback.
        """
        with session_scope(self._engine) as s:
            yield RepoBundle(
                calibration=CalibrationRepo(s),
                scan_workflow=ScanWorkflowRepo(s),
            )

    def close(self) -> None:
        """Engine dispose — connection pool 비움. Windows SQLite file lock 해제."""
        self._engine.dispose()
