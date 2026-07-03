"""ScanRepository — scan 모듈 영속성 (Database-per-Module).

calibration repository 와 동형: advanced-alchemy sub-repo 를 모듈 private CRUD 헬퍼로만
사용, 도메인 로직(allocate_scan_id monotonic / CASCADE delete)은 직접. 옛 backend
scan_workflow 의 repo 이월.
"""

from __future__ import annotations

from collections.abc import Callable

from advanced_alchemy.repository import SQLAlchemySyncRepository
from sqlalchemy import func, select
from sqlalchemy.orm import Session

from ..contract import ReconstructionRecord, ScanRecord, ScanSessionRecord
from .orm import (
    ReconstructionOrm,
    ScanOrm,
    ScanSessionOrm,
    orm_to_reconstruction,
    orm_to_scan,
    orm_to_session,
    reconstruction_to_orm,
    scan_to_orm,
    session_to_orm,
)


class _SessionRepo(SQLAlchemySyncRepository[ScanSessionOrm]):
    model_type = ScanSessionOrm


class _ScanRepo(SQLAlchemySyncRepository[ScanOrm]):
    model_type = ScanOrm


class _ReconRepo(SQLAlchemySyncRepository[ReconstructionOrm]):
    model_type = ReconstructionOrm


class ScanRepository:
    def __init__(self, session_factory: Callable[[], Session]):
        self._session_factory = session_factory

    # ── sessions ──────────────────────────────────────────────
    def insert_session(self, rec: ScanSessionRecord) -> ScanSessionRecord:
        with self._session_factory() as session:
            repo = _SessionRepo(session=session, auto_commit=True, wrap_exceptions=False)
            orm = repo.add(session_to_orm(rec))
            return orm_to_session(orm)

    def get_session(self, session_row_id: int) -> ScanSessionRecord | None:
        with self._session_factory() as session:
            orm = _SessionRepo(
                session=session, wrap_exceptions=False
            ).get_one_or_none(ScanSessionOrm.id == session_row_id)
            return orm_to_session(orm) if orm is not None else None

    def list_sessions(self, robot_id: str) -> list[ScanSessionRecord]:
        with self._session_factory() as session:
            rows = _SessionRepo(session=session, wrap_exceptions=False).get_many(
                ScanSessionOrm.robot_id == robot_id,
                order_by=ScanSessionOrm.id.desc(),
            )
            return [orm_to_session(o) for o in rows]

    def delete_session(self, session_row_id: int) -> None:
        with self._session_factory() as session:
            repo = _SessionRepo(session=session, auto_commit=True, wrap_exceptions=False)
            if repo.get_one_or_none(ScanSessionOrm.id == session_row_id) is None:
                raise KeyError(f"scan session {session_row_id} 없음")
            repo.delete(session_row_id)  # FK CASCADE → scans/reconstructions

    # ── scans ─────────────────────────────────────────────────
    def allocate_scan_id(self, session_row_id: int) -> int:
        """session 내 monotonic scan_id (MAX+1). 삭제해도 안 줄어듦."""
        with self._session_factory() as session:
            cur = session.scalar(
                select(func.max(ScanOrm.scan_id)).where(
                    ScanOrm.session_row_id == session_row_id
                )
            )
            return (cur or 0) + 1

    def insert_scan(self, rec: ScanRecord) -> ScanRecord:
        with self._session_factory() as session:
            repo = _ScanRepo(session=session, auto_commit=True, wrap_exceptions=False)
            orm = repo.add(scan_to_orm(rec))
            return orm_to_scan(orm)

    def get_scan(self, scan_row_id: int) -> ScanRecord | None:
        with self._session_factory() as session:
            orm = _ScanRepo(session=session, wrap_exceptions=False).get_one_or_none(
                ScanOrm.id == scan_row_id
            )
            return orm_to_scan(orm) if orm is not None else None

    def list_scans(self, session_row_id: int) -> list[ScanRecord]:
        with self._session_factory() as session:
            rows = _ScanRepo(session=session, wrap_exceptions=False).get_many(
                ScanOrm.session_row_id == session_row_id,
                order_by=ScanOrm.scan_id.asc(),
            )
            return [orm_to_scan(o) for o in rows]

    def delete_scan(self, scan_row_id: int) -> None:
        with self._session_factory() as session:
            repo = _ScanRepo(session=session, auto_commit=True, wrap_exceptions=False)
            if repo.get_one_or_none(ScanOrm.id == scan_row_id) is None:
                raise KeyError(f"scan {scan_row_id} 없음")
            repo.delete(scan_row_id)

    # ── reconstructions ───────────────────────────────────────
    def insert_reconstruction(self, rec: ReconstructionRecord) -> ReconstructionRecord:
        with self._session_factory() as session:
            repo = _ReconRepo(session=session, auto_commit=True, wrap_exceptions=False)
            orm = repo.add(reconstruction_to_orm(rec))
            return orm_to_reconstruction(orm)

    def get_reconstruction(self, recon_row_id: int) -> ReconstructionRecord | None:
        with self._session_factory() as session:
            orm = _ReconRepo(session=session, wrap_exceptions=False).get_one_or_none(
                ReconstructionOrm.id == recon_row_id
            )
            return orm_to_reconstruction(orm) if orm is not None else None

    def list_reconstructions(self, session_row_id: int) -> list[ReconstructionRecord]:
        with self._session_factory() as session:
            rows = _ReconRepo(session=session, wrap_exceptions=False).get_many(
                ReconstructionOrm.session_row_id == session_row_id,
                order_by=ReconstructionOrm.id.desc(),
            )
            return [orm_to_reconstruction(o) for o in rows]
