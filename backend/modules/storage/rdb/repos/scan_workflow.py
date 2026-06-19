"""Scan workflow domain repository — sessions / scans / reconstructions 3 entity.

append-only blob + immutable metadata row 패턴 — is_active / ACTIVATE 자리 X
(캘 특유 패턴 안 빌림, storage_layer.md §3).

Advanced Alchemy `SQLAlchemySyncRepository` 위 도메인 facade — caller 가 session
주입 (RdbStore.session() context manager). `auto_commit=False` → 메서드 안 flush
만, commit 은 session_scope `__exit__` 가 담당.
"""

from __future__ import annotations

import logging

from advanced_alchemy.filters import LimitOffset
from advanced_alchemy.repository import SQLAlchemySyncRepository
from sqlalchemy import func, select
from sqlalchemy.orm import Session

from modules.scan_workflow.orm import (
    ReconstructionOrm,
    ScanOrm,
    ScanSessionOrm,
    orm_to_reconstruction,
    orm_to_scan,
    orm_to_scan_session,
    reconstruction_record_to_orm,
    scan_record_to_orm,
    session_record_to_orm,
)
from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)

logger = logging.getLogger(__name__)


# ─── Entity sub-repos (Advanced Alchemy CRUD baseline) ───
# pyright 자리 ModelProtocol 호환성 자리 ignore — calibration.py 와 같은 이유.


class _SessionRepo(SQLAlchemySyncRepository[ScanSessionOrm]):  # type: ignore[type-var]
    model_type = ScanSessionOrm


class _ScanRepo(SQLAlchemySyncRepository[ScanOrm]):  # type: ignore[type-var]
    model_type = ScanOrm


class _ReconstructionRepo(SQLAlchemySyncRepository[ReconstructionOrm]):  # type: ignore[type-var]
    model_type = ReconstructionOrm


# ─── Domain facade ───


class ScanWorkflowRepo:
    """scan session + scan + reconstruction 3 entity — entity sub-repo 컴포지션."""

    def __init__(self, session: Session) -> None:
        self.session = session
        self.sessions = _SessionRepo(session=session, wrap_exceptions=False)
        self.scans = _ScanRepo(session=session, wrap_exceptions=False)
        self.reconstructions = _ReconstructionRepo(
            session=session, wrap_exceptions=False
        )

    # ─── sessions ────────────────────────────────────────────

    def insert_session(self, record: ScanSessionRecord) -> int:
        # (robot_id, session_id) UNIQUE — pre-check 로 친절한 ValueError.
        existing = self.sessions.get_one_or_none(
            ScanSessionOrm.robot_id == record.robot_id,
            ScanSessionOrm.session_id == record.session_id,
        )
        if existing is not None:
            raise ValueError(
                f"scan_session (robot_id={record.robot_id}, "
                f"session_id={record.session_id}) 이미 존재"
            )
        orm = self.sessions.add(session_record_to_orm(record))
        assert orm.id is not None
        return orm.id

    def get_session(self, session_row_id: int) -> ScanSessionRecord | None:
        orm = self.sessions.get_one_or_none(ScanSessionOrm.id == session_row_id)
        return orm_to_scan_session(orm) if orm else None

    def find_session_by_id(
        self, robot_id: str, session_id: str
    ) -> ScanSessionRecord | None:
        orm = self.sessions.get_one_or_none(
            ScanSessionOrm.robot_id == robot_id,
            ScanSessionOrm.session_id == session_id,
        )
        return orm_to_scan_session(orm) if orm else None

    def list_sessions(
        self, robot_id: str, limit: int = 100
    ) -> list[ScanSessionRecord]:
        orms = self.sessions.get_many(
            ScanSessionOrm.robot_id == robot_id,
            LimitOffset(limit=limit, offset=0),
            order_by=ScanSessionOrm.created_at.desc(),
        )
        return [orm_to_scan_session(o) for o in orms]

    def delete_session(self, session_row_id: int) -> None:
        # FK ON DELETE CASCADE — scans / reconstructions 자동 삭제.
        orm = self.sessions.get_one_or_none(ScanSessionOrm.id == session_row_id)
        if orm is not None:
            self.session.delete(orm)
            self.session.flush()

    # ─── scans ───────────────────────────────────────────────

    def allocate_scan_id(self, session_row_id: int) -> int:
        # transaction lock 안 MAX+1 — concurrent insert 시 race 차단.
        row = self.session.execute(
            select(func.coalesce(func.max(ScanOrm.scan_id), 0)).where(
                ScanOrm.session_row_id == session_row_id
            )
        ).scalar_one()
        return int(row) + 1

    def insert_scan(self, record: ScanRecord) -> int:
        # (session_row_id, scan_id) UNIQUE — pre-check.
        existing = self.scans.get_one_or_none(
            ScanOrm.session_row_id == record.session_row_id,
            ScanOrm.scan_id == record.scan_id,
        )
        if existing is not None:
            raise ValueError(
                f"scan (session_row_id={record.session_row_id}, "
                f"scan_id={record.scan_id}) 이미 존재"
            )
        orm = self.scans.add(scan_record_to_orm(record))
        assert orm.id is not None
        return orm.id

    def list_scans(self, session_row_id: int) -> list[ScanRecord]:
        orms = self.scans.get_many(
            ScanOrm.session_row_id == session_row_id,
            order_by=ScanOrm.scan_id.asc(),
        )
        return [orm_to_scan(o) for o in orms]

    def get_scan(self, scan_row_id: int) -> ScanRecord | None:
        orm = self.scans.get_one_or_none(ScanOrm.id == scan_row_id)
        return orm_to_scan(orm) if orm else None

    def delete_scan(self, scan_row_id: int) -> None:
        orm = self.scans.get_one_or_none(ScanOrm.id == scan_row_id)
        if orm is not None:
            self.session.delete(orm)
            self.session.flush()

    # ─── reconstructions ─────────────────────────────────────

    def insert_reconstruction(self, record: ReconstructionRecord) -> int:
        orm = self.reconstructions.add(reconstruction_record_to_orm(record))
        assert orm.id is not None
        return orm.id

    def list_reconstructions(
        self, session_row_id: int
    ) -> list[ReconstructionRecord]:
        orms = self.reconstructions.get_many(
            ReconstructionOrm.session_row_id == session_row_id,
            order_by=ReconstructionOrm.created_at.desc(),
        )
        return [orm_to_reconstruction(o) for o in orms]

    def get_reconstruction(
        self, recon_row_id: int
    ) -> ReconstructionRecord | None:
        orm = self.reconstructions.get_one_or_none(ReconstructionOrm.id == recon_row_id)
        return orm_to_reconstruction(orm) if orm else None

    def delete_reconstruction(self, recon_row_id: int) -> None:
        orm = self.reconstructions.get_one_or_none(ReconstructionOrm.id == recon_row_id)
        if orm is not None:
            self.session.delete(orm)
            self.session.flush()
