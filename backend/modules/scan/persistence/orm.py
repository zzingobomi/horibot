"""Scan ORM — 3 entity (scan_sessions / scans / reconstructions).

scan 모듈이 자기 영속성 소유 (Database-per-Module). 공유 infra Base 에 등록
(소유는 이 모듈, 마이그레이션은 루트 alembic). 옛 backend scan_workflow/orm.py 이월.
"""

from __future__ import annotations

import json
from datetime import datetime

from sqlalchemy import (
    Float,
    ForeignKey,
    Index,
    Integer,
    String,
    Text,
    UniqueConstraint,
)
from sqlalchemy.orm import Mapped, mapped_column

from infra.database.base import Base
from infra.database.types import UtcDateTime

from ..contract import ReconstructionRecord, ScanRecord, ScanSessionRecord


class ScanSessionOrm(Base):
    __tablename__ = "scan_sessions"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    session_id: Mapped[str] = mapped_column(String, nullable=False)
    created_at: Mapped[datetime] = mapped_column(UtcDateTime, nullable=False)
    label: Mapped[str | None] = mapped_column(Text, nullable=True)

    __table_args__ = (
        UniqueConstraint("robot_id", "session_id"),
        Index("idx_scan_sessions_robot", "robot_id", "created_at"),
    )


class ScanOrm(Base):
    __tablename__ = "scans"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    session_row_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("scan_sessions.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    scan_id: Mapped[int] = mapped_column(Integer, nullable=False)
    created_at: Mapped[datetime] = mapped_column(UtcDateTime, nullable=False)
    blob_key: Mapped[str] = mapped_column(String, nullable=False)
    num_frames: Mapped[int] = mapped_column(Integer, nullable=False)
    width: Mapped[int] = mapped_column(Integer, nullable=False)
    height: Mapped[int] = mapped_column(Integer, nullable=False)
    fx: Mapped[float] = mapped_column(Float, nullable=False)
    fy: Mapped[float] = mapped_column(Float, nullable=False)
    cx: Mapped[float] = mapped_column(Float, nullable=False)
    cy: Mapped[float] = mapped_column(Float, nullable=False)
    depth_scale: Mapped[float] = mapped_column(Float, nullable=False)
    motor_positions: Mapped[str] = mapped_column(Text, nullable=False)  # JSON list[int]
    arm_motor_ids: Mapped[str] = mapped_column(Text, nullable=False)  # JSON list[int]

    __table_args__ = (
        Index("idx_scans_session", "session_row_id", "scan_id"),
    )


class ReconstructionOrm(Base):
    __tablename__ = "reconstructions"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    session_row_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("scan_sessions.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    created_at: Mapped[datetime] = mapped_column(UtcDateTime, nullable=False)
    blob_key: Mapped[str] = mapped_column(String, nullable=False)
    voxel_size: Mapped[float] = mapped_column(Float, nullable=False)
    sdf_trunc: Mapped[float] = mapped_column(Float, nullable=False)
    depth_trunc: Mapped[float] = mapped_column(Float, nullable=False)
    icp_max_dist: Mapped[float] = mapped_column(Float, nullable=False)
    n_scans: Mapped[int] = mapped_column(Integer, nullable=False)
    n_edges: Mapped[int] = mapped_column(Integer, nullable=False)
    vertex_count: Mapped[int] = mapped_column(Integer, nullable=False)
    triangle_count: Mapped[int] = mapped_column(Integer, nullable=False)
    elapsed: Mapped[float] = mapped_column(Float, nullable=False)

    __table_args__ = (
        Index("idx_reconstructions_session", "session_row_id", "created_at"),
    )


# ─── record ↔ ORM mapper ────────────────────────────────────────────


def session_to_orm(rec: ScanSessionRecord) -> ScanSessionOrm:
    return ScanSessionOrm(
        robot_id=rec.robot_id,
        session_id=rec.session_id,
        created_at=rec.created_at,
        label=rec.label,
    )


def orm_to_session(orm: ScanSessionOrm) -> ScanSessionRecord:
    return ScanSessionRecord(
        id=orm.id,
        robot_id=orm.robot_id,
        session_id=orm.session_id,
        created_at=orm.created_at,
        label=orm.label,
    )


def scan_to_orm(rec: ScanRecord) -> ScanOrm:
    return ScanOrm(
        session_row_id=rec.session_row_id,
        robot_id=rec.robot_id,
        scan_id=rec.scan_id,
        created_at=rec.created_at,
        blob_key=rec.blob_key,
        num_frames=rec.num_frames,
        width=rec.width,
        height=rec.height,
        fx=rec.fx,
        fy=rec.fy,
        cx=rec.cx,
        cy=rec.cy,
        depth_scale=rec.depth_scale,
        motor_positions=json.dumps(rec.motor_positions),
        arm_motor_ids=json.dumps(rec.arm_motor_ids),
    )


def orm_to_scan(orm: ScanOrm) -> ScanRecord:
    return ScanRecord(
        id=orm.id,
        session_row_id=orm.session_row_id,
        robot_id=orm.robot_id,
        scan_id=orm.scan_id,
        created_at=orm.created_at,
        blob_key=orm.blob_key,
        num_frames=orm.num_frames,
        width=orm.width,
        height=orm.height,
        fx=orm.fx,
        fy=orm.fy,
        cx=orm.cx,
        cy=orm.cy,
        depth_scale=orm.depth_scale,
        motor_positions=[int(x) for x in json.loads(orm.motor_positions)],
        arm_motor_ids=[int(x) for x in json.loads(orm.arm_motor_ids)],
    )


def reconstruction_to_orm(rec: ReconstructionRecord) -> ReconstructionOrm:
    return ReconstructionOrm(
        session_row_id=rec.session_row_id,
        robot_id=rec.robot_id,
        created_at=rec.created_at,
        blob_key=rec.blob_key,
        voxel_size=rec.voxel_size,
        sdf_trunc=rec.sdf_trunc,
        depth_trunc=rec.depth_trunc,
        icp_max_dist=rec.icp_max_dist,
        n_scans=rec.n_scans,
        n_edges=rec.n_edges,
        vertex_count=rec.vertex_count,
        triangle_count=rec.triangle_count,
        elapsed=rec.elapsed,
    )


def orm_to_reconstruction(orm: ReconstructionOrm) -> ReconstructionRecord:
    return ReconstructionRecord(
        id=orm.id,
        session_row_id=orm.session_row_id,
        robot_id=orm.robot_id,
        created_at=orm.created_at,
        blob_key=orm.blob_key,
        voxel_size=orm.voxel_size,
        sdf_trunc=orm.sdf_trunc,
        depth_trunc=orm.depth_trunc,
        icp_max_dist=orm.icp_max_dist,
        n_scans=orm.n_scans,
        n_edges=orm.n_edges,
        vertex_count=orm.vertex_count,
        triangle_count=orm.triangle_count,
        elapsed=orm.elapsed,
    )
