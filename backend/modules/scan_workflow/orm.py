"""Scan workflow entity 의 SQLAlchemy ORM 모델 + Pydantic Record 변환.

scan_sessions / scans / reconstructions — append-only blob (ObjectStore) +
immutable metadata row 패턴. is_active / ACTIVATE 자리 X (캘 특유 패턴).
docs/storage_layer.md §3 + §6.
"""

from __future__ import annotations

import json

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

from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)
from modules.storage.rdb.base import Base


class ScanSessionOrm(Base):
    """Scan session — 한 번의 multi-pose scan 묶음."""

    __tablename__ = "scan_sessions"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    session_id: Mapped[str] = mapped_column(String, nullable=False)
    created_at: Mapped[float] = mapped_column(Float, nullable=False)
    label: Mapped[str | None] = mapped_column(Text, nullable=True)
    note: Mapped[str | None] = mapped_column(Text, nullable=True)

    __table_args__ = (
        UniqueConstraint("robot_id", "session_id"),
        Index("idx_scan_sessions_lookup", "robot_id", "created_at"),
    )


class ScanOrm(Base):
    """Scan — 한 자세에서 캡처한 RGBD frame metadata."""

    __tablename__ = "scans"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    session_row_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("scan_sessions.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    scan_id: Mapped[int] = mapped_column(Integer, nullable=False)
    created_at: Mapped[float] = mapped_column(Float, nullable=False)
    blob_key: Mapped[str] = mapped_column(String, nullable=False)
    num_frames: Mapped[int] = mapped_column(Integer, nullable=False)
    width: Mapped[int] = mapped_column(Integer, nullable=False)
    height: Mapped[int] = mapped_column(Integer, nullable=False)
    fx: Mapped[float] = mapped_column(Float, nullable=False)
    fy: Mapped[float] = mapped_column(Float, nullable=False)
    cx: Mapped[float] = mapped_column(Float, nullable=False)
    cy: Mapped[float] = mapped_column(Float, nullable=False)
    depth_scale: Mapped[float] = mapped_column(Float, nullable=False)
    # JSON list — motor_positions / arm_motor_ids
    motor_positions: Mapped[str] = mapped_column(Text, nullable=False)
    arm_motor_ids: Mapped[str] = mapped_column(Text, nullable=False)

    __table_args__ = (
        UniqueConstraint("session_row_id", "scan_id"),
        Index("idx_scans_session", "session_row_id", "scan_id"),
    )


class ReconstructionOrm(Base):
    """Reconstruction — multi-scan ICP+PoseGraph+TSDF mesh 결과 metadata."""

    __tablename__ = "reconstructions"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    session_row_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("scan_sessions.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    created_at: Mapped[float] = mapped_column(Float, nullable=False)
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


# ─── Pydantic Record ↔ ORM 양방향 boundary mapper ────────────────


def session_record_to_orm(record: ScanSessionRecord) -> ScanSessionOrm:
    return ScanSessionOrm(
        robot_id=record.robot_id,
        session_id=record.session_id,
        created_at=record.created_at,
        label=record.label,
        note=record.note,
    )


def scan_record_to_orm(record: ScanRecord) -> ScanOrm:
    return ScanOrm(
        session_row_id=record.session_row_id,
        robot_id=record.robot_id,
        scan_id=record.scan_id,
        created_at=record.created_at,
        blob_key=record.blob_key,
        num_frames=record.num_frames,
        width=record.width,
        height=record.height,
        fx=record.fx,
        fy=record.fy,
        cx=record.cx,
        cy=record.cy,
        depth_scale=record.depth_scale,
        motor_positions=json.dumps(record.motor_positions),
        arm_motor_ids=json.dumps(record.arm_motor_ids),
    )


def reconstruction_record_to_orm(record: ReconstructionRecord) -> ReconstructionOrm:
    return ReconstructionOrm(
        session_row_id=record.session_row_id,
        robot_id=record.robot_id,
        created_at=record.created_at,
        blob_key=record.blob_key,
        voxel_size=record.voxel_size,
        sdf_trunc=record.sdf_trunc,
        depth_trunc=record.depth_trunc,
        icp_max_dist=record.icp_max_dist,
        n_scans=record.n_scans,
        n_edges=record.n_edges,
        vertex_count=record.vertex_count,
        triangle_count=record.triangle_count,
        elapsed=record.elapsed,
    )


def orm_to_scan_session(orm: ScanSessionOrm) -> ScanSessionRecord:
    return ScanSessionRecord(
        id=orm.id,
        robot_id=orm.robot_id,
        session_id=orm.session_id,
        created_at=orm.created_at,
        label=orm.label,
        note=orm.note,
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
        motor_positions=json.loads(orm.motor_positions),
        arm_motor_ids=json.loads(orm.arm_motor_ids),
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
