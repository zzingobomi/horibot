"""Waypoint ORM — 3 entity (waypoints / waypoint_groups / waypoint_group_members).

Robot Asset Layer 영속성. joint 는 rad JSON 저장 (Motion.TcpState 계약 단위).
Database-per-Module: 소유=이 모듈, 공유 Base 등록, 마이그레이션=루트 alembic.
WaypointGroupMember 는 order 있는 join (docs/backend_v2.md §17.2 D5) — reorder/
add/remove 가 행 단위 + position 컬럼이 드래그 UI 와 1:1.
"""

from __future__ import annotations

import json
from datetime import datetime

from sqlalchemy import (
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

from ..contract import WaypointGroupRecord, WaypointRecord


class WaypointOrm(Base):
    __tablename__ = "waypoints"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    name: Mapped[str] = mapped_column(String, nullable=False)
    joint_values: Mapped[str] = mapped_column(Text, nullable=False)  # JSON list[float] rad
    joint_names: Mapped[str] = mapped_column(Text, nullable=False)  # JSON list[str]
    created_at: Mapped[datetime] = mapped_column(UtcDateTime, nullable=False)

    __table_args__ = (
        UniqueConstraint("robot_id", "name"),  # robot 당 이름 유일
        Index("idx_waypoints_robot", "robot_id", "name"),
    )


class WaypointGroupOrm(Base):
    __tablename__ = "waypoint_groups"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    name: Mapped[str] = mapped_column(String, nullable=False)

    __table_args__ = (
        UniqueConstraint("robot_id", "name"),
        Index("idx_waypoint_groups_robot", "robot_id", "name"),
    )


class WaypointGroupMemberOrm(Base):
    __tablename__ = "waypoint_group_members"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    group_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("waypoint_groups.id", ondelete="CASCADE"),
        nullable=False,
    )
    waypoint_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("waypoints.id", ondelete="CASCADE"),
        nullable=False,
    )
    # 'order' 는 SQL 예약어 → 'position'. group 내 표시/실행 순서 (0-based).
    position: Mapped[int] = mapped_column(Integer, nullable=False)

    __table_args__ = (
        UniqueConstraint("group_id", "waypoint_id"),  # 한 group 에 한 번만
        Index("idx_wgm_group", "group_id", "position"),
    )


# ─── record ↔ ORM mapper ────────────────────────────────────────────


def waypoint_to_orm(rec: WaypointRecord) -> WaypointOrm:
    return WaypointOrm(
        robot_id=rec.robot_id,
        name=rec.name,
        joint_values=json.dumps(rec.joint_values),
        joint_names=json.dumps(rec.joint_names),
        created_at=rec.created_at,
    )


def orm_to_waypoint(orm: WaypointOrm) -> WaypointRecord:
    return WaypointRecord(
        id=orm.id,
        robot_id=orm.robot_id,
        name=orm.name,
        joint_values=[float(x) for x in json.loads(orm.joint_values)],
        joint_names=[str(x) for x in json.loads(orm.joint_names)],
        created_at=orm.created_at,
    )


def group_to_orm(rec: WaypointGroupRecord) -> WaypointGroupOrm:
    return WaypointGroupOrm(robot_id=rec.robot_id, name=rec.name)


def orm_to_group(orm: WaypointGroupOrm) -> WaypointGroupRecord:
    return WaypointGroupRecord(id=orm.id, robot_id=orm.robot_id, name=orm.name)
