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
    joint_values: Mapped[str] = mapped_column(
        Text, nullable=False
    )  # JSON list[float] rad
    joint_names: Mapped[str] = mapped_column(Text, nullable=False)  # JSON list[str]
    created_at: Mapped[datetime] = mapped_column(UtcDateTime, nullable=False)

    __table_args__ = (
        UniqueConstraint("robot_id", "name"),
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
    position: Mapped[int] = mapped_column(Integer, nullable=False)

    __table_args__ = (
        UniqueConstraint("group_id", "waypoint_id"),
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
