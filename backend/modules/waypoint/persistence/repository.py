from __future__ import annotations

from collections.abc import Callable

from advanced_alchemy.repository import SQLAlchemySyncRepository
from sqlalchemy import delete, func, select
from sqlalchemy.orm import Session

from ..contract import WaypointGroupRecord, WaypointRecord
from .orm import (
    WaypointGroupMemberOrm,
    WaypointGroupOrm,
    WaypointOrm,
    group_to_orm,
    orm_to_group,
    orm_to_waypoint,
    waypoint_to_orm,
)


class _WaypointRepo(SQLAlchemySyncRepository[WaypointOrm]):
    model_type = WaypointOrm


class _GroupRepo(SQLAlchemySyncRepository[WaypointGroupOrm]):
    model_type = WaypointGroupOrm


class WaypointRepository:
    def __init__(self, session_factory: Callable[[], Session]):
        self._session_factory = session_factory

    # ── waypoints ─────────────────────────────────────────────
    def insert_waypoint(self, rec: WaypointRecord) -> WaypointRecord:
        with self._session_factory() as session:
            repo = _WaypointRepo(
                session=session, auto_commit=True, wrap_exceptions=False
            )
            orm = repo.add(waypoint_to_orm(rec))
            return orm_to_waypoint(orm)

    def get_waypoint(self, waypoint_row_id: int) -> WaypointRecord | None:
        with self._session_factory() as session:
            orm = _WaypointRepo(session=session, wrap_exceptions=False).get_one_or_none(
                WaypointOrm.id == waypoint_row_id
            )
            return orm_to_waypoint(orm) if orm is not None else None

    def get_waypoint_by_name(self, robot_id: str, name: str) -> WaypointRecord | None:
        with self._session_factory() as session:
            orm = _WaypointRepo(session=session, wrap_exceptions=False).get_one_or_none(
                WaypointOrm.robot_id == robot_id, WaypointOrm.name == name
            )
            return orm_to_waypoint(orm) if orm is not None else None

    def list_waypoints(self, robot_id: str) -> list[WaypointRecord]:
        with self._session_factory() as session:
            rows = _WaypointRepo(session=session, wrap_exceptions=False).get_many(
                WaypointOrm.robot_id == robot_id, order_by=WaypointOrm.name.asc()
            )
            return [orm_to_waypoint(o) for o in rows]

    def rename_waypoint(self, waypoint_row_id: int, name: str) -> None:
        with self._session_factory() as session:
            orm = session.get(WaypointOrm, waypoint_row_id)
            if orm is None:
                raise KeyError(f"waypoint {waypoint_row_id} 없음")
            orm.name = name
            session.commit()

    def delete_waypoint(self, waypoint_row_id: int) -> bool:
        with self._session_factory() as session:
            repo = _WaypointRepo(
                session=session, auto_commit=True, wrap_exceptions=False
            )
            if repo.get_one_or_none(WaypointOrm.id == waypoint_row_id) is None:
                return False
            repo.delete(waypoint_row_id)
            return True

    # ── groups ────────────────────────────────────────────────
    def insert_group(self, rec: WaypointGroupRecord) -> WaypointGroupRecord:
        with self._session_factory() as session:
            repo = _GroupRepo(session=session, auto_commit=True, wrap_exceptions=False)
            orm = repo.add(group_to_orm(rec))
            return orm_to_group(orm)

    def get_group_by_name(self, robot_id: str, name: str) -> WaypointGroupRecord | None:
        with self._session_factory() as session:
            orm = _GroupRepo(session=session, wrap_exceptions=False).get_one_or_none(
                WaypointGroupOrm.robot_id == robot_id, WaypointGroupOrm.name == name
            )
            return orm_to_group(orm) if orm is not None else None

    def list_groups(self, robot_id: str) -> list[WaypointGroupRecord]:
        with self._session_factory() as session:
            rows = _GroupRepo(session=session, wrap_exceptions=False).get_many(
                WaypointGroupOrm.robot_id == robot_id,
                order_by=WaypointGroupOrm.name.asc(),
            )
            return [orm_to_group(o) for o in rows]

    def delete_group(self, group_row_id: int) -> bool:
        with self._session_factory() as session:
            repo = _GroupRepo(session=session, auto_commit=True, wrap_exceptions=False)
            if repo.get_one_or_none(WaypointGroupOrm.id == group_row_id) is None:
                return False
            repo.delete(group_row_id)
            return True

    # ── group membership ──────────────────
    def add_member(self, group_id: int, waypoint_id: int) -> None:
        with self._session_factory() as session:
            exists = session.scalar(
                select(WaypointGroupMemberOrm).where(
                    WaypointGroupMemberOrm.group_id == group_id,
                    WaypointGroupMemberOrm.waypoint_id == waypoint_id,
                )
            )
            if exists is not None:
                return
            maxpos = session.scalar(
                select(func.max(WaypointGroupMemberOrm.position)).where(
                    WaypointGroupMemberOrm.group_id == group_id
                )
            )
            next_pos = 0 if maxpos is None else maxpos + 1
            session.add(
                WaypointGroupMemberOrm(
                    group_id=group_id, waypoint_id=waypoint_id, position=next_pos
                )
            )
            session.commit()

    def remove_member(self, group_id: int, waypoint_id: int) -> None:
        with self._session_factory() as session:
            session.execute(
                delete(WaypointGroupMemberOrm).where(
                    WaypointGroupMemberOrm.group_id == group_id,
                    WaypointGroupMemberOrm.waypoint_id == waypoint_id,
                )
            )
            session.commit()

    def reorder_group(self, group_id: int, ordered_waypoint_ids: list[int]) -> None:
        with self._session_factory() as session:
            members = session.scalars(
                select(WaypointGroupMemberOrm).where(
                    WaypointGroupMemberOrm.group_id == group_id
                )
            ).all()
            by_wp = {m.waypoint_id: m for m in members}
            for idx, wid in enumerate(ordered_waypoint_ids):
                m = by_wp.get(wid)
                if m is not None:
                    m.position = idx
            session.commit()

    def list_group_members(self, group_id: int) -> list[WaypointRecord]:
        with self._session_factory() as session:
            stmt = (
                select(WaypointOrm)
                .join(
                    WaypointGroupMemberOrm,
                    WaypointGroupMemberOrm.waypoint_id == WaypointOrm.id,
                )
                .where(WaypointGroupMemberOrm.group_id == group_id)
                .order_by(WaypointGroupMemberOrm.position.asc())
            )
            rows = session.scalars(stmt).all()
            return [orm_to_waypoint(o) for o in rows]
