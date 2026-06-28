from __future__ import annotations

from pathlib import Path

import pytest
from sqlalchemy import String, select
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column

from framework.persistence.protocol import Repository
from infra.database.sqlite import open_sqlite


class Base(DeclarativeBase):
    pass


class Widget(Base):
    __tablename__ = "widgets"

    id: Mapped[int] = mapped_column(primary_key=True)
    name: Mapped[str] = mapped_column(String(64))


def test_sqlite_in_memory_insert_select():
    engine, session_factory = open_sqlite(":memory:")
    Base.metadata.create_all(engine)

    with session_factory() as session:
        session.add(Widget(id=1, name="alpha"))
        session.add(Widget(id=2, name="beta"))
        session.commit()

    with session_factory() as session:
        rows = session.scalars(select(Widget).order_by(Widget.id)).all()
        assert [(w.id, w.name) for w in rows] == [(1, "alpha"), (2, "beta")]


def test_sqlite_file_based_persists_across_sessions(tmp_path: Path):
    db_path = tmp_path / "test.db"
    engine, session_factory = open_sqlite(db_path)
    Base.metadata.create_all(engine)

    with session_factory() as session:
        session.add(Widget(id=42, name="persist"))
        session.commit()

    engine.dispose()

    engine2, factory2 = open_sqlite(db_path)
    with factory2() as session:
        widget = session.get(Widget, 42)
        assert widget is not None
        assert widget.name == "persist"
    engine2.dispose()


class FakeWidgetRepository:
    def __init__(self):
        self._store: dict[int, Widget] = {}

    def get(self, id: int) -> Widget | None:
        return self._store.get(id)

    def save(self, entity: Widget) -> None:
        self._store[entity.id] = entity

    def delete(self, id: int) -> None:
        del self._store[id]


def test_fake_repository_satisfies_protocol():
    repo: Repository[Widget] = FakeWidgetRepository()
    repo.save(Widget(id=1, name="x"))
    assert repo.get(1) is not None
    assert repo.get(999) is None
    repo.delete(1)
    assert repo.get(1) is None
    with pytest.raises(KeyError):
        repo.delete(1)


class SqlWidgetRepository:
    def __init__(self, session_factory):
        self._session_factory = session_factory

    def get(self, id: int) -> Widget | None:
        with self._session_factory() as session:
            return session.get(Widget, id)

    def save(self, entity: Widget) -> None:
        with self._session_factory() as session:
            session.merge(entity)
            session.commit()

    def delete(self, id: int) -> None:
        with self._session_factory() as session:
            obj = session.get(Widget, id)
            if obj is None:
                raise KeyError(id)
            session.delete(obj)
            session.commit()


def test_sql_repository_round_trip():
    engine, factory = open_sqlite(":memory:")
    Base.metadata.create_all(engine)

    repo: Repository[Widget] = SqlWidgetRepository(factory)
    repo.save(Widget(id=10, name="hello"))
    fetched = repo.get(10)
    assert fetched is not None
    assert fetched.name == "hello"

    repo.delete(10)
    assert repo.get(10) is None

    with pytest.raises(KeyError):
        repo.delete(10)
