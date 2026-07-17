from __future__ import annotations

from pathlib import Path

from sqlalchemy import String, select
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column

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


