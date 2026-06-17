"""Storage Phase 2 — scan workflow CRUD round-trip.

Memory + Sqlite 두 adapter 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 같은 contract
자체 자리 자체 자리 — host_mock (memory) / host_dev (sqlite) 자체 자리 자체 자리 자체 자리
swap 호환.

scan_workflow.persistence_models + modules/storage/rdb/adapters/{memory,sqlite}.py
+ modules/storage/object_store/adapters/memory.py 의 contract 통과 자체 자리 자체 자리.
"""

from __future__ import annotations

import sqlite3
from pathlib import Path
from typing import Iterator

import pytest

from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)
from modules.storage.object_store.adapters.memory import MemoryObjectStore
from modules.storage.rdb.adapters.memory import MemoryRdbStore
from modules.storage.rdb.adapters.sqlite import SqliteStore
from modules.storage.rdb.store import RdbStore


# ─── fixture — Memory + Sqlite 양쪽 같은 contract 자체 자리 ──────


@pytest.fixture(params=["memory", "sqlite"])
def rdb(request, tmp_path: Path) -> Iterator[RdbStore]:
    """parametrize — 같은 test 자체 자리 자체 자리 자체 자리 두 adapter 자체 자리 자체 자리."""
    if request.param == "memory":
        yield MemoryRdbStore()
    else:
        store = SqliteStore(tmp_path / "test.db")
        try:
            yield store
        finally:
            store.close()


# ─── ScanSession ──────────────────────────────────────────────


def _make_session(robot_id: str = "test_robot", session_id: str = "s1") -> ScanSessionRecord:
    return ScanSessionRecord(
        robot_id=robot_id, session_id=session_id, created_at=100.0, label=None
    )


def test_scan_session_insert_get_round_trip(rdb: RdbStore):
    row_id = rdb.insert_scan_session(_make_session())
    assert row_id > 0

    fetched = rdb.get_scan_session(row_id)
    assert fetched is not None
    assert fetched.id == row_id
    assert fetched.robot_id == "test_robot"
    assert fetched.session_id == "s1"


def test_scan_session_find_by_id(rdb: RdbStore):
    rdb.insert_scan_session(_make_session(session_id="findme"))

    found = rdb.find_scan_session_by_id("test_robot", "findme")
    assert found is not None
    assert found.session_id == "findme"

    not_found = rdb.find_scan_session_by_id("test_robot", "nope")
    assert not_found is None


def test_scan_session_unique_constraint(rdb: RdbStore):
    """(robot_id, session_id) unique — idempotent NEW_SCAN_SESSION 자리 의존."""
    rdb.insert_scan_session(_make_session(session_id="dup"))
    # Memory 자체 자리 자체 자리 ValueError, Sqlite 자체 자리 자체 자리 IntegrityError 자체 자리 자체 자리.
    with pytest.raises((ValueError, sqlite3.IntegrityError)):
        rdb.insert_scan_session(_make_session(session_id="dup"))


def test_scan_session_list_sorted_desc(rdb: RdbStore):
    for i, ts in enumerate([100.0, 300.0, 200.0]):
        rdb.insert_scan_session(
            ScanSessionRecord(
                robot_id="r", session_id=f"s{i}", created_at=ts
            )
        )
    sessions = rdb.list_scan_sessions("r")
    assert [s.created_at for s in sessions] == [300.0, 200.0, 100.0]


# ─── Scan ─────────────────────────────────────────────────────


def _make_scan(session_row_id: int, scan_id: int) -> ScanRecord:
    return ScanRecord(
        session_row_id=session_row_id,
        robot_id="r",
        scan_id=scan_id,
        created_at=100.0,
        blob_key=f"scans/r/s/{scan_id:03d}.bin",
        num_frames=10,
        width=640,
        height=480,
        fx=600.0, fy=600.0, cx=320.0, cy=240.0,
        depth_scale=0.001,
        motor_positions=[2048, 2048, 2048, 2048, 2048],
        arm_motor_ids=[1, 2, 3, 4, 5],
    )


def test_scan_allocate_id_monotonic(rdb: RdbStore):
    sid = rdb.insert_scan_session(_make_session())

    # 빈 session — allocate 1
    assert rdb.allocate_scan_id(sid) == 1
    # 한 row 박은 후 allocate 2
    rdb.insert_scan(_make_scan(sid, scan_id=1))
    assert rdb.allocate_scan_id(sid) == 2
    # 두 번째 row + 삭제 후에도 monotonic
    rdb.insert_scan(_make_scan(sid, scan_id=2))
    assert rdb.allocate_scan_id(sid) == 3


def test_scan_insert_list_get_delete(rdb: RdbStore):
    sid = rdb.insert_scan_session(_make_session())
    row1 = rdb.insert_scan(_make_scan(sid, scan_id=1))
    row2 = rdb.insert_scan(_make_scan(sid, scan_id=2))

    scans = rdb.list_scans(sid)
    assert [s.scan_id for s in scans] == [1, 2]

    fetched = rdb.get_scan(row1)
    assert fetched is not None
    assert fetched.scan_id == 1
    # motor_positions JSON serde round-trip (Sqlite 자체 자리)
    assert fetched.motor_positions == [2048, 2048, 2048, 2048, 2048]

    rdb.delete_scan(row2)
    scans = rdb.list_scans(sid)
    assert [s.scan_id for s in scans] == [1]


def test_scan_unique_constraint_within_session(rdb: RdbStore):
    sid = rdb.insert_scan_session(_make_session())
    rdb.insert_scan(_make_scan(sid, scan_id=1))
    with pytest.raises((ValueError, sqlite3.IntegrityError)):
        rdb.insert_scan(_make_scan(sid, scan_id=1))


# ─── Reconstruction ───────────────────────────────────────────


def _make_recon(session_row_id: int) -> ReconstructionRecord:
    return ReconstructionRecord(
        session_row_id=session_row_id,
        robot_id="r",
        created_at=100.0,
        blob_key="reconstructions/r/s/recon_1.ply",
        voxel_size=0.002,
        sdf_trunc=0.010,
        depth_trunc=0.5,
        icp_max_dist=0.010,
        n_scans=3,
        n_edges=4,
        vertex_count=1000,
        triangle_count=2000,
        elapsed=5.5,
    )


def test_reconstruction_insert_list_get_delete(rdb: RdbStore):
    sid = rdb.insert_scan_session(_make_session())

    row = rdb.insert_reconstruction(_make_recon(sid))
    recons = rdb.list_reconstructions(sid)
    assert len(recons) == 1
    assert recons[0].id == row

    fetched = rdb.get_reconstruction(row)
    assert fetched is not None
    assert fetched.n_scans == 3
    assert fetched.vertex_count == 1000

    rdb.delete_reconstruction(row)
    assert rdb.list_reconstructions(sid) == []


# ─── CASCADE delete (Memory: 명시 + Sqlite: FK ON DELETE CASCADE) ─


def test_delete_scan_session_cascade(rdb: RdbStore):
    """delete_scan_session 자리 자식 scans / reconstructions 자체 자리 자체 자리 자동 삭제.

    Memory: 명시적 자체 자리 자체 자리 자체 자리 dict 정리.
    Sqlite: FK ON DELETE CASCADE + PRAGMA foreign_keys=ON.
    """
    sid = rdb.insert_scan_session(_make_session())
    rdb.insert_scan(_make_scan(sid, scan_id=1))
    rdb.insert_scan(_make_scan(sid, scan_id=2))
    rdb.insert_reconstruction(_make_recon(sid))

    rdb.delete_scan_session(sid)

    # 자식 다 사라짐
    assert rdb.get_scan_session(sid) is None
    assert rdb.list_scans(sid) == []
    assert rdb.list_reconstructions(sid) == []


# ─── ObjectStore round-trip ───────────────────────────────────


def test_object_store_put_get_round_trip():
    obj = MemoryObjectStore()
    data = b"\x00\x01\x02" + b"x" * 1000
    obj.put("scans/r/s/001.bin", data)

    got = obj.get("scans/r/s/001.bin")
    assert got == data


def test_object_store_delete():
    obj = MemoryObjectStore()
    obj.put("k", b"hello")
    obj.delete("k")
    with pytest.raises(KeyError):
        obj.get("k")


def test_object_store_list_prefix():
    obj = MemoryObjectStore()
    obj.put("scans/r/s1/001.bin", b"a")
    obj.put("scans/r/s1/002.bin", b"b")
    obj.put("scans/r/s2/001.bin", b"c")
    obj.put("reconstructions/r/s1/recon_1.ply", b"d")

    keys = sorted(obj.list("scans/r/s1/"))
    assert keys == ["scans/r/s1/001.bin", "scans/r/s1/002.bin"]
