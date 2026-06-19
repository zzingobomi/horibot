"""Storage Phase 2 — scan workflow CRUD round-trip.

SQLite `:memory:` (sim/mock 자리) + SQLite file (dev/pc 자리) 두 fixture —
같은 RdbStore + ScanWorkflowRepo 위 같은 contract 자체. host yaml 의 rdb_uri 만
다른 두 운용 모드 sw 두 자리 동일 코드 경로 검증.

`repos` fixture 는 한 transaction 전체로 — production storage_node 의 `with
rdb.session() as repos:` 와 같은 의미. 테스트 안 여러 operation 이 한 session
공유 → flush 후 즉시 자기 변경 read 가능.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterator

import pytest
from sqlalchemy.exc import IntegrityError

from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)
from modules.storage.object_store.adapters.memory import MemoryObjectStore
from modules.storage.rdb.base import Base, make_engine
from modules.storage.rdb.store import RdbStore, RepoBundle


# ─── fixture — sqlite-memory + sqlite-file 양쪽 같은 contract ──────


@pytest.fixture(params=["sqlite_memory", "sqlite_file"])
def repos(request, tmp_path: Path) -> Iterator[RepoBundle]:
    """parametrize — 같은 RdbStore 위 in-memory + file 두 자리. session 한 번."""
    if request.param == "sqlite_memory":
        engine = make_engine("sqlite:///:memory:")
    else:
        engine = make_engine(f"sqlite:///{tmp_path / 'test.db'}")
    Base.metadata.create_all(engine)
    store = RdbStore(engine)
    try:
        with store.session() as r:
            yield r
    finally:
        store.close()


# ─── ScanSession ──────────────────────────────────────────────


def _make_session(robot_id: str = "test_robot", session_id: str = "s1") -> ScanSessionRecord:
    return ScanSessionRecord(
        robot_id=robot_id, session_id=session_id, created_at=100.0, label=None
    )


def test_scan_session_insert_get_round_trip(repos: RepoBundle):
    row_id = repos.scan_workflow.insert_session(_make_session())
    assert row_id > 0

    fetched = repos.scan_workflow.get_session(row_id)
    assert fetched is not None
    assert fetched.id == row_id
    assert fetched.robot_id == "test_robot"
    assert fetched.session_id == "s1"


def test_scan_session_find_by_id(repos: RepoBundle):
    repos.scan_workflow.insert_session(_make_session(session_id="findme"))

    found = repos.scan_workflow.find_session_by_id("test_robot", "findme")
    assert found is not None
    assert found.session_id == "findme"

    not_found = repos.scan_workflow.find_session_by_id("test_robot", "nope")
    assert not_found is None


def test_scan_session_unique_constraint(repos: RepoBundle):
    """(robot_id, session_id) unique — idempotent NEW_SCAN_SESSION 의존."""
    repos.scan_workflow.insert_session(_make_session(session_id="dup"))
    # pre-check ValueError. race 시 IntegrityError 도 허용.
    with pytest.raises((ValueError, IntegrityError)):
        repos.scan_workflow.insert_session(_make_session(session_id="dup"))


def test_scan_session_list_sorted_desc(repos: RepoBundle):
    for i, ts in enumerate([100.0, 300.0, 200.0]):
        repos.scan_workflow.insert_session(
            ScanSessionRecord(
                robot_id="r", session_id=f"s{i}", created_at=ts
            )
        )
    sessions = repos.scan_workflow.list_sessions("r")
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


def test_scan_allocate_id_monotonic(repos: RepoBundle):
    sid = repos.scan_workflow.insert_session(_make_session())

    # 빈 session — allocate 1
    assert repos.scan_workflow.allocate_scan_id(sid) == 1
    # 한 row 박은 후 allocate 2
    repos.scan_workflow.insert_scan(_make_scan(sid, scan_id=1))
    assert repos.scan_workflow.allocate_scan_id(sid) == 2
    # 두 번째 row + 삭제 후에도 monotonic
    repos.scan_workflow.insert_scan(_make_scan(sid, scan_id=2))
    assert repos.scan_workflow.allocate_scan_id(sid) == 3


def test_scan_insert_list_get_delete(repos: RepoBundle):
    sid = repos.scan_workflow.insert_session(_make_session())
    row1 = repos.scan_workflow.insert_scan(_make_scan(sid, scan_id=1))
    row2 = repos.scan_workflow.insert_scan(_make_scan(sid, scan_id=2))

    scans = repos.scan_workflow.list_scans(sid)
    assert [s.scan_id for s in scans] == [1, 2]

    fetched = repos.scan_workflow.get_scan(row1)
    assert fetched is not None
    assert fetched.scan_id == 1
    # motor_positions JSON serde round-trip
    assert fetched.motor_positions == [2048, 2048, 2048, 2048, 2048]

    repos.scan_workflow.delete_scan(row2)
    scans = repos.scan_workflow.list_scans(sid)
    assert [s.scan_id for s in scans] == [1]


def test_scan_unique_constraint_within_session(repos: RepoBundle):
    sid = repos.scan_workflow.insert_session(_make_session())
    repos.scan_workflow.insert_scan(_make_scan(sid, scan_id=1))
    with pytest.raises((ValueError, IntegrityError)):
        repos.scan_workflow.insert_scan(_make_scan(sid, scan_id=1))


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


def test_reconstruction_insert_list_get_delete(repos: RepoBundle):
    sid = repos.scan_workflow.insert_session(_make_session())

    row = repos.scan_workflow.insert_reconstruction(_make_recon(sid))
    recons = repos.scan_workflow.list_reconstructions(sid)
    assert len(recons) == 1
    assert recons[0].id == row

    fetched = repos.scan_workflow.get_reconstruction(row)
    assert fetched is not None
    assert fetched.n_scans == 3
    assert fetched.vertex_count == 1000

    repos.scan_workflow.delete_reconstruction(row)
    assert repos.scan_workflow.list_reconstructions(sid) == []


# ─── CASCADE delete (Sqlite: FK ON DELETE CASCADE + PRAGMA foreign_keys=ON) ─


def test_delete_scan_session_cascade(repos: RepoBundle):
    """delete_session 시 자식 scans / reconstructions 자동 삭제."""
    sid = repos.scan_workflow.insert_session(_make_session())
    repos.scan_workflow.insert_scan(_make_scan(sid, scan_id=1))
    repos.scan_workflow.insert_scan(_make_scan(sid, scan_id=2))
    repos.scan_workflow.insert_reconstruction(_make_recon(sid))

    repos.scan_workflow.delete_session(sid)

    # 자식 다 사라짐
    assert repos.scan_workflow.get_session(sid) is None
    assert repos.scan_workflow.list_scans(sid) == []
    assert repos.scan_workflow.list_reconstructions(sid) == []


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
