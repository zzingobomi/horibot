from __future__ import annotations

from pathlib import Path

import pytest

from framework.storage.protocol import ObjectStore
from infra.object_store.filesystem import FilesystemObjectStore


def test_filesystem_satisfies_object_store_protocol(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    assert isinstance(store, ObjectStore)


def test_put_get_round_trip(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    payload = b"\x00\x01\x02\xff"
    store.put("hello.bin", payload)
    assert store.get("hello.bin") == payload


def test_put_overwrites_existing(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    store.put("k", b"first")
    store.put("k", b"second")
    assert store.get("k") == b"second"


def test_put_hierarchical_key_creates_subdirs(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    store.put("scans/omx_f_0/sess_001/000.bin", b"depth-data")
    target = tmp_path / "scans" / "omx_f_0" / "sess_001" / "000.bin"
    assert target.is_file()
    assert store.get("scans/omx_f_0/sess_001/000.bin") == b"depth-data"


def test_get_missing_raises_keyerror(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    with pytest.raises(KeyError):
        store.get("nope")


def test_delete_missing_raises_keyerror(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    with pytest.raises(KeyError):
        store.delete("nope")


def test_delete_removes_file(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    store.put("k", b"x")
    store.delete("k")
    with pytest.raises(KeyError):
        store.get("k")


def test_list_with_prefix(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    store.put("scans/a/1.bin", b"1")
    store.put("scans/a/2.bin", b"2")
    store.put("scans/b/3.bin", b"3")
    store.put("recons/x.ply", b"mesh")

    assert store.list("scans/a") == ["scans/a/1.bin", "scans/a/2.bin"]
    assert store.list("scans") == [
        "scans/a/1.bin",
        "scans/a/2.bin",
        "scans/b/3.bin",
    ]
    assert store.list("recons") == ["recons/x.ply"]


def test_list_empty_prefix_returns_all(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    store.put("a.bin", b"1")
    store.put("sub/b.bin", b"2")
    assert store.list("") == ["a.bin", "sub/b.bin"]


def test_list_nonexistent_prefix_returns_empty(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    assert store.list("nope/") == []


def test_put_escape_raises(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    with pytest.raises(ValueError):
        store.put("../escape.bin", b"x")


def test_get_escape_raises(tmp_path: Path):
    store = FilesystemObjectStore(tmp_path)
    with pytest.raises(ValueError):
        store.get("../escape.bin")


def test_base_dir_auto_created(tmp_path: Path):
    nested = tmp_path / "deep" / "nested" / "store"
    assert not nested.exists()
    FilesystemObjectStore(nested)
    assert nested.is_dir()
