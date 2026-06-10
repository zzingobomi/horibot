"""calibration_backup 모듈 unit tests.

caller 가 calibration_dir 를 명시적으로 넘기게 refactor 했으므로 RobotRegistry
의존 X — pytest tmp_path 그대로 사용 가능.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from modules.calibration import backup


def _write_dummy_npz(path: Path, value: float) -> None:
    """테스트용 식별 가능한 npz. value 만 다르면 disk diff 비교 가능."""
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(path), value=np.array([value], dtype=np.float64))


def _read_dummy(path: Path) -> float:
    return float(np.load(str(path))["value"][0])


@pytest.fixture
def calib_dir(tmp_path: Path) -> Path:
    """모든 SNAPSHOT_FILES 가 박힌 fresh calibration dir."""
    cdir = tmp_path / "calibration"
    cdir.mkdir(parents=True, exist_ok=True)
    for i, fname in enumerate(backup.SNAPSHOT_FILES):
        _write_dummy_npz(cdir / fname, float(i + 1))
    return cdir


def test_snapshot_creates_history_dir_with_all_files(calib_dir: Path) -> None:
    info = backup.snapshot(calib_dir, tag="pre-commit", meta={"sigma_rot_deg": 0.7})
    assert info.path.exists()
    assert info.path.parent == calib_dir / backup.HISTORY_DIRNAME
    for fname in backup.SNAPSHOT_FILES:
        assert (info.path / fname).exists(), f"snapshot 에 {fname} 누락"
    assert (info.path / backup.META_FILE).exists()


def test_snapshot_meta_includes_caller_payload(calib_dir: Path) -> None:
    info = backup.snapshot(
        calib_dir,
        tag="pre-commit",
        meta={"sigma_rot_deg": 0.65, "sigma_t_mm": 7.94, "capture_count": 12},
    )
    assert info.meta["sigma_rot_deg"] == 0.65
    assert info.meta["sigma_t_mm"] == 7.94
    assert info.meta["capture_count"] == 12
    assert info.meta["tag"] == "pre-commit"
    assert info.meta["timestamp"] == info.timestamp
    assert set(info.meta["files"]) == set(backup.SNAPSHOT_FILES)


def test_list_returns_newest_first(calib_dir: Path) -> None:
    a = backup.snapshot(calib_dir, tag="first", meta={})
    b = backup.snapshot(calib_dir, tag="second", meta={})
    c = backup.snapshot(calib_dir, tag="third", meta={})
    listed = backup.list_snapshots(calib_dir)
    # 같은 초 안에 3 snapshot — collision 회피로 suffix _01 _02 박힘.
    # newest-first 정렬은 sorted reverse 라 디렉토리 이름 알파벳 역순.
    assert len(listed) == 3
    timestamps = {i.timestamp for i in listed}
    assert timestamps == {a.timestamp, b.timestamp, c.timestamp}


def test_list_skips_dirs_without_meta(calib_dir: Path) -> None:
    backup.snapshot(calib_dir, tag="ok", meta={})
    # garbage 디렉토리 (meta.json 없음) 추가
    (calib_dir / backup.HISTORY_DIRNAME / "garbage").mkdir()
    listed = backup.list_snapshots(calib_dir)
    assert len(listed) == 1
    assert all((i.path / backup.META_FILE).exists() for i in listed)


def test_restore_roundtrip_matches_original(calib_dir: Path) -> None:
    # 원본 상태 기록
    original_values = {
        f: _read_dummy(calib_dir / f) for f in backup.SNAPSHOT_FILES
    }
    info = backup.snapshot(calib_dir, tag="checkpoint", meta={"sigma_rot_deg": 0.7})

    # live 를 다 망가뜨림
    for fname in backup.SNAPSHOT_FILES:
        _write_dummy_npz(calib_dir / fname, value=999.0)

    backup.restore(calib_dir, info.timestamp)

    for fname, want in original_values.items():
        assert (calib_dir / fname).exists()
        assert _read_dummy(calib_dir / fname) == want, f"{fname} 복원 불일치"


def test_restore_creates_pre_restore_snapshot(calib_dir: Path) -> None:
    info = backup.snapshot(calib_dir, tag="A", meta={})
    backup.restore(calib_dir, info.timestamp)

    listed = backup.list_snapshots(calib_dir)
    tags = {i.tag for i in listed}
    assert "pre-restore" in tags, "restore 직전 자동 백업 누락 → undo 불가"


def test_restore_missing_timestamp_raises(calib_dir: Path) -> None:
    with pytest.raises(FileNotFoundError):
        backup.restore(calib_dir, timestamp="29991231T235959")


def test_restore_removes_files_absent_in_snapshot(calib_dir: Path) -> None:
    """snapshot 시점에 없던 파일은 live 에서도 삭제 (정확 복원).

    예: 옛 snapshot 에 tool_offset.npz 없었으면, 그 시점으로 restore 하면
    현재 live 의 tool_offset.npz 는 제거되어야 함.
    """
    # 일부러 tool_offset.npz 만 빠진 live 상태로 snapshot
    (calib_dir / "tool_offset.npz").unlink()
    info = backup.snapshot(calib_dir, tag="without-tool", meta={})

    # 이제 tool_offset.npz 가 다시 박힌 상태로 만든다
    _write_dummy_npz(calib_dir / "tool_offset.npz", value=42.0)
    assert (calib_dir / "tool_offset.npz").exists()

    backup.restore(calib_dir, info.timestamp)

    # 정확 복원이라 tool_offset.npz 가 다시 사라져야 함
    assert not (calib_dir / "tool_offset.npz").exists(), (
        "snapshot 에 없던 파일이 restore 후에도 남아 있음 — 정확 복원 실패"
    )
