"""캘리브레이션 산출물의 timestamp snapshot 저장/조회/복원.

`_srv_handeye_commit` 진입 시 현재 live disk 상태를 통째로 `.history/<timestamp>_<tag>/`
로 복사 — 이후 사용자가 σ 후퇴 시 picker 에서 옛 snapshot 으로 되돌릴 수 있게.

snapshot 단위: 4종 absolute npz + intrinsic + hand_eye + handeye_poses 묶음. kind 별
독립 timestamp 면 picker 에서 matching 지옥 — calibration 1 round = 1 snapshot.

Retention 없음. 한 snapshot ~tens of KB (4종 + intrinsic 다 합쳐도 ~10KB 수준).
picker 길이는 frontend default view (최근 30일 등) 으로 제어.

분산 동기: `.history/` 는 git 추적 X — runtime artifact 로 취급. PC 머신 한 곳에서만
누적. 분산 머신은 live npz 만 git pull 로 동기 (기존 동일).

API: caller (calibration_node) 가 calibration_dir 를 명시적으로 넘김. RobotRegistry
singleton 의존 X → unit test 에서 tmp_path 그대로 사용 가능.
"""

from __future__ import annotations

import json
import logging
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)


SNAPSHOT_FILES = (
    "joint_offsets.npz",
    "link_offsets.npz",
    "sag_offsets.npz",
    "hand_eye.npz",
    "tool_offset.npz",
    "intrinsic.npz",
    "handeye_poses.npz",
)

META_FILE = "meta.json"
HISTORY_DIRNAME = ".history"
TIMESTAMP_FORMAT = "%Y%m%dT%H%M%S"


def _history_dir(calibration_dir: Path) -> Path:
    return calibration_dir / HISTORY_DIRNAME


def _slugify_tag(tag: str) -> str:
    """디렉토리 이름용 sanitize. 알파넘 + _ - . 만 유지, 길이 60."""
    if not tag:
        return "snapshot"
    safe = "".join(
        ch if (ch.isalnum() or ch in ("_", "-", ".")) else "_" for ch in tag
    )
    return safe[:60] or "snapshot"


def _alloc_snap_dir(
    calibration_dir: Path, slug: str
) -> tuple[Path, str]:
    """timestamp 충돌 회피 (같은 초 안 두 snapshot)."""
    ts = time.strftime(TIMESTAMP_FORMAT)
    hdir = _history_dir(calibration_dir)
    base = hdir / f"{ts}_{slug}"
    if not base.exists():
        return base, ts
    for n in range(1, 100):
        cand = hdir / f"{ts}_{slug}_{n:02d}"
        if not cand.exists():
            return cand, ts
    raise RuntimeError(f"snapshot 디렉토리 할당 실패: {ts}")


@dataclass(frozen=True)
class SnapshotInfo:
    timestamp: str
    tag: str
    path: Path
    meta: dict


def snapshot(
    calibration_dir: Path, tag: str, meta: dict
) -> SnapshotInfo:
    """현재 live disk → `.history/<ts>_<tag>/`. meta 는 picker 표시용 (σ 등)."""
    slug = _slugify_tag(tag)
    snap_dir, ts = _alloc_snap_dir(calibration_dir, slug)
    snap_dir.mkdir(parents=True, exist_ok=True)

    copied: list[str] = []
    for fname in SNAPSHOT_FILES:
        src = calibration_dir / fname
        if src.exists():
            shutil.copy2(src, snap_dir / fname)
            copied.append(fname)

    meta_payload = dict(meta)
    meta_payload["timestamp"] = ts
    meta_payload["tag"] = tag
    meta_payload["files"] = copied
    (snap_dir / META_FILE).write_text(
        json.dumps(meta_payload, indent=2, default=str), encoding="utf-8"
    )

    logger.info(
        "calibration snapshot 저장: %s (files=%d)", snap_dir, len(copied)
    )
    return SnapshotInfo(timestamp=ts, tag=tag, path=snap_dir, meta=meta_payload)


def list_snapshots(calibration_dir: Path) -> list[SnapshotInfo]:
    """newest first. meta 손상 / 비-디렉토리 항목은 skip."""
    hdir = _history_dir(calibration_dir)
    if not hdir.exists():
        return []

    infos: list[SnapshotInfo] = []
    for snap_dir in sorted(hdir.iterdir(), reverse=True):
        if not snap_dir.is_dir():
            continue
        meta_path = snap_dir / META_FILE
        if not meta_path.exists():
            continue
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        except Exception as e:
            logger.warning("snapshot meta 로드 실패 (%s): %s", snap_dir, e)
            continue
        infos.append(
            SnapshotInfo(
                timestamp=str(
                    meta.get("timestamp", snap_dir.name.split("_", 1)[0])
                ),
                tag=str(meta.get("tag", "")),
                path=snap_dir,
                meta=meta,
            )
        )
    return infos


def restore(calibration_dir: Path, timestamp: str) -> SnapshotInfo:
    """주어진 timestamp snapshot 을 live 위치로 복원.

    복원 전 현재 live 도 자동 snapshot ("pre-restore") — restore 도 undo 가능.

    snapshot 시점에 없던 파일은 live 에서도 제거 (정확 복원). caller (calibration_node)
    가 *Coordinates singletons 의 reload() 와 calibration_node._states 의 hand_eye/
    intrinsic load 를 후속 호출해 메모리 동기. LinkCoordinates 는 URDF patch 라
    재시작 필요 (caller 가 restart_required 표시).
    """
    infos = list_snapshots(calibration_dir)
    match = next((i for i in infos if i.timestamp == timestamp), None)
    if match is None:
        raise FileNotFoundError(f"snapshot 없음: timestamp={timestamp}")

    snapshot(
        calibration_dir, tag="pre-restore", meta={"restore_target": timestamp}
    )

    src_dir = match.path
    files_in_src = {p.name for p in src_dir.iterdir() if p.is_file()}
    for fname in SNAPSHOT_FILES:
        live = calibration_dir / fname
        if fname in files_in_src:
            shutil.copy2(src_dir / fname, live)
        elif live.exists():
            live.unlink()

    logger.info(
        "calibration snapshot 복원: %s → live (%d files)",
        match.path,
        sum(1 for f in SNAPSHOT_FILES if f in files_in_src),
    )
    return match
