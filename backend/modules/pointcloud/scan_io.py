"""TSDF scan 디렉토리/파일 IO.

scan은 *raw motor positions* + RGBD + 캘 메타로 저장. capture 시점 캘이 변해도
raw는 불변이므로 build 단계에서 *현재 캘*로 자유롭게 재계산 가능.

scan_id는 monotonic (재사용 X). session 디렉토리의 `_meta.json`에 next_id 보관.
삭제 후 새 캡처 시에도 인덱스 시프트 없음 — 캘리브 안정 ID와 같은 패턴.
"""

from __future__ import annotations

import json
import re
import time
from pathlib import Path
from typing import Sequence

import numpy as np

ROBOT_DIR = Path(__file__).parents[3] / "robot"
SCANS_DIR = ROBOT_DIR / "scans"
MODELS_DIR = ROBOT_DIR / "models"
CALIB_DIR = ROBOT_DIR / "calibration"

_SESSION_RE = re.compile(r"^[A-Za-z0-9_\-]+$")
_SCAN_FILE_RE = re.compile(r"^scan_(\d+)\.npz$")


def validate_session_id(sid: str) -> str:
    if not sid or not _SESSION_RE.match(sid):
        raise ValueError("session_id는 영문/숫자/_- 만 허용")
    return sid


def make_default_session_id() -> str:
    return time.strftime("session_%Y%m%d_%H%M%S")


def session_dir(sid: str) -> Path:
    return SCANS_DIR / validate_session_id(sid)


def list_session_ids() -> list[str]:
    if not SCANS_DIR.exists():
        return []
    return sorted(
        p.name for p in SCANS_DIR.iterdir() if p.is_dir() and _SESSION_RE.match(p.name)
    )


# ─── 안정 ID 카운터 (_meta.json) ──────────────────────────────────


def _meta_path(sdir: Path) -> Path:
    return sdir / "_meta.json"


def _load_meta(sdir: Path) -> dict:
    p = _meta_path(sdir)
    if not p.exists():
        return {"next_id": 1}
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        # 깨졌으면 디스크의 scan_*.npz 중 max(id)+1로 복구
        max_id = 0
        for f in sdir.glob("scan_*.npz"):
            m = _SCAN_FILE_RE.match(f.name)
            if m:
                max_id = max(max_id, int(m.group(1)))
        return {"next_id": max_id + 1}


def _save_meta(sdir: Path, meta: dict) -> None:
    _meta_path(sdir).write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def allocate_scan_id(sdir: Path) -> int:
    """monotonic id 발급. 호출마다 next_id++ 후 save. 삭제해도 안 줄임."""
    meta = _load_meta(sdir)
    scan_id = int(meta.get("next_id", 1))
    meta["next_id"] = scan_id + 1
    _save_meta(sdir, meta)
    return scan_id


def scan_path_for_id(sdir: Path, scan_id: int) -> Path:
    return sdir / f"scan_{scan_id:03d}.npz"


def parse_scan_id(path: Path) -> int | None:
    m = _SCAN_FILE_RE.match(path.name)
    return int(m.group(1)) if m else None


def list_scans(sdir: Path) -> list[Path]:
    if not sdir.exists():
        return []
    paths = [p for p in sdir.glob("scan_*.npz") if _SCAN_FILE_RE.match(p.name)]
    paths.sort(key=lambda p: parse_scan_id(p) or 0)
    return paths


# ─── 캘 메타 (mtime 묶음) ──────────────────────────────────────────


def calib_meta_dict() -> dict:
    """robot/calibration/*.npz의 mtime 묶음. build 시점에 캘 변경 감지용."""
    paths = {
        "joint_offsets_mtime": CALIB_DIR / "joint_offsets.npz",
        "link_offsets_mtime": CALIB_DIR / "link_offsets.npz",
        "sag_offsets_mtime": CALIB_DIR / "sag_offsets.npz",
        "hand_eye_mtime": CALIB_DIR / "hand_eye.npz",
        "intrinsic_mtime": CALIB_DIR / "intrinsic.npz",
    }
    return {k: (p.stat().st_mtime if p.exists() else 0.0) for k, p in paths.items()}


# ─── Save / Load ─────────────────────────────────────────────────


def save_scan(
    scan_path: Path,
    *,
    scan_id: int,
    color_bgr: np.ndarray,
    depth_z16: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: int,
    height: int,
    depth_scale: float,
    raw_motor_positions: Sequence[int],
    arm_motor_ids: Sequence[int],
    num_frames: int,
) -> None:
    np.savez_compressed(
        scan_path,
        scan_id=np.int32(scan_id),
        color_bgr=color_bgr,
        depth_z16=depth_z16,
        fx=np.float64(fx),
        fy=np.float64(fy),
        cx=np.float64(cx),
        cy=np.float64(cy),
        width=np.int32(width),
        height=np.int32(height),
        depth_scale=np.float64(depth_scale),
        raw_motor_positions=np.asarray(raw_motor_positions, dtype=np.int32),
        arm_motor_ids=np.asarray(arm_motor_ids, dtype=np.int32),
        calib_meta=json.dumps(calib_meta_dict()),
        timestamp=np.float64(time.time()),
        num_frames=np.int32(num_frames),
    )


def load_scan(scan_path: Path) -> dict:
    s = np.load(scan_path, allow_pickle=False)
    return {
        "scan_id": int(s["scan_id"]) if "scan_id" in s.files else 0,
        "color_bgr": s["color_bgr"],
        "depth_z16": s["depth_z16"],
        "fx": float(s["fx"]),
        "fy": float(s["fy"]),
        "cx": float(s["cx"]),
        "cy": float(s["cy"]),
        "width": int(s["width"]),
        "height": int(s["height"]),
        "depth_scale": float(s["depth_scale"]),
        "raw_motor_positions": s["raw_motor_positions"].tolist(),
        "arm_motor_ids": s["arm_motor_ids"].tolist(),
        "calib_meta": json.loads(str(s["calib_meta"])),
        "timestamp": float(s["timestamp"]),
        "num_frames": int(s["num_frames"]),
    }


def scan_meta(scan_path: Path) -> dict:
    """리스트 API용 경량 메타 (전체 numpy load 없이)."""
    s = np.load(scan_path, allow_pickle=False)
    return {
        "id": int(s["scan_id"]) if "scan_id" in s.files else 0,
        "path": scan_path.relative_to(ROBOT_DIR).as_posix(),
        "timestamp": float(s["timestamp"]),
        "num_frames": int(s["num_frames"]),
    }


def delete_scan(sdir: Path, scan_id: int) -> bool:
    p = scan_path_for_id(sdir, scan_id)
    if not p.exists():
        return False
    p.unlink()
    return True
