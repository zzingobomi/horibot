"""calibration capture blob (.bin) 을 사람이 볼 수 있는 PNG/JPG/JSON 으로 추출.

사용 예:
    # 가장 최근 run 의 모든 capture 추출 → dumped/ 폴더에
    uv run python scripts/dump_calib_blob.py --robot so101_6dof_0

    # 특정 run + 특정 pose 만
    uv run python scripts/dump_calib_blob.py --robot so101_6dof_0 --run-id 3 --pose 5

    # 다른 출력 폴더
    uv run python scripts/dump_calib_blob.py ... --out d:/somewhere

출력 구조:
    <out>/<run_id>/<pose:03d>/
        color.jpg       # ChArUco 가 검출된 이미지 (browser viewable)
        depth.png       # 16-bit grayscale (어두워 보이지만 ImageJ/등으로 조절 가능)
        depth_vis.png   # 8-bit colorized depth (사람 확인용)
        meta.json       # intrinsic + timestamp + pose_index 정보
"""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np

BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))

from modules.camera import depth_frame as dframe  # noqa: E402


def dump_capture(
    blob_path: Path, out_dir: Path, pose_index: int, db_meta: dict
) -> bool:
    """blob 1개 추출. 정상 처리되면 True, 손상이면 False."""
    raw = blob_path.read_bytes()
    if len(raw) < 10_000:
        print(
            f"  pose #{pose_index}: blob {len(raw)} bytes — corruption (Pydantic "
            "Base64Bytes bug 시점 데이터). skip."
        )
        return False
    try:
        df = dframe.decode(raw)
    except Exception as e:
        print(f"  pose #{pose_index}: decode 실패 ({e}). skip.")
        return False

    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Color JPG (depth-align 된 color frame).
    cv2.imwrite(str(out_dir / "color.jpg"), df.color_bgr)

    # 2. Depth 16-bit PNG (원본). z16 그대로 — 1 unit = depth_scale m.
    cv2.imwrite(str(out_dir / "depth.png"), df.depth_z16)

    # 3. Depth 8-bit colorized — 시각적 확인.
    valid = df.depth_z16[df.depth_z16 > 0]
    if valid.size > 0:
        z_min = int(np.percentile(valid, 2))
        z_max = int(np.percentile(valid, 98))
        z_min = max(z_min, 1)
        z_max = max(z_max, z_min + 1)
        depth_norm = np.clip(
            (df.depth_z16.astype(np.float32) - z_min) / (z_max - z_min) * 255,
            0, 255,
        ).astype(np.uint8)
        depth_norm[df.depth_z16 == 0] = 0
        depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        cv2.imwrite(str(out_dir / "depth_vis.png"), depth_vis)

    # 4. Metadata JSON.
    meta = {
        "pose_index": pose_index,
        "timestamp": df.timestamp,
        "width": df.width,
        "height": df.height,
        "depth_scale": df.depth_scale,
        "intrinsic": {"fx": df.fx, "fy": df.fy, "cx": df.cx, "cy": df.cy},
        "depth_stats_mm": {
            "min": float(valid.min() * df.depth_scale * 1000) if valid.size else 0.0,
            "max": float(valid.max() * df.depth_scale * 1000) if valid.size else 0.0,
            "median": (
                float(np.median(valid) * df.depth_scale * 1000) if valid.size else 0.0
            ),
            "valid_pixels": int(valid.size),
            "total_pixels": int(df.depth_z16.size),
        },
        "db_meta": db_meta,
    }
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))

    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, help="robot_id (예: so101_6dof_0)")
    parser.add_argument(
        "--run-id", type=int, default=None,
        help="특정 run. 미지정 시 가장 최근 hand_eye run.",
    )
    parser.add_argument(
        "--pose", type=int, default=None,
        help="특정 pose_index 만 추출. 미지정 시 전부.",
    )
    parser.add_argument(
        "--db", type=Path,
        default=BACKEND / "storage" / "horibot.db",
    )
    parser.add_argument(
        "--blobs", type=Path,
        default=BACKEND / "storage" / "blobs",
    )
    parser.add_argument(
        "--out", type=Path, default=None,
        help="출력 폴더. 미지정 시 storage/blobs/.../dumped/",
    )
    args = parser.parse_args()
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except (AttributeError, Exception):
        pass

    # run_id 결정.
    con = sqlite3.connect(str(args.db))
    cur = con.cursor()
    if args.run_id is None:
        row = cur.execute(
            "SELECT id FROM calibration_runs "
            "WHERE robot_id=? AND kind='hand_eye' "
            "ORDER BY started_at DESC LIMIT 1",
            (args.robot,),
        ).fetchone()
        if row is None:
            print(f"hand_eye run 없음 (robot={args.robot})")
            con.close()
            return 1
        run_id = int(row[0])
    else:
        run_id = args.run_id

    # captures 메타 + primary blob_key (LEFT JOIN artifacts).
    rows = cur.execute(
        "SELECT c.pose_index, a.blob_key, c.reproj_rms_px, c.tilt_deg, "
        "c.motor_positions, c.board_in_cam "
        "FROM calibration_captures c "
        "LEFT JOIN calibration_capture_artifacts a "
        "  ON a.capture_id = c.id AND a.kind = 'primary' "
        "WHERE c.run_id=? ORDER BY c.pose_index ASC",
        (run_id,),
    ).fetchall()
    con.close()

    if not rows:
        print(f"run {run_id} 의 capture 없음")
        return 1

    out_root = args.out or (
        args.blobs / "calib_captures" / args.robot / str(run_id) / "dumped"
    )
    print(f"=== Dump run {run_id} ({len(rows)} captures) → {out_root} ===")

    ok_count = 0
    skip_count = 0
    for pi, blob_key, rms, tilt, mp_json, bic_json in rows:
        if args.pose is not None and pi != args.pose:
            continue
        if not blob_key:
            print(f"  pose #{pi}: blob_key 없음. skip.")
            skip_count += 1
            continue
        blob_path = args.blobs / blob_key
        if not blob_path.exists():
            print(f"  pose #{pi}: blob 파일 없음 ({blob_path}). skip.")
            skip_count += 1
            continue
        db_meta = {
            "reproj_rms_px": rms,
            "tilt_deg": tilt,
            "motor_positions": json.loads(mp_json) if mp_json else None,
            "board_in_cam": json.loads(bic_json) if bic_json else None,
        }
        out_dir = out_root / f"{pi:03d}"
        if dump_capture(blob_path, out_dir, pi, db_meta):
            ok_count += 1
            print(f"  pose #{pi}: → {out_dir}")
        else:
            skip_count += 1

    print()
    print(f"완료: {ok_count} 정상, {skip_count} skip (없음/손상)")
    if ok_count == 0:
        print("→ 추출된 파일 없음. 데이터 손상 가능성 (Pydantic Base64Bytes bug 자리).")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
