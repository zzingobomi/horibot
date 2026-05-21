"""npz 파일 내용 빠른 확인 — calibration 결과/포즈 데이터 점검용.

사용법:
    python npz_view.py                                  # robot/calibration/*.npz 다
    python npz_view.py robot/calibration/hand_eye.npz   # 특정 파일
    python npz_view.py robot/calibration/*.npz -n 5     # 큰 배열은 첫 5행만
    python npz_view.py --full robot/calibration/handeye_poses.npz  # 전체 출력
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def _format_array(arr: np.ndarray, limit: int, *, full: bool) -> str:
    """배열을 적절히 잘라 문자열화. 첫 차원이 limit 초과 시 head만 + 잔여 개수 표시."""
    if full or arr.ndim == 0:
        return str(arr)
    if arr.ndim >= 1 and arr.shape[0] > limit:
        head = arr[:limit]
        rest = arr.shape[0] - limit
        return f"{head}\n... (+{rest} more, full shape={arr.shape})"
    return str(arr)


def view(path: Path, limit: int, *, full: bool) -> None:
    print(f"\n=== {path} ===")
    if not path.exists():
        print("  (파일 없음)")
        return
    try:
        data = np.load(str(path), allow_pickle=False)
    except Exception as e:
        print(f"  (로드 실패: {e})")
        return
    for k in data.files:
        v = data[k]
        print(f"\n[{k}]  shape={v.shape} dtype={v.dtype}")
        print(_format_array(v, limit, full=full))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="npz 파일 내용 빠른 확인 — calibration 결과/포즈 데이터 점검용",
    )
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        help="볼 .npz 파일들. 미지정 시 robot/calibration/*.npz 전체",
    )
    parser.add_argument(
        "-n", "--limit",
        type=int,
        default=10,
        help="큰 배열(첫 차원이 N 초과)의 헤드 N행만 표시 (default 10)",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="limit 무시, 전체 출력 (포즈 41개 다 보고 싶을 때)",
    )
    args = parser.parse_args()

    paths: list[Path]
    if args.paths:
        paths = args.paths
    else:
        default_dir = Path("robot/calibration")
        paths = sorted(default_dir.glob("*.npz"))
        if not paths:
            print(f"{default_dir}/ 에 *.npz 파일 없음")
            sys.exit(1)

    for path in paths:
        view(path, args.limit, full=args.full)


if __name__ == "__main__":
    main()
