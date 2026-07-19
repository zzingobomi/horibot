"""GDINO 단독 vs 합동 프롬프트 score 분포 비교 — 합동 추론 기본값 결정의 데이터.

배경 (2026-07-19 멀티 프롬프트 일반화): detector 는 prompt 별 단독 추론 N-loop
이 기본이고, GDINO 1-forward 합동 추론(detector_joint_inference)은 opt-in.
합동 쿼리("white cube. box.")는 텍스트 토큰 정렬 상호작용으로 같은 물체에 다른
score 를 줄 수 있는데, task 의 신뢰 컷(_PICK_SCORE_MIN=0.45)은 실측 마진 0.05
(진짜 큐브 min 0.49 / 오검출 max 0.44) 위에 서 있다 — 분포가 밀리면 컷이 진짜
물체를 죽이거나 오검출을 통과시킨다. **켜기 전에 이 스크립트로 실물 덤프
(debug/detect/*/\\*_color.png) 에서 두 모드를 나란히 돌려 실측 판단한다.**

실행 (backend/, 로봇/Zenoh 불필요 — 모델+이미지 파일만):
    .venv\\Scripts\\python.exe scripts/compare_joint_prompt_scores.py \\
        --prompts "white small round cube" "blue box" --limit 24

출력: 이미지별 표 + 요약 (Δ=joint−single, 컷 교차 카운트) + JSON 저장
(debug/joint_prompt_compare/<ts>/results.json — 재분석 소스).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # backend/ 루트

from modules.detector.drivers.gdino import GroundingDino  # noqa: E402

_DETECT_ROOT = Path("debug/detect")
_OUT_ROOT = Path("debug/joint_prompt_compare")
# task 신뢰 컷 (steps._PICK_SCORE_MIN) — 여기서는 판정 기준 상수로만 재기술
# (import 하면 tasks 패키지가 끌려온다 — 이 스크립트는 detector 계층만).
_SCORE_CUT = 0.45


def _iou(a: tuple[float, float, float, float],
         b: tuple[float, float, float, float]) -> float:
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / area


def _pick_images(limit: int, min_session_files: int) -> list[Path]:
    """최근 세션 우선으로 color 원본 표집 — 세션당 고르게 (한 장면 편중 방지).

    min_session_files: 이 미만 세션 제외 — 단일 파일 세션은 pytest 가 남긴
    **합성(흑색) 이미지**가 다수 (2026-07-19 1차 런 오염 실측: 검은 화면에
    single 'blue box' 0.4118 환각 14건이 통계를 뒤집었다). 실 스윕 세션은
    pose 여러 장이라 파일 수로 구분된다."""
    sessions = sorted(
        (d for d in _DETECT_ROOT.iterdir() if d.is_dir()), reverse=True
    )
    picked: list[Path] = []
    round_idx = 0
    per_session = [
        imgs
        for s in sessions
        if len(imgs := sorted(s.glob("*_color.png"), reverse=True))
        >= min_session_files
    ]
    while len(picked) < limit and any(per_session):
        for imgs in per_session:
            if round_idx < len(imgs):
                picked.append(imgs[round_idx])
                if len(picked) >= limit:
                    break
        round_idx += 1
        if round_idx > 200:  # 안전 상한
            break
    return picked


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--prompts", nargs="+", required=True)
    ap.add_argument("--limit", type=int, default=24)
    ap.add_argument("--top-k", type=int, default=5)
    ap.add_argument("--min-session-files", type=int, default=3)
    args = ap.parse_args()
    prompts: list[str] = args.prompts

    images = _pick_images(args.limit, args.min_session_files)
    if not images:
        raise SystemExit(f"{_DETECT_ROOT} 에 *_color.png 없음")
    print(f"이미지 {len(images)}장, prompts={prompts}")

    dino = GroundingDino()
    dino.preload()

    rows: list[dict] = []
    t_single_ms: list[float] = []
    t_joint_ms: list[float] = []
    for path in images:
        img = cv2.imread(str(path))
        if img is None:
            continue
        row: dict = {"image": str(path)}
        t0 = time.perf_counter()
        singles = {p: dino.detect_boxes(img, p, args.top_k) for p in prompts}
        t_single_ms.append((time.perf_counter() - t0) * 1000)
        t0 = time.perf_counter()
        joint = dino.detect_boxes_joint(img, prompts, args.top_k)
        t_joint_ms.append((time.perf_counter() - t0) * 1000)
        for p in prompts:
            s_top = singles[p][0] if singles[p] else None
            j_cands = [t for t in joint if t[2] == p]
            j_top = j_cands[0] if j_cands else None
            row[p] = {
                "single": round(s_top[1], 4) if s_top else None,
                "joint": round(j_top[1], 4) if j_top else None,
                "delta": (
                    round(j_top[1] - s_top[1], 4)
                    if (s_top and j_top) else None
                ),
                # 두 모드 top-1 이 같은 물체인가 (귀속/일관성 sanity)
                "iou": (
                    round(_iou(s_top[0], j_top[0]), 3)
                    if (s_top and j_top) else None
                ),
            }
        rows.append(row)

    # ─── 요약 ───
    out_dir = _OUT_ROOT / time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    summary: dict = {"prompts": prompts, "n_images": len(rows), "per_prompt": {}}
    print(f"\n{'=' * 72}")
    for p in prompts:
        pairs = [r[p] for r in rows if r[p]["delta"] is not None]
        deltas = np.array([x["delta"] for x in pairs], dtype=float)
        singles_s = np.array(
            [r[p]["single"] for r in rows if r[p]["single"] is not None]
        )
        joints_s = np.array(
            [r[p]["joint"] for r in rows if r[p]["joint"] is not None]
        )
        only_single = sum(
            1 for r in rows
            if r[p]["single"] is not None and r[p]["joint"] is None
        )
        only_joint = sum(
            1 for r in rows
            if r[p]["joint"] is not None and r[p]["single"] is None
        )
        # 컷 교차 = 단독은 컷 위인데 합동이 컷 아래로 (진짜 물체 사망 클래스)
        cut_break = sum(
            1 for x in pairs
            if x["single"] >= _SCORE_CUT and x["joint"] < _SCORE_CUT
        )
        low_iou = sum(
            1 for x in pairs if x["iou"] is not None and x["iou"] < 0.5
        )
        stats = {
            "n_pairs": len(pairs),
            "delta_mean": round(float(deltas.mean()), 4) if len(deltas) else None,
            "delta_min": round(float(deltas.min()), 4) if len(deltas) else None,
            "delta_max": round(float(deltas.max()), 4) if len(deltas) else None,
            "single_min": round(float(singles_s.min()), 4) if len(singles_s) else None,
            "joint_min": round(float(joints_s.min()), 4) if len(joints_s) else None,
            "detected_only_single": only_single,
            "detected_only_joint": only_joint,
            f"cut_break(<{_SCORE_CUT})": cut_break,
            "top1_iou<0.5": low_iou,
        }
        summary["per_prompt"][p] = stats
        print(f"[{p}]")
        for k, v in stats.items():
            print(f"  {k}: {v}")
    summary["latency_ms"] = {
        "single_sum_mean": round(float(np.mean(t_single_ms)), 1),
        "joint_mean": round(float(np.mean(t_joint_ms)), 1),
    }
    print(f"latency: 단독 {len(prompts)}회 합계 평균 "
          f"{summary['latency_ms']['single_sum_mean']}ms vs 합동 1회 평균 "
          f"{summary['latency_ms']['joint_mean']}ms")
    (out_dir / "results.json").write_text(
        json.dumps({"summary": summary, "rows": rows}, ensure_ascii=False,
                   indent=2),
        encoding="utf-8",
    )
    print(f"\n저장: {out_dir / 'results.json'}")


if __name__ == "__main__":
    main()
