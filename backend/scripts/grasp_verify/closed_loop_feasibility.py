"""closed-loop 착수 전 offline 검증 (docs/closed_loop_grasp_handoff.md §5).

정지 큐브의 실물 debug 세션(뷰별 raw depth+mask+intrinsic+pose)으로 두 가지를 잰다:

1) **cam-frame centroid 안정성** — 뷰(자세)마다 mask→depth→cam-frame centroid 를
   구하고, 그 뷰의 "카메라→물체 거리" 를 병기. centroid 가 physical 하게 말이 되는지
   (거리와 일치하는지), 튀는 뷰가 있는지. centroid 가 튀면 어떤 control law 도 못 선다.
2) **오차의 거리비례(=closed-loop 성립 전제)** — 각 뷰의 base-frame 물체 중심 추정이
   "가까운 뷰 합의"에서 얼마나 벗어나는지를 카메라 거리와 대조. 가까울수록 오차가
   줄어야 loop 를 물체 근처에서 닫는 설계가 성립한다.

실행: backend 에서 .venv\Scripts\python.exe scripts\grasp_verify\closed_loop_feasibility.py [session]
하드웨어 0 — debug/detect/<session>/ 재분석.
"""
import glob
import json
import os
import sys

import cv2
import numpy as np

S = sys.argv[1] if len(sys.argv) > 1 else "20260715_234827"
d = f"debug/detect/{S}"


def unproj(depth, scale, K, ys, xs):
    z = depth[ys, xs].astype(np.float64) * scale
    x = (xs - K["cx"]) / K["fx"] * z
    y = (ys - K["cy"]) / K["fy"] * z
    return np.stack([x, y, z], 1)


rows = []
for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
    m = json.load(open(jf, encoding="utf-8"))
    depth = cv2.imread(os.path.join(d, m["depth_png"]), cv2.IMREAD_UNCHANGED)
    K, scale = m["intrinsics"], m["depth_scale"]
    he, tcp = m["hand_eye_cam2ee"], m["tcp_ee2base"]
    R_be = np.array(tcp["R"])
    t_be = np.array(tcp["t"])
    R_ce = np.array(he["R"])
    t_ce = np.array(he["t"])
    mask = (
        cv2.imread(os.path.join(d, m["candidates"][0]["mask_png"]),
                   cv2.IMREAD_UNCHANGED) > 0
    )
    valid = depth > 0
    area = int(mask.sum())
    nvalid = int((mask & valid).sum())
    ys, xs = np.nonzero(mask & valid)
    if len(ys) < 30:
        continue
    cam = unproj(depth, scale, K, ys, xs)
    cam_cen = np.median(cam, 0)  # cam-frame centroid (m)
    base = (cam @ R_ce.T + t_ce) @ R_be.T + t_be
    base_cen = np.median(base, 0)
    cam_pos = R_be @ t_ce + t_be  # 카메라 위치 (base)
    tcp_pos = t_be
    dist = float(np.linalg.norm(base_cen - cam_pos))  # 카메라→물체 거리
    # TCP-relative 물체 벡터 — closed-loop 이 실제로 명령에 쓰는 양
    rel = base_cen - tcp_pos
    rows.append({
        "view": os.path.basename(jf)[:4],
        "cam_cen": cam_cen, "base_cen": base_cen, "rel": rel,
        "cam_dist_mm": dist * 1000.0,
        "valid_pct": 100.0 * nvalid / max(area, 1), "n": len(ys),
    })

if not rows:
    print(f"세션 {S} 에 cube 뷰 없음")
    sys.exit(1)

print(f"=== {S}: 뷰별 관측 (정지 큐브) ===")
print(f"{'뷰':>5} {'cam-frame centroid(mm)':>26} {'|c|':>6} {'camdist':>7} "
      f"{'base center(mm)':>24} {'valid%':>6} {'N':>5}")
for r in rows:
    c = r["cam_cen"] * 1000
    b = r["base_cen"] * 1000
    print(f"{r['view']:>5} ({c[0]:7.1f},{c[1]:7.1f},{c[2]:7.1f}) "
          f"{np.linalg.norm(c):6.1f} {r['cam_dist_mm']:7.1f} "
          f"({b[0]:6.1f},{b[1]:6.1f},{b[2]:6.1f}) "
          f"{r['valid_pct']:6.1f} {r['n']:5d}")

# cam-frame centroid 의 물리 정합: |centroid| 와 camdist 는 같은 양을 두 경로로
# 잰 것 (unproject median vs base 기하). 두 값의 차가 크면 centroid 산출이 병듦.
errs = [abs(np.linalg.norm(r["cam_cen"]) * 1000 - r["cam_dist_mm"]) for r in rows]
print(f"\n[1] cam-centroid |c| vs 기하 거리 차: max {max(errs):.1f}mm "
      f"(작으면 centroid 산출 자체는 건강)")

# 거리비례 검증: 가장 가까운 뷰 3개의 base center 중앙값을 '합의'로 놓고,
# 각 뷰의 편차를 카메라 거리와 대조.
srt = sorted(rows, key=lambda r: r["cam_dist_mm"])
consensus = np.median(np.stack([r["base_cen"] for r in srt[:3]]), 0)
print("\n[2] base center 편차 vs 카메라 거리 (합의 = 최근접 3뷰 median)")
print(f"{'뷰':>5} {'camdist(mm)':>11} {'|편차|(mm)':>10} {'편차 xyz(mm)':>24}")
pairs = []
for r in srt:
    dev = (r["base_cen"] - consensus) * 1000
    print(f"{r['view']:>5} {r['cam_dist_mm']:11.1f} {np.linalg.norm(dev):10.1f} "
          f"({dev[0]:6.1f},{dev[1]:6.1f},{dev[2]:6.1f})")
    pairs.append((r["cam_dist_mm"], float(np.linalg.norm(dev))))
# mask 오검출 outlier (편차가 물체 크기의 몇 배) 는 상관 계산에서 제외 — 그
# 자체가 "detection 을 tick 단위로 gate 해야 한다" 는 별도 증거로 보고.
p = np.array([(a, b) for a, b in pairs if b < 100.0])
n_out = len(pairs) - len(p)
if n_out:
    print(f"\n※ 편차 >100mm outlier {n_out}뷰 제외 (mask 오검출 — loop 의 tick "
          "gate 가 걸러야 할 케이스)")
if len(p) >= 3 and float(np.ptp(p[:, 0])) > 1:
    corr = np.corrcoef(p[:, 0], p[:, 1])[0, 1]
    print(f"거리↔편차 상관: r={corr:.2f} "
          "(양수·크면 '가까울수록 정확' = closed-loop 성립 전제 지지)")
print("\n※ ground truth 없음 — '합의' 는 가까운 뷰가 더 정확하다는 가설 하의 기준점."
      "\n   목적은 절대오차 측정이 아니라 거리-경향과 튀는 뷰 유무 확인.")
