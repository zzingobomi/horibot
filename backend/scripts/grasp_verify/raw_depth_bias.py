"""raw depth 편향 계측 (§7.3-1) — base-frame PLY 가 아니라 센서 원본(_depth.png)에서
mask 픽셀의 카메라→표면 거리(mm)를 직접 재서 depth dropout/near-bias 를 정량 고발.

각 검출 .json + _depth.png + _mask_c*.png 로:
  - mask 내 valid depth 비율 (저텍스처 = dropout 많음)
  - mask 내 raw 거리 분포 (mm) + 표준편차 (저텍스처 = 노이즈/near-bias)
  - 같은 세션의 흰 큐브 vs (텍스처 있는) blue box 대조

하드웨어 0 — 저장된 debug 데이터만."""
import sys
import glob
import os
import json
import numpy as np
import cv2


def analyze(jpath):
    d = os.path.dirname(jpath)
    with open(jpath, encoding="utf-8") as f:
        meta = json.load(f)
    depth = cv2.imread(os.path.join(d, meta["depth_png"]), cv2.IMREAD_UNCHANGED)
    scale = meta["depth_scale"]  # raw*scale = meters
    name = os.path.basename(jpath).replace("white_small_round_cube", "cube")
    print(f"\n### {name}  prompt='{meta['prompt']}'")
    for c in meta["candidates"]:
        mp = c.get("mask_png")
        if not mp:
            continue
        mask = cv2.imread(os.path.join(d, mp), cv2.IMREAD_UNCHANGED)
        if mask is None:
            continue
        m = mask > 0
        area = int(m.sum())
        raw = depth[m]
        valid = raw > 0
        nvalid = int(valid.sum())
        frac = nvalid / max(area, 1)
        dist_mm = raw[valid].astype(np.float64) * scale * 1000.0  # 카메라→표면 mm
        if nvalid:
            p = np.percentile(dist_mm, [2, 50, 98])
            print(
                f"  c{c['index']} score={c['score']:.2f} "
                f"mask_area={area:5d}px valid={nvalid:5d} ({frac*100:4.1f}%) "
                f"dist(mm) p2={p[0]:.0f} p50={p[1]:.0f} p98={p[2]:.0f} "
                f"std={dist_mm.std():.1f} span={p[2]-p[0]:.0f} "
                f"base_z={c['base_z']*1000:.0f}mm h={c['height']*1000:.0f}mm pts={c['points']}"
            )
        else:
            print(f"  c{c['index']} score={c['score']:.2f} area={area} valid=0 (전멸)")


session = sys.argv[1] if len(sys.argv) > 1 else "20260715_211723"
d = f"debug/detect/{session}"
for jf in sorted(glob.glob(os.path.join(d, "*.json"))):
    analyze(jf)
