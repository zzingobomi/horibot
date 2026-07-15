"""close 뷰들이 왜 서로 다른지 근본 규명.
rigid transform 은 크기 불변 → 큐브 extent 가 뷰마다 다르면 그건 '카메라 frame 점군'
자체가 다른 것 = depth/mask 노이즈 (캘/FK 아님).
각 close 뷰: 카메라 frame 큐브 extent + raw depth 분포 + mask 픽셀수 + valid%.
하드웨어 0."""
import json, glob, os
import numpy as np
import cv2

S = "20260715_234827"
d = f"debug/detect/{S}"

print(f"=== {S}: 각 뷰 — 카메라 frame 큐브 (변환 전, 순수 depth+mask) ===")
print("(rigid 변환은 크기 불변 → cam-frame extent 가 뷰마다 다르면 = depth/mask 노이즈)\n")
print(f"{'뷰':>5} {'mask px':>7} {'valid%':>6} | {'cam extent X/Y/Z mm':>22} | "
      f"{'raw dist p2/50/98 mm':>22} {'std':>5}")
for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
    if "0003" in jf:
        continue
    m = json.load(open(jf, encoding="utf-8"))
    depth = cv2.imread(os.path.join(d, m["depth_png"]), cv2.IMREAD_UNCHANGED)
    K, scale = m["intrinsics"], m["depth_scale"]
    mask = cv2.imread(os.path.join(d, m["candidates"][0]["mask_png"]), cv2.IMREAD_UNCHANGED) > 0
    valid = depth > 0
    area = int(mask.sum()); nvalid = int((mask & valid).sum())
    ys, xs = np.nonzero(mask & valid)
    z = depth[ys, xs].astype(np.float64) * scale
    x = (xs - K["cx"]) / K["fx"] * z
    y = (ys - K["cy"]) / K["fy"] * z
    cam = np.stack([x, y, z], 1)
    ext = (cam.max(0) - cam.min(0)) * 1000
    dist = z * 1000
    p = np.percentile(dist, [2, 50, 98])
    name = os.path.basename(jf)[:4]
    print(f"{name:>5} {area:7d} {100*nvalid/max(area,1):6.1f} | "
          f"{ext[0]:6.1f}/{ext[1]:5.1f}/{ext[2]:5.1f} | "
          f"{p[0]:6.0f}/{p[1]:5.0f}/{p[2]:5.0f} {dist.std():5.1f}")
print("\n해석:")
print(" - cam extent Z(깊이방향)가 25mm 훨씬 넘으면 = 그 뷰 depth 에 뜬 점/배경 섞임(노이즈)")
print(" - raw dist std 크거나 p98-p2 큰 뷰 = depth 노이즈 심한 뷰")
print(" - mask px 가 뷰마다 크게 다르면 = SAM mask 가 뷰마다 다른 영역 잡음")
