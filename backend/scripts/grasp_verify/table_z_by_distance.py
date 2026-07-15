"""테이블 z 오프셋이 edge-bleed 인지 전역 캘 오차인지 판별.
큐브 mask 에서 거리(픽셀 링)별로 테이블 base-z 를 재서:
- 가까울수록 뜨고 멀수록 0 → depth edge-bleed(뜬 픽셀) 아티팩트
- 거리 무관 일정하게 +15~20mm → 전역 캘/FK 오차 (진짜 근본)
하드웨어 0."""
import sys, os, json, glob
import numpy as np
import cv2


def unproject(depth, scale, K, ys, xs):
    z = depth[ys, xs].astype(np.float64) * scale
    x = (xs - K["cx"]) / K["fx"] * z
    y = (ys - K["cy"]) / K["fy"] * z
    return np.stack([x, y, z], axis=-1)


def to_base(cam, he, tcp):
    ee = cam @ np.array(he["R"]).T + np.array(he["t"])
    return ee @ np.array(tcp["R"]).T + np.array(tcp["t"])


session = sys.argv[1] if len(sys.argv) > 1 else "20260715_211723"
d = f"debug/detect/{session}"
# 픽셀 거리 버킷 (마스크에서 dilate 반경)
rings = [(3, 15), (15, 30), (30, 60), (60, 100), (100, 160), (160, 240)]
for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
    if "0003" in jf:
        continue
    with open(jf, encoding="utf-8") as f:
        meta = json.load(f)
    depth = cv2.imread(os.path.join(d, meta["depth_png"]), cv2.IMREAD_UNCHANGED)
    scale, K = meta["depth_scale"], meta["intrinsics"]
    he, tcp = meta["hand_eye_cam2ee"], meta["tcp_ee2base"]
    c0 = meta["candidates"][0]
    mask = cv2.imread(os.path.join(d, c0["mask_png"]), cv2.IMREAD_UNCHANGED) > 0
    valid = depth > 0
    print(f"\n### {os.path.basename(jf).replace('_det_white_small_round_cube','')} "
          f"(큐브 base_z={c0['base_z']*1000:.0f}mm)  테이블 z@거리버킷:")
    mu8 = mask.astype(np.uint8)
    for lo, hi in rings:
        klo = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * lo + 1, 2 * lo + 1))
        khi = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * hi + 1, 2 * hi + 1))
        band = (cv2.dilate(mu8, khi) > 0) & ~(cv2.dilate(mu8, klo) > 0) & valid
        ys, xs = np.nonzero(band)
        if len(ys) < 30:
            print(f"   {lo:3d}-{hi:3d}px: (점 부족)")
            continue
        base = to_base(unproject(depth, scale, K, ys, xs), he, tcp)
        z = base[:, 2] * 1000
        # 로버스트: 중앙값 주변 (테이블 평면)
        zmed = np.median(z)
        zc = z[np.abs(z - zmed) < 15]
        mmpp = np.median(depth[ys, xs]) * scale * 1000  # 대략 거리
        print(f"   {lo:3d}-{hi:3d}px (~{(lo+hi)/2*mmpp/650:4.0f}mm실거리): "
              f"z p50={np.percentile(zc,50):+5.1f} p16={np.percentile(zc,16):+5.1f} "
              f"p84={np.percentile(zc,84):+5.1f}mm  (n={len(zc)})")
