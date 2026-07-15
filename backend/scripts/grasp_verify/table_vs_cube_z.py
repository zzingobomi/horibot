"""결정적 테스트: 같은 프레임에서 큐브 mask 픽셀 vs 그 주변 테이블(나뭇결) 픽셀을
동일한 transform 으로 base frame 에 올려 z 를 비교.

- 테이블 z≈0 & 큐브 z≈+15mm  → 큐브 depth 가 object-specific 하게 편향 (near-bias 지지)
- 테이블도 z≈+15mm            → 이 자세의 cal/FK 오차 (depth 무죄, 상류가 상류)

하드웨어 0."""
import sys
import os
import json
import numpy as np
import cv2


def unproject(depth, scale, K):
    """전체 depth → 카메라 좌표 (H,W,3) meters."""
    H, W = depth.shape
    ys, xs = np.mgrid[0:H, 0:W]
    z = depth.astype(np.float64) * scale
    x = (xs - K["cx"]) / K["fx"] * z
    y = (ys - K["cy"]) / K["fy"] * z
    return np.stack([x, y, z], axis=-1)


def to_base(cam_pts, he, tcp):
    """cam(N,3) → ee → base."""
    Rhe = np.array(he["R"]); the = np.array(he["t"])
    Rt = np.array(tcp["R"]); tt = np.array(tcp["t"])
    ee = cam_pts @ Rhe.T + the
    return ee @ Rt.T + tt


def zbase(pts_cam_flat, he, tcp):
    b = to_base(pts_cam_flat, he, tcp)
    return b[:, 2] * 1000.0  # mm


def analyze(jpath):
    d = os.path.dirname(jpath)
    with open(jpath, encoding="utf-8") as f:
        meta = json.load(f)
    depth = cv2.imread(os.path.join(d, meta["depth_png"]), cv2.IMREAD_UNCHANGED)
    scale = meta["depth_scale"]
    K = meta["intrinsics"]
    he = meta["hand_eye_cam2ee"]
    tcp = meta["tcp_ee2base"]
    cam = unproject(depth, scale, K)  # (H,W,3)
    name = os.path.basename(jpath).replace("white_small_round_cube", "cube")
    print(f"\n### {name}  prompt='{meta['prompt']}'")
    for c in meta["candidates"]:
        mp = c.get("mask_png")
        if not mp:
            continue
        mask = cv2.imread(os.path.join(d, mp), cv2.IMREAD_UNCHANGED) > 0
        valid = depth > 0
        # 큐브: mask & valid
        cube_m = mask & valid
        # 테이블 ring: mask 를 dilate 한 고리에서 mask 제외 (물체 주변 평면)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (35, 35))
        ring = (cv2.dilate(mask.astype(np.uint8), k) > 0) & (~mask) & valid
        cz = zbase(cam[cube_m], he, tcp)
        rz = zbase(cam[ring], he, tcp)
        if len(cz) and len(rz):
            print(
                f"  c{c['index']} score={c['score']:.2f}\n"
                f"    큐브 mask  z(mm): p2={np.percentile(cz,2):.0f} "
                f"p50={np.percentile(cz,50):.0f} p98={np.percentile(cz,98):.0f} "
                f"min={cz.min():.0f} max={cz.max():.0f} (N={len(cz)})\n"
                f"    주변 테이블 z(mm): p2={np.percentile(rz,2):.0f} "
                f"p50={np.percentile(rz,50):.0f} p98={np.percentile(rz,98):.0f} "
                f"min={rz.min():.0f} max={rz.max():.0f} (N={len(rz)})\n"
                f"    → 테이블 바닥 z 가 0 이어야 정상. 큐브 bottom - 테이블 = 큐브 실제 착지 여부"
            )


session = sys.argv[1] if len(sys.argv) > 1 else "20260715_211723"
d = f"debug/detect/{session}"
import glob
for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
    analyze(jf)
# blue box 대조 1개
for jf in sorted(glob.glob(os.path.join(d, "*blue_box*.json")))[-1:]:
    analyze(jf)
