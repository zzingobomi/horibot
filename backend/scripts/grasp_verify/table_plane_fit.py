"""테이블 평면을 base frame 에서 RANSAC 피팅 → 오차의 '종류'를 판별.

테이블은 물리적으로 z=0, 법선=(0,0,1) 이어야 한다.
- 법선이 (0,0,1) 에서 크게 틀어짐        → hand-eye/FK '회전' 오차
- 법선은 맞는데 z 오프셋만 +15~20mm      → hand-eye/FK '병진'(수직) 오차
- 프레임마다 오프셋이 자세 따라 varying    → hand-eye(카메라 extrinsic) 유력
- 프레임마다 오프셋이 일정                 → FK(로봇 kinematic) 유력

하드웨어 0."""
import sys
import os
import json
import glob
import numpy as np
import cv2


def unproject_masked(depth, scale, K, pixmask):
    ys, xs = np.nonzero(pixmask)
    z = depth[ys, xs].astype(np.float64) * scale
    x = (xs - K["cx"]) / K["fx"] * z
    y = (ys - K["cy"]) / K["fy"] * z
    return np.stack([x, y, z], axis=-1)


def to_base(cam, he, tcp):
    ee = cam @ np.array(he["R"]).T + np.array(he["t"])
    return ee @ np.array(tcp["R"]).T + np.array(tcp["t"])


def fit_plane(pts, iters=500, thr=0.003):
    best_n, best_d, best_in = None, None, -1
    n = len(pts)
    rng = np.arange(n)
    # 결정적: 고정 seed 없이 균등 샘플 (재현 위해 인덱스 순회)
    step = max(n // iters, 1)
    for i in range(0, n - 3, step):
        p3 = pts[[i, (i + step) % n, (i + 2 * step) % n]]
        v1, v2 = p3[1] - p3[0], p3[2] - p3[0]
        nrm = np.cross(v1, v2)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        d = -nrm @ p3[0]
        dist = np.abs(pts @ nrm + d)
        inl = int((dist < thr).sum())
        if inl > best_in:
            best_in, best_n, best_d = inl, nrm, d
    # inlier 로 refit (SVD)
    dist = np.abs(pts @ best_n + best_d)
    inl = pts[dist < thr]
    c = inl.mean(0)
    _, _, vt = np.linalg.svd(inl - c)
    nrm = vt[2]
    if nrm[2] < 0:
        nrm = -nrm
    d = -nrm @ c
    return nrm, d, c, len(inl), len(pts)


session = sys.argv[1] if len(sys.argv) > 1 else "20260715_211723"
d = f"debug/detect/{session}"
print(f"=== {session} : 테이블 평면 base-frame 피팅 ===")
print("정상 = 법선(0,0,1), 평면 z=0\n")
for jf in sorted(glob.glob(os.path.join(d, "*cube*.json"))):
    with open(jf, encoding="utf-8") as f:
        meta = json.load(f)
    if "0003" in jf:  # 테이블 통째로 잡은 오검출 skip
        continue
    depth = cv2.imread(os.path.join(d, meta["depth_png"]), cv2.IMREAD_UNCHANGED)
    scale, K = meta["depth_scale"], meta["intrinsics"]
    he, tcp = meta["hand_eye_cam2ee"], meta["tcp_ee2base"]
    c0 = meta["candidates"][0]
    mask = cv2.imread(os.path.join(d, c0["mask_png"]), cv2.IMREAD_UNCHANGED) > 0
    valid = depth > 0
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (55, 55))
    ring = (cv2.dilate(mask.astype(np.uint8), k) > 0) & (~mask) & valid
    cam = unproject_masked(depth, scale, K, ring)
    base = to_base(cam, he, tcp)
    # 극단 outlier 제거 (오검출 프레임 방어)
    zmed = np.median(base[:, 2])
    base = base[np.abs(base[:, 2] - zmed) < 0.05]
    if len(base) < 50:
        print(f"{os.path.basename(jf)}: 테이블 점 부족")
        continue
    nrm, dd, cen, ni, nt = fit_plane(base)
    tilt = np.degrees(np.arccos(min(abs(nrm[2]), 1.0)))
    plane_z_at_cube = -(nrm[0] * cen[0] + nrm[1] * cen[1] + dd) / nrm[2]
    print(
        f"{os.path.basename(jf).replace('_det_white_small_round_cube','')}  "
        f"ee_pos=({tcp['t'][0]:.3f},{tcp['t'][1]:.3f},{tcp['t'][2]:.3f})\n"
        f"   테이블 평면: 법선=({nrm[0]:+.3f},{nrm[1]:+.3f},{nrm[2]:+.3f}) "
        f"기울기={tilt:.1f}° | 평면 z@중심={plane_z_at_cube*1000:+.1f}mm "
        f"(정상=0) | inlier {ni}/{nt}"
    )
