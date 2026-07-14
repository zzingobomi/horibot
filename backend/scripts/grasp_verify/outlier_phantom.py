"""실제 phantom 재현 — 멀리 아래로 튀는 depth outlier (flying-pixel/배경 누출).

옛 실물 버그: base_z −0.23m, height 19cm. 인접 책상이 아니라 **물체보다 한참 아래**
점들. mask 경계 flying-pixel 이나 책상 모서리 너머 배경이 물체 점군에 섞이면 발생.
검증: outlier 비율별로 현재 2-percentile bottom 이 무너지나 + z-gap 군집 완화책이
막나. (footprint 는 윗면 band 라 무관 예상.)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pybullet as p

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "backend"))

from modules.detector import geometry as DG  # noqa: E402
from modules.detector.contract import OrientedDetection  # noqa: E402
from modules.tasks.pick_and_place import geometry as TG  # noqa: E402

BASE_Z = -0.045
W, H_PX, NEAR, FAR, FOV_V = 424, 240, 0.02, 0.6, 58.0
RNG = np.random.default_rng(2)


def render_cloud(obj_half, obj_center, cam_pos, z_c):
    cid = p.connect(p.DIRECT)
    try:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=obj_half, physicsClientId=cid)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=obj_half, physicsClientId=cid)
        body = p.createMultiBody(0, col, vis, obj_center, physicsClientId=cid)
        upw = np.array([1.0, 0.0, 0.0]) if abs(z_c[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
        view = p.computeViewMatrix(cam_pos.tolist(), (cam_pos + z_c).tolist(),
                                   upw.tolist(), physicsClientId=cid)
        proj = p.computeProjectionMatrixFOV(FOV_V, W / H_PX, NEAR, FAR, physicsClientId=cid)
        _, _, _, depth, seg = p.getCameraImage(
            W, H_PX, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=cid)
        depth = np.array(depth).reshape(H_PX, W); seg = np.array(seg).reshape(H_PX, W)
        V = np.array(view).reshape(4, 4, order="F"); P = np.array(proj).reshape(4, 4, order="F")
        inv = np.linalg.inv(P @ V)
        ys, xs = np.where(seg == body)
        d = depth[ys, xs]
        clip = np.stack([2.0*xs/W-1.0, 1.0-2.0*ys/H_PX, 2.0*d-1.0, np.ones_like(d)], axis=1)
        world = clip @ inv.T
        return world[:, :3] / world[:, 3:4]
    finally:
        p.disconnect(cid)


def add_outliers(cloud, center, frac):
    n = int(len(cloud) * frac)
    if n == 0:
        return cloud
    ox = center[0] + RNG.normal(0, 0.02, n)
    oy = center[1] + RNG.normal(0, 0.02, n)
    oz = RNG.uniform(-0.30, -0.12, n)  # 물체보다 한참 아래 (phantom 영역)
    return np.vstack([cloud, np.stack([ox, oy, oz], axis=1)])


def robust_bottom(cloud):
    """top 에서 아래로 5mm 빈 틈 만나기 전까지 = 물체 몸통 (아래 봉우리 절단)."""
    z = np.sort(cloud[:, 2])[::-1]
    bottom = z[0]
    for a, b in zip(z[:-1], z[1:]):
        if a - b > 0.005:
            break
        bottom = b
    return bottom


def main():
    half = (0.0115, 0.0110, 0.0115)
    center = np.array([0.24, 0.10, BASE_Z + half[2]])
    tb, tt = center[2]-half[2], center[2]+half[2]
    el = math.radians(45); az = math.radians(20)
    up = np.array([math.cos(az)*math.cos(el), math.sin(az)*math.cos(el), math.sin(el)])
    base = render_cloud(list(half), center.tolist(), center + 0.16*up, -up)

    print(f"cube2.3 사선45° + 아래-outlier (GT bottom {tb*100:.1f}cm height 2.3cm)\n")
    print("outlier | 현재 2%ile: base_z/height/grasp | 완화(z-gap): base_z/height/grasp | fp_err")
    for frac in (0.0, 0.01, 0.03, 0.05, 0.1):
        cloud = add_outliers(base, center, frac)
        m = DG.object_metrics_from_points(cloud)
        obb = DG.obb_from_base_points(DG.top_face_points(cloud))
        pos, bz, h = m
        det = OrientedDetection(prompt="o", position=pos, score=1.0, base_z=bz, height=h,
                                grasp_yaw=obb.yaw_rad, footprint=obb.footprint)
        gz = TG.plan_grasp(det)[0].grasp[2]; ok = tb-0.002 <= gz <= tt+0.002
        fp_err = max(abs(obb.footprint[0]-0.023), abs(obb.footprint[1]-0.022))*1000
        bz2 = robust_bottom(cloud); h2 = max(0.0, pos[2]-bz2)
        det2 = OrientedDetection(prompt="o", position=pos, score=1.0, base_z=bz2, height=h2,
                                 grasp_yaw=obb.yaw_rad, footprint=obb.footprint)
        gz2 = TG.plan_grasp(det2)[0].grasp[2]; ok2 = tb-0.002 <= gz2 <= tt+0.002
        print(f"  {frac*100:3.0f}%  | bz={bz*100:6.1f} h={h*100:5.1f} {'OK ' if ok else 'FAIL'}"
              f"       | bz={bz2*100:6.1f} h={h2*100:4.1f} {'OK ' if ok2 else 'FAIL'}"
              f"      | {fp_err:.1f}mm")


if __name__ == "__main__":
    main()
