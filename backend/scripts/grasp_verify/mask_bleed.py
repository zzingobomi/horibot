"""마스크 불완전(책상 bleed) 하 object-centric 기하 견고성 — 옛 phantom 의 마스크판.

SAM 마스크가 물체 경계 밖 책상 픽셀을 조금 물면, 물체 점군에 z≈table 점이 섞인다.
object_metrics 의 base_z(하위 percentile)가 책상으로 끌려가 height 부풀 → grasp_z
오류 (옛 ring-floor phantom 과 같은 클래스). 검증: bleed 비율별로
  (a) 현재 코드(2 percentile bottom)가 얼마나 망가지나
  (b) footprint(윗면 band)는 보호되나
  (c) 완화책 — 최대 밀도 z-군집(물체 몸통)만 취하면 회복되나
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
RNG = np.random.default_rng(1)


def render_cloud(obj_half, obj_center, cam_pos, z_c):
    cid = p.connect(p.DIRECT)
    try:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=obj_half, physicsClientId=cid)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=obj_half, physicsClientId=cid)
        body = p.createMultiBody(0, col, vis, obj_center, physicsClientId=cid)
        upw = np.array([0.0, 0.0, 1.0])
        if abs(float(upw @ z_c)) > 0.95:
            upw = np.array([1.0, 0.0, 0.0])
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


def add_table_bleed(cloud, center, frac):
    """물체 주변 책상 점(z≈BASE_Z)을 물체 점의 frac 비율만큼 섞음 (마스크 경계 누출)."""
    n = int(len(cloud) * frac)
    if n == 0:
        return cloud
    r = 0.03
    ang = RNG.uniform(0, 2*math.pi, n)
    rad = RNG.uniform(0.01, r, n)
    tx = center[0] + rad*np.cos(ang)
    ty = center[1] + rad*np.sin(ang)
    tz = np.full(n, BASE_Z) + RNG.normal(0, 0.0007, n)
    return np.vstack([cloud, np.stack([tx, ty, tz], axis=1)])


def robust_metrics(cloud):
    """완화책: z 히스토그램에서 물체 몸통(최대 밀도 구간)만 추려 base_z/height.

    책상 bleed 는 z=table 에 별도 봉우리를 만든다. 물체 점(top~side)은 더 위 봉우리.
    최상위 봉우리(물체)에서 bottom 을 잡으면 책상에 안 끌린다.
    """
    z = cloud[:, 2]
    top_z = float(np.percentile(z, 98))
    # 위에서부터 연속적인 물체 몸통: top 에서 아래로 gap(>5mm 빈 구간) 만나기 전까지
    zs = np.sort(z)[::-1]
    bottom = zs[0]
    for a, b in zip(zs[:-1], zs[1:]):
        if a - b > 0.005:  # 5mm 빈 틈 = 물체와 책상 사이
            break
        bottom = b
    return top_z, bottom, max(0.0, top_z - bottom)


def main():
    half = (0.0115, 0.0110, 0.0115)  # 2.3cm cube
    center = np.array([0.24, 0.10, BASE_Z + half[2]])
    tb, tt = center[2]-half[2], center[2]+half[2]
    el = math.radians(45); az = math.radians(20)
    up = np.array([math.cos(az)*math.cos(el), math.sin(az)*math.cos(el), math.sin(el)])
    base = render_cloud(list(half), center.tolist(), center + 0.16*up, -up)

    print("cube2.3 사선45° + 책상 bleed 비율별 (GT height 2.3cm, top -3.9→bottom -4.5cm)\n")
    print("bleed  | 현재코드 base_z/height/grasp유효 | 완화책 base_z/height/grasp유효 | footprint_err")
    for frac in (0.0, 0.05, 0.1, 0.2, 0.4):
        cloud = add_table_bleed(base, center, frac)
        # 현재 코드
        m = DG.object_metrics_from_points(cloud)
        obb = DG.obb_from_base_points(DG.top_face_points(cloud))
        pos, bz, h = m
        det = OrientedDetection(prompt="o", position=pos, score=1.0, base_z=bz, height=h,
                                grasp_yaw=obb.yaw_rad, footprint=obb.footprint)
        gz = TG.plan_grasp(det)[0].grasp[2]
        ok = tb-0.002 <= gz <= tt+0.002
        fp_err = max(abs(obb.footprint[0]-0.023), abs(obb.footprint[1]-0.022))*1000
        # 완화책
        tz2, bz2, h2 = robust_metrics(cloud)
        det2 = OrientedDetection(prompt="o", position=(pos[0], pos[1], tz2), score=1.0,
                                 base_z=bz2, height=h2, grasp_yaw=obb.yaw_rad,
                                 footprint=obb.footprint)
        gz2 = TG.plan_grasp(det2)[0].grasp[2]
        ok2 = tb-0.002 <= gz2 <= tt+0.002
        print(f" {frac*100:3.0f}%  | bz={bz*100:5.1f} h={h*100:4.1f} {'OK ' if ok else 'FAIL'}"
              f"        | bz={bz2*100:5.1f} h={h2*100:4.1f} {'OK ' if ok2 else 'FAIL'}"
              f"       | {fp_err:.1f}mm")

    print("\n해석: 현재 2-percentile bottom 은 책상 bleed 에 base_z 가 끌려가 height 부풀 "
          "→ grasp_z 오류(옛 phantom 재현). footprint(윗면 band)는 bleed 무관하게 견고. "
          "z-gap 군집(완화책)이면 책상 봉우리를 잘라내 회복되는지 확인.")


if __name__ == "__main__":
    main()
