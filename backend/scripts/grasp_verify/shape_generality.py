"""단일 뷰 충분성이 형상 일반적인가 — prismatic vs 비-prismatic 대조.

"윗면 footprint 로 파지 폭 결정"은 **높이 따라 수평 단면이 일정한(prismatic)** 형상
에서만 맞다. 박스/세운 원기둥=OK. 구/원뿔/눕힌 원기둥=윗면이 파지 높이 단면과
달라 footprint 오판 → 조 폭 틀림. 단일 사선 뷰 렌더로 측정 footprint vs 파지높이
실제 단면을 비교해 경계를 데이터로 확인한다.
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


def make_shape(cid, kind, dims, center):
    if kind == "box":
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=dims, physicsClientId=cid)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=dims, physicsClientId=cid)
    elif kind == "cyl_v":  # 세운 원기둥 (prismatic)
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=dims[0], height=dims[1],
                                     physicsClientId=cid)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=dims[0], length=dims[1],
                                  physicsClientId=cid)
    elif kind == "sphere":  # 구 (비-prismatic)
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=dims[0], physicsClientId=cid)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=dims[0], physicsClientId=cid)
    return p.createMultiBody(0, col, vis, center, physicsClientId=cid)


def render(kind, dims, center, cam_pos, z_c):
    cid = p.connect(p.DIRECT)
    try:
        body = make_shape(cid, kind, dims, center)
        upw = np.array([1.0, 0.0, 0.0]) if abs(z_c[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
        view = p.computeViewMatrix(cam_pos.tolist(), (cam_pos + z_c).tolist(),
                                   upw.tolist(), physicsClientId=cid)
        proj = p.computeProjectionMatrixFOV(FOV_V, W / H_PX, NEAR, FAR, physicsClientId=cid)
        _, _, _, depth, seg = p.getCameraImage(W, H_PX, view, proj,
                                               renderer=p.ER_TINY_RENDERER,
                                               physicsClientId=cid)
        depth = np.array(depth).reshape(H_PX, W); seg = np.array(seg).reshape(H_PX, W)
        V = np.array(view).reshape(4, 4, order="F"); P = np.array(proj).reshape(4, 4, order="F")
        inv = np.linalg.inv(P @ V)
        ys, xs = np.where(seg == body)
        d = depth[ys, xs]
        clip = np.stack([2.0*xs/W-1.0, 1.0-2.0*ys/H_PX, 2.0*d-1.0, np.ones_like(d)], axis=1)
        w = clip @ inv.T
        return w[:, :3] / w[:, 3:4]
    finally:
        p.disconnect(cid)


def true_cross_section(kind, dims, z_from_center):
    """파지 높이(중심 기준 z)에서 실제 수평 단면 최대폭 (조가 벌려야 할 폭)."""
    if kind == "box":
        return 2 * min(dims[0], dims[1])  # 짧은 변
    if kind == "cyl_v":
        return 2 * dims[0]  # 지름 (높이 무관)
    if kind == "sphere":
        R = dims[0]
        r = math.sqrt(max(0.0, R*R - z_from_center*z_from_center))
        return 2 * r
    return 0.0


def main():
    center_xy = (0.24, 0.10)
    shapes = [
        ("box", (0.0115, 0.011, 0.0115), 0.0115, "prismatic"),
        ("cyl_v", (0.012, 0.030), 0.015, "prismatic"),   # r=1.2 h=3 → 반높이1.5
        ("sphere", (0.016,), 0.016, "비-prismatic"),      # R=1.6cm
    ]
    el = math.radians(45); az = math.radians(20)
    up = np.array([math.cos(az)*math.cos(el), math.sin(az)*math.cos(el), math.sin(el)])

    print("단일 사선45° 뷰: 측정 footprint(짧은변) vs 파지높이 실제단면 → 조 폭 맞나\n")
    for kind, dims, halfz, klass in shapes:
        center = np.array([center_xy[0], center_xy[1], BASE_Z + halfz])
        cloud = render(kind, dims, center.tolist(), center + 0.16*up, -up)
        if cloud is None or len(cloud) < 20:
            print(f"[{kind}] 점군 부족"); continue
        m = DG.object_metrics_from_points(cloud)
        obb = DG.obb_from_base_points(DG.top_face_points(cloud))
        if m is None or obb is None:
            print(f"[{kind}] 측정 실패"); continue
        pos, bz, h = m
        det = OrientedDetection(prompt="o", position=pos, score=1.0, base_z=bz, height=h,
                                grasp_yaw=obb.yaw_rad, footprint=obb.footprint)
        cand = TG.plan_grasp(det)[0]
        gz = cand.grasp[2]
        z_from_center = gz - center[2]
        true_w = true_cross_section(kind, dims, z_from_center)
        meas_w = min(obb.footprint)  # 조가 무는 짧은 변
        err = (meas_w - true_w) * 1000
        verdict = "OK" if abs(err) < 5 else f"조폭 {abs(err):.0f}mm {'과소→헛집음' if err<0 else '과대'}"
        print(f"[{kind:7s} {klass:11s}] 측정단면={meas_w*100:.1f}cm  "
              f"파지높이 실제단면={true_w*100:.1f}cm  오차={err:+.0f}mm  → {verdict}")

    print("\n해석: prismatic(box/세운원기둥)은 윗면=파지단면이라 단일뷰 OK. "
          "구는 윗면 band 가 파지높이(적도) 단면보다 좁게 잡혀 조 폭 과소 → 헛집음. "
          "= 단일뷰 충분은 prismatic 한정 결론.")


if __name__ == "__main__":
    main()
