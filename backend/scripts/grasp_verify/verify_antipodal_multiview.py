"""일반 형상에서 표면 antipodal 파지: 단일 뷰 vs 멀티뷰 — 깨지는 케이스 위주.

thesis(형): 일반 형상은 단일 뷰가 반대쪽 접촉면을 못 봐(occlusion) antipodal 파지쌍을
못 찾는다 → 멀티뷰 누적이 필요. 이걸 깨지는 형상(구/눕힌원기둥/L자 concave)에서 실증.

파이프라인: PyBullet 렌더로 물리적 부분 점군 → open3d 법선 → antipodal 탐색
(마주보는 두 표면점: 접근선이 양 법선의 마찰콘 안 + 폭≤그리퍼). 단일 뷰 vs 3뷰 융합
에서 유효 파지쌍 수 + 최선 파지가 물체 반대쪽 두 면을 진짜 무는지.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import pybullet as p

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "backend"))

BASE_Z = -0.045
W, H_PX, NEAR, FAR, FOV_V = 424, 240, 0.02, 0.6, 58.0
MAX_W, MIN_W = 0.035, 0.004   # 그리퍼 개폭 가정 (m)
ANG_TOL = math.radians(25)    # antipodal 법선 정렬 허용각
LAT_TOL = 0.004               # 접근선에서 접촉점 측방 허용 (m)


def make_body(cid, kind, center):
    """convex 는 단일 shape, L자 concave 는 두 박스 합체(base+link)."""
    if kind == "box":
        s = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.018, 0.012, 0.015], physicsClientId=cid)
        return p.createMultiBody(0, s, -1, center, physicsClientId=cid), 0.015
    if kind == "cyl_v":
        s = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.013, height=0.03, physicsClientId=cid)
        return p.createMultiBody(0, s, -1, center, physicsClientId=cid), 0.015
    if kind == "cyl_h":  # 눕힌 원기둥 (축 = x)
        s = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.013, height=0.05, physicsClientId=cid)
        q = p.getQuaternionFromEuler([0, math.pi/2, 0])
        return p.createMultiBody(0, s, -1, center, baseOrientation=q, physicsClientId=cid), 0.013
    if kind == "sphere":
        s = p.createCollisionShape(p.GEOM_SPHERE, radius=0.016, physicsClientId=cid)
        return p.createMultiBody(0, s, -1, center, physicsClientId=cid), 0.016
    if kind == "Lshape":  # concave — 두 박스
        s1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.022, 0.010, 0.010], physicsClientId=cid)
        s2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.010, 0.010, 0.018], physicsClientId=cid)
        return p.createMultiBody(
            0, s1, -1, center, physicsClientId=cid,
            linkMasses=[0], linkCollisionShapeIndices=[s2], linkVisualShapeIndices=[-1],
            linkPositions=[[0.012, 0, 0.020]], linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0]], linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0], linkJointTypes=[p.JOINT_FIXED], linkJointAxis=[[0, 0, 1]],
        ), 0.010


def render(kind, center, cam_pos, z_c):
    cid = p.connect(p.DIRECT)
    try:
        body, _ = make_body(cid, kind, center)
        upw = np.array([1.0, 0.0, 0.0]) if abs(z_c[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
        view = p.computeViewMatrix(cam_pos.tolist(), (cam_pos + z_c).tolist(), upw.tolist(), physicsClientId=cid)
        proj = p.computeProjectionMatrixFOV(FOV_V, W / H_PX, NEAR, FAR, physicsClientId=cid)
        _, _, _, depth, seg = p.getCameraImage(W, H_PX, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=cid)
        depth = np.array(depth).reshape(H_PX, W); seg = np.array(seg).reshape(H_PX, W)
        V = np.array(view).reshape(4, 4, order="F"); P = np.array(proj).reshape(4, 4, order="F")
        inv = np.linalg.inv(P @ V)
        ys, xs = np.where(seg >= 0)  # 물체(base+link 모두 seg>=0), 배경 -1
        if xs.size == 0:
            return None
        d = depth[ys, xs]
        clip = np.stack([2.0*xs/W-1.0, 1.0-2.0*ys/H_PX, 2.0*d-1.0, np.ones_like(d)], axis=1)
        w = clip @ inv.T
        return w[:, :3] / w[:, 3:4]
    finally:
        p.disconnect(cid)


def cam(center, elev, az, r=0.16):
    el, a = math.radians(elev), math.radians(az)
    up = np.array([math.cos(a)*math.cos(el), math.sin(a)*math.cos(el), math.sin(el)])
    return center + r*up, -up


def normals(cloud):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(cloud)
    pc = pc.voxel_down_sample(0.003)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    pts = np.asarray(pc.points)
    nrm = np.asarray(pc.normals)
    c = pts.mean(axis=0)
    flip = np.sum((pts - c) * nrm, axis=1) < 0  # 바깥 향하게
    nrm[flip] *= -1
    return pts, nrm


def antipodal_grasps(pts, nrm):
    """유효 antipodal 파지쌍 리스트 (i, j, width). 접근선이 양 법선 마찰콘 안 + 폭 범위."""
    out = []
    n = len(pts)
    step = max(1, n // 400)  # 후보 상한
    for i in range(0, n, step):
        pi, ni = pts[i], nrm[i]
        d = -ni  # 물체 안으로 (반대 접촉면 방향)
        rel = pts - pi
        t = rel @ d
        lat = np.linalg.norm(rel - np.outer(t, d), axis=1)
        cand = (t > MIN_W) & (t < MAX_W) & (lat < LAT_TOL)
        if not cand.any():
            continue
        # 반대 법선 anti-parallel + 접근선 정렬
        nj = nrm[cand]
        # 접촉면 j 의 법선이 접근선(d)과 마주봐야: nj·d > cos(tol)  (nj 바깥향 → d 와 같은 쪽)
        aligned = (nj @ d) > math.cos(ANG_TOL)
        # 내 쪽 접촉: ni 가 -d 와 정렬 (자명) — i 는 표면점이라 통과
        if aligned.any():
            js = np.where(cand)[0][aligned]
            j = js[np.argmin(lat[cand][aligned])]
            out.append((i, int(j), float(t[j])))
    return out


def evaluate(pts, nrm, center):
    grasps = antipodal_grasps(pts, nrm)
    if not grasps:
        return 0, None
    # 최선 = 접촉 중점이 물체 중심(xy)에 가장 가까운 것 (안정)
    best = min(grasps, key=lambda g: np.linalg.norm(
        (pts[g[0]] + pts[g[1]]) / 2 - center)[..., None].sum())
    i, j, wdt = best
    mid = (pts[i] + pts[j]) / 2
    # 두 접촉이 중심 기준 반대편인가 (진짜 감싸 무는지)
    opp = float((pts[i] - mid) @ (pts[j] - mid)) < 0
    return len(grasps), (wdt, opp)


def main():
    shapes = ["box", "cyl_v", "cyl_h", "sphere", "Lshape"]
    cxy = (0.24, 0.10)
    views = [(50, 20), (45, 140), (55, 260)]  # 3 뷰 방위 분산
    print("형상별: 단일뷰 antipodal쌍수(반대편무는지) | 3뷰융합 antipodal쌍수(반대편무는지)\n")
    for kind in shapes:
        center = np.array([cxy[0], cxy[1], BASE_Z + 0.018])
        clouds = []
        for elev, az in views:
            cp, zc = cam(center, elev, az)
            cl = render(kind, center.tolist(), cp, zc)
            if cl is not None and len(cl) > 50:
                clouds.append(cl)
        if not clouds:
            print(f"[{kind}] 렌더 실패"); continue
        # 단일 뷰 (첫 뷰)
        pts1, nrm1 = normals(clouds[0])
        n1, best1 = evaluate(pts1, nrm1, center)
        # 3뷰 융합
        fused = np.vstack(clouds)
        ptsf, nrmf = normals(fused)
        nf, bestf = evaluate(ptsf, nrmf, center)
        def fmt(n, b):
            if b is None:
                return f"{n:3d}쌍 (파지쌍 없음)"
            return f"{n:3d}쌍 폭{b[0]*100:.1f}cm {'반대편O' if b[1] else '반대편X(한쪽만)'}"
        print(f"[{kind:7s}] 단일: {fmt(n1, best1):32s} | 3뷰: {fmt(nf, bestf)}")

    print("\n(단일뷰에서 파지쌍 0/반대편X 인데 3뷰에서 살아나면 = 멀티뷰 필요 실증)")


if __name__ == "__main__":
    main()
