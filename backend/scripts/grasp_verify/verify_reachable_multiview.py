"""닿는 뷰만으로 멀티뷰 융합 → antipodal 파지 사나 (adaptive 관측 급소).

앞 실험은 뷰 방위를 임의 지정했다. 실제로는 팔이 닿는 뷰만 쓸 수 있고, 그게 base
쪽으로 쏠리면 물체 먼 면을 못 봐 antipodal 이 안 될 수 있다. 실 kinematics 로 닿는
뷰를 모아(방위 스프레드 기록) 그 점군만 융합해 antipodal 을 재검증.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import pybullet as p
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "backend"))

from apps.config import load_robots  # noqa: E402
from infra.database.sqlite import open_sqlite  # noqa: E402
from modules.calibration.persistence.repository import CalibrationRepository  # noqa: E402
from modules.motion.adapters.pybullet import PybulletKinematics  # noqa: E402
from modules.motion.kinematics_builder import build_calibrated_kinematics  # noqa: E402
from modules.motor.contract import MotorKind  # noqa: E402

BASE_Z = -0.045
W, H_PX, NEAR, FAR, FOV_V = 424, 240, 0.02, 0.6, 58.0
MAX_W, MIN_W = 0.035, 0.004
ANG_TOL, LAT_TOL = math.radians(25), 0.004
SEED = [-1.07, 2.26, -1.82, 0.81, 0.61, 0.64]


def build_kin():
    robots = load_robots(REPO / "robot")
    r = robots["so101_6dof_0"]
    arm = [m for m in r.motors if m.kind != MotorKind.GRIPPER]
    _, factory = open_sqlite(REPO / "backend" / "horibot.db")
    bundle = CalibrationRepository(factory).get_active_bundle("so101_6dof_0")
    urdf = REPO / "robot" / r.type / "urdf" / f"{r.type}.urdf"
    b = build_calibrated_kinematics(urdf, "so101_6dof_0", arm, bundle, PybulletKinematics)
    b.kinematics.initialize()
    return b.kinematics, bundle.hand_eye.result_data


def make_body(cid, kind, center):
    if kind == "box":
        s = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.018, 0.012, 0.015], physicsClientId=cid)
        return p.createMultiBody(0, s, -1, center, physicsClientId=cid)
    if kind == "cyl_h":
        s = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.013, height=0.05, physicsClientId=cid)
        q = p.getQuaternionFromEuler([0, math.pi/2, 0])
        return p.createMultiBody(0, s, -1, center, baseOrientation=q, physicsClientId=cid)
    if kind == "sphere":
        s = p.createCollisionShape(p.GEOM_SPHERE, radius=0.016, physicsClientId=cid)
        return p.createMultiBody(0, s, -1, center, physicsClientId=cid)


def reachable_cam(kin, tgt, r_ce, t_ce, radius, elev, az):
    tgt = np.array(tgt); el, a = math.radians(elev), math.radians(az)
    up = np.array([math.cos(a)*math.cos(el), math.sin(a)*math.cos(el), math.sin(el)])
    cam_pos = tgt + radius*up
    if cam_pos[2] < BASE_Z + 0.02:
        return None
    z_c = -up
    tmp = np.array([0.0, 0.0, 1.0]) if abs(z_c[2]) <= 0.95 else np.array([1.0, 0.0, 0.0])
    x0 = np.cross(tmp, z_c); x0 /= np.linalg.norm(x0); y0 = np.cross(z_c, x0)
    for roll in range(0, 360, 45):
        rr = math.radians(roll)
        xc = math.cos(rr)*x0 + math.sin(rr)*y0
        r_bc = np.column_stack([xc, np.cross(z_c, xc), z_c])
        r_be = r_bc @ r_ce.T
        t_be = cam_pos - r_be @ t_ce
        q = tuple(float(v) for v in Rotation.from_matrix(r_be).as_quat())
        sol = kin.ik((float(t_be[0]), float(t_be[1]), float(t_be[2])), q, SEED, 25)
        if sol is not None:
            r_be2, t_be2 = kin.fk_to_matrix(sol)
            r_be2 = np.array(r_be2); t_be2 = np.array(t_be2)
            return t_be2 + r_be2 @ t_ce, r_be2 @ r_ce, az
    return None


def render(kind, center, cam_pos, r_bc):
    cid = p.connect(p.DIRECT)
    try:
        body = make_body(cid, kind, center)
        z_c = r_bc[:, 2]; up = -r_bc[:, 1]
        view = p.computeViewMatrix(cam_pos.tolist(), (cam_pos+z_c).tolist(), up.tolist(), physicsClientId=cid)
        proj = p.computeProjectionMatrixFOV(FOV_V, W/H_PX, NEAR, FAR, physicsClientId=cid)
        _, _, _, depth, seg = p.getCameraImage(W, H_PX, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=cid)
        depth = np.array(depth).reshape(H_PX, W); seg = np.array(seg).reshape(H_PX, W)
        V = np.array(view).reshape(4, 4, order="F"); P = np.array(proj).reshape(4, 4, order="F")
        inv = np.linalg.inv(P @ V)
        ys, xs = np.where(seg >= 0)
        if xs.size == 0:
            return None
        d = depth[ys, xs]
        clip = np.stack([2.0*xs/W-1.0, 1.0-2.0*ys/H_PX, 2.0*d-1.0, np.ones_like(d)], axis=1)
        w = clip @ inv.T
        return w[:, :3] / w[:, 3:4]
    finally:
        p.disconnect(cid)


def antipodal(cloud, center):
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(cloud)
    pc = pc.voxel_down_sample(0.003)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    pts = np.asarray(pc.points); nrm = np.asarray(pc.normals)
    c = pts.mean(axis=0); nrm[np.sum((pts-c)*nrm, axis=1) < 0] *= -1
    cnt = 0; best = None
    for i in range(0, len(pts), max(1, len(pts)//400)):
        d = -nrm[i]; rel = pts - pts[i]; t = rel @ d
        lat = np.linalg.norm(rel - np.outer(t, d), axis=1)
        cand = (t > MIN_W) & (t < MAX_W) & (lat < LAT_TOL)
        if not cand.any():
            continue
        al = (nrm[cand] @ d) > math.cos(ANG_TOL)
        if al.any():
            cnt += 1
            j = np.where(cand)[0][al][0]
            mid = (pts[i]+pts[j])/2
            if best is None and (pts[i]-mid) @ (pts[j]-mid) < 0:
                best = (float(t[j]), True)
    return cnt, best


def main():
    kin, hd = build_kin()
    r_ce = np.array(hd.R_cam2gripper, float); t_ce = np.array(hd.t_cam2gripper, float).reshape(3)
    print("타깃 위치별·형상별: 닿는뷰수(방위스프레드°) → 융합 antipodal쌍수(반대편O?)\n")
    for tx, ty in [(0.24, 0.10), (0.20, 0.00), (0.27, 0.18)]:
        for kind in ["box", "cyl_h", "sphere"]:
            center = np.array([tx, ty, BASE_Z + 0.018])
            views, azs = [], []
            for radius in (0.13, 0.16):
                for elev in (40, 55, 70):
                    for az in range(0, 360, 30):
                        rc = reachable_cam(kin, center, r_ce, t_ce, radius, elev, az)
                        if rc is None:
                            continue
                        cl = render(kind, center.tolist(), rc[0], rc[1])
                        if cl is not None and len(cl) > 50:
                            views.append(cl); azs.append(rc[2])
                        if len(views) >= 8:
                            break
                    if len(views) >= 8:
                        break
                if len(views) >= 8:
                    break
            if not views:
                print(f"  ({tx:.2f},{ty:+.2f}) {kind:6s}: 닿는 뷰 0"); continue
            spread = max(azs) - min(azs) if len(azs) > 1 else 0
            fused = np.vstack(views)
            cnt, best = antipodal(fused, center)
            bt = f"폭{best[0]*100:.1f}cm 반대편O" if best else "반대편 무는쌍 없음"
            print(f"  ({tx:.2f},{ty:+.2f}) {kind:6s}: 닿는뷰 {len(views)}개(방위{spread:3.0f}°) "
                  f"→ {cnt:3d}쌍, {bt}")
        print()


if __name__ == "__main__":
    main()
