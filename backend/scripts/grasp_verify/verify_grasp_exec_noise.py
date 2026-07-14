"""노이즈+마스크bleed+outlier 하 antipodal→실행 견고성 (KEPT 파이프라인 스트레스).

앞 end-to-end 는 깨끗한 점군. 실 센서는 노이즈+마스크 누출+flying-pixel outlier.
이걸 융합 점군에 주입하고 Monte-Carlo 로 '실행 가능한 파지가 여전히 나오나' 성공률.
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
ANG_TOL, LAT_TOL = math.radians(25), 0.006
SEED = [-1.07, 2.26, -1.82, 0.81, 0.61, 0.64]
TCP_TO_FIXED_JAW, FIXED_JAW_CLEAR, APPROACH_CLEAR = 0.0079, 0.005, 0.06
TILTS = (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90)
HORIZ_TOL = math.radians(20)
RNG = np.random.default_rng(7)


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
        r_be = r_bc @ r_ce.T; t_be = cam_pos - r_be @ t_ce
        q = tuple(float(v) for v in Rotation.from_matrix(r_be).as_quat())
        sol = kin.ik((float(t_be[0]), float(t_be[1]), float(t_be[2])), q, SEED, 25)
        if sol is not None:
            r2, t2 = kin.fk_to_matrix(sol)
            return np.array(t2) + np.array(r2) @ t_ce, np.array(r2) @ r_ce
    return None


def render(kind, center, cam_pos, r_bc):
    cid = p.connect(p.DIRECT)
    try:
        make_body(cid, kind, center)
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


def fused_cloud(kin, kind, center, r_ce, t_ce):
    views = []
    for radius in (0.13, 0.16):
        for elev in (40, 55, 70):
            for az in range(0, 360, 30):
                rc = reachable_cam(kin, center, r_ce, t_ce, radius, elev, az)
                if rc is None:
                    continue
                cl = render(kind, center.tolist(), rc[0], rc[1])
                if cl is not None and len(cl) > 50:
                    views.append(cl)
                if len(views) >= 8:
                    break
            if len(views) >= 8:
                break
        if len(views) >= 8:
            break
    return np.vstack(views) if views else None


def corrupt(cloud, center, sigma, bleed, outlier):
    keep = RNG.random(len(cloud)) > 0.1
    c = cloud[keep] + RNG.normal(0, sigma, (keep.sum(), 3))
    parts = [c]
    nb = int(len(c)*bleed)
    if nb:
        ang = RNG.uniform(0, 2*math.pi, nb); rad = RNG.uniform(0.01, 0.03, nb)
        parts.append(np.stack([center[0]+rad*np.cos(ang), center[1]+rad*np.sin(ang),
                               np.full(nb, BASE_Z)+RNG.normal(0, 0.0007, nb)], axis=1))
    no = int(len(c)*outlier)
    if no:
        parts.append(np.stack([center[0]+RNG.normal(0, 0.02, no), center[1]+RNG.normal(0, 0.02, no),
                               RNG.uniform(-0.30, -0.12, no)], axis=1))
    return np.vstack(parts)


def horizontal_antipodal(cloud):
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(cloud)
    pc = pc.voxel_down_sample(0.003)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.012, max_nn=30))
    pts = np.asarray(pc.points); nrm = np.asarray(pc.normals)
    c = pts.mean(axis=0); nrm[np.sum((pts-c)*nrm, axis=1) < 0] *= -1
    out = []
    for i in range(0, len(pts), max(1, len(pts)//400)):
        d = -nrm[i]
        if abs(d[2]) > math.sin(HORIZ_TOL):
            continue
        rel = pts - pts[i]; t = rel @ d
        lat = np.linalg.norm(rel - np.outer(t, d), axis=1)
        cand = (t > MIN_W) & (t < MAX_W) & (lat < LAT_TOL)
        if not cand.any():
            continue
        al = (nrm[cand] @ d) > math.cos(ANG_TOL)
        if al.any():
            j = np.where(cand)[0][al][np.argmin(lat[cand][al])]
            axis = pts[j]-pts[i]; axis[2] = 0.0
            na = np.linalg.norm(axis)
            if na > 1e-6:
                out.append(((pts[i]+pts[j])/2, axis/na, float(t[j])))
    return out


def executable(kin, center, y, width):
    a0 = np.array([0.0, 0.0, -1.0]); lateral = width/2 + FIXED_JAW_CLEAR - TCP_TO_FIXED_JAW
    for tilt in TILTS:
        a = Rotation.from_rotvec(y*math.radians(tilt)).apply(a0)
        R = np.column_stack([a, y, np.cross(a, y)])
        tcp = center + R @ np.array([0.0, lateral, 0.0]); pre = tcp - a*APPROACH_CLEAR
        q = tuple(float(v) for v in Rotation.from_matrix(R).as_quat())
        s1 = kin.ik(tuple(pre), q, SEED, 40)
        if s1 is None:
            continue
        s2 = kin.ik(tuple(tcp), q, s1, 40)
        if s2 is None:
            continue
        if kin.floor_collision(s2, BASE_Z-0.005) or kin.floor_collision(s1, BASE_Z-0.005):
            continue
        return True
    return False


def main():
    kin, hd = build_kin()
    r_ce = np.array(hd.R_cam2gripper, float); t_ce = np.array(hd.t_cam2gripper, float).reshape(3)
    TR = 5
    print(f"노이즈σ1mm + bleed10% + outlier2%, {TR}회 — 실행가능 파지 성공률\n")
    for tx, ty in [(0.24, 0.10), (0.20, 0.00)]:
        for kind in ["box", "cyl_h", "sphere"]:
            center = np.array([tx, ty, BASE_Z + 0.018])
            clean = fused_cloud(kin, kind, center, r_ce, t_ce)
            if clean is None:
                print(f"  ({tx:.2f},{ty:+.2f}) {kind}: 융합 실패"); continue
            succ = 0
            for _ in range(TR):
                cloud = corrupt(clean, center, 0.001, 0.10, 0.02)
                pairs = horizontal_antipodal(cloud)
                ok = False
                for (m, axis, wdt) in pairs[:40]:
                    if executable(kin, m, axis, wdt):
                        ok = True; break
                succ += ok
            print(f"  ({tx:.2f},{ty:+.2f}) {kind:6s}: 성공 {succ}/{TR}")
        print()


if __name__ == "__main__":
    main()
