"""end-to-end: 관측→antipodal→그리퍼 pose→IK+충돌. "집으러 갈 수 있나" 최종 검증.

닿는 뷰 융합 점군에서 antipodal 쌍(조 축 수평=SO-101 옆파지 가능 자세) 찾고, 각 쌍을
SO-101 단일조 그리퍼 TCP pose 로 변환(Phase-1 상수 그대로), 접근 tilt 스윕하며 pre+grasp
IK + 바닥충돌 검사 → 실행 가능한 파지가 하나라도 나오나. 형상·위치별.
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
# Phase-1 그리퍼 상수 (geometry.py)
TCP_TO_FIXED_JAW, FIXED_JAW_CLEAR, APPROACH_CLEAR, FINGER_TABLE_CLEAR = 0.0079, 0.005, 0.06, 0.008
TILTS = (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90)
HORIZ_TOL = math.radians(20)  # 조 축 수평 허용 (옆파지)


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


def horizontal_antipodal(cloud):
    """조 축 수평인 antipodal 쌍 (center, jaw_axis(수평단위), width) 리스트."""
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(cloud)
    pc = pc.voxel_down_sample(0.003)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    pts = np.asarray(pc.points); nrm = np.asarray(pc.normals)
    c = pts.mean(axis=0); nrm[np.sum((pts-c)*nrm, axis=1) < 0] *= -1
    out = []
    for i in range(0, len(pts), max(1, len(pts)//400)):
        d = -nrm[i]
        if abs(d[2]) > math.sin(HORIZ_TOL):  # 접근선(=조 축 방향) 수평 아니면 skip
            continue
        rel = pts - pts[i]; t = rel @ d
        lat = np.linalg.norm(rel - np.outer(t, d), axis=1)
        cand = (t > MIN_W) & (t < MAX_W) & (lat < LAT_TOL)
        if not cand.any():
            continue
        al = (nrm[cand] @ d) > math.cos(ANG_TOL)
        if al.any():
            j = np.where(cand)[0][al][np.argmin(lat[cand][al])]
            axis = pts[j] - pts[i]; axis[2] = 0.0
            nrm_ax = np.linalg.norm(axis)
            if nrm_ax < 1e-6:
                continue
            out.append(((pts[i]+pts[j])/2, axis/nrm_ax, float(t[j])))
    return out


def executable(kin, center, jaw_axis, width):
    """조 축 수평 antipodal → tilt 스윕 그리퍼 pose → pre+grasp IK + 바닥충돌.
    실행 가능하면 True."""
    y = jaw_axis  # tool y = 조 축
    a0 = np.array([0.0, 0.0, -1.0])  # 기본 접근 = 수직 하강
    lateral = width/2 + FIXED_JAW_CLEAR - TCP_TO_FIXED_JAW
    for tilt in TILTS:
        a = Rotation.from_rotvec(y * math.radians(tilt)).apply(a0)  # 조 축 둘레 tilt
        x = a; z = np.cross(x, y)
        R = np.column_stack([x, y, z])
        tcp_grasp = center + R @ np.array([0.0, lateral, 0.0])
        pre = tcp_grasp - a * APPROACH_CLEAR
        q = tuple(float(v) for v in Rotation.from_matrix(R).as_quat())
        s1 = kin.ik(tuple(pre), q, SEED, 40)
        if s1 is None:
            continue
        s2 = kin.ik(tuple(tcp_grasp), q, s1, 40)
        if s2 is None:
            continue
        if kin.floor_collision(s2, BASE_Z - 0.005) or kin.floor_collision(s1, BASE_Z - 0.005):
            continue
        return True, tilt
    return False, None


def main():
    kin, hd = build_kin()
    r_ce = np.array(hd.R_cam2gripper, float); t_ce = np.array(hd.t_cam2gripper, float).reshape(3)
    print("관측→antipodal→그리퍼 IK+충돌: 실행가능한 파지 나오나\n")
    for tx, ty in [(0.24, 0.10), (0.20, 0.00), (0.27, 0.18)]:
        for kind in ["box", "cyl_h", "sphere"]:
            center = np.array([tx, ty, BASE_Z + 0.018])
            cloud = fused_cloud(kin, kind, center, r_ce, t_ce)
            if cloud is None:
                print(f"  ({tx:.2f},{ty:+.2f}) {kind:6s}: 융합 실패"); continue
            pairs = horizontal_antipodal(cloud)
            found, tilt_used, tried = False, None, 0
            for (m, axis, wdt) in pairs:
                tried += 1
                ok, tl = executable(kin, m, axis, wdt)
                if ok:
                    found, tilt_used = True, tl
                    break
                if tried >= 40:
                    break
            msg = (f"실행가능 (tilt={tilt_used:+d}°)" if found
                   else f"실행 불가 ({len(pairs)}쌍 중 {tried} 시도 전멸)")
            print(f"  ({tx:.2f},{ty:+.2f}) {kind:6s}: 수평쌍 {len(pairs):3d}개 → {msg}")
        print()


if __name__ == "__main__":
    main()
