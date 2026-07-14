"""남은 sim 검증 2/2: clutter — 타깃 주변 이웃 물체(가림 + 접근 충돌).

이웃이 (a) 타깃을 물리적으로 가려 점군이 줄고 (b) 파지 접근이 이웃과 충돌할 수 있다.
render 는 전 물체 포함(타깃 seg 픽셀만 점군화 → 이웃이 물리 가림) + 파지 실행의
그리퍼-물체 충돌은 **타깃+이웃 전부** 대상. 타깃이 여전히 파지 가능+이웃 충돌 없나.
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
W, H_PX, NEAR, FAR, FOV_V = 320, 200, 0.02, 0.6, 58.0
MAX_W, MIN_W = 0.035, 0.004
ANG_TOL, LAT_TOL = math.radians(25), 0.005
SEED = [-1.07, 2.26, -1.82, 0.81, 0.61, 0.64]
TCP_TO_FIXED_JAW, FIXED_JAW_CLEAR, APPROACH_CLEAR = 0.0079, 0.005, 0.06
TILTS = (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90)
HORIZ_TOL = math.radians(20)
GRIPPER_OPEN_RAD = 1.5
TGT_HALF = [0.012, 0.012, 0.015]


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


def spawn_scene(cid, tgt_center, neighbors):
    """타깃(id 반환) + 이웃들 생성. 이웃 = (offset, half)."""
    ts = p.createCollisionShape(p.GEOM_BOX, halfExtents=TGT_HALF, physicsClientId=cid)
    tv = p.createVisualShape(p.GEOM_BOX, halfExtents=TGT_HALF, physicsClientId=cid)
    tid = p.createMultiBody(0, ts, tv, tgt_center, physicsClientId=cid)
    ids = [tid]
    for off, half in neighbors:
        c = [tgt_center[0]+off[0], tgt_center[1]+off[1], BASE_Z+half[2]]
        s = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=cid)
        v = p.createVisualShape(p.GEOM_BOX, halfExtents=half, physicsClientId=cid)
        ids.append(p.createMultiBody(0, s, v, c, physicsClientId=cid))
    return tid, ids


def reachable_cam(kin, tgt, r_ce, t_ce, radius, elev, az):
    tgt = np.array(tgt); el, a = math.radians(elev), math.radians(az)
    up = np.array([math.cos(a)*math.cos(el), math.sin(a)*math.cos(el), math.sin(el)])
    cam_pos = tgt + radius*up
    if cam_pos[2] < BASE_Z + 0.02:
        return None
    z_c = -up
    tmp = np.array([0.0, 0.0, 1.0]) if abs(z_c[2]) <= 0.95 else np.array([1.0, 0.0, 0.0])
    x0 = np.cross(tmp, z_c); x0 /= np.linalg.norm(x0); y0 = np.cross(z_c, x0)
    for roll in range(0, 360, 60):
        rr = math.radians(roll)
        xc = math.cos(rr)*x0 + math.sin(rr)*y0
        r_bc = np.column_stack([xc, np.cross(z_c, xc), z_c])
        r_be = r_bc @ r_ce.T; t_be = cam_pos - r_be @ t_ce
        q = tuple(float(v) for v in Rotation.from_matrix(r_be).as_quat())
        sol = kin.ik((float(t_be[0]), float(t_be[1]), float(t_be[2])), q, SEED, 20)
        if sol is not None:
            r2, t2 = kin.fk_to_matrix(sol)
            return np.array(t2) + np.array(r2) @ t_ce, np.array(r2) @ r_ce
    return None


def render_target(tgt_center, neighbors, cam_pos, r_bc):
    """전 물체 렌더, 타깃 seg 픽셀만 점군화 (이웃이 물리 가림)."""
    cid = p.connect(p.DIRECT)
    try:
        tid, _ = spawn_scene(cid, tgt_center, neighbors)
        z_c = r_bc[:, 2]; up = -r_bc[:, 1]
        view = p.computeViewMatrix(cam_pos.tolist(), (cam_pos+z_c).tolist(), up.tolist(), physicsClientId=cid)
        proj = p.computeProjectionMatrixFOV(FOV_V, W/H_PX, NEAR, FAR, physicsClientId=cid)
        _, _, _, depth, seg = p.getCameraImage(W, H_PX, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=cid)
        depth = np.array(depth).reshape(H_PX, W); seg = np.array(seg).reshape(H_PX, W)
        V = np.array(view).reshape(4, 4, order="F"); P = np.array(proj).reshape(4, 4, order="F")
        inv = np.linalg.inv(P @ V)
        ys, xs = np.where(seg == tid)  # 타깃 픽셀만
        if xs.size == 0:
            return None
        d = depth[ys, xs]
        clip = np.stack([2.0*xs/W-1.0, 1.0-2.0*ys/H_PX, 2.0*d-1.0, np.ones_like(d)], axis=1)
        w = clip @ inv.T
        return w[:, :3] / w[:, 3:4]
    finally:
        p.disconnect(cid)


def horiz_antipodal(cloud):
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(cloud)
    pc = pc.voxel_down_sample(0.003)
    if len(pc.points) < 10:
        return []
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.012, max_nn=30))
    pts = np.asarray(pc.points); nrm = np.asarray(pc.normals)
    c = pts.mean(axis=0); nrm[np.sum((pts-c)*nrm, axis=1) < 0] *= -1
    out = []
    for i in range(0, len(pts), max(1, len(pts)//300)):
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
            ax = pts[j]-pts[i]; ax[2] = 0.0; na = np.linalg.norm(ax)
            if na > 1e-6:
                out.append(((pts[i]+pts[j])/2, ax/na, float(t[j])))
    return out


def gripper_idx(kin):
    inner = kin._inner if hasattr(kin, "_inner") else kin
    return [i for i in inner._movable_indices if i not in inner._chain_indices]


def hits_scene(kin, q, tgt_center, neighbors):
    inner = kin._inner if hasattr(kin, "_inner") else kin
    cid = inner._client; robot = inner._robot
    _, ids = spawn_scene(cid, tgt_center, neighbors)
    try:
        inner._set_chain(list(q))
        for gi in gripper_idx(kin):
            p.resetJointState(robot, gi, GRIPPER_OPEN_RAD, physicsClientId=cid)
        p.performCollisionDetection(physicsClientId=cid)
        for ob in ids:
            if any(c[8] < -0.003 for c in p.getClosestPoints(robot, ob, 0.02, physicsClientId=cid)):
                return True
        return False
    finally:
        for ob in ids:
            p.removeBody(ob, physicsClientId=cid)


def executable(kin, center, y, width, tgt_center, neighbors):
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
        if hits_scene(kin, s2, tgt_center, neighbors) or hits_scene(kin, s1, tgt_center, neighbors):
            continue
        return True, tilt
    return False, None


def main():
    kin, hd = build_kin()
    r_ce = np.array(hd.R_cam2gripper, float); t_ce = np.array(hd.t_cam2gripper, float).reshape(3)
    # 이웃 배치 3종: 한쪽 / 양옆 / 빽빽
    layouts = {
        "이웃 한쪽(3.5cm)": [((0.035, 0.0), [0.012, 0.012, 0.02])],
        "양옆(±3.5cm)": [((0.035, 0.0), [0.012, 0.012, 0.02]), ((-0.035, 0.0), [0.012, 0.012, 0.02])],
        "빽빽(3면 2.8cm)": [((0.028, 0.0), [0.010, 0.012, 0.025]), ((-0.028, 0.0), [0.010, 0.012, 0.025]), ((0.0, 0.030), [0.012, 0.010, 0.025])],
    }
    tgt = np.array([0.24, 0.05, BASE_Z + TGT_HALF[2]])
    print("clutter: 타깃 점군수(단독 대비) → antipodal → 이웃충돌 배제 파지 가능?\n")
    # 기준: 단독
    solo = []
    for radius in (0.13, 0.16):
        for elev in (40, 55, 70):
            for az in range(0, 360, 40):
                rc = reachable_cam(kin, tgt, r_ce, t_ce, radius, elev, az)
                if rc is None:
                    continue
                cl = render_target(tgt.tolist(), [], rc[0], rc[1])
                if cl is not None and len(cl) > 40:
                    solo.append(cl)
                if len(solo) >= 6:
                    break
            if len(solo) >= 6:
                break
        if len(solo) >= 6:
            break
    solo_pts = len(np.vstack(solo)) if solo else 0

    for name, nb in layouts.items():
        views = []
        for radius in (0.13, 0.16):
            for elev in (40, 55, 70):
                for az in range(0, 360, 40):
                    rc = reachable_cam(kin, tgt, r_ce, t_ce, radius, elev, az)
                    if rc is None:
                        continue
                    cl = render_target(tgt.tolist(), nb, rc[0], rc[1])
                    if cl is not None and len(cl) > 20:
                        views.append(cl)
                    if len(views) >= 7:
                        break
                if len(views) >= 7:
                    break
            if len(views) >= 7:
                break
        if not views:
            print(f"  {name}: 타깃 뷰 0 (완전 가림)"); continue
        fused = np.vstack(views)
        pairs = horiz_antipodal(fused)
        ok = False; tl = None
        for (m, ax, wd) in pairs[:40]:
            okk, t = executable(kin, m, ax, wd, tgt.tolist(), nb)
            if okk:
                ok, tl = True, t; break
        print(f"  {name}: 타깃점군 {len(fused)}(단독{solo_pts}) antipodal {len(pairs)}쌍 "
              f"→ {'파지O tilt='+str(tl) if ok else '파지X(이웃충돌/가림)'}")


if __name__ == "__main__":
    main()
