"""남은 sim 검증 1/2: 관측 뷰 간 이동 경로 충돌 + 충분성 정지 기준(몇 뷰면 파지 가능).

- 정지 기준: 뷰를 하나씩 누적하며 매번 antipodal+실행 파지가 서는지 → 처음 서는 뷰 수 N.
  (고정 7뷰가 과한지, 2~3뷰면 되는지 데이터로.)
- 이동 경로: 실제 방문 뷰 관절해들 사이 joint-space 보간(MoveJ)이 self+바닥+물체 충돌
  없이 지나가나 (각 뷰 도달만이 아니라 그 사이 동선까지).
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


def add_shape(cid, kind, center):
    if kind == "box":
        s = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.018, 0.012, 0.015], physicsClientId=cid)
    elif kind == "sphere":
        s = p.createCollisionShape(p.GEOM_SPHERE, radius=0.016, physicsClientId=cid)
    elif kind == "Lshape":
        s1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.022, 0.010, 0.010], physicsClientId=cid)
        s2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.010, 0.010, 0.018], physicsClientId=cid)
        return p.createMultiBody(0, s1, -1, center, physicsClientId=cid,
            linkMasses=[0], linkCollisionShapeIndices=[s2], linkVisualShapeIndices=[-1],
            linkPositions=[[0.012, 0, 0.020]], linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0]], linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0], linkJointTypes=[p.JOINT_FIXED], linkJointAxis=[[0, 0, 1]])
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
    for roll in range(0, 360, 60):
        rr = math.radians(roll)
        xc = math.cos(rr)*x0 + math.sin(rr)*y0
        r_bc = np.column_stack([xc, np.cross(z_c, xc), z_c])
        r_be = r_bc @ r_ce.T; t_be = cam_pos - r_be @ t_ce
        q = tuple(float(v) for v in Rotation.from_matrix(r_be).as_quat())
        sol = kin.ik((float(t_be[0]), float(t_be[1]), float(t_be[2])), q, SEED, 20)
        if sol is not None:
            r2, t2 = kin.fk_to_matrix(sol)
            return (np.array(t2) + np.array(r2) @ t_ce, np.array(r2) @ r_ce, list(sol))
    return None


def render(kind, center, cam_pos, r_bc):
    cid = p.connect(p.DIRECT)
    try:
        add_shape(cid, kind, center)
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


def robot_obj_penetrate(kin, q, bodies_fn, gripper_open=True):
    """관절 q 에서 로봇↔물체(bodies_fn 이 kin 씬에 물체 생성) 침투(>3mm)?"""
    inner = kin._inner if hasattr(kin, "_inner") else kin
    cid = inner._client; robot = inner._robot
    objs = bodies_fn(cid)
    try:
        inner._set_chain(list(q))
        if gripper_open:
            for gi in gripper_idx(kin):
                p.resetJointState(robot, gi, GRIPPER_OPEN_RAD, physicsClientId=cid)
        p.performCollisionDetection(physicsClientId=cid)
        for ob in objs:
            if any(c[8] < -0.003 for c in p.getClosestPoints(robot, ob, 0.02, physicsClientId=cid)):
                return True
        return False
    finally:
        for ob in objs:
            p.removeBody(ob, physicsClientId=cid)


def executable(kin, center, y, width, bodies_fn):
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
        if robot_obj_penetrate(kin, s2, bodies_fn):
            continue
        return True
    return False


def path_reason(kin, qa, qb, bodies_fn, n=16):
    """qa→qb joint 보간 충돌 원인 ('' = 없음 / 'self' / 'floor' / 'obj')."""
    qa, qb = np.array(qa), np.array(qb)
    for k in range(1, n+1):
        q = list(qa + (qb-qa)*k/n)
        if kin.self_collision(q):
            return "self"
        if kin.floor_collision(q, BASE_Z-0.005):
            return "floor"
        if k % 4 == 0 and robot_obj_penetrate(kin, q, bodies_fn, gripper_open=True):
            return "obj"
    return ""


def seg_ok(kin, qa, qb, bodies_fn):
    """직접 qa→qb, 막히면 qa→SEED→qb (retract 경유) 재시도. (ok, 방법, 원인)."""
    r = path_reason(kin, qa, qb, bodies_fn)
    if not r:
        return True, "직접", ""
    r1 = path_reason(kin, qa, SEED, bodies_fn)
    r2 = path_reason(kin, SEED, qb, bodies_fn)
    if not r1 and not r2:
        return True, "SEED경유", r
    return False, "실패", f"{r}(retract도 {r1 or 'ok'}/{r2 or 'ok'})"


def main():
    kin, hd = build_kin()
    r_ce = np.array(hd.R_cam2gripper, float); t_ce = np.array(hd.t_cam2gripper, float).reshape(3)
    cases = [(0.24, 0.05), (0.20, 0.00), (0.28, 0.18)]
    shapes = ["box", "sphere", "Lshape"]
    print("정지기준(파지 서는 최소 뷰수) / 이동경로 충돌 없나 (SEED→뷰들 순회)\n")
    for tx, ty in cases:
        for kind in shapes:
            center = np.array([tx, ty, BASE_Z + 0.018])
            bodies_fn = lambda cid, k=kind, c=center: [add_shape(cid, k, list(c))]
            # 뷰 수집 (관절해 포함)
            views = []
            for radius in (0.13, 0.16):
                for elev in (40, 55, 70):
                    for az in range(0, 360, 40):
                        rc = reachable_cam(kin, center, r_ce, t_ce, radius, elev, az)
                        if rc is None:
                            continue
                        cl = render(kind, center.tolist(), rc[0], rc[1])
                        if cl is not None and len(cl) > 40:
                            views.append((cl, rc[2]))
                        if len(views) >= 7:
                            break
                    if len(views) >= 7:
                        break
                if len(views) >= 7:
                    break
            if not views:
                print(f"  ({tx:.2f},{ty:+.2f}) {kind}: 뷰 0"); continue
            # 정지 기준: 하나씩 누적하며 파지 서는 최소 뷰
            n_stop = None
            for k in range(1, len(views)+1):
                fused = np.vstack([v[0] for v in views[:k]])
                pairs = horiz_antipodal(fused)
                if any(executable(kin, m, ax, wd, bodies_fn) for (m, ax, wd) in pairs[:25]):
                    n_stop = k; break
            # 이동 경로: SEED → 방문 뷰들 순회, 직접/SEED경유 완화 포함
            nseg = n_stop or len(views)
            direct_bad = retract_saved = still_bad = 0
            reasons = []
            prev = SEED
            for (_, qv) in views[:nseg]:
                ok, how, why = seg_ok(kin, prev, qv, bodies_fn)
                if how == "직접":
                    pass
                elif how == "SEED경유":
                    direct_bad += 1; retract_saved += 1; reasons.append(why)
                else:
                    direct_bad += 1; still_bad += 1; reasons.append(why)
                prev = qv
            rtxt = f" 원인={','.join(reasons)}" if reasons else ""
            print(f"  ({tx:.2f},{ty:+.2f}) {kind:6s}: 최소뷰={nseg}  "
                  f"직접충돌 {direct_bad}/{nseg} → retract로 해결 {retract_saved}, 잔여 {still_bad}{rtxt}")
        print()


if __name__ == "__main__":
    main()
