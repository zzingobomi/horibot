"""프로덕션 파이프라인 sim 검증 (§10.4 구현 후) — 프로토타입이 아니라 **실제
모듈 코드**를 실 캘 kinematics + 물리 렌더 부분 점군으로 end-to-end 돌린다.

경로 (전부 production):
  detector.geometry.object_metrics_from_points (z-gap bottom)
  → tasks.pick_and_place.geometry.view_directions / view_pose_groups (adaptive 뷰)
  → tasks.pick_and_place.antipodal.horizontal_antipodal_pairs (표면 antipodal)
  → tasks.pick_and_place.geometry.plan_grasp (접촉쌍→tilt 가족)
  → motion 게이트 프리미티브: kin.ik / floor_collision / set_obstacle_points /
    obstacle_collision(gripper_open) / _linear_path_blocker (module 의 그 함수)
    + home→pre 관절 보간 충돌 (resolve ④ 등가)

adaptive 루프도 steps 와 동일 의미: 뷰 하나 추가할 때마다 파지 성립 검사, 서면
멈춤. 출력 = 형상×위치별 (파지 성립 여부, 사용 뷰 수, z-gap bottom 오차).
노이즈 케이스: σ1mm + 아래-outlier 3% 주입.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pybullet as p

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "backend"))

from apps.config import load_robots  # noqa: E402
from infra.database.sqlite import open_sqlite  # noqa: E402
from modules.calibration.persistence.repository import CalibrationRepository  # noqa: E402
from modules.detector import geometry as DG  # noqa: E402
from modules.motion.adapters.pybullet import PybulletKinematics  # noqa: E402
from modules.motion.kinematics_builder import build_calibrated_kinematics  # noqa: E402
from modules.motion.module import _linear_path_blocker  # noqa: E402
from modules.motor.contract import MotorKind  # noqa: E402
from modules.tasks.pick_and_place import antipodal as AP  # noqa: E402
from modules.tasks.pick_and_place import geometry as TG  # noqa: E402

BASE_Z = -0.045
W, H_PX, NEAR, FAR, FOV_V = 320, 200, 0.02, 0.6, 58.0
HOME = [-1.07, 2.26, -1.82, 0.81, 0.61, 0.64]  # 실물 시연 자세 (경유/seed)
MAX_VIEWS = 6  # steps._VIEW_MAX_REACHED 동치
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


def add_shape(cid, kind, center):
    if kind == "box":
        s = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.018, 0.012, 0.015], physicsClientId=cid)
        return p.createMultiBody(0, s, -1, center, physicsClientId=cid)
    if kind == "cyl_h":
        s = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.013, height=0.05, physicsClientId=cid)
        q = p.getQuaternionFromEuler([0, math.pi / 2, 0])
        return p.createMultiBody(0, s, -1, center, baseOrientation=q, physicsClientId=cid)
    if kind == "sphere":
        s = p.createCollisionShape(p.GEOM_SPHERE, radius=0.016, physicsClientId=cid)
        return p.createMultiBody(0, s, -1, center, physicsClientId=cid)
    if kind == "Lshape":
        s1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.022, 0.010, 0.010], physicsClientId=cid)
        s2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.010, 0.010, 0.018], physicsClientId=cid)
        return p.createMultiBody(0, s1, -1, center, physicsClientId=cid,
            linkMasses=[0], linkCollisionShapeIndices=[s2], linkVisualShapeIndices=[-1],
            linkPositions=[[0.012, 0, 0.020]], linkOrientations=[[0, 0, 0, 1]],
            linkInertialFramePositions=[[0, 0, 0]], linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0], linkJointTypes=[p.JOINT_FIXED], linkJointAxis=[[0, 0, 1]])


def render(kind, center, cam_pos, r_bc):
    """카메라 pose 에서 물체 부분 점군 렌더 (가려진 면 = 점 없음)."""
    cid = p.connect(p.DIRECT)
    try:
        add_shape(cid, kind, center)
        z_c = r_bc[:, 2]; up = -r_bc[:, 1]
        view = p.computeViewMatrix(cam_pos.tolist(), (cam_pos + z_c).tolist(), up.tolist(), physicsClientId=cid)
        proj = p.computeProjectionMatrixFOV(FOV_V, W / H_PX, NEAR, FAR, physicsClientId=cid)
        _, _, _, depth, seg = p.getCameraImage(W, H_PX, view, proj, renderer=p.ER_TINY_RENDERER, physicsClientId=cid)
        depth = np.array(depth).reshape(H_PX, W); seg = np.array(seg).reshape(H_PX, W)
        V = np.array(view).reshape(4, 4, order="F"); P = np.array(proj).reshape(4, 4, order="F")
        inv = np.linalg.inv(P @ V)
        ys, xs = np.where(seg >= 0)
        if xs.size == 0:
            return None
        d = depth[ys, xs]
        clip = np.stack([2.0 * xs / W - 1.0, 1.0 - 2.0 * ys / H_PX, 2.0 * d - 1.0, np.ones_like(d)], axis=1)
        w = clip @ inv.T
        return w[:, :3] / w[:, 3:4]
    finally:
        p.disconnect(cid)


def corrupt(cloud, center, *, sigma=0.001, outlier_frac=0.03):
    """depth 노이즈 σ + 아래-outlier (phantom 재료) 주입 — E/F 케이스 등가."""
    out = cloud + RNG.normal(0, sigma, cloud.shape)
    n = int(len(cloud) * outlier_frac)
    if n:
        ox = center[0] + RNG.normal(0, 0.02, n)
        oy = center[1] + RNG.normal(0, 0.02, n)
        oz = RNG.uniform(-0.30, -0.12, n)
        out = np.vstack([out, np.stack([ox, oy, oz], axis=1)])
    return out


def joint_path_clear(kin, qa, qb, floor_z, *, gripper_open):
    """resolve ④ 등가 — qa→qb 관절 보간의 self/floor/obstacle 충돌 검사."""
    qa, qb = np.asarray(qa), np.asarray(qb)
    n = max(2, int(math.ceil(float(np.max(np.abs(qb - qa))) / math.radians(5.0))))
    for k in range(1, n + 1):
        q = [float(v) for v in qa + (qb - qa) * (k / n)]
        if kin.self_collision(q):
            return False
        if kin.floor_collision(q, floor_z):
            return False
        if kin.obstacle_collision(q, gripper_open=gripper_open):
            return False
    return True


def grasp_stands(kin, cloud, floor_z):
    """production 파지 성립 검사 — antipodal → plan_grasp → resolve 게이트 등가.

    steps.try_plan_grasp 와 같은 의미: 끝점 IK + 바닥 + 그리퍼(벌림)↔점군 충돌
    + home→pre 관절 경로 + pre→grasp 직선. 게이트 통과 첫 후보 label 반환.
    """
    pairs = AP.horizontal_antipodal_pairs(cloud)
    if not pairs:
        return None
    plan = TG.plan_grasp(pairs)
    kin.set_obstacle_points([tuple(pt) for pt in cloud])
    try:
        for c in plan:
            s1 = kin.ik(c.pre, c.quat, HOME, 40)
            if s1 is None:
                continue
            s2 = kin.ik(c.grasp, c.quat, s1, 40)
            if s2 is None:
                continue
            if kin.floor_collision(s1, floor_z) or kin.floor_collision(s2, floor_z):
                continue
            if kin.obstacle_collision(s1, gripper_open=True) or kin.obstacle_collision(
                s2, gripper_open=True
            ):
                continue
            if not joint_path_clear(kin, HOME, s1, floor_z, gripper_open=True):
                continue
            if _linear_path_blocker(
                kin, c.pre, c.quat, c.grasp, c.quat, list(s1), restarts=10
            ) is not None:
                continue
            return c.label
    finally:
        kin.set_obstacle_points(None)
    return None


def adaptive_observe_and_plan(kin, hd, kind, center, *, noisy=False):
    """steps.observe_and_plan_grasp 등가 sim — adaptive 뷰 누적 + 성립 시 정지."""
    r_ce = np.array(hd.R_cam2gripper, float)
    t_ce = np.array(hd.t_cam2gripper, float).reshape(3)
    floor_z = BASE_Z - 0.005
    clouds: list[np.ndarray] = []
    reached = 0
    for radius, elev, az in TG.view_directions(tuple(center)):
        if reached >= MAX_VIEWS:
            break
        # 뷰 스크리닝 — production view_pose_groups + IK/floor/obstacle (resolve 등가)
        chosen = None
        obstacle = np.vstack(clouds) if clouds else None
        kin.set_obstacle_points(
            [tuple(pt) for pt in obstacle] if obstacle is not None else None
        )
        try:
            for pos, quat in TG.view_pose_groups(
                tuple(center), r_ce, t_ce,
                radius_m=radius, elev_deg=elev, az_rad=az,
            ):
                sol = kin.ik(pos, quat, HOME, 20)
                if sol is None:
                    continue
                if kin.floor_collision(sol, floor_z):
                    continue
                if obstacle is not None and kin.obstacle_collision(sol):
                    continue
                if not joint_path_clear(kin, HOME, sol, floor_z, gripper_open=False):
                    continue
                chosen = sol
                break
        finally:
            kin.set_obstacle_points(None)
        if chosen is None:
            continue
        reached += 1
        # 채택 뷰의 실 카메라 pose (FK + hand_eye) 로 부분 점군 렌더
        r2, t2 = kin.fk_to_matrix(chosen)
        cam_pos = np.array(t2) + np.array(r2) @ t_ce
        r_bc = np.array(r2) @ r_ce
        cl = render(kind, center.tolist(), cam_pos, r_bc)
        if cl is None or len(cl) < 40:
            continue
        if noisy:
            cl = corrupt(cl, center)
        clouds.append(cl)
        fused = np.vstack(clouds)
        label = grasp_stands(kin, fused, floor_z)
        if label is not None:
            m = DG.object_metrics_from_points(fused)
            return reached, label, m
    return reached, None, None


def main():
    kin, hd = build_kin()
    cases = [(0.20, 0.00), (0.24, 0.10), (0.28, 0.18)]
    shapes = ["box", "cyl_h", "sphere", "Lshape"]
    print("프로덕션 파이프라인: adaptive 관측 → antipodal → plan_grasp → 게이트\n")
    ok = tot = 0
    for tx, ty in cases:
        for kind in shapes:
            for noisy in (False, True):
                center = np.array([tx, ty, BASE_Z + 0.018])
                views, label, m = adaptive_observe_and_plan(
                    kin, hd, kind, center, noisy=noisy
                )
                tot += 1
                tag = "노이즈" if noisy else "클린 "
                if label is None:
                    print(f"  ({tx:.2f},{ty:+.2f}) {kind:6s} {tag}: 파지X (뷰 {views})")
                    continue
                ok += 1
                _, bz, h = m
                bz_err = abs(bz - BASE_Z) * 1000
                print(
                    f"  ({tx:.2f},{ty:+.2f}) {kind:6s} {tag}: 파지O 뷰={views} "
                    f"bottom오차={bz_err:.1f}mm h={h * 100:.1f}cm [{label}]"
                )
    print(f"\n총 {ok}/{tot} 파지 성립 (z-gap bottom 은 노이즈+outlier 케이스 포함)")


if __name__ == "__main__":
    main()
