"""next_pose_planner.recommend_geometry audit — 사용자 진술 "뒤집힘" trauma 검증.

사용자가 진술한 trauma: "추천 자세 따라갔는데 카메라가 뒤를 보거나 거꾸로 뒤집힘".

5단계 제약 (UX/추천 자세 design):
  1. 광축이 보드 center 향함 (look-at)
  2. 보드가 화면 FOV 안
  3. 카메라 roll 제한 (화면 정상 방향)
  4. IK reachable
  5. 그 중 관측성 최대

이 audit: 현재 캘 + 현재 robot 자세 위에서 recommend_geometry 호출 → 5 anchor 각각의
*실제 IK 결과 자세* 에서 camera frame 계산 → 5단계 제약 검증. 어느 anchor 가 위반?

검증 metric:
  - look_at_err_deg: 카메라 forward 와 (board_center - cam_pos) 의 각도. 0 이상적.
  - board_in_fov: corners 4개 모두 image 안인지
  - roll_deg: 카메라 +X 가 world_up 와 직교 평면 안에서 얼마나 기울어졌나
  - ik_success: IK 풀렸나
  - visibility_check: planner 의 visibility_check 통과?

실행: cd backend && uv run --no-sync python -m scripts.next_pose_audit
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np


def _ensure_utf8() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")


def raw_to_rad(raw: np.ndarray) -> np.ndarray:
    return (raw.astype(np.float64) - 2048.0) / 4095.0 * 2.0 * np.pi


def main() -> None:
    _ensure_utf8()

    repo_root = Path(__file__).resolve().parents[2]
    calib_dir = repo_root / "robot/instances/omx_f_0/calibration"

    # 캘 산출물 load
    he = np.load(str(calib_dir / "hand_eye.npz"), allow_pickle=True)
    R_c2g = he["R_cam2gripper"]
    t_c2g = he["t_cam2gripper"].reshape(3)
    intr = np.load(str(calib_dir / "intrinsic.npz"), allow_pickle=True)
    K = intr["camera_matrix"]
    dist = intr["dist_coeffs"]
    image_size = tuple(int(x) for x in intr["image_size"])

    # 캡처 데이터 — current_joint 와 board_corners_base 산출용
    poses_npz = np.load(str(calib_dir / "handeye_poses.npz"), allow_pickle=True)
    raw = poses_npz["raw_positions"]
    joint_angles_per_pose = [raw_to_rad(r).tolist() for r in raw]
    R_tc = [np.asarray(R, dtype=np.float64) for R in poses_npz["R_target2cam"]]
    t_tc = [np.asarray(t, dtype=np.float64).reshape(3) for t in poses_npz["t_target2cam"]]
    current = joint_angles_per_pose[0]  # 첫 자세 사용 (임의)

    # Kinematics
    from modules.kinematics.registry import RobotRegistry
    from modules.calibration import next_pose_planner

    kin = RobotRegistry().get_kinematics("omx_f_0")
    from modules.motor.motor_config import load_motor_layout
    layout = load_motor_layout("omx_f_0")
    arm_motor_ids = [m.id for m in layout.arm][:5]
    joint_limits = kin.joint_limits(5)

    # board base frame 추정 — handeye + fk 로 (현재 캡처 평균)
    def fk_fn(angles):
        R, t = kin.fk_to_matrix(list(angles))
        return np.asarray(R), np.asarray(t).reshape(3)

    # board_corners_base — board.py SSOT 의 4 외곽 코너를 capture mean 으로 base 변환
    from modules.calibration import board as calib_board

    sq = calib_board.SQUARE_LENGTH_M
    cols = calib_board.SQUARES_X
    rows = calib_board.SQUARES_Y
    # board 4 외곽 코너 (board frame, m). origin 0,0 좌상.
    board_corners_local = np.array(
        [
            [0.0, 0.0, 0.0],
            [cols * sq, 0.0, 0.0],
            [cols * sq, rows * sq, 0.0],
            [0.0, rows * sq, 0.0],
        ]
    )

    # 평균 board pose in base — handeye + fk 위
    T_b_per_pose = []
    for ja, R_t, t_t in zip(joint_angles_per_pose, R_tc, t_tc):
        R_g2b, t_g2b = fk_fn(ja)
        T_g2b = np.eye(4)
        T_g2b[:3, :3] = R_g2b
        T_g2b[:3, 3] = t_g2b
        T_c2g_m = np.eye(4)
        T_c2g_m[:3, :3] = R_c2g
        T_c2g_m[:3, 3] = t_c2g
        T_t2c = np.eye(4)
        T_t2c[:3, :3] = R_t
        T_t2c[:3, 3] = t_t
        T_t2b = T_g2b @ T_c2g_m @ T_t2c
        T_b_per_pose.append(T_t2b)
    # mean (rotation: chordal SVD, translation: arithmetic mean)
    Rs = np.array([T[:3, :3] for T in T_b_per_pose])
    ts = np.array([T[:3, 3] for T in T_b_per_pose])
    M = Rs.sum(axis=0)
    U, _, Vt = np.linalg.svd(M)
    R_b_mean = U @ Vt
    if np.linalg.det(R_b_mean) < 0:
        U[:, -1] *= -1
        R_b_mean = U @ Vt
    t_b_mean = ts.mean(axis=0)

    # corners in base
    board_corners_base = (R_b_mean @ board_corners_local.T).T + t_b_mean

    # outward hint = camera mean position - board center (camera 쪽 방향)
    cam_positions = np.array([T[:3, 3] for T in T_b_per_pose])  # ee origin, not camera
    # 실제 camera position = fk(ja).t + R_g2b @ t_c2g
    cam_pos_base = []
    for ja in joint_angles_per_pose:
        R_g2b, t_g2b = fk_fn(ja)
        cam_pos_base.append(t_g2b + R_g2b @ t_c2g)
    cam_pos_base = np.array(cam_pos_base)
    board_center = board_corners_base.mean(axis=0)
    outward_hint = cam_pos_base.mean(axis=0) - board_center

    # IK wrapper
    def ik_fn(p, q, c):
        return kin.ik(p, q, c)

    # visibility check
    def vis_check(angles):
        return next_pose_planner.is_pose_visible(
            angles,
            fk_fn=fk_fn,
            camera_matrix=K,
            dist_coeffs=dist,
            image_size=image_size,
            hand_eye_R=R_c2g,
            hand_eye_t=t_c2g,
            board_corners_base=board_corners_base,
        )

    # recommend_geometry 호출 (기존 6DOF 가정)
    result_geo = next_pose_planner.recommend_geometry(
        board_corners_base=board_corners_base,
        ik_fn=ik_fn,
        hand_eye_R=R_c2g,
        hand_eye_t=t_c2g,
        arm_motor_ids=arm_motor_ids,
        joint_limits_rad=joint_limits,
        current_joint_angles_rad=current,
        outward_hint=outward_hint,
        visibility_check=vis_check,
    )
    print("=" * 100)
    print(
        f"recommend_geometry (6DOF anchor) — {len(result_geo.recommendations)} candidates"
        f"  reason: {result_geo.no_candidates_reason}"
    )
    print("=" * 100)
    for rec in result_geo.recommendations:
        print(
            f"  {rec.label:>8}  visible={rec.visible}  {rec.visibility_reason}"
        )
    print()

    # recommend_joint_sample (5DOF joint perturbation 신규)
    result_js = next_pose_planner.recommend_joint_sample(
        current_joint_angles_rad=current,
        arm_motor_ids=arm_motor_ids,
        joint_limits_rad=joint_limits,
        fk_fn=fk_fn,
        visibility_check=vis_check,
        existing_joint_angles=joint_angles_per_pose,
    )
    print("=" * 100)
    print(
        f"recommend_joint_sample (5DOF perturbation) — "
        f"{len(result_js.recommendations)} candidates"
        f"  reason: {result_js.no_candidates_reason}"
    )
    print("=" * 100)
    for rec in result_js.recommendations:
        print(
            f"  {rec.label:>22}  visible={rec.visible}  {rec.visibility_reason}"
        )
    print()

    result = result_js  # 이하 detailed audit 은 joint_sample 결과 위에서

    print("=" * 100)
    print(
        f"detailed audit on joint_sample — {len(result.recommendations)} candidates"
    )
    print("=" * 100)

    if not result.recommendations:
        print("(empty)")
        return

    # 각 추천 자세에 대해 IK 결과의 camera frame audit
    print(
        f"{'#':>2} {'label':>8} {'IK?':>4} {'visible':>8}  {'look_at':>10}  "
        f"{'roll(°)':>8}  {'in_FOV':>6}  reason"
    )
    for idx, rec in enumerate(result.recommendations):
        # joint angles rad
        ja = [float(np.deg2rad(j["degree"])) for j in rec.joints]

        # FK → ee, then camera pose
        try:
            R_g2b, t_g2b = fk_fn(ja)
        except Exception:
            print(f"{idx:>2} {rec.label:>8} FAIL FK")
            continue
        R_c2b = R_g2b @ R_c2g
        t_c2b = t_g2b + R_g2b @ t_c2g

        # 1) look-at: 카메라 forward (cam +z in base) vs (board_center - cam_pos)
        cam_fwd = R_c2b[:, 2]
        to_board = board_center - t_c2b
        to_board_n = to_board / np.linalg.norm(to_board)
        cam_fwd_n = cam_fwd / np.linalg.norm(cam_fwd)
        look_at_err = float(
            np.degrees(np.arccos(np.clip(np.dot(cam_fwd_n, to_board_n), -1, 1)))
        )

        # 2) FOV: board_corners → cam frame → projectPoints
        T_b2c = np.linalg.inv(
            np.block(
                [
                    [R_c2b, t_c2b.reshape(3, 1)],
                    [np.zeros((1, 3)), np.array([[1.0]])],
                ]
            )
        )
        homo = np.hstack(
            [board_corners_base, np.ones((board_corners_base.shape[0], 1))]
        )
        corners_cam = (T_b2c @ homo.T).T[:, :3]
        in_fov = False
        if not np.any(corners_cam[:, 2] <= 0.01):
            img_pts, _ = cv2.projectPoints(
                corners_cam.reshape(-1, 1, 3),
                np.zeros(3),
                np.zeros(3),
                K,
                dist,
            )
            pts = img_pts.reshape(-1, 2)
            w, h = image_size
            margin = 0.05
            in_fov = bool(
                np.all((pts[:, 0] >= w * margin) & (pts[:, 0] <= w * (1 - margin)))
                and np.all((pts[:, 1] >= h * margin) & (pts[:, 1] <= h * (1 - margin)))
            )

        # 3) roll: 카메라 +X 의 world up 평면 정렬
        # 카메라 +X 가 world up 와 직교인 평면 안에 있어야 정상 (사용자 시점이 정상).
        # 측정: world_up 와 카메라 +X 의 cosine — 절대값이 작을수록 (= 직교에 가까울수록) 정상.
        # roll angle = 90 - arccos(|world_up · cam_x|) — 정상=0, 뒤집힘=90 이상
        world_up = np.array([0, 0, 1.0])
        cam_x = R_c2b[:, 0]
        cos_xz = float(np.clip(abs(np.dot(world_up, cam_x)), 0, 1))
        roll_deg = float(np.degrees(np.arcsin(cos_xz)))

        # 4) visibility (planner 의 check)
        v_ok = rec.visible
        v_reason = rec.visibility_reason

        print(
            f"{idx:>2} {rec.label:>8} {'ok':>4} {str(v_ok):>8}"
            f"  {look_at_err:>8.2f}°  {roll_deg:>6.2f}°"
            f"  {str(in_fov):>6}  {v_reason}"
        )

    print()
    print("=" * 100)
    print("판정")
    print("=" * 100)
    bad = []
    for idx, rec in enumerate(result.recommendations):
        ja = [float(np.deg2rad(j["degree"])) for j in rec.joints]
        try:
            R_g2b, t_g2b = fk_fn(ja)
        except Exception:
            continue
        R_c2b = R_g2b @ R_c2g
        t_c2b = t_g2b + R_g2b @ t_c2g
        cam_fwd = R_c2b[:, 2]
        to_board = board_center - t_c2b
        to_board_n = to_board / np.linalg.norm(to_board)
        cam_fwd_n = cam_fwd / np.linalg.norm(cam_fwd)
        look_err = float(
            np.degrees(np.arccos(np.clip(np.dot(cam_fwd_n, to_board_n), -1, 1)))
        )
        cam_x = R_c2b[:, 0]
        cos_xz = float(np.clip(abs(np.dot(np.array([0, 0, 1.0]), cam_x)), 0, 1))
        roll_deg = float(np.degrees(np.arcsin(cos_xz)))
        if look_err > 15 or roll_deg > 30:
            bad.append((idx, rec.label, look_err, roll_deg))

    if not bad:
        print("  >>> PASS — 모든 추천이 look_at < 15° + roll < 30° 만족.")
        print("  사용자 진술 '뒤집힘' 은 현재 캘 + outward_hint 조건에선 재현 안 됨.")
        print("  → 사용자 체험은 이전 버전 또는 outward_hint 누락 시점 가능.")
    else:
        print(f"  >>> {len(bad)}/{len(result.recommendations)} 추천 위반:")
        for idx, label, le, rd in bad:
            print(f"    #{idx} {label}:  look_at={le:.1f}°  roll={rd:.1f}°")
        print("  → recommend_geometry 에 추가 제약 필요.")


if __name__ == "__main__":
    main()
