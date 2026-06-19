"""Hand-Eye 캘 현재 데이터 직접 진단.

사용자 trauma — σ_rot 1.27° / outlier 후보 8/10 (>20%). 추측 / 하드웨어 핑계 X.
실 데이터로만 root cause 압축.

진단 항목:
  1. cv2 5 method (TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS) 의 X estimate 비교
     → method 간 spread 작으면 데이터 깨끗, 크면 데이터 내 inconsistency
  2. per-pose AX-XB motion equation residual (진짜 hand-eye SSOT)
     → 자세별 residual 분포. 평균 vs 일부 자세 튀는지
  3. LOOCV — 자세 하나씩 빼고 X estimate 변화
     → 그 자세가 진짜 outlier 면 빠지면 σ 큰 폭으로 떨어짐. 아니면 거의 안 변함
  4. 절대 임계 OUTLIER_ABS_ROT_DEG=1.5° false positive 여부
     → 자세별 residual 분포 vs 임계 직접 비교
  5. relative motion 분석 — 인접 자세 pair 자리 rotation/translation 크기
     → 모션 spread 부족 = Tsai-Lenz observability 부족

직접 출력. 추측 줄 X. backend/uv env 자리 실행.
"""

from __future__ import annotations

import json
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation


REPO_ROOT = Path(__file__).resolve().parents[2]
DB_PATH = REPO_ROOT / "backend" / "storage" / "horibot.db"
URDF_PATH = REPO_ROOT / "robot" / "so101_6dof" / "urdf" / "so101_6dof.urdf"

OUTLIER_ABS_ROT_DEG = 1.5
OUTLIER_ABS_T_MM = 15.0


def fetch_data(run_id: int) -> tuple[list, dict, str]:
    """captures + intrinsic + robot_id."""
    con = sqlite3.connect(str(DB_PATH))
    cur = con.cursor()
    cur.execute(
        "SELECT robot_id FROM calibration_runs WHERE id=?", (run_id,)
    )
    row = cur.fetchone()
    if not row:
        sys.exit(f"run_id={run_id} 없음")
    robot_id = row[0]

    cur.execute(
        "SELECT pose_index, joint_angles, board_in_cam, residual_rot, residual_trans, weight "
        "FROM calibration_captures WHERE run_id=? ORDER BY pose_index",
        (run_id,),
    )
    captures = cur.fetchall()

    cur.execute(
        "SELECT result_data FROM calibration_results "
        "WHERE robot_id=? AND kind='intrinsic' AND is_active=1",
        (robot_id,),
    )
    intr = json.loads(cur.fetchone()[0])
    con.close()
    return captures, intr, robot_id


def setup_pybullet() -> tuple[int, int, list[int], int]:
    cid = p.connect(p.DIRECT)
    rid = p.loadURDF(str(URDF_PATH), useFixedBase=True, physicsClientId=cid)
    n = p.getNumJoints(rid, physicsClientId=cid)
    tcp_idx = -1
    revolute = []
    for i in range(n):
        info = p.getJointInfo(rid, i, physicsClientId=cid)
        if info[12].decode() == "tcp":
            tcp_idx = i
        if info[2] == p.JOINT_REVOLUTE:
            revolute.append(i)
    return cid, rid, revolute, tcp_idx


def fk(cid: int, rid: int, revolute: list[int], tcp_idx: int, ja: list[float]) -> np.ndarray:
    """joint_angles (rad) → T_gripper2base (4x4)."""
    for jidx, a in zip(revolute, ja):
        p.resetJointState(rid, jidx, a, physicsClientId=cid)
    state = p.getLinkState(rid, tcp_idx, computeForwardKinematics=True, physicsClientId=cid)
    T = np.eye(4)
    T[:3, :3] = np.array(p.getMatrixFromQuaternion(state[5])).reshape(3, 3)
    T[:3, 3] = np.array(state[4])
    return T


def ax_xb_residual(
    T_g2b_a: np.ndarray,
    T_g2b_b: np.ndarray,
    T_t2c_a: np.ndarray,
    T_t2c_b: np.ndarray,
    X: np.ndarray,
) -> tuple[float, float]:
    """AX - XB residual (rotation deg, translation mm).

    A = T_g2b_b^-1 @ T_g2b_a (gripper relative motion, base frame)
    B = T_t2c_a @ T_t2c_b^-1 (camera-to-target relative motion)
    X = T_c2g (hand-eye)
    AX = XB 자리 should hold. residual = AX - XB.
    """
    A = np.linalg.inv(T_g2b_b) @ T_g2b_a
    B = T_t2c_a @ np.linalg.inv(T_t2c_b)
    AX = A @ X
    XB = X @ B
    dR = AX[:3, :3] @ XB[:3, :3].T
    angle = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
    dt = AX[:3, 3] - XB[:3, 3]
    return float(np.degrees(angle)), float(np.linalg.norm(dt) * 1000)


def calibrate(R_g2b, t_g2b, R_t2c, t_t2c, method) -> np.ndarray:
    R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
    X = np.eye(4)
    X[:3, :3] = R
    X[:3, 3] = t.flatten()
    return X


def per_pose_residual(
    R_g2b, t_g2b, R_t2c, t_t2c, X
) -> list[tuple[float, float]]:
    """모든 pair (i, j=i+1) 의 AX-XB residual. 자세 i 가 책임."""
    N = len(R_g2b)
    results = []
    for i in range(N):
        # 자세 i 에 대해 *모든 다른 자세* 와 pair 평균
        rots, ts = [], []
        for j in range(N):
            if i == j:
                continue
            T_g2b_i = np.eye(4); T_g2b_i[:3, :3] = R_g2b[i]; T_g2b_i[:3, 3] = t_g2b[i]
            T_g2b_j = np.eye(4); T_g2b_j[:3, :3] = R_g2b[j]; T_g2b_j[:3, 3] = t_g2b[j]
            T_t2c_i = np.eye(4); T_t2c_i[:3, :3] = R_t2c[i]; T_t2c_i[:3, 3] = t_t2c[i]
            T_t2c_j = np.eye(4); T_t2c_j[:3, :3] = R_t2c[j]; T_t2c_j[:3, 3] = t_t2c[j]
            r, tt = ax_xb_residual(T_g2b_i, T_g2b_j, T_t2c_i, T_t2c_j, X)
            rots.append(r)
            ts.append(tt)
        results.append((float(np.median(rots)), float(np.median(ts))))
    return results


def x_summary(X: np.ndarray) -> str:
    rv = Rotation.from_matrix(X[:3, :3]).as_rotvec()
    rv_norm = np.linalg.norm(rv) * 180 / np.pi
    rv_axis = rv / max(np.linalg.norm(rv), 1e-9)
    t_mm = X[:3, 3] * 1000
    return (
        f"axis=[{rv_axis[0]:+.3f},{rv_axis[1]:+.3f},{rv_axis[2]:+.3f}] "
        f"angle={rv_norm:.2f}° | t=[{t_mm[0]:+.1f},{t_mm[1]:+.1f},{t_mm[2]:+.1f}]mm"
    )


def main(run_id: int = 2) -> None:
    captures, intr, robot_id = fetch_data(run_id)
    print(f"== Run {run_id} ({robot_id}), {len(captures)} 장 ==")
    K = np.array(intr["camera_matrix"])
    D = np.array(intr["dist_coeffs"])
    print(f"intrinsic K[0,0]={K[0,0]:.1f} K[1,1]={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    print(f"distortion = {D.flatten()}")

    cid, rid, revolute, tcp_idx = setup_pybullet()

    pis, R_g2b, t_g2b, R_t2c, t_t2c, joint_lists = [], [], [], [], [], []
    for pi, ja_s, bic_s, *_ in captures:
        if bic_s is None:
            continue
        ja = json.loads(ja_s)
        T_g2b = fk(cid, rid, revolute, tcp_idx, ja)
        T_t2c = np.array(json.loads(bic_s))
        pis.append(pi)
        R_g2b.append(T_g2b[:3, :3])
        t_g2b.append(T_g2b[:3, 3])
        R_t2c.append(T_t2c[:3, :3])
        t_t2c.append(T_t2c[:3, 3])
        joint_lists.append(ja)
    N = len(pis)
    print(f"검증 가능 captures = {N} 장 (pose_index: {pis})")
    print()

    # ─── 1. cv2 5 method 비교 ───
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    print("=== 1. cv2 5 method X estimate 비교 ===")
    Xs = {}
    for name, m in methods.items():
        try:
            X = calibrate(R_g2b, t_g2b, R_t2c, t_t2c, m)
            Xs[name] = X
            print(f"  {name:<12} {x_summary(X)}")
        except Exception as e:
            print(f"  {name:<12} 실패: {e}")

    # spread
    if "TSAI" in Xs and len(Xs) > 1:
        base = Xs["TSAI"]
        print("  ─ TSAI 기준 spread ─")
        for name, X in Xs.items():
            if name == "TSAI":
                continue
            dR = base[:3, :3].T @ X[:3, :3]
            ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
            dt_mm = np.linalg.norm(base[:3, 3] - X[:3, 3]) * 1000
            print(f"  TSAI-{name:<10} Δrot={ang:6.3f}° Δt={dt_mm:6.2f}mm")
    print()

    # ─── 2. per-pose AX-XB residual (TSAI X) ───
    print("=== 2. per-pose AX-XB residual (TSAI X 기준, median over pair) ===")
    if "TSAI" in Xs:
        res = per_pose_residual(R_g2b, t_g2b, R_t2c, t_t2c, Xs["TSAI"])
        print(f"  {'pose':>4} {'rot(°)':>9} {'trans(mm)':>11}  {'임계 (1.5°/15mm) hit':<20}")
        rots = np.array([r[0] for r in res])
        tts = np.array([r[1] for r in res])
        for i, pi in enumerate(pis):
            r, t = res[i]
            hit = []
            if r > OUTLIER_ABS_ROT_DEG: hit.append(f"ROT({r:.2f}>{OUTLIER_ABS_ROT_DEG})")
            if t > OUTLIER_ABS_T_MM: hit.append(f"T({t:.1f}>{OUTLIER_ABS_T_MM})")
            print(f"  {pi:>4} {r:>9.3f} {t:>11.2f}  {','.join(hit) if hit else '깨끗':<20}")
        print(f"  ── median rot={np.median(rots):.3f}° trans={np.median(tts):.2f}mm")
        print(f"  ── mean   rot={rots.mean():.3f}° trans={tts.mean():.2f}mm")
        print(f"  ── std    rot={rots.std():.3f}° trans={tts.std():.2f}mm")
        n_hit = sum(1 for r, t in res if r > OUTLIER_ABS_ROT_DEG or t > OUTLIER_ABS_T_MM)
        print(f"  ── 임계 hit: {n_hit}/{N} ({100*n_hit/N:.0f}%)")
    print()

    # ─── 3. LOOCV — 자세 하나씩 빼고 X 변화 ───
    print("=== 3. LOOCV (자세 i 빼고 TSAI X 추정 → 그 X 의 cv2 cost) ===")
    if N >= 5:
        full_X = Xs.get("TSAI")
        if full_X is not None:
            print(f"  {'빠진_pose':>10} {'X_axis_Δ(°)':>13} {'X_t_Δ(mm)':>11}")
            loo_changes = []
            for i in range(N):
                R_g2b_loo = [R_g2b[j] for j in range(N) if j != i]
                t_g2b_loo = [t_g2b[j] for j in range(N) if j != i]
                R_t2c_loo = [R_t2c[j] for j in range(N) if j != i]
                t_t2c_loo = [t_t2c[j] for j in range(N) if j != i]
                X_loo = calibrate(R_g2b_loo, t_g2b_loo, R_t2c_loo, t_t2c_loo, cv2.CALIB_HAND_EYE_TSAI)
                dR = full_X[:3, :3].T @ X_loo[:3, :3]
                ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
                dt_mm = np.linalg.norm(full_X[:3, 3] - X_loo[:3, 3]) * 1000
                loo_changes.append((pis[i], ang, dt_mm))
            loo_changes.sort(key=lambda x: -x[1])
            for pi, ang, dt_mm in loo_changes:
                print(f"  {pi:>10} {ang:>13.3f} {dt_mm:>11.2f}")
            print("  → ΔX 큰 자세 = 그 자세 빠지면 estimate 크게 변함 = 진짜 outlier 또는 정보 핵심")
    print()

    # ─── 4. Joint angles 분포 ───
    print("=== 4. Joint angles 분포 (deg, std/range) ===")
    ja_arr = np.degrees(np.array(joint_lists))
    print(f"  {'axis':>4} {'std':>8} {'min':>8} {'max':>8} {'range':>8}")
    for i in range(ja_arr.shape[1]):
        print(f"  J{i+1:>3} {ja_arr[:,i].std():>8.1f} {ja_arr[:,i].min():>8.1f} "
              f"{ja_arr[:,i].max():>8.1f} {(ja_arr[:,i].max()-ja_arr[:,i].min()):>8.1f}")
    print()

    # ─── 5. Relative motion 분포 ───
    print("=== 5. Relative motion 분포 (인접 자세 pair) ===")
    motions = []
    for i in range(N - 1):
        T_a = np.eye(4); T_a[:3,:3] = R_g2b[i]; T_a[:3,3] = t_g2b[i]
        T_b = np.eye(4); T_b[:3,:3] = R_g2b[i+1]; T_b[:3,3] = t_g2b[i+1]
        T_rel = np.linalg.inv(T_a) @ T_b
        dR = T_rel[:3, :3]
        ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
        dt = np.linalg.norm(T_rel[:3, 3]) * 1000
        motions.append((pis[i], pis[i+1], ang, dt))
    print(f"  {'pose_a':>6} {'pose_b':>6} {'rot(°)':>8} {'trans(mm)':>11}")
    for pa, pb, ang, dt in motions:
        print(f"  {pa:>6} {pb:>6} {ang:>8.2f} {dt:>11.1f}")
    rots_m = np.array([m[2] for m in motions])
    ts_m = np.array([m[3] for m in motions])
    print(f"  ── relative rotation:    mean={rots_m.mean():.1f}° std={rots_m.std():.1f}° min={rots_m.min():.1f}° max={rots_m.max():.1f}°")
    print(f"  ── relative translation: mean={ts_m.mean():.0f}mm std={ts_m.std():.0f}mm min={ts_m.min():.0f}mm max={ts_m.max():.0f}mm")

    p.disconnect(cid)
    print()
    print("=== 6. Iterative greedy outlier removal (best subset 탐색) ===")
    print("  매 step LOOCV → ΔX 가장 큰 자세 제거 → cv2 재계산 → σ 추적")
    print("  목표: 자세 모두 keep 가능한 N (>=8) 자리에서 σ 최소 set 발견")
    current_indices = list(range(N))
    current_pis = list(pis)
    R_g2b_w, t_g2b_w = list(R_g2b), list(t_g2b)
    R_t2c_w, t_t2c_w = list(R_t2c), list(t_t2c)

    def cv2_method_spread(R_g, t_g, R_t, t_t) -> tuple[float, float]:
        """5 method X estimate 중 TSAI 대비 max rotation/translation spread."""
        if len(R_g) < 3:
            return float("inf"), float("inf")
        try:
            X_tsai = calibrate(R_g, t_g, R_t, t_t, cv2.CALIB_HAND_EYE_TSAI)
        except Exception:
            return float("inf"), float("inf")
        max_rot, max_t = 0.0, 0.0
        for m in (cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_HORAUD, cv2.CALIB_HAND_EYE_DANIILIDIS):
            try:
                X = calibrate(R_g, t_g, R_t, t_t, m)
                dR = X_tsai[:3, :3].T @ X[:3, :3]
                ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
                dt = np.linalg.norm(X_tsai[:3, 3] - X[:3, 3]) * 1000
                max_rot = max(max_rot, ang)
                max_t = max(max_t, dt)
            except Exception:
                continue
        return max_rot, max_t

    def base_frame_board_std(R_g, t_g, R_t, t_t, X) -> tuple[float, float]:
        """모든 자세 자리 T_target2base 자리 같아야 — std (mm, °)."""
        T_c2g = np.eye(4); T_c2g[:3, :3] = X[:3, :3]; T_c2g[:3, 3] = X[:3, 3]
        positions, rotvecs = [], []
        for i in range(len(R_g)):
            T_g2b = np.eye(4); T_g2b[:3,:3] = R_g[i]; T_g2b[:3,3] = t_g[i]
            T_t2c_i = np.eye(4); T_t2c_i[:3,:3] = R_t[i]; T_t2c_i[:3,3] = t_t[i]
            T_t2b = T_g2b @ T_c2g @ T_t2c_i
            positions.append(T_t2b[:3, 3])
            rotvecs.append(Rotation.from_matrix(T_t2b[:3, :3]).as_rotvec())
        positions = np.array(positions)
        rotvecs = np.array(rotvecs)
        std_pos = np.linalg.norm(positions - positions.mean(axis=0), axis=1).std() * 1000
        # rotation std via mean rotvec distance
        mean_rv = rotvecs.mean(axis=0)
        rot_dists = []
        for rv in rotvecs:
            dR = Rotation.from_rotvec(rv) * Rotation.from_rotvec(mean_rv).inv()
            rot_dists.append(np.linalg.norm(dR.as_rotvec()) * 180 / np.pi)
        std_rot = float(np.std(rot_dists))
        return std_pos, std_rot

    print(f"\n  {'step':>4} {'N':>3} {'TSAI-PARK Δrot':>15} {'TSAI-DANI Δrot':>15} "
          f"{'target2base std_pos(mm)':>22} {'std_rot(°)':>11} {'제거된 pose':>15}")
    history = []
    removed = []
    while len(current_indices) >= 5:
        # cv2 method spread
        max_rot_spread, _ = cv2_method_spread(R_g2b_w, t_g2b_w, R_t2c_w, t_t2c_w)
        X_tsai = calibrate(R_g2b_w, t_g2b_w, R_t2c_w, t_t2c_w, cv2.CALIB_HAND_EYE_TSAI)
        X_dani = calibrate(R_g2b_w, t_g2b_w, R_t2c_w, t_t2c_w, cv2.CALIB_HAND_EYE_DANIILIDIS)
        dR = X_tsai[:3,:3].T @ X_dani[:3,:3]
        ang_dani = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
        # base frame board std (TSAI X)
        std_pos, std_rot = base_frame_board_std(R_g2b_w, t_g2b_w, R_t2c_w, t_t2c_w, X_tsai)

        last_removed = f"#{removed[-1]}" if removed else "-"
        print(f"  {len(history):>4} {len(current_indices):>3} {max_rot_spread:>15.3f} "
              f"{ang_dani:>15.3f} {std_pos:>22.2f} {std_rot:>11.3f} {last_removed:>15}")
        history.append((len(current_indices), max_rot_spread, ang_dani, std_pos, std_rot, removed.copy()))

        if len(current_indices) <= 8:
            break  # MIN_POSES_FOR_COMPUTE

        # LOOCV 자리 max ΔX 자세 찾기
        worst_loo_i = -1
        worst_dX = -1.0
        for i in range(len(current_indices)):
            R_g_loo = [R_g2b_w[j] for j in range(len(current_indices)) if j != i]
            t_g_loo = [t_g2b_w[j] for j in range(len(current_indices)) if j != i]
            R_t_loo = [R_t2c_w[j] for j in range(len(current_indices)) if j != i]
            t_t_loo = [t_t2c_w[j] for j in range(len(current_indices)) if j != i]
            X_loo = calibrate(R_g_loo, t_g_loo, R_t_loo, t_t_loo, cv2.CALIB_HAND_EYE_TSAI)
            dR = X_tsai[:3, :3].T @ X_loo[:3, :3]
            dang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
            if dang > worst_dX:
                worst_dX = dang
                worst_loo_i = i

        # remove
        removed_pi = current_pis[worst_loo_i]
        removed.append(removed_pi)
        del current_indices[worst_loo_i]
        del current_pis[worst_loo_i]
        del R_g2b_w[worst_loo_i]
        del t_g2b_w[worst_loo_i]
        del R_t2c_w[worst_loo_i]
        del t_t2c_w[worst_loo_i]

    # 최적 step 찾기 (std_pos + std_rot 최소)
    print(f"\n  ── 자세 제거 순서 (큰 outlier 부터): {removed}")
    best = min(history, key=lambda h: h[3] + h[4] * 10)  # weight: 1mm ≈ 0.1° rot
    best_N, _, _, best_std_pos, best_std_rot, best_removed = best
    kept = [p for p in pis if p not in best_removed]
    print(f"\n  ── 최적 subset (target2base std 최소): N={best_N}")
    print(f"     keep pose_index: {kept}")
    print(f"     제거된 pose_index: {best_removed}")
    print(f"     std_pos={best_std_pos:.2f}mm, std_rot={best_std_rot:.3f}°")

    p.disconnect(cid)
    print()
    print("=== 진단 요약 ===")
    print(f"  - 전체 {N} 자세 → 최적 subset N={best_N}")
    print(f"  - 제거 추천: pose_index {best_removed}")
    print(f"  - 최적 std_pos={best_std_pos:.2f}mm, std_rot={best_std_rot:.3f}°")


if __name__ == "__main__":
    run_id = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    main(run_id)
