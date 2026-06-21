"""Offline hand-eye 캘리브레이션 분석 + commit.

새 capture-only 시나리오의 짝궁 — frontend 가 raw blob + record 만 저장, 본 스크립트가
storage 직접 (SQLite + filesystem ObjectStore) 읽어 다단계 BA + sanity check + commit.

backend storage_node 자리 떠 있으면 RDB lock 충돌 가능 — backend 종료 후 실행 권장.

5 stage BA — 각 stage 가 추가 자유도 unlock:
  A: hand-eye R, t + per-pose target pose (board in base frame)
  B: A + joint zero offsets (n_arm DOF)
  C: B + link offsets (Δxyz + Δrpy, 6 × n_arm DOF)
  D: C + sag stiffness (sag joint 별 k_rad_per_m)
  E: D + depth 3D residual (per corner depth triangulation)

Sanity check (수치 자리에만 매몰되지 않기 자리) — 매 stage 의 추정값을 *물리적으로
합리적인지* 검증. RED FLAG 자리 = 모델 fit 됐지만 파라미터가 비현실적 → 다른 자리 자리
원인. 예: |joint_offset| > 5° = servo zero 가 5° 빗나가지 않음. 다른 모델 오차 흡수 의심.

LOOCV: leave-one-out cross-validation. 25 자세 자리 24장 BA 후 빠진 1장 predict
잔차. train 잔차 자리 크게 차이 자리 = overfit.

CLI:
  uv run python scripts/calibrate_offline.py --robot so101_6dof_0
  uv run python scripts/calibrate_offline.py --robot so101_6dof_0 --run-id 2
  uv run python scripts/calibrate_offline.py ... --stage E
  uv run python scripts/calibrate_offline.py ... --skip-depth
  uv run python scripts/calibrate_offline.py ... --commit
"""

from __future__ import annotations

import argparse
import json
import logging
import sqlite3
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
from scipy.stats import median_abs_deviation

# Repo imports (script standalone).
BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))

from core.robot.robot_registry import RobotRegistry  # noqa: E402
from core.units import raw_to_rad  # noqa: E402
from modules.calibration import board as calib_board  # noqa: E402
from modules.camera import depth_frame as dframe  # noqa: E402
from modules.motor.motor_config import MotorConfig, load_motor_layout  # noqa: E402

logger = logging.getLogger(__name__)


# ─── Sanity check thresholds (root cause 의심 자리 임계값) ───────
#
# 산업 robot 의 "올바른 calibration" 기준이 아니라 SO-101 + D405 같은 DIY 자리의
# *경고* 임계. 넘으면 다른 모델 오차 흡수 의심.

# Hand-eye translation 크기 (m) — 손목 마운트 카메라 자리 reasonable 5-20cm.
HE_T_MAX_OK = 0.20  # YELLOW above
HE_T_MAX_RED = 0.40  # RED above

# Joint zero offset (deg) — DIY robot 자리 manual home 자리 자리 자리 자리 ~5-10°. 진단 결과
# strict prior(1°) 에서도 9-10° 자리 fit → 진짜 servo zero err 자리 보임.
JOINT_OFF_OK_DEG = 5.0
JOINT_OFF_RED_DEG = 12.0

# Link offset Δxyz (mm) — 3D-printed (FFF) parts 자리 ±5mm 인쇄 tolerance + 조립 누적.
# 진단 결과 strict prior 에서 ~15mm 자리 fit → 진짜 build tolerance 자리 보임.
LINK_T_OK_MM = 10.0
LINK_T_RED_MM = 25.0

# Link offset Δrpy (deg) — 조립 + 인쇄 warpage.
LINK_R_OK_DEG = 2.0
LINK_R_RED_DEG = 5.0

# Sag stiffness k_rad_per_m — lumped mass 자리 0.01 ~ 0.3.
SAG_K_OK_MAX = 0.30
SAG_K_RED_MAX = 0.60

# Outlier rate (per-pose weight < 0.5).
OUTLIER_RATE_OK = 0.20  # 20% 이하 = OK
OUTLIER_RATE_RED = 0.40  # 40% 초과 = 자리 자리 자리 자리 자리 fit 자리 의심

# LOOCV / train RMS ratio — overfit 지표.
LOOCV_RATIO_OK = 1.5
LOOCV_RATIO_RED = 2.0

# Prior σ (Tikhonov regularization) — overfit 차단 강도. STRICT 자리 박음 — 진단
# 실험 (calibrate_diagnose.py + strict prior test) 결과 자리 link 67mm vs 15mm 자리
# 가른 prior 자리. 1mm/0.2°/1° 자리 link/rot/joint 자리 절대 자리 큰 자리 자리 reject
# (prior cost 자리 dominate). 데이터 자리 진짜 demand 자리 자리 자리 prior 자리 자리 자리.
PRIOR_JOINT_RAD = np.deg2rad(1.0)
PRIOR_LINK_T_M = 0.001
PRIOR_LINK_R_RAD = np.deg2rad(0.2)
PRIOR_SAG = 0.10

# Huber loss threshold (residual scale).
HUBER_2D_PX = 1.5  # px
HUBER_3D_PER_PX_EQUIV = 0.005 / HUBER_2D_PX  # 3D 1mm 자리 px equivalent.


# ─── 데이터 컨테이너 ─────────────────────────────────────────────


@dataclass
class CapturePose:
    """한 캡처의 BA 입력 — DB row + blob 디코드 결과 + 보조 계산."""

    pose_index: int
    # raw motor → rad. JointCoordinates.motor_to_urdf 의 *joint_offset 가산 전* 값 —
    # BA 가 joint_offset 을 변수로 추정.
    joint_angles_rad_raw: np.ndarray  # (n_arm,)
    corners_2d: np.ndarray  # (N, 2) sub-pixel
    corner_ids: np.ndarray  # (N,) int
    # board frame 의 corner 3D 좌표 (m). matchImagePoints 결과.
    board_obj_pts: np.ndarray  # (N, 3)
    # 진단 (BA 외부 — sanity check 자리).
    pnp_reproj_rms_px: float
    tilt_deg: float
    # PnP 결과 — BA seed.
    R_target2cam_seed: np.ndarray  # (3, 3)
    t_target2cam_seed: np.ndarray  # (3,)
    # Depth (optional — Stage E only).
    depth_z16: np.ndarray | None = None  # (H, W) uint16
    depth_scale: float = 0.001


class SanityLevel(Enum):
    OK = "OK"
    YELLOW = "WARN"
    RED = "RED"


@dataclass
class SanityFlag:
    """Sanity check 결과 1건."""

    category: str  # "handeye_t" / "joint_offset" / "link_trans" / ...
    level: SanityLevel
    message: str
    value: float | None = None


@dataclass
class StageResult:
    """한 BA stage 결과 — motor_id 매핑 + σ + sanity 까지."""

    name: str  # "A" / "B" / "C" / "D" / "E"
    estimated: set[str]  # {"handeye", "joint", "link", "sag", "depth"} 부분집합
    # 핵심 추정값.
    handeye_R: np.ndarray  # (3, 3) — cam in EE (gripper) frame
    handeye_t: np.ndarray  # (3,) m
    target_R: np.ndarray  # (3, 3) — board in base frame
    target_t: np.ndarray  # (3,) m
    joint_offsets: dict[int, float] = field(default_factory=dict)  # motor_id → rad
    link_trans: dict[int, np.ndarray] = field(default_factory=dict)  # motor_id → (3,) m
    link_rot: dict[int, np.ndarray] = field(default_factory=dict)  # motor_id → (3,) rotvec
    sag_k: dict[int, float] = field(default_factory=dict)  # motor_id → k_rad_per_m
    # 진단.
    cost: float = 0.0  # final ½||r||²
    converged: bool = False
    n_iters: int = 0
    n_residuals: int = 0
    reproj_rms_px: float = float("inf")
    # σ 두 종류 (project_calibration_sigma_dual_metric):
    #   sigma_handeye_*           = BA Jacobian σ (parameter confidence — (JᵀJ)⁻¹·σ²)
    #   effective_sigma_handeye_* = board_in_base std (accuracy). commit 결정 metric.
    sigma_handeye_rot_deg: float = float("inf")  # from BA covariance
    sigma_handeye_t_mm: float = float("inf")
    sigma_target_rot_deg: float = float("inf")
    sigma_target_t_mm: float = float("inf")
    effective_sigma_handeye_rot_deg: float = float("inf")
    effective_sigma_handeye_t_mm: float = float("inf")
    per_pose_rms_px: list[float] = field(default_factory=list)
    per_pose_weight: list[float] = field(default_factory=list)
    n_outliers: int = 0  # weight < 0.5
    loocv_rms_px: float = float("inf")
    sanity: list[SanityFlag] = field(default_factory=list)

    @property
    def worst_sanity(self) -> SanityLevel:
        if any(f.level == SanityLevel.RED for f in self.sanity):
            return SanityLevel.RED
        if any(f.level == SanityLevel.YELLOW for f in self.sanity):
            return SanityLevel.YELLOW
        return SanityLevel.OK


# ─── BA config ──────────────────────────────────────────────────


@dataclass
class BAConfig:
    estimate_joint: bool = False
    estimate_link: bool = False
    estimate_sag: bool = False
    use_depth: bool = False


# ─── 데이터 loading ──────────────────────────────────────────────


def load_data(
    db_path: Path,
    blob_root: Path,
    robot_id: str,
    run_id: int | None,
    *,
    load_depth: bool,
) -> tuple[dict, list[CapturePose], dict, list[MotorConfig]]:
    """SQLite + filesystem 에서 run + captures + intrinsic + arm_cfgs 로드.

    Returns:
        (run_dict, captures, intrinsic_dict, arm_cfgs)
    """
    if not db_path.exists():
        raise FileNotFoundError(f"DB 없음: {db_path}")
    con = sqlite3.connect(str(db_path))
    try:
        # 1. Run 선택.
        cur = con.cursor()
        if run_id is None:
            row = cur.execute(
                "SELECT id FROM calibration_runs "
                "WHERE robot_id=? AND kind='hand_eye' "
                "AND status IN ('ready_for_analysis','in_progress') "
                "ORDER BY started_at DESC LIMIT 1",
                (robot_id,),
            ).fetchone()
            if row is None:
                raise RuntimeError(
                    f"분석 대상 run 없음 (robot={robot_id}, "
                    "ready_for_analysis hand_eye run 부재)"
                )
            run_id = int(row[0])

        run_row = cur.execute(
            "SELECT id, robot_id, started_at, ended_at, algorithm, "
            "algorithm_params, status, kind FROM calibration_runs WHERE id=?",
            (run_id,),
        ).fetchone()
        if run_row is None:
            raise RuntimeError(f"run id={run_id} 없음")
        if run_row[1] != robot_id:
            raise RuntimeError(
                f"run id={run_id} 의 robot_id={run_row[1]!r} ≠ {robot_id!r}"
            )
        if run_row[7] != "hand_eye":
            raise RuntimeError(
                f"run id={run_id} 의 kind={run_row[7]!r} — hand_eye 만 지원"
            )
        run = {
            "id": run_row[0],
            "robot_id": run_row[1],
            "started_at": run_row[2],
            "ended_at": run_row[3],
            "algorithm": run_row[4],
            "algorithm_params": json.loads(run_row[5] or "{}"),
            "status": run_row[6],
            "kind": run_row[7],
        }

        # 2. Intrinsic — session snapshot 우선, 없으면 active result.
        snap = run["algorithm_params"].get("intrinsic_snapshot")
        if isinstance(snap, dict):
            intrinsic = {
                "camera_matrix": np.array(snap["camera_matrix"], dtype=np.float64),
                "dist_coeffs": np.array(snap["dist_coeffs"], dtype=np.float64),
                "image_size": tuple(snap.get("image_size", (1280, 720))),
                "source": "session_snapshot",
            }
        else:
            intr_row = cur.execute(
                "SELECT result_data FROM calibration_results "
                "WHERE robot_id=? AND kind='intrinsic' AND is_active=1",
                (robot_id,),
            ).fetchone()
            if intr_row is None:
                raise RuntimeError("active intrinsic 없음 — 캘 불가")
            d = json.loads(intr_row[0])
            intrinsic = {
                "camera_matrix": np.array(d["camera_matrix"], dtype=np.float64),
                "dist_coeffs": np.array(d["dist_coeffs"], dtype=np.float64),
                "image_size": tuple(d.get("image_size", (1280, 720))),
                "source": "active_result",
            }

        # 3. Captures (LEFT JOIN artifact[kind=primary] 자리 blob_key 자리).
        cap_rows = cur.execute(
            "SELECT c.pose_index, c.motor_positions, c.corners_2d, c.corner_ids, "
            "c.board_in_cam, c.reproj_rms_px, c.tilt_deg, a.blob_key "
            "FROM calibration_captures c "
            "LEFT JOIN calibration_capture_artifacts a "
            "  ON a.capture_id = c.id AND a.kind = 'primary' "
            "WHERE c.run_id=? ORDER BY c.pose_index ASC",
            (run_id,),
        ).fetchall()
    finally:
        con.close()

    if len(cap_rows) < 4:
        raise RuntimeError(f"capture 부족: {len(cap_rows)} (최소 4)")

    arm_cfgs = load_motor_layout(robot_id).arm
    K = intrinsic["camera_matrix"]
    dist = intrinsic["dist_coeffs"]

    captures: list[CapturePose] = []
    for (
        pose_index, motor_positions_json, corners_2d_json, corner_ids_json,
        board_in_cam_json, reproj_rms_px, tilt_deg, blob_key,
    ) in cap_rows:
        if not motor_positions_json or not corners_2d_json or not corner_ids_json:
            logger.warning("capture #%d 결손 — skip", pose_index)
            continue
        motor_positions = {
            int(k): int(v) for k, v in json.loads(motor_positions_json).items()
        }
        # raw motor → rad. joint_offset 자리 BA 변수.
        joint_angles_raw = np.array(
            [
                raw_to_rad(motor_positions[c.id], reverse=c.reverse)
                for c in arm_cfgs
            ],
            dtype=np.float64,
        )
        corners_2d = np.asarray(json.loads(corners_2d_json), dtype=np.float64)
        corner_ids = np.asarray(json.loads(corner_ids_json), dtype=np.int32)

        # 보드 frame 3D 코너 — board.matchImagePoints (캐시 corners_2d 는 sub-pixel
        # observed; obj_pts 자리 corner_ids 자리 매핑).
        obj_pts, _ = calib_board.match_object_points(
            corners_2d.reshape(-1, 1, 2).astype(np.float32),
            corner_ids.reshape(-1, 1).astype(np.int32),
        )
        board_obj_pts = obj_pts.reshape(-1, 3).astype(np.float64)

        # PnP seed — cached board_in_cam 자리 있으면 그거 자리, 없으면 fresh PnP.
        if board_in_cam_json:
            T = np.asarray(json.loads(board_in_cam_json), dtype=np.float64)
            R_t2c = T[:3, :3]
            t_t2c = T[:3, 3]
        else:
            ok, rvec, tvec = cv2.solvePnP(
                board_obj_pts.reshape(-1, 1, 3).astype(np.float64),
                corners_2d.reshape(-1, 1, 2).astype(np.float64),
                K, dist,
            )
            if not ok:
                logger.warning("capture #%d PnP 실패 — skip", pose_index)
                continue
            R_t2c, _ = cv2.Rodrigues(rvec)
            t_t2c = np.asarray(tvec).reshape(3)

        # Depth blob (옵션). 손상된 blob (Pydantic Base64Bytes corruption bug 시점 데이터)
        # 은 조용히 skip — Stage E 가 알아서 valid 한 capture 만 사용.
        depth_z16: np.ndarray | None = None
        depth_scale = 0.001
        if load_depth and blob_key:
            blob_path = blob_root / blob_key
            if blob_path.exists():
                try:
                    raw = blob_path.read_bytes()
                    if len(raw) < 10_000:
                        # corruption 의심 — depth_frame 은 raw RGBD 라 보통 200KB 이상.
                        logger.warning(
                            "capture #%d blob %s 가 너무 작음 (%d bytes) — corruption 의심, skip depth",
                            pose_index, blob_key, len(raw),
                        )
                    else:
                        df = dframe.decode(raw)
                        depth_z16 = df.depth_z16
                        depth_scale = df.depth_scale
                except Exception as e:
                    logger.warning(
                        "capture #%d blob decode 실패 (%s): %s — skip depth",
                        pose_index, blob_key, e,
                    )

        captures.append(
            CapturePose(
                pose_index=pose_index,
                joint_angles_rad_raw=joint_angles_raw,
                corners_2d=corners_2d,
                corner_ids=corner_ids,
                board_obj_pts=board_obj_pts,
                pnp_reproj_rms_px=float(reproj_rms_px or 0.0),
                tilt_deg=float(tilt_deg or 0.0),
                R_target2cam_seed=R_t2c,
                t_target2cam_seed=t_t2c,
                depth_z16=depth_z16,
                depth_scale=depth_scale,
            )
        )

    return run, captures, intrinsic, arm_cfgs


# ─── SE(3) 헬퍼 ──────────────────────────────────────────────────


def matrix_to_rvec(R: np.ndarray) -> np.ndarray:
    return Rot.from_matrix(R).as_rotvec()


def rvec_to_matrix(rv: np.ndarray) -> np.ndarray:
    return Rot.from_rotvec(rv).as_matrix()


def average_se3(
    Rs: list[np.ndarray], ts: list[np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    """quaternion 평균 (Markley) + t 평균."""
    qs = np.array([Rot.from_matrix(R).as_quat() for R in Rs])
    for i in range(1, len(qs)):
        if np.dot(qs[0], qs[i]) < 0:
            qs[i] = -qs[i]
    M = qs.T @ qs
    _, eigvecs = np.linalg.eigh(M)
    q_mean = eigvecs[:, -1]
    R_mean = Rot.from_quat(q_mean).as_matrix()
    t_mean = np.mean(np.stack(ts), axis=0)
    return R_mean, t_mean


# ─── BA 변수 packing ────────────────────────────────────────────


def pack_params(
    handeye_R: np.ndarray, handeye_t: np.ndarray,
    target_R: np.ndarray, target_t: np.ndarray,
    joint_off: np.ndarray, link_t: np.ndarray, link_r: np.ndarray,
    sag_k: np.ndarray,
    cfg: BAConfig,
) -> np.ndarray:
    parts = [
        matrix_to_rvec(handeye_R), handeye_t,
        matrix_to_rvec(target_R), target_t,
    ]
    if cfg.estimate_joint:
        parts.append(joint_off)
    if cfg.estimate_link:
        parts.append(link_t.flatten())
        parts.append(link_r.flatten())
    if cfg.estimate_sag:
        parts.append(sag_k)
    return np.concatenate(parts)


def unpack_params(
    x: np.ndarray, n_arm: int, n_sag: int, cfg: BAConfig
) -> tuple[
    np.ndarray, np.ndarray, np.ndarray, np.ndarray,
    np.ndarray, np.ndarray, np.ndarray, np.ndarray,
]:
    i = 0
    handeye_R = rvec_to_matrix(x[i:i + 3])
    i += 3
    handeye_t = x[i:i + 3]
    i += 3
    target_R = rvec_to_matrix(x[i:i + 3])
    i += 3
    target_t = x[i:i + 3]
    i += 3
    if cfg.estimate_joint:
        joint_off = x[i:i + n_arm].copy()
        i += n_arm
    else:
        joint_off = np.zeros(n_arm)
    if cfg.estimate_link:
        link_t = x[i:i + n_arm * 3].reshape(n_arm, 3).copy()
        i += n_arm * 3
        link_r = x[i:i + n_arm * 3].reshape(n_arm, 3).copy()
        i += n_arm * 3
    else:
        link_t = np.zeros((n_arm, 3))
        link_r = np.zeros((n_arm, 3))
    if cfg.estimate_sag:
        sag_k = x[i:i + n_sag].copy()
        i += n_sag
    else:
        sag_k = np.zeros(n_sag)
    return handeye_R, handeye_t, target_R, target_t, joint_off, link_t, link_r, sag_k


# ─── FK + sag ────────────────────────────────────────────────────


def fk_with_sag(
    fk_chain,
    joint_angles: np.ndarray,
    link_trans: np.ndarray | None,
    link_rot: np.ndarray | None,
    sag_k_full: np.ndarray,  # (n_arm,) — non-sag idx 는 0
) -> tuple[np.ndarray, np.ndarray]:
    """FK + sag correction. sag 자리는 joint axis × gravity torque (lumped mass)."""
    if not np.any(sag_k_full):
        # sag 자리 없음 — 단순 FK.
        return fk_chain.fk(joint_angles, link_trans, link_rot)

    # 1차 FK 자리 origin/axis 자리 자리.
    _, t_ee, origins, axes = fk_chain.fk_with_axes(
        joint_angles, link_trans, link_rot
    )
    # τ_i = ((tcp - origin_i) × gravity) · axis_i. gravity = -z (fk_chain spec).
    gravity = np.array([0.0, 0.0, -1.0])
    sag_delta = np.zeros_like(joint_angles)
    for i in range(len(sag_k_full)):
        if abs(sag_k_full[i]) < 1e-12:
            continue
        r = t_ee - origins[i]
        tau = float(np.dot(np.cross(r, gravity), axes[i]))
        sag_delta[i] = sag_k_full[i] * tau

    # sag 보정 적용 후 다시 FK.
    return fk_chain.fk(joint_angles + sag_delta, link_trans, link_rot)


# ─── BA residual ────────────────────────────────────────────────


def compute_residuals(
    x: np.ndarray,
    captures: list[CapturePose],
    fk_chain,
    K: np.ndarray,
    sag_arm_indices: list[int],
    cfg: BAConfig,
    *,
    pose_weights: np.ndarray,
) -> np.ndarray:
    """모든 capture × corner residual stack + prior block.

    pose_weights: (n_captures,) ∈ (0, 1] — IRLS weight. residual 각 row 에 sqrt(w)
    곱 = squared sum 자리 w * r² 자리.
    """
    n_arm = fk_chain.n_arm
    n_sag = len(sag_arm_indices)
    (
        handeye_R, handeye_t, target_R, target_t,
        joint_off, link_t, link_r, sag_k_sparse,
    ) = unpack_params(x, n_arm, n_sag, cfg)

    sag_k_full = np.zeros(n_arm)
    for i, idx in enumerate(sag_arm_indices):
        sag_k_full[idx] = sag_k_sparse[i]

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    residuals: list[np.ndarray] = []
    for cap_i, cap in enumerate(captures):
        w_sqrt = np.sqrt(max(pose_weights[cap_i], 1e-6))

        angles = cap.joint_angles_rad_raw + joint_off
        R_g2b, t_g2b = fk_with_sag(
            fk_chain, angles,
            link_t if cfg.estimate_link else None,
            link_r if cfg.estimate_link else None,
            sag_k_full,
        )

        # T_cam2base = T_g2b @ T_he.
        R_c2b = R_g2b @ handeye_R
        t_c2b = R_g2b @ handeye_t + t_g2b

        # T_target2cam = inv(T_cam2base) @ T_target2base.
        R_b2c = R_c2b.T
        R_t2c = R_b2c @ target_R
        t_t2c = R_b2c @ target_t - R_b2c @ t_c2b

        # 3D in cam → 2D.
        pts_cam = (R_t2c @ cap.board_obj_pts.T).T + t_t2c
        z = pts_cam[:, 2]
        z_safe = np.where(z > 1e-6, z, 1e-6)
        u = fx * pts_cam[:, 0] / z_safe + cx
        v = fy * pts_cam[:, 1] / z_safe + cy
        proj = np.stack([u, v], axis=-1)
        # 2D residual (px) × sqrt(weight).
        resid_2d = ((proj - cap.corners_2d) * w_sqrt).flatten()
        residuals.append(resid_2d)

        # 3D depth residual (optional). 1m 자리 = HUBER_2D_PX / HUBER_3D_PER_PX_EQUIV
        # px 자리 equivalent → scale = HUBER_2D_PX / 0.005 = 300 px/m.
        if cfg.use_depth and cap.depth_z16 is not None:
            scale = 1.0 / HUBER_3D_PER_PX_EQUIV  # m → "px-equivalent"
            d_resid = compute_depth_residual(
                cap, pts_cam, K, scale=scale
            )
            if d_resid.size > 0:
                residuals.append(d_resid * w_sqrt)

    # Tikhonov priors — overfit 차단.
    if cfg.estimate_joint:
        residuals.append(joint_off / PRIOR_JOINT_RAD)
    if cfg.estimate_link:
        residuals.append(link_t.flatten() / PRIOR_LINK_T_M)
        residuals.append(link_r.flatten() / PRIOR_LINK_R_RAD)
    if cfg.estimate_sag:
        residuals.append(sag_k_sparse / PRIOR_SAG)

    return np.concatenate(residuals)


def compute_depth_residual(
    cap: CapturePose,
    pts_cam_pred: np.ndarray,
    K: np.ndarray,
    *,
    scale: float,
) -> np.ndarray:
    """ChArUco corner 의 depth-triangulated 3D 위치 vs BA-predicted 3D 자리.

    depth value 가 0 또는 image 밖이면 그 corner 자리 skip. residual = m × scale.
    """
    H, W = cap.depth_z16.shape  # type: ignore[union-attr]
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    out: list[float] = []
    for i, (u, v) in enumerate(cap.corners_2d):
        u_int, v_int = int(round(u)), int(round(v))
        if not (0 <= u_int < W and 0 <= v_int < H):
            continue
        z_raw = int(cap.depth_z16[v_int, u_int])  # type: ignore[index]
        if z_raw == 0:
            continue
        z = z_raw * cap.depth_scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        p_meas = np.array([x, y, z])
        diff = (p_meas - pts_cam_pred[i]) * scale
        out.extend(diff.tolist())
    return np.array(out, dtype=np.float64) if out else np.zeros(0)


# ─── BA stage runner ────────────────────────────────────────────


def seed_handeye(
    captures: list[CapturePose], fk_chain
) -> tuple[np.ndarray, np.ndarray, str]:
    """cv2.calibrateHandEye 5 method → AX=XB 자리 자리 자리 잔차 자리 best."""
    R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list = [], [], [], []
    for cap in captures:
        R_g, t_g = fk_chain.fk(cap.joint_angles_rad_raw)
        R_g2b_list.append(R_g)
        t_g2b_list.append(t_g.reshape(3, 1))
        R_t2c_list.append(cap.R_target2cam_seed)
        t_t2c_list.append(cap.t_target2cam_seed.reshape(3, 1))

    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    best_name = None
    best_R: np.ndarray | None = None
    best_t: np.ndarray | None = None
    best_residual = float("inf")
    for name, m in methods.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, method=m
            )
            residual = axxb_residual(
                R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, R, t
            )
            logger.info("  seed[%-11s] residual=%.5f", name, residual)
            if residual < best_residual:
                best_residual = residual
                best_name = name
                best_R = R
                best_t = np.asarray(t).reshape(3)
        except cv2.error as e:
            logger.debug("seed[%s] cv2 fail: %s", name, e)
    if best_R is None or best_t is None:
        raise RuntimeError("cv2.calibrateHandEye 5 method 모두 실패")
    return best_R, best_t, best_name or "?"


def _mat4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def axxb_residual(
    R_g2b, t_g2b, R_t2c, t_t2c, R_he, t_he
) -> float:
    n = len(R_g2b)
    he = _mat4(R_he, t_he)
    residuals = []
    for i in range(n):
        for j in range(i + 1, n):
            A_i = _mat4(R_g2b[i], t_g2b[i])
            A_j = _mat4(R_g2b[j], t_g2b[j])
            A = np.linalg.inv(A_j) @ A_i
            B_i = _mat4(R_t2c[i], t_t2c[i])
            B_j = _mat4(R_t2c[j], t_t2c[j])
            B = B_j @ np.linalg.inv(B_i)
            residuals.append(np.linalg.norm(A @ he - he @ B))
    return float(np.mean(residuals)) if residuals else float("inf")


def estimate_target_seed(
    captures, fk_chain, handeye_R, handeye_t
) -> tuple[np.ndarray, np.ndarray]:
    """초기 target_in_base 추정 — 각 capture 의 PnP × handeye × FK → 평균 SE(3)."""
    Rs, ts = [], []
    for cap in captures:
        R_g2b, t_g2b = fk_chain.fk(cap.joint_angles_rad_raw)
        R_c2b = R_g2b @ handeye_R
        t_c2b = R_g2b @ handeye_t + t_g2b
        R_t2b = R_c2b @ cap.R_target2cam_seed
        t_t2b = R_c2b @ cap.t_target2cam_seed + t_c2b
        Rs.append(R_t2b)
        ts.append(t_t2b)
    return average_se3(Rs, ts)


def measure_effective_sigma(
    captures: list[CapturePose],
    fk_chain,
    arm_cfgs: list[MotorConfig],
    result: "StageResult",
) -> tuple[float, float]:
    """BA fit 적용 후 board_in_base 의 std → (σ_R deg, σ_t mm).

    BA Jacobian σ (parameter confidence) 와 다른 metric — 같은 fit 으로 reproject 한
    target pose 들이 *얼마나 한 자리에 모이나* 의 accuracy 지표. commit 결정 기준.
    [[project-calibration-sigma-dual-metric]].
    """
    n_arm = fk_chain.n_arm
    joint_off = np.zeros(n_arm)
    link_t = np.zeros((n_arm, 3))
    link_r = np.zeros((n_arm, 3))
    for i, c in enumerate(arm_cfgs):
        joint_off[i] = result.joint_offsets.get(c.id, 0.0)
        if c.id in result.link_trans:
            link_t[i] = result.link_trans[c.id]
        if c.id in result.link_rot:
            link_r[i] = result.link_rot[c.id]

    origins, rots = [], []
    for cap in captures:
        angles = cap.joint_angles_rad_raw + joint_off
        R_g2b, t_g2b = fk_chain.fk(angles, link_t, link_r)
        R_c2b = R_g2b @ result.handeye_R
        t_c2b = R_g2b @ result.handeye_t + t_g2b
        R_t2b = R_c2b @ cap.R_target2cam_seed
        t_t2b = R_c2b @ cap.t_target2cam_seed + t_c2b
        origins.append(t_t2b)
        rots.append(R_t2b)

    origins_arr = np.array(origins)
    pos_std_mm = float(np.linalg.norm(origins_arr.std(axis=0) * 1000.0))

    qs = np.array([Rot.from_matrix(R).as_quat() for R in rots])
    for i in range(1, len(qs)):
        if np.dot(qs[0], qs[i]) < 0:
            qs[i] = -qs[i]
    M = qs.T @ qs
    _, ev = np.linalg.eigh(M)
    R_mean = Rot.from_quat(ev[:, -1]).as_matrix()
    angs = []
    for R in rots:
        R_rel = R @ R_mean.T
        tr = (np.trace(R_rel) - 1.0) * 0.5
        angs.append(np.arccos(np.clip(tr, -1.0, 1.0)))
    rot_std_deg = float(np.rad2deg(np.std(angs)))
    return rot_std_deg, pos_std_mm


def _per_pose_residual_breakdown(
    r_full: np.ndarray,
    captures: list[CapturePose],
    cfg: BAConfig,
) -> tuple[list[float], list[int]]:
    """residual 벡터 자리 per-pose 2D RMS + per-pose residual count 자리 분리.

    prior block 자리 제외 — caller 가 알면서 자리. 다음 fn 자리는 prior 빼고 호출.
    """
    rms_list: list[float] = []
    count_list: list[int] = []
    idx = 0
    for cap in captures:
        n_corners = len(cap.board_obj_pts)
        n_2d = 2 * n_corners
        block = r_full[idx:idx + n_2d]
        rms_list.append(float(np.sqrt(np.mean(block ** 2))))
        idx += n_2d
        count_list.append(n_2d)
        if cfg.use_depth and cap.depth_z16 is not None:
            H, W = cap.depth_z16.shape
            n_valid_depth = 0
            for u, v in cap.corners_2d:
                u_int, v_int = int(round(u)), int(round(v))
                if 0 <= u_int < W and 0 <= v_int < H:
                    if cap.depth_z16[v_int, u_int] > 0:
                        n_valid_depth += 1
            idx += 3 * n_valid_depth
            count_list[-1] += 3 * n_valid_depth
    return rms_list, count_list


def run_ba_stage(
    captures: list[CapturePose],
    fk_chain,
    K: np.ndarray,
    sag_arm_indices: list[int],
    cfg: BAConfig,
    *,
    name: str,
    seed_handeye_R: np.ndarray,
    seed_handeye_t: np.ndarray,
    arm_cfgs: list[MotorConfig],
    irls_outer: int = 3,
    max_nfev: int = 400,
) -> StageResult:
    """한 stage BA — least_squares + IRLS pose-level Huber re-weighting.

    1. seed handeye + target_in_base 추정 → x0
    2. least_squares (Levenberg-Marquardt + Huber loss row-level)
    3. IRLS outer iter: per-pose RMS → MAD-Huber weight → re-fit with weighted residual
    4. σ 자리 BA covariance (J^T J)^-1 자리 자리 자리 estimate
    """
    n_arm = fk_chain.n_arm
    n_sag = len(sag_arm_indices)

    target_R0, target_t0 = estimate_target_seed(
        captures, fk_chain, seed_handeye_R, seed_handeye_t
    )

    x0 = pack_params(
        seed_handeye_R, seed_handeye_t, target_R0, target_t0,
        np.zeros(n_arm), np.zeros((n_arm, 3)), np.zeros((n_arm, 3)),
        np.zeros(n_sag),
        cfg,
    )

    # IRLS — least_squares 안은 loss="linear" (standard LS), pose-level robustify 는
    # weights 자리 residual 곱셈으로 외부 자리 (적분 다 hujber inside LS 자리 bad
    # seed 자리 gradient clamp 자리 → 수렴 안 됨).
    weights = np.ones(len(captures))

    def residual_fn(x: np.ndarray) -> np.ndarray:
        return compute_residuals(
            x, captures, fk_chain, K, sag_arm_indices, cfg,
            pose_weights=weights,
        )

    x_opt = x0
    result = None
    for outer in range(irls_outer):
        result = least_squares(
            residual_fn, x_opt, method="trf", loss="linear",
            max_nfev=max_nfev, verbose=0,
        )
        x_opt = result.x

        # per-pose RMS → MAD-Huber weight.
        r_unweighted = compute_residuals(
            x_opt, captures, fk_chain, K, sag_arm_indices, cfg,
            pose_weights=np.ones(len(captures)),
        )
        # prior block 자리 길이 — joint(n_arm) + link(6n_arm) + sag(n_sag) — 차감.
        prior_len = 0
        if cfg.estimate_joint:
            prior_len += n_arm
        if cfg.estimate_link:
            prior_len += 6 * n_arm
        if cfg.estimate_sag:
            prior_len += n_sag
        r_data = r_unweighted[:-prior_len] if prior_len else r_unweighted

        rms_per_pose, _ = _per_pose_residual_breakdown(r_data, captures, cfg)
        rms_arr = np.array(rms_per_pose)
        # scale="normal" 자리 = MAD * 1.4826 자리 자리. 수동으로 박음 (typing 자리).
        mad = float(median_abs_deviation(rms_arr) * 1.4826)
        kappa = 1.345 * max(mad, 0.5)  # px floor — capture 가 다 좋으면 weight=1 그대로
        weights = np.minimum(1.0, kappa / np.maximum(rms_arr, 1e-6))
        logger.debug(
            "  IRLS outer=%d κ=%.3fpx median_rms=%.3f down-weight count=%d",
            outer, kappa, float(np.median(rms_arr)),
            int(np.sum(weights < 0.5)),
        )

    assert result is not None

    # 최종 residual + σ estimate.
    r_final_unweighted = compute_residuals(
        x_opt, captures, fk_chain, K, sag_arm_indices, cfg,
        pose_weights=np.ones(len(captures)),
    )
    prior_len = 0
    if cfg.estimate_joint:
        prior_len += n_arm
    if cfg.estimate_link:
        prior_len += 6 * n_arm
    if cfg.estimate_sag:
        prior_len += n_sag
    r_data_final = (
        r_final_unweighted[:-prior_len] if prior_len else r_final_unweighted
    )
    rms_per_pose, _ = _per_pose_residual_breakdown(r_data_final, captures, cfg)
    reproj_rms = float(np.sqrt(np.mean(r_data_final ** 2)))

    # σ from Jacobian (J^T J)^-1 — handeye/target 블록의 sqrt(trace).
    sigma_he_rot_deg = float("inf")
    sigma_he_t_mm = float("inf")
    sigma_target_rot_deg = float("inf")
    sigma_target_t_mm = float("inf")
    try:
        J = result.jac  # (n_residuals, n_params)
        # residual 단위 자리 px 자리 → noise σ^2 ≈ MSE. param Cov = (JᵀJ)⁻¹ × σ².
        n_resid = J.shape[0]
        n_params = J.shape[1]
        dof = max(n_resid - n_params, 1)
        sigma2 = float(np.sum(r_data_final ** 2) / dof)
        JTJ = J.T @ J
        # regularization 자리 condition number 자리 — small ε for numerical stability.
        cov = np.linalg.inv(JTJ + 1e-10 * np.eye(n_params)) * sigma2
        # handeye_R = idx 0-2, handeye_t = idx 3-5, target_R = idx 6-8, target_t = idx 9-11.
        sigma_he_rot_deg = float(
            np.rad2deg(np.sqrt(np.trace(cov[:3, :3])))
        )
        sigma_he_t_mm = float(1000.0 * np.sqrt(np.trace(cov[3:6, 3:6])))
        sigma_target_rot_deg = float(
            np.rad2deg(np.sqrt(np.trace(cov[6:9, 6:9])))
        )
        sigma_target_t_mm = float(1000.0 * np.sqrt(np.trace(cov[9:12, 9:12])))
    except np.linalg.LinAlgError:
        logger.warning("σ 추정 실패 — Jacobian rank-deficient")

    # Unpack final.
    (
        handeye_R, handeye_t, target_R, target_t,
        joint_off, link_t, link_r, sag_sparse,
    ) = unpack_params(x_opt, n_arm, n_sag, cfg)

    # motor_id mapping.
    joint_offsets: dict[int, float] = {}
    link_trans: dict[int, np.ndarray] = {}
    link_rot: dict[int, np.ndarray] = {}
    if cfg.estimate_joint:
        for i, c in enumerate(arm_cfgs):
            joint_offsets[c.id] = float(joint_off[i])
    if cfg.estimate_link:
        for i, c in enumerate(arm_cfgs):
            link_trans[c.id] = link_t[i].copy()
            link_rot[c.id] = link_r[i].copy()
    sag_k: dict[int, float] = {}
    if cfg.estimate_sag:
        for i, arm_idx in enumerate(sag_arm_indices):
            sag_k[arm_cfgs[arm_idx].id] = float(sag_sparse[i])

    stage = StageResult(
        name=name,
        estimated=(
            {"handeye"}
            | ({"joint"} if cfg.estimate_joint else set())
            | ({"link"} if cfg.estimate_link else set())
            | ({"sag"} if cfg.estimate_sag else set())
            | ({"depth"} if cfg.use_depth else set())
        ),
        handeye_R=handeye_R, handeye_t=handeye_t,
        target_R=target_R, target_t=target_t,
        joint_offsets=joint_offsets,
        link_trans=link_trans,
        link_rot=link_rot,
        sag_k=sag_k,
        cost=float(result.cost),
        converged=bool(result.success),
        n_iters=int(result.nfev),
        n_residuals=int(r_data_final.size),
        reproj_rms_px=reproj_rms,
        sigma_handeye_rot_deg=sigma_he_rot_deg,
        sigma_handeye_t_mm=sigma_he_t_mm,
        sigma_target_rot_deg=sigma_target_rot_deg,
        sigma_target_t_mm=sigma_target_t_mm,
        per_pose_rms_px=rms_per_pose,
        per_pose_weight=weights.tolist(),
        n_outliers=int(np.sum(weights < 0.5)),
    )

    try:
        eff_R, eff_t = measure_effective_sigma(
            captures, fk_chain, arm_cfgs, stage
        )
        stage.effective_sigma_handeye_rot_deg = eff_R
        stage.effective_sigma_handeye_t_mm = eff_t
    except Exception:
        logger.exception("effective σ 계산 실패 — inf 유지")

    return stage


# ─── Sanity checks ──────────────────────────────────────────────


def check_sanity(
    result: StageResult,
    arm_cfgs: list[MotorConfig],
    train_rms_px: float | None = None,
) -> list[SanityFlag]:
    """추정 파라미터의 물리적 합리성 검증."""
    flags: list[SanityFlag] = []

    # 1. Hand-eye translation.
    t_norm = float(np.linalg.norm(result.handeye_t))
    if t_norm > HE_T_MAX_RED:
        flags.append(SanityFlag(
            "handeye_t", SanityLevel.RED,
            f"hand-eye |t|={t_norm*1000:.1f}mm > {HE_T_MAX_RED*1000:.0f}mm "
            "— 손목 마운트 자리 카메라 자리 너무 멀리 자리. 모델 오차 흡수 의심.",
            value=t_norm,
        ))
    elif t_norm > HE_T_MAX_OK:
        flags.append(SanityFlag(
            "handeye_t", SanityLevel.YELLOW,
            f"hand-eye |t|={t_norm*1000:.1f}mm > {HE_T_MAX_OK*1000:.0f}mm "
            "— EE 부터 거리 보통 5-15cm. 마운트 자리 확인.",
            value=t_norm,
        ))
    else:
        flags.append(SanityFlag(
            "handeye_t", SanityLevel.OK,
            f"hand-eye |t|={t_norm*1000:.1f}mm ✓",
            value=t_norm,
        ))

    # 2. Joint offsets.
    if "joint" in result.estimated:
        for motor_id, off_rad in result.joint_offsets.items():
            off_deg = abs(np.rad2deg(off_rad))
            if off_deg > JOINT_OFF_RED_DEG:
                flags.append(SanityFlag(
                    f"joint_offset_J{motor_id}", SanityLevel.RED,
                    f"J{motor_id} offset={off_deg:.2f}° > {JOINT_OFF_RED_DEG}° "
                    "— servo zero 자리 5° 자리 자리 벗어남 자리 비현실적.",
                    value=off_rad,
                ))
            elif off_deg > JOINT_OFF_OK_DEG:
                flags.append(SanityFlag(
                    f"joint_offset_J{motor_id}", SanityLevel.YELLOW,
                    f"J{motor_id} offset={off_deg:.2f}° > {JOINT_OFF_OK_DEG}° "
                    "— 다소 큼. 다른 자리 자리 흡수 자리 의심.",
                    value=off_rad,
                ))

    # 3. Link offsets (Δxyz).
    if "link" in result.estimated:
        for motor_id, trans in result.link_trans.items():
            mag_mm = float(np.linalg.norm(trans)) * 1000.0
            if mag_mm > LINK_T_RED_MM:
                flags.append(SanityFlag(
                    f"link_trans_J{motor_id}", SanityLevel.RED,
                    f"J{motor_id} link Δxyz={mag_mm:.1f}mm > {LINK_T_RED_MM}mm "
                    "— CAD ↔ 실측 자리 tolerance 자리 자리 자리 보임.",
                    value=mag_mm / 1000.0,
                ))
            elif mag_mm > LINK_T_OK_MM:
                flags.append(SanityFlag(
                    f"link_trans_J{motor_id}", SanityLevel.YELLOW,
                    f"J{motor_id} link Δxyz={mag_mm:.1f}mm > {LINK_T_OK_MM}mm",
                    value=mag_mm / 1000.0,
                ))

        # 4. Link offsets (Δrpy).
        for motor_id, rot in result.link_rot.items():
            mag_deg = float(np.rad2deg(np.linalg.norm(rot)))
            if mag_deg > LINK_R_RED_DEG:
                flags.append(SanityFlag(
                    f"link_rot_J{motor_id}", SanityLevel.RED,
                    f"J{motor_id} link Δrpy={mag_deg:.2f}° > {LINK_R_RED_DEG}° "
                    "— 조립 tolerance 자리 자리 큼.",
                    value=mag_deg,
                ))
            elif mag_deg > LINK_R_OK_DEG:
                flags.append(SanityFlag(
                    f"link_rot_J{motor_id}", SanityLevel.YELLOW,
                    f"J{motor_id} link Δrpy={mag_deg:.2f}° > {LINK_R_OK_DEG}°",
                    value=mag_deg,
                ))

    # 5. Sag stiffness.
    if "sag" in result.estimated:
        for motor_id, k in result.sag_k.items():
            k_abs = abs(k)
            if k_abs > SAG_K_RED_MAX:
                flags.append(SanityFlag(
                    f"sag_k_J{motor_id}", SanityLevel.RED,
                    f"J{motor_id} sag_k={k:.3f} 자리 자리 자리 큼 (> {SAG_K_RED_MAX}) "
                    "— lumped mass 자리 자리 자리 비현실적.",
                    value=k,
                ))
            elif k_abs > SAG_K_OK_MAX:
                flags.append(SanityFlag(
                    f"sag_k_J{motor_id}", SanityLevel.YELLOW,
                    f"J{motor_id} sag_k={k:.3f} 자리 다소 큼 (> {SAG_K_OK_MAX})",
                    value=k,
                ))

    # 6. Outlier rate.
    if result.per_pose_weight:
        outlier_rate = result.n_outliers / len(result.per_pose_weight)
        if outlier_rate > OUTLIER_RATE_RED:
            flags.append(SanityFlag(
                "outlier_rate", SanityLevel.RED,
                f"{result.n_outliers}/{len(result.per_pose_weight)} pose 자리 "
                f"({outlier_rate*100:.0f}%) down-weight — "
                "데이터 자리 자리 자리 절반 자리 자리 자리 fit 자리 의심.",
                value=outlier_rate,
            ))
        elif outlier_rate > OUTLIER_RATE_OK:
            flags.append(SanityFlag(
                "outlier_rate", SanityLevel.YELLOW,
                f"{result.n_outliers}/{len(result.per_pose_weight)} pose 자리 "
                f"({outlier_rate*100:.0f}%) down-weight",
                value=outlier_rate,
            ))
        else:
            flags.append(SanityFlag(
                "outlier_rate", SanityLevel.OK,
                f"{result.n_outliers}/{len(result.per_pose_weight)} pose 자리 "
                f"({outlier_rate*100:.0f}%) down-weight ✓",
                value=outlier_rate,
            ))

    # 7. LOOCV vs train ratio.
    if (
        result.loocv_rms_px < float("inf")
        and train_rms_px is not None
        and train_rms_px > 0
    ):
        ratio = result.loocv_rms_px / train_rms_px
        if ratio > LOOCV_RATIO_RED:
            flags.append(SanityFlag(
                "loocv_ratio", SanityLevel.RED,
                f"LOOCV/train = {ratio:.2f}× > {LOOCV_RATIO_RED} "
                "— overfit. 더 단순 stage 자리 자리.",
                value=ratio,
            ))
        elif ratio > LOOCV_RATIO_OK:
            flags.append(SanityFlag(
                "loocv_ratio", SanityLevel.YELLOW,
                f"LOOCV/train = {ratio:.2f}× > {LOOCV_RATIO_OK}",
                value=ratio,
            ))
        else:
            flags.append(SanityFlag(
                "loocv_ratio", SanityLevel.OK,
                f"LOOCV/train = {ratio:.2f}× ✓",
                value=ratio,
            ))

    return flags


# ─── LOOCV ───────────────────────────────────────────────────────


def compute_loocv(
    captures: list[CapturePose],
    fk_chain,
    K: np.ndarray,
    sag_arm_indices: list[int],
    cfg: BAConfig,
    seed_R: np.ndarray, seed_t: np.ndarray,
    arm_cfgs: list[MotorConfig],
) -> float:
    """Leave-one-out CV: 각 capture 빼고 BA → held-out reprojection RMS 자리 자리."""
    if len(captures) < 5:
        return float("inf")
    held_out_rms: list[float] = []
    for skip_idx in range(len(captures)):
        train = [c for i, c in enumerate(captures) if i != skip_idx]
        try:
            res = run_ba_stage(
                train, fk_chain, K, sag_arm_indices, cfg,
                name="loocv", seed_handeye_R=seed_R, seed_handeye_t=seed_t,
                arm_cfgs=arm_cfgs,
                irls_outer=1, max_nfev=200,  # 빠르게.
            )
        except Exception:
            held_out_rms.append(float("inf"))
            continue
        # held-out 자리 res 의 파라미터 자리 plug-in → residual 계산.
        n_arm = fk_chain.n_arm
        n_sag = len(sag_arm_indices)
        # joint/link/sag 자리 arrays 자리 재구성.
        joint_off = np.zeros(n_arm)
        link_t = np.zeros((n_arm, 3))
        link_r = np.zeros((n_arm, 3))
        sag_sparse = np.zeros(n_sag)
        if "joint" in res.estimated:
            for i, c in enumerate(arm_cfgs):
                joint_off[i] = res.joint_offsets.get(c.id, 0.0)
        if "link" in res.estimated:
            for i, c in enumerate(arm_cfgs):
                if c.id in res.link_trans:
                    link_t[i] = res.link_trans[c.id]
                if c.id in res.link_rot:
                    link_r[i] = res.link_rot[c.id]
        if "sag" in res.estimated:
            for i, arm_idx in enumerate(sag_arm_indices):
                sag_sparse[i] = res.sag_k.get(arm_cfgs[arm_idx].id, 0.0)
        x = pack_params(
            res.handeye_R, res.handeye_t, res.target_R, res.target_t,
            joint_off, link_t, link_r, sag_sparse,
            cfg,
        )
        r = compute_residuals(
            x, [captures[skip_idx]], fk_chain, K, sag_arm_indices, cfg,
            pose_weights=np.ones(1),
        )
        # 2D 잔차만 (priors + depth 제외 자리 자리 자리 자리 자리, 단순화 자리 2*n_corners 만).
        n_corners = len(captures[skip_idx].board_obj_pts)
        r2d = r[:2 * n_corners]
        held_out_rms.append(float(np.sqrt(np.mean(r2d ** 2))))
    return float(np.mean(held_out_rms))


# ─── Stage 선택 ──────────────────────────────────────────────────


def pick_best_stage(
    stages: dict[str, StageResult],
    *,
    force: str | None = None,
) -> str:
    """LOOCV 우선, RED 자리 자리 자리 자리 자리 자리. tie 자리 자리 simpler."""
    if force is not None:
        if force not in stages:
            raise ValueError(f"강제 stage {force} 자리 결과 없음")
        return force

    # RED flag 없는 stage 만 후보 (있는 모든 stage 가 RED 면 어쩔 수 없이 그 중 best).
    no_red = {
        n: r for n, r in stages.items()
        if not any(f.level == SanityLevel.RED for f in r.sanity)
    }
    pool = no_red if no_red else stages

    def score(name: str) -> tuple[float, int, float]:
        r = pool[name]
        # LOOCV 우선, 없으면 train RMS.
        primary = r.loocv_rms_px if r.loocv_rms_px < float("inf") else r.reproj_rms_px
        complexity = len(r.estimated)
        # tiebreaker — train RMS.
        return (primary, complexity, r.reproj_rms_px)

    return min(pool.keys(), key=score)


# ─── Report ──────────────────────────────────────────────────────


def format_report(
    run: dict,
    stages: dict[str, StageResult],
    best: str,
    arm_cfgs: list[MotorConfig],
    sag_arm_indices: list[int],
) -> str:
    """사람이 읽기 쉬운 multi-stage report."""
    lines: list[str] = []
    lines.append("=" * 78)
    lines.append(
        f" Offline Calibration Report — robot={run['robot_id']} run_id={run['id']}"
    )
    lines.append("=" * 78)
    lines.append("")

    # Stage summary table.
    lines.append("Stage  | est'd                  | RMS (px) | LOOCV    | σ_he_R | σ_he_t  | outliers | sanity")
    lines.append("-" * 110)
    for n, r in stages.items():
        est = "+".join(sorted(r.estimated))
        marker = " ← BEST" if n == best else ""
        sanity_str = r.worst_sanity.value
        lines.append(
            f"  {n}    | {est:22s} | {r.reproj_rms_px:8.4f} | "
            f"{r.loocv_rms_px:8.4f} | {r.sigma_handeye_rot_deg:5.2f}° | "
            f"{r.sigma_handeye_t_mm:6.2f}mm | "
            f"{r.n_outliers}/{len(r.per_pose_weight):<3d}   | {sanity_str}{marker}"
        )
    lines.append("")

    # Best stage 자세히.
    br = stages[best]
    lines.append(f"━━━ Best: Stage {best} ({'+'.join(sorted(br.estimated))}) ━━━")
    lines.append("")

    # Hand-eye 자리 시각화.
    he_R_euler = Rot.from_matrix(br.handeye_R).as_euler("xyz", degrees=True)
    lines.append("[Hand-eye]")
    lines.append(
        f"  t (cam in EE) = [{br.handeye_t[0]*1000:+.2f}, "
        f"{br.handeye_t[1]*1000:+.2f}, {br.handeye_t[2]*1000:+.2f}] mm "
        f"(|t|={np.linalg.norm(br.handeye_t)*1000:.1f}mm)"
    )
    lines.append(
        f"  R euler XYZ = [{he_R_euler[0]:+.2f}, {he_R_euler[1]:+.2f}, "
        f"{he_R_euler[2]:+.2f}] deg"
    )
    lines.append("")

    # Per-joint estimated values.
    if "joint" in br.estimated:
        lines.append("[Joint offsets (servo zero correction)]")
        for c in arm_cfgs:
            off_rad = br.joint_offsets.get(c.id, 0.0)
            lines.append(
                f"  J{c.id} ({c.name:20s}): {np.rad2deg(off_rad):+7.3f}°  "
                f"= {off_rad:+.5f} rad"
            )
        lines.append("")

    if "link" in br.estimated:
        lines.append("[Link offsets (URDF Δxyz / Δrpy per joint origin)]")
        for c in arm_cfgs:
            t = br.link_trans.get(c.id, np.zeros(3))
            r = br.link_rot.get(c.id, np.zeros(3))
            lines.append(
                f"  J{c.id} ({c.name:20s}): "
                f"Δxyz=[{t[0]*1000:+.2f}, {t[1]*1000:+.2f}, {t[2]*1000:+.2f}] mm  "
                f"Δrpy=[{np.rad2deg(r[0]):+.3f}, {np.rad2deg(r[1]):+.3f}, "
                f"{np.rad2deg(r[2]):+.3f}]°"
            )
        lines.append("")

    if "sag" in br.estimated:
        lines.append("[Sag stiffness (joint deflection per gravity torque)]")
        for arm_idx in sag_arm_indices:
            mc = arm_cfgs[arm_idx]
            k = br.sag_k.get(mc.id, 0.0)
            lines.append(
                f"  J{mc.id} ({mc.name:20s}): k = {k:+.4f} rad/(m·g_unit)"
            )
        lines.append("")

    # Per-pose RMS + weight.
    lines.append("[Per-pose RMS + IRLS weight]")
    lines.append("  pose  | rms (px) | weight | status")
    lines.append("  ------+----------+--------+--------")
    for i, (rms, w) in enumerate(zip(br.per_pose_rms_px, br.per_pose_weight)):
        status = ""
        if w < 0.3:
            status = "  ← excluded (low weight)"
        elif w < 0.7:
            status = "  ← down-weighted"
        lines.append(
            f"  #{i:3d}  | {rms:8.4f} | {w:6.3f} | {status}"
        )
    lines.append("")

    # Sanity flags.
    lines.append("[Sanity Checks]")
    if not br.sanity:
        lines.append("  (no flags)")
    else:
        for f in br.sanity:
            icon = {"OK": "✓", "WARN": "⚠", "RED": "✗"}[f.level.value]
            lines.append(f"  {icon} [{f.level.value:4s}] {f.category}: {f.message}")
    lines.append("")

    return "\n".join(lines)


# ─── Commit (직접 SQLite 자리) ────────────────────────────────────


def commit_results(
    db_path: Path, result: StageResult, run_id: int, robot_id: str,
    arm_cfgs: list[MotorConfig], sag_arm_indices: list[int],
) -> dict:
    """SQLite 직접 — finalize_run + Result rows INSERT + activate (UNIQUE partial index).

    Returns dict: {finalized_run_id, inserted_result_ids, activated}
    """
    now = time.time()
    con = sqlite3.connect(str(db_path))
    con.execute("PRAGMA foreign_keys = ON")
    cur = con.cursor()
    try:
        # Run status: in_progress/ready_for_analysis → success.
        cur.execute(
            "UPDATE calibration_runs SET status='success', ended_at=? WHERE id=?",
            (now, run_id),
        )
        if cur.rowcount == 0:
            raise RuntimeError(f"run id={run_id} update 실패 (없음?)")

        result_ids: list[tuple[str, int]] = []
        method = f"offline_BA_stage_{result.name}"

        # 1. Hand-eye 항상 commit. σ 두 종류 박음 — sigma_* = Jacobian (confidence),
        # effective_sigma_* = board_in_base std (accuracy, commit 결정 metric).
        he_data = {
            "R_cam2gripper": result.handeye_R.tolist(),
            "t_cam2gripper": result.handeye_t.reshape(3, 1).tolist(),
            "method": method,
        }
        cur.execute(
            "INSERT INTO calibration_results "
            "(run_id, robot_id, kind, created_at, is_active, "
            "sigma_rot, sigma_t, effective_sigma_rot, effective_sigma_t, result_data) "
            "VALUES (?,?,?,?,?,?,?,?,?,?)",
            (
                run_id, robot_id, "hand_eye", now, 0,
                result.sigma_handeye_rot_deg, result.sigma_handeye_t_mm,
                result.effective_sigma_handeye_rot_deg,
                result.effective_sigma_handeye_t_mm,
                json.dumps(he_data),
            ),
        )
        rid = cur.lastrowid
        assert rid is not None
        result_ids.append(("hand_eye", rid))

        # 2. Joint offsets.
        if "joint" in result.estimated:
            jdata = {
                "offsets": {str(k): v for k, v in result.joint_offsets.items()},
                "method": method,
            }
            cur.execute(
                "INSERT INTO calibration_results "
                "(run_id, robot_id, kind, created_at, is_active, sigma_rot, sigma_t, result_data) "
                "VALUES (?,?,?,?,?,?,?,?)",
                (
                    run_id, robot_id, "joint_offset", now, 0,
                    None, None, json.dumps(jdata),
                ),
            )
            rid = cur.lastrowid
            assert rid is not None
            result_ids.append(("joint_offset", rid))

        # 3. Link offsets.
        if "link" in result.estimated:
            entries = []
            for c in arm_cfgs:
                t = result.link_trans.get(c.id, np.zeros(3))
                r = result.link_rot.get(c.id, np.zeros(3))
                entries.append({
                    "joint_id": c.id,
                    "trans_m": [float(t[0]), float(t[1]), float(t[2])],
                    "rot_rad": [float(r[0]), float(r[1]), float(r[2])],
                })
            ldata = {"offsets": entries, "method": method}
            cur.execute(
                "INSERT INTO calibration_results "
                "(run_id, robot_id, kind, created_at, is_active, sigma_rot, sigma_t, result_data) "
                "VALUES (?,?,?,?,?,?,?,?)",
                (
                    run_id, robot_id, "link_offset", now, 0,
                    None, None, json.dumps(ldata),
                ),
            )
            rid = cur.lastrowid
            assert rid is not None
            result_ids.append(("link_offset", rid))

        # 4. Sag.
        if "sag" in result.estimated:
            sdata = {
                "k_rad_per_m": {str(k): v for k, v in result.sag_k.items()},
                "method": method,
            }
            cur.execute(
                "INSERT INTO calibration_results "
                "(run_id, robot_id, kind, created_at, is_active, sigma_rot, sigma_t, result_data) "
                "VALUES (?,?,?,?,?,?,?,?)",
                (
                    run_id, robot_id, "sag", now, 0,
                    None, None, json.dumps(sdata),
                ),
            )
            rid = cur.lastrowid
            assert rid is not None
            result_ids.append(("sag", rid))

        # 5. Activate — 같은 (robot_id, kind) 의 기존 is_active=1 → 0, 새 row → 1.
        # UNIQUE partial index 자리 보장.
        activated: list[tuple[str, int]] = []
        for kind, rid in result_ids:
            cur.execute(
                "UPDATE calibration_results SET is_active=0 "
                "WHERE robot_id=? AND kind=? AND is_active=1 AND id != ?",
                (robot_id, kind, rid),
            )
            cur.execute(
                "UPDATE calibration_results SET is_active=1 WHERE id=?",
                (rid,),
            )
            activated.append((kind, rid))

        con.commit()
        logger.info(
            "Commit 완료 — run=%d → success, %d kind activate",
            run_id, len(activated),
        )
        return {
            "finalized_run_id": run_id,
            "activated": activated,
            "n_results": len(result_ids),
        }
    except Exception:
        con.rollback()
        raise
    finally:
        con.close()


# ─── 메인 ────────────────────────────────────────────────────────


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--robot", required=True, help="robot_id")
    parser.add_argument(
        "--run-id", type=int, default=None,
        help="분석할 run_id (미지정 시 최신 ready_for_analysis hand_eye)",
    )
    parser.add_argument(
        "--db", type=Path,
        default=BACKEND / "storage" / "horibot.db",
    )
    parser.add_argument(
        "--blobs", type=Path,
        default=BACKEND / "storage" / "blobs",
    )
    parser.add_argument(
        "--stage", choices=["A", "B", "C", "D", "E", "auto"], default="auto",
    )
    parser.add_argument("--skip-depth", action="store_true",
                        help="Stage E 자리 skip (depth blob load 자리 무거움)")
    parser.add_argument("--skip-loocv", action="store_true",
                        help="LOOCV skip (stage 자리 4-5종 × n_captures BA, 수분)")
    parser.add_argument("--commit", action="store_true",
                        help="best stage 결과 storage SQLite 직접 commit + activate")
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument(
        "--drop-poses", type=int, nargs="+", default=[],
        help="명시 제외할 pose_index 리스트 (예: --drop-poses 6 21). outlier 명시 제거 — "
        "BA IRLS down-weight 보다 강력 (완전 배제). 사용자가 capture quality 가 명백히 "
        "안 좋은 자세를 골라 빼는 용도.",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname).1s] %(message)s",
    )
    # Windows cp949 console 자리 unicode (✓, ✗, ⚠, ─, etc.) 출력 가능하도록.
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
        sys.stderr.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except (AttributeError, Exception):
        pass

    # ─── Load ─────────────────────────────────────────────────
    logger.info("=== Load ===")
    run, captures, intrinsic, arm_cfgs = load_data(
        args.db, args.blobs, args.robot, args.run_id,
        load_depth=not args.skip_depth,
    )
    logger.info(
        "run_id=%d status=%s captures=%d intrinsic=%s",
        run["id"], run["status"], len(captures), intrinsic["source"],
    )

    # ─── 명시 outlier 제거 ──────────────────────────────────────
    if args.drop_poses:
        before = len(captures)
        drop_set = set(args.drop_poses)
        captures = [c for c in captures if c.pose_index not in drop_set]
        logger.info(
            "--drop-poses %s → %d → %d captures",
            sorted(drop_set), before, len(captures),
        )

    # ─── Setup ────────────────────────────────────────────────
    from modules.kinematics.fk_chain import FkChain
    registry = RobotRegistry()
    fk_chain: FkChain = registry.get_fk_chain(args.robot)  # type: ignore[assignment]
    K = intrinsic["camera_matrix"]
    cfg_robot = registry.get(args.robot)
    sag_arm_indices = [m - 1 for m in cfg_robot.sag_joint_motor_ids]
    logger.info(
        "n_arm=%d sag_joint_motor_ids (1-based)=%s",
        fk_chain.n_arm, cfg_robot.sag_joint_motor_ids,
    )

    # ─── Seed ─────────────────────────────────────────────────
    logger.info("=== Seed (cv2.calibrateHandEye 5 methods) ===")
    seed_R, seed_t, seed_name = seed_handeye(captures, fk_chain)
    logger.info("Seed best: %s  t=%s mm", seed_name, np.round(seed_t * 1000, 2))

    # ─── Multi-stage BA ───────────────────────────────────────
    stage_configs: dict[str, BAConfig] = {
        "A": BAConfig(),
        "B": BAConfig(estimate_joint=True),
        "C": BAConfig(estimate_joint=True, estimate_link=True),
        "D": BAConfig(estimate_joint=True, estimate_link=True, estimate_sag=True),
    }
    has_depth = any(c.depth_z16 is not None for c in captures)
    if not args.skip_depth and has_depth:
        stage_configs["E"] = BAConfig(
            estimate_joint=True, estimate_link=True,
            estimate_sag=True, use_depth=True,
        )

    stages: dict[str, StageResult] = {}
    cur_seed_R, cur_seed_t = seed_R, seed_t
    for name, cfg in stage_configs.items():
        logger.info("=== BA Stage %s ===", name)
        t0 = time.time()
        try:
            res = run_ba_stage(
                captures, fk_chain, K, sag_arm_indices, cfg,
                name=name,
                seed_handeye_R=cur_seed_R, seed_handeye_t=cur_seed_t,
                arm_cfgs=arm_cfgs,
                irls_outer=3,
            )
            stages[name] = res
            logger.info(
                "  reproj_rms=%.4fpx  σ_he_R=%.3f°  σ_he_t=%.3fmm  "
                "outliers=%d/%d  iters=%d  (%.1fs)",
                res.reproj_rms_px,
                res.sigma_handeye_rot_deg, res.sigma_handeye_t_mm,
                res.n_outliers, len(captures), res.n_iters,
                time.time() - t0,
            )
            cur_seed_R, cur_seed_t = res.handeye_R, res.handeye_t
        except Exception:
            logger.exception("Stage %s 실패", name)

    if not stages:
        logger.error("모든 stage 실패")
        return 1

    # ─── LOOCV ────────────────────────────────────────────────
    if not args.skip_loocv:
        logger.info("=== LOOCV (각 stage 25× BA) ===")
        for name, cfg in stage_configs.items():
            if name not in stages:
                continue
            t0 = time.time()
            loocv = compute_loocv(
                captures, fk_chain, K, sag_arm_indices, cfg,
                seed_R, seed_t, arm_cfgs,
            )
            stages[name].loocv_rms_px = loocv
            logger.info(
                "  Stage %s LOOCV=%.4fpx  (train=%.4fpx, ratio=%.2f×, %.1fs)",
                name, loocv, stages[name].reproj_rms_px,
                loocv / stages[name].reproj_rms_px if stages[name].reproj_rms_px > 0 else float("inf"),
                time.time() - t0,
            )

    # ─── Sanity ───────────────────────────────────────────────
    for name, res in stages.items():
        res.sanity = check_sanity(res, arm_cfgs, train_rms_px=res.reproj_rms_px)

    # ─── Stage 선택 ───────────────────────────────────────────
    force = None if args.stage == "auto" else args.stage
    try:
        best_name = pick_best_stage(stages, force=force)
    except ValueError as e:
        logger.error(str(e))
        return 1

    # ─── Report ───────────────────────────────────────────────
    report = format_report(run, stages, best_name, arm_cfgs, sag_arm_indices)
    print()
    print(report)

    if args.output_json:
        summary = {
            "robot_id": args.robot,
            "run_id": run["id"],
            "n_captures": len(captures),
            "seed_method": seed_name,
            "best_stage": best_name,
            "stages": {
                n: {
                    "estimated": sorted(r.estimated),
                    "reproj_rms_px": r.reproj_rms_px,
                    "loocv_rms_px": r.loocv_rms_px,
                    "sigma_handeye_rot_deg": r.sigma_handeye_rot_deg,
                    "sigma_handeye_t_mm": r.sigma_handeye_t_mm,
                    "n_outliers": r.n_outliers,
                    "n_iters": r.n_iters,
                    "converged": r.converged,
                    "handeye_t_mm": (r.handeye_t * 1000).tolist(),
                    "handeye_R_euler_deg": Rot.from_matrix(r.handeye_R).as_euler("xyz", degrees=True).tolist(),
                    "joint_offsets_deg": {
                        str(k): float(np.rad2deg(v)) for k, v in r.joint_offsets.items()
                    },
                    "link_trans_mm": {
                        str(k): (v * 1000).tolist() for k, v in r.link_trans.items()
                    },
                    "link_rot_deg": {
                        str(k): np.rad2deg(v).tolist() for k, v in r.link_rot.items()
                    },
                    "sag_k": {str(k): v for k, v in r.sag_k.items()},
                    "per_pose_rms_px": r.per_pose_rms_px,
                    "per_pose_weight": r.per_pose_weight,
                    "sanity": [
                        {"category": f.category, "level": f.level.value, "message": f.message}
                        for f in r.sanity
                    ],
                }
                for n, r in stages.items()
            },
        }
        args.output_json.write_text(json.dumps(summary, indent=2))
        logger.info("결과 JSON → %s", args.output_json)

    # ─── Commit ───────────────────────────────────────────────
    if args.commit:
        worst = stages[best_name].worst_sanity
        if worst == SanityLevel.RED:
            logger.warning(
                "RED FLAG 자리 있음 — commit 자리 보류 권장. "
                "강제 진행 자리 자리 자리 --commit 자리 자리 자리 자리 자리. "
                "지금 자리 자리 abort."
            )
            return 2
        result = commit_results(
            args.db, stages[best_name], run["id"], args.robot,
            arm_cfgs, sag_arm_indices,
        )
        logger.info("Commit OK: %s", result)
        logger.info(
            "다음 단계: backend 재시작 → calibration_node._setup_runtime_calibration "
            "이 새 active result 자리 fetch → CalibrationCache + URDF patch 자리 적용."
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
