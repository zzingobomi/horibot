import cv2
import numpy as np
import logging
import time
from pathlib import Path
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class HandEyeResult:
    R_cam2gripper: np.ndarray  # 회전 행렬 (카메라 → 그리퍼)
    t_cam2gripper: np.ndarray  # 이동 벡터
    method: str


@dataclass
class Pose:
    R_gripper2base: np.ndarray  # 로봇 FK에서 얻은 회전 행렬
    t_gripper2base: np.ndarray  # 로봇 FK에서 얻은 이동 벡터
    R_target2cam: np.ndarray  # 체커보드 → 카메라 회전
    t_target2cam: np.ndarray  # 체커보드 → 카메라 이동
    id: int = -1
    timestamp: float = field(default_factory=time.time)
    joint_angles_rad: list[float] = field(default_factory=list)


_METHOD_NAMES = {
    cv2.CALIB_HAND_EYE_TSAI: "TSAI",
    cv2.CALIB_HAND_EYE_PARK: "PARK",
    cv2.CALIB_HAND_EYE_HORAUD: "HORAUD",
    cv2.CALIB_HAND_EYE_ANDREFF: "ANDREFF",
    cv2.CALIB_HAND_EYE_DANIILIDIS: "DANIILIDIS",
}

_COMPARE_METHODS = [
    cv2.CALIB_HAND_EYE_TSAI,
    cv2.CALIB_HAND_EYE_PARK,
    cv2.CALIB_HAND_EYE_DANIILIDIS,
]


class HandEyeCalibration:
    def __init__(self):
        self._next_id: int = 0
        self.poses: list[Pose] = []
        self.result: HandEyeResult | None = None

    def add_pose(self, pose: Pose) -> None:
        if pose.id < 0:
            pose.id = self._next_id
            self._next_id += 1
        else:
            self._next_id = max(self._next_id, pose.id + 1)
        self.poses.append(pose)
        logger.info(f"포즈 #{pose.id} 추가됨 ({len(self.poses)}개)")

    def remove_pose_by_id(self, pose_id: int) -> bool:
        for i, p in enumerate(self.poses):
            if p.id == pose_id:
                del self.poses[i]
                logger.info(
                    f"포즈 #{pose_id} 제거됨 (남은 포즈: {len(self.poses)}개)"
                )
                return True
        return False

    def calibrate(self, method: int = cv2.CALIB_HAND_EYE_TSAI) -> HandEyeResult | None:
        if len(self.poses) < 3:
            logger.warning(f"포즈 부족: {len(self.poses)}개 (최소 3개 필요)")
            return None

        R_gripper2base = [p.R_gripper2base for p in self.poses]
        t_gripper2base = [p.t_gripper2base for p in self.poses]
        R_target2cam = [p.R_target2cam for p in self.poses]
        t_target2cam = [p.t_target2cam for p in self.poses]

        R, t = cv2.calibrateHandEye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            method=method,
        )

        self.result = HandEyeResult(
            R_cam2gripper=R,
            t_cam2gripper=t,
            method=_METHOD_NAMES.get(method, "UNKNOWN"),
        )
        logger.info(
            f"Hand-Eye 캘리브레이션 완료 (method={self.result.method}, "
            f"poses={len(self.poses)})"
        )
        return self.result

    def compute_with_diagnostics(
        self, method: int = cv2.CALIB_HAND_EYE_TSAI
    ) -> dict | None:
        """calibrate() + method 비교 + 포즈별 T_target2base 분산 잔차를 한 번에 반환.

        잔차 의미: 좋은 캘이면 모든 포즈에서 T_target2base = T_gripper2base · X ·
        T_target2cam이 같은 값. (X = T_cam2gripper, 체커보드가 안 움직였으니까)
        per-pose 편차가 hand-eye + FK 오차의 직접 측정치.
        """
        result = self.calibrate(method=method)
        if result is None:
            return None

        # ── method 비교 (self-consistency) ────────────────────────
        compare = []
        ref_R: np.ndarray | None = None
        ref_t: np.ndarray | None = None
        for m in _COMPARE_METHODS:
            try:
                Rm, tm = cv2.calibrateHandEye(
                    [p.R_gripper2base for p in self.poses],
                    [p.t_gripper2base for p in self.poses],
                    [p.R_target2cam for p in self.poses],
                    [p.t_target2cam for p in self.poses],
                    method=m,
                )
                name = _METHOD_NAMES.get(m, "?")
                if ref_R is None:
                    ref_R, ref_t = Rm, tm
                    compare.append(
                        {"method": name, "drot_deg": 0.0,
                            "dt_mm": 0.0, "ref": True}
                    )
                else:
                    assert ref_t is not None
                    drot = _rotation_diff_deg(ref_R, Rm)
                    dt_mm = float(np.linalg.norm(ref_t - tm)) * 1000.0
                    compare.append(
                        {
                            "method": name,
                            "drot_deg": drot,
                            "dt_mm": dt_mm,
                            "ref": False,
                        }
                    )
            except cv2.error as e:
                logger.warning(f"  {_METHOD_NAMES.get(m, '?')} 실패: {e}")

        # ── 포즈별 T_target2base 잔차 ─────────────────────────────
        per_pose, sigma_rot_deg, sigma_t_mm = self._compute_residuals(
            result.R_cam2gripper, result.t_cam2gripper
        )

        _log_method_compare(compare)
        logger.info(
            "─── per-pose 잔차 (σ_rot=%.3f°, σ_t=%.1fmm) ───",
            sigma_rot_deg,
            sigma_t_mm,
        )

        diagnosis = _diagnose(
            per_pose, sigma_rot_deg, sigma_t_mm, compare, self.poses
        )
        logger.info("진단: [%s] %s",
                    diagnosis["status"], diagnosis["message"])

        return {
            "R_cam2gripper": result.R_cam2gripper.tolist(),
            "t_cam2gripper": result.t_cam2gripper.flatten().tolist(),
            "method": result.method,
            "pose_count": len(self.poses),
            "method_compare": compare,
            "per_pose_residual": per_pose,
            "sigma_rot_deg": sigma_rot_deg,
            "sigma_t_mm": sigma_t_mm,
            "diagnosis": diagnosis,
        }

    def compute_with_bundle(self, solver) -> dict | None:
        """Bundle Adjustment 기반 hand-eye + joint zero offset 동시 최적화.

        cv2.calibrateHandEye(TSAI)를 seed로 사용. 결과는 self.result에 method="BUNDLE"로
        저장되어 COMMIT으로 hand_eye.npz에 저장 가능. joint_offsets는 별도 키로 반환.

        Args:
            solver: PybulletSolver 인스턴스 (FK 재계산용).

        Returns:
            dict (BA 결과 + seed 비교) or None.
        """
        if len(self.poses) < 3:
            logger.warning(f"BA는 최소 3포즈 필요 (현재 {len(self.poses)})")
            return None

        # 1) TSAI seed
        seed = self.calibrate(method=cv2.CALIB_HAND_EYE_TSAI)
        if seed is None:
            return None
        seed_R = seed.R_cam2gripper
        seed_t = seed.t_cam2gripper.reshape(3)

        # 2) seed의 σ — BEFORE/AFTER 비교용
        _, seed_sigma_rot, seed_sigma_t = self._compute_residuals(seed_R, seed_t)

        # 3) BA 실행
        from .bundle_adjust import bundle_adjust

        try:
            ba = bundle_adjust(self.poses, solver, seed_R, seed_t)
        except Exception as e:
            logger.exception("BA 실패: %s", e)
            return None

        # 4) self.result 갱신 (COMMIT 가능하게)
        self.result = HandEyeResult(
            R_cam2gripper=ba["R_cam2gripper"],
            t_cam2gripper=ba["t_cam2gripper"].reshape(3, 1),
            method="BUNDLE",
        )

        return {
            "R_cam2gripper": ba["R_cam2gripper"].tolist(),
            "t_cam2gripper": ba["t_cam2gripper"].tolist(),
            "joint_offsets_rad": ba["joint_offsets_rad"].tolist(),
            "joint_offsets_deg": np.degrees(ba["joint_offsets_rad"]).tolist(),
            "method": "BUNDLE",
            "pose_count": len(self.poses),
            "sigma_rot_deg": ba["sigma_rot_deg"],
            "sigma_t_mm": ba["sigma_t_mm"],
            "per_pose_residual": ba["per_pose_residual"],
            "seed_sigma_rot_deg": seed_sigma_rot,
            "seed_sigma_t_mm": seed_sigma_t,
            "iterations": ba["iterations"],
            "cost_initial": ba["cost_initial"],
            "cost_final": ba["cost_final"],
            "elapsed_sec": ba["elapsed_sec"],
            "success": ba["success"],
        }

    def _compute_residuals(
        self, R_cam2gripper: np.ndarray, t_cam2gripper: np.ndarray
    ) -> tuple[list[dict], float, float]:
        T_cam2gripper = _make_T(R_cam2gripper, t_cam2gripper.reshape(3, 1))

        T_target2base_list: list[np.ndarray] = []
        for p in self.poses:
            T_gripper2base = _make_T(
                p.R_gripper2base, p.t_gripper2base.reshape(3, 1))
            T_target2cam = _make_T(
                p.R_target2cam, p.t_target2cam.reshape(3, 1))
            T_target2base_list.append(
                T_gripper2base @ T_cam2gripper @ T_target2cam)

        # holistic 평균: 모든 포즈를 대칭으로 본다.
        # - 위치는 산술 평균
        # - 회전은 quaternion 평균 (Markley/Crassidis: M=Σq_i q_i^T 의 최대 고유벡터)
        positions = np.array([T[:3, 3] for T in T_target2base_list])
        mean_pos = positions.mean(axis=0)
        rotations = [T[:3, :3] for T in T_target2base_list]
        mean_R = _mean_rotation(rotations)

        per_pose: list[dict] = []
        rot_devs: list[float] = []
        pos_devs: list[float] = []
        for pose, T in zip(self.poses, T_target2base_list):
            drot = _rotation_diff_deg(mean_R, T[:3, :3])
            dt_mm = float(np.linalg.norm(T[:3, 3] - mean_pos)) * 1000.0
            per_pose.append({"id": pose.id, "drot_deg": drot, "dt_mm": dt_mm})
            rot_devs.append(drot)
            pos_devs.append(dt_mm)

        # RMS — 통상 σ 의미와 일치 (np.std는 편차의 std라 잘못된 값)
        sigma_rot = float(
            np.sqrt(np.mean(np.square(rot_devs)))) if rot_devs else 0.0
        sigma_t = float(
            np.sqrt(np.mean(np.square(pos_devs)))) if pos_devs else 0.0
        return per_pose, sigma_rot, sigma_t

    def list_poses_meta(self) -> list[dict]:
        return [
            {
                "id": p.id,
                "timestamp": p.timestamp,
                "joint_angles_rad": p.joint_angles_rad,
            }
            for p in self.poses
        ]

    def save(self, path: str | Path) -> bool:
        if self.result is None:
            logger.warning("저장할 Hand-Eye 결과가 없습니다")
            return False

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            str(path),
            R_cam2gripper=self.result.R_cam2gripper,
            t_cam2gripper=self.result.t_cam2gripper,
            method=self.result.method,
        )
        logger.info(f"Hand-Eye 결과 저장: {path}")
        return True

    def load(self, path: str | Path) -> HandEyeResult | None:
        path = Path(path)
        if not path.exists():
            return None

        data = np.load(str(path), allow_pickle=True)
        self.result = HandEyeResult(
            R_cam2gripper=data["R_cam2gripper"],
            t_cam2gripper=data["t_cam2gripper"],
            method=str(data["method"]),
        )
        return self.result

    def reset(self) -> None:
        self._next_id = 0
        self.poses.clear()
        self.result = None


# 5DOF arm: joint 1=base yaw, joint 4=wrist pitch, joint 5=wrist roll.
# Hand-eye 자세 다양성에 결정적인 회전 자유도. 임계값은 도(°).
_DIVERSITY_THRESHOLD_DEG = {0: 60.0, 3: 40.0, 4: 40.0}
_JOINT_NAMES = {1: "base yaw", 4: "wrist pitch", 5: "wrist roll"}

_SIGMA_ROT_GOOD_DEG = 0.5
_SIGMA_T_GOOD_MM = 5.0
_PARK_AGREE_DEG = 1.0
_MAD_K = 3.0  # MAD 기반 outlier threshold 계수


def _diagnose(
    per_pose: list[dict],
    sigma_rot_deg: float,
    sigma_t_mm: float,
    method_compare: list[dict],
    poses: list[Pose],
) -> dict:
    """캘 결과를 보고 사용자에게 다음 액션을 안내.

    판단 순서:
      1) MAD 기반 단일 outlier 검출 → 있으면 'outlier_present'
      2) joint 다양성 부족 + σ 미달 → 'insufficient_diversity'
      3) σ 목표 충족 → 'good'
      4) 그 외 (outlier 없음 + 다양성 OK + σ 정체) → 'fk_floor_reached'
    """
    # 1) MAD 기반 outlier 검출 — robust
    drots = np.array([p["drot_deg"] for p in per_pose], dtype=np.float64)
    if drots.size >= 3:
        median = float(np.median(drots))
        mad = float(np.median(np.abs(drots - median)))
        if mad > 1e-6:
            threshold = median + _MAD_K * mad
            outlier_ids = [
                int(p["id"]) for p in per_pose if p["drot_deg"] > threshold
            ]
            if outlier_ids:
                return {
                    "status": "outlier_present",
                    "severity": "action_required",
                    "message": (
                        f"포즈 #{outlier_ids}이(가) 평균에서 도드라짐 — "
                        "삭제 후 재 COMPUTE"
                    ),
                    "outlier_ids": outlier_ids,
                }

    # 2) 자세 다양성 부족 검출
    if poses:
        joints = np.array(
            [p.joint_angles_rad for p in poses if p.joint_angles_rad],
            dtype=np.float64,
        )
        if joints.size > 0 and joints.ndim == 2:
            ranges_deg = np.degrees(joints.max(axis=0) - joints.min(axis=0))
            insufficient: list[tuple[int, float]] = []
            for idx, thr in _DIVERSITY_THRESHOLD_DEG.items():
                if idx < ranges_deg.size and ranges_deg[idx] < thr:
                    insufficient.append(
                        (idx + 1, float(ranges_deg[idx])))
            if insufficient and sigma_rot_deg >= _SIGMA_ROT_GOOD_DEG:
                details = ", ".join(
                    f"joint {i}({_JOINT_NAMES.get(i, '?')}) {r:.0f}°"
                    for i, r in insufficient
                )
                return {
                    "status": "insufficient_diversity",
                    "severity": "action_required",
                    "message": f"다양성 부족: {details} — 부족한 축의 자세 추가 캡처",
                    "low_diversity_joints": [i for i, _ in insufficient],
                }

    # 3) 캘 품질 충분
    if sigma_rot_deg < _SIGMA_ROT_GOOD_DEG and sigma_t_mm < _SIGMA_T_GOOD_MM:
        return {
            "status": "good",
            "severity": "success",
            "message": (
                f"품질 충분 (σ_rot {sigma_rot_deg:.2f}°, "
                f"σ_t {sigma_t_mm:.1f}mm) — COMMIT 권장"
            ),
        }

    # 4) FK floor 도달 — BA 필요
    park_drot = next(
        (c["drot_deg"] for c in method_compare if c.get("method") == "PARK"),
        None,
    )
    park_ok = park_drot is not None and park_drot < _PARK_AGREE_DEG
    return {
        "status": "fk_floor_reached",
        "severity": "action_required",
        "message": (
            f"σ_rot {sigma_rot_deg:.2f}° 정체 (outlier 없음 + 다양성 충분"
            f"{' + PARK 합의' if park_ok else ''}) — cv2 한계. "
            "Bundle Adjustment 필요."
        ),
        "sigma_rot_deg": sigma_rot_deg,
        "park_drot_deg": park_drot,
    }


def _R_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 회전 → 단위 quaternion [w, x, y, z]. Shepperd's method (수치 안정)."""
    m = R
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def _quat_to_R(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    """회전 행렬 리스트의 평균. Markley/Crassidis quaternion 평균.

    M = Σ q_i q_i^T 의 최대 고유벡터가 평균 quaternion. q와 -q가 같은 회전이지만
    M = q q^T 형태라 부호와 무관.
    """
    if not rotations:
        return np.eye(3)
    M = np.zeros((4, 4))
    for R in rotations:
        q = _R_to_quat(R).reshape(4, 1)
        M += q @ q.T
    M /= len(rotations)
    _, eigvecs = np.linalg.eigh(M)
    q_mean = eigvecs[:, -1]  # 최대 고유값 대응 고유벡터
    return _quat_to_R(q_mean)


def _rotation_diff_deg(R_ref: np.ndarray, R: np.ndarray) -> float:
    """두 회전 행렬 사이의 axis-angle 각도 (degree)."""
    R_diff = R @ R_ref.T
    cos = (np.trace(R_diff) - 1.0) * 0.5
    cos = float(np.clip(cos, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))


def _log_method_compare(compare: list[dict]) -> None:
    if not compare:
        return
    ref_name = next((c["method"]
                    for c in compare if c.get("ref")), compare[0]["method"])
    logger.info("─── method 비교 (기준: %s) ───", ref_name)
    for c in compare:
        if c.get("ref"):
            logger.info(
                "  %-12s  Δrot=  0.000°  Δt=  0.0mm  (기준)", c["method"])
        else:
            logger.info(
                "  %-12s  Δrot=%6.3f°  Δt=%5.1fmm",
                c["method"],
                c["drot_deg"],
                c["dt_mm"],
            )


def _make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """3x3 R + 3x1 t → 4x4 homogeneous."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T
