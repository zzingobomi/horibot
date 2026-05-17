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
        self.poses: list[Pose] = []
        self.result: HandEyeResult | None = None

    def add_pose(self, pose: Pose) -> None:
        self.poses.append(pose)
        logger.info(f"포즈 추가됨 ({len(self.poses)}개)")

    def remove_pose(self, index: int) -> bool:
        if not (0 <= index < len(self.poses)):
            return False
        del self.poses[index]
        logger.info(f"포즈 #{index} 제거됨 (남은 포즈: {len(self.poses)}개)")
        return True

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

        return {
            "R_cam2gripper": result.R_cam2gripper.tolist(),
            "t_cam2gripper": result.t_cam2gripper.flatten().tolist(),
            "method": result.method,
            "pose_count": len(self.poses),
            "method_compare": compare,
            "per_pose_residual": per_pose,
            "sigma_rot_deg": sigma_rot_deg,
            "sigma_t_mm": sigma_t_mm,
        }

    def validate(
        self, R_cam2gripper: np.ndarray, t_cam2gripper: np.ndarray
    ) -> dict | None:
        """주어진 hand-eye 행렬로 누적된 포즈들의 T_target←base 분산을 계산.

        cv2.calibrateHandEye 결과(자기 자신) 검증뿐 아니라, 저장된 .npz나
        Bundle Adjustment 결과 같은 임의의 R/t로도 검증할 때 사용.
        """
        if len(self.poses) < 2:
            return None
        per_pose, sigma_rot, sigma_t = self._compute_residuals(
            R_cam2gripper, t_cam2gripper
        )
        return {
            "pose_count": len(self.poses),
            "per_pose_residual": per_pose,
            "sigma_rot_deg": sigma_rot,
            "sigma_t_mm": sigma_t,
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

        # 평균 회전 / 평균 위치
        positions = np.array([T[:3, 3] for T in T_target2base_list])
        mean_pos = positions.mean(axis=0)

        # 회전 평균은 quaternion 평균이 정확하지만, 보통 분산은 작으므로
        # 첫 포즈를 기준으로 한 axis-angle 편차로 측정 (덜 비싸고 충분히 진단적)
        ref_R = T_target2base_list[0][:3, :3]

        per_pose: list[dict] = []
        rot_devs: list[float] = []
        pos_devs: list[float] = []
        for i, T in enumerate(T_target2base_list):
            drot = _rotation_diff_deg(ref_R, T[:3, :3])
            dt_mm = float(np.linalg.norm(T[:3, 3] - mean_pos)) * 1000.0
            per_pose.append({"index": i, "drot_deg": drot, "dt_mm": dt_mm})
            rot_devs.append(drot)
            pos_devs.append(dt_mm)

        sigma_rot = float(np.std(rot_devs)) if rot_devs else 0.0
        sigma_t = float(np.std(pos_devs)) if pos_devs else 0.0
        return per_pose, sigma_rot, sigma_t

    def list_poses_meta(self) -> list[dict]:
        return [
            {
                "index": i,
                "timestamp": p.timestamp,
                "joint_angles_rad": p.joint_angles_rad,
            }
            for i, p in enumerate(self.poses)
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
        self.poses.clear()
        self.result = None


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
