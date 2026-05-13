import cv2
import numpy as np
import logging
from pathlib import Path
from dataclasses import dataclass

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


class HandEyeCalibration:
    def __init__(self):
        self.poses: list[Pose] = []
        self.result: HandEyeResult | None = None

    def add_pose(self, pose: Pose) -> None:
        self.poses.append(pose)
        logger.info(f"포즈 추가됨 ({len(self.poses)}개)")

    def calibrate(self, method: int = cv2.CALIB_HAND_EYE_TSAI) -> HandEyeResult | None:
        if len(self.poses) < 3:
            logger.warning(f"포즈 부족: {len(self.poses)}개 (최소 3개 필요)")
            return None

        R_gripper2base = [p.R_gripper2base for p in self.poses]
        t_gripper2base = [p.t_gripper2base for p in self.poses]
        R_target2cam = [p.R_target2cam for p in self.poses]
        t_target2cam = [p.t_target2cam for p in self.poses]

        method_name = {
            cv2.CALIB_HAND_EYE_TSAI: "TSAI",
            cv2.CALIB_HAND_EYE_PARK: "PARK",
            cv2.CALIB_HAND_EYE_HORAUD: "HORAUD",
            cv2.CALIB_HAND_EYE_ANDREFF: "ANDREFF",
            cv2.CALIB_HAND_EYE_DANIILIDIS: "DANIILIDIS",
        }

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
            method=method_name.get(method, "UNKNOWN"),
        )
        logger.info(
            f"Hand-Eye 캘리브레이션 완료 (method={self.result.method}, "
            f"poses={len(self.poses)})"
        )

        # multi-method 진단 — 같은 데이터로 다른 method도 풀어 self-consistency 확인.
        # 셋이 1° 이내면 데이터 일관성 OK(자세 추가 필요), 크면 자세 다양성/품질 부족.
        compare_methods = [
            cv2.CALIB_HAND_EYE_TSAI,
            cv2.CALIB_HAND_EYE_PARK,
            cv2.CALIB_HAND_EYE_DANIILIDIS,
        ]
        results: list[tuple[str, np.ndarray, np.ndarray]] = []
        for m in compare_methods:
            try:
                Rm, tm = cv2.calibrateHandEye(
                    R_gripper2base,
                    t_gripper2base,
                    R_target2cam,
                    t_target2cam,
                    method=m,
                )
                results.append((method_name.get(m, "?"), Rm, tm))
            except cv2.error as e:
                logger.warning(f"  {method_name.get(m, '?')} 실패: {e}")

        if results:
            ref_name, ref_R, ref_t = results[0]
            logger.info("─── method 비교 (기준: %s) ───", ref_name)
            logger.info(
                "  %-12s  Δrot=  0.000°  Δt=  0.0mm  (기준)", ref_name
            )
            for name, Rm, tm in results[1:]:
                drot_deg = _rotation_diff_deg(ref_R, Rm)
                dt_mm = float(np.linalg.norm(ref_t - tm)) * 1000.0
                logger.info(
                    "  %-12s  Δrot=%6.3f°  Δt=%5.1fmm",
                    name, drot_deg, dt_mm,
                )
            logger.info(
                "  해석: Δrot이 셋 다 1° 미만이면 데이터 self-consistent — "
                "자세 추가/체커보드 점검 방향. 크면 자세 다양성/품질 부족."
            )

        return self.result

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
