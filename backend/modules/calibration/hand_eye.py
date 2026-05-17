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
