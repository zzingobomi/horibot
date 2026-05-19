import cv2
import numpy as np
import logging
import time
from pathlib import Path
from dataclasses import dataclass, field

from scipy.stats import median_abs_deviation

from . import thresholds as T
from .bundle_adjust import BundleAdjustResult, FkFn, bundle_adjust_hand_eye
from .coach import diagnose
from .se3 import make_T

logger = logging.getLogger(__name__)


@dataclass
class HandEyeResult:
    R_cam2gripper: np.ndarray  # 회전 행렬 (카메라 → 그리퍼)
    t_cam2gripper: np.ndarray  # 이동 벡터
    method: str  # "BA(huber)" / "TSAI" 등 — 최종 채택된 방법


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

    def calibrate_cv2(self, method: int = cv2.CALIB_HAND_EYE_TSAI) -> HandEyeResult | None:
        """cv2.calibrateHandEye 단독 실행 — BA의 seed 또는 비교용."""
        if len(self.poses) < 3:
            logger.warning(f"포즈 부족: {len(self.poses)}개 (최소 3개 필요)")
            return None

        R, t = cv2.calibrateHandEye(
            [p.R_gripper2base for p in self.poses],
            [p.t_gripper2base for p in self.poses],
            [p.R_target2cam for p in self.poses],
            [p.t_target2cam for p in self.poses],
            method=method,
        )
        return HandEyeResult(
            R_cam2gripper=R,
            t_cam2gripper=t,
            method=_METHOD_NAMES.get(method, "UNKNOWN"),
        )

    def compute_with_diagnostics(
        self,
        *,
        fk_fn: FkFn,
        arm_motor_ids: list[int],
        estimate_joint_offsets: bool = True,
    ) -> dict | None:
        """파이프라인: cv2 TSAI seed → BA → outlier 자동 제거 → 깨끗한 set 재BA → 진단.

        DIY 컨텍스트:
            FK 자체에 systematic 오차(zero offset, 링크 톨러런스, 중력 sag)가
            있어 BA 전 잔차가 다 같이 1~2° 떠 있음. BA가 joint_offset도 같이
            추정해 FK floor를 흡수한 *후* 잔차 분포를 보고 procedural outlier
            (모션블러 / PnP 오검출 / 로봇 이동중 캡처)만 골라내는 게 정직함.

        Outlier 판단:
            (a) Iglewicz-Hoaglin modified Z-score > THRESHOLD  OR
            (b) 절대 잔차 > (OUTLIER_ABS_ROT_DEG, OUTLIER_ABS_T_MM)
            추가 가드:
            - 제거 비율 > OUTLIER_REMOVAL_CAP_RATIO 이면 자동 제거 중단
              (= BA가 FK floor 흡수 못 함 → 자세 다양성 추가가 옳음)

        반환 dict 구조 (프론트 ComputeData와 정렬):
            method, ba_converged, R/t_cam2gripper,
            sigma_rot_deg / sigma_t_mm : **RMS on clean set** (정직한 정확도)
            per_pose_residual: [{id, drot_deg, dt_mm, excluded}]
            excluded_pose_ids: 자동 제외된 id 리스트 (사용자 정보용)
            joint_offset_*, method_compare, coach, pose_count
        """
        if len(self.poses) < T.MIN_POSES_FOR_COMPUTE:
            logger.warning(
                f"포즈 부족: {len(self.poses)}개 (최소 {T.MIN_POSES_FOR_COMPUTE}개 필요)"
            )
            return None

        # ── 1. cv2 seed (TSAI) ───────────────────────────────────
        seed = self.calibrate_cv2(cv2.CALIB_HAND_EYE_TSAI)
        if seed is None:
            return None

        # ── 2. cv2 method 비교 (self-consistency 진단용) ─────────
        method_compare = self._compute_method_compare()

        # ── 3. 1차 BA (X + joint offset) — outlier 식별용 ────────
        ba_first = self._run_ba(
            poses=self.poses,
            seed=seed,
            fk_fn=fk_fn,
            estimate_joint_offsets=estimate_joint_offsets,
        )

        excluded_ids: list[int] = []
        cap_hit = False
        if ba_first is not None and ba_first.success:
            excluded_ids, cap_hit = self._identify_outliers(
                self.poses,
                ba_first.residual_rot_deg,
                ba_first.residual_t_mm,
            )

        # ── 4. 깨끗한 set으로 재BA (outlier가 있을 때만) ──────────
        clean_poses = [p for p in self.poses if p.id not in set(excluded_ids)]
        if (
            excluded_ids
            and len(clean_poses) >= T.MIN_POSES_FOR_COMPUTE
            and ba_first is not None
        ):
            ba_final = self._run_ba(
                poses=clean_poses,
                seed=seed,
                fk_fn=fk_fn,
                estimate_joint_offsets=estimate_joint_offsets,
            )
        else:
            ba_final = ba_first

        # ── 5. 최종 X / 잔차 / σ 결정 ────────────────────────────
        joint_offset_rad = np.zeros(len(arm_motor_ids), dtype=np.float64)
        joint_offsets_estimated = False

        if ba_final is not None and ba_final.success:
            final_R = ba_final.R_cam2gripper
            final_t = ba_final.t_cam2gripper.reshape(3, 1)
            method_name = (
                "BA(huber, +offset)"
                if ba_final.n_joint_vars > 0
                else "BA(huber)"
            )
            # ba_final은 clean_poses(또는 outlier 없으면 self.poses)에 fit됨.
            # excluded 포즈의 잔차는 1차 BA 값 그대로 유지 — "왜 빠졌는지" 보여줘야 하니까.
            final_pose_ids = [p.id for p in clean_poses] if excluded_ids else [
                p.id for p in self.poses
            ]
            id_to_clean_idx = {pid: i for i, pid in enumerate(final_pose_ids)}

            per_pose: list[dict] = []
            for i, p in enumerate(self.poses):
                if p.id in id_to_clean_idx:
                    idx = id_to_clean_idx[p.id]
                    drot = float(ba_final.residual_rot_deg[idx])
                    dt_mm = float(ba_final.residual_t_mm[idx])
                    excl = False
                else:
                    # excluded — 1차 BA 잔차 사용 (왜 빠졌는지 표시)
                    drot = float(ba_first.residual_rot_deg[i]) if ba_first else 0.0
                    dt_mm = float(ba_first.residual_t_mm[i]) if ba_first else 0.0
                    excl = True
                per_pose.append(
                    {"id": p.id, "drot_deg": drot, "dt_mm": dt_mm, "excluded": excl}
                )

            # 헤드라인 σ는 깨끗한 set의 **RMS** — 정직한 정확도. excluded는 빠짐.
            clean_rot = ba_final.residual_rot_deg
            clean_t = ba_final.residual_t_mm
            sigma_rot = float(np.sqrt(np.mean(clean_rot**2)))
            sigma_t = float(np.sqrt(np.mean(clean_t**2)))

            ba_converged = True
            ba_message = ba_final.message
            if ba_final.n_joint_vars > 0:
                joint_offset_rad = ba_final.joint_offset_rad.copy()
                joint_offsets_estimated = True
        else:
            # BA 실패 시 cv2 seed로 fallback. outlier 제거 불가 (잔차 없음).
            final_R = seed.R_cam2gripper
            final_t = seed.t_cam2gripper
            method_name = "TSAI (BA fallback)"
            pairwise, sigma_rot, sigma_t = self._residuals_pairwise(
                final_R, final_t
            )
            per_pose = [{**r, "excluded": False} for r in pairwise]
            excluded_ids = []
            ba_converged = False
            ba_message = ba_final.message if ba_final is not None else "BA 미실행"

        self.result = HandEyeResult(
            R_cam2gripper=final_R,
            t_cam2gripper=final_t,
            method=method_name,
        )

        # joint_offset_rad는 arm_motor_ids 순서. 프론트엔드 표시용으로 id 페어로 변환.
        joint_offset_list = [
            {
                "motor_id": int(mid),
                "offset_deg": float(np.degrees(joint_offset_rad[i])),
                "offset_rad": float(joint_offset_rad[i]),
            }
            for i, mid in enumerate(arm_motor_ids[: len(joint_offset_rad)])
        ]

        # ── 6. coach 진단 ────────────────────────────────────────
        coach_report = diagnose(
            pose_count=len(self.poses),
            joint_angles_per_pose=[p.joint_angles_rad for p in self.poses],
            per_pose_residuals=per_pose,
            sigma_rot_deg=sigma_rot,
            sigma_t_mm=sigma_t,
            method_compare=method_compare,
            excluded_pose_ids=excluded_ids,
            excluded_cap_hit=cap_hit,
        )

        _log_method_compare(method_compare)
        logger.info(
            "─── per-pose 잔차 (σ_rot=%.3f°, σ_t=%.1fmm, method=%s, excluded=%s) ───",
            sigma_rot,
            sigma_t,
            method_name,
            excluded_ids,
        )
        logger.info("coach verdict: %s", coach_report.verdict)

        return {
            "R_cam2gripper": final_R.tolist(),
            "t_cam2gripper": final_t.flatten().tolist(),
            "method": method_name,
            "ba_converged": ba_converged,
            "ba_message": ba_message,
            "pose_count": len(self.poses),
            "method_compare": method_compare,
            "per_pose_residual": per_pose,
            "excluded_pose_ids": excluded_ids,
            "sigma_rot_deg": sigma_rot,
            "sigma_t_mm": sigma_t,
            "coach": coach_report.to_dict(),
            "joint_offset_estimated": joint_offsets_estimated,
            "joint_offset_delta": joint_offset_list,
        }

    def _run_ba(
        self,
        *,
        poses: list[Pose],
        seed: HandEyeResult,
        fk_fn: FkFn,
        estimate_joint_offsets: bool,
    ) -> BundleAdjustResult | None:
        """BA 한 번 실행 — try/except로 감싸 fail 시 None 반환."""
        try:
            return bundle_adjust_hand_eye(
                joint_angles_per_pose=[list(p.joint_angles_rad) for p in poses],
                R_target2cam=[p.R_target2cam for p in poses],
                t_target2cam=[p.t_target2cam for p in poses],
                X_init=(seed.R_cam2gripper, seed.t_cam2gripper),
                fk_fn=fk_fn,
                estimate_joint_offsets=estimate_joint_offsets,
            )
        except Exception as e:
            logger.exception("BA 실패: %s", e)
            return None

    def _identify_outliers(
        self,
        poses: list[Pose],
        residual_rot_deg: np.ndarray,
        residual_t_mm: np.ndarray,
    ) -> tuple[list[int], bool]:
        """Iglewicz-Hoaglin modified Z-score + 절대 임계 + 비율/다양성 가드.

        반환:
            (제거할 pose.id 리스트, cap_hit)
            가드에 걸리면 빈 리스트. cap_hit=True면 비율 초과로 제거 보류.
        """
        N = len(poses)
        if N < T.MIN_POSES_FOR_COMPUTE + 1:
            # 4개 미만이면 outlier 통계 자체가 무의미
            return [], False

        candidates: set[int] = set()

        # (a) 상대 임계 — modified Z-score (scipy MAD with scale='normal'은
        # 이미 1.4826 보정 → z = (x-med)/MAD가 표준 Iglewicz-Hoaglin과 동치).
        for arr in (residual_rot_deg, residual_t_mm):
            med = float(np.median(arr))
            # Iglewicz-Hoaglin: scale=1.4826로 정규화 → z = (x-med)/mad 가
            # 표준편차와 동치 (Gaussian 데이터 가정).
            mad = 1.4826 * float(median_abs_deviation(arr))
            if mad <= 0.0:
                continue
            z = (arr - med) / mad
            for i, zi in enumerate(z):
                if zi > T.OUTLIER_MOD_Z_THRESHOLD:
                    candidates.add(poses[i].id)

        # (b) 절대 임계 — TSDF 품질 기준
        for i, p in enumerate(poses):
            if (
                residual_rot_deg[i] > T.OUTLIER_ABS_ROT_DEG
                or residual_t_mm[i] > T.OUTLIER_ABS_T_MM
            ):
                candidates.add(p.id)

        if not candidates:
            return [], False

        # (c) 비율 가드 — 너무 많이 잘리면 BA가 FK floor 흡수 못한 신호.
        cap_hit = len(candidates) / N > T.OUTLIER_REMOVAL_CAP_RATIO
        if cap_hit:
            logger.warning(
                "outlier 후보 %d/%d (>%.0f%%) — 자동 제거 중단, 데이터 품질 문제",
                len(candidates),
                N,
                T.OUTLIER_REMOVAL_CAP_RATIO * 100,
            )
            return [], True

        # (d) 다양성 가드 — 제거 후 어떤 축의 std가 임계 미만으로 떨어지면 보류.
        # joint_offset 추정이 underdetermined가 되는 걸 막음.
        remaining = [p for p in poses if p.id not in candidates]
        if remaining and self._diversity_collapsed(remaining):
            logger.info(
                "outlier %d개 식별됐으나 제거 시 자세 다양성 무너짐 → 보류",
                len(candidates),
            )
            return [], False

        return sorted(candidates), False

    @staticmethod
    def _diversity_collapsed(poses: list[Pose]) -> bool:
        """남은 포즈들의 조인트 std가 다양성 임계 미만이면 True."""
        if not poses or not all(len(p.joint_angles_rad) >= 5 for p in poses):
            return False
        joints = np.array([p.joint_angles_rad[:5] for p in poses])
        std_deg = np.degrees(joints.std(axis=0))
        for s, thr in zip(std_deg, T.JOINT_DIVERSITY_THRESHOLD_DEG):
            if s < thr:
                return True
        return False

    def _compute_method_compare(self) -> list[dict]:
        """cv2 TSAI/PARK/DANIILIDIS 간 결과 차이 — self-consistency 진단용."""
        compare: list[dict] = []
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
                        {"method": name, "drot_deg": 0.0, "dt_mm": 0.0, "ref": True}
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
        return compare

    def _residuals_pairwise(
        self,
        R_cam2gripper: np.ndarray,
        t_cam2gripper: np.ndarray,
    ) -> tuple[list[dict], float, float]:
        """BA fallback 시 — 첫 포즈를 anchor로 한 편차 (구방식)."""
        T_cam2gripper = make_T(R_cam2gripper, t_cam2gripper.reshape(3))
        T_target2base_list: list[np.ndarray] = []
        for p in self.poses:
            T_gb = make_T(p.R_gripper2base, p.t_gripper2base.reshape(3))
            T_tc = make_T(p.R_target2cam, p.t_target2cam.reshape(3))
            T_target2base_list.append(T_gb @ T_cam2gripper @ T_tc)

        ref_R = T_target2base_list[0][:3, :3]
        mean_pos = np.mean([T[:3, 3] for T in T_target2base_list], axis=0)

        per_pose: list[dict] = []
        rot_devs: list[float] = []
        pos_devs: list[float] = []
        for p, T_pose in zip(self.poses, T_target2base_list):
            drot = _rotation_diff_deg(ref_R, T_pose[:3, :3])
            dt_mm = float(np.linalg.norm(T_pose[:3, 3] - mean_pos)) * 1000.0
            per_pose.append({"id": p.id, "drot_deg": drot, "dt_mm": dt_mm})
            rot_devs.append(drot)
            pos_devs.append(dt_mm)

        sigma_rot = float(np.std(rot_devs)) if rot_devs else 0.0
        sigma_t = float(np.std(pos_devs)) if pos_devs else 0.0
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
        self.poses.clear()
        self.result = None
        self._next_id = 0

    # ── 포즈 영구화 ──────────────────────────────────────────
    # 포즈 자체는 thresholds와 무관한 raw 관측 데이터. 디스크에 두면 백엔드
    # 재시작/threshold 튜닝 후에도 재캡처 없이 COMPUTE만 다시 실행 가능.

    def save_poses(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if not self.poses:
            # 빈 상태도 명시적으로 저장 (리셋 후 빈 파일 남기지 않으려면 삭제 측에서 처리)
            np.savez(
                str(path),
                R_gripper2base=np.empty((0, 3, 3)),
                t_gripper2base=np.empty((0, 3, 1)),
                R_target2cam=np.empty((0, 3, 3)),
                t_target2cam=np.empty((0, 3, 1)),
                ids=np.empty((0,), dtype=np.int64),
                timestamps=np.empty((0,), dtype=np.float64),
                joint_angles_rad=np.empty((0, 0), dtype=np.float64),
                next_id=np.int64(self._next_id),
            )
            return
        J = len(self.poses[0].joint_angles_rad)
        np.savez(
            str(path),
            R_gripper2base=np.stack([p.R_gripper2base for p in self.poses]),
            t_gripper2base=np.stack(
                [np.asarray(p.t_gripper2base).reshape(3, 1) for p in self.poses]
            ),
            R_target2cam=np.stack([p.R_target2cam for p in self.poses]),
            t_target2cam=np.stack(
                [np.asarray(p.t_target2cam).reshape(3, 1) for p in self.poses]
            ),
            ids=np.array([p.id for p in self.poses], dtype=np.int64),
            timestamps=np.array(
                [p.timestamp for p in self.poses], dtype=np.float64
            ),
            joint_angles_rad=np.array(
                [p.joint_angles_rad for p in self.poses], dtype=np.float64
            ).reshape(len(self.poses), J),
            next_id=np.int64(self._next_id),
        )

    def load_poses(self, path: str | Path) -> int:
        """디스크에서 포즈 복원. 반환: 로드된 포즈 수."""
        path = Path(path)
        if not path.exists():
            return 0
        data = np.load(str(path))
        ids = data["ids"]
        if len(ids) == 0:
            self._next_id = int(data["next_id"]) if "next_id" in data else 0
            return 0
        R_gb = data["R_gripper2base"]
        t_gb = data["t_gripper2base"]
        R_tc = data["R_target2cam"]
        t_tc = data["t_target2cam"]
        ts = data["timestamps"]
        ja = data["joint_angles_rad"]
        self.poses = [
            Pose(
                R_gripper2base=R_gb[i],
                t_gripper2base=t_gb[i],
                R_target2cam=R_tc[i],
                t_target2cam=t_tc[i],
                id=int(ids[i]),
                timestamp=float(ts[i]),
                joint_angles_rad=ja[i].tolist(),
            )
            for i in range(len(ids))
        ]
        self._next_id = (
            int(data["next_id"]) if "next_id" in data else int(ids.max()) + 1
        )
        return len(self.poses)


def _rotation_diff_deg(R_ref: np.ndarray, R: np.ndarray) -> float:
    """두 회전 행렬 사이의 axis-angle 각도 (degree)."""
    R_diff = R @ R_ref.T
    cos = (np.trace(R_diff) - 1.0) * 0.5
    cos = float(np.clip(cos, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))


def _log_method_compare(compare: list[dict]) -> None:
    if not compare:
        return
    ref_name = next(
        (c["method"] for c in compare if c.get("ref")), compare[0]["method"]
    )
    logger.info("─── cv2 method 비교 (기준: %s) ───", ref_name)
    for c in compare:
        if c.get("ref"):
            logger.info(
                "  %-12s  Δrot=  0.000°  Δt=  0.0mm  (기준)", c["method"]
            )
        else:
            logger.info(
                "  %-12s  Δrot=%6.3f°  Δt=%5.1fmm",
                c["method"],
                c["drot_deg"],
                c["dt_mm"],
            )
