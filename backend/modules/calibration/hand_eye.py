import cv2
import numpy as np
import logging
import time
from pathlib import Path
from dataclasses import dataclass, field

from scipy.stats import median_abs_deviation

from core.coords.joint_coordinates import JointCoordinates
from modules.motor.motor_config import MotorConfig

from . import thresholds as T
from .bundle_adjust import (
    BundleAdjustExtendedResult,
    BundleAdjustPhysicalSagResult,
    BundleAdjustResult,
    FkFn,
    bundle_adjust_hand_eye,
    bundle_adjust_hand_eye_extended,
    bundle_adjust_hand_eye_physical_sag,
)
from .coach import diagnose
from .se3 import make_T

# sag 모델은 J2, J3에만 적용 (motor id 2, 3). bundle_adjust의 sag_k_rad_per_m
# (2,) 배열의 순서와 일치. solver.py의 _SAG_JOINT_IDS와 같은 정의.
_SAG_MOTOR_IDS: list[int] = [2, 3]

# 결과 dispatch 시 사용. Union 매번 풀어쓰는 것 방지.
_BaResultT = (
    BundleAdjustResult | BundleAdjustExtendedResult | BundleAdjustPhysicalSagResult
)

logger = logging.getLogger(__name__)


@dataclass
class HandEyeResult:
    R_cam2gripper: np.ndarray  # 회전 행렬 (카메라 → 그리퍼)
    t_cam2gripper: np.ndarray  # 이동 벡터
    method: str  # "BA(huber)" / "TSAI" 등 — 최종 채택된 방법


@dataclass
class Pose:
    """캡처 시점의 raw 측정값. URDF rad / FK는 베이크하지 않음.

    이번 캘 세션 안에서 [계산]을 여러 번 누를 때 *그때마다의 시스템 offset*으로
    모든 포즈가 일관되게 재해석되도록 raw만 보관. 옛 라운드와 새 라운드 포즈가
    같은 baseline 위에서 풀려야 BA가 잔여 delta만 추정 → σ 수렴.
    """
    # arm 모터 id → raw position. 시점 독립.
    raw_motor_positions: dict[int, int]
    R_target2cam: np.ndarray  # 체커보드 → 카메라 회전
    t_target2cam: np.ndarray  # 체커보드 → 카메라 이동
    id: int = -1
    timestamp: float = field(default_factory=time.time)


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
        # id는 항상 내부 카운터가 부여. load_poses는 add_pose 우회하고 self.poses에
        # 직접 채워 디스크 id를 보존하므로 여기는 새 캡처 전용.
        pose.id = self._next_id
        self._next_id += 1
        self.poses.append(pose)
        logger.info(f"포즈 #{pose.id} 추가됨 ({len(self.poses)}개)")

    def _resolve_pose_arrays(
        self,
        *,
        arm_motor_cfgs: list[MotorConfig],
        fk_fn: FkFn,
    ) -> tuple[
        list[int],  # pose_ids
        list[list[float]],  # joint_angles_per_pose (URDF rad, 현재 offset 적용 후)
        list[np.ndarray],  # R_gripper2base
        list[np.ndarray],  # t_gripper2base (3,1)
        list[np.ndarray],  # R_target2cam
        list[np.ndarray],  # t_target2cam (3,1)
    ]:
        """매 COMPUTE 시점에 *현재 JointCoordinates offset*으로 모든 포즈를 재해석.

        포즈는 raw motor + R/t_target2cam만 영구 저장. URDF rad / FK 결과는
        시스템 상태(offset) 의존이라 매번 새로 계산. 이게 반복 캘이 작동하는 핵심:
        라운드 1에서 추정/커밋된 offset이 라운드 2 COMPUTE에서 *모든 옛 포즈*에
        같이 적용되어 baseline이 항상 통일됨.
        """
        coords = JointCoordinates()
        pose_ids: list[int] = []
        ja_list: list[list[float]] = []
        R_gb_list: list[np.ndarray] = []
        t_gb_list: list[np.ndarray] = []
        R_tc_list: list[np.ndarray] = []
        t_tc_list: list[np.ndarray] = []
        for p in self.poses:
            angles: list[float] = []
            for cfg in arm_motor_cfgs:
                raw = p.raw_motor_positions.get(cfg.id)
                if raw is None:
                    raise ValueError(
                        f"포즈 #{p.id}에 모터 {cfg.id} raw 없음 — 데이터 손상"
                    )
                angles.append(coords.motor_to_urdf(int(raw), cfg))
            R_gb, t_gb = fk_fn(angles)
            pose_ids.append(p.id)
            ja_list.append(angles)
            R_gb_list.append(np.asarray(R_gb, dtype=np.float64))
            t_gb_list.append(np.asarray(t_gb, dtype=np.float64).reshape(3, 1))
            R_tc_list.append(np.asarray(p.R_target2cam, dtype=np.float64))
            t_tc_list.append(np.asarray(p.t_target2cam, dtype=np.float64).reshape(3, 1))
        return pose_ids, ja_list, R_gb_list, t_gb_list, R_tc_list, t_tc_list

    def compute_with_diagnostics(
        self,
        *,
        fk_fn: FkFn,
        arm_motor_cfgs: list[MotorConfig],
        joint_limits_rad: list[tuple[float, float]],
        estimate_joint_offsets: bool = True,
        use_extended_ba: bool = False,
        use_physical_sag: bool = False,
    ) -> dict | None:
        """파이프라인: 매 COMPUTE 시점 *현재 offset*으로 포즈 재해석 →
        cv2 multi-seed BA → outlier 자동 제거 → 깨끗한 set 재BA → 진단.

        Args:
            estimate_joint_offsets: 기존 BA(11자유도)에서 joint_offset도 풀지.
                use_extended_ba/use_physical_sag=True면 무시 (각각 항상 joint_offset 추정).
            use_extended_ba: True면 확장 BA(41자유도) 사용.
                joint_offset + link_trans + link_rot + R/t 동시 추정.
                fk_fn 대신 modules.kinematics.fk_chain의 numpy 체인을 내부 호출
                (PybulletKinematics는 URDF 고정이라 link_offset 변수화 불가능).
            use_physical_sag: True면 확장 BA + 자세 의존 sag (43자유도) 사용.
                위 + sag_k_J2, sag_k_J3 동시 추정. lumped mass + 모멘트 암 기반.
                σ_rot ~0.65° / σ_t ~7.9mm 달성 가능 (vs extended_ba 1.30°/9.3mm).
                use_extended_ba와 동시 True면 use_physical_sag가 우선.

        반복 캘 작동 보장:
            포즈는 raw로 저장되어 *영속*. 매번 _resolve_pose_arrays가 *현재*
            JointCoordinates offset으로 URDF rad를 새로 생성. fk_fn (solver.fk_to_matrix)
            은 cv2.calibrateHandEye의 seed 산출에만 사용 (R_gripper2base) — BA 내부
            FK는 fk_chain (original URDF + 변수 link_t/link_r/sag_k) 사용.

            출력값 semantics (commit 시 적용 방식 결정):
              - joint_offset_rad     : **delta** — ja에 disk값 이미 가산됨 → cumulative
              - link_trans_m / rot   : **absolute total** (original URDF 기준) → overwrite
              - sag_k_rad_per_m      : **absolute total** → overwrite
              - R/t_cam2gripper      : absolute → overwrite

            라운드가 거듭되면:
              - joint_offset_delta는 점점 0으로 (cumulative 누적이 수렴)
              - link_t/sag_k는 absolute 값 자체가 안정값으로 수렴 (각 라운드마다 다시 채택)
              - σ가 system floor에 수렴

            (참조: docs/accuracy_squeeze_plan.md §1.6 — 과거 link/sag도 cumulative였으나
             absolute 출력값을 누적해 손상되던 버그를 2026-05-28 overwrite로 fix.)

        Outlier 판단 + 가드는 기존과 동일 (thresholds.py).

        반환 dict 구조 (프론트 ComputeData와 정렬):
            method, ba_converged, R/t_cam2gripper,
            sigma_rot_deg / sigma_t_mm : **RMS on clean set**
            per_pose_residual, excluded_pose_ids,
            joint_offset_*, link_offset_*(확장 BA시),
            method_compare, coach, pose_count
        """
        if len(self.poses) < T.MIN_POSES_FOR_COMPUTE:
            logger.warning(
                f"포즈 부족: {len(self.poses)}개 (최소 {T.MIN_POSES_FOR_COMPUTE}개 필요)"
            )
            return None

        arm_motor_ids = [cfg.id for cfg in arm_motor_cfgs]

        # ── 0. 현재 baseline으로 포즈 재해석 ─────────────────────
        pose_ids, ja_list, R_gb_list, t_gb_list, R_tc_list, t_tc_list = (
            self._resolve_pose_arrays(
                arm_motor_cfgs=arm_motor_cfgs, fk_fn=fk_fn
            )
        )

        # ── 1. cv2 method 비교 (self-consistency 진단용) ─────────
        method_compare = self._compute_method_compare_lists(
            R_gb_list, t_gb_list, R_tc_list, t_tc_list
        )

        # ── 2. 1차 BA (multiseed) — outlier 식별용 ────────────────
        # cv2 seed는 _multiseed_ba_lists 내부에서 TSAI/PARK/DANIILIDIS로 직접 생성.
        ba_first: _BaResultT | None
        if use_physical_sag:
            ba_first, ba_first_seed = self._multiseed_ba_physical_sag_lists(
                ja_list=ja_list,
                R_gb_list=R_gb_list,
                t_gb_list=t_gb_list,
                R_tc_list=R_tc_list,
                t_tc_list=t_tc_list,
            )
        elif use_extended_ba:
            ba_first, ba_first_seed = self._multiseed_ba_extended_lists(
                ja_list=ja_list,
                R_gb_list=R_gb_list,
                t_gb_list=t_gb_list,
                R_tc_list=R_tc_list,
                t_tc_list=t_tc_list,
            )
        else:
            ba_first, ba_first_seed = self._multiseed_ba_lists(
                ja_list=ja_list,
                R_gb_list=R_gb_list,
                t_gb_list=t_gb_list,
                R_tc_list=R_tc_list,
                t_tc_list=t_tc_list,
                fk_fn=fk_fn,
                estimate_joint_offsets=estimate_joint_offsets,
            )

        excluded_ids: list[int] = []
        cap_hit = False
        if ba_first is not None and ba_first.success:
            excluded_ids, cap_hit = self._identify_outliers_lists(
                pose_ids=pose_ids,
                joint_angles_list=ja_list,
                residual_rot_deg=ba_first.residual_rot_deg,
                residual_t_mm=ba_first.residual_t_mm,
            )

        # ── 4. 깨끗한 set으로 재BA (outlier가 있을 때만) ──────────
        excl_set = set(excluded_ids)
        clean_idx = [i for i, pid in enumerate(pose_ids) if pid not in excl_set]
        ba_final: _BaResultT | None
        if (
            excluded_ids
            and len(clean_idx) >= T.MIN_POSES_FOR_COMPUTE
            and ba_first is not None
        ):
            if use_physical_sag:
                ba_final, ba_final_seed = self._multiseed_ba_physical_sag_lists(
                    ja_list=[ja_list[i] for i in clean_idx],
                    R_gb_list=[R_gb_list[i] for i in clean_idx],
                    t_gb_list=[t_gb_list[i] for i in clean_idx],
                    R_tc_list=[R_tc_list[i] for i in clean_idx],
                    t_tc_list=[t_tc_list[i] for i in clean_idx],
                )
            elif use_extended_ba:
                ba_final, ba_final_seed = self._multiseed_ba_extended_lists(
                    ja_list=[ja_list[i] for i in clean_idx],
                    R_gb_list=[R_gb_list[i] for i in clean_idx],
                    t_gb_list=[t_gb_list[i] for i in clean_idx],
                    R_tc_list=[R_tc_list[i] for i in clean_idx],
                    t_tc_list=[t_tc_list[i] for i in clean_idx],
                )
            else:
                ba_final, ba_final_seed = self._multiseed_ba_lists(
                    ja_list=[ja_list[i] for i in clean_idx],
                    R_gb_list=[R_gb_list[i] for i in clean_idx],
                    t_gb_list=[t_gb_list[i] for i in clean_idx],
                    R_tc_list=[R_tc_list[i] for i in clean_idx],
                    t_tc_list=[t_tc_list[i] for i in clean_idx],
                    fk_fn=fk_fn,
                    estimate_joint_offsets=estimate_joint_offsets,
                )
        else:
            ba_final, ba_final_seed = ba_first, ba_first_seed

        # ── 5. 최종 X / 잔차 / σ 결정 ────────────────────────────
        joint_offset_rad = np.zeros(len(arm_motor_ids), dtype=np.float64)
        joint_offsets_estimated = False
        link_trans_delta = np.zeros((5, 3), dtype=np.float64)
        link_rot_delta = np.zeros((5, 3), dtype=np.float64)
        link_offsets_estimated = False
        sag_k_rad_per_m = np.zeros(len(_SAG_MOTOR_IDS), dtype=np.float64)
        max_sag_deg = np.zeros(len(_SAG_MOTOR_IDS), dtype=np.float64)
        sag_offsets_estimated = False

        if ba_final is not None and ba_final.success:
            final_R = ba_final.R_cam2gripper
            final_t = ba_final.t_cam2gripper.reshape(3, 1)
            if isinstance(ba_final, BundleAdjustPhysicalSagResult):
                method_name = f"BA(+offset+link+sag, seed={ba_final_seed})"
            elif isinstance(ba_final, BundleAdjustExtendedResult):
                method_name = f"BA(+offset+link, seed={ba_final_seed})"
            elif ba_final.n_joint_vars > 0:
                method_name = f"BA(+offset, seed={ba_final_seed})"
            else:
                method_name = f"BA(seed={ba_final_seed})"
            # ba_final은 clean set에 fit. excluded 포즈의 잔차는 1차 BA 값 유지.
            final_pose_ids = (
                [pose_ids[i] for i in clean_idx] if excluded_ids else pose_ids
            )
            id_to_clean_idx = {pid: i for i, pid in enumerate(final_pose_ids)}

            per_pose: list[dict] = []
            for i, pid in enumerate(pose_ids):
                if pid in id_to_clean_idx:
                    idx = id_to_clean_idx[pid]
                    drot = float(ba_final.residual_rot_deg[idx])
                    dt_mm = float(ba_final.residual_t_mm[idx])
                    excl = False
                else:
                    drot = float(ba_first.residual_rot_deg[i]) if ba_first else 0.0
                    dt_mm = float(ba_first.residual_t_mm[i]) if ba_first else 0.0
                    excl = True
                per_pose.append(
                    {"id": pid, "drot_deg": drot, "dt_mm": dt_mm, "excluded": excl}
                )

            sigma_rot = float(np.sqrt(np.mean(ba_final.residual_rot_deg**2)))
            sigma_t = float(np.sqrt(np.mean(ba_final.residual_t_mm**2)))

            ba_converged = True
            ba_message = ba_final.message
            if isinstance(ba_final, BundleAdjustPhysicalSagResult):
                joint_offset_rad = ba_final.joint_offset_rad.copy()
                joint_offsets_estimated = True
                link_trans_delta = ba_final.link_trans_m.copy()
                link_rot_delta = ba_final.link_rot_rad.copy()
                link_offsets_estimated = True
                sag_k_rad_per_m = ba_final.sag_k_rad_per_m.copy()
                max_sag_deg = ba_final.max_sag_deg.copy()
                sag_offsets_estimated = True
            elif isinstance(ba_final, BundleAdjustExtendedResult):
                joint_offset_rad = ba_final.joint_offset_rad.copy()
                joint_offsets_estimated = True
                link_trans_delta = ba_final.link_trans_m.copy()
                link_rot_delta = ba_final.link_rot_rad.copy()
                link_offsets_estimated = True
            elif ba_final.n_joint_vars > 0:
                joint_offset_rad = ba_final.joint_offset_rad.copy()
                joint_offsets_estimated = True
        else:
            # BA 3 seed 모두 실패 — 마지막 수단으로 TSAI raw 결과 사용.
            # cv2.error가 나면 데이터가 진짜 망가진 상황이라 그대로 propagate.
            final_R, final_t = cv2.calibrateHandEye(
                R_gb_list, t_gb_list, R_tc_list, t_tc_list,
                method=cv2.CALIB_HAND_EYE_TSAI,
            )
            method_name = "TSAI (BA fallback)"
            pairwise, sigma_rot, sigma_t = self._residuals_pairwise_lists(
                final_R,
                final_t,
                pose_ids=pose_ids,
                R_gb_list=R_gb_list,
                t_gb_list=t_gb_list,
                R_tc_list=R_tc_list,
                t_tc_list=t_tc_list,
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

        joint_offset_list = [
            {
                "motor_id": int(mid),
                "offset_deg": float(np.degrees(joint_offset_rad[i])),
                "offset_rad": float(joint_offset_rad[i]),
            }
            for i, mid in enumerate(arm_motor_ids[: len(joint_offset_rad)])
        ]
        # 확장 BA에서만 채워짐. 기존 BA(11 DOF)면 모두 0.
        n_link = min(5, len(arm_motor_ids))
        link_trans_list = [
            {
                "motor_id": int(arm_motor_ids[i]),
                "x_mm": float(link_trans_delta[i, 0] * 1000.0),
                "y_mm": float(link_trans_delta[i, 1] * 1000.0),
                "z_mm": float(link_trans_delta[i, 2] * 1000.0),
                "x_m": float(link_trans_delta[i, 0]),
                "y_m": float(link_trans_delta[i, 1]),
                "z_m": float(link_trans_delta[i, 2]),
            }
            for i in range(n_link)
        ]
        link_rot_list = [
            {
                "motor_id": int(arm_motor_ids[i]),
                "rx_deg": float(np.degrees(link_rot_delta[i, 0])),
                "ry_deg": float(np.degrees(link_rot_delta[i, 1])),
                "rz_deg": float(np.degrees(link_rot_delta[i, 2])),
                "rx_rad": float(link_rot_delta[i, 0]),
                "ry_rad": float(link_rot_delta[i, 1]),
                "rz_rad": float(link_rot_delta[i, 2]),
            }
            for i in range(n_link)
        ]
        # 물리 sag BA에서만 채워짐. extended/basic BA면 모두 0.
        sag_offset_list = [
            {
                "motor_id": int(mid),
                "k_rad_per_m": float(sag_k_rad_per_m[i]),
                "max_sag_deg": float(max_sag_deg[i]),
            }
            for i, mid in enumerate(_SAG_MOTOR_IDS)
        ]

        # ── 6. coach 진단 ────────────────────────────────────────
        coach_report = diagnose(
            pose_count=len(self.poses),
            joint_angles_per_pose=ja_list,
            per_pose_residuals=per_pose,
            sigma_rot_deg=sigma_rot,
            sigma_t_mm=sigma_t,
            method_compare=method_compare,
            excluded_pose_ids=excluded_ids,
            excluded_cap_hit=cap_hit,
            arm_motor_ids=arm_motor_ids,
            joint_limits_rad=joint_limits_rad,
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
            "link_offset_estimated": link_offsets_estimated,
            "link_trans_delta": link_trans_list,
            "link_rot_delta": link_rot_list,
            "sag_offset_estimated": sag_offsets_estimated,
            "sag_offset_delta": sag_offset_list,
            # planner가 다음 추천 산출에 사용 (잔차 기반 MVP)
            "joint_angles_per_pose": ja_list,
        }

    @staticmethod
    def _run_ba_lists(
        *,
        ja_list: list[list[float]],
        R_tc_list: list[np.ndarray],
        t_tc_list: list[np.ndarray],
        seed: HandEyeResult,
        fk_fn: FkFn,
        estimate_joint_offsets: bool,
    ) -> BundleAdjustResult | None:
        """BA 한 번 실행 — try/except로 감싸 fail 시 None 반환."""
        try:
            return bundle_adjust_hand_eye(
                joint_angles_per_pose=[list(a) for a in ja_list],
                R_target2cam=R_tc_list,
                t_target2cam=t_tc_list,
                X_init=(seed.R_cam2gripper, seed.t_cam2gripper),
                fk_fn=fk_fn,
                estimate_joint_offsets=estimate_joint_offsets,
            )
        except Exception as e:
            logger.exception("BA 실패: %s", e)
            return None

    @staticmethod
    def _run_ba_extended_lists(
        *,
        ja_list: list[list[float]],
        R_tc_list: list[np.ndarray],
        t_tc_list: list[np.ndarray],
        seed: HandEyeResult,
    ) -> BundleAdjustExtendedResult | None:
        """확장 BA 한 번 실행 (joint_offset + link_trans + link_rot 동시 추정)."""
        try:
            return bundle_adjust_hand_eye_extended(
                joint_angles_per_pose=[list(a) for a in ja_list],
                R_target2cam=R_tc_list,
                t_target2cam=[
                    np.asarray(t, dtype=np.float64).reshape(3) for t in t_tc_list
                ],
                X_init=(seed.R_cam2gripper, seed.t_cam2gripper),
            )
        except Exception as e:
            logger.exception("확장 BA 실패: %s", e)
            return None

    @staticmethod
    def _run_ba_physical_sag_lists(
        *,
        ja_list: list[list[float]],
        R_tc_list: list[np.ndarray],
        t_tc_list: list[np.ndarray],
        seed: HandEyeResult,
    ) -> BundleAdjustPhysicalSagResult | None:
        """물리 sag BA 한 번 실행 (extended + sag_k 동시 추정)."""
        try:
            return bundle_adjust_hand_eye_physical_sag(
                joint_angles_per_pose=[list(a) for a in ja_list],
                R_target2cam=R_tc_list,
                t_target2cam=[
                    np.asarray(t, dtype=np.float64).reshape(3) for t in t_tc_list
                ],
                X_init=(seed.R_cam2gripper, seed.t_cam2gripper),
            )
        except Exception as e:
            logger.exception("물리 sag BA 실패: %s", e)
            return None

    def _multiseed_ba_lists(
        self,
        *,
        ja_list: list[list[float]],
        R_gb_list: list[np.ndarray],
        t_gb_list: list[np.ndarray],
        R_tc_list: list[np.ndarray],
        t_tc_list: list[np.ndarray],
        fk_fn: FkFn,
        estimate_joint_offsets: bool,
    ) -> tuple[BundleAdjustResult | None, str | None]:
        """TSAI/PARK/DANIILIDIS 3개 seed로 BA 실행, cost 최소 채택."""
        best_ba: BundleAdjustResult | None = None
        best_seed_name: str | None = None
        for method in _COMPARE_METHODS:
            try:
                R, t = cv2.calibrateHandEye(
                    R_gb_list, t_gb_list, R_tc_list, t_tc_list, method=method
                )
            except cv2.error:
                continue
            method_name = _METHOD_NAMES.get(method, "UNKNOWN")
            seed = HandEyeResult(
                R_cam2gripper=R, t_cam2gripper=t, method=method_name
            )
            ba = self._run_ba_lists(
                ja_list=ja_list,
                R_tc_list=R_tc_list,
                t_tc_list=t_tc_list,
                seed=seed,
                fk_fn=fk_fn,
                estimate_joint_offsets=estimate_joint_offsets,
            )
            if ba is None or not ba.success:
                continue
            if best_ba is None or ba.cost < best_ba.cost:
                best_ba = ba
                best_seed_name = method_name
        if best_ba is not None:
            logger.info(
                "BA seed 선택: %s (cost=%.4f)", best_seed_name, best_ba.cost
            )
        return best_ba, best_seed_name

    def _multiseed_ba_extended_lists(
        self,
        *,
        ja_list: list[list[float]],
        R_gb_list: list[np.ndarray],
        t_gb_list: list[np.ndarray],
        R_tc_list: list[np.ndarray],
        t_tc_list: list[np.ndarray],
    ) -> tuple[BundleAdjustExtendedResult | None, str | None]:
        """확장 BA — TSAI/PARK/DANIILIDIS 3 seed로 실행 후 cost 최소 채택."""
        best_ba: BundleAdjustExtendedResult | None = None
        best_seed_name: str | None = None
        for method in _COMPARE_METHODS:
            try:
                R, t = cv2.calibrateHandEye(
                    R_gb_list, t_gb_list, R_tc_list, t_tc_list, method=method
                )
            except cv2.error:
                continue
            method_name = _METHOD_NAMES.get(method, "UNKNOWN")
            seed = HandEyeResult(
                R_cam2gripper=R, t_cam2gripper=t, method=method_name
            )
            ba = self._run_ba_extended_lists(
                ja_list=ja_list,
                R_tc_list=R_tc_list,
                t_tc_list=t_tc_list,
                seed=seed,
            )
            if ba is None or not ba.success:
                continue
            if best_ba is None or ba.cost < best_ba.cost:
                best_ba = ba
                best_seed_name = method_name
        if best_ba is not None:
            logger.info(
                "확장 BA seed 선택: %s (cost=%.4f)",
                best_seed_name,
                best_ba.cost,
            )
        return best_ba, best_seed_name

    def _multiseed_ba_physical_sag_lists(
        self,
        *,
        ja_list: list[list[float]],
        R_gb_list: list[np.ndarray],
        t_gb_list: list[np.ndarray],
        R_tc_list: list[np.ndarray],
        t_tc_list: list[np.ndarray],
    ) -> tuple[BundleAdjustPhysicalSagResult | None, str | None]:
        """물리 sag BA — TSAI/PARK/DANIILIDIS 3 seed로 실행 후 cost 최소 채택."""
        best_ba: BundleAdjustPhysicalSagResult | None = None
        best_seed_name: str | None = None
        for method in _COMPARE_METHODS:
            try:
                R, t = cv2.calibrateHandEye(
                    R_gb_list, t_gb_list, R_tc_list, t_tc_list, method=method
                )
            except cv2.error:
                continue
            method_name = _METHOD_NAMES.get(method, "UNKNOWN")
            seed = HandEyeResult(
                R_cam2gripper=R, t_cam2gripper=t, method=method_name
            )
            ba = self._run_ba_physical_sag_lists(
                ja_list=ja_list,
                R_tc_list=R_tc_list,
                t_tc_list=t_tc_list,
                seed=seed,
            )
            if ba is None or not ba.success:
                continue
            if best_ba is None or ba.cost < best_ba.cost:
                best_ba = ba
                best_seed_name = method_name
        if best_ba is not None:
            logger.info(
                "물리 sag BA seed 선택: %s (cost=%.4f)",
                best_seed_name,
                best_ba.cost,
            )
        return best_ba, best_seed_name

    def _identify_outliers_lists(
        self,
        *,
        pose_ids: list[int],
        joint_angles_list: list[list[float]],
        residual_rot_deg: np.ndarray,
        residual_t_mm: np.ndarray,
    ) -> tuple[list[int], bool]:
        """Iglewicz-Hoaglin modified Z-score + 절대 임계 + 비율/다양성 가드."""
        N = len(pose_ids)
        if N < T.MIN_POSES_FOR_COMPUTE + 1:
            return [], False

        candidates: set[int] = set()

        # (a) 상대 임계 — modified Z-score
        for arr in (residual_rot_deg, residual_t_mm):
            med = float(np.median(arr))
            mad = 1.4826 * float(median_abs_deviation(arr))
            if mad <= 0.0:
                continue
            z = (arr - med) / mad
            for i, zi in enumerate(z):
                if zi > T.OUTLIER_MOD_Z_THRESHOLD:
                    candidates.add(pose_ids[i])

        # (b) 절대 임계 — TSDF 품질 기준
        for i, pid in enumerate(pose_ids):
            if (
                residual_rot_deg[i] > T.OUTLIER_ABS_ROT_DEG
                or residual_t_mm[i] > T.OUTLIER_ABS_T_MM
            ):
                candidates.add(pid)

        if not candidates:
            return [], False

        cap_hit = len(candidates) / N > T.OUTLIER_REMOVAL_CAP_RATIO
        if cap_hit:
            logger.warning(
                "outlier 후보 %d/%d (>%.0f%%) — 자동 제거 중단, 데이터 품질 문제",
                len(candidates),
                N,
                T.OUTLIER_REMOVAL_CAP_RATIO * 100,
            )
            return [], True

        # (d) 다양성 가드
        remaining_ja = [
            joint_angles_list[i] for i, pid in enumerate(pose_ids) if pid not in candidates
        ]
        if remaining_ja and self._diversity_collapsed_lists(remaining_ja):
            logger.info(
                "outlier %d개 식별됐으나 제거 시 자세 다양성 무너짐 → 보류",
                len(candidates),
            )
            return [], False

        return sorted(candidates), False

    @staticmethod
    def _diversity_collapsed_lists(joint_angles_list: list[list[float]]) -> bool:
        """남은 포즈들의 조인트 std가 다양성 임계 미만이면 True."""
        if not joint_angles_list or not all(len(j) >= 5 for j in joint_angles_list):
            return False
        joints = np.array([j[:5] for j in joint_angles_list])
        std_deg = np.degrees(joints.std(axis=0))
        for s, thr in zip(std_deg, T.JOINT_DIVERSITY_THRESHOLD_DEG):
            if s < thr:
                return True
        return False

    @staticmethod
    def _compute_method_compare_lists(
        R_gb_list: list[np.ndarray],
        t_gb_list: list[np.ndarray],
        R_tc_list: list[np.ndarray],
        t_tc_list: list[np.ndarray],
    ) -> list[dict]:
        """cv2 TSAI/PARK/DANIILIDIS 간 결과 차이 — self-consistency 진단용."""
        compare: list[dict] = []
        ref_R: np.ndarray | None = None
        ref_t: np.ndarray | None = None
        for m in _COMPARE_METHODS:
            try:
                Rm, tm = cv2.calibrateHandEye(
                    R_gb_list, t_gb_list, R_tc_list, t_tc_list, method=m
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

    @staticmethod
    def _residuals_pairwise_lists(
        R_cam2gripper: np.ndarray,
        t_cam2gripper: np.ndarray,
        *,
        pose_ids: list[int],
        R_gb_list: list[np.ndarray],
        t_gb_list: list[np.ndarray],
        R_tc_list: list[np.ndarray],
        t_tc_list: list[np.ndarray],
    ) -> tuple[list[dict], float, float]:
        """BA fallback 시 — 첫 포즈를 anchor로 한 편차."""
        T_cam2gripper = make_T(R_cam2gripper, t_cam2gripper.reshape(3))
        T_target2base_list: list[np.ndarray] = []
        for R_gb, t_gb, R_tc, t_tc in zip(R_gb_list, t_gb_list, R_tc_list, t_tc_list):
            T_gb = make_T(np.asarray(R_gb), np.asarray(t_gb).reshape(3))
            T_tc = make_T(np.asarray(R_tc), np.asarray(t_tc).reshape(3))
            T_target2base_list.append(T_gb @ T_cam2gripper @ T_tc)

        ref_R = T_target2base_list[0][:3, :3]
        mean_pos = np.mean([T[:3, 3] for T in T_target2base_list], axis=0)

        per_pose: list[dict] = []
        rot_devs: list[float] = []
        pos_devs: list[float] = []
        for pid, T_pose in zip(pose_ids, T_target2base_list):
            drot = _rotation_diff_deg(ref_R, T_pose[:3, :3])
            dt_mm = float(np.linalg.norm(T_pose[:3, 3] - mean_pos)) * 1000.0
            per_pose.append({"id": pid, "drot_deg": drot, "dt_mm": dt_mm})
            rot_devs.append(drot)
            pos_devs.append(dt_mm)

        sigma_rot = float(np.std(rot_devs)) if rot_devs else 0.0
        sigma_t = float(np.std(pos_devs)) if pos_devs else 0.0
        return per_pose, sigma_rot, sigma_t

    def list_poses_meta(
        self,
        arm_motor_cfgs: list[MotorConfig] | None = None,
    ) -> list[dict]:
        """포즈 목록 메타. arm_motor_cfgs 주면 *현재 offset*으로 변환된 표시용
        joint_angles_rad도 같이 (frontend가 자세 표시할 때 사용).
        """
        coords = JointCoordinates() if arm_motor_cfgs else None
        out: list[dict] = []
        for p in self.poses:
            entry: dict = {
                "id": p.id,
                "timestamp": p.timestamp,
                "raw_motor_positions": {
                    int(mid): int(raw)
                    for mid, raw in p.raw_motor_positions.items()
                },
            }
            if coords is not None and arm_motor_cfgs is not None:
                angles: list[float] = []
                for cfg in arm_motor_cfgs:
                    raw = p.raw_motor_positions.get(cfg.id)
                    if raw is None:
                        break
                    angles.append(coords.motor_to_urdf(int(raw), cfg))
                else:
                    entry["joint_angles_rad"] = angles
            out.append(entry)
        return out

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
    # raw motor + R/t_target2cam만 저장. 이번 캘 세션 안에서 [계산]을 여러 번
    # 누를 때 *그때마다의 시스템 offset*으로 모든 포즈가 일관 재해석되도록 하는 게
    # 유일 목적. 백엔드 재시작 시 캡처 진행 상태 복원에도 사용.

    def save_poses(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if not self.poses:
            np.savez(
                str(path),
                motor_ids=np.empty((0,), dtype=np.int64),
                raw_positions=np.empty((0, 0), dtype=np.int64),
                R_target2cam=np.empty((0, 3, 3)),
                t_target2cam=np.empty((0, 3, 1)),
                ids=np.empty((0,), dtype=np.int64),
                timestamps=np.empty((0,), dtype=np.float64),
                next_id=np.int64(self._next_id),
            )
            return
        motor_ids = sorted(self.poses[0].raw_motor_positions.keys())
        raw_arr = np.array(
            [[p.raw_motor_positions[mid] for mid in motor_ids] for p in self.poses],
            dtype=np.int64,
        )
        np.savez(
            str(path),
            motor_ids=np.array(motor_ids, dtype=np.int64),
            raw_positions=raw_arr,
            R_target2cam=np.stack([p.R_target2cam for p in self.poses]),
            t_target2cam=np.stack(
                [np.asarray(p.t_target2cam).reshape(3, 1) for p in self.poses]
            ),
            ids=np.array([p.id for p in self.poses], dtype=np.int64),
            timestamps=np.array(
                [p.timestamp for p in self.poses], dtype=np.float64
            ),
            next_id=np.int64(self._next_id),
        )

    def load_poses(self, path: str | Path) -> int:
        """디스크에서 포즈 복원. 반환: 로드된 포즈 수."""
        path = Path(path)
        if not path.exists():
            return 0
        data = np.load(str(path))
        if "motor_ids" not in data.files or "raw_positions" not in data.files:
            # 기대하는 raw 기반 포맷 아님 → 없는 것처럼 처리.
            logger.info("기대 포맷 아닌 handeye_poses.npz — 무시 (%s)", path)
            return 0

        ids = data["ids"]
        if len(ids) == 0:
            self._next_id = int(data["next_id"]) if "next_id" in data.files else 0
            return 0

        motor_ids = data["motor_ids"].astype(int).tolist()
        raw_positions = data["raw_positions"]
        R_tc = data["R_target2cam"]
        t_tc = data["t_target2cam"]
        ts = data["timestamps"]
        self.poses = [
            Pose(
                raw_motor_positions={
                    int(mid): int(raw_positions[i, k])
                    for k, mid in enumerate(motor_ids)
                },
                R_target2cam=R_tc[i],
                t_target2cam=t_tc[i],
                id=int(ids[i]),
                timestamp=float(ts[i]),
            )
            for i in range(len(ids))
        ]
        self._next_id = (
            int(data["next_id"]) if "next_id" in data.files else int(ids.max()) + 1
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
