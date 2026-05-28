import logging
import threading
import time
import cv2
import numpy as np
from pathlib import Path

from core.base_node import BaseNode
from core.joint_coordinates import JointCoordinates
from core.link_coordinates import LinkCoordinates
from core.sag_coordinates import SagCoordinates
from modules.calibration.sag_offsets import SagOffsets
from core.topic_map import Service, Topic
from core.frame_cache import FrameCache
from core.joint_state_cache import JointStateCache
from core.common import GRIPPER_ID
from modules.dynamixel.motor_config import load_motor_config
from modules.camera.stream import frame_to_base64
from modules.calibration.intrinsic import CHECKERBOARD, IntrinsicCalibration
from modules.calibration.hand_eye import HandEyeCalibration, Pose
from modules.calibration import next_pose_planner
from modules.calibration import thresholds as calib_thresholds
from modules.calibration.link_offsets import LinkOffsets
from modules.calibration.pose_estimator import PoseEstimator
from modules.kinematics.solver import PybulletSolver

logger = logging.getLogger(__name__)

SAVE_DIR = Path(__file__).parents[2] / "robot" / "calibration"
HANDEYE_POSES_PATH = SAVE_DIR / "handeye_poses.npz"

PREVIEW_INTERVAL = 0.2  # 5Hz


class CalibrationNode(BaseNode):
    def __init__(self) -> None:
        super().__init__("calibration_node")

        self._frame_cache = FrameCache()
        self.intrinsic = IntrinsicCalibration()
        self.hand_eye = HandEyeCalibration()
        self.pose_estimator = PoseEstimator()
        self.solver = PybulletSolver()

        _, motor_cfgs = load_motor_config()
        self._arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
        self._cache = JointStateCache()
        self._cache.subscribe(self)
        self._frame_cache.subscribe(self)

        path = SAVE_DIR / "intrinsic.npz"
        loaded = self.intrinsic.load(path)

        if loaded:
            logger.info(f"Intrinsic 로드 완료: {path}")
        else:
            logger.warning("Intrinsic 파일 없음")

        self._last_compute: dict | None = None
        self._preview_enabled = False
        self._preview_thread: threading.Thread | None = None

        # 내부 캘리브레이션
        self.create_service(Service.CALIB_CAPTURE, self._srv_capture)
        self.create_service(Service.CALIB_INTRINSIC_START,
                            self._srv_intrinsic_start)
        self.create_service(Service.CALIB_INTRINSIC_SAVE,
                            self._srv_intrinsic_save)

        # Hand-Eye 캘리브레이션
        self.create_service(Service.CALIB_HANDEYE_CAPTURE,
                            self._srv_handeye_capture)
        self.create_service(Service.CALIB_HANDEYE_RESET,
                            self._srv_handeye_reset)
        self.create_service(Service.CALIB_HANDEYE_COMPUTE,
                            self._srv_handeye_compute)
        self.create_service(Service.CALIB_HANDEYE_COMMIT,
                            self._srv_handeye_commit)
        self.create_service(
            Service.CALIB_HANDEYE_LIST_POSES, self._srv_handeye_list_poses
        )
        self.create_service(
            Service.CALIB_HANDEYE_PREVIEW_ENABLE, self._srv_handeye_preview_enable
        )
        self.create_service(
            Service.CALIB_HANDEYE_THRESHOLDS, self._srv_handeye_thresholds
        )

    def start(self) -> None:
        super().start()
        self._preview_thread = threading.Thread(
            target=self._preview_loop,
            daemon=True,
            name="calib-preview",
        )
        self._preview_thread.start()
        # joint_offsets 분산 전파는 git 추적이 담당 (모든 머신 같은 commit).
        # 프론트엔드는 mount 시 /calibration/results로 HTTP fetch.
        loaded = self.hand_eye.load_poses(HANDEYE_POSES_PATH)
        if loaded > 0:
            logger.info(f"이전 Hand-Eye 포즈 {loaded}개 복원됨")

    # ─── 이미지 캡처 ─────────────────────────────────────────

    def _srv_capture(self, req: dict) -> dict:
        mode = req.get("data", {}).get("mode", "intrinsic")

        ret, frame = self._frame_cache.get_frame()
        if not ret or frame is None:
            return {
                "success": False,
                "message": "카메라 프레임을 읽을 수 없습니다",
                "data": {},
            }

        if mode == "intrinsic":
            detected, vis = self.intrinsic.capture(frame)
            b64 = frame_to_base64(vis)
            return {
                "success": True,
                "message": "체커보드 감지됨" if detected else "체커보드 미감지",
                "data": {
                    "detected": detected,
                    "captured_count": len(self.intrinsic.obj_points),
                    "preview": b64,
                },
            }

        return {"success": False, "message": f"알 수 없는 mode: {mode}", "data": {}}

    # ─── 내부 캘리브레이션 ────────────────────────────────────

    def _srv_intrinsic_start(self, req: dict) -> dict:
        self.intrinsic.reset()
        return {"success": True, "message": "내부 캘리브레이션 초기화됨", "data": {}}

    def _srv_intrinsic_save(self, req: dict) -> dict:
        width = self._frame_cache.width
        height = self._frame_cache.height
        if width is None or height is None:
            return {
                "success": False,
                "message": "카메라 status(width/height) 미수신",
                "data": {},
            }
        image_size = (width, height)
        result = self.intrinsic.calibrate(image_size)

        if result is None:
            return {
                "success": False,
                "message": f"캘리브레이션 실패 (캡처 수: {len(self.intrinsic.obj_points)})",
                "data": {},
            }

        path = SAVE_DIR / "intrinsic.npz"
        self.intrinsic.save(path)

        return {
            "success": True,
            "message": f"저장 완료: {path}",
            "data": {
                "rms_error": result.rms_error,
                "camera_matrix": result.camera_matrix.tolist(),
                "dist_coeffs": result.dist_coeffs.tolist(),
                "captured_count": result.captured_count,
            },
        }

    # ─── Hand-Eye 캘리브레이션 ────────────────────────────────

    def _srv_handeye_capture(self, req: dict) -> dict:
        if self.intrinsic.result is None:
            return {
                "success": False,
                "message": "내부 캘리브레이션 결과가 필요합니다",
                "data": {},
            }

        # raw motor — 시점 독립 ground truth. URDF rad / FK는 COMPUTE 시점에 계산.
        raw_positions = self._cache.get_raw_motor_positions(self._arm_cfgs)
        if raw_positions is None:
            return {
                "success": False,
                "message": "관절 상태 수신 전",
                "data": {},
            }

        # 카메라 캡처 + 체커보드 검출
        ret, frame = self._frame_cache.get_frame()
        if not ret or frame is None:
            return {"success": False, "message": "카메라 프레임 읽기 실패", "data": {}}

        detected, _ = self.intrinsic.capture(frame)
        if not detected:
            return {
                "success": False,
                "message": "체커보드 미감지",
                "data": {"detected": False, "pose_count": len(self.hand_eye.poses)},
            }

        pose = self.pose_estimator.estimate(
            obj_points=self.intrinsic.obj_points[-1],
            img_points=self.intrinsic.img_points[-1],
            camera_matrix=self.intrinsic.result.camera_matrix,
            dist_coeffs=self.intrinsic.result.dist_coeffs,
        )
        if pose is None:
            return {"success": False, "message": "포즈 추정 실패", "data": {}}

        self.hand_eye.add_pose(
            Pose(
                raw_motor_positions=raw_positions,
                R_target2cam=pose.R,
                t_target2cam=pose.t,
            )
        )

        self._last_compute = None  # 새 포즈 추가 시 이전 계산 결과 무효화
        try:
            self.hand_eye.save_poses(HANDEYE_POSES_PATH)
        except Exception as e:
            logger.warning("포즈 디스크 저장 실패 (메모리에는 남음): %s", e)

        # 캡처 응답에는 추천 포함 X — 추천은 [계산] 응답에서만. 사용자 흐름:
        # 캡처 → 계산 → 피드백+추천 → 이동 → 캡처 → 계산 → ... → 커밋.
        return {
            "success": True,
            "message": f"포즈 기록됨 ({len(self.hand_eye.poses)}개) — [계산]을 눌러 진척 확인",
            "data": {
                "detected": True,
                "pose_count": len(self.hand_eye.poses),
            },
        }

    def _srv_handeye_reset(self, req: dict) -> dict:
        self.hand_eye.reset()
        self._last_compute = None
        # 디스크 파일도 삭제 — "처음부터 다시" 의도와 일치.
        if HANDEYE_POSES_PATH.exists():
            try:
                HANDEYE_POSES_PATH.unlink()
            except OSError as e:
                logger.warning("포즈 파일 삭제 실패: %s", e)
        return {
            "success": True,
            "message": "Hand-Eye 누적 포즈 초기화됨",
            "data": {"pose_count": 0},
        }

    def _srv_handeye_list_poses(self, req: dict) -> dict:
        return {
            "success": True,
            "message": "ok",
            "data": {
                # *현재 offset*으로 변환된 표시용 joint_angles_rad 포함
                "poses": self.hand_eye.list_poses_meta(self._arm_cfgs),
                "pose_count": len(self.hand_eye.poses),
            },
        }

    def _srv_handeye_compute(self, req: dict) -> dict:
        arm_motor_ids = [cfg.id for cfg in self._arm_cfgs]
        joint_limits = self.solver.joint_limits(len(arm_motor_ids))
        # mode 옵션:
        #   "physical_sag" (기본, 43 DOF) — extended + 자세 의존 sag (k_J2, k_J3).
        #       σ_rot ~0.65°/σ_t ~7.9mm 달성 (lumped mass + 모멘트 암 sag 모델로 검증).
        #       lumped mass 가정이라 URDF의 D405 카메라 mass 누락에도 robust.
        #   "extended" (41 DOF) — link_trans/link_rot 풀고 sag X. σ_rot ~1.3°/σ_t ~9mm.
        #       사용자가 sag 모델 회귀 진단 필요할 때.
        #   "standard" (11 DOF) — joint_offset만. 더 옛 회귀.
        mode = str(req.get("mode", "physical_sag")).lower()
        use_physical_sag = mode == "physical_sag"
        use_extended_ba = mode in ("physical_sag", "extended")
        diag = self.hand_eye.compute_with_diagnostics(
            fk_fn=self.solver.fk_to_matrix,
            arm_motor_cfgs=self._arm_cfgs,
            joint_limits_rad=joint_limits,
            use_extended_ba=use_extended_ba,
            use_physical_sag=use_physical_sag,
        )
        if diag is None:
            return {
                "success": False,
                "message": f"Hand-Eye 실패 (포즈 수: {len(self.hand_eye.poses)})",
                "data": {},
            }
        self._last_compute = diag
        # 사용자 흐름: 캡처 → [계산] → 피드백+후보리스트 → [이동] → [캡처] → ...
        # 후보 리스트를 응답에 묶어 round-trip 줄임. 사용자는 다음 [계산] 전까지
        # 이 리스트로만 [이동]함 — 캡처해도 자동 재계산/갱신 X (의도된 페이스).
        diag["recommendations"] = self._compute_recommendations()
        return {
            "success": True,
            "message": f"compute 완료 (poses={diag['pose_count']})",
            "data": diag,
        }

    def _srv_handeye_commit(self, req: dict) -> dict:
        if self._last_compute is None or self.hand_eye.result is None:
            return {
                "success": False,
                "message": "먼저 COMPUTE를 실행하세요",
                "data": {},
            }

        # 1) hand_eye.npz — 카메라↔그리퍼 외부 보정
        hand_eye_path = SAVE_DIR / "hand_eye.npz"
        self.hand_eye.save(hand_eye_path)

        # 2) joint_offsets.npz — BA가 추정한 delta offset을 cumulative 합산해 디스크 저장 +
        # PC 메모리 (JointCoordinates) 즉시 갱신. 다른 머신 적용은 git pull + 재시작.
        applied: dict[int, float] = {}
        offset_msg = ""
        if self._last_compute.get("joint_offset_estimated"):
            delta_list = self._last_compute.get("joint_offset_delta", [])
            delta_by_id = {
                int(e["motor_id"]): float(e["offset_rad"]) for e in delta_list
            }
            applied = JointCoordinates().commit_offsets(
                delta_by_id, method=self.hand_eye.result.method,
            )
            applied_deg = {
                i: round(float(np.degrees(v)), 3) for i, v in applied.items()
            }
            offset_msg = f" + joint_offsets 갱신 (cumulative, deg={applied_deg})"
            logger.info("joint_offsets 즉시 적용: %s", applied_deg)

        # 3) link_offsets.npz — 확장 BA가 추정한 link origin 보정을 *overwrite*.
        # BA의 link_t는 original URDF 기준 absolute total 값 (delta 아님). 따라서
        # disk를 cumulative 가산이 아니라 그대로 덮어씀.
        # (이력: 과거 cumulative 가산이었음 → BA가 absolute 출력하는데 매 commit마다
        #  누적 손상 발생. 2026-05-28 발견, overwrite로 fix. docs/accuracy_squeeze_plan.md §1.6).
        # PybulletSolver는 URDF를 부팅 시 1회 로드라 메모리 자동 갱신 X
        # → 적용은 다음 부팅 (patched URDF 자동 재생성). 사용자가 백엔드 재시작 필요.
        # diag dict의 키는 "link_trans_delta"/"link_rot_delta"로 남아있지만 실제로는
        # absolute 값. 프론트엔드 호환 위해 키명은 유지 (TODO: 향후 *_absolute로 rename).
        link_msg = ""
        link_applied_meta: list[dict] = []
        restart_required = False
        if self._last_compute.get("link_offset_estimated"):
            trans_list = self._last_compute.get("link_trans_delta", [])
            rot_list = self._last_compute.get("link_rot_delta", [])
            new_link = LinkOffsets(
                trans={
                    int(e["motor_id"]): np.array(
                        [e["x_m"], e["y_m"], e["z_m"]], dtype=np.float64
                    )
                    for e in trans_list
                },
                rot={
                    int(e["motor_id"]): np.array(
                        [e["rx_rad"], e["ry_rad"], e["rz_rad"]], dtype=np.float64
                    )
                    for e in rot_list
                },
            )
            link_applied = LinkCoordinates().commit_offsets(
                new_link, method=self.hand_eye.result.method,
            )
            n_joints = len(link_applied.trans)
            link_msg = (
                f" + link_offsets 갱신 (overwrite, n={n_joints}, 백엔드 재시작 후 FK/IK 적용)"
            )
            link_applied_meta = [
                {
                    "motor_id": int(jid),
                    "trans_m": link_applied.get_trans(jid).tolist(),
                    "rot_rad": link_applied.get_rot(jid).tolist(),
                }
                for jid in sorted(link_applied.trans.keys())
            ]
            restart_required = True
            logger.info(
                "link_offsets 디스크 적용 (overwrite, 재시작 필요): n=%d", n_joints
            )

        # 4) sag_offsets.npz — 물리 sag BA가 추정한 k_J2, k_J3 *overwrite*.
        # link_offsets와 같은 이유로 absolute total 값을 그대로 덮어씀 (cumulative 금지).
        # PybulletSolver의 sag 캐시는 매 FK/IK 호출마다 메모리에서 읽으므로 PC는
        # 즉시 반영 (solver._reload_sag_cache 호출). 다른 머신은 git pull + 재시작.
        sag_msg = ""
        sag_applied_meta: list[dict] = []
        if self._last_compute.get("sag_offset_estimated"):
            sag_delta_list = self._last_compute.get("sag_offset_delta", [])
            new_sag = SagOffsets(
                k_rad_per_m={
                    int(e["motor_id"]): float(e["k_rad_per_m"])
                    for e in sag_delta_list
                },
            )
            sag_applied = SagCoordinates().commit_offsets(
                new_sag, method=self.hand_eye.result.method,
            )
            # PC 메모리의 PybulletSolver 캐시도 즉시 갱신 (재시작 X)
            self.solver._reload_sag_cache()
            sag_applied_meta = [
                {
                    "motor_id": int(jid),
                    "k_rad_per_m": float(sag_applied.get_k(jid)),
                }
                for jid in sorted(sag_applied.k_rad_per_m.keys())
            ]
            n_sag = len(sag_applied.k_rad_per_m)
            sag_msg = f" + sag_offsets 갱신 (overwrite, n={n_sag}, 즉시 적용)"
            logger.info(
                "sag_offsets 즉시 적용: %s",
                {m["motor_id"]: round(m["k_rad_per_m"], 5)
                 for m in sag_applied_meta},
            )

        return {
            "success": True,
            "message": f"저장 완료: {hand_eye_path}{offset_msg}{link_msg}{sag_msg}",
            "data": {
                "path": str(hand_eye_path),
                "method": self.hand_eye.result.method,
                "joint_offsets_applied": self._last_compute.get(
                    "joint_offset_estimated", False
                ),
                "joint_offsets": [
                    {"motor_id": int(mid), "offset_rad": float(off)}
                    for mid, off in sorted(applied.items())
                ],
                "link_offsets_applied": self._last_compute.get(
                    "link_offset_estimated", False
                ),
                "link_offsets": link_applied_meta,
                "sag_offsets_applied": self._last_compute.get(
                    "sag_offset_estimated", False
                ),
                "sag_offsets": sag_applied_meta,
                "restart_required": restart_required,
            },
        }

    def _srv_handeye_thresholds(self, req: dict) -> dict:
        """프론트엔드가 mount 시 1회 fetch. 단일 출처 보장."""
        return {
            "success": True,
            "message": "ok",
            "data": calib_thresholds.as_dict(),
        }

    # ─── 다음 자세 후보 리스트 산출 ────────────────────────────
    def _compute_recommendations(self) -> list[dict]:
        """next_pose_planner.recommend_many()를 호출해 dict 리스트로 직렬화.

        planner는 직전 _srv_handeye_compute 결과(self._last_compute)의 BA 잔차를
        주 신호로 사용. last_compute 없으면 (이 함수는 compute 직후에만 호출되니
        사실상 항상 있음) 분포 기반만 채움.

        모터 상태 수신 전이면 빈 리스트 반환.
        """
        current = self._cache.get_joint_angles_rad(self._arm_cfgs)
        if current is None:
            return []
        arm_motor_ids = [cfg.id for cfg in self._arm_cfgs]
        joint_limits = self.solver.joint_limits(len(arm_motor_ids))
        ja_at_compute = (
            self._last_compute.get("joint_angles_per_pose")
            if self._last_compute
            else None
        )
        recs = next_pose_planner.recommend_many(
            last_compute=self._last_compute,
            joint_angles_per_pose_at_compute=ja_at_compute,
            current_joint_angles_rad=list(current),
            arm_motor_ids=arm_motor_ids,
            joint_limits_rad=joint_limits,
        )
        return [next_pose_planner.to_dict(r) for r in recs]

    def _srv_handeye_preview_enable(self, req: dict) -> dict:
        enabled = bool(req.get("data", {}).get("enabled", False))
        self._preview_enabled = enabled
        return {
            "success": True,
            "message": f"preview {'enabled' if enabled else 'disabled'}",
            "data": {"enabled": enabled},
        }

    def _preview_loop(self) -> None:
        # SB는 조명/블러에 강함. preview는 속도 우선이라 EXHAUSTIVE/ACCURACY 미사용.
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        while self._running:
            if not self._preview_enabled:
                time.sleep(PREVIEW_INTERVAL)
                continue

            try:
                ret, frame = self._frame_cache.get_frame()
                if not ret or frame is None:
                    self.publish(
                        Topic.CALIB_HANDEYE_PREVIEW,
                        {
                            "timestamp": time.time(),
                            "detected": False,
                            "reason": "no_frame",
                        },
                    )
                    time.sleep(PREVIEW_INTERVAL)
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                h, w = gray.shape[:2]
                found, corners = cv2.findChessboardCornersSB(
                    gray, CHECKERBOARD, flags=flags
                )

                payload: dict = {
                    "timestamp": time.time(),
                    "detected": bool(found),
                    "image_size": [int(w), int(h)],
                }

                if found and corners is not None:
                    pts = corners.reshape(-1, 2)
                    payload["corners"] = pts.tolist()
                    xs, ys = pts[:, 0], pts[:, 1]
                    bbox_w = float(xs.max() - xs.min())
                    bbox_h = float(ys.max() - ys.min())
                    payload["bbox"] = [
                        float(xs.min()),
                        float(ys.min()),
                        bbox_w,
                        bbox_h,
                    ]
                    payload["coverage_ratio"] = (
                        bbox_w * bbox_h) / float(w * h)

                    # tilt: 보드 평면과 카메라 이미지 평면 사이 각도.
                    # R_target2cam의 board Z축이 카메라 Z축과 얼마나 평행한가로 측정.
                    if self.intrinsic.result is not None:
                        try:
                            ok, rvec, _tvec = cv2.solvePnP(
                                self.intrinsic._objp_template,
                                corners,
                                self.intrinsic.result.camera_matrix,
                                self.intrinsic.result.dist_coeffs,
                                flags=cv2.SOLVEPNP_ITERATIVE,
                            )
                            R, _ = cv2.Rodrigues(rvec)
                            # R[2,2] = board Z축의 카메라 Z성분.
                            # |R[2,2]|=1 → 보드 평면이 이미지 평면과 평행 → tilt 0°
                            cos_v = float(np.clip(abs(R[2, 2]), 0.0, 1.0))
                            payload["tilt_deg"] = float(
                                np.degrees(np.arccos(cos_v)))
                        except cv2.error:
                            pass

                self.publish(Topic.CALIB_HANDEYE_PREVIEW, payload)
            except Exception as e:
                logger.debug("preview loop 오류: %s", e)

            time.sleep(PREVIEW_INTERVAL)
