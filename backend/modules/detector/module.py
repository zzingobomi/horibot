"""DetectorModule — `Detect Object` (Day-1 primitive). PC 배치, **robot-agnostic**.

host 당 1 인스턴스 (backend_v2.md §2.7) — 무거운 모델(GDINO)을 1회 로드하고,
매 DETECT 요청의 `req.robot_id` 로 그 로봇의 camera/캘/TCP 를 조회해 dispatch. __init__ 에
robot_id 없음 (framework 계약: host-level = robot_id 미보유).

flow (DETECT): calibration bundle(intrinsic+hand_eye) + camera color/depth snapshot +
motion TCP → adapter 검출(Top-K bbox) → 후보별 depth median Z → base 투영 + size →
Top-K Detection (§17.5). prior 적용/최종 선택은 소비자(task SelectTarget).

모델은 DetectorBackend adapter 뒤 (§0). 투영 수학은 projection.py (결정적).
다른 모듈 호출은 `await self.runtime.call(...)` 로 통일 (framework_async_call_contract.md).
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    SnapshotBundleRequest,
)
from modules.camera.contract import (
    Camera,
    CameraDecodedFrame,
    CameraDepthDecodedFrame,
    DecodedSnapshotRequest,
    DepthDecodedSnapshotRequest,
)
from modules.motion.contract import Motion, TcpSnapshotRequest, TcpState

from . import geometry, projection
from .contract import (
    DetectOrientedResponse,
    DetectRequest,
    DetectResponse,
    Detection,
    DetectionsUpdate,
    Detector,
    OrientedDetection,
    OrientedDetectionsUpdate,
)
from .drivers.protocol import Bbox, DetectorBackend

logger = logging.getLogger(__name__)

# detect_oriented 디버그 덤프 — SAM mask 원본 픽셀 알파 오버레이 PNG (draft 단계).
# 패널 오버레이는 contour 폴리곤 근사라 세그멘테이션 정밀 확인은 이 파일로.
_DEBUG_DIR = Path("debug")


@dataclass(slots=True)
class _Proj:
    """한 프레임의 투영 파라미터 (모든 후보 공통) — intrinsic + TCP pose + hand_eye.
    detect_oriented 가 base OBB 코너를 픽셀로 reproject (오버레이) 할 때 사용."""

    fx: float
    fy: float
    cx: float
    cy: float
    r_be: np.ndarray
    t_be: np.ndarray
    r_ce: np.ndarray
    t_ce: np.ndarray


@dataclass(slots=True)
class _Cand:
    """후보 1개의 base frame 산출 — detect(AABB) / detect_oriented(OBB) 공통 중간형.

    base_points: mask 픽셀의 base 점군 (obb 소스). detect 는 무시, detect_oriented 만
    geometry 로 OBB 산출. depth 무효로 점군 못 만들면 None. mask: SAM 픽셀 mask
    (윤곽 오버레이 소스, detect 는 무시).
    """

    bbox: Bbox
    score: float
    position: tuple[float, float, float]
    base_z: float
    height: float
    base_points: np.ndarray | None
    mask: np.ndarray


@dataclass(slots=True)
class _DetectResult:
    """_detect_candidates 출력 — 두 서비스가 각자 응답형으로 포맷. color dims 는 빈
    결과에도 오버레이 clear publish 에 필요. proj = 프레임 확보 시만 (없으면 None)."""

    cands: list[_Cand]
    color_w: int
    color_h: int
    message: str
    proj: _Proj | None = None
    img: np.ndarray | None = None  # color BGR — detect_oriented 디버그 덤프용


@publishes(
    (Detector.Stream.DETECTIONS, DetectionsUpdate),
    (Detector.Stream.DETECTIONS_ORIENTED, OrientedDetectionsUpdate),
)
class DetectorModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        backend: DetectorBackend,
    ) -> None:
        self.runtime = runtime
        self._backend = backend
        self._preload_task: asyncio.Task[None] | None = None
        self._detections_seq = 0

    async def start(self) -> None:
        logger.info("DetectorModule start (host-level)")
        # 백그라운드 preload — boot 를 막지 않는다 (모델 다운로드/로드 수십 초~수 분).
        # blocking 로드 → to_thread. 실패해도 첫 detect 가 lazy 재시도.
        self._preload_task = asyncio.create_task(self._preload())

    async def _preload(self) -> None:
        try:
            await asyncio.to_thread(self._backend.preload)
        except Exception:
            logger.exception("detector backend preload 실패 — 첫 detect 시 재시도")

    async def stop(self) -> None:
        logger.info("DetectorModule stop (host-level)")
        if self._preload_task is not None:
            self._preload_task.cancel()
            self._preload_task = None

    async def _detect_candidates(
        self, robot_id: str, prompt: str, top_k: int
    ) -> _DetectResult:
        """공통 파이프라인 — 캘/frame/backend 검출 → 후보별 base 투영. SSOT.

        detect(AABB) / detect_oriented(OBB) 가 공유. 실패는 빈 cands + 사용자 message
        (found=False 로 매핑). blocking 추론(GPU)만 to_thread, 나머지 순수 계산.
        """
        # 1. intrinsic + hand_eye (같은 캘 출처 → 일관). 없으면 검출 불가.
        # calibration 도 robot-agnostic — 대상 robot 은 req 필드.
        bundle = await self.runtime.call(
            Calibration.Service.SNAPSHOT_BUNDLE,
            SnapshotBundleRequest(robot_id=robot_id),
            CalibrationBundle,
        )
        if bundle.intrinsic is None or bundle.hand_eye is None:
            return _DetectResult([], 0, 0, "intrinsic/hand_eye 캘 없음 — 캘 먼저")

        # 2. color snapshot → 모델 입력
        color = await self.runtime.call(
            Camera.Service.DECODED_SNAPSHOT,
            DecodedSnapshotRequest(),
            CameraDecodedFrame,
            robot_id=robot_id,
        )
        img = np.frombuffer(color.ndarray_bytes, dtype=np.uint8).reshape(
            color.height, color.width, 3
        )

        # 3. depth snapshot → Z (D405 aligned depth)
        depth_f = await self.runtime.call(
            Camera.Service.DEPTH_DECODED_SNAPSHOT,
            DepthDecodedSnapshotRequest(),
            CameraDepthDecodedFrame,
            robot_id=robot_id,
        )
        depth = np.frombuffer(depth_f.depth_bytes, dtype=np.uint16).reshape(
            depth_f.height, depth_f.width
        )

        # 4. adapter 검출 (Top-K RawDetection: bbox+mask+score). blocking(GPU) → to_thread.
        raws = await asyncio.to_thread(self._backend.detect, img, prompt, top_k)
        if not raws:
            return _DetectResult([], color.width, color.height, f"'{prompt}' 감지 실패")

        # 5. 후보 공통 자원 1회 — TCP pose (ee → base) + intrinsic + hand_eye (cam → ee).
        tcp = await self.runtime.call(
            Motion.Service.TCP_SNAPSHOT,
            TcpSnapshotRequest(),
            TcpState,
            robot_id=robot_id,
        )
        r_be = Rotation.from_quat(tcp.quaternion).as_matrix()
        t_be = np.array(tcp.position, dtype=float)
        cm = bundle.intrinsic.result_data.camera_matrix
        fx, fy, cx, cy = cm[0][0], cm[1][1], cm[0][2], cm[1][2]
        r_ce = np.array(bundle.hand_eye.result_data.R_cam2gripper, dtype=float)
        t_ce = np.array(
            bundle.hand_eye.result_data.t_cam2gripper, dtype=float
        ).reshape(3)

        # 6. 후보별 depth median Z → base 투영 + floor_z/height (§17.5 기하 prior) +
        #    mask 점군(OBB 소스). depth 없는 후보는 base 좌표 산출 불가 → 누락.
        cands: list[_Cand] = []
        for raw in raws:
            # 윗면 픽셀들의 실제 3D centroid = 큐브 윗면 중심(=바닥 중심 x/y). bbox 중심
            # 픽셀 + 윗면 depth 짝짓기의 systematic 편향(비스듬한 시점 → 파지점이 카메라
            # 쪽 모서리로 밀림) fix.
            base = projection.object_top_center_base(
                depth, raw.bbox, depth_f.depth_scale, fx, fy, cx, cy,
                r_be, t_be, r_ce, t_ce,
            )
            if base is None:
                continue
            floor_z, height = projection.floor_z_and_height(
                depth, raw.bbox, depth_f.depth_scale, fx, fy, cx, cy,
                r_be, t_be, r_ce, t_ce, obj_top_base_z=float(base[2]),
            )
            base_pts = projection.base_points_from_mask(
                depth, raw.mask, depth_f.depth_scale, fx, fy, cx, cy,
                r_be, t_be, r_ce, t_ce,
            )
            cands.append(
                _Cand(
                    bbox=raw.bbox,
                    score=float(raw.score),
                    position=(float(base[0]), float(base[1]), float(base[2])),
                    base_z=float(floor_z),
                    height=float(height),
                    base_points=base_pts,
                    mask=raw.mask,
                )
            )
        msg = "" if cands else "후보 bbox depth 전부 무효"
        proj = _Proj(fx, fy, cx, cy, r_be, t_be, r_ce, t_ce)
        return _DetectResult(
            cands, color.width, color.height, msg, proj=proj, img=img
        )

    @service(Detector.Service.DETECT)
    async def detect(self, req: DetectRequest) -> DetectResponse:
        prompt = req.prompt.strip()
        if not prompt:
            return DetectResponse(found=False, message="prompt 필요")
        result = await self._detect_candidates(req.robot_id, prompt, req.top_k)
        detections = [
            Detection(
                prompt=prompt,
                position=c.position,
                score=c.score,
                base_z=c.base_z,
                height=c.height,
                bbox_2d=c.bbox,
            )
            for c in result.cands
        ]
        # frontend 카메라 오버레이 — frame 확보 시 결과 스냅샷 publish (빈 결과 포함).
        if result.color_w > 0:
            self._publish_detections(
                req.robot_id, prompt, result.color_w, result.color_h, detections
            )
        if not detections:
            return DetectResponse(found=False, message=result.message)
        # backend 가 score desc 로 줌 → candidates 도 desc 유지 (task SelectTarget 소비).
        return DetectResponse(found=True, candidates=detections)

    @service(Detector.Service.DETECT_ORIENTED)
    async def detect_oriented(self, req: DetectRequest) -> DetectOrientedResponse:
        """[DRAFT] DETECT + mask→base 점군→minAreaRect OBB (grasp yaw + footprint).

        오버레이용 image-space 도 산출: obb_2d = base OBB 코너를 픽셀로 reproject(회전
        사각형), mask_contour = SAM mask 윤곽(실루엣). DETECTIONS_ORIENTED 스트림 publish.
        shape 굳으면 Detection 승격 + DETECT 흡수. depth 점군 부족으로 OBB 못 만든 후보는
        누락 (draft — 실물에서 임계 tuning).
        """
        prompt = req.prompt.strip()
        if not prompt:
            return DetectOrientedResponse(found=False, message="prompt 필요")
        result = await self._detect_candidates(req.robot_id, prompt, req.top_k)
        oriented: list[OrientedDetection] = []
        debug_rows: list[tuple[np.ndarray, list[tuple[float, float]] | None]] = []
        for c in result.cands:
            # 윗면 band 만 — mask 전체는 옆면/배경 bleed 로 footprint 부풀림 + yaw 비틂
            # (geometry.top_face_points 주석, 2026-07-09 실물 확인).
            obb = geometry.obb_from_base_points(
                geometry.top_face_points(c.base_points)
            )
            if obb is None:
                continue
            obb_2d: list[tuple[float, float]] | None = None
            if result.proj is not None:
                p = result.proj
                # base OBB 코너를 물체 윗면 z 평면에 놓고 픽셀로 reproject (오버레이).
                corners = geometry.obb_corners(obb, z=c.position[2])
                px = projection.project_base_to_pixel(
                    corners, p.fx, p.fy, p.cx, p.cy, p.r_be, p.t_be, p.r_ce, p.t_ce
                )
                obb_2d = [(float(u), float(v)) for u, v in px]
            contour = geometry.mask_contour(c.mask)
            mask_contour = (
                [(float(x), float(y)) for x, y in contour]
                if contour is not None
                else None
            )
            debug_rows.append((c.mask, obb_2d))
            oriented.append(
                OrientedDetection(
                    prompt=prompt,
                    position=c.position,
                    score=c.score,
                    base_z=c.base_z,
                    height=c.height,
                    grasp_yaw=obb.yaw_rad,
                    footprint=obb.footprint,
                    bbox_2d=c.bbox,
                    obb_2d=obb_2d,
                    mask_contour=mask_contour,
                )
            )
        # 오버레이 스냅샷 publish — frame 확보 시 (빈 결과 포함, clear).
        if result.color_w > 0:
            self._publish_oriented(
                req.robot_id, prompt, result.color_w, result.color_h, oriented
            )
        # 디버그 PNG 덤프 — 원본 mask 픽셀 알파 오버레이 (실패해도 서비스는 무사).
        if result.img is not None and debug_rows:
            try:
                self._dump_debug_image(result.img, oriented, debug_rows)
            except Exception:
                logger.exception("detect_oriented 디버그 덤프 실패 (서비스 영향 없음)")
        if not oriented:
            return DetectOrientedResponse(
                found=False, message=result.message or "OBB 산출 실패"
            )
        return DetectOrientedResponse(found=True, candidates=oriented)

    def _dump_debug_image(
        self,
        img_bgr: np.ndarray,
        oriented: list[OrientedDetection],
        rows: list[tuple[np.ndarray, list[tuple[float, float]] | None]],
    ) -> None:
        """SAM mask 알파 오버레이(원본 픽셀) + bbox + OBB + yaw 를 그린 PNG 저장.

        패널 오버레이는 contour 폴리곤 근사 — 세그멘테이션이 실제로 어떻게 됐는지
        픽셀 단위 확인은 이 파일 (draft 디버그). 마지막 호출 1장 overwrite.
        """
        canvas = img_bgr.copy()
        for det, (mask, obb_2d) in zip(oriented, rows, strict=True):
            tint = canvas.copy()
            tint[mask] = (239, 70, 217)  # fuchsia (BGR) — 패널 contour 색과 통일
            canvas = cv2.addWeighted(tint, 0.35, canvas, 0.65, 0)
            if det.bbox_2d is not None:
                x1, y1, x2, y2 = (int(round(v)) for v in det.bbox_2d)
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (153, 211, 52), 2)
            if obb_2d is not None:
                pts = np.array(obb_2d, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(canvas, [pts], True, (11, 158, 245), 3)
                cv2.putText(
                    canvas,
                    f"yaw {np.degrees(det.grasp_yaw):.0f}deg "
                    f"{det.footprint[0] * 1000:.0f}x{det.footprint[1] * 1000:.0f}mm",
                    (int(obb_2d[0][0]), max(int(obb_2d[0][1]) - 8, 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (11, 158, 245),
                    2,
                )
        _DEBUG_DIR.mkdir(parents=True, exist_ok=True)
        out = _DEBUG_DIR / "detect_oriented_last.png"
        cv2.imwrite(str(out), canvas)
        logger.info("detect_oriented 디버그 덤프: %s", out.resolve())

    def _publish_detections(
        self,
        robot_id: str,
        prompt: str,
        width: int,
        height: int,
        detections: list[Detection],
    ) -> None:
        self.runtime.publish(
            Detector.Stream.DETECTIONS,
            DetectionsUpdate(
                robot_id=robot_id,
                seq=self._detections_seq,
                timestamp_unix=time.time(),
                prompt=prompt,
                image_width=width,
                image_height=height,
                candidates=detections,
            ),
        )
        self._detections_seq += 1

    def _publish_oriented(
        self,
        robot_id: str,
        prompt: str,
        width: int,
        height: int,
        candidates: list[OrientedDetection],
    ) -> None:
        self.runtime.publish(
            Detector.Stream.DETECTIONS_ORIENTED,
            OrientedDetectionsUpdate(
                robot_id=robot_id,
                seq=self._detections_seq,
                timestamp_unix=time.time(),
                prompt=prompt,
                image_width=width,
                image_height=height,
                candidates=candidates,
            ),
        )
        self._detections_seq += 1
