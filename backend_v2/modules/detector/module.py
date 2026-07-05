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

import numpy as np
from scipy.spatial.transform import Rotation

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

from . import projection
from .contract import DetectRequest, DetectResponse, Detection, Detector
from .drivers.protocol import DetectorBackend

logger = logging.getLogger(__name__)


class DetectorModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        backend: DetectorBackend,
    ) -> None:
        self.runtime = runtime
        self._backend = backend
        self._preload_task: asyncio.Task[None] | None = None

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

    @service(Detector.Service.DETECT)
    async def detect(self, req: DetectRequest) -> DetectResponse:
        prompt = req.prompt.strip()
        if not prompt:
            return DetectResponse(found=False, message="prompt 필요")
        robot_id = req.robot_id  # host당 1 — 매 요청이 대상 로봇 명시

        # 1. intrinsic + hand_eye (같은 캘 출처 → 일관). 없으면 검출 불가.
        # calibration 도 robot-agnostic — 대상 robot 은 req 필드.
        bundle = await self.runtime.call(
            Calibration.Service.SNAPSHOT_BUNDLE,
            SnapshotBundleRequest(robot_id=robot_id),
            CalibrationBundle,
        )
        if bundle.intrinsic is None or bundle.hand_eye is None:
            return DetectResponse(
                found=False, message="intrinsic/hand_eye 캘 없음 — 캘 먼저"
            )

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

        # 4. adapter 검출 (Top-K bbox, score desc). blocking 추론(GPU) → to_thread.
        cands = await asyncio.to_thread(
            self._backend.detect, img, prompt, req.top_k
        )
        if not cands:
            return DetectResponse(found=False, message=f"'{prompt}' 감지 실패")

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

        # 6. 후보별 depth median Z → base 투영 + floor_z/height (§17.5 기하 prior).
        detections: list[Detection] = []
        for bbox, score in cands:
            z_cam = projection.z_cam_from_depth_bbox(
                depth, bbox, depth_f.depth_scale
            )
            if z_cam is None or z_cam <= 0:
                continue  # depth 없는 후보는 base 좌표 산출 불가 → 누락
            u = (bbox[0] + bbox[2]) / 2.0
            v = (bbox[1] + bbox[3]) / 2.0
            base = projection.unproject_to_base(
                u, v, z_cam, fx, fy, cx, cy, r_be, t_be, r_ce, t_ce
            )
            floor_z, height = projection.floor_z_and_height(
                depth, bbox, depth_f.depth_scale, fx, fy, cx, cy,
                r_be, t_be, r_ce, t_ce, obj_top_base_z=float(base[2]),
            )
            detections.append(
                Detection(
                    prompt=prompt,
                    position=(float(base[0]), float(base[1]), float(base[2])),
                    score=float(score),
                    base_z=float(floor_z),
                    height=float(height),
                )
            )

        if not detections:
            return DetectResponse(found=False, message="후보 bbox depth 전부 무효")
        # backend 가 score desc 로 줌 → candidates 도 desc 유지 (task SelectTarget 소비).
        return DetectResponse(found=True, candidates=detections)
