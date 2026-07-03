"""DetectorModule — `Detect Object` (Day-1 primitive). PC 배치, **robot-agnostic**.

host 당 1 인스턴스 (backend_v2.md §2.7) — 무거운 모델(GDINO)을 1회 로드하고,
매 DETECT 요청의 `req.robot_id` 로 그 로봇의 camera/캘/TCP 를 조회해 dispatch. __init__ 에
robot_id 없음 (framework 계약: host-level = robot_id 미보유).

flow (DETECT): calibration bundle(intrinsic+hand_eye) + camera color/depth snapshot +
motion TCP → adapter 검출(bbox) → depth median Z → base 투영 → Detection.

모델은 DetectorBackend adapter 뒤 (§0). 투영 수학은 projection.py (결정적).
다른 모듈 호출은 `await self.runtime.call(...)` 로 통일 (framework_async_call_contract.md).
"""

from __future__ import annotations

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
from .backend import DetectorBackend
from .contract import DetectRequest, DetectResponse, Detection, Detector

logger = logging.getLogger(__name__)


class DetectorModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        backend: DetectorBackend,
    ) -> None:
        self.runtime = runtime
        self._backend = backend

    async def start(self) -> None:
        logger.info("DetectorModule start (host-level)")

    async def stop(self) -> None:
        logger.info("DetectorModule stop (host-level)")

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

        # 4. adapter 검출 (bbox)
        det = self._backend.detect(img, prompt)
        if det is None:
            return DetectResponse(found=False, message=f"'{prompt}' 감지 실패")
        bbox, score = det

        # 5. bbox depth median → Z_cam
        z_cam = projection.z_cam_from_depth_bbox(depth, bbox, depth_f.depth_scale)
        if z_cam is None or z_cam <= 0:
            return DetectResponse(
                found=False, message="bbox 영역 valid depth 없음/무효"
            )

        # 6. TCP pose (ee → base)
        tcp = await self.runtime.call(
            Motion.Service.TCP_SNAPSHOT,
            TcpSnapshotRequest(),
            TcpState,
            robot_id=robot_id,
        )
        r_be = Rotation.from_quat(tcp.quaternion).as_matrix()
        t_be = np.array(tcp.position, dtype=float)

        # 7. intrinsic (camera_matrix) + hand_eye (cam → ee)
        cm = bundle.intrinsic.result_data.camera_matrix
        fx, fy, cx, cy = cm[0][0], cm[1][1], cm[0][2], cm[1][2]
        r_ce = np.array(bundle.hand_eye.result_data.R_cam2gripper, dtype=float)
        t_ce = np.array(
            bundle.hand_eye.result_data.t_cam2gripper, dtype=float
        ).reshape(3)

        # 8. bbox 중심 → base 투영
        u = (bbox[0] + bbox[2]) / 2.0
        v = (bbox[1] + bbox[3]) / 2.0
        base = projection.unproject_to_base(
            u, v, z_cam, fx, fy, cx, cy, r_be, t_be, r_ce, t_ce
        )
        return DetectResponse(
            found=True,
            detection=Detection(
                prompt=prompt,
                position=(float(base[0]), float(base[1]), float(base[2])),
                score=float(score),
            ),
        )
