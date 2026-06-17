"""Scene3DNode — RGBD primitive sensor 자리.

책임 (storage_layer.md §3 + scene3d_decoupling.md):
- depth_frame topic subscribe → latest_frame 자리
- live point cloud stream publish (refcount 0 자리면 idle)
- SCENE3D_SNAPSHOT service — 단발 RGBD 캡처 (caller 가 storage put / fuse 등)
- SCENE3D_SET_STREAM service — continuous toggle (Scene Controls 자리)

scan / mesh / reconstruction 자리 X — Storage Phase 2 (storage_node) +
ReconstructionNode + ScanTask 자리 분리.

분산 자리 (CLAUDE.md §아키텍처):
- Scene3DNode = PC (application). 카메라 Pi = device, raw depth_frame topic 자리.
- CAMERA_SET_DEPTH_STREAM service 호출 자리 LAN ~10ms.
- refcount 가 idle 자리 LAN traffic 0 KB/s 자리 (전 소비자 release 시 카메라 끔).
"""

import logging
import struct
import threading
import time
import uuid

import cv2
import numpy as np
import open3d as o3d
import zstandard as zstd

from core.transport.application_node import ApplicationNode
from core.transport.messages.base import ServiceRequest, ServiceResponse
from core.transport.messages.camera import (
    CameraSetDepthStreamReq,
    CameraSetDepthStreamRes,
)
from core.transport.messages.scene3d import (
    Scene3DIntrinsic,
    Scene3DSetStreamReq,
    Scene3DSetStreamRes,
    Scene3DSnapshotReq,
    Scene3DSnapshotRes,
    Scene3DState,
)
from core.cache.joint_state_cache import JointStateCache
from core.transport.topic_map import Service, Topic, topic_for
from modules.camera.depth_frame import DepthFrame, decode as decode_depth_frame
from modules.motor.motor_config import MotorConfig, load_motor_layout
from modules.scene3d import consensus

logger = logging.getLogger(__name__)

DEFAULT_VOXEL_SIZE = 0.005  # 5mm — 라이브 스트림용
TARGET_FPS = 8.0
IDLE_SLEEP = 0.1
DEPTH_TRUNC = 1.0  # m


class _RobotState:
    def __init__(self, arm_cfgs: list[MotorConfig]) -> None:
        self.arm_cfgs = arm_cfgs
        self.cfg_lock = threading.Lock()
        self.frame_lock = threading.Lock()
        self.enabled = False
        self.voxel_size = DEFAULT_VOXEL_SIZE
        self.latest_frame: DepthFrame | None = None
        # depth_consumer token set — snapshot uuid + persistent "stream" 자리
        # 같은 refcount path. consumers 자리 비면 CAMERA depth stream 끔.
        self.consumers: set[str] = set()


class Scene3DNode(ApplicationNode):
    """RGBD primitive sensor 자리 — rgbd capability robot 자리만 dispatch."""

    def __init__(self) -> None:
        super().__init__("scene3d_node")
        # rgbd capability robot 자리만 — robots.yaml 의 capabilities 확인.
        # 없는 자리 robot 자리 자동 skip (snapshot service 등록 X).
        self._states: dict[str, _RobotState] = {}
        for rid in self.enabled_robot_ids:
            cfg = self._registry.get(rid)
            if "rgbd" not in cfg.capabilities:
                continue
            self._states[rid] = _RobotState(load_motor_layout(rid).arm)
        self._joint_cache = JointStateCache()
        self._stream_thread: threading.Thread | None = None

    def start(self) -> None:
        for rid in self._states.keys():
            self.create_service(
                topic_for(Service.SCENE3D_SNAPSHOT, rid),
                Scene3DSnapshotReq,
                Scene3DSnapshotRes,
                lambda req, _rid=rid: self._srv_snapshot(req, _rid),
            )
            self.create_service(
                topic_for(Service.SCENE3D_SET_STREAM, rid),
                Scene3DSetStreamReq,
                Scene3DSetStreamRes,
                lambda req, _rid=rid: self._srv_set_stream(req, _rid),
            )
            self.create_raw_subscriber(
                topic_for(Topic.CAMERA_DEPTH_FRAME, rid),
                lambda payload, _rid=rid: self._on_depth_frame(_rid, payload),
            )

        super().start()
        self._joint_cache.subscribe(self)

        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="scene3d-stream",
            daemon=True,
        )
        self._stream_thread.start()
        for rid in self._states.keys():
            self._publish_state(rid)

        logger.info(
            "Scene3DNode 시작 (rgbd robots=%s)", list(self._states.keys())
        )

    # ─── depth_frame subscriber ──────────────────────────────

    def _on_depth_frame(self, robot_id: str, payload: bytes) -> None:
        try:
            frame = decode_depth_frame(payload)
        except Exception as e:
            logger.warning(f"depth_frame[{robot_id}] 디코드 실패: {e}")
            return
        st = self._states[robot_id]
        with st.frame_lock:
            st.latest_frame = frame

    # ─── depth consumer refcount ─────────────────────────────
    # snapshot / set_stream 자리 같은 refcount path 공유. consumers set 자리
    # 비면 CAMERA 끔. 분산 자리 idle 0 KB/s.

    def _acquire_depth_consumer(self, robot_id: str, token: str) -> bool | None:
        st = self._states[robot_id]
        with st.cfg_lock:
            if token in st.consumers:
                return False
            was_empty = len(st.consumers) == 0
            st.consumers.add(token)
        if was_empty:
            res = self.call_service(
                topic_for(Service.CAMERA_SET_DEPTH_STREAM, robot_id),
                CameraSetDepthStreamReq(enabled=True),
                CameraSetDepthStreamRes,
            )
            if not res.success:
                with st.cfg_lock:
                    st.consumers.discard(token)
                logger.warning(
                    "[%s] CAMERA depth stream enable 실패: %s",
                    robot_id, res.message,
                )
                return None
            with st.cfg_lock:
                st.enabled = True
            self._publish_state(robot_id)
        return True

    def _release_depth_consumer(self, robot_id: str, token: str) -> None:
        st = self._states[robot_id]
        with st.cfg_lock:
            if token not in st.consumers:
                return
            st.consumers.discard(token)
            now_empty = len(st.consumers) == 0
        if now_empty:
            res = self.call_service(
                topic_for(Service.CAMERA_SET_DEPTH_STREAM, robot_id),
                CameraSetDepthStreamReq(enabled=False),
                CameraSetDepthStreamRes,
            )
            if not res.success:
                logger.warning(
                    "[%s] CAMERA depth stream disable 실패: %s",
                    robot_id, res.message,
                )
            with st.cfg_lock:
                st.enabled = False
            with st.frame_lock:
                st.latest_frame = None
            self._publish_state(robot_id)

    # ─── set_stream — Scene Controls 의 토글 ─────────────────

    def _srv_set_stream(
        self, req: ServiceRequest[Scene3DSetStreamReq], robot_id: str
    ) -> ServiceResponse[Scene3DSetStreamRes]:
        ok = self._set_stream(robot_id, req.data.enabled)
        if ok is None:
            return ServiceResponse(
                success=False, message="카메라 depth 스트림 전환 실패", data=None
            )
        st = self._states[robot_id]
        with st.cfg_lock:
            current = st.enabled
        return ServiceResponse(
            success=True, message="ok",
            data=Scene3DSetStreamRes(enabled=current),
        )

    def _set_stream(self, robot_id: str, enabled: bool) -> bool | None:
        """`stream` 토큰 acquire/release. idempotent."""
        if enabled:
            ok = self._acquire_depth_consumer(robot_id, "stream")
            return ok if ok is not None else None
        self._release_depth_consumer(robot_id, "stream")
        return True

    # ─── snapshot — ScanTask 의 CaptureScan / 캘 verification ─

    def _srv_snapshot(
        self, req: ServiceRequest[Scene3DSnapshotReq], robot_id: str
    ) -> ServiceResponse[Scene3DSnapshotRes]:
        """단발 RGBD 캡처 — N frame consensus median.

        consumers refcount 에 1회성 token → fresh frame N개 모음 → consensus →
        JPEG/zstd 압축 → token 제거. set_stream 동시 ON 자리 stream 끄지 않음.
        """
        st = self._states[robot_id]
        num_frames = max(1, int(req.data.num_frames))
        timeout_s = float(req.data.timeout_s)

        token = f"snap-{uuid.uuid4().hex[:8]}"
        ok = self._acquire_depth_consumer(robot_id, token)
        if ok is None:
            return ServiceResponse(
                success=False, message="카메라 depth 스트림 시작 실패",
            )
        try:
            def _get_frame() -> DepthFrame | None:
                with st.frame_lock:
                    return st.latest_frame
            try:
                frames = consensus.gather_frames(
                    _get_frame, n=num_frames, timeout=timeout_s,
                )
            except TimeoutError as e:
                return ServiceResponse(success=False, message=str(e))

            depth_z16 = (
                consensus.consensus_depth(frames)
                if num_frames > 1
                else frames[0].depth_z16
            )
            color_bgr = (
                consensus.consensus_color(frames)
                if num_frames > 1
                else frames[0].color_bgr
            )

            raw_dict = self._joint_cache.get_raw_motor_positions(
                st.arm_cfgs, robot_id=robot_id
            )
            if raw_dict is None:
                return ServiceResponse(
                    success=False,
                    message="motor state 없음 — motor 노드 확인",
                )
            arm_motor_ids = [cfg.id for cfg in st.arm_cfgs]
            motor_positions = [raw_dict[mid] for mid in arm_motor_ids]

            ok_jpeg, jpeg_buf = cv2.imencode(".jpg", color_bgr)
            if not ok_jpeg:
                return ServiceResponse(
                    success=False, message="color JPEG 인코딩 실패",
                )
            depth_zstd = zstd.ZstdCompressor().compress(depth_z16.tobytes())

            f0 = frames[0]
            return ServiceResponse(
                success=True,
                message="ok",
                data=Scene3DSnapshotRes(
                    color_bgr_jpeg=bytes(jpeg_buf),
                    depth_z16_zstd=depth_zstd,
                    intrinsic=Scene3DIntrinsic(
                        width=f0.width,
                        height=f0.height,
                        fx=f0.fx,
                        fy=f0.fy,
                        cx=f0.cx,
                        cy=f0.cy,
                        depth_scale=f0.depth_scale,
                    ),
                    motor_positions=motor_positions,
                    arm_motor_ids=arm_motor_ids,
                    timestamp=f0.timestamp,
                    num_frames=len(frames),
                ),
            )
        except Exception as e:
            logger.exception("[%s] snapshot 실패", robot_id)
            return ServiceResponse(success=False, message=str(e))
        finally:
            self._release_depth_consumer(robot_id, token)

    def _publish_state(self, robot_id: str) -> None:
        st = self._states[robot_id]
        with st.cfg_lock:
            self.publish(
                topic_for(Topic.SCENE3D_STATE, robot_id),
                Scene3DState(
                    timestamp=time.time(),
                    enabled=st.enabled,
                    voxel_size=st.voxel_size,
                ),
            )

    # ─── live stream loop ────────────────────────────────────

    def _stream_loop(self) -> None:
        period = 1.0 / TARGET_FPS
        last_processed_ts: dict[str, float] = {
            rid: 0.0 for rid in self._states.keys()
        }

        while self._running:
            any_processed = False
            for rid in self._states.keys():
                st = self._states[rid]
                with st.cfg_lock:
                    enabled = st.enabled
                    voxel = st.voxel_size

                if not enabled:
                    continue

                with st.frame_lock:
                    frame = st.latest_frame

                if frame is None or frame.timestamp <= last_processed_ts[rid]:
                    continue

                t0 = time.time()
                try:
                    payload = self._build_payload(frame, voxel)
                except Exception as e:
                    logger.warning(f"[{rid}] 포인트클라우드 생성 실패: {e}")
                    continue

                try:
                    self.session.put(
                        topic_for(Topic.SCENE3D_STREAM, rid), payload
                    )
                    last_processed_ts[rid] = frame.timestamp
                    any_processed = True
                except Exception as e:
                    logger.warning(f"[{rid}] 포인트클라우드 발행 실패: {e}")

                elapsed = time.time() - t0
                if elapsed > period:
                    break

            if not any_processed:
                time.sleep(IDLE_SLEEP)
            else:
                time.sleep(max(0.0, period / max(1, len(self._states))))

    def _build_payload(self, frame: DepthFrame, voxel_size: float) -> bytes:
        pcd = _build_pcd(frame)
        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size)

        xyz = np.asarray(pcd.points, dtype=np.float32)
        rgb_f = np.asarray(pcd.colors, dtype=np.float32)
        rgb_u8 = (np.clip(rgb_f, 0.0, 1.0) * 255.0).astype(np.uint8)

        n = xyz.shape[0]
        return struct.pack("<I", n) + xyz.tobytes() + rgb_u8.tobytes()


def _build_pcd(frame: DepthFrame) -> "o3d.geometry.PointCloud":
    rgb = np.ascontiguousarray(frame.color_bgr[:, :, ::-1])
    color_o3d = o3d.geometry.Image(rgb)
    depth_o3d = o3d.geometry.Image(frame.depth_z16)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1.0 / frame.depth_scale,
        depth_trunc=DEPTH_TRUNC,
        convert_rgb_to_intensity=False,
    )
    pinhole = o3d.camera.PinholeCameraIntrinsic(
        width=frame.width,
        height=frame.height,
        fx=frame.fx,
        fy=frame.fy,
        cx=frame.cx,
        cy=frame.cy,
    )
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole)
