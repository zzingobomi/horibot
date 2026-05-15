"""포인트클라우드 라이브 스트림 + 스냅샷/세션 캡처 노드.

라이브 스트림:
  D405 depth+color depth_frame 토픽 구독 → Open3D RGBD → cloud → voxel down
  → 바이너리 [u32 N][float32 xyz×3N][u8 rgb×3N] (LE)
  → `omx/pointcloud/stream` (카메라 프레임 그대로 publish — 프론트에서 cam→base transform).

세션 캡처 (Phase 4a):
  포인트마다 정지 → CAPTURE → 카메라 Pi에서 N장 raw 받아 median → cam→base 변환
  → npz(원본) + PLY(시각화) 저장 → snapshot 토픽 publish.
  여러 자세의 scan을 한 세션 디렉토리(`robot/scans/{session_id}/`)에 누적.
  추후 BUILD_MESH가 세션 전체를 TSDF로 합쳐 메시 1개 산출.
"""

import base64
import logging
import re
import struct
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d

from core.base_node import BaseNode
from core.topic_map import Service, Topic
from modules.calibration.loader import load_calibration
from modules.camera.depth_frame import (
    DepthFrame,
    decode as decode_depth_frame,
    envelope_decode,
)

logger = logging.getLogger(__name__)

DEFAULT_VOXEL_SIZE = 0.005  # 5mm
TARGET_FPS = 8.0
IDLE_SLEEP = 0.1
DEPTH_TRUNC_STREAM = 1.0  # m (라이브)
DEPTH_TRUNC_CAPTURE = 0.8  # m (캡처)
DEFAULT_NUM_FRAMES = 5
CAPTURE_TIMEOUT = 3.0

ROBOT_DIR = Path(__file__).parents[2] / "robot"
SCANS_DIR = ROBOT_DIR / "scans"
MODELS_DIR = ROBOT_DIR / "models"

DEFAULT_TSDF_VOXEL = 0.002  # 2mm
DEFAULT_TSDF_DEPTH_TRUNC = 0.8  # m

_SESSION_ID_RE = re.compile(r"^[A-Za-z0-9_\-]+$")


class PointCloudNode(BaseNode):
    def __init__(self) -> None:
        super().__init__("pointcloud_node")
        self._cfg_lock = threading.Lock()
        self._enabled = False
        self._voxel_size = DEFAULT_VOXEL_SIZE

        self._frame_lock = threading.Lock()
        self._latest_frame: DepthFrame | None = None

        self._session_lock = threading.Lock()
        self._current_session_id: str | None = None

        self._stream_thread: threading.Thread | None = None

    def start(self) -> None:
        self.create_service(Service.POINTCLOUD_CONFIGURE, self._srv_configure)
        self.create_service(Service.POINTCLOUD_CAPTURE, self._srv_capture)
        self.create_service(Service.POINTCLOUD_NEW_SESSION, self._srv_new_session)
        self.create_service(Service.POINTCLOUD_LIST_SCANS, self._srv_list_scans)
        self.create_service(Service.POINTCLOUD_LOAD_SCAN, self._srv_load_scan)
        self.create_service(
            Service.POINTCLOUD_CLEAR_SNAPSHOT, self._srv_clear_snapshot
        )
        self.create_service(Service.POINTCLOUD_BUILD_MESH, self._srv_build_mesh)
        self.create_service(
            Service.POINTCLOUD_LIST_MESHES, self._srv_list_meshes
        )
        self.create_raw_subscriber(
            Topic.CAMERA_DEPTH_FRAME, self._on_depth_frame
        )
        super().start()
        self._stream_thread = threading.Thread(
            target=self._stream_loop,
            name="pointcloud-stream",
            daemon=True,
        )
        self._stream_thread.start()
        self._publish_state()

    # ─── Subscriber ──────────────────────────────────────────

    def _on_depth_frame(self, payload: bytes) -> None:
        try:
            frame = decode_depth_frame(payload)
        except Exception as e:
            logger.warning(f"depth_frame 디코드 실패: {e}")
            return
        with self._frame_lock:
            self._latest_frame = frame

    # ─── Service: configure (라이브 스트림) ──────────────────

    def _srv_configure(self, req: dict) -> dict:
        data = req.get("data", {}) or {}

        if "voxel_size" in data:
            v = float(data["voxel_size"])
            if v <= 0:
                return {
                    "success": False,
                    "message": "voxel_size > 0 필요",
                    "data": {},
                }
            with self._cfg_lock:
                self._voxel_size = v

        if "enabled" in data:
            target = bool(data["enabled"])
            res = self.call_service(
                Service.CAMERA_SET_DEPTH_STREAM,
                {"enabled": target},
            )
            if not res.get("success"):
                return {
                    "success": False,
                    "message": f"카메라 depth 스트림 전환 실패: {res.get('message')}",
                    "data": {},
                }
            with self._cfg_lock:
                self._enabled = target
            if not target:
                with self._frame_lock:
                    self._latest_frame = None

        with self._cfg_lock:
            state = {
                "enabled": self._enabled,
                "voxel_size": self._voxel_size,
            }
        self._publish_state()
        return {"success": True, "message": "ok", "data": state}

    def _publish_state(self) -> None:
        with self._cfg_lock, self._session_lock:
            self.publish(
                Topic.POINTCLOUD_STATE,
                {
                    "timestamp": time.time(),
                    "enabled": self._enabled,
                    "voxel_size": self._voxel_size,
                    "session_id": self._current_session_id,
                },
            )

    # ─── Service: new_session ────────────────────────────────

    def _srv_new_session(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        explicit = data.get("session_id")
        if explicit is not None:
            sid = str(explicit)
            if not _SESSION_ID_RE.match(sid):
                return {
                    "success": False,
                    "message": "session_id는 영문/숫자/_- 만 허용",
                    "data": {},
                }
        else:
            sid = _new_session_id()

        session_dir = SCANS_DIR / sid
        try:
            session_dir.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            return {
                "success": False,
                "message": f"세션 디렉토리 생성 실패: {e}",
                "data": {},
            }

        with self._session_lock:
            self._current_session_id = sid
        self._publish_state()
        rel = session_dir.relative_to(ROBOT_DIR).as_posix()
        return {
            "success": True,
            "message": f"세션 시작: {sid}",
            "data": {"session_id": sid, "path": rel},
        }

    # ─── Service: capture ────────────────────────────────────

    def _srv_capture(self, req: dict) -> dict:
        data = req.get("data", {}) or {}

        calib = load_calibration()
        if calib.hand_eye is None:
            return {
                "success": False,
                "message": "hand_eye 캘리브레이션 필요",
                "data": {},
            }

        num_frames = _coerce_int(
            data.get("num_frames"), DEFAULT_NUM_FRAMES
        )
        if num_frames <= 0 or num_frames > 30:
            return {
                "success": False,
                "message": "num_frames는 1~30 범위",
                "data": {},
            }

        explicit_sid = data.get("session_id")
        with self._session_lock:
            sid = (
                str(explicit_sid)
                if explicit_sid is not None
                else self._current_session_id
            )
        if sid is None:
            sid = _new_session_id()
            with self._session_lock:
                self._current_session_id = sid
            self._publish_state()
            logger.info(f"세션 자동 생성: {sid}")

        if not _SESSION_ID_RE.match(sid):
            return {
                "success": False,
                "message": "session_id는 영문/숫자/_- 만 허용",
                "data": {},
            }

        session_dir = SCANS_DIR / sid
        try:
            session_dir.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            return {
                "success": False,
                "message": f"세션 디렉토리 생성 실패: {e}",
                "data": {},
            }

        tcp_res = self.call_service(Service.MOTION_GET_TCP, {})
        if not tcp_res.get("success"):
            return {
                "success": False,
                "message": f"TCP pose 실패: {tcp_res.get('message')}",
                "data": {},
            }
        tcp_data = tcp_res.get("data", {})
        pos = tcp_data.get("position")
        quat = tcp_data.get("quaternion")
        if pos is None or quat is None:
            return {"success": False, "message": "TCP pose 없음", "data": {}}

        cam_res = self.call_service(
            Service.CAMERA_CAPTURE_DEPTH_FRAMES,
            {"num_frames": num_frames, "timeout": CAPTURE_TIMEOUT},
            timeout=CAPTURE_TIMEOUT + 2.0,
        )
        if not cam_res.get("success"):
            return {
                "success": False,
                "message": f"카메라 캡처 실패: {cam_res.get('message')}",
                "data": {},
            }
        payload_b64 = (cam_res.get("data") or {}).get("payload_b64")
        if not payload_b64:
            return {
                "success": False,
                "message": "카메라 응답에 payload_b64 없음",
                "data": {},
            }

        try:
            envelope = base64.b64decode(payload_b64)
            frame_blobs = envelope_decode(envelope)
            frames = [decode_depth_frame(b) for b in frame_blobs]
        except Exception as e:
            return {
                "success": False,
                "message": f"frame envelope 디코드 실패: {e}",
                "data": {},
            }

        if not frames:
            return {
                "success": False,
                "message": "frame 0개 수신",
                "data": {},
            }

        # ── averaging (depth median, color: last) ───────────
        try:
            depth_avg, color_picked = _median_depth_pick_color(frames)
        except Exception as e:
            return {
                "success": False,
                "message": f"averaging 실패: {e}",
                "data": {},
            }
        ref = frames[0]

        # ── cam→base 변환 ───────────────────────────────────
        try:
            T_cam_to_base = _build_cam_to_base(
                quat,
                pos,
                np.asarray(calib.hand_eye.R, dtype=np.float64),
                np.asarray(calib.hand_eye.t, dtype=np.float64).flatten(),
            )
        except Exception as e:
            return {
                "success": False,
                "message": f"변환 행렬 빌드 실패: {e}",
                "data": {},
            }

        # ── 포인트클라우드 빌드 (base frame으로 변환) ────────
        try:
            pcd = _build_pcd_from_arrays(
                color_picked,
                depth_avg,
                fx=ref.fx,
                fy=ref.fy,
                cx=ref.cx,
                cy=ref.cy,
                width=ref.width,
                height=ref.height,
                depth_scale=ref.depth_scale,
                depth_trunc=DEPTH_TRUNC_CAPTURE,
            )
            pcd.transform(T_cam_to_base)
        except Exception as e:
            return {
                "success": False,
                "message": f"포인트클라우드 생성 실패: {e}",
                "data": {},
            }

        # ── 저장 (npz + ply) ────────────────────────────────
        scan_idx = _next_scan_index(session_dir)
        scan_stem = f"scan_{scan_idx:03d}"
        npz_path = session_dir / f"{scan_stem}.npz"
        ply_path = session_dir / f"{scan_stem}.ply"

        try:
            np.savez_compressed(
                npz_path,
                depth_z16=depth_avg,
                color_bgr=color_picked,
                fx=np.float64(ref.fx),
                fy=np.float64(ref.fy),
                cx=np.float64(ref.cx),
                cy=np.float64(ref.cy),
                width=np.int32(ref.width),
                height=np.int32(ref.height),
                depth_scale=np.float64(ref.depth_scale),
                tcp_position=np.asarray(pos, dtype=np.float64),
                tcp_quaternion=np.asarray(quat, dtype=np.float64),
                hand_eye_R=np.asarray(calib.hand_eye.R, dtype=np.float64),
                hand_eye_t=np.asarray(
                    calib.hand_eye.t, dtype=np.float64
                ).flatten(),
                timestamp=np.float64(time.time()),
                depth_trunc=np.float64(DEPTH_TRUNC_CAPTURE),
                num_frames=np.int32(len(frames)),
            )
        except Exception as e:
            return {
                "success": False,
                "message": f"npz 저장 실패: {e}",
                "data": {},
            }

        try:
            o3d.io.write_point_cloud(str(ply_path), pcd)
        except Exception as e:
            return {
                "success": False,
                "message": f"PLY 저장 실패: {e}",
                "data": {},
            }

        self._publish_snapshot(pcd)

        npz_rel = npz_path.relative_to(ROBOT_DIR).as_posix()
        ply_rel = ply_path.relative_to(ROBOT_DIR).as_posix()
        point_count = len(pcd.points)
        return {
            "success": True,
            "message": f"저장: {ply_rel}",
            "data": {
                "session_id": sid,
                "scan_index": scan_idx,
                "npz_path": npz_rel,
                "ply_path": ply_rel,
                "point_count": point_count,
                "num_frames": len(frames),
            },
        }

    # ─── Service: list_scans ─────────────────────────────────

    def _srv_list_scans(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        target_sid = data.get("session_id")

        if target_sid is None:
            sessions: list[dict] = []
            if SCANS_DIR.exists():
                for d in sorted(SCANS_DIR.iterdir()):
                    if not d.is_dir():
                        continue
                    plys = sorted(d.glob("scan_*.ply"))
                    sessions.append({
                        "session_id": d.name,
                        "path": d.relative_to(ROBOT_DIR).as_posix(),
                        "scan_count": len(plys),
                    })
            with self._session_lock:
                current = self._current_session_id
            return {
                "success": True,
                "message": "ok",
                "data": {
                    "sessions": sessions,
                    "current_session_id": current,
                },
            }

        sid = str(target_sid)
        if not _SESSION_ID_RE.match(sid):
            return {
                "success": False,
                "message": "session_id는 영문/숫자/_- 만 허용",
                "data": {},
            }
        session_dir = SCANS_DIR / sid
        if not session_dir.exists():
            return {
                "success": False,
                "message": f"세션 없음: {sid}",
                "data": {},
            }

        scans: list[dict] = []
        for ply in sorted(session_dir.glob("scan_*.ply")):
            scans.append({
                "name": ply.stem,
                "ply_path": ply.relative_to(ROBOT_DIR).as_posix(),
                "size": ply.stat().st_size,
            })
        return {
            "success": True,
            "message": "ok",
            "data": {"session_id": sid, "scans": scans},
        }

    # ─── Service: load_scan ──────────────────────────────────

    def _srv_load_scan(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        rel = data.get("path") or data.get("ply_path")
        if not rel:
            return {"success": False, "message": "path 필요", "data": {}}

        path = ROBOT_DIR / rel
        try:
            resolved = path.resolve()
            resolved.relative_to(ROBOT_DIR.resolve())
        except (ValueError, OSError):
            return {"success": False, "message": "잘못된 경로", "data": {}}
        if not resolved.exists() or resolved.suffix.lower() != ".ply":
            return {
                "success": False,
                "message": f"PLY 파일 없음: {rel}",
                "data": {},
            }

        try:
            pcd = o3d.io.read_point_cloud(str(resolved))
        except Exception as e:
            return {
                "success": False,
                "message": f"PLY 로드 실패: {e}",
                "data": {},
            }

        self._publish_snapshot(pcd)
        return {
            "success": True,
            "message": "ok",
            "data": {
                "path": rel,
                "point_count": len(pcd.points),
            },
        }

    # ─── Service: clear_snapshot ─────────────────────────────

    def _srv_clear_snapshot(self, req: dict) -> dict:
        empty = struct.pack("<I", 0)
        try:
            self.session.put(Topic.POINTCLOUD_SNAPSHOT, empty)
        except Exception as e:
            return {"success": False, "message": str(e), "data": {}}
        return {"success": True, "message": "ok", "data": {}}

    # ─── Service: build_mesh (TSDF) ──────────────────────────

    def _srv_build_mesh(self, req: dict) -> dict:
        data = req.get("data", {}) or {}
        sid = data.get("session_id")
        if not sid:
            return {"success": False, "message": "session_id 필요", "data": {}}
        sid = str(sid)
        if not _SESSION_ID_RE.match(sid):
            return {
                "success": False,
                "message": "session_id는 영문/숫자/_- 만 허용",
                "data": {},
            }

        session_dir = SCANS_DIR / sid
        if not session_dir.exists():
            return {
                "success": False,
                "message": f"세션 없음: {sid}",
                "data": {},
            }

        npz_paths = sorted(session_dir.glob("scan_*.npz"))
        if not npz_paths:
            return {
                "success": False,
                "message": f"scan_*.npz 없음 (session={sid})",
                "data": {},
            }

        voxel = float(data.get("voxel_size", DEFAULT_TSDF_VOXEL))
        sdf_trunc = float(data.get("sdf_trunc", voxel * 5))
        depth_trunc = float(data.get("depth_trunc", DEFAULT_TSDF_DEPTH_TRUNC))
        if voxel <= 0 or sdf_trunc <= 0 or depth_trunc <= 0:
            return {
                "success": False,
                "message": "voxel_size / sdf_trunc / depth_trunc는 양수",
                "data": {},
            }

        t0 = time.time()
        try:
            volume = o3d.pipelines.integration.ScalableTSDFVolume(
                voxel_length=voxel,
                sdf_trunc=sdf_trunc,
                color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
            )
        except Exception as e:
            return {
                "success": False,
                "message": f"TSDFVolume 생성 실패: {e}",
                "data": {},
            }

        integrated = 0
        for npz_path in npz_paths:
            try:
                _integrate_scan(volume, npz_path, depth_trunc)
                integrated += 1
            except Exception as e:
                logger.warning(f"scan {npz_path.name} 적분 실패: {e}")

        if integrated == 0:
            return {
                "success": False,
                "message": "모든 scan 적분 실패",
                "data": {},
            }

        try:
            mesh = volume.extract_triangle_mesh()
            mesh.compute_vertex_normals()
        except Exception as e:
            return {
                "success": False,
                "message": f"메시 추출 실패: {e}",
                "data": {},
            }

        vertex_count = len(mesh.vertices)
        triangle_count = len(mesh.triangles)
        if vertex_count == 0:
            return {
                "success": False,
                "message": "메시 비어있음 (volume 적분 결과 없음)",
                "data": {},
            }

        try:
            MODELS_DIR.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            return {
                "success": False,
                "message": f"models 디렉토리 생성 실패: {e}",
                "data": {},
            }

        out_path = MODELS_DIR / f"mesh_{sid}.ply"
        try:
            o3d.io.write_triangle_mesh(
                str(out_path), mesh, write_vertex_normals=True
            )
        except Exception as e:
            return {
                "success": False,
                "message": f"PLY 저장 실패: {e}",
                "data": {},
            }

        elapsed = time.time() - t0
        rel = out_path.relative_to(ROBOT_DIR).as_posix()
        logger.info(
            f"build_mesh session={sid} scans={integrated}/{len(npz_paths)} "
            f"vertices={vertex_count} triangles={triangle_count} "
            f"voxel={voxel*1000:.1f}mm elapsed={elapsed:.2f}s"
        )
        return {
            "success": True,
            "message": (
                f"mesh: {rel} ({vertex_count} vertices, {triangle_count} triangles)"
            ),
            "data": {
                "session_id": sid,
                "path": rel,
                "vertex_count": vertex_count,
                "triangle_count": triangle_count,
                "integrated_scans": integrated,
                "total_scans": len(npz_paths),
                "voxel_size": voxel,
                "sdf_trunc": sdf_trunc,
                "depth_trunc": depth_trunc,
                "elapsed": elapsed,
            },
        }

    # ─── Service: list_meshes ────────────────────────────────

    def _srv_list_meshes(self, _req: dict) -> dict:
        meshes: list[dict] = []
        if MODELS_DIR.exists():
            for ply in sorted(MODELS_DIR.glob("mesh_*.ply")):
                try:
                    st = ply.stat()
                except OSError:
                    continue
                meshes.append({
                    "name": ply.stem,
                    "path": ply.relative_to(ROBOT_DIR).as_posix(),
                    "size": st.st_size,
                    "mtime": st.st_mtime,
                })
        return {
            "success": True,
            "message": "ok",
            "data": {"meshes": meshes},
        }

    # ─── Snapshot publish ────────────────────────────────────

    def _publish_snapshot(self, pcd: "o3d.geometry.PointCloud") -> None:
        xyz = np.asarray(pcd.points, dtype=np.float32)
        if pcd.has_colors():
            rgb_f = np.asarray(pcd.colors, dtype=np.float32)
            rgb = (np.clip(rgb_f, 0.0, 1.0) * 255.0).astype(np.uint8)
        else:
            rgb = np.full((xyz.shape[0], 3), 200, dtype=np.uint8)
        n = xyz.shape[0]
        payload = struct.pack("<I", n) + xyz.tobytes() + rgb.tobytes()
        self.session.put(Topic.POINTCLOUD_SNAPSHOT, payload)

    # ─── Stream Loop (라이브) ────────────────────────────────

    def _stream_loop(self) -> None:
        period = 1.0 / TARGET_FPS
        last_processed_ts = 0.0

        while self._running:
            with self._cfg_lock:
                enabled = self._enabled
                voxel = self._voxel_size

            if not enabled:
                time.sleep(IDLE_SLEEP)
                continue

            with self._frame_lock:
                frame = self._latest_frame

            if frame is None or frame.timestamp <= last_processed_ts:
                time.sleep(IDLE_SLEEP)
                continue

            t0 = time.time()
            try:
                payload = self._build_stream_payload(frame, voxel)
            except Exception as e:
                logger.warning(f"포인트클라우드 생성 실패: {e}")
                time.sleep(IDLE_SLEEP)
                continue

            try:
                self.session.put(Topic.POINTCLOUD_STREAM, payload)
                last_processed_ts = frame.timestamp
            except Exception as e:
                logger.warning(f"포인트클라우드 발행 실패: {e}")

            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))

    def _build_stream_payload(
        self, frame: DepthFrame, voxel_size: float
    ) -> bytes:
        pcd = _build_pcd_from_arrays(
            frame.color_bgr,
            frame.depth_z16,
            fx=frame.fx,
            fy=frame.fy,
            cx=frame.cx,
            cy=frame.cy,
            width=frame.width,
            height=frame.height,
            depth_scale=frame.depth_scale,
            depth_trunc=DEPTH_TRUNC_STREAM,
        )
        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size)

        xyz = np.asarray(pcd.points, dtype=np.float32)
        rgb_f = np.asarray(pcd.colors, dtype=np.float32)
        rgb_u8 = (np.clip(rgb_f, 0.0, 1.0) * 255.0).astype(np.uint8)

        n = xyz.shape[0]
        return struct.pack("<I", n) + xyz.tobytes() + rgb_u8.tobytes()


# ─── helpers ─────────────────────────────────────────────────


def _coerce_int(v: Any, default: int) -> int:
    if v is None:
        return default
    try:
        return int(v)
    except (TypeError, ValueError):
        return default


def _new_session_id() -> str:
    return time.strftime("session_%Y%m%d_%H%M%S")


def _next_scan_index(session_dir: Path) -> int:
    indices: list[int] = []
    for p in session_dir.glob("scan_*.ply"):
        m = re.match(r"scan_(\d+)\.ply$", p.name)
        if m:
            try:
                indices.append(int(m.group(1)))
            except ValueError:
                pass
    return (max(indices) + 1) if indices else 1


def _median_depth_pick_color(
    frames: list[DepthFrame],
) -> tuple[np.ndarray, np.ndarray]:
    """N개 depth 픽셀별 median (0=invalid 제외), color는 마지막 1장."""
    depths = np.stack([f.depth_z16 for f in frames], axis=0)  # (N, H, W) uint16
    INVALID_HIGH = np.iinfo(np.uint16).max
    masked = np.where(depths == 0, INVALID_HIGH, depths)
    median = np.median(masked, axis=0).astype(np.uint16)
    median[median == INVALID_HIGH] = 0
    color = frames[-1].color_bgr
    return median, color


def _build_pcd_from_arrays(
    color_bgr: np.ndarray,
    depth_z16: np.ndarray,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    width: int,
    height: int,
    depth_scale: float,
    depth_trunc: float,
) -> "o3d.geometry.PointCloud":
    rgb = np.ascontiguousarray(color_bgr[:, :, ::-1])
    color_o3d = o3d.geometry.Image(rgb)
    depth_o3d = o3d.geometry.Image(depth_z16)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1.0 / depth_scale,
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=False,
    )
    pinhole = o3d.camera.PinholeCameraIntrinsic(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
    )
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole)


def _build_cam_to_base(
    quat_xyzw: list[float],
    t_eb: list[float],
    R_ce: np.ndarray,
    t_ce: np.ndarray,
) -> np.ndarray:
    R_be = _quat_to_rot(quat_xyzw)
    T_eb = np.eye(4)
    T_eb[:3, :3] = R_be
    T_eb[:3, 3] = np.asarray(t_eb)
    T_ce = np.eye(4)
    T_ce[:3, :3] = R_ce
    T_ce[:3, 3] = t_ce
    return T_eb @ T_ce


def _integrate_scan(
    volume: "o3d.pipelines.integration.ScalableTSDFVolume",
    npz_path: Path,
    depth_trunc: float,
) -> None:
    """단일 scan_*.npz를 TSDF volume에 적분.

    Open3D `volume.integrate(rgbd, intrinsic, extrinsic)`의 extrinsic은
    `T_camera ← world` 컨벤션 (world point를 camera frame으로 변환).
    npz에 저장된 hand_eye(R/t)와 tcp_quaternion/position으로
    `T_cam_to_base = T_base←ee · T_ee←cam`을 빌드하고 그 역행렬을 넘긴다.
    """
    s = np.load(npz_path)
    color_bgr = s["color_bgr"]
    depth_z16 = s["depth_z16"]
    fx = float(s["fx"])
    fy = float(s["fy"])
    cx = float(s["cx"])
    cy = float(s["cy"])
    width = int(s["width"])
    height = int(s["height"])
    depth_scale = float(s["depth_scale"])

    rgb = np.ascontiguousarray(color_bgr[:, :, ::-1])
    color_o3d = o3d.geometry.Image(rgb)
    depth_o3d = o3d.geometry.Image(depth_z16)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1.0 / depth_scale,
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=False,
    )
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy
    )

    T_cam_to_base = _build_cam_to_base(
        s["tcp_quaternion"].tolist(),
        s["tcp_position"].tolist(),
        np.asarray(s["hand_eye_R"], dtype=np.float64),
        np.asarray(s["hand_eye_t"], dtype=np.float64).flatten(),
    )
    extrinsic = np.linalg.inv(T_cam_to_base)
    volume.integrate(rgbd, intrinsic, extrinsic)


def _quat_to_rot(q: list[float]) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
