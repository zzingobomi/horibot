"""PointCloud 노드 토픽 / 서비스 payload schema.

토픽:
- POINTCLOUD_STREAM (publish, raw binary) — typed 안 함 (binary 트랙)
- POINTCLOUD_STATE  (publish) — PointcloudState

서비스 (request data / response data):
- POINTCLOUD_CONFIGURE       — PointcloudConfigureReq / PointcloudConfigureRes
- POINTCLOUD_NEW_SESSION     — PointcloudNewSessionReq / PointcloudNewSessionRes
- POINTCLOUD_CAPTURE         — PointcloudCaptureReq / PointcloudCaptureRes
- POINTCLOUD_LIST_SESSIONS   — EmptyData / PointcloudListSessionsRes
- POINTCLOUD_LIST_SCANS      — PointcloudListScansReq / PointcloudListScansRes
- POINTCLOUD_DELETE_SCAN     — PointcloudDeleteScanReq / PointcloudDeleteScanRes
- POINTCLOUD_BUILD_MESH      — PointcloudBuildMeshReq / PointcloudBuildMeshRes
- POINTCLOUD_LIST_MESHES     — EmptyData / PointcloudListMeshesRes
"""

from __future__ import annotations

from core.transport.messages.base import StrictModel



# ─── Topic: POINTCLOUD_STATE ─────────────────────────────────────────


class PointcloudState(StrictModel):
    timestamp: float
    enabled: bool
    voxel_size: float


# ─── Service: POINTCLOUD_CONFIGURE ───────────────────────────────────


class PointcloudConfigureReq(StrictModel):
    """변경할 필드만 명시 (None 이면 미변경)."""

    enabled: bool | None = None
    voxel_size: float | None = None


class PointcloudConfigureRes(StrictModel):
    """전환 후 현재 상태 echo."""

    enabled: bool
    voxel_size: float


# ─── Service: POINTCLOUD_NEW_SESSION ─────────────────────────────────


class PointcloudNewSessionReq(StrictModel):
    """빈 문자열이면 자동 default (현재 시각 기반)."""

    session_id: str = ""


class PointcloudNewSessionRes(StrictModel):
    session_id: str


# ─── Service: POINTCLOUD_CAPTURE ─────────────────────────────────────


class PointcloudCaptureReq(StrictModel):
    session_id: str
    num_frames: int | None = None


class PointcloudCaptureRes(StrictModel):
    session_id: str
    scan_id: int
    path: str  # robot/ 기준 상대경로
    num_frames: int


# ─── Service: POINTCLOUD_LIST_SESSIONS ───────────────────────────────


class PointcloudListSessionsRes(StrictModel):
    sessions: list[str]


# ─── Service: POINTCLOUD_LIST_SCANS ──────────────────────────────────


class PointcloudListScansReq(StrictModel):
    session_id: str


class ScanMeta(StrictModel):
    """scan_io.scan_meta() 결과 1개."""

    id: int
    path: str
    timestamp: float
    num_frames: int


class PointcloudListScansRes(StrictModel):
    session_id: str
    scans: list[ScanMeta]


# ─── Service: POINTCLOUD_DELETE_SCAN ─────────────────────────────────


class PointcloudDeleteScanReq(StrictModel):
    session_id: str
    scan_id: int


class PointcloudDeleteScanRes(StrictModel):
    session_id: str
    scan_id: int


# ─── Service: POINTCLOUD_BUILD_MESH ──────────────────────────────────


class PointcloudBuildMeshReq(StrictModel):
    """TSDF 파라미터는 모두 optional — None 이면 tsdf_builder default."""

    session_id: str
    voxel_size: float | None = None
    sdf_trunc: float | None = None
    depth_trunc: float | None = None
    icp_max_dist: float | None = None


class PointcloudBuildMeshRes(StrictModel):
    session_id: str
    path: str
    vertex_count: int
    triangle_count: int
    n_scans: int
    n_edges: int
    elapsed: float


# ─── Service: POINTCLOUD_LIST_MESHES ─────────────────────────────────


class MeshMeta(StrictModel):
    session_id: str
    path: str
    size: int
    mtime: float


class PointcloudListMeshesRes(StrictModel):
    meshes: list[MeshMeta]
