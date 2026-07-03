"""Scan domain — public contract surface.

scan workflow + persistence + reconstruction. 옛 backend 의 StorageNode(scan 엔티티)
+ ReconstructionNode + ScanTask orchestration 을 v2 Database-per-Module 로 통합
(centralized Storage Module 은 v2 에서 폐기 — 각 module 이 자기 영속성 소유).

Task DSL 없이 frontend 가 서비스 직접 호출 (실용 슬라이스). 3 엔티티:
scan_sessions / scans / reconstructions (append-only blob + immutable metadata row).
"""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum

from pydantic import BaseModel, ConfigDict


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


# ─── records (DB row ↔ wire) ────────────────────────────────────────


class ScanSessionRecord(_Strict):
    id: int | None = None
    robot_id: str
    session_id: str  # human-readable (session_YYYYMMDD_HHMMSS)
    created_at: datetime
    label: str | None = None


class ScanRecord(_Strict):
    id: int | None = None
    session_row_id: int
    robot_id: str
    scan_id: int  # session 내 monotonic (삭제해도 안 줄어듦)
    created_at: datetime
    blob_key: str  # ObjectStore key
    num_frames: int
    # snapshot 시점 intrinsic (depth 해상도 기준)
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float
    # 캡처 시점 raw motor position (arm_motor_ids 와 parallel). raw SSOT — build 시
    # 현재 캘로 FK 재계산.
    motor_positions: list[int]
    arm_motor_ids: list[int]


class ReconstructionRecord(_Strict):
    id: int | None = None
    session_row_id: int
    robot_id: str
    created_at: datetime
    blob_key: str  # .ply
    voxel_size: float
    sdf_trunc: float
    depth_trunc: float
    icp_max_dist: float
    n_scans: int
    n_edges: int
    vertex_count: int
    triangle_count: int
    elapsed: float


# ─── nested contract ────────────────────────────────────────────────


class Scan:
    class Service(StrEnum):
        NEW_SESSION = "srv/scan/{robot_id}/new_session"
        LIST_SESSIONS = "srv/scan/{robot_id}/list_sessions"
        DELETE_SESSION = "srv/scan/{robot_id}/delete_session"
        CAPTURE = "srv/scan/{robot_id}/capture"
        LIST_SCANS = "srv/scan/{robot_id}/list_scans"
        DELETE_SCAN = "srv/scan/{robot_id}/delete_scan"
        BUILD = "srv/scan/{robot_id}/build"
        LIST_RECONSTRUCTIONS = "srv/scan/{robot_id}/list_reconstructions"
        GET_MESH = "srv/scan/{robot_id}/get_mesh"

    class Stream(StrEnum):
        BUILD_PROGRESS = "stream/scan/{robot_id}/build_progress"


BuildStage = str  # loading_scans/pairwise_registration/pose_graph/tsdf/mesh/done


# ─── request / response ─────────────────────────────────────────────


class NewSessionRequest(BaseModel):
    label: str | None = None


class NewSessionResponse(BaseModel):
    session: ScanSessionRecord


class ListSessionsRequest(BaseModel):
    pass


class ListSessionsResponse(BaseModel):
    sessions: list[ScanSessionRecord]


class DeleteSessionRequest(BaseModel):
    session_row_id: int


class DeleteSessionResponse(BaseModel):
    ok: bool


class CaptureRequest(BaseModel):
    session_row_id: int
    num_frames: int = 10


class CaptureResponse(BaseModel):
    accepted: bool
    scan: ScanRecord | None = None
    scan_count: int = 0
    message: str = ""


class ListScansRequest(BaseModel):
    session_row_id: int


class ListScansResponse(BaseModel):
    scans: list[ScanRecord]


class DeleteScanRequest(BaseModel):
    scan_row_id: int


class DeleteScanResponse(BaseModel):
    ok: bool


class BuildRequest(BaseModel):
    session_row_id: int
    voxel_size: float | None = None
    sdf_trunc: float | None = None
    depth_trunc: float | None = None
    icp_max_dist: float | None = None


class BuildResponse(BaseModel):
    accepted: bool
    reconstruction: ReconstructionRecord | None = None
    message: str = ""


class ListReconstructionsRequest(BaseModel):
    session_row_id: int


class ListReconstructionsResponse(BaseModel):
    reconstructions: list[ReconstructionRecord]


class GetMeshRequest(BaseModel):
    reconstruction_row_id: int


class GetMeshResponse(BaseModel):
    ply_bytes: bytes  # msgpack bin — frontend PLYLoader
    vertex_count: int
    triangle_count: int


# ─── stream payload ─────────────────────────────────────────────────


class BuildProgress(BaseModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    session_row_id: int
    stage: str
    percent: float  # 0..1 (stage 내)
    message: str = ""
    reconstruction_row_id: int | None = None  # done stage 에서 set
