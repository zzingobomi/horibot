"""Storage 노드 service / topic payload schema.

docs/storage_layer.md §2 — Zenoh service gateway. 4 service + 1 topic.

Service:
- STORAGE_GET_ACTIVE_CALIBRATION    — (req: kind, robot_id) → 활성 result 수치
- STORAGE_LIST_CALIBRATIONS         — (req: kind, robot_id, limit) → list (history)
- STORAGE_COMMIT_CALIBRATION        — (req: run + result + captures) → run_id + result_ids
- STORAGE_ACTIVATE_CALIBRATION      — (req: result_id) → activated result

Topic:
- STORAGE_CALIBRATION_INVALIDATED   — (payload: robot_id, kind) — ACTIVATE 마다 1회

상세는 docs/storage_layer.md §4 (commit/activate 흐름) + §7 (노드 측 패턴).
"""

from __future__ import annotations

from pydantic import Base64Bytes

from core.transport.messages.base import StrictModel
from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationRunRecord,
)
from modules.scan_workflow.persistence_models import (
    ReconstructionRecord,
    ScanRecord,
    ScanSessionRecord,
)


# ─── Service: STORAGE_GET_ACTIVE_CALIBRATION ───────────────────


class StorageGetActiveReq(StrictModel):
    robot_id: str
    kind: CalibrationKind


class StorageGetActiveRes(StrictModel):
    """found=False 면 활성 result 없음 — 첫 부팅 robot. caller 가 default fallback."""

    found: bool
    result: CalibrationResultRecord | None = None


# ─── Service: STORAGE_LIST_CALIBRATIONS ────────────────────────


class StorageListReq(StrictModel):
    robot_id: str
    kind: CalibrationKind
    limit: int = 100


class StorageListRes(StrictModel):
    results: list[CalibrationResultRecord]


# ─── Service: STORAGE_LIST_CALIBRATION_RUNS ────────────────────


class CalibrationRunSummary(StrictModel):
    """Run + 그 Run 의 모든 kind Result. frontend list/ACTIVATE 패널이 한 Run
    한 row 로 펼침 — 5 kind 같이 보이고 ACTIVATE 도 Run 전체 / kind 별 양쪽 가능.

    storage_layer.md §13.7 Stage 4 design A.
    """

    run: CalibrationRunRecord
    results: list[CalibrationResultRecord]


class StorageListRunsReq(StrictModel):
    robot_id: str
    limit: int = 50


class StorageListRunsRes(StrictModel):
    """`run.started_at DESC` 정렬. 각 Run 마다 그 Run 의 모든 Result 가 묶여 옴."""

    runs: list[CalibrationRunSummary]


# ─── Service: STORAGE_COMMIT_CALIBRATION ───────────────────────


class StorageCommitReq(StrictModel):
    """한 Run + 그 산출물 (Result list) + Evidence (Capture list) atomic INSERT.

    run.id / results[*].id 는 무시 (storage 가 부여). results[*].run_id 도
    무시 (storage 가 새 run_id 로 덮어씀). caller 가 임시 placeholder 채우거나
    None 두면 됨.

    INSERT 시 모든 result.is_active=false — caller 가 받은 result_id 로
    ACTIVATE 별도 호출.
    """

    run: CalibrationRunRecord
    results: list[CalibrationResultRecord]
    captures: list[CalibrationCaptureRecord] = []


class StorageCommitRes(StrictModel):
    run_id: int
    result_ids: list[int]


# ─── Service: STORAGE_ACTIVATE_CALIBRATION ─────────────────────


class StorageActivateReq(StrictModel):
    result_id: int


class StorageActivateRes(StrictModel):
    """activated result 의 robot_id / kind 는 frontend 가 invalidation 확인 시 사용."""

    result: CalibrationResultRecord


# ─── Phase 2 — scan workflow ───────────────────────────────────
# blob_key 자리는 server 결정 (race 차단). blob bytes wire = opaque — caller 가
# scan_workflow.blob 의 encode/decode 자리 사용. GET_BLOB 자리 generic (scan /
# reconstruction 공통).


# ── scan_sessions
class StorageNewScanSessionReq(StrictModel):
    robot_id: str
    session_id: str = ""  # 빈 자리 server 가 시간 기반 default
    label: str | None = None
    note: str | None = None


class StorageNewScanSessionRes(StrictModel):
    session: ScanSessionRecord


class StorageListScanSessionsReq(StrictModel):
    robot_id: str
    limit: int = 100


class StorageListScanSessionsRes(StrictModel):
    sessions: list[ScanSessionRecord]


class StorageDeleteScanSessionReq(StrictModel):
    """CASCADE — 자식 scans / reconstructions 자리 자동 삭제 (RDB + ObjectStore blob).

    blob 자리도 server 가 같이 삭제 자리 — RDB row 자리 fetch 후 blob_key 순회.
    """

    session_row_id: int


# ── scans
class StoragePutScanReq(StrictModel):
    """snapshot 자리 받아 storage 자리 commit. scan_id 자리는 server alloc."""

    session_row_id: int
    blob_bytes: Base64Bytes  # scan_workflow.blob.encode_snapshot() 결과 — opaque
    # snapshot metadata (Scene3DSnapshotRes 자리 자리)
    num_frames: int
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float
    motor_positions: list[int]
    arm_motor_ids: list[int]


class StoragePutScanRes(StrictModel):
    scan: ScanRecord


class StorageListScansReq(StrictModel):
    session_row_id: int


class StorageListScansRes(StrictModel):
    """metadata 만 자리 — blob 자체 X (GET_BLOB 자리 별도)."""

    scans: list[ScanRecord]


class StorageDeleteScanReq(StrictModel):
    scan_row_id: int


# ── blob (generic — scan / reconstruction 공통)
class StorageGetBlobReq(StrictModel):
    blob_key: str


class StorageGetBlobRes(StrictModel):
    blob_bytes: Base64Bytes


# ── reconstructions
class StoragePutReconstructionReq(StrictModel):
    """ReconstructionNode 가 build 끝나면 호출. metadata + .ply blob."""

    session_row_id: int
    blob_bytes: Base64Bytes  # .ply binary
    voxel_size: float
    sdf_trunc: float
    depth_trunc: float
    icp_max_dist: float
    n_scans: int
    n_edges: int
    vertex_count: int
    triangle_count: int
    elapsed: float


class StoragePutReconstructionRes(StrictModel):
    reconstruction: ReconstructionRecord


class StorageListReconstructionsReq(StrictModel):
    session_row_id: int


class StorageListReconstructionsRes(StrictModel):
    reconstructions: list[ReconstructionRecord]


class StorageDeleteReconstructionReq(StrictModel):
    recon_row_id: int


# ─── Topic: STORAGE_CALIBRATION_INVALIDATED ────────────────────


class CalibrationInvalidated(StrictModel):
    """ACTIVATE 마다 1회 발행. 노드들의 CalibrationCache 가 refetch trigger.

    docs/storage_layer.md §7 — payload 에 (robot_id, kind) 만 — subscriber 가
    자기 robot 만 filter. event stream 정석 (global 1개 topic).
    """

    robot_id: str
    kind: CalibrationKind
    result_id: int
    timestamp: float
