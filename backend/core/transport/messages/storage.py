"""Storage 노드 service / topic payload schema.

docs/storage_layer.md §2 — Zenoh service gateway. 4 service + 1 topic.

Naming: verb-first + sub-domain prefix (docs/naming_conventions.md §1).
- calibration sub-domain — `*Calibration*` / `*CalibrationRun*` / `*CalibrationCapture*`
- scan workflow sub-domain — `*Scan*` / `*ScanSession*` / `*Reconstruction*` / `*Blob*`

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


class GetActiveCalibrationReq(StrictModel):
    robot_id: str
    kind: CalibrationKind


class GetActiveCalibrationRes(StrictModel):
    """found=False 면 활성 result 없음 — 첫 부팅 robot. caller 가 default fallback."""

    found: bool
    result: CalibrationResultRecord | None = None


# ─── Service: STORAGE_LIST_CALIBRATIONS ────────────────────────


class ListCalibrationsReq(StrictModel):
    robot_id: str
    kind: CalibrationKind
    limit: int = 100


class ListCalibrationsRes(StrictModel):
    results: list[CalibrationResultRecord]


# ─── Service: STORAGE_LIST_CALIBRATION_RUNS ────────────────────


class CalibrationRunSummary(StrictModel):
    """Run + 그 Run 의 모든 kind Result. frontend list/ACTIVATE 패널이 한 Run
    한 row 로 펼침 — 5 kind 같이 보이고 ACTIVATE 도 Run 전체 / kind 별 양쪽 가능.

    storage_layer.md §13.7 Stage 4 design A.
    """

    run: CalibrationRunRecord
    results: list[CalibrationResultRecord]


class ListCalibrationRunsReq(StrictModel):
    robot_id: str
    limit: int = 50


class ListCalibrationRunsRes(StrictModel):
    """`run.started_at DESC` 정렬. 각 Run 마다 그 Run 의 모든 Result 가 묶여 옴."""

    runs: list[CalibrationRunSummary]


# ─── Service: STORAGE_COMMIT_CALIBRATION ───────────────────────


class CommitCalibrationReq(StrictModel):
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


class CommitCalibrationRes(StrictModel):
    run_id: int
    result_ids: list[int]


# ─── Service: STORAGE_ACTIVATE_CALIBRATION ─────────────────────


class ActivateCalibrationReq(StrictModel):
    result_id: int


class ActivateCalibrationRes(StrictModel):
    """activated result 의 robot_id / kind 는 frontend 가 invalidation 확인 시 사용."""

    result: CalibrationResultRecord


# ─── Draft run / capture-as-you-go (사용자 [캘 시작] flow) ─────


class CreateCalibrationRunReq(StrictModel):
    """[캘 시작] — in_progress run 생성. caller 가 run.kind 채워야 함.
    같은 (robot_id, kind) 의 기존 in_progress 가 있으면 서버가 reject."""

    run: CalibrationRunRecord


class CreateCalibrationRunRes(StrictModel):
    run_id: int


class AppendCalibrationCaptureReq(StrictModel):
    """[캡처] — draft run 에 capture 1장 append. caller 가 capture.run_id 채워야 함."""

    capture: CalibrationCaptureRecord


class AppendCalibrationCaptureRes(StrictModel):
    capture_id: int


class DeleteLastCalibrationCaptureReq(StrictModel):
    """[되돌리기] — 마지막 capture 1장 삭제."""

    run_id: int


class DeleteLastCalibrationCaptureRes(StrictModel):
    """deleted_pose_index None = 삭제할 capture 없음."""

    deleted_pose_index: int | None = None


class GetInProgressCalibrationRunReq(StrictModel):
    """부팅 시 복원 — 사용자 진행 중이던 세션 자리."""

    robot_id: str
    kind: CalibrationKind


class GetInProgressCalibrationRunRes(StrictModel):
    """found=False 면 진행 중 세션 없음."""

    found: bool
    run: CalibrationRunRecord | None = None
    captures: list[CalibrationCaptureRecord] = []


class ListRunCapturesReq(StrictModel):
    """임의 run_id 의 captures fetch — 직전 캘 자세 import (move-to-pose 흐름)."""

    run_id: int


class ListRunCapturesRes(StrictModel):
    captures: list[CalibrationCaptureRecord]


class DeleteCalibrationRunReq(StrictModel):
    """[리셋] — run + captures + results cascade delete."""

    run_id: int


class FinalizeCalibrationRunReq(StrictModel):
    """[커밋] — in_progress → success, result rows INSERT, captures 의 residual UPDATE.

    capture_residuals: pose_index → (residual_rot, residual_trans, weight) 매핑.
    BA 출력 자리 자체 (hand_eye 자리 BA 결과). None 이면 update skip (intrinsic 등 residual 없는 자리).
    """

    run_id: int
    results: list[CalibrationResultRecord]
    capture_residuals: dict[int, tuple[float | None, float | None, float | None]] | None = (
        None
    )


class FinalizeCalibrationRunRes(StrictModel):
    result_ids: list[int]


# ─── Phase 2 — scan workflow ───────────────────────────────────
# blob_key 자리는 server 결정 (race 차단). blob bytes wire = opaque — caller 가
# scan_workflow.blob 의 encode/decode 자리 사용. GET_BLOB 자리 generic (scan /
# reconstruction 공통).


# ── scan_sessions
class CreateScanSessionReq(StrictModel):
    robot_id: str
    session_id: str = ""  # 빈 자리 server 가 시간 기반 default
    label: str | None = None
    note: str | None = None


class CreateScanSessionRes(StrictModel):
    session: ScanSessionRecord


class ListScanSessionsReq(StrictModel):
    robot_id: str
    limit: int = 100


class ListScanSessionsRes(StrictModel):
    sessions: list[ScanSessionRecord]


class DeleteScanSessionReq(StrictModel):
    """CASCADE — 자식 scans / reconstructions 자리 자동 삭제 (RDB + ObjectStore blob).

    blob 자리도 server 가 같이 삭제 자리 — RDB row 자리 fetch 후 blob_key 순회.
    """

    session_row_id: int


# ── scans
class PutScanReq(StrictModel):
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


class PutScanRes(StrictModel):
    scan: ScanRecord


class ListScansReq(StrictModel):
    session_row_id: int


class ListScansRes(StrictModel):
    """metadata 만 자리 — blob 자체 X (GET_BLOB 자리 별도)."""

    scans: list[ScanRecord]


class DeleteScanReq(StrictModel):
    scan_row_id: int


# ── blob (generic — scan / reconstruction 공통)
class GetBlobReq(StrictModel):
    blob_key: str


class GetBlobRes(StrictModel):
    blob_bytes: Base64Bytes


# ── reconstructions
class PutReconstructionReq(StrictModel):
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


class PutReconstructionRes(StrictModel):
    reconstruction: ReconstructionRecord


class ListReconstructionsReq(StrictModel):
    session_row_id: int


class ListReconstructionsRes(StrictModel):
    reconstructions: list[ReconstructionRecord]


class DeleteReconstructionReq(StrictModel):
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
