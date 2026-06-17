"""Scan workflow persistence records — Storage Phase 2 entity.

세 entity 자리 (scan_sessions / scans / reconstructions). Storage Phase 1 (캘)
와 같은 패턴 — `modules/calibration/persistence_models.py` 의 record shape 와
정합. 다만 캘 특유 패턴 (is_active / ACTIVATE / invalidation) 은 안 빌림 —
storage_layer.md §3 의 append-only blob + immutable metadata row 자리.

ObjectStore key 컨벤션:
- scan blob:           scans/<robot_id>/<session_id>/<scan_id>.npz
- reconstruction blob: reconstructions/<robot_id>/<session_id>/recon_<row_id>.ply

scan_id 는 (session_row_id, scan_id) unique — session 안 monotonic. legacy
scan_io.allocate_scan_id 자리 호환 (RDB 자리에서 같은 monotonic).
"""

from __future__ import annotations

from core.transport.messages.base import StrictModel


class ScanSessionRecord(StrictModel):
    """Scan session — 한 번의 multi-pose scan 묶음. label / note 자리 수정 가능."""

    id: int | None = None  # auto-increment (RDB 가 발급)
    robot_id: str
    session_id: str  # human-readable (e.g., "session_20260617_120000")
    created_at: float
    label: str | None = None
    note: str | None = None


class ScanRecord(StrictModel):
    """Scan — 한 자세에서 캡처한 RGBD frame.

    blob (raw depth_z16 + color_bgr + ...) 는 ObjectStore 자리. row 는 metadata
    (intrinsic / motor positions) 자리만.
    """

    id: int | None = None  # auto-increment (RDB row id)
    session_row_id: int  # FK → scan_sessions.id
    robot_id: str
    scan_id: int  # monotonic within session_row_id
    created_at: float
    blob_key: str  # ObjectStore key
    num_frames: int
    # snapshot metadata — reconstruction 자리 fresh 재계산 위해 (캘 변경 시)
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float
    motor_positions: list[int]
    arm_motor_ids: list[int]


class ReconstructionRecord(StrictModel):
    """Reconstruction — multi-scan ICP+PoseGraph+TSDF mesh 결과.

    blob (.ply) 는 ObjectStore. row 는 metadata + ICP/TSDF 통계 자리.
    """

    id: int | None = None
    session_row_id: int  # FK → scan_sessions.id
    robot_id: str
    created_at: float
    blob_key: str  # ObjectStore key (.ply)
    # ICP / TSDF 파라미터 — 재현용
    voxel_size: float
    sdf_trunc: float
    depth_trunc: float
    icp_max_dist: float
    # 결과 통계
    n_scans: int
    n_edges: int
    vertex_count: int
    triangle_count: int
    elapsed: float
