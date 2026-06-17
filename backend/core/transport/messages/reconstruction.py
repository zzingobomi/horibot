"""Reconstruction 노드 service / topic schema.

Multi-view reconstruction pipeline (ICP + PoseGraph + TSDF + mesh extract) 의
heavy compute 자리. PC host-level 1개 (robot-scoped X). ScanTask 의
BuildReconstruction step 자리 caller.

Service:
- RECONSTRUCTION_BUILD — (session_row_id + 파라미터) → ReconstructionRecord
                         (storage put 자체 자리에서 완료, row id 반환)

Topic:
- RECONSTRUCTION_PROGRESS — stage / percent / message 자리. BuildReconstruction
                            step 자리 progress bar 자리 사용.
"""

from __future__ import annotations

from typing import Literal

from core.transport.messages.base import StrictModel
from modules.scan_workflow.persistence_models import ReconstructionRecord


# 5 stage 자리 — tsdf_builder 자리 단계. percent 0.0~1.0 (각 stage 안 진행률).
ReconstructionStage = Literal[
    "loading_scans",
    "pairwise_registration",
    "pose_graph_optimization",
    "tsdf_integration",
    "mesh_extraction",
]


class ReconstructionBuildReq(StrictModel):
    """ScanTask 의 BuildReconstruction step 자리 자리 호출. session 안 모든 scan
    fetch + ICP + PoseGraph + TSDF + mesh + storage put 자리 한 자리."""

    session_row_id: int
    voxel_size: float | None = None  # None 이면 build.py 의 default
    sdf_trunc: float | None = None
    depth_trunc: float | None = None
    icp_max_dist: float | None = None


class ReconstructionBuildRes(StrictModel):
    reconstruction: ReconstructionRecord


class ReconstructionProgress(StrictModel):
    session_row_id: int
    stage: ReconstructionStage
    percent: float  # 0.0 ~ 1.0 (stage 안 진행률)
    message: str = ""
