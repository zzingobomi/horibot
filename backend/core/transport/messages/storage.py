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

from core.transport.messages.base import StrictModel
from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationRunRecord,
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
