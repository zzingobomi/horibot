"""Calibration 의 persistence 모델 — DB / object store 에 저장되는 row 의 Pydantic shape.

본 파일은 calibration 도메인의 일부 — "캘이 무엇을 저장하는지" 의 소유권은 캘에
있음. storage 모듈은 "어떻게 저장하는지" (SQL connection / blob put-get) 만 다룸.
캘 schema 변경 (컬럼 추가 / kind 추가 등) 은 본 파일만 건드림 — storage/ 안 건드림.

`result_models.py` 와의 구분:
- `result_models.py`     — 계산 결과 shape (HandEyeResultData ...)
- `persistence_models.py` — DB row shape (CalibrationResultRecord ...)

3 테이블 (docs/storage_layer.md §6):
- calibration_runs       — 한 번의 캘 실행 (immutable, run_id)
- calibration_results    — Run 의 산출물 (kind 별, is_active 토글). Discriminated
                           union — kind 가 result_data 의 shape 을 결정 (Pydantic
                           이 자동 validate). drift 차단.
- calibration_captures   — Evidence (per-pose 자세 + residual + IRLS weight)
"""

from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import Field, TypeAdapter

from core.transport.messages.base import StrictModel
from modules.calibration.result_models import (
    HandEyeResultData,
    IntrinsicResultData,
    JointOffsetResultData,
    LinkOffsetResultData,
    SagOffsetResultData,
)


# 캘 5종. SSOT — RDB row 의 kind 컬럼 값과 일치.
CalibrationKind = Literal[
    "intrinsic",
    "hand_eye",
    "joint_offset",
    "link_offset",
    "sag",
]


# Run status — draft / 종료 상태 분리. 사용자가 commit 누르기 전 단계 자체는 in_progress.
# 캡처 row 들은 in_progress run 에 append (FK 가 commit 시점에야 생기는 문제 해결).
CalibrationRunStatus = Literal["in_progress", "success", "failed"]


# ─── Run ──────────────────────────────────────────────────────


class CalibrationRunRecord(StrictModel):
    """한 번의 캘 실행 이력 (immutable).

    한 Run 이 여러 Result 만들 수 있음 (예: 확장 BA → hand_eye + joint + link +
    sag 동시 산출). algorithm 은 'extended_ba_irls' 등 식별자.

    `kind` 는 run 의 *목적* (사용자가 누른 버튼). 예: hand_eye run 은 4 kind result
    만들지만 kind='hand_eye'. in_progress run 의 robot/kind lookup 용. None=legacy.
    """

    # id 는 storage 가 부여 (INSERT 후 채워짐). insert 호출 시 None.
    id: int | None = None
    robot_id: str
    started_at: float  # epoch seconds
    ended_at: float | None = None
    operator: str | None = None
    note: str | None = None
    algorithm: str
    algorithm_params: dict[str, Any] = {}
    status: CalibrationRunStatus = "success"
    kind: CalibrationKind | None = None  # run 의 목적 (intrinsic / hand_eye). draft lookup 용.


# ─── Result — discriminated union ─────────────────────────────


class _ResultRecordBase(StrictModel):
    """공통 컬럼 — kind / result_data 만 sub-class 가 차이."""

    id: int | None = None
    run_id: int  # FK → calibration_runs.id
    robot_id: str
    created_at: float  # epoch seconds
    is_active: bool = False
    sigma_rot: float | None = None  # joint_offset 등 σ 없는 kind 는 None
    sigma_t: float | None = None


class HandEyeResultRecord(_ResultRecordBase):
    kind: Literal["hand_eye"] = "hand_eye"
    result_data: HandEyeResultData


class IntrinsicResultRecord(_ResultRecordBase):
    kind: Literal["intrinsic"] = "intrinsic"
    result_data: IntrinsicResultData


class JointOffsetResultRecord(_ResultRecordBase):
    kind: Literal["joint_offset"] = "joint_offset"
    result_data: JointOffsetResultData


class LinkOffsetResultRecord(_ResultRecordBase):
    kind: Literal["link_offset"] = "link_offset"
    result_data: LinkOffsetResultData


class SagOffsetResultRecord(_ResultRecordBase):
    kind: Literal["sag"] = "sag"
    result_data: SagOffsetResultData


# Pydantic 의 `Field(discriminator="kind")` — `kind` 컬럼 값 보고 자동으로 알맞은
# arm 골라 validate. 잘못된 (kind, result_data) 조합은 ValidationError. drift 차단.
CalibrationResultRecord = Annotated[
    HandEyeResultRecord
    | IntrinsicResultRecord
    | JointOffsetResultRecord
    | LinkOffsetResultRecord
    | SagOffsetResultRecord,
    Field(discriminator="kind"),
]

# TypeAdapter — 외부 dict (SQL row → JSON parse 결과 등) 를 union 으로 validate.
# CalibrationRepo 의 row → record 변환에서 사용.
CalibrationResultRecordAdapter: TypeAdapter[CalibrationResultRecord] = TypeAdapter(
    CalibrationResultRecord
)


# ─── Capture (Evidence) ───────────────────────────────────────


class CalibrationCaptureRecord(StrictModel):
    """Evidence — per-pose 자세 정보 (BA 입력 + 출력 residual + IRLS weight)."""

    id: int | None = None
    run_id: int  # FK → calibration_runs.id
    pose_index: int
    joint_angles: list[float]
    # ChArUco 검출 결과 4x4 matrix (board_in_cam). intrinsic 캘에는 None.
    board_in_cam: list[list[float]] | None = None
    residual_rot: float | None = None
    residual_trans: float | None = None
    weight: float | None = None  # IRLS Huber weight (1.0 = 정상)
