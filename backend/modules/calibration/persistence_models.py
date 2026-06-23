from __future__ import annotations

from datetime import datetime
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


CalibrationKind = Literal[
    "intrinsic",
    "hand_eye",
    "joint_offset",
    "link_offset",
    "sag",
]


CalibrationRunStatus = Literal["in_progress", "ready_for_analysis", "success", "failed"]


class CalibrationRunRecord(StrictModel):
    id: int | None = None
    robot_id: str
    started_at: datetime
    ended_at: datetime | None = None
    operator: str | None = None
    note: str | None = None
    algorithm: str
    algorithm_params: dict[str, Any] = {}
    status: CalibrationRunStatus
    kind: CalibrationKind


class _ResultRecordBase(StrictModel):
    id: int | None = None
    run_id: int
    robot_id: str
    created_at: datetime
    is_active: bool = False

    # Calibration 오차/신뢰도 지표.
    # sigma_*: 최적화 결과의 추정 불확실성.
    # effective_sigma_*: 실제 측정 오차 기반 정확도.
    sigma_rot: float | None = None
    sigma_t: float | None = None
    effective_sigma_rot: float | None = None
    effective_sigma_t: float | None = None


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


# discriminator(kind): kind 값을 보고 사용할 결과 모델을 선택
CalibrationResultRecord = Annotated[
    HandEyeResultRecord
    | IntrinsicResultRecord
    | JointOffsetResultRecord
    | LinkOffsetResultRecord
    | SagOffsetResultRecord,
    Field(discriminator="kind"),
]

# DB에서 읽은 dict 데이터를 kind 기준으로 알맞은 ResultRecord 모델로 변환/검증
CalibrationResultRecordAdapter: TypeAdapter[CalibrationResultRecord] = TypeAdapter(
    CalibrationResultRecord
)


CalibrationArtifactKind = Literal["primary", "color", "depth", "depth_vis", "ply"]


class CalibrationCaptureArtifactRecord(StrictModel):
    id: int | None = None
    capture_id: int
    kind: CalibrationArtifactKind
    blob_key: str
    size_bytes: int | None = None
    content_type: str | None = None
    created_at: datetime


class CalibrationCaptureRecord(StrictModel):
    id: int | None = None
    run_id: int  # FK → calibration_runs.id
    pose_index: int

    # 캘리브레이션 당시 raw motor position (motor_id → position).
    motor_positions: dict[int, int] | None = None

    # PnP 결과 board pose (4x4 transform matrix).
    board_in_cam: list[list[float]] | None = None

    # ChArUco 검출 결과 cache.
    corners_2d: list[list[float]] | None = None
    corner_ids: list[int] | None = None

    # Capture 품질/진단 metric.
    reproj_rms_px: float | None = None
    tilt_deg: float | None = None

    artifacts: list[CalibrationCaptureArtifactRecord] = []

    def find_artifact(
        self, kind: CalibrationArtifactKind
    ) -> CalibrationCaptureArtifactRecord | None:
        for a in self.artifacts:
            if a.kind == kind:
                return a
        return None

    @property
    def primary_blob_key(self) -> str | None:
        a = self.find_artifact("primary")
        return a.blob_key if a is not None else None
