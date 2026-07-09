"""Calibration Module — 외부 Public Surface (wire contract).

boundary spec = [docs/calibration_module_boundary.md]. 순수 schema 만 — result
data 모델에 numpy / helper method 박지 X (v2 contract = pure data, 옛 backend
`result_models.py` 의 get_trans / as_array / from_calibration_result 는 consumer
(Motion kinematics build / offline BA) 책임으로 이동).

**Bundle 은 boot-time configuration** (Mirror 아님, §6). consumer 가 start() 에서
`snapshot_bundle` 1회 조회. 변경은 다음 부팅부터.
"""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum
from typing import Annotated, Any, Literal

from pydantic import BaseModel, Field, TypeAdapter

from framework.contract.model import StrictModel


# ─── 어휘 (kind / status / artifact) — 실 DB SSOT ──────────────────
# 주의: sag kind 는 `"sag"` (문서 초안의 `sag_offset` 아님 — 실 DB horibot.db 확인).

CalibrationKind = Literal[
    "intrinsic",
    "hand_eye",
    "joint_offset",
    "link_offset",
    "sag",
]

CalibrationRunStatus = Literal[
    "in_progress",
    "ready_for_analysis",
    "success",
    "failed",
]

CalibrationArtifactKind = Literal["primary", "color", "depth", "depth_vis", "ply"]


# ─── result data (kind 별 payload) — 순수 data, 실 result_data shape ─


class IntrinsicResultData(StrictModel):
    camera_matrix: list[list[float]]  # (3, 3)
    dist_coeffs: list[list[float]]  # (1, N)
    image_size: list[int] | None = None
    # cv2.calibrateCamera reprojection RMS (px). factory seed 는 None.
    rms_px: float | None = None


class HandEyeResultData(StrictModel):
    R_cam2gripper: list[list[float]]  # (3, 3)
    t_cam2gripper: list[list[float]]  # (3, 1)
    method: str  # "BA(physical_sag_irls)" / "TSAI" 등


class JointOffsetResultData(StrictModel):
    offsets: dict[int, float]  # motor_id → rad offset
    method: str


class LinkOffsetEntry(StrictModel):
    joint_id: int
    trans_m: list[float]  # (3,) — m
    rot_rad: list[float]  # (3,) — rad rotvec (Rodrigues)


class LinkOffsetResultData(StrictModel):
    offsets: list[LinkOffsetEntry]
    method: str


class SagOffsetResultData(StrictModel):
    k_rad_per_m: dict[int, float]  # joint_id → stiffness
    method: str


# ─── result record (DB row ↔ wire) — discriminated union by kind ───


class _ResultRecordBase(StrictModel):
    id: int | None = None
    run_id: int
    robot_id: str
    created_at: datetime
    is_active: bool = False
    # sigma_*: 최적화 추정 불확실성 (Jacobian). effective_sigma_*: 실측 정확도.
    sigma_rot: float | None = None
    sigma_t: float | None = None
    effective_sigma_rot: float | None = None
    effective_sigma_t: float | None = None


class IntrinsicResultRecord(_ResultRecordBase):
    kind: Literal["intrinsic"] = "intrinsic"
    result_data: IntrinsicResultData


class HandEyeResultRecord(_ResultRecordBase):
    kind: Literal["hand_eye"] = "hand_eye"
    result_data: HandEyeResultData


class JointOffsetResultRecord(_ResultRecordBase):
    kind: Literal["joint_offset"] = "joint_offset"
    result_data: JointOffsetResultData


class LinkOffsetResultRecord(_ResultRecordBase):
    kind: Literal["link_offset"] = "link_offset"
    result_data: LinkOffsetResultData


class SagOffsetResultRecord(_ResultRecordBase):
    kind: Literal["sag"] = "sag"
    result_data: SagOffsetResultData


CalibrationResultRecord = Annotated[
    IntrinsicResultRecord
    | HandEyeResultRecord
    | JointOffsetResultRecord
    | LinkOffsetResultRecord
    | SagOffsetResultRecord,
    Field(discriminator="kind"),
]

CalibrationResultRecordAdapter: TypeAdapter[CalibrationResultRecord] = TypeAdapter(
    CalibrationResultRecord
)


# ─── run / capture / artifact record ───────────────────────────────


class CalibrationRunRecord(StrictModel):
    id: int | None = None
    robot_id: str
    started_at: datetime
    ended_at: datetime | None = None
    operator: str | None = None
    note: str | None = None
    algorithm: str  # "d405_factory" / "hand_eye_capture_only" 등
    algorithm_params: dict[str, Any] = Field(default_factory=dict)
    status: CalibrationRunStatus
    kind: CalibrationKind


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
    run_id: int
    pose_index: int
    # 캡처 시점 raw motor position (motor_id → raw). raw SSOT — joint_offset 갱신돼도
    # 불변, offline BA 가 현재 캘로 재해석.
    motor_positions: dict[int, int] | None = None
    board_in_cam: list[list[float]] | None = None  # PnP board pose (4x4)
    corners_2d: list[list[float]] | None = None  # ChArUco 검출 cache
    corner_ids: list[int] | None = None
    reproj_rms_px: float | None = None
    tilt_deg: float | None = None
    artifacts: list[CalibrationCaptureArtifactRecord] = Field(default_factory=list)

    def find_artifact(
        self, kind: CalibrationArtifactKind
    ) -> CalibrationCaptureArtifactRecord | None:
        return next((a for a in self.artifacts if a.kind == kind), None)


# ─── Bundle — boot-time config snapshot (§6.1) ─────────────────────


class CalibrationBundle(BaseModel):
    """현재 active 5 kind snapshot. consumer(Motion 등)가 boot 때 1회 조회.

    boundary 초안의 `bundle_id` / monotonic `version` 은 **제거** — 실 DB 엔 bundle
    엔티티가 없고 kind 별 result row 가 독립 (각자 id / is_active). "변경됐나?" 판단은
    active result id 집합(`signature`)으로 충분 (frontend "재시작 필요" 비교).
    """

    robot_id: str
    intrinsic: IntrinsicResultRecord | None = None
    hand_eye: HandEyeResultRecord | None = None
    joint_offset: JointOffsetResultRecord | None = None
    link_offset: LinkOffsetResultRecord | None = None
    sag: SagOffsetResultRecord | None = None

    def signature(self) -> tuple[tuple[str, int], ...]:
        """(kind, result_id) 정렬 튜플 — 두 Bundle 이 같은 active 조합인지 비교용."""
        pairs: list[tuple[str, int]] = []
        for kind in ("intrinsic", "hand_eye", "joint_offset", "link_offset", "sag"):
            rec = getattr(self, kind)
            if rec is not None and rec.id is not None:
                pairs.append((kind, rec.id))
        return tuple(sorted(pairs))


# ─── nested contract (Service / Stream / Event) — boundary §4/§5 ───


class Calibration:
    class Service(StrEnum):
        # robot-agnostic (host 당 1, backend_v2.md §2.7) — 대상 robot 은
        # req 필드로 식별: 새 세션(start_run/preview/조회)은 `req.robot_id`,
        # 진행 중 자원은 그 식별자(run_id/result_id)에서 파생 (robot_id 중복 채널 X).
        # commands (write)
        START_RUN = "srv/calibration/start_run"
        CAPTURE = "srv/calibration/capture"
        UNDO_LAST_CAPTURE = "srv/calibration/undo_last_capture"
        FINALIZE_RUN = "srv/calibration/finalize_run"
        ACTIVATE_RESULT = "srv/calibration/activate_result"
        PREVIEW_ENABLE = "srv/calibration/preview_enable"
        # queries (read)
        SNAPSHOT_BUNDLE = "srv/calibration/snapshot_bundle"
        LIST_RUNS = "srv/calibration/list_runs"
        LIST_RESULTS = "srv/calibration/list_results"
        GET_THRESHOLDS = "srv/calibration/get_thresholds"

    class Stream(StrEnum):
        # robot-scoped 키 유지 — payload 의 robot_id 로 framework 가 라우팅
        # (frontend 는 관심 robot 만 구독). host-level publisher 무관.
        PREVIEW = "stream/calibration/{robot_id}/preview"  # 5Hz ChArUco overlay

    class Event(StrEnum):
        # "재시작 필요" 알림 (Mirror trigger 아님 — bundle=config, §5)
        ACTIVATED = "event/calibration/{robot_id}/activated"
        COMMITTED = "event/calibration/{robot_id}/committed"


# ─── request / response (service payload) ──────────────────────────
#
# robot_id 규칙 (host-level dispatch): 서비스가 다른 식별자로 robot 을 특정할 수
# 없을 때만 req 에 robot_id. run_id/result_id 가 있는 요청은 DB row 에서 파생 —
# "run A 에 robot B 캡처" 류 불일치 채널 자체를 없앰.


class SnapshotBundleRequest(BaseModel):
    robot_id: str


class StartRunRequest(BaseModel):
    robot_id: str
    kind: CalibrationKind
    algorithm: str


class StartRunResponse(BaseModel):
    run_id: int


class ActivateResultRequest(BaseModel):
    result_id: int


class ActivateResultResponse(BaseModel):
    ok: bool


class FinalizeRunRequest(BaseModel):
    run_id: int


class FinalizeRunResponse(BaseModel):
    ok: bool
    # intrinsic finalize 는 compute 까지 수행 — 결과 요약 (RMS/coverage) 또는
    # 부족 사유 (캡처 수 미달 등). hand_eye 는 빈 문자열.
    message: str = ""


class UndoLastCaptureRequest(BaseModel):
    run_id: int


class UndoLastCaptureResponse(BaseModel):
    ok: bool


class ListRunsRequest(BaseModel):
    robot_id: str
    kind: CalibrationKind | None = None


class ListRunsResponse(BaseModel):
    runs: list[CalibrationRunRecord]


class ListResultsRequest(BaseModel):
    robot_id: str
    kind: CalibrationKind | None = None


class ListResultsResponse(BaseModel):
    results: list[CalibrationResultRecord]


class CaptureRequest(BaseModel):
    run_id: int
    pose_index: int


class CaptureQualityPayload(BaseModel):
    verdict: str  # "green" | "yellow" | "red"
    reasons: list[str] = Field(default_factory=list)


class CaptureResponse(BaseModel):
    accepted: bool  # gate 통과 여부 (reject 시 capture_id None)
    capture_id: int | None = None
    reproj_rms_px: float | None = None
    tilt_deg: float | None = None
    quality: CaptureQualityPayload | None = None
    message: str = ""


class PreviewEnableRequest(BaseModel):
    robot_id: str
    enabled: bool


class PreviewEnableResponse(BaseModel):
    ok: bool


class GetThresholdsRequest(BaseModel):
    pass


class GetThresholdsResponse(BaseModel):
    thresholds: dict[str, float | int]


class CalibrationPreview(BaseModel):
    """5Hz preview stream — ChArUco 검출 상태 + traffic light (frontend overlay).

    frontend 는 camera/stream(color MJPEG) 위에 corners_2d 를 canvas 오버레이로 그림 —
    MJPEG 과 별 채널이라 좌표를 렌더 이미지 크기로 스케일하려면 원본 크기(image_width/
    height)가 필요. corners_2d 는 intrinsic 유무와 무관하게 채움 (raw ChArUco 픽셀).
    """

    robot_id: str
    seq: int
    timestamp_unix: float
    detected: bool
    corner_count: int
    tilt_deg: float | None = None
    verdict: str  # green/yellow/red
    reasons: list[str] = Field(default_factory=list)
    corners_2d: list[list[float]] = Field(default_factory=list)  # 검출 코너 픽셀 (N,2)
    image_width: int | None = None  # 원본 프레임 크기 (overlay 좌표 스케일용)
    image_height: int | None = None


class CalibrationActivated(BaseModel):
    robot_id: str
    result_id: int
    kind: CalibrationKind


class CalibrationCommitted(BaseModel):
    robot_id: str
    run_id: int
