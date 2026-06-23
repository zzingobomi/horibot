from __future__ import annotations

import json
from datetime import datetime

from pydantic import TypeAdapter
from sqlalchemy import (
    Boolean,
    Float,
    ForeignKey,
    Index,
    Integer,
    String,
    Text,
    UniqueConstraint,
    text,
)
from sqlalchemy.orm import Mapped, mapped_column

from modules.calibration.persistence_models import (
    CalibrationArtifactKind,
    CalibrationCaptureArtifactRecord,
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationResultRecordAdapter,
    CalibrationRunRecord,
    CalibrationRunStatus,
)
from modules.storage.rdb.base import Base, UtcDateTime

# DB String ↔ Domain Literal 경계에서 잘못된 값 유입을 검증한다.
# Literal 제약이 있는 field만 adapter 사용.
_RUN_STATUS_ADAPTER: TypeAdapter[CalibrationRunStatus] = TypeAdapter(
    CalibrationRunStatus
)
_RUN_KIND_ADAPTER: TypeAdapter[CalibrationKind] = TypeAdapter(CalibrationKind)
_ARTIFACT_KIND_ADAPTER: TypeAdapter[CalibrationArtifactKind] = TypeAdapter(
    CalibrationArtifactKind
)


class CalibrationRunOrm(Base):
    __tablename__ = "calibration_runs"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    started_at: Mapped[datetime] = mapped_column(UtcDateTime, nullable=False)
    ended_at: Mapped[datetime | None] = mapped_column(UtcDateTime, nullable=True)
    operator: Mapped[str | None] = mapped_column(Text, nullable=True)
    note: Mapped[str | None] = mapped_column(Text, nullable=True)
    # Calibration 결과를 만든 알고리즘 이름 (예: intrinsic_chessboard, hand_eye_capture_only).
    algorithm: Mapped[str] = mapped_column(String, nullable=False)
    # 해당 Calibration 실행 당시 사용한 입력값 snapshot (예: intrinsic 정보, board 설정).
    algorithm_params: Mapped[str] = mapped_column(Text, nullable=False, default="{}")

    status: Mapped[str] = mapped_column(String, nullable=False, default="success")
    kind: Mapped[str] = mapped_column(String, nullable=False)

    __table_args__ = (
        Index(
            "idx_calibration_runs_in_progress",
            "robot_id",
            "kind",
            sqlite_where=text("status = 'in_progress'"),
            postgresql_where=text("status = 'in_progress'"),
        ),
    )


class CalibrationResultOrm(Base):
    __tablename__ = "calibration_results"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    run_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("calibration_runs.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    kind: Mapped[str] = mapped_column(String, nullable=False)
    created_at: Mapped[datetime] = mapped_column(UtcDateTime, nullable=False)
    is_active: Mapped[bool] = mapped_column(Boolean, nullable=False, default=False)

    # Calibration 오차/신뢰도 지표.
    # sigma_*: 최적화 결과의 추정 불확실성.
    # effective_sigma_*: 실제 측정 오차 기반 정확도.
    sigma_rot: Mapped[float | None] = mapped_column(Float, nullable=True)
    sigma_t: Mapped[float | None] = mapped_column(Float, nullable=True)
    effective_sigma_rot: Mapped[float | None] = mapped_column(Float, nullable=True)
    effective_sigma_t: Mapped[float | None] = mapped_column(Float, nullable=True)

    # Calibration 결과 데이터(JSON).
    # kind 값에 따라 해당 ResultData 모델로 변환/검증.
    result_data: Mapped[str] = mapped_column(Text, nullable=False)

    __table_args__ = (
        # 같은 robot/kind 는 활성 결과를 하나만 유지.
        Index(
            "idx_calibration_results_active",
            "robot_id",
            "kind",
            unique=True,
            sqlite_where=text("is_active = 1"),
            postgresql_where=text("is_active = TRUE"),
        ),
        Index(
            "idx_calibration_results_lookup",
            "robot_id",
            "kind",
            "created_at",
        ),
    )


class CalibrationCaptureOrm(Base):
    __tablename__ = "calibration_captures"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    run_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("calibration_runs.id", ondelete="CASCADE"),
        nullable=False,
    )
    pose_index: Mapped[int] = mapped_column(Integer, nullable=False)

    # 캘리브레이션 당시 raw motor position (motor_id → position).
    motor_positions: Mapped[str | None] = mapped_column(Text, nullable=True)

    # PnP 결과 board pose (4x4 transform matrix).
    board_in_cam: Mapped[str | None] = mapped_column(Text, nullable=True)

    # ChArUco 검출 결과 cache.
    corners_2d: Mapped[str | None] = mapped_column(Text, nullable=True)
    corner_ids: Mapped[str | None] = mapped_column(Text, nullable=True)

    # Capture 품질/진단 metric.
    reproj_rms_px: Mapped[float | None] = mapped_column(Float, nullable=True)
    tilt_deg: Mapped[float | None] = mapped_column(Float, nullable=True)

    __table_args__ = (Index("idx_calibration_captures_run", "run_id", "pose_index"),)


class CalibrationCaptureArtifactOrm(Base):
    __tablename__ = "calibration_capture_artifacts"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    capture_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("calibration_captures.id", ondelete="CASCADE"),
        nullable=False,
    )

    # Artifact 종류 (예: primary, color, depth, ply).
    kind: Mapped[str] = mapped_column(String, nullable=False)

    # ObjectStore에 저장된 blob 식별자.
    blob_key: Mapped[str] = mapped_column(String, nullable=False)

    size_bytes: Mapped[int | None] = mapped_column(Integer, nullable=True)
    content_type: Mapped[str | None] = mapped_column(String, nullable=True)
    created_at: Mapped[datetime] = mapped_column(UtcDateTime, nullable=False)

    __table_args__ = (
        Index(
            "idx_calibration_capture_artifacts_capture",
            "capture_id",
        ),
        # 같은 capture의 같은 종류 artifact는 하나만 허용.
        UniqueConstraint("capture_id", "kind"),
    )


# ─── Pydantic Record ↔ ORM 양방향 boundary mapper ────────────────


def run_record_to_orm(
    run: CalibrationRunRecord,
    *,
    force_status: str | None = None,
) -> CalibrationRunOrm:
    return CalibrationRunOrm(
        robot_id=run.robot_id,
        started_at=run.started_at,
        ended_at=run.ended_at,
        operator=run.operator,
        note=run.note,
        algorithm=run.algorithm,
        algorithm_params=json.dumps(run.algorithm_params),
        status=force_status if force_status is not None else run.status,
        kind=run.kind,
    )


def result_record_to_orm(
    record: CalibrationResultRecord,
    *,
    run_id: int,
    is_active: bool,
) -> CalibrationResultOrm:
    return CalibrationResultOrm(
        run_id=run_id,
        robot_id=record.robot_id,
        kind=record.kind,
        created_at=record.created_at,
        is_active=is_active,
        sigma_rot=record.sigma_rot,
        sigma_t=record.sigma_t,
        effective_sigma_rot=record.effective_sigma_rot,
        effective_sigma_t=record.effective_sigma_t,
        result_data=record.result_data.model_dump_json(),
    )


def capture_record_to_orm(
    capture: CalibrationCaptureRecord,
    *,
    run_id: int,
) -> CalibrationCaptureOrm:
    # DB 저장용 ORM 변환:
    # - dict/list 데이터는 JSON 문자열로 저장
    # - JSON 변환 시 dict key가 문자열화되므로 역변환 시 주의
    # - artifact 저장은 별도 mapper/repository 책임
    return CalibrationCaptureOrm(
        run_id=run_id,
        pose_index=capture.pose_index,
        motor_positions=(
            json.dumps(capture.motor_positions)
            if capture.motor_positions is not None
            else None
        ),
        board_in_cam=(
            json.dumps(capture.board_in_cam)
            if capture.board_in_cam is not None
            else None
        ),
        corners_2d=(
            json.dumps(capture.corners_2d) if capture.corners_2d is not None else None
        ),
        corner_ids=(
            json.dumps(capture.corner_ids) if capture.corner_ids is not None else None
        ),
        reproj_rms_px=capture.reproj_rms_px,
        tilt_deg=capture.tilt_deg,
    )


def artifact_record_to_orm(
    rec: CalibrationCaptureArtifactRecord, *, capture_id: int
) -> CalibrationCaptureArtifactOrm:
    return CalibrationCaptureArtifactOrm(
        capture_id=capture_id,
        kind=rec.kind,
        blob_key=rec.blob_key,
        size_bytes=rec.size_bytes,
        content_type=rec.content_type,
        created_at=rec.created_at,
    )


def orm_to_artifact(
    orm: CalibrationCaptureArtifactOrm,
) -> CalibrationCaptureArtifactRecord:
    return CalibrationCaptureArtifactRecord(
        id=orm.id,
        capture_id=orm.capture_id,
        kind=_ARTIFACT_KIND_ADAPTER.validate_python(orm.kind),
        blob_key=orm.blob_key,
        size_bytes=orm.size_bytes,
        content_type=orm.content_type,
        created_at=orm.created_at,
    )


def orm_to_run(orm: CalibrationRunOrm) -> CalibrationRunRecord:
    return CalibrationRunRecord(
        id=orm.id,
        robot_id=orm.robot_id,
        started_at=orm.started_at,
        ended_at=orm.ended_at,
        operator=orm.operator,
        note=orm.note,
        algorithm=orm.algorithm,
        algorithm_params=json.loads(orm.algorithm_params or "{}"),
        status=_RUN_STATUS_ADAPTER.validate_python(orm.status),
        kind=_RUN_KIND_ADAPTER.validate_python(orm.kind),
    )


# result는 kind 값에 따라 다른 결과 모델로 변환되는 union 타입이라
# TypeAdapter가 알맞은 모델 선택과 데이터 검증을 처리한다.
def orm_to_result(orm: CalibrationResultOrm) -> CalibrationResultRecord:
    return CalibrationResultRecordAdapter.validate_python(
        {
            "id": orm.id,
            "run_id": orm.run_id,
            "robot_id": orm.robot_id,
            "kind": orm.kind,
            "created_at": orm.created_at,
            "is_active": bool(orm.is_active),
            "sigma_rot": orm.sigma_rot,
            "sigma_t": orm.sigma_t,
            "effective_sigma_rot": orm.effective_sigma_rot,
            "effective_sigma_t": orm.effective_sigma_t,
            "result_data": json.loads(orm.result_data),
        }
    )


def orm_to_capture(
    orm: CalibrationCaptureOrm,
    *,
    artifacts: list[CalibrationCaptureArtifactRecord] | None = None,
) -> CalibrationCaptureRecord:
    motor_positions_raw = (
        json.loads(orm.motor_positions) if orm.motor_positions else None
    )
    motor_positions = (
        {int(k): int(v) for k, v in motor_positions_raw.items()}
        if motor_positions_raw is not None
        else None
    )
    return CalibrationCaptureRecord(
        id=orm.id,
        run_id=orm.run_id,
        pose_index=orm.pose_index,
        motor_positions=motor_positions,
        board_in_cam=json.loads(orm.board_in_cam) if orm.board_in_cam else None,
        corners_2d=json.loads(orm.corners_2d) if orm.corners_2d else None,
        corner_ids=json.loads(orm.corner_ids) if orm.corner_ids else None,
        reproj_rms_px=orm.reproj_rms_px,
        tilt_deg=orm.tilt_deg,
        artifacts=artifacts or [],
    )
