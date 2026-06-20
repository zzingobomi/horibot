"""Calibration entity 의 SQLAlchemy ORM 모델 + Pydantic Record 변환.

`persistence_models.py` 의 Pydantic Record (wire + domain SSOT) 와 짝궁 —
본 파일은 *persistence SSOT* (DB schema 정의). 두 boundary 분리 (DDD mapper).

ORM 의 lazy loading / dirty tracking 의존 X — caller 가 명시적 `select()` +
`session.add()` + `session.commit()`. relationship 자리 없음 (필요 자리 등장
시 `selectinload` 명시).
"""

from __future__ import annotations

import json

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
    CalibrationCaptureArtifactRecord,
    CalibrationCaptureRecord,
    CalibrationResultRecord,
    CalibrationResultRecordAdapter,
    CalibrationRunRecord,
)
from modules.storage.rdb.base import Base


class CalibrationRunOrm(Base):
    """한 번의 캘 실행 (immutable). `kind` 는 run 의 목적 — draft lookup 용."""

    __tablename__ = "calibration_runs"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    started_at: Mapped[float] = mapped_column(Float, nullable=False)
    ended_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    operator: Mapped[str | None] = mapped_column(Text, nullable=True)
    note: Mapped[str | None] = mapped_column(Text, nullable=True)
    algorithm: Mapped[str] = mapped_column(String, nullable=False)
    # algorithm_params: dict[str, Any] — JSON serde 자리. SQLAlchemy 의 JSON
    # type 대신 Text + 명시적 json.dumps/loads — Postgres 진입 시 JSONB 로 옮길
    # 자리는 그때 별도 결정.
    algorithm_params: Mapped[str] = mapped_column(
        Text, nullable=False, default="{}"
    )
    status: Mapped[str] = mapped_column(String, nullable=False, default="success")
    # kind = run 의 목적 (intrinsic / hand_eye 등). draft (in_progress) lookup
    # 용 partial index 동반. legacy row 면 NULL.
    kind: Mapped[str | None] = mapped_column(String, nullable=True)

    __table_args__ = (
        # in_progress run 의 (robot_id, kind) lookup 가속 — get_in_progress_run.
        Index(
            "idx_calibration_runs_in_progress",
            "robot_id",
            "kind",
            sqlite_where=text("status = 'in_progress'"),
            postgresql_where=text("status = 'in_progress'"),
        ),
    )


class CalibrationResultOrm(Base):
    """Run 의 산출물 — kind 별 한 row. (robot_id, kind) 자리 active 토글."""

    __tablename__ = "calibration_results"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    run_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("calibration_runs.id", ondelete="CASCADE"),
        nullable=False,
    )
    robot_id: Mapped[str] = mapped_column(String, nullable=False)
    kind: Mapped[str] = mapped_column(String, nullable=False)
    created_at: Mapped[float] = mapped_column(Float, nullable=False)
    is_active: Mapped[bool] = mapped_column(
        Boolean, nullable=False, default=False
    )
    sigma_rot: Mapped[float | None] = mapped_column(Float, nullable=True)
    sigma_t: Mapped[float | None] = mapped_column(Float, nullable=True)
    # result_data: 5종 Pydantic ResultData (kind discriminator) JSON 직렬화.
    # ORM 은 raw text 들고 있고, orm_to_result 가 TypeAdapter 로 union arm
    # 자동 선택 + validate.
    result_data: Mapped[str] = mapped_column(Text, nullable=False)

    __table_args__ = (
        # per-kind active row 1개만 — UNIQUE partial index. ACTIVATE transaction
        # 의 "deactivate 후 activate" 가 한 transaction 안에서 일관 보장.
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
    """Evidence — per-pose raw sensor 데이터 + BA 입력 캐시 + 출력 residual.

    `motor_positions` = raw int SSOT (drift-free).
    Blob (primary .bin + 디버깅 artifact 들) 는 별도 정규화 테이블
    `calibration_capture_artifacts` 에서 관리 — kind ('primary'/'color'/'depth'/
    'depth_vis'/'ply') 별 ObjectStore key.
    """

    __tablename__ = "calibration_captures"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    run_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("calibration_runs.id", ondelete="CASCADE"),
        nullable=False,
    )
    pose_index: Mapped[int] = mapped_column(Integer, nullable=False)
    # JSON dict[int, int] — motor_id → raw position. nullable (intrinsic 캡처).
    motor_positions: Mapped[str | None] = mapped_column(Text, nullable=True)
    # 4x4 matrix (PnP board_in_cam 캐시) — JSON list[list[float]].
    board_in_cam: Mapped[str | None] = mapped_column(Text, nullable=True)
    # ChArUco 검출 결과 캐시. corners_2d: (N,2) sub-pixel, corner_ids: (N,) int. JSON.
    corners_2d: Mapped[str | None] = mapped_column(Text, nullable=True)
    corner_ids: Mapped[str | None] = mapped_column(Text, nullable=True)
    # 진단 metric.
    reproj_rms_px: Mapped[float | None] = mapped_column(Float, nullable=True)
    tilt_deg: Mapped[float | None] = mapped_column(Float, nullable=True)
    # BA output — offline 분석 결과 backfill 자리.
    residual_rot: Mapped[float | None] = mapped_column(Float, nullable=True)
    residual_trans: Mapped[float | None] = mapped_column(Float, nullable=True)
    weight: Mapped[float | None] = mapped_column(Float, nullable=True)

    __table_args__ = (
        Index("idx_calibration_captures_run", "run_id", "pose_index"),
    )


class CalibrationCaptureArtifactOrm(Base):
    """Capture 1장의 ObjectStore blob 1개 — primary .bin 또는 디버깅 artifact.

    한 capture 는 0..N artifacts (보통 5개: primary + color + depth + depth_vis + ply).
    `kind` 는 도메인 vocabulary — DB schema 는 string 자유 허용 (Pydantic 자리 validate).
    UNIQUE(capture_id, kind) — 같은 capture 의 같은 kind 자리 2개 금지.
    """

    __tablename__ = "calibration_capture_artifacts"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    capture_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("calibration_captures.id", ondelete="CASCADE"),
        nullable=False,
    )
    kind: Mapped[str] = mapped_column(String, nullable=False)
    blob_key: Mapped[str] = mapped_column(String, nullable=False)
    size_bytes: Mapped[int | None] = mapped_column(Integer, nullable=True)
    content_type: Mapped[str | None] = mapped_column(String, nullable=True)
    created_at: Mapped[float] = mapped_column(Float, nullable=False)

    __table_args__ = (
        Index(
            "idx_calibration_capture_artifacts_capture",
            "capture_id",
        ),
        # 같은 capture 의 같은 kind 2개 금지 — upsert 일관성.
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
        result_data=record.result_data.model_dump_json(),
    )


def capture_record_to_orm(
    capture: CalibrationCaptureRecord,
    *,
    run_id: int,
) -> CalibrationCaptureOrm:
    # dict[int, int] 의 key 는 SQLite Text JSON 직렬화 시 str 로 변환됨 — 역으로
    # 읽을 때 int(key) 캐스팅 필요 (orm_to_capture 자리).
    # 주의: artifacts 는 별도 테이블이라 본 mapper 가 다루지 X. caller (repo) 가
    # capture INSERT 후 artifact_record_to_orm 으로 별도 INSERT.
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
            json.dumps(capture.corners_2d)
            if capture.corners_2d is not None
            else None
        ),
        corner_ids=(
            json.dumps(capture.corner_ids)
            if capture.corner_ids is not None
            else None
        ),
        reproj_rms_px=capture.reproj_rms_px,
        tilt_deg=capture.tilt_deg,
        residual_rot=capture.residual_rot,
        residual_trans=capture.residual_trans,
        weight=capture.weight,
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
        kind=orm.kind,  # type: ignore[arg-type]
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
        status=orm.status,  # type: ignore[arg-type]
        kind=orm.kind,  # type: ignore[arg-type]
    )


def orm_to_result(orm: CalibrationResultOrm) -> CalibrationResultRecord:
    # TypeAdapter — `kind` 보고 union arm 자동 선택 + result_data 를 알맞은
    # ResultData 모델로 validate. drift 즉시 ValidationError.
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
            "result_data": json.loads(orm.result_data),
        }
    )


def orm_to_capture(
    orm: CalibrationCaptureOrm,
    *,
    artifacts: list[CalibrationCaptureArtifactRecord] | None = None,
) -> CalibrationCaptureRecord:
    # motor_positions: JSON dict 의 key 가 str 로 직렬화되어 있으니 int 로 캐스팅.
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
        residual_rot=orm.residual_rot,
        residual_trans=orm.residual_trans,
        weight=orm.weight,
        artifacts=artifacts or [],
    )
