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
    text,
)
from sqlalchemy.orm import Mapped, mapped_column

from modules.calibration.persistence_models import (
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
    """Evidence — per-pose 자세 정보 (BA 입력 + 출력 residual + IRLS weight)."""

    __tablename__ = "calibration_captures"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    run_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("calibration_runs.id", ondelete="CASCADE"),
        nullable=False,
    )
    pose_index: Mapped[int] = mapped_column(Integer, nullable=False)
    # joint_angles: list[float] — JSON. board_in_cam: 4x4 matrix or None.
    joint_angles: Mapped[str] = mapped_column(Text, nullable=False)
    board_in_cam: Mapped[str | None] = mapped_column(Text, nullable=True)
    residual_rot: Mapped[float | None] = mapped_column(Float, nullable=True)
    residual_trans: Mapped[float | None] = mapped_column(Float, nullable=True)
    weight: Mapped[float | None] = mapped_column(Float, nullable=True)

    __table_args__ = (
        Index("idx_calibration_captures_run", "run_id", "pose_index"),
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
    return CalibrationCaptureOrm(
        run_id=run_id,
        pose_index=capture.pose_index,
        joint_angles=json.dumps(capture.joint_angles),
        board_in_cam=(
            json.dumps(capture.board_in_cam)
            if capture.board_in_cam is not None
            else None
        ),
        residual_rot=capture.residual_rot,
        residual_trans=capture.residual_trans,
        weight=capture.weight,
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


def orm_to_capture(orm: CalibrationCaptureOrm) -> CalibrationCaptureRecord:
    board = json.loads(orm.board_in_cam) if orm.board_in_cam else None
    return CalibrationCaptureRecord(
        id=orm.id,
        run_id=orm.run_id,
        pose_index=orm.pose_index,
        joint_angles=json.loads(orm.joint_angles),
        board_in_cam=board,
        residual_rot=orm.residual_rot,
        residual_trans=orm.residual_trans,
        weight=orm.weight,
    )
