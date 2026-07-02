"""CalibrationRepository — calibration 모듈이 자기 영속성 소유 (Database-per-Module).

CRUD ceremony (add/commit/refresh, get-or-none, delete) 는 advanced-alchemy
`SQLAlchemySyncRepository` 를 **모듈 내부 헬퍼로만** 사용 (framework 는 이걸 전제 X —
[backend_v2.md §10.4]: framework = plain Repository Protocol). 라이브러리 의존은 이
파일 안에 갇힘 — 외부엔 `CalibrationRepository` 만 보이고, 나중에 빼면 내부만 교체.

도메인 로직 (activate_result atomic 2-step / get_active_bundle 5-kind aggregate /
undo cascade) 은 라이브러리가 안 주는 자리라 직접 구현.
boundary spec = [docs/calibration_module_boundary.md §2].
"""

from __future__ import annotations

from collections.abc import Callable
from datetime import UTC, datetime

from advanced_alchemy.repository import SQLAlchemySyncRepository
from sqlalchemy import select
from sqlalchemy.orm import Session

from ..contract import (
    CalibrationBundle,
    CalibrationCaptureArtifactRecord,
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationRunRecord,
    CalibrationRunStatus,
)
from .orm import (
    CalibrationCaptureArtifactOrm,
    CalibrationCaptureOrm,
    CalibrationResultOrm,
    CalibrationRunOrm,
    artifact_record_to_orm,
    capture_record_to_orm,
    orm_to_artifact,
    orm_to_capture,
    orm_to_result,
    orm_to_run,
    result_record_to_orm,
)

_BUNDLE_FIELD = {
    "intrinsic": "intrinsic",
    "hand_eye": "hand_eye",
    "joint_offset": "joint_offset",
    "link_offset": "link_offset",
    "sag": "sag",
}


# ── advanced-alchemy sub-repo (모듈 private CRUD 헬퍼 — 외부 노출 X) ──
# model_type 바인딩용 얇은 subclass. wrap_exceptions=False → raw SQLAlchemy 예외 유지.


class _RunRepo(SQLAlchemySyncRepository[CalibrationRunOrm]):
    model_type = CalibrationRunOrm


class _ResultRepo(SQLAlchemySyncRepository[CalibrationResultOrm]):
    model_type = CalibrationResultOrm


class _CaptureRepo(SQLAlchemySyncRepository[CalibrationCaptureOrm]):
    model_type = CalibrationCaptureOrm


class _ArtifactRepo(SQLAlchemySyncRepository[CalibrationCaptureArtifactOrm]):
    model_type = CalibrationCaptureArtifactOrm


class CalibrationRepository:
    def __init__(self, session_factory: Callable[[], Session]):
        self._session_factory = session_factory

    # ── run lifecycle ─────────────────────────────────────────
    def create_run(
        self, robot_id: str, kind: CalibrationKind, algorithm: str
    ) -> CalibrationRunRecord:
        """새 run (status=in_progress). started_at = now(UTC)."""
        with self._session_factory() as session:
            repo = _RunRepo(session=session, auto_commit=True, wrap_exceptions=False)
            orm = repo.add(
                CalibrationRunOrm(
                    robot_id=robot_id,
                    started_at=datetime.now(UTC),
                    algorithm=algorithm,
                    algorithm_params="{}",
                    status="in_progress",
                    kind=kind,
                )
            )
            return orm_to_run(orm)

    def get_run(self, run_id: int) -> CalibrationRunRecord | None:
        with self._session_factory() as session:
            orm = _RunRepo(session=session, wrap_exceptions=False).get_one_or_none(
                CalibrationRunOrm.id == run_id
            )
            return orm_to_run(orm) if orm is not None else None

    def get_in_progress_run(
        self, robot_id: str, kind: CalibrationKind
    ) -> CalibrationRunRecord | None:
        with self._session_factory() as session:
            rows = _RunRepo(session=session, wrap_exceptions=False).get_many(
                CalibrationRunOrm.robot_id == robot_id,
                CalibrationRunOrm.kind == kind,
                CalibrationRunOrm.status == "in_progress",
                order_by=CalibrationRunOrm.id.desc(),
            )
            return orm_to_run(rows[0]) if rows else None

    def finalize_run(self, run_id: int, status: CalibrationRunStatus) -> None:
        with self._session_factory() as session:
            repo = _RunRepo(session=session, wrap_exceptions=False)
            orm = repo.get_one_or_none(CalibrationRunOrm.id == run_id)
            if orm is None:
                raise KeyError(f"run {run_id} 없음")
            orm.status = status
            orm.ended_at = datetime.now(UTC)
            session.commit()

    # ── capture (per-pose) ────────────────────────────────────
    def append_capture(self, run_id: int, capture: CalibrationCaptureRecord) -> int:
        with self._session_factory() as session:
            repo = _CaptureRepo(session=session, auto_commit=True, wrap_exceptions=False)
            orm = repo.add(capture_record_to_orm(capture, run_id=run_id))
            return orm.id

    def list_captures(self, run_id: int) -> list[CalibrationCaptureRecord]:
        with self._session_factory() as session:
            caps = _CaptureRepo(session=session, wrap_exceptions=False).get_many(
                CalibrationCaptureOrm.run_id == run_id,
                order_by=CalibrationCaptureOrm.pose_index.asc(),
            )
            art_repo = _ArtifactRepo(session=session, wrap_exceptions=False)
            out: list[CalibrationCaptureRecord] = []
            for cap in caps:
                arts = art_repo.get_many(
                    CalibrationCaptureArtifactOrm.capture_id == cap.id
                )
                out.append(
                    orm_to_capture(cap, artifacts=[orm_to_artifact(a) for a in arts])
                )
            return out

    def undo_last_capture(self, run_id: int) -> None:
        """마지막(pose_index 최대) capture 삭제. artifact 는 FK CASCADE."""
        with self._session_factory() as session:
            repo = _CaptureRepo(session=session, auto_commit=True, wrap_exceptions=False)
            rows = repo.get_many(
                CalibrationCaptureOrm.run_id == run_id,
                order_by=CalibrationCaptureOrm.pose_index.desc(),
            )
            if not rows:
                raise KeyError(f"run {run_id} 에 capture 없음")
            repo.delete(rows[0].id)

    def save_artifact(
        self, capture_id: int, artifact: CalibrationCaptureArtifactRecord
    ) -> None:
        with self._session_factory() as session:
            repo = _ArtifactRepo(
                session=session, auto_commit=True, wrap_exceptions=False
            )
            repo.add(artifact_record_to_orm(artifact, capture_id=capture_id))

    # ── result (5 kind, activate atomic) ─────────────────────
    def save_result(self, run_id: int, result: CalibrationResultRecord) -> int:
        """result INSERT (is_active=False). activate 는 별도 (atomic)."""
        with self._session_factory() as session:
            repo = _ResultRepo(session=session, auto_commit=True, wrap_exceptions=False)
            orm = repo.add(result_record_to_orm(result, run_id=run_id, is_active=False))
            return orm.id

    def activate_result(self, result_id: int) -> CalibrationResultRecord:
        """atomic: 같은 (robot_id, kind) 의 기존 active 해제 → 새 result 활성.

        활성화한 record 를 반환 (caller 가 ACTIVATED event 에 kind 등 필요).
        도메인 로직 (UPDATE + UPDATE) — 라이브러리 CRUD 밖. partial unique index
        (robot_id, kind WHERE is_active) 때문에 기존 해제를 **먼저** flush 해야 2-active
        순간 unique 위반이 안 남. 한 session/transaction 안에서 raw 로 처리.
        """
        with self._session_factory() as session:
            target = session.get(CalibrationResultOrm, result_id)
            if target is None:
                raise KeyError(f"result {result_id} 없음")
            current = session.scalars(
                select(CalibrationResultOrm).where(
                    CalibrationResultOrm.robot_id == target.robot_id,
                    CalibrationResultOrm.kind == target.kind,
                    CalibrationResultOrm.is_active.is_(True),
                    CalibrationResultOrm.id != result_id,
                )
            ).all()
            for c in current:
                c.is_active = False
            session.flush()  # 해제 먼저 반영 (unique 위반 회피)
            target.is_active = True
            session.commit()
            return orm_to_result(target)

    def get_active(
        self, robot_id: str, kind: CalibrationKind
    ) -> CalibrationResultRecord | None:
        with self._session_factory() as session:
            orm = _ResultRepo(
                session=session, wrap_exceptions=False
            ).get_one_or_none(
                CalibrationResultOrm.robot_id == robot_id,
                CalibrationResultOrm.kind == kind,
                CalibrationResultOrm.is_active.is_(True),
            )
            return orm_to_result(orm) if orm is not None else None

    def get_active_bundle(self, robot_id: str) -> CalibrationBundle:
        """현재 active 5 kind 를 한번에 → CalibrationBundle (atomic snapshot)."""
        with self._session_factory() as session:
            actives = _ResultRepo(session=session, wrap_exceptions=False).get_many(
                CalibrationResultOrm.robot_id == robot_id,
                CalibrationResultOrm.is_active.is_(True),
            )
            fields: dict[str, CalibrationResultRecord] = {}
            for orm in actives:
                fields[_BUNDLE_FIELD[orm.kind]] = orm_to_result(orm)
            return CalibrationBundle(robot_id=robot_id, **fields)  # type: ignore[arg-type]

    # ── history (readonly) ────────────────────────────────────
    def list_runs(
        self, robot_id: str, kind: CalibrationKind | None = None
    ) -> list[CalibrationRunRecord]:
        with self._session_factory() as session:
            filters = [CalibrationRunOrm.robot_id == robot_id]
            if kind is not None:
                filters.append(CalibrationRunOrm.kind == kind)
            rows = _RunRepo(session=session, wrap_exceptions=False).get_many(
                *filters, order_by=CalibrationRunOrm.id.desc()
            )
            return [orm_to_run(o) for o in rows]

    def list_results(
        self, robot_id: str, kind: CalibrationKind | None = None
    ) -> list[CalibrationResultRecord]:
        with self._session_factory() as session:
            filters = [CalibrationResultOrm.robot_id == robot_id]
            if kind is not None:
                filters.append(CalibrationResultOrm.kind == kind)
            rows = _ResultRepo(session=session, wrap_exceptions=False).get_many(
                *filters, order_by=CalibrationResultOrm.created_at.desc()
            )
            return [orm_to_result(o) for o in rows]
