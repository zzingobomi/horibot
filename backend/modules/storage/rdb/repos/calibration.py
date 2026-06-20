"""Calibration domain repository — run / result / capture 3 테이블.

Advanced Alchemy `SQLAlchemySyncRepository` 위 도메인 facade. entity 별 sub-repo
(`runs` / `results` / `captures`) 컴포지션 + workflow 메서드 (`commit` /
`activate_result` / `finalize_run`).

session 은 caller 가 주입 (RdbStore.session() context manager) — repo lifecycle
= session lifecycle = transaction 경계. `auto_commit=False` 라서 메서드들 안에서
flush 만, 실제 commit 은 `session_scope` 의 `__exit__` 가 담당. 즉 한 `with
rdb.session() as repos:` 블록이 한 transaction.
"""

from __future__ import annotations

import logging

from advanced_alchemy.filters import LimitOffset
from advanced_alchemy.repository import SQLAlchemySyncRepository
from sqlalchemy import select, update
from sqlalchemy.orm import Session

from modules.calibration.orm import (
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
    run_record_to_orm,
)
from modules.calibration.persistence_models import (
    CalibrationCaptureArtifactRecord,
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationResultRecord,
    CalibrationRunRecord,
)

logger = logging.getLogger(__name__)


# ─── Entity sub-repos (Advanced Alchemy CRUD baseline) ───
# pyright 자리 `ModelProtocol` 가 `__table__` / `__mapper__` 의 ClassVar 명시 요구 —
# SQLAlchemy 2.x DeclarativeBase 의 metaclass 가 attribute 설정 자리라 strict
# check 가 무난한 호환성 자리 다 안 잡음. 런타임 정상.


class _RunRepo(SQLAlchemySyncRepository[CalibrationRunOrm]):  # type: ignore[type-var]
    model_type = CalibrationRunOrm


class _ResultRepo(SQLAlchemySyncRepository[CalibrationResultOrm]):  # type: ignore[type-var]
    model_type = CalibrationResultOrm


class _CaptureRepo(SQLAlchemySyncRepository[CalibrationCaptureOrm]):  # type: ignore[type-var]
    model_type = CalibrationCaptureOrm


class _ArtifactRepo(SQLAlchemySyncRepository[CalibrationCaptureArtifactOrm]):  # type: ignore[type-var]
    model_type = CalibrationCaptureArtifactOrm


# ─── Domain facade ───


class CalibrationRepo:
    """캘리브레이션 도메인 — entity sub-repo 컴포지션 + workflow 메서드."""

    def __init__(self, session: Session) -> None:
        self.session = session
        self.runs = _RunRepo(session=session, wrap_exceptions=False)
        self.results = _ResultRepo(session=session, wrap_exceptions=False)
        self.captures = _CaptureRepo(session=session, wrap_exceptions=False)
        self.artifacts = _ArtifactRepo(session=session, wrap_exceptions=False)

    # ─── Capture + Artifact 결합 read helpers ────────────────

    def _artifacts_for_captures(
        self, capture_ids: list[int]
    ) -> dict[int, list[CalibrationCaptureArtifactRecord]]:
        """capture_id → artifacts list 매핑 — N+1 query 회피."""
        if not capture_ids:
            return {}
        orms = self.session.scalars(
            select(CalibrationCaptureArtifactOrm)
            .where(CalibrationCaptureArtifactOrm.capture_id.in_(capture_ids))
            .order_by(
                CalibrationCaptureArtifactOrm.capture_id,
                CalibrationCaptureArtifactOrm.kind,
            )
        ).all()
        out: dict[int, list[CalibrationCaptureArtifactRecord]] = {
            cid: [] for cid in capture_ids
        }
        for o in orms:
            out.setdefault(o.capture_id, []).append(orm_to_artifact(o))
        return out

    # ─── Read ────────────────────────────────────────────────

    def get_active_result(
        self, robot_id: str, kind: CalibrationKind
    ) -> CalibrationResultRecord | None:
        orm = self.results.get_one_or_none(
            CalibrationResultOrm.robot_id == robot_id,
            CalibrationResultOrm.kind == kind,
            CalibrationResultOrm.is_active.is_(True),
        )
        return orm_to_result(orm) if orm else None

    def list_results(
        self, robot_id: str, kind: CalibrationKind, limit: int = 100
    ) -> list[CalibrationResultRecord]:
        orms = self.results.get_many(
            CalibrationResultOrm.robot_id == robot_id,
            CalibrationResultOrm.kind == kind,
            LimitOffset(limit=limit, offset=0),
            order_by=CalibrationResultOrm.created_at.desc(),
        )
        return [orm_to_result(o) for o in orms]

    def list_runs(
        self, robot_id: str, limit: int = 50
    ) -> list[tuple[CalibrationRunRecord, list[CalibrationResultRecord]]]:
        run_orms = self.runs.get_many(
            CalibrationRunOrm.robot_id == robot_id,
            LimitOffset(limit=limit, offset=0),
            order_by=CalibrationRunOrm.started_at.desc(),
        )
        if not run_orms:
            return []
        runs = [orm_to_run(o) for o in run_orms]
        run_ids = [r.id for r in runs if r.id is not None]

        # 한 번에 모든 Result fetch — N+1 query 회피 (IN 절).
        result_orms = self.results.get_many(
            CalibrationResultOrm.run_id.in_(run_ids),
            order_by=CalibrationResultOrm.created_at.desc(),
        )
        results_by_run: dict[int, list[CalibrationResultRecord]] = {
            rid: [] for rid in run_ids
        }
        for o in result_orms:
            results_by_run.setdefault(o.run_id, []).append(orm_to_result(o))
        return [
            (run, results_by_run.get(run.id, []) if run.id is not None else [])
            for run in runs
        ]

    def get_result(self, result_id: int) -> CalibrationResultRecord | None:
        orm = self.results.get_one_or_none(CalibrationResultOrm.id == result_id)
        return orm_to_result(orm) if orm else None

    def get_run(self, run_id: int) -> CalibrationRunRecord | None:
        orm = self.runs.get_one_or_none(CalibrationRunOrm.id == run_id)
        return orm_to_run(orm) if orm else None

    def list_captures(self, run_id: int) -> list[CalibrationCaptureRecord]:
        orms = self.captures.get_many(
            CalibrationCaptureOrm.run_id == run_id,
            order_by=CalibrationCaptureOrm.pose_index.asc(),
        )
        ids = [o.id for o in orms if o.id is not None]
        arts_by_cap = self._artifacts_for_captures(ids)
        return [
            orm_to_capture(o, artifacts=arts_by_cap.get(o.id, []) if o.id else [])
            for o in orms
        ]

    # ─── Write (atomic transaction per method) ───────────────

    def commit(
        self,
        run: CalibrationRunRecord,
        results: list[CalibrationResultRecord],
        captures: list[CalibrationCaptureRecord],
    ) -> tuple[int, list[int]]:
        run_orm = self.runs.add(run_record_to_orm(run))
        run_id = run_orm.id
        assert run_id is not None

        result_ids: list[int] = []
        for r in results:
            ro = self.results.add(
                result_record_to_orm(r, run_id=run_id, is_active=False)
            )
            assert ro.id is not None
            result_ids.append(ro.id)

        if captures:
            self.captures.add_many(
                [capture_record_to_orm(c, run_id=run_id) for c in captures]
            )

        return run_id, result_ids

    # ─── Draft run (사용자 [캘 시작] flow) ───────────────────

    def new_run(self, run: CalibrationRunRecord) -> int:
        orm = self.runs.add(run_record_to_orm(run, force_status="in_progress"))
        assert orm.id is not None
        return orm.id

    def append_capture(
        self,
        capture: CalibrationCaptureRecord,
        artifacts: list[CalibrationCaptureArtifactRecord] | None = None,
    ) -> int:
        """capture row + artifacts (옵션) atomic INSERT. capture.id 반환.

        in_progress run 만 허용. artifacts 자리 capture row 의 id 채워 같이 INSERT.
        """
        run_orm = self.runs.get_one_or_none(CalibrationRunOrm.id == capture.run_id)
        if run_orm is None:
            raise KeyError(f"run id={capture.run_id} 없음")
        if run_orm.status != "in_progress":
            raise ValueError(
                f"run id={capture.run_id} status={run_orm.status!r} — "
                "capture append 불가 (in_progress 만 허용)"
            )
        orm = self.captures.add(
            capture_record_to_orm(capture, run_id=capture.run_id)
        )
        assert orm.id is not None
        cid = orm.id
        if artifacts:
            self.artifacts.add_many(
                [artifact_record_to_orm(a, capture_id=cid) for a in artifacts]
            )
        return cid

    def delete_last_capture(
        self, run_id: int
    ) -> tuple[int, list[CalibrationCaptureArtifactRecord]] | None:
        """마지막 capture row + 자식 artifact 들 cascade 삭제. 삭제된 artifact
        record 목록 반환 — caller (handler) 가 ObjectStore cleanup.
        """
        orm = self.session.scalars(
            select(CalibrationCaptureOrm)
            .where(CalibrationCaptureOrm.run_id == run_id)
            .order_by(CalibrationCaptureOrm.pose_index.desc())
            .limit(1)
        ).first()
        if orm is None:
            return None
        pose_index = orm.pose_index
        cid = orm.id
        # artifacts 자리 cascade 자리 삭제되기 전 fetch.
        artifact_orms = self.session.scalars(
            select(CalibrationCaptureArtifactOrm).where(
                CalibrationCaptureArtifactOrm.capture_id == cid
            )
        ).all()
        artifacts = [orm_to_artifact(a) for a in artifact_orms]
        self.session.delete(orm)  # CASCADE 가 자식 artifact row 자동 삭제.
        self.session.flush()
        return pose_index, artifacts

    def get_in_progress_run(
        self, robot_id: str, kind: CalibrationKind
    ) -> tuple[CalibrationRunRecord, list[CalibrationCaptureRecord]] | None:
        # "in_progress + 최신 1건" — get_one_or_none 자리 order_by X, raw 사용.
        run_orm = self.session.scalars(
            select(CalibrationRunOrm)
            .where(
                CalibrationRunOrm.robot_id == robot_id,
                CalibrationRunOrm.kind == kind,
                CalibrationRunOrm.status == "in_progress",
            )
            .order_by(CalibrationRunOrm.started_at.desc())
            .limit(1)
        ).first()
        if run_orm is None:
            return None
        run = orm_to_run(run_orm)
        cap_orms = self.captures.get_many(
            CalibrationCaptureOrm.run_id == run_orm.id,
            order_by=CalibrationCaptureOrm.pose_index.asc(),
        )
        ids = [c.id for c in cap_orms if c.id is not None]
        arts_by_cap = self._artifacts_for_captures(ids)
        captures = [
            orm_to_capture(c, artifacts=arts_by_cap.get(c.id, []) if c.id else [])
            for c in cap_orms
        ]
        return run, captures

    def list_run_artifacts(
        self, run_id: int
    ) -> list[CalibrationCaptureArtifactRecord]:
        """run 의 모든 capture artifacts — delete_run 전 ObjectStore cleanup 용."""
        cap_ids = self.session.scalars(
            select(CalibrationCaptureOrm.id).where(
                CalibrationCaptureOrm.run_id == run_id
            )
        ).all()
        if not cap_ids:
            return []
        orms = self.session.scalars(
            select(CalibrationCaptureArtifactOrm).where(
                CalibrationCaptureArtifactOrm.capture_id.in_(list(cap_ids))
            )
        ).all()
        return [orm_to_artifact(a) for a in orms]

    def delete_run(self, run_id: int) -> None:
        # FK ON DELETE CASCADE + PRAGMA foreign_keys=ON → 자식 captures / results /
        # artifacts 자동. ObjectStore blob 자리 호출자 (handler) 가 list_run_artifacts
        # 먼저 fetch 후 별도 ObjectStore.delete — RDB cascade 가 외부 store 까지 못 미침.
        orm = self.runs.get_one_or_none(CalibrationRunOrm.id == run_id)
        if orm is not None:
            self.session.delete(orm)
            self.session.flush()

    def mark_run_ready(self, run_id: int) -> CalibrationRunRecord:
        """in_progress → ready_for_analysis. 다른 status 는 ValueError.

        ready_for_analysis 진입 후엔 capture append 차단 (handler 가 status 체크).
        """
        orm = self.runs.get_one_or_none(CalibrationRunOrm.id == run_id)
        if orm is None:
            raise KeyError(f"run id={run_id} 없음")
        if orm.status != "in_progress":
            raise ValueError(
                f"run id={run_id} status={orm.status!r} — in_progress 가 아님"
            )
        orm.status = "ready_for_analysis"
        self.session.flush()
        return orm_to_run(orm)

    def finalize_run(
        self,
        run_id: int,
        results: list[CalibrationResultRecord],
        capture_residuals: dict[int, tuple[float | None, float | None, float | None]]
        | None = None,
    ) -> list[int]:
        run = self.runs.get_one_or_none(CalibrationRunOrm.id == run_id)
        # ready_for_analysis (offline 스크립트 정상 경로) + in_progress (legacy /
        # 직접 finalize 경로) 둘 다 허용.
        if run is None or run.status not in ("in_progress", "ready_for_analysis"):
            raise KeyError(
                f"finalize 가능 run id={run_id} 없음 / 이미 종료 "
                f"(status={run.status if run else 'None'!r})"
            )

        ended_at = results[0].created_at if results else run.started_at
        run.status = "success"
        run.ended_at = ended_at

        if capture_residuals:
            # pose_index 별 residual UPDATE — IRLS BA output.
            for pose_index, (rrot, rtrans, weight) in capture_residuals.items():
                self.session.execute(
                    update(CalibrationCaptureOrm)
                    .where(
                        CalibrationCaptureOrm.run_id == run_id,
                        CalibrationCaptureOrm.pose_index == pose_index,
                    )
                    .values(
                        residual_rot=rrot,
                        residual_trans=rtrans,
                        weight=weight,
                    )
                )

        result_ids: list[int] = []
        for r in results:
            ro = self.results.add(
                result_record_to_orm(r, run_id=run_id, is_active=False)
            )
            assert ro.id is not None
            result_ids.append(ro.id)

        return result_ids

    # ─── ACTIVATE (atomic toggle) ────────────────────────────

    def activate_result(self, result_id: int) -> CalibrationResultRecord:
        target = self.results.get_one_or_none(CalibrationResultOrm.id == result_id)
        if target is None:
            raise KeyError(f"result_id={result_id} 없음")
        # UNIQUE partial index 일관성 위해 deactivate 먼저, activate 나중.
        self.session.execute(
            update(CalibrationResultOrm)
            .where(
                CalibrationResultOrm.robot_id == target.robot_id,
                CalibrationResultOrm.kind == target.kind,
                CalibrationResultOrm.is_active.is_(True),
                CalibrationResultOrm.id != result_id,
            )
            .values(is_active=False)
        )
        target.is_active = True
        self.session.flush()
        return orm_to_result(target)
