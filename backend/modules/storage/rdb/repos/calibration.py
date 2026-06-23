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


class _RunRepo(SQLAlchemySyncRepository[CalibrationRunOrm]):
    model_type = CalibrationRunOrm


class _ResultRepo(SQLAlchemySyncRepository[CalibrationResultOrm]):
    model_type = CalibrationResultOrm


class _CaptureRepo(SQLAlchemySyncRepository[CalibrationCaptureOrm]):
    model_type = CalibrationCaptureOrm


class _ArtifactRepo(SQLAlchemySyncRepository[CalibrationCaptureArtifactOrm]):
    model_type = CalibrationCaptureArtifactOrm


class CalibrationRepo:
    def __init__(self, session: Session) -> None:
        self.session = session
        self.runs = _RunRepo(session=session, wrap_exceptions=False)
        self.results = _ResultRepo(session=session, wrap_exceptions=False)
        self.captures = _CaptureRepo(session=session, wrap_exceptions=False)
        self.artifacts = _ArtifactRepo(session=session, wrap_exceptions=False)

    # ----------------------------------
    # Lifecycle
    # ----------------------------------

    def new_run(self, run: CalibrationRunRecord) -> int:
        orm = self.runs.add(run_record_to_orm(run, force_status="in_progress"))
        assert orm.id is not None
        return orm.id

    def append_capture(
        self,
        capture: CalibrationCaptureRecord,
        artifacts: list[CalibrationCaptureArtifactRecord] | None = None,
    ) -> int:
        run_orm = self.runs.get_one_or_none(CalibrationRunOrm.id == capture.run_id)
        if run_orm is None:
            raise KeyError(f"run id={capture.run_id} 없음")
        if run_orm.status != "in_progress":
            raise ValueError(
                f"run id={capture.run_id} status={run_orm.status!r} — "
                "capture append 불가 (in_progress 만 허용)"
            )
        orm = self.captures.add(capture_record_to_orm(capture, run_id=capture.run_id))
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
        # Capture 삭제 시 artifact row 도 CASCADE 로 제거된다.
        # ObjectStore cleanup 에 사용할 artifact metadata 를 미리 조회.
        artifact_orms = self.session.scalars(
            select(CalibrationCaptureArtifactOrm).where(
                CalibrationCaptureArtifactOrm.capture_id == cid
            )
        ).all()
        artifacts = [orm_to_artifact(a) for a in artifact_orms]
        self.session.delete(orm)
        self.session.flush()
        return pose_index, artifacts

    def mark_run_ready(self, run_id: int) -> CalibrationRunRecord:
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
    ) -> list[int]:
        run = self.runs.get_one_or_none(CalibrationRunOrm.id == run_id)
        if run is None or run.status != "ready_for_analysis":
            raise KeyError(
                f"finalize 가능 run id={run_id} 없음 / 이미 종료 / mark_run_ready 안 함 "
                f"(status={run.status if run else 'None'!r})"
            )

        ended_at = results[0].created_at if results else run.started_at
        run.status = "success"
        run.ended_at = ended_at

        result_ids: list[int] = []
        for r in results:
            ro = self.results.add(
                result_record_to_orm(r, run_id=run_id, is_active=False)
            )
            assert ro.id is not None
            result_ids.append(ro.id)

        return result_ids

    def activate_result(self, result_id: int) -> CalibrationResultRecord:
        target = self.results.get_one_or_none(CalibrationResultOrm.id == result_id)
        if target is None:
            raise KeyError(f"result_id={result_id} 없음")
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

    def commit(
        self,
        run: CalibrationRunRecord,
        results: list[CalibrationResultRecord],
        captures: list[CalibrationCaptureRecord],
    ) -> tuple[int, list[int]]:
        """완료된 캘리브레이션을 한 transaction 으로 저장한다.

        run, results, captures 를 원자적으로 INSERT 한다.

        이 메서드는 결과가 이미 계산된 경우 사용하는 저장 경로이며,
        보통 intrinsic calibration 이 해당된다.

        Hand-Eye calibration 의 staged workflow:

            new_run()
            -> append_capture()
            -> mark_run_ready()
            -> finalize_run()

        를 축약한 것이 아니다. Hand-Eye 는 여러 transaction 에 걸쳐
        데이터를 수집하고, 나중에 결과를 생성한다.

        반면 본 메서드는 결과가 준비된 상태에서 run 과 result 를
        한 번에 저장한다.
        """
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

    # ----------------------------------
    # Queries
    # ----------------------------------

    def get_run(self, run_id: int) -> CalibrationRunRecord | None:
        orm = self.runs.get_one_or_none(CalibrationRunOrm.id == run_id)
        return orm_to_run(orm) if orm else None

    def get_result(self, result_id: int) -> CalibrationResultRecord | None:
        orm = self.results.get_one_or_none(CalibrationResultOrm.id == result_id)
        return orm_to_result(orm) if orm else None

    def get_active_result(
        self, robot_id: str, kind: CalibrationKind
    ) -> CalibrationResultRecord | None:
        orm = self.results.get_one_or_none(
            CalibrationResultOrm.robot_id == robot_id,
            CalibrationResultOrm.kind == kind,
            CalibrationResultOrm.is_active.is_(True),
        )
        return orm_to_result(orm) if orm else None

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

        # 모든 run 의 result 를 한 번에 조회.
        # run 별로 개별 query 를 날리면 N+1 문제가 생기므로
        # IN (...) 으로 묶어서 가져온 뒤 메모리에서 그룹핑한다.
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

    def get_in_progress_run(
        self, robot_id: str, kind: CalibrationKind
    ) -> tuple[CalibrationRunRecord, list[CalibrationCaptureRecord]] | None:
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

    # ----------------------------------
    # Deletion
    #
    # delete flow:
    #   list_run_artifacts()
    #       -> ObjectStore.delete(blob_key)
    #       -> delete_run()
    #
    # list_run_artifacts() 는 조회 함수지만 delete cleanup 과정에서만
    # 사용되므로 delete_run() 과 함께 배치한다.
    # ----------------------------------

    def list_run_artifacts(self, run_id: int) -> list[CalibrationCaptureArtifactRecord]:
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
        # Run 삭제. FK CASCADE 로 captures/results/artifacts 도 함께 삭제된다.
        orm = self.runs.get_one_or_none(CalibrationRunOrm.id == run_id)
        if orm is not None:
            self.session.delete(orm)
            self.session.flush()

    # ----------------------------------
    # Internal
    # ----------------------------------

    def _artifacts_for_captures(
        self, capture_ids: list[int]
    ) -> dict[int, list[CalibrationCaptureArtifactRecord]]:
        """capture_id -> artifacts[] 매핑 생성 (bulk fetch, N+1 회피)."""
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
