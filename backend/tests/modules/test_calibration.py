"""Calibration persistence 검증.

3 축:
1. **실 DB fixture** — backend/storage/horibot.db (실 하드웨어 캘 결과) 를 복사해 읽어
   포팅한 ORM/Repository 가 실데이터를 그대로 복원하는지 (schema 동일 이월 검증).
2. **repo round-trip** — in-memory create_all 위 CRUD.
3. **activate invariant** — (robot, kind) active 하나 (atomic switch) + Bundle + CASCADE.
"""

from __future__ import annotations

import shutil
from datetime import UTC, datetime
from pathlib import Path

import pytest
from sqlalchemy import func, select

from infra.database.sqlite import open_sqlite
from modules.calibration.contract import (
    CalibrationCaptureArtifactRecord,
    CalibrationCaptureRecord,
    HandEyeResultData,
    HandEyeResultRecord,
    IntrinsicResultData,
    IntrinsicResultRecord,
    JointOffsetResultData,
    JointOffsetResultRecord,
    LinkOffsetEntry,
    LinkOffsetResultData,
    LinkOffsetResultRecord,
    SagOffsetResultData,
    SagOffsetResultRecord,
)
from modules.calibration.persistence.orm import Base, CalibrationResultOrm
from modules.calibration.persistence.repository import CalibrationRepository

_REAL_DB = Path(__file__).resolve().parents[3] / "backend" / "storage" / "horibot.db"
_ROBOT = "so101_6dof_0"


def _fresh_repo(tmp_path: Path) -> CalibrationRepository:
    engine, factory = open_sqlite(tmp_path / "calib.db")
    Base.metadata.create_all(engine)
    return CalibrationRepository(factory)


def _now() -> datetime:
    return datetime.now(UTC)


# ─────────────────────────── 1. 실 DB fixture ───────────────────────────


@pytest.mark.skipif(not _REAL_DB.exists(), reason="실 horibot.db 없음 (집 머신 자리)")
def test_real_db_runs_captures_results_roundtrip(tmp_path: Path):
    """실 하드웨어 캘 데이터를 포팅 스키마로 그대로 읽어낸다."""
    db = tmp_path / "real.db"
    shutil.copy(_REAL_DB, db)  # 원본 mutate 방지 (WAL pragma write)
    _, factory = open_sqlite(db)
    repo = CalibrationRepository(factory)

    # runs — so101 3개 (intrinsic / hand_eye / intrinsic)
    runs = repo.list_runs(_ROBOT)
    assert len(runs) == 3
    assert {r.kind for r in runs} == {"intrinsic", "hand_eye"}

    # captures — hand_eye run(2) 34개, BA 입력 완비
    he_run = next(r for r in runs if r.kind == "hand_eye")
    assert he_run.id is not None
    caps = repo.list_captures(he_run.id)
    assert len(caps) == 34
    for c in caps:
        assert c.motor_positions is not None and len(c.motor_positions) == 6
        assert all(isinstance(k, int) for k in c.motor_positions)  # int key 복원
        assert c.board_in_cam is not None and len(c.board_in_cam) == 4
        assert c.corners_2d and c.corner_ids

    # results — 5 kind 전부 discriminated-union parse 성공 (drift 없음)
    results = repo.list_results(_ROBOT)
    assert {r.kind for r in results} == {
        "intrinsic",
        "hand_eye",
        "joint_offset",
        "link_offset",
        "sag",
    }


@pytest.mark.skipif(not _REAL_DB.exists(), reason="실 horibot.db 없음 (집 머신 자리)")
def test_real_db_active_bundle_and_known_sigma(tmp_path: Path):
    """active bundle = 5 kind + hand_eye σ 가 known-good (0.818°/7.538mm)."""
    db = tmp_path / "real.db"
    shutil.copy(_REAL_DB, db)
    _, factory = open_sqlite(db)
    repo = CalibrationRepository(factory)

    bundle = repo.get_active_bundle(_ROBOT)
    assert bundle.robot_id == _ROBOT
    assert bundle.intrinsic is not None
    assert bundle.hand_eye is not None
    assert bundle.joint_offset is not None
    assert bundle.link_offset is not None
    assert bundle.sag is not None
    assert len(bundle.signature()) == 5

    he = bundle.hand_eye
    assert he.effective_sigma_rot == pytest.approx(0.818, abs=0.01)
    assert he.effective_sigma_t == pytest.approx(7.538, abs=0.01)
    # result_data 실 shape
    assert len(he.result_data.R_cam2gripper) == 3
    assert len(he.result_data.t_cam2gripper) == 3


# ─────────────────────────── 2. repo round-trip ───────────────────────────


def test_run_lifecycle_and_capture_roundtrip(tmp_path: Path):
    repo = _fresh_repo(tmp_path)

    run = repo.create_run(_ROBOT, "hand_eye", "hand_eye_capture_only")
    assert run.id is not None
    assert run.status == "in_progress"
    assert repo.get_in_progress_run(_ROBOT, "hand_eye") is not None

    cap_id = repo.append_capture(
        run.id,
        CalibrationCaptureRecord(
            run_id=run.id,
            pose_index=0,
            motor_positions={1: 2041, 2: 2342, 3: 903, 4: 2846, 5: 2120, 6: 3122},
            board_in_cam=[[1.0, 0, 0, 0.1], [0, 1, 0, 0.2], [0, 0, 1, 0.3], [0, 0, 0, 1]],
            corners_2d=[[1.0, 2.0], [3.0, 4.0]],
            corner_ids=[0, 1],
            reproj_rms_px=0.12,
            tilt_deg=45.0,
        ),
    )
    repo.save_artifact(
        cap_id,
        CalibrationCaptureArtifactRecord(
            capture_id=cap_id,
            kind="primary",
            blob_key=f"calib_captures/{_ROBOT}/{run.id}/000.bin",
            size_bytes=1234,
            created_at=_now(),
        ),
    )

    caps = repo.list_captures(run.id)
    assert len(caps) == 1
    assert caps[0].motor_positions == {1: 2041, 2: 2342, 3: 903, 4: 2846, 5: 2120, 6: 3122}
    assert caps[0].find_artifact("primary") is not None

    repo.finalize_run(run.id, "ready_for_analysis")
    assert repo.get_run(run.id).status == "ready_for_analysis"  # type: ignore[union-attr]
    assert repo.get_in_progress_run(_ROBOT, "hand_eye") is None


def test_undo_last_capture_cascades_artifacts(tmp_path: Path):
    """undo → 마지막 capture + 그 artifact(FK CASCADE) 삭제. FK pragma 검증."""
    repo = _fresh_repo(tmp_path)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None
    c0 = repo.append_capture(run.id, CalibrationCaptureRecord(run_id=run.id, pose_index=0))
    c1 = repo.append_capture(run.id, CalibrationCaptureRecord(run_id=run.id, pose_index=1))
    repo.save_artifact(
        c1,
        CalibrationCaptureArtifactRecord(
            capture_id=c1, kind="color", blob_key="k", created_at=_now()
        ),
    )

    repo.undo_last_capture(run.id)

    caps = repo.list_captures(run.id)
    assert [c.pose_index for c in caps] == [0]
    assert caps[0].id == c0
    # c1 의 artifact 가 CASCADE 로 삭제됐는지 (FK pragma OFF 면 orphan 남음)
    with repo._session_factory() as s:  # noqa: SLF001 — test 내부 검증
        from modules.calibration.persistence.orm import CalibrationCaptureArtifactOrm

        n = s.scalar(
            select(func.count()).select_from(CalibrationCaptureArtifactOrm)
        )
        assert n == 0


# ─────────────────────────── 3. activate invariant ───────────────────────────


def _save_active(repo: CalibrationRepository, run_id: int, rec) -> int:
    rid = repo.save_result(run_id, rec)
    repo.activate_result(rid)
    return rid


def test_activate_switches_atomically_single_active(tmp_path: Path):
    repo = _fresh_repo(tmp_path)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None

    def _he(method: str) -> HandEyeResultRecord:
        return HandEyeResultRecord(
            run_id=run.id,  # type: ignore[arg-type]
            robot_id=_ROBOT,
            created_at=_now(),
            result_data=HandEyeResultData(
                R_cam2gripper=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                t_cam2gripper=[[0], [0], [0]],
                method=method,
            ),
        )

    first = repo.save_result(run.id, _he("TSAI"))
    second = repo.save_result(run.id, _he("BA"))

    repo.activate_result(first)
    active = repo.get_active(_ROBOT, "hand_eye")
    assert active is not None and active.id == first

    repo.activate_result(second)  # atomic switch
    active = repo.get_active(_ROBOT, "hand_eye")
    assert isinstance(active, HandEyeResultRecord) and active.id == second
    assert active.result_data.method == "BA"

    # invariant: (robot, hand_eye) active 정확히 1
    with repo._session_factory() as s:  # noqa: SLF001
        n = s.scalar(
            select(func.count())
            .select_from(CalibrationResultOrm)
            .where(
                CalibrationResultOrm.robot_id == _ROBOT,
                CalibrationResultOrm.kind == "hand_eye",
                CalibrationResultOrm.is_active.is_(True),
            )
        )
        assert n == 1


def test_full_bundle_all_five_kinds(tmp_path: Path):
    repo = _fresh_repo(tmp_path)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None
    rid = run.id

    _save_active(
        repo,
        rid,
        IntrinsicResultRecord(
            run_id=rid,
            robot_id=_ROBOT,
            created_at=_now(),
            result_data=IntrinsicResultData(
                camera_matrix=[[600, 0, 320], [0, 600, 240], [0, 0, 1]],
                dist_coeffs=[[0.0, 0.0, 0.0, 0.0, 0.0]],
                image_size=[640, 480],
            ),
        ),
    )
    _save_active(
        repo,
        rid,
        HandEyeResultRecord(
            run_id=rid,
            robot_id=_ROBOT,
            created_at=_now(),
            result_data=HandEyeResultData(
                R_cam2gripper=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                t_cam2gripper=[[0], [0], [0]],
                method="BA",
            ),
        ),
    )
    _save_active(
        repo,
        rid,
        JointOffsetResultRecord(
            run_id=rid,
            robot_id=_ROBOT,
            created_at=_now(),
            result_data=JointOffsetResultData(offsets={1: 0.01, 2: -0.02}, method="ba"),
        ),
    )
    _save_active(
        repo,
        rid,
        LinkOffsetResultRecord(
            run_id=rid,
            robot_id=_ROBOT,
            created_at=_now(),
            result_data=LinkOffsetResultData(
                offsets=[LinkOffsetEntry(joint_id=2, trans_m=[0, 0, 0.001], rot_rad=[0, 0, 0])],
                method="ba",
            ),
        ),
    )
    _save_active(
        repo,
        rid,
        SagOffsetResultRecord(
            run_id=rid,
            robot_id=_ROBOT,
            created_at=_now(),
            result_data=SagOffsetResultData(k_rad_per_m={2: 0.0005, 3: 0.0003}, method="ba"),
        ),
    )

    bundle = repo.get_active_bundle(_ROBOT)
    assert bundle.intrinsic is not None
    assert bundle.hand_eye is not None
    assert bundle.joint_offset is not None
    assert bundle.link_offset is not None
    assert bundle.sag is not None
    assert len(bundle.signature()) == 5
    # kind 별 field 정합 (엉뚱한 자리 매핑 없음)
    assert bundle.joint_offset.result_data.offsets == {1: 0.01, 2: -0.02}
    assert bundle.sag.result_data.k_rad_per_m == {2: 0.0005, 3: 0.0003}
