"""Storage — calibration ORM round-trip + transaction semantics.

`RdbStore` + `CalibrationRepo` 가 SQLAlchemy 2.x ORM 위에서 Pydantic Record ↔
ORM 변환을 정확히 하는지 (`orm_to_run` / `orm_to_result` / `orm_to_capture`
boundary mapper), 그리고 atomic transaction (commit/activate/finalize) 가
semantics 보존하는지.

`:memory:` SQLite engine — 매 테스트 fresh DB.
"""

from __future__ import annotations

from typing import Iterator

import pytest

from modules.calibration.persistence_models import (
    CalibrationCaptureRecord,
    CalibrationKind,
    CalibrationRunRecord,
    HandEyeResultRecord,
    IntrinsicResultRecord,
)
from modules.calibration.result_models import (
    HandEyeResultData,
    IntrinsicResultData,
)
from modules.storage.rdb.base import Base, make_engine
from modules.storage.rdb.store import RdbStore, RepoBundle


@pytest.fixture
def store() -> Iterator[RepoBundle]:
    """한 transaction 안 RepoBundle — production storage_node 의 session() 과 동일."""
    engine = make_engine("sqlite:///:memory:")
    Base.metadata.create_all(engine)
    s = RdbStore(engine)
    try:
        with s.session() as r:
            yield r
    finally:
        s.close()


def _make_run(robot_id: str = "test_robot", kind: CalibrationKind | None = "hand_eye") -> CalibrationRunRecord:
    return CalibrationRunRecord(
        robot_id=robot_id,
        started_at=100.0,
        ended_at=110.0,
        algorithm="extended_ba_irls",
        algorithm_params={"max_iter": 50, "huber_c": 1.345},
        kind=kind,
    )


def _make_handeye_result(
    robot_id: str = "test_robot", sigma_rot: float = 0.65
) -> HandEyeResultRecord:
    return HandEyeResultRecord(  # type: ignore[call-arg]
        run_id=0,  # caller 가 채움
        robot_id=robot_id,
        created_at=100.0,
        sigma_rot=sigma_rot,
        sigma_t=7.94,
        result_data=HandEyeResultData(
            R_cam2gripper=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            t_cam2gripper=[[0.05], [0.0], [0.10]],
            method="BA(physical_sag_irls)",
        ),
    )


def _make_intrinsic_result(
    robot_id: str = "test_robot",
) -> IntrinsicResultRecord:
    return IntrinsicResultRecord(  # type: ignore[call-arg]
        run_id=0,
        robot_id=robot_id,
        created_at=100.0,
        result_data=IntrinsicResultData(
            camera_matrix=[[600, 0, 320], [0, 600, 240], [0, 0, 1]],
            dist_coeffs=[[0.0, 0.0, 0.0, 0.0, 0.0]],
            image_size=[640, 480],
        ),
    )


def _make_capture(pose_index: int) -> CalibrationCaptureRecord:
    return CalibrationCaptureRecord(
        run_id=0,
        pose_index=pose_index,
        motor_positions={1: 2048 + pose_index, 2: 2048, 3: 2048, 4: 2048, 5: 2048},
        board_in_cam=[
            [1.0, 0.0, 0.0, 0.01],
            [0.0, 1.0, 0.0, 0.02],
            [0.0, 0.0, 1.0, 0.30],
            [0.0, 0.0, 0.0, 1.0],
        ],
        corners_2d=[[100.0, 100.0], [200.0, 100.0]],
        corner_ids=[0, 1],
        reproj_rms_px=0.5,
        tilt_deg=45.0,
    )


# ─── Round-trip: run + result + captures ──────────────────────


def test_commit_round_trip_handeye(store: RepoBundle):
    """commit_calibration → list_runs → 같은 데이터 그대로."""
    run = _make_run()
    result = _make_handeye_result()
    captures = [_make_capture(i) for i in range(3)]

    run_id, result_ids = store.calibration.commit(run, [result], captures)
    assert run_id > 0
    assert len(result_ids) == 1

    fetched_run = store.calibration.get_run(run_id)
    assert fetched_run is not None
    assert fetched_run.robot_id == "test_robot"
    assert fetched_run.algorithm == "extended_ba_irls"
    # algorithm_params JSON round-trip
    assert fetched_run.algorithm_params == {"max_iter": 50, "huber_c": 1.345}
    assert fetched_run.kind == "hand_eye"

    fetched_result = store.calibration.get_result(result_ids[0])
    assert fetched_result is not None
    assert fetched_result.kind == "hand_eye"
    assert fetched_result.run_id == run_id
    assert fetched_result.is_active is False  # COMMIT 직후 항상 inactive
    assert fetched_result.sigma_rot == pytest.approx(0.65)
    # discriminated union — result_data 가 HandEyeResultData 로 validate 됐는지
    assert fetched_result.result_data.method == "BA(physical_sag_irls)"

    fetched_caps = store.calibration.list_captures(run_id)
    assert len(fetched_caps) == 3
    assert [c.pose_index for c in fetched_caps] == [0, 1, 2]
    # board_in_cam JSON round-trip
    assert fetched_caps[0].board_in_cam is not None
    assert fetched_caps[0].board_in_cam[0][3] == pytest.approx(0.01)


def test_activate_toggles_atomically(store: RepoBundle):
    """ACTIVATE — 같은 (robot_id, kind) 의 직전 active 자동 해제 + 새 활성."""
    # 두 번 commit — 두 result.
    run1_id, [r1_id] = store.calibration.commit(
        _make_run(), [_make_handeye_result(sigma_rot=0.9)], []
    )
    run2_id, [r2_id] = store.calibration.commit(
        _make_run(), [_make_handeye_result(sigma_rot=0.65)], []
    )

    # 둘 다 COMMIT 직후 inactive.
    assert store.calibration.get_active_result("test_robot", "hand_eye") is None

    # r1 활성
    store.calibration.activate_result(r1_id)
    active = store.calibration.get_active_result("test_robot", "hand_eye")
    assert active is not None
    assert active.id == r1_id

    # r2 활성 — r1 자동 해제 + UNIQUE partial index 위반 X.
    store.calibration.activate_result(r2_id)
    active = store.calibration.get_active_result("test_robot", "hand_eye")
    assert active is not None
    assert active.id == r2_id
    # r1 이 inactive 됐는지 명시 확인
    r1_now = store.calibration.get_result(r1_id)
    assert r1_now is not None
    assert r1_now.is_active is False


def test_different_kind_can_both_be_active(store: RepoBundle):
    """active UNIQUE partial index 는 (robot_id, kind) 단위 — kind 가 다르면 동시 active OK."""
    _, [he_id] = store.calibration.commit(
        _make_run(kind="hand_eye"), [_make_handeye_result()], []
    )
    _, [intr_id] = store.calibration.commit(
        _make_run(kind="intrinsic"), [_make_intrinsic_result()], []
    )

    store.calibration.activate_result(he_id)
    store.calibration.activate_result(intr_id)

    assert store.calibration.get_active_result("test_robot", "hand_eye") is not None
    assert store.calibration.get_active_result("test_robot", "intrinsic") is not None


# ─── Draft run flow ───────────────────────────────────────────


def test_draft_run_append_finalize(store: RepoBundle):
    """[캘 시작] → [캡처] x N → [커밋] flow."""
    run = _make_run()
    run_id = store.calibration.new_run(run)
    assert run_id > 0

    # in_progress lookup
    in_prog = store.calibration.get_in_progress_run("test_robot", "hand_eye")
    assert in_prog is not None
    found_run, caps = in_prog
    assert found_run.id == run_id
    assert found_run.status == "in_progress"
    assert caps == []

    # append captures
    cap_ids: list[int] = []
    for i in range(3):
        c = _make_capture(i)
        c_filled = c.model_copy(update={"run_id": run_id})
        cap_ids.append(store.calibration.append_capture(c_filled))

    in_prog = store.calibration.get_in_progress_run("test_robot", "hand_eye")
    assert in_prog is not None
    _, caps = in_prog
    assert [c.pose_index for c in caps] == [0, 1, 2]

    # delete last → pose_index 2 사라짐.
    result = store.calibration.delete_last_capture(run_id)
    assert result is not None
    deleted, _artifacts = result
    assert deleted == 2

    caps = store.calibration.list_captures(run_id)
    assert [c.pose_index for c in caps] == [0, 1]

    # finalize — IRLS BA output 가 capture_residuals 채움.
    residuals: dict[int, tuple[float | None, float | None, float | None]] = {
        0: (0.01, 0.005, 1.0),
        1: (0.5, 0.20, 0.118),  # outlier
    }
    result = _make_handeye_result()
    result_ids = store.calibration.finalize_run(run_id, [result], residuals)
    assert len(result_ids) == 1

    # status flip 확인 + capture residuals UPDATE 확인
    fin_run = store.calibration.get_run(run_id)
    assert fin_run is not None
    assert fin_run.status == "success"

    fin_caps = store.calibration.list_captures(run_id)
    assert fin_caps[0].weight == pytest.approx(1.0)
    assert fin_caps[1].weight == pytest.approx(0.118)


def test_delete_run_cascades_to_captures_and_results(store: RepoBundle):
    """delete_calibration_run — FK CASCADE 가 자식 captures / results 자동 삭제."""
    run = _make_run()
    captures = [_make_capture(i) for i in range(2)]
    run_id, result_ids = store.calibration.commit(
        run, [_make_handeye_result()], captures
    )

    store.calibration.delete_run(run_id)

    assert store.calibration.get_run(run_id) is None
    assert store.calibration.list_captures(run_id) == []
    assert store.calibration.get_result(result_ids[0]) is None


def test_list_runs_returns_run_with_results(store: RepoBundle):
    """list_runs — Run + 그 Run 의 모든 Result tuple. N+1 query 회피 검증."""
    for _ in range(3):
        store.calibration.commit(
            _make_run(), [_make_handeye_result()], [_make_capture(0)]
        )

    rows = store.calibration.list_runs("test_robot", limit=10)
    assert len(rows) == 3
    # 각 row = (Run, [Result, ...])
    for run, results in rows:
        assert run.robot_id == "test_robot"
        assert len(results) == 1
        assert results[0].run_id == run.id


def test_activate_unknown_result_raises(store: RepoBundle):
    with pytest.raises(KeyError):
        store.calibration.activate_result(99999)


def test_finalize_already_finalized_raises(store: RepoBundle):
    run_id = store.calibration.new_run(_make_run())
    store.calibration.finalize_run(run_id, [_make_handeye_result()], None)
    with pytest.raises(KeyError):
        store.calibration.finalize_run(run_id, [_make_handeye_result()], None)
