"""CalibrationModule (@service wire) 검증 — in-process, hardware 불요.

DB-backed 서비스가 Repository 로 올바로 배선되고 ACTIVATED/COMMITTED 이벤트를 내는지.
capture/preview/factory-intrinsic 은 다음 slice (camera 결합) 라 여기 없음.
"""

from __future__ import annotations

import asyncio
from datetime import UTC, datetime
from pathlib import Path

import cv2
import numpy as np
from pydantic import BaseModel

from framework.runtime.discovery import discover_services
from infra.database.sqlite import open_sqlite
from infra.object_store.filesystem import FilesystemObjectStore
from modules.calibration.vision import sim_board
from modules.calibration.contract import (
    ActivateResultRequest,
    Calibration,
    CalibrationActivated,
    CalibrationCommitted,
    CaptureRequest,
    FinalizeRunRequest,
    HandEyeResultData,
    HandEyeResultRecord,
    IntrinsicResultData,
    IntrinsicResultRecord,
    ListResultsRequest,
    ListRunsRequest,
    PreviewEnableRequest,
    SnapshotBundleRequest,
    StartRunRequest,
    UndoLastCaptureRequest,
)
from modules.calibration.persistence.orm import Base
from modules.calibration.module import CalibrationModule
from modules.calibration.persistence.repository import CalibrationRepository
from modules.calibration.vision.se3 import make_T
from modules.camera.contract import CameraDecodedFrame
from modules.motor.contract import JointState

_ROBOT = "so101_6dof_0"
_MOTOR_IDS = [1, 2, 3, 4, 5, 6]
_W, _H = 1280, 720
_CM = [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
_DIST = [[0.0, 0.0, 0.0, 0.0, 0.0]]


class _FakeRuntime:
    """publish 를 캡처하는 최소 ModuleRuntime."""

    def __init__(self) -> None:
        self.events: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.events.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):  # noqa: ANN001,ANN002
        raise AssertionError("이 slice 에서 call 안 씀")


def _module(
    tmp_path: Path,
) -> tuple[CalibrationModule, _FakeRuntime, CalibrationRepository]:
    engine, factory = open_sqlite(tmp_path / "m.db")
    Base.metadata.create_all(engine)
    repo = CalibrationRepository(factory)
    rt = _FakeRuntime()
    store = FilesystemObjectStore(tmp_path / "blobs")
    mod = CalibrationModule(
        runtime=rt,
        robot_id=_ROBOT,
        repository=repo,
        object_store=store,
        motor_ids=_MOTOR_IDS,
    )
    return mod, rt, repo


def _seed_intrinsic(repo: CalibrationRepository) -> None:
    run = repo.create_run(_ROBOT, "intrinsic", "test")
    assert run.id is not None
    rid = repo.save_result(
        run.id,
        IntrinsicResultRecord(
            run_id=run.id,
            robot_id=_ROBOT,
            created_at=datetime.now(UTC),
            result_data=IntrinsicResultData(
                camera_matrix=_CM, dist_coeffs=_DIST, image_size=[_W, _H]
            ),
        ),
    )
    repo.activate_result(rid)


def _board_frame(rvec: list[float], tvec: list[float]) -> CameraDecodedFrame:
    """rvec/tvec 포즈의 sim ChArUco 보드를 렌더한 decoded frame."""
    bic = make_T(cv2.Rodrigues(np.array(rvec))[0], np.array(tvec))
    img = sim_board.render_charuco_at_pose(
        width=_W,
        height=_H,
        camera_matrix=np.array(_CM),
        dist_coeffs=np.array(_DIST),
        board_in_cam=bic,
    )
    return CameraDecodedFrame(
        robot_id=_ROBOT,
        seq=0,
        timestamp_unix=0.0,
        ndarray_bytes=img.tobytes(),
        width=_W,
        height=_H,
    )


def _feed(mod: CalibrationModule, frame: CameraDecodedFrame, raw: list[int]) -> None:
    mod.on_decoded_frame(frame)
    mod.on_motor_raw(
        JointState(robot_id=_ROBOT, seq=0, timestamp_unix=0.0, positions_raw=raw)
    )


def _he(run_id: int) -> HandEyeResultRecord:
    return HandEyeResultRecord(
        run_id=run_id,
        robot_id=_ROBOT,
        created_at=datetime.now(UTC),
        result_data=HandEyeResultData(
            R_cam2gripper=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            t_cam2gripper=[[0], [0], [0]],
            method="BA",
        ),
    )


def test_service_wiring_discovers_all_keys(tmp_path: Path):
    mod, _, _ = _module(tmp_path)
    keys = {spec.wire_key for _m, spec in discover_services(mod)}
    assert keys == {
        Calibration.Service.START_RUN,
        Calibration.Service.CAPTURE,
        Calibration.Service.FINALIZE_RUN,
        Calibration.Service.ACTIVATE_RESULT,
        Calibration.Service.UNDO_LAST_CAPTURE,
        Calibration.Service.PREVIEW_ENABLE,
        Calibration.Service.SNAPSHOT_BUNDLE,
        Calibration.Service.LIST_RUNS,
        Calibration.Service.LIST_RESULTS,
        Calibration.Service.GET_THRESHOLDS,
    }


def test_start_run_creates_in_progress(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    res = mod.start_run(StartRunRequest(kind="hand_eye", algorithm="hand_eye_capture_only"))
    run = repo.get_run(res.run_id)
    assert run is not None and run.status == "in_progress" and run.kind == "hand_eye"


def test_activate_result_publishes_activated_event(tmp_path: Path):
    mod, rt, repo = _module(tmp_path)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None
    rid = repo.save_result(run.id, _he(run.id))

    res = mod.activate_result(ActivateResultRequest(result_id=rid))
    assert res.ok
    assert repo.get_active(_ROBOT, "hand_eye").id == rid  # type: ignore[union-attr]
    # ACTIVATED 이벤트 (kind 포함) — "재시작 필요" 알림
    published = [e for k, e in rt.events if k == Calibration.Event.ACTIVATED]
    assert len(published) == 1
    ev = published[0]
    assert isinstance(ev, CalibrationActivated)
    assert ev.robot_id == _ROBOT and ev.result_id == rid and ev.kind == "hand_eye"


def test_finalize_run_publishes_committed_and_sets_status(tmp_path: Path):
    mod, rt, repo = _module(tmp_path)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None

    res = mod.finalize_run(FinalizeRunRequest(run_id=run.id))
    assert res.ok
    assert repo.get_run(run.id).status == "ready_for_analysis"  # type: ignore[union-attr]
    committed = [e for k, e in rt.events if k == Calibration.Event.COMMITTED]
    assert len(committed) == 1
    assert isinstance(committed[0], CalibrationCommitted)
    assert committed[0].run_id == run.id


def test_snapshot_bundle_returns_active(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None
    rid = repo.save_result(run.id, _he(run.id))
    repo.activate_result(rid)

    bundle = mod.snapshot_bundle(SnapshotBundleRequest())
    assert bundle.robot_id == _ROBOT
    assert bundle.hand_eye is not None and bundle.hand_eye.id == rid


def test_list_runs_and_results_and_undo(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None
    repo.save_result(run.id, _he(run.id))
    from modules.calibration.contract import CalibrationCaptureRecord

    repo.append_capture(run.id, CalibrationCaptureRecord(run_id=run.id, pose_index=0))
    repo.append_capture(run.id, CalibrationCaptureRecord(run_id=run.id, pose_index=1))

    assert len(mod.list_runs(ListRunsRequest()).runs) == 1
    assert len(mod.list_results(ListResultsRequest(kind="hand_eye")).results) == 1

    assert mod.undo_last_capture(UndoLastCaptureRequest(run_id=run.id)).ok
    assert [c.pose_index for c in repo.list_captures(run.id)] == [0]


# ─────────────────────────── capture (sim board) ───────────────────────────


def test_capture_detects_pnp_stores_row_and_blob(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    run = repo.create_run(_ROBOT, "hand_eye", "hand_eye_capture_only")
    assert run.id is not None
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2041, 2342, 903, 2846, 2120, 3122])

    res = mod.capture(CaptureRequest(run_id=run.id, pose_index=0))
    assert res.accepted, res.message
    assert res.capture_id is not None
    assert res.reproj_rms_px is not None and res.reproj_rms_px < 1.5
    assert res.tilt_deg is not None and res.tilt_deg > 0
    assert res.quality is not None and res.quality.verdict == "green"  # 첫 자세

    caps = repo.list_captures(run.id)
    assert len(caps) == 1
    c = caps[0]
    assert c.motor_positions == {1: 2041, 2: 2342, 3: 903, 4: 2846, 5: 2120, 6: 3122}
    assert c.board_in_cam is not None and len(c.board_in_cam) == 4
    assert c.corner_ids and len(c.corner_ids) >= 12
    # color blob 저장됨 (artifact row + object store)
    art = c.find_artifact("color")
    assert art is not None and art.blob_key.endswith("000_color.jpg")


def test_capture_rejects_when_no_board(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None
    # 빈(보드 없는) frame
    blank = CameraDecodedFrame(
        robot_id=_ROBOT,
        seq=0,
        timestamp_unix=0.0,
        ndarray_bytes=np.full((_H, _W, 3), 50, dtype=np.uint8).tobytes(),
        width=_W,
        height=_H,
    )
    _feed(mod, blank, [2048] * 6)

    res = mod.capture(CaptureRequest(run_id=run.id, pose_index=0))
    assert not res.accepted
    assert res.quality is not None and res.quality.verdict == "red"
    assert repo.list_captures(run.id) == []  # 저장 안 됨 (gate 입구 차단)


def test_capture_rejects_when_no_intrinsic(tmp_path: Path):
    mod, _, repo = _module(tmp_path)  # intrinsic seed 안 함
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2048] * 6)

    res = mod.capture(CaptureRequest(run_id=run.id, pose_index=0))
    assert not res.accepted
    assert "intrinsic" in res.message


def test_capture_second_pose_quality_diversity(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    run = repo.create_run(_ROBOT, "hand_eye", "x")
    assert run.id is not None
    # 1st capture
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2041, 2342, 903, 2846, 2120, 3122])
    assert mod.capture(CaptureRequest(run_id=run.id, pose_index=0)).accepted
    # 거의 같은 자세 2nd → red (기존과 거의 동일)
    _feed(mod, _board_frame([0.31, 0.2, 0.0], [0.0, 0.0, 0.35]), [2043, 2344, 905, 2848, 2122, 3124])
    res = mod.capture(CaptureRequest(run_id=run.id, pose_index=1))
    assert res.accepted  # gate 는 통과 (검출 OK)
    assert res.quality is not None and res.quality.verdict in ("red", "yellow")


# ─────────────────────────── preview stream ───────────────────────────


async def test_preview_publishes_detection_when_enabled(tmp_path: Path):
    mod, rt, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    repo.create_run(_ROBOT, "hand_eye", "x")  # in-progress run (quality 비교 대상)
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2041, 2342, 903, 2846, 2120, 3122])

    await mod.start()
    try:
        mod.preview_enable(PreviewEnableRequest(enabled=True))
        previews: list = []
        for _ in range(30):  # ~1.5s 까지 대기 (5Hz)
            await asyncio.sleep(0.05)
            previews = [e for k, e in rt.events if k == Calibration.Stream.PREVIEW]
            if previews:
                break
    finally:
        await mod.stop()

    assert previews, "preview stream 이 발행 안 됨"
    p = previews[-1]
    assert p.detected and p.corner_count >= 12
    assert p.tilt_deg is not None and p.tilt_deg > 0
    assert p.verdict in ("green", "yellow", "red")


async def test_preview_silent_when_disabled(tmp_path: Path):
    mod, rt, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2048] * 6)
    await mod.start()
    try:
        await asyncio.sleep(0.3)  # preview_enable 안 함 → 조용
    finally:
        await mod.stop()
    assert not [e for k, e in rt.events if k == Calibration.Stream.PREVIEW]
