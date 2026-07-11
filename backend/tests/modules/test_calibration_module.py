"""CalibrationModule (@service wire) 검증 — in-process, hardware 불요.

robot-agnostic (host 당 1) — 단일 인스턴스가 req.robot_id / run 파생으로 dispatch.
DB-backed 서비스가 Repository 로 올바로 배선되고 ACTIVATED/COMMITTED 이벤트를 내는지
+ **multi-robot 눈속임 방지** (backend.md §2.7.3): so101(6DOF) 과
omx(5DOF) 를 같은 인스턴스로 구동 — 한쪽 하드코딩 잔재는 다른쪽 경로에서 터짐.
"""

from __future__ import annotations

import asyncio
from datetime import UTC, datetime
from pathlib import Path

import cv2
import numpy as np
import pytest
from pydantic import BaseModel

from framework.runtime.discovery import discover_services
from infra.database.sqlite import open_sqlite
from infra.object_store.filesystem import FilesystemObjectStore
from modules.calibration.vision import sim_board
from modules.calibration.contract import (
    AbortRunRequest,
    ActivateResultRequest,
    Calibration,
    CalibrationActivated,
    CalibrationCommitted,
    CalibrationPreview,
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
from modules.calibration.module import CalibrationModule, CalibrationRobotSpec
from modules.calibration.persistence.repository import CalibrationRepository
from modules.calibration.vision.se3 import make_T
from modules.camera.contract import CameraDecodedFrame
from modules.motor.contract import JointState

_SO101 = "so101_6dof_0"
_OMX = "omx_f_0"
# motors.yaml SSOT — so101 arm 6개(1..6), omx_f arm 5개(1..5, 6=gripper 제외)
_ROBOTS = {
    _SO101: CalibrationRobotSpec(motor_ids=[1, 2, 3, 4, 5, 6], has_camera=True),
    _OMX: CalibrationRobotSpec(motor_ids=[1, 2, 3, 4, 5], has_camera=True),
}
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
        repository=repo,
        object_store=store,
        robots=dict(_ROBOTS),
    )
    return mod, rt, repo


def _seed_intrinsic(
    repo: CalibrationRepository, robot_id: str = _SO101, fx: float = 900.0
) -> None:
    cm = [[fx, 0.0, 640.0], [0.0, fx, 360.0], [0.0, 0.0, 1.0]]
    run = repo.create_run(robot_id, "intrinsic", "test")
    assert run.id is not None
    rid = repo.save_result(
        run.id,
        IntrinsicResultRecord(
            run_id=run.id,
            robot_id=robot_id,
            created_at=datetime.now(UTC),
            result_data=IntrinsicResultData(
                camera_matrix=cm, dist_coeffs=_DIST, image_size=[_W, _H]
            ),
        ),
    )
    # 실제 seed(_seed_factory_intrinsic)/finalize 흐름과 동일하게 run 을 닫는다 —
    # in_progress 로 남기면 preview 가 intrinsic 세션 중으로 (올바르게) 판정
    repo.finalize_run(run.id, "success")
    repo.activate_result(rid)


def _board_frame(
    rvec: list[float], tvec: list[float], robot_id: str = _SO101, fx: float = 900.0
) -> CameraDecodedFrame:
    """rvec/tvec 포즈의 sim ChArUco 보드를 렌더한 decoded frame."""
    cm = np.array([[fx, 0.0, 640.0], [0.0, fx, 360.0], [0.0, 0.0, 1.0]])
    bic = make_T(cv2.Rodrigues(np.array(rvec))[0], np.array(tvec))
    img = sim_board.render_charuco_at_pose(
        width=_W,
        height=_H,
        camera_matrix=cm,
        dist_coeffs=np.array(_DIST),
        board_in_cam=bic,
    )
    return CameraDecodedFrame(
        robot_id=robot_id,
        seq=0,
        timestamp_unix=0.0,
        ndarray_bytes=img.tobytes(),
        width=_W,
        height=_H,
    )


def _feed(
    mod: CalibrationModule,
    frame: CameraDecodedFrame,
    raw: list[int],
    robot_id: str = _SO101,
) -> None:
    mod.on_decoded_frame(frame)
    mod.on_motor_raw(
        JointState(robot_id=robot_id, seq=0, timestamp_unix=0.0, positions_raw=raw)
    )


def _he(run_id: int, robot_id: str = _SO101) -> HandEyeResultRecord:
    return HandEyeResultRecord(
        run_id=run_id,
        robot_id=robot_id,
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
        Calibration.Service.ABORT_RUN,
        Calibration.Service.ACTIVATE_RESULT,
        Calibration.Service.UNDO_LAST_CAPTURE,
        Calibration.Service.PREVIEW_ENABLE,
        Calibration.Service.SNAPSHOT_BUNDLE,
        Calibration.Service.LIST_RUNS,
        Calibration.Service.LIST_RESULTS,
        Calibration.Service.GET_THRESHOLDS,
    }
    # robot-agnostic — 서비스 키에 {robot_id} placeholder 없음 (§2.7.3 acceptance 1)
    assert all("{robot_id}" not in k for k in keys)


def test_module_is_host_level_no_robot_id_attr(tmp_path: Path):
    # §2.7.3 acceptance 1 — host-level 인스턴스는 self.robot_id 미보유
    mod, _, _ = _module(tmp_path)
    assert not hasattr(mod, "robot_id")


def test_start_run_creates_in_progress(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    res = mod.start_run(
        StartRunRequest(
            robot_id=_SO101, kind="hand_eye", algorithm="hand_eye_capture_only"
        )
    )
    run = repo.get_run(res.run_id)
    assert run is not None and run.status == "in_progress" and run.kind == "hand_eye"
    assert run.robot_id == _SO101


def test_activate_result_publishes_activated_event(tmp_path: Path):
    mod, rt, repo = _module(tmp_path)
    run = repo.create_run(_SO101, "hand_eye", "x")
    assert run.id is not None
    rid = repo.save_result(run.id, _he(run.id))

    res = mod.activate_result(ActivateResultRequest(result_id=rid))
    assert res.ok
    assert repo.get_active(_SO101, "hand_eye").id == rid  # type: ignore[union-attr]
    # ACTIVATED 이벤트 (kind 포함) — robot_id 는 result row 에서 파생
    published = [e for k, e in rt.events if k == Calibration.Event.ACTIVATED]
    assert len(published) == 1
    ev = published[0]
    assert isinstance(ev, CalibrationActivated)
    assert ev.robot_id == _SO101 and ev.result_id == rid and ev.kind == "hand_eye"


def test_finalize_run_publishes_committed_and_sets_status(tmp_path: Path):
    mod, rt, repo = _module(tmp_path)
    run = repo.create_run(_SO101, "hand_eye", "x")
    assert run.id is not None

    res = mod.finalize_run(FinalizeRunRequest(run_id=run.id))
    assert res.ok
    assert repo.get_run(run.id).status == "ready_for_analysis"  # type: ignore[union-attr]
    committed = [e for k, e in rt.events if k == Calibration.Event.COMMITTED]
    assert len(committed) == 1
    assert isinstance(committed[0], CalibrationCommitted)
    # robot_id 는 run row 에서 파생 (req 에 없음)
    assert committed[0].run_id == run.id and committed[0].robot_id == _SO101


def test_abort_run_marks_failed_and_rejects_double_abort(tmp_path: Path):
    """세션 탈출구 계약 — 0장 세션에서 finalize(캡처부족 거부)+undo(비활성)로
    갇히던 결함의 회귀망. abort = in_progress → failed, 이미 종료된 run 은 ok=False."""
    mod, _, repo = _module(tmp_path)
    run = repo.create_run(_SO101, "intrinsic", "charuco_manual")
    assert run.id is not None

    res = mod.abort_run(AbortRunRequest(run_id=run.id))
    assert res.ok
    assert repo.get_run(run.id).status == "failed"  # type: ignore[union-attr]

    res2 = mod.abort_run(AbortRunRequest(run_id=run.id))
    assert not res2.ok and "failed" in res2.message


def test_start_run_aborts_stale_in_progress_same_robot(tmp_path: Path):
    """도메인 규칙: robot 당 활성 세션 1개 — reload 로 UI 가 runId 를 잃어도
    새 시작이 orphan in_progress 를 failed 로 정리 (preview 오판 방지)."""
    mod, _, repo = _module(tmp_path)
    stale = repo.create_run(_SO101, "intrinsic", "charuco_manual")
    other_robot = repo.create_run(_OMX, "intrinsic", "charuco_manual")
    assert stale.id is not None and other_robot.id is not None

    res = mod.start_run(
        StartRunRequest(
            robot_id=_SO101, kind="hand_eye", algorithm="hand_eye_capture_only"
        )
    )

    assert repo.get_run(stale.id).status == "failed"  # type: ignore[union-attr]
    assert repo.get_run(res.run_id).status == "in_progress"  # type: ignore[union-attr]
    # 다른 robot 의 세션은 건드리지 않음
    assert repo.get_run(other_robot.id).status == "in_progress"  # type: ignore[union-attr]


def test_snapshot_bundle_returns_active(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    run = repo.create_run(_SO101, "hand_eye", "x")
    assert run.id is not None
    rid = repo.save_result(run.id, _he(run.id))
    repo.activate_result(rid)

    bundle = mod.snapshot_bundle(SnapshotBundleRequest(robot_id=_SO101))
    assert bundle.robot_id == _SO101
    assert bundle.hand_eye is not None and bundle.hand_eye.id == rid


def test_list_runs_and_results_and_undo(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    run = repo.create_run(_SO101, "hand_eye", "x")
    assert run.id is not None
    repo.save_result(run.id, _he(run.id))
    from modules.calibration.contract import CalibrationCaptureRecord

    repo.append_capture(run.id, CalibrationCaptureRecord(run_id=run.id, pose_index=0))
    repo.append_capture(run.id, CalibrationCaptureRecord(run_id=run.id, pose_index=1))

    assert len(mod.list_runs(ListRunsRequest(robot_id=_SO101)).runs) == 1
    assert (
        len(
            mod.list_results(
                ListResultsRequest(robot_id=_SO101, kind="hand_eye")
            ).results
        )
        == 1
    )

    assert mod.undo_last_capture(UndoLastCaptureRequest(run_id=run.id)).ok
    assert [c.pose_index for c in repo.list_captures(run.id)] == [0]


# ─────────────────────────── preview overlay (sim board) ────────────────────


def _last_preview(rt: _FakeRuntime) -> CalibrationPreview:
    previews = [e for k, e in rt.events if k == Calibration.Stream.PREVIEW]
    assert previews, "PREVIEW 이벤트 없음"
    pv = previews[-1]
    assert isinstance(pv, CalibrationPreview)
    return pv


def test_preview_emits_corners_and_image_dims_with_intrinsic(tmp_path: Path):
    # intrinsic 있으면 PnP 경로 — corners_2d(overlay 좌표) + tilt + 원본 크기 발행.
    # frontend ChArUcoOverlay 가 이 좌표를 camera/stream 위에 그림.
    mod, rt, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2048] * 6)

    mod._publish_preview(_SO101)
    pv = _last_preview(rt)

    assert pv.detected is True
    assert pv.corner_count > 0
    assert len(pv.corners_2d) == pv.corner_count  # 개수 정합
    assert all(len(c) == 2 for c in pv.corners_2d)  # (N,2) 픽셀
    # 좌표는 원본 프레임 안 (overlay 스케일 기준)
    assert all(0 <= x <= _W and 0 <= y <= _H for x, y in pv.corners_2d)
    assert pv.image_width == _W and pv.image_height == _H
    assert pv.tilt_deg is not None and pv.tilt_deg > 0


def test_preview_emits_corners_without_intrinsic(tmp_path: Path):
    # intrinsic 없어도(=PnP 불가) overlay 용 raw ChArUco 코너는 채운다 —
    # 사용자가 자세 잡을 때 "보드가 잡히나"를 봐야 하므로 (tilt 만 None).
    mod, rt, _ = _module(tmp_path)
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2048] * 6)

    mod._publish_preview(_SO101)
    pv = _last_preview(rt)

    assert pv.detected is True
    assert len(pv.corners_2d) == pv.corner_count > 0
    assert pv.image_width == _W and pv.image_height == _H
    assert pv.tilt_deg is None  # PnP 없어 tilt 미산출


# ─────────────────────────── cross 세션 (공유 보드 크로스캘) ────────────────


def test_cross_run_capture_pnp_and_finalize_ready(tmp_path: Path):
    """cross run = hand_eye 와 동일한 PnP 캡처 경로 (board_in_cam + motor_positions
    저장) + finalize 는 ready_for_analysis — offline cross_calibrate.py 입력."""
    mod, _, repo = _module(tmp_path)
    _seed_intrinsic(repo, robot_id=_OMX)
    run_id = mod.start_run(
        StartRunRequest(robot_id=_OMX, kind="cross", algorithm="cross_capture_only")
    ).run_id

    _feed(
        mod,
        _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], robot_id=_OMX),
        [2041, 2342, 903, 2846, 2120],
        robot_id=_OMX,
    )
    res = mod.capture(CaptureRequest(run_id=run_id, pose_index=0))
    assert res.accepted, res.message
    c = repo.list_captures(run_id)[0]
    assert c.board_in_cam is not None  # PnP 경로 (intrinsic detect-only 아님)
    assert c.motor_positions is not None  # FK 용 joint 저장

    fin = mod.finalize_run(FinalizeRunRequest(run_id=run_id))
    assert fin.ok
    assert repo.get_run(run_id).status == "ready_for_analysis"  # type: ignore[union-attr]


def test_cross_session_preview_uses_diversity_not_first_pose_green(tmp_path: Path):
    """preview 의 PnP 세션 탐색이 hand_eye 하드코드면 cross 세션 중 상시
    green("첫 자세") — intrinsic 재캘 사고와 같은 클래스의 회귀 차단."""
    mod, rt, repo = _module(tmp_path)
    _seed_intrinsic(repo, robot_id=_OMX)
    run_id = mod.start_run(
        StartRunRequest(robot_id=_OMX, kind="cross", algorithm="cross_capture_only")
    ).run_id
    _feed(
        mod,
        _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], robot_id=_OMX),
        [2048] * 5,
        robot_id=_OMX,
    )
    assert mod.capture(CaptureRequest(run_id=run_id, pose_index=0)).accepted

    # 캡처 직후 같은 자세/같은 뷰 → 다양성 판정이 "첫 자세" green 이면 안 됨
    mod._publish_preview(_OMX)
    pv = _last_preview(rt)
    assert pv.verdict in ("red", "yellow"), (pv.verdict, pv.reasons)
    assert "첫 자세" not in " ".join(pv.reasons)


def test_start_run_rejects_non_session_kind(tmp_path: Path):
    """joint/link/sag 는 offline BA 산출물 — 캡처 세션 생성 거부."""
    mod, _, _ = _module(tmp_path)
    with pytest.raises(ValueError, match="캡처 세션"):
        mod.start_run(
            StartRunRequest(robot_id=_OMX, kind="joint_offset", algorithm="x")
        )


def test_start_run_aborts_stale_cross_run(tmp_path: Path):
    """cross 세션도 stale cleanup 대상 — 새 start_run 이 orphan cross 를 정리."""
    mod, _, repo = _module(tmp_path)
    stale_id = mod.start_run(
        StartRunRequest(robot_id=_OMX, kind="cross", algorithm="x")
    ).run_id
    mod.start_run(StartRunRequest(robot_id=_OMX, kind="intrinsic", algorithm="y"))
    assert repo.get_run(stale_id).status == "failed"  # type: ignore[union-attr]


# ─────────────────────────── capture (sim board) ───────────────────────────


def test_capture_detects_pnp_stores_row_and_blob(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    run = repo.create_run(_SO101, "hand_eye", "hand_eye_capture_only")
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
    # color blob 저장됨 (artifact row + object store) — 키가 run 소유 robot 경로
    art = c.find_artifact("color")
    assert art is not None and art.blob_key == f"calib_captures/{_SO101}/{run.id}/000_color.jpg"


def test_capture_rejects_when_no_board(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    run = repo.create_run(_SO101, "hand_eye", "x")
    assert run.id is not None
    # 빈(보드 없는) frame
    blank = CameraDecodedFrame(
        robot_id=_SO101,
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
    run = repo.create_run(_SO101, "hand_eye", "x")
    assert run.id is not None
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2048] * 6)

    res = mod.capture(CaptureRequest(run_id=run.id, pose_index=0))
    assert not res.accepted
    assert "intrinsic" in res.message


def test_capture_second_pose_quality_diversity(tmp_path: Path):
    mod, _, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    run = repo.create_run(_SO101, "hand_eye", "x")
    assert run.id is not None
    # 1st capture
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2041, 2342, 903, 2846, 2120, 3122])
    assert mod.capture(CaptureRequest(run_id=run.id, pose_index=0)).accepted
    # 거의 같은 자세 2nd → red (기존과 거의 동일)
    _feed(mod, _board_frame([0.31, 0.2, 0.0], [0.0, 0.0, 0.35]), [2043, 2344, 905, 2848, 2122, 3124])
    res = mod.capture(CaptureRequest(run_id=run.id, pose_index=1))
    assert res.accepted  # gate 는 통과 (검출 OK)
    assert res.quality is not None and res.quality.verdict in ("red", "yellow")


# ──────────────── multi-robot 눈속임 방지 (§2.7.3 acceptance) ────────────────


def test_single_instance_serves_so101_and_omx_isolated(tmp_path: Path):
    """★ 리트머스 — 한 host-level 인스턴스가 6DOF so101 + 5DOF omx 동시 구동.

    각 robot 의 세션이 자기 frame/raw/intrinsic 만 보고 서로 안 새는지: so101
    하드코딩 잔재가 있으면 omx 경로(5 모터, 다른 보드 자세, 다른 fx)가 깨져 잡힘.
    """
    mod, _, repo = _module(tmp_path)
    _seed_intrinsic(repo, _SO101, fx=900.0)
    _seed_intrinsic(repo, _OMX, fx=700.0)  # 다른 카메라 (fx 로 구분)

    run_so = mod.start_run(
        StartRunRequest(robot_id=_SO101, kind="hand_eye", algorithm="x")
    ).run_id
    run_omx = mod.start_run(
        StartRunRequest(robot_id=_OMX, kind="hand_eye", algorithm="x")
    ).run_id

    # 두 robot 의 frame + raw 를 **둘 다** 캐시에 넣은 뒤 (interleave) 각각 capture —
    # 상대 frame 이 섞이면 board z / motor 개수가 어긋나 fail.
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], _SO101, fx=900.0),
          [2041, 2342, 903, 2846, 2120, 3122], _SO101)
    _feed(mod, _board_frame([-0.2, 0.3, 0.1], [0.05, 0.0, 0.55], _OMX, fx=700.0),
          [2100, 2200, 2300, 2400, 2500], _OMX)

    res_so = mod.capture(CaptureRequest(run_id=run_so, pose_index=0))
    res_omx = mod.capture(CaptureRequest(run_id=run_omx, pose_index=0))
    assert res_so.accepted, res_so.message
    assert res_omx.accepted, res_omx.message

    cap_so = repo.list_captures(run_so)[0]
    cap_omx = repo.list_captures(run_omx)[0]
    # motor raw — DOF 와 값이 각 robot 의 것 (6 vs 5)
    assert cap_so.motor_positions == {1: 2041, 2: 2342, 3: 903, 4: 2846, 5: 2120, 6: 3122}
    assert cap_omx.motor_positions == {1: 2100, 2: 2200, 3: 2300, 4: 2400, 5: 2500}
    # board 자세 — 각자 자기 frame 의 PnP (z 0.35 vs 0.55)
    assert cap_so.board_in_cam is not None and cap_omx.board_in_cam is not None
    assert abs(cap_so.board_in_cam[2][3] - 0.35) < 0.02
    assert abs(cap_omx.board_in_cam[2][3] - 0.55) < 0.02
    # blob 경로 — robot 별 분리
    assert cap_so.find_artifact("color").blob_key.startswith(f"calib_captures/{_SO101}/")  # type: ignore[union-attr]
    assert cap_omx.find_artifact("color").blob_key.startswith(f"calib_captures/{_OMX}/")  # type: ignore[union-attr]

    # bundle / run 목록 — DB 멀티테넌트 격리
    assert mod.snapshot_bundle(SnapshotBundleRequest(robot_id=_SO101)).intrinsic.result_data.camera_matrix[0][0] == 900.0  # type: ignore[union-attr]
    assert mod.snapshot_bundle(SnapshotBundleRequest(robot_id=_OMX)).intrinsic.result_data.camera_matrix[0][0] == 700.0  # type: ignore[union-attr]
    so_runs = {r.id for r in mod.list_runs(ListRunsRequest(robot_id=_SO101, kind="hand_eye")).runs}
    omx_runs = {r.id for r in mod.list_runs(ListRunsRequest(robot_id=_OMX, kind="hand_eye")).runs}
    assert so_runs == {run_so} and omx_runs == {run_omx}


async def test_preview_per_robot_isolated(tmp_path: Path):
    """preview 도 robot 별 — so101 만 enable 시 omx frame 있어도 so101 만 발행."""
    mod, rt, repo = _module(tmp_path)
    _seed_intrinsic(repo, _SO101, fx=900.0)
    _seed_intrinsic(repo, _OMX, fx=700.0)
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], _SO101),
          [2041, 2342, 903, 2846, 2120, 3122], _SO101)
    _feed(mod, _board_frame([-0.2, 0.3, 0.1], [0.05, 0.0, 0.55], _OMX, fx=700.0),
          [2100, 2200, 2300, 2400, 2500], _OMX)

    await mod.start()
    try:
        mod.preview_enable(PreviewEnableRequest(robot_id=_SO101, enabled=True))
        previews: list = []
        for _ in range(30):  # ~1.5s 까지 대기 (5Hz)
            await asyncio.sleep(0.05)
            previews = [e for k, e in rt.events if k == Calibration.Stream.PREVIEW]
            if previews:
                break
    finally:
        await mod.stop()

    assert previews, "preview stream 이 발행 안 됨"
    assert all(p.robot_id == _SO101 for p in previews)  # omx 발행 X


# ─────────────────────────── preview stream ───────────────────────────


async def test_preview_publishes_detection_when_enabled(tmp_path: Path):
    mod, rt, repo = _module(tmp_path)
    _seed_intrinsic(repo)
    repo.create_run(_SO101, "hand_eye", "x")  # in-progress run (quality 비교 대상)
    _feed(mod, _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35]), [2041, 2342, 903, 2846, 2120, 3122])

    await mod.start()
    try:
        mod.preview_enable(PreviewEnableRequest(robot_id=_SO101, enabled=True))
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


# ─────────────────────── intrinsic 사용자 캘 (UVC 닭-달걀 회피) ──────────────


def test_intrinsic_user_cal_capture_and_finalize(tmp_path: Path):
    """active intrinsic 0 에서 detect-only capture → finalize 가 calibrateCamera
    + 저장 + activate 까지 (USB UVC 사용자 캘 경로). 렌더 fx=900 근사 복원 검증
    — 뒤집으면(코너 매칭/이미지 크기 복원 회귀) fx 가 튀어 잡힘."""
    mod, rt, repo = _module(tmp_path)
    run = repo.create_run(_OMX, "intrinsic", "user_charuco")
    assert run.id is not None

    # 보드를 image plane 여러 영역(3×3 cell) + 다양한 tilt 로 — intrinsic diversity
    poses = [
        ([0.30, 0.15, 0.0], [0.00, 0.00, 0.35]),
        ([0.35, -0.2, 0.1], [-0.12, -0.06, 0.40]),
        ([-0.3, 0.30, 0.0], [0.12, -0.06, 0.40]),
        ([0.25, 0.35, -0.1], [-0.12, 0.06, 0.45]),
        ([0.40, 0.10, 0.1], [0.12, 0.06, 0.45]),
        ([-0.2, -0.3, 0.0], [0.00, 0.08, 0.50]),
    ]
    for i, (rvec, tvec) in enumerate(poses):
        _feed(
            mod, _board_frame(rvec, tvec, robot_id=_OMX), [2048] * 5, robot_id=_OMX
        )
        res = mod.capture(CaptureRequest(run_id=run.id, pose_index=i))
        assert res.accepted, f"pose {i}: {res.message}"
        assert res.reproj_rms_px is None  # detect-only — PnP 안 함
        assert res.quality is not None and res.quality.verdict in ("green", "yellow")

    fin = mod.finalize_run(FinalizeRunRequest(run_id=run.id))
    assert fin.ok, fin.message

    active = repo.get_active(_OMX, "intrinsic")
    assert isinstance(active, IntrinsicResultRecord)
    rd = active.result_data
    assert rd.image_size == [_W, _H]
    assert rd.rms_px is not None and rd.rms_px < 1.0
    # 렌더 카메라 fx=fy=900 근사 복원 (planar 6 views — 5% 이내)
    assert abs(rd.camera_matrix[0][0] - 900.0) / 900.0 < 0.05
    assert abs(rd.camera_matrix[1][1] - 900.0) / 900.0 < 0.05
    assert repo.get_run(run.id).status == "success"  # type: ignore[union-attr]
    keys = [k for k, _ in rt.events]
    assert Calibration.Event.COMMITTED in keys
    assert Calibration.Event.ACTIVATED in keys


def test_preview_intrinsic_recal_session_uses_coverage_not_handeye(tmp_path: Path):
    """재캘 실사고 (2026-07-11): active intrinsic 이 이미 있으면 preview 가 PnP
    성공 → hand-eye 판정으로 새서, intrinsic 세션 중인데 같은 뷰에 상시 green
    ("첫 자세") 이 떴다. intrinsic 세션 중엔 active intrinsic 유무와 무관하게
    coverage 판정이어야 한다 — 같은 cell 재방문은 yellow."""
    mod, rt, repo = _module(tmp_path)
    _seed_intrinsic(repo, robot_id=_OMX)  # 재캘 = active intrinsic 존재
    run = repo.create_run(_OMX, "intrinsic", "user_charuco")
    assert run.id is not None

    _feed(
        mod,
        _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], robot_id=_OMX),
        [2048] * 5,
        robot_id=_OMX,
    )
    assert mod.capture(CaptureRequest(run_id=run.id, pose_index=0)).accepted

    # 캡처 직후 똑같은 카메라 뷰 → 같은 cell = "이미 커버한 영역" yellow
    mod._publish_preview(_OMX)
    pv = _last_preview(rt)
    assert pv.detected is True
    assert pv.verdict == "yellow", (pv.verdict, pv.reasons)
    assert any("이미 커버" in r for r in pv.reasons)


def test_preview_intrinsic_counts_captures_not_cells(tmp_path: Path):
    """preview 의 n_existing 이 커버 cell 수를 세면 (장수 아님) 권장 장수를
    아무리 채워도 "충분 — 종료 가능" green 에 도달 못 한다 — 장수 기준 검증."""
    mod, rt, repo = _module(tmp_path)
    run = repo.create_run(_OMX, "intrinsic", "user_charuco")
    assert run.id is not None

    _feed(
        mod,
        _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], robot_id=_OMX),
        [2048] * 5,
        robot_id=_OMX,
    )
    for i in range(10):  # 같은 cell 에 권장 장수만큼 (cell 은 1개)
        assert mod.capture(CaptureRequest(run_id=run.id, pose_index=i)).accepted

    mod._publish_preview(_OMX)
    pv = _last_preview(rt)
    assert pv.verdict == "green", (pv.verdict, pv.reasons)
    assert any("충분" in r for r in pv.reasons)


def test_preview_verdict_hysteresis_suppresses_single_frame_flap(tmp_path: Path):
    """신호등 널뜀 실사고 (2026-07-11): 보드 고정인데 경계 검출로 프레임마다
    미검출(red)↔OK(green) 가 튀어 캡처 타이밍을 못 잡음. 단일 프레임 dropout 은
    표시 verdict 를 못 바꾸고, 지속(4/5)돼야 전환된다."""
    mod, rt, repo = _module(tmp_path)
    run = repo.create_run(_OMX, "intrinsic", "user_charuco")
    assert run.id is not None
    board = _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], robot_id=_OMX)
    blank = CameraDecodedFrame(
        robot_id=_OMX,
        seq=0,
        timestamp_unix=0.0,
        ndarray_bytes=np.full((_H, _W, 3), 50, dtype=np.uint8).tobytes(),
        width=_W,
        height=_H,
    )

    # 안정 green 확립 (5프레임)
    _feed(mod, board, [2048] * 5, robot_id=_OMX)
    for _ in range(5):
        mod._publish_preview(_OMX)
    assert _last_preview(rt).verdict == "green"

    # 단일 프레임 미검출 — 신호등은 green 유지, overlay(detected)는 raw 정직
    _feed(mod, blank, [2048] * 5, robot_id=_OMX)
    mod._publish_preview(_OMX)
    pv = _last_preview(rt)
    assert pv.detected is False
    assert pv.verdict == "green", (pv.verdict, pv.reasons)

    # 미검출 지속 → red 전환 (4/5 도달)
    for _ in range(3):
        mod._publish_preview(_OMX)
    pv = _last_preview(rt)
    assert pv.verdict == "red", (pv.verdict, pv.reasons)


def test_preview_verdict_resets_after_capture(tmp_path: Path):
    """캡처는 coverage 기준을 바꾸므로 smoothing 이 리셋돼 다음 판정이 즉시 표시 —
    같은 뷰가 hysteresis 지연 없이 바로 yellow (이미 커버) 로 떨어져야 한다."""
    mod, rt, repo = _module(tmp_path)
    run = repo.create_run(_OMX, "intrinsic", "user_charuco")
    assert run.id is not None
    _feed(
        mod,
        _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], robot_id=_OMX),
        [2048] * 5,
        robot_id=_OMX,
    )
    for _ in range(5):  # 캡처 전 green 으로 포화된 window
        mod._publish_preview(_OMX)
    assert _last_preview(rt).verdict == "green"

    assert mod.capture(CaptureRequest(run_id=run.id, pose_index=0)).accepted
    mod._publish_preview(_OMX)  # 같은 뷰 — 리셋 덕에 1프레임 만에 yellow
    pv = _last_preview(rt)
    assert pv.verdict == "yellow", (pv.verdict, pv.reasons)


def test_intrinsic_finalize_insufficient_captures_keeps_run_alive(tmp_path: Path):
    # 최소 장수 미달 finalize → run 을 죽이지 않고 ok=False (더 캡처 후 재시도).
    mod, _, repo = _module(tmp_path)
    run = repo.create_run(_OMX, "intrinsic", "user_charuco")
    assert run.id is not None
    _feed(
        mod,
        _board_frame([0.3, 0.2, 0.0], [0.0, 0.0, 0.35], robot_id=_OMX),
        [2048] * 5,
        robot_id=_OMX,
    )
    assert mod.capture(CaptureRequest(run_id=run.id, pose_index=0)).accepted

    fin = mod.finalize_run(FinalizeRunRequest(run_id=run.id))
    assert not fin.ok and "캡처 부족" in fin.message
    assert repo.get_run(run.id).status == "in_progress"  # type: ignore[union-attr]
