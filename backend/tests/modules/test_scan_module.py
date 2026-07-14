"""ScanModule (@service wire) 검증 — in-process, hardware 불요.

robot-agnostic (host 당 1) — **multi-robot 눈속임 방지**
(backend.md §2.7.3): 6DOF so101 + 5DOF omx 세션이 한 인스턴스
안에서 격리 (arm ids / raw / blob 경로 / scene3d dispatch). 전체 wire + TSDF build
는 test_scan_e2e 가 커버 — 여기선 capture dispatch/격리를 fake 로 직접.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import BaseModel

from framework.runtime.discovery import discover_services
from infra.database.sqlite import open_sqlite
from infra.object_store.filesystem import FilesystemObjectStore
from modules.motor.contract import JointState, MotorKind
from modules.motor.layout import MotorSpec
from modules.scan.contract import (
    CaptureRequest,
    ListSessionsRequest,
    NewSessionRequest,
    Scan,
)
from modules.scan.module import ScanModule, ScanRobotSpec
from modules.scan.persistence.orm import Base
from modules.scan.persistence.repository import ScanRepository
from modules.scene3d.contract import Scene3d, Scene3dIntrinsic, SnapshotResponse

_SO101 = "so101_6dof_0"
_OMX = "omx_f_0"


def _arm(ids: list[int]) -> list[MotorSpec]:
    return [
        MotorSpec(
            id=i,
            name=f"joint{i}",
            model="TEST",
            kind=MotorKind.JOINT,
            home=2048,
            limit_min=0,
            limit_max=4095,
            velocity_dps=60.0,
            acceleration_dpss=300.0,
        )
        for i in ids
    ]


class _FakeKinematics:
    """Kinematics Protocol 충족 fake — capture 경로엔 미사용 (build 는 e2e 가 커버)."""

    def initialize(self) -> None: ...

    def close(self) -> None: ...

    @property
    def dof(self) -> int:
        return 6

    @property
    def tcp_link_name(self) -> str:
        return "tcp"

    def fk(self, joint_angles):  # noqa: ANN001, ANN201
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

    def ik(self, target_position, target_quaternion, current_joint_angles=None, restarts=None):  # noqa: ANN001, ANN201
        return None

    def fk_to_matrix(self, joint_angles):  # noqa: ANN001, ANN201
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], (0.0, 0.0, 0.0)

    def joint_limits(self, n=None):  # noqa: ANN001, ANN201
        return []

    def self_collision(self, joint_angles) -> bool:  # noqa: ANN001
        return False

    def floor_collision(self, joint_angles, floor_z) -> bool:  # noqa: ANN001
        return False

    def set_obstacle_points(self, points) -> None:  # noqa: ANN001
        ...

    def obstacle_collision(self, joint_angles, *, gripper_open=False) -> bool:  # noqa: ANN001
        return False


def _snap(fx: float) -> SnapshotResponse:
    return SnapshotResponse(
        color_jpeg=b"jpg",
        depth_zstd=b"zstd",
        intrinsic=Scene3dIntrinsic(
            width=64, height=48, fx=fx, fy=fx, cx=32.0, cy=24.0, depth_scale=0.001
        ),
        num_frames=3,
        timestamp_unix=0.0,
    )


class _FakeRuntime:
    """(key, robot_id) → canned. scene3d SNAPSHOT req 의 robot_id 필드로 dispatch."""

    def __init__(self, responses: dict) -> None:
        self._responses = responses
        self.events: list[tuple[str, BaseModel]] = []
        self.calls: list[tuple[str, str | None]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.events.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):  # noqa: ANN001,ANN002
        rid = robot_id or getattr(req, "robot_id", None)
        self.calls.append((str(key), rid))
        return self._responses[(str(key), rid)]


def _module(tmp_path: Path) -> tuple[ScanModule, _FakeRuntime, ScanRepository]:
    engine, factory = open_sqlite(tmp_path / "scan.db")
    Base.metadata.create_all(engine)
    repo = ScanRepository(factory)
    rt = _FakeRuntime(
        {
            (str(Scene3d.Service.SNAPSHOT), _SO101): _snap(fx=600.0),
            (str(Scene3d.Service.SNAPSHOT), _OMX): _snap(fx=700.0),
        }
    )
    mod = ScanModule(
        runtime=rt,
        repository=repo,
        object_store=FilesystemObjectStore(tmp_path / "blobs"),
        robots={
            _SO101: ScanRobotSpec(
                kinematics_factory=lambda p: _FakeKinematics(),
                urdf_path=tmp_path / "so101.urdf",
                arm_specs=_arm([1, 2, 3, 4, 5, 6]),
            ),
            _OMX: ScanRobotSpec(
                kinematics_factory=lambda p: _FakeKinematics(),
                urdf_path=tmp_path / "omx.urdf",
                arm_specs=_arm([1, 2, 3, 4, 5]),
            ),
        },
    )
    return mod, rt, repo


def _feed_raw(mod: ScanModule, robot_id: str, raw: list[int]) -> None:
    mod.on_motor_raw(
        JointState(robot_id=robot_id, seq=0, timestamp_unix=0.0, positions_raw=raw)
    )


def test_service_wiring_agnostic_keys(tmp_path: Path):
    mod, _, _ = _module(tmp_path)
    keys = {spec.wire_key for _m, spec in discover_services(mod)}
    assert keys == {
        Scan.Service.NEW_SESSION,
        Scan.Service.LIST_SESSIONS,
        Scan.Service.DELETE_SESSION,
        Scan.Service.CAPTURE,
        Scan.Service.LIST_SCANS,
        Scan.Service.DELETE_SCAN,
        Scan.Service.BUILD,
        Scan.Service.LIST_RECONSTRUCTIONS,
        Scan.Service.GET_MESH,
    }
    # robot-agnostic — 서비스 키에 {robot_id} placeholder 없음 (§2.7.3 acceptance 1)
    assert all("{robot_id}" not in k for k in keys)
    assert not hasattr(mod, "robot_id")


async def test_single_instance_serves_so101_and_omx_isolated(tmp_path: Path):
    """★ 리트머스 — 한 host-level 인스턴스가 6DOF so101 + 5DOF omx capture 격리.

    세션 소유 robot 파생 dispatch: 각 capture 가 자기 robot 의 raw(DOF)/arm ids/
    scene3d snapshot(fx)/blob 경로만 쓰는지.
    """
    mod, rt, repo = _module(tmp_path)
    _feed_raw(mod, _SO101, [2001, 2002, 2003, 2004, 2005, 2006, 9999])  # +gripper
    _feed_raw(mod, _OMX, [3001, 3002, 3003, 3004, 3005, 9999])

    so_sess = mod.new_session(NewSessionRequest(robot_id=_SO101, label="so"))
    omx_sess = mod.new_session(NewSessionRequest(robot_id=_OMX, label="omx"))
    so_sid = so_sess.session.id
    omx_sid = omx_sess.session.id
    assert so_sid is not None and omx_sid is not None

    cap_so = await mod.capture(CaptureRequest(session_row_id=so_sid, num_frames=3))
    cap_omx = await mod.capture(CaptureRequest(session_row_id=omx_sid, num_frames=3))
    assert cap_so.accepted, cap_so.message
    assert cap_omx.accepted, cap_omx.message
    assert cap_so.scan is not None and cap_omx.scan is not None

    # raw / arm ids — DOF 6 vs 5, 각 robot 의 값 (gripper 는 잘림)
    assert cap_so.scan.motor_positions == [2001, 2002, 2003, 2004, 2005, 2006]
    assert cap_so.scan.arm_motor_ids == [1, 2, 3, 4, 5, 6]
    assert cap_omx.scan.motor_positions == [3001, 3002, 3003, 3004, 3005]
    assert cap_omx.scan.arm_motor_ids == [1, 2, 3, 4, 5]
    # scene3d dispatch — 각 세션 소유 robot 으로 호출 + 그 robot 의 intrinsic
    assert (str(Scene3d.Service.SNAPSHOT), _SO101) in rt.calls
    assert (str(Scene3d.Service.SNAPSHOT), _OMX) in rt.calls
    assert cap_so.scan.fx == 600.0 and cap_omx.scan.fx == 700.0
    # blob 경로 robot 별 분리
    assert cap_so.scan.blob_key.startswith(f"scans/{_SO101}/")
    assert cap_omx.scan.blob_key.startswith(f"scans/{_OMX}/")

    # 세션 목록 격리
    so_list = mod.list_sessions(ListSessionsRequest(robot_id=_SO101)).sessions
    omx_list = mod.list_sessions(ListSessionsRequest(robot_id=_OMX)).sessions
    assert [s.id for s in so_list] == [so_sid]
    assert [s.id for s in omx_list] == [omx_sid]


async def test_capture_unknown_fleet_robot_rejected(tmp_path: Path):
    """세션은 있는데 그 robot 이 이 host fleet 에 없음 → 명확히 reject."""
    mod, _, _ = _module(tmp_path)
    ghost = mod.new_session(NewSessionRequest(robot_id="ghost_0"))
    assert ghost.session.id is not None
    res = await mod.capture(CaptureRequest(session_row_id=ghost.session.id))
    assert not res.accepted and "fleet" in res.message


# ─── build 캘 적용 — "raw 저장 + build 시 현재 캘로 재계산" FK 절반 ──
# (2026-07-07: 옛 코드는 plain kinematics + joint_offset 미적용 — anchor 절대
#  배치가 첫 scan FK 오차만큼 틀어지고 ICP 초기값 품질 저하. 공유 빌더로 수정.)


class _RecordingKin:
    """fk_to_matrix 입력 기록 + init/close 추적 — build 캘 적용 수치 검증용."""

    def __init__(self) -> None:
        self.fk_inputs: list[list[float]] = []
        self.initialized = False
        self.closed = False

    def initialize(self) -> None:
        self.initialized = True

    def close(self) -> None:
        self.closed = True

    @property
    def dof(self) -> int:
        return 6

    @property
    def tcp_link_name(self) -> str:
        return "tcp"

    def fk(self, joint_angles):  # noqa: ANN001, ANN201
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

    def ik(self, target_position, target_quaternion, current_joint_angles=None, restarts=None):  # noqa: ANN001, ANN201
        return None

    def fk_to_matrix(self, joint_angles):  # noqa: ANN001, ANN201
        self.fk_inputs.append(list(joint_angles))
        # 고정 EE pose — hand_eye 합성 수치 검증에 사용 (R=I, t=(0.1, 0.2, 0.3))
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], (0.1, 0.2, 0.3)

    def joint_limits(self, n=None):  # noqa: ANN001, ANN201
        return []

    def self_collision(self, joint_angles) -> bool:  # noqa: ANN001
        return False

    def floor_collision(self, joint_angles, floor_z) -> bool:  # noqa: ANN001
        return False

    def set_obstacle_points(self, points) -> None:  # noqa: ANN001
        ...

    def obstacle_collision(self, joint_angles, *, gripper_open=False) -> bool:  # noqa: ANN001
        return False


def _real_blob() -> tuple[bytes, int, int]:
    """scan_blob.decode 가 실제로 풀 수 있는 blob (진짜 JPEG + zstd uint16)."""
    import cv2
    import numpy as np
    import zstandard as zstd

    from modules.scan import blob as scan_blob

    w, h = 64, 48
    ok, jpeg = cv2.imencode(".jpg", np.zeros((h, w, 3), dtype=np.uint8))
    assert ok
    depth = np.full((h, w), 500, dtype=np.uint16)
    depth_zstd = zstd.ZstdCompressor().compress(depth.tobytes())
    return scan_blob.encode(jpeg.tobytes(), depth_zstd), w, h


async def test_build_applies_fresh_calibration_to_fk(tmp_path: Path, monkeypatch):
    """★ build 가 fresh bundle 의 joint_offset(+빌더 경유 link/sag)을 FK 에 적용:
    ① factory 로 kin 을 build 시점 구성 + initialize/close lifecycle
    ② fk 입력 = raw_to_rad + joint_offset (수치)
    ③ t_base_cam_init = t_base_ee · hand_eye (수치)
    joint_offset 빠뜨리면(옛 코드) ② 가 red — 회귀 잠금.
    """
    from datetime import UTC, datetime

    import numpy as np

    from modules.calibration.contract import (
        Calibration,
        CalibrationBundle,
        HandEyeResultData,
        HandEyeResultRecord,
        JointOffsetResultData,
        JointOffsetResultRecord,
    )
    from modules.motion import units
    from modules.scan import build as recon
    from modules.scan.contract import BuildRequest
    from modules.scan.contract import ScanRecord as ScanRow

    arm = _arm([1, 2, 3, 4, 5, 6])
    kins: list[_RecordingKin] = []

    def factory(p: Path) -> _RecordingKin:
        k = _RecordingKin()
        kins.append(k)
        return k

    # bundle — joint_offset(모터 id 3 → +0.113) + hand_eye(R=I, t=(0.01, 0.02, 0.03))
    kw = {"run_id": 1, "robot_id": _SO101, "created_at": datetime.now(UTC)}
    bundle = CalibrationBundle(
        robot_id=_SO101,
        hand_eye=HandEyeResultRecord(
            **kw,
            result_data=HandEyeResultData(
                R_cam2gripper=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                t_cam2gripper=[[0.01], [0.02], [0.03]],
                method="test",
            ),
        ),
        joint_offset=JointOffsetResultRecord(
            **kw,
            result_data=JointOffsetResultData(offsets={3: 0.113}, method="test"),
        ),
    )

    engine, factory_db = open_sqlite(tmp_path / "scan.db")
    Base.metadata.create_all(engine)
    repo = ScanRepository(factory_db)
    store = FilesystemObjectStore(tmp_path / "blobs")
    rt = _FakeRuntime(
        {(str(Calibration.Service.SNAPSHOT_BUNDLE), _SO101): bundle}
    )
    mod = ScanModule(
        runtime=rt,
        repository=repo,
        object_store=store,
        robots={
            _SO101: ScanRobotSpec(
                kinematics_factory=factory,
                urdf_path=tmp_path / "so101.urdf",
                arm_specs=arm,
            )
        },
    )

    # scan 2개 직접 삽입 (capture 경유 X — build 만 격리 검증)
    sess = mod.new_session(NewSessionRequest(robot_id=_SO101, label="t"))
    sid = sess.session.id
    assert sid is not None
    blob_bytes, w, h = _real_blob()
    raws = [[2001, 2002, 2003, 2004, 2005, 2006], [2101, 2102, 2103, 2104, 2105, 2106]]
    for i, raw in enumerate(raws):
        scan_id = repo.allocate_scan_id(sid)
        key = f"scans/{_SO101}/s/{scan_id:03d}.bin"
        store.put(key, blob_bytes)
        repo.insert_scan(
            ScanRow(
                session_row_id=sid,
                robot_id=_SO101,
                scan_id=scan_id,
                created_at=datetime.now(UTC),
                blob_key=key,
                num_frames=1,
                width=w,
                height=h,
                fx=600.0,
                fy=600.0,
                cx=32.0,
                cy=24.0,
                depth_scale=0.001,
                motor_positions=raw,
                arm_motor_ids=[1, 2, 3, 4, 5, 6],
            )
        )

    # heavy Open3D 대신 inputs 캡처 (여기 관심 = FK/pose 준비 정확성)
    captured: list[list[recon.BuildScanInput]] = []

    def fake_build_mesh(inputs, *, progress, **kwargs):  # noqa: ANN001, ANN002, ANN003
        captured.append(list(inputs))
        return recon.BuildResult(
            mesh_bytes=b"ply", vertex_count=1, triangle_count=1,
            n_scans=len(inputs), n_edges=0,
        )

    import modules.scan.module as scan_module_mod

    monkeypatch.setattr(scan_module_mod.recon, "build_mesh", fake_build_mesh)

    res = await mod.build(BuildRequest(session_row_id=sid))
    assert res.accepted, res.message

    # ① factory 구성 + lifecycle
    assert len(kins) == 1, "build 마다 fresh kin 1개 구성"
    assert kins[0].initialized and kins[0].closed

    # ② fk 입력 = raw_to_rad + joint_offset (모터 id 3 = arm index 2 만 +0.113)
    assert len(kins[0].fk_inputs) == 2
    for raw, got in zip(raws, kins[0].fk_inputs):
        plain = [units.raw_to_rad(raw[i], arm[i]) for i in range(6)]
        expected = [
            p + (0.113 if arm[i].id == 3 else 0.0) for i, p in enumerate(plain)
        ]
        assert got == pytest.approx(expected), (
            "joint_offset 미적용 (옛 코드 회귀) — fk 입력이 plain raw_to_rad"
        )

    # ③ t_base_cam_init = t_base_ee(R=I, t=(0.1,0.2,0.3)) · hand_eye(t=(0.01,...))
    t_init = captured[0][0].t_base_cam_init
    assert np.allclose(t_init[:3, 3], [0.11, 0.22, 0.33]), (
        f"hand_eye 합성 어긋남: {t_init[:3, 3]}"
    )
    assert np.allclose(t_init[:3, :3], np.eye(3))
