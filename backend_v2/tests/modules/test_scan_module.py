"""ScanModule (@service wire) 검증 — in-process, hardware 불요.

robot-agnostic (host 당 1) — **multi-robot 눈속임 방지**
(backend_v2.md §2.7.3): 6DOF so101 + 5DOF omx 세션이 한 인스턴스
안에서 격리 (arm ids / raw / blob 경로 / scene3d dispatch). 전체 wire + TSDF build
는 test_scan_e2e 가 커버 — 여기선 capture dispatch/격리를 fake 로 직접.
"""

from __future__ import annotations

from pathlib import Path

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

    def ik(self, target_position, target_quaternion, current_joint_angles=None):  # noqa: ANN001, ANN201
        return None

    def fk_to_matrix(self, joint_angles):  # noqa: ANN001, ANN201
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], (0.0, 0.0, 0.0)

    def joint_limits(self, n=None):  # noqa: ANN001, ANN201
        return []

    def self_collision(self, joint_angles) -> bool:  # noqa: ANN001
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

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):  # noqa: ANN001,ANN002
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
                kinematics=_FakeKinematics(), arm_specs=_arm([1, 2, 3, 4, 5, 6])
            ),
            _OMX: ScanRobotSpec(
                kinematics=_FakeKinematics(), arm_specs=_arm([1, 2, 3, 4, 5])
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
