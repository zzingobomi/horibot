"""world_scan task 시나리오 검증 (FakeContext — 하드웨어/wire 없음).

pick 편승에서 분리된 전용 스캔 task 의 계약: 조 열기 → 세션(+프루닝) → 스윕
(pose 마다 이동→캡처→빌드 성장). **실패는 침묵하지 않는다** — 캡처/빌드 거부 =
TaskError raise (§2.4 silent best-effort 재발 방지). 각 구멍 = 결정적 테스트 1개.
"""

from __future__ import annotations

import pytest
from datetime import UTC, datetime

from modules.motion.contract import Motion, MoveJResponse
from modules.motor.contract import Motor, SetGripperResponse
from modules.scan.contract import (
    BuildResponse,
    CaptureResponse,
    DeleteSessionResponse,
    ListSessionsResponse,
    NewSessionResponse,
    ReconstructionRecord,
    Scan,
    ScanSessionRecord,
)
from modules.tasks.core.errors import TaskError
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.world_scan import steps
from modules.tasks.world_scan.module import WorldScanModule
from modules.waypoint.contract import (
    ListGroupMembersResponse,
    ListGroupsResponse,
    Waypoint,
    WaypointGroupRecord,
    WaypointRecord,
)

_BOT = "so101_6dof_0"
_TS = datetime.fromtimestamp(0, UTC)
_SPEC = TaskRobotSpec(
    gripper_open_raw=3186,
    gripper_close_raw=1935,
    gripper_index=5,
    gripper_held_threshold_raw=2100,
)

_GRIP = str(Motor.Service.SET_GRIPPER)
_MOVE_J = str(Motion.Service.MOVE_J)
_NEW = str(Scan.Service.NEW_SESSION)
_LIST_SESS = str(Scan.Service.LIST_SESSIONS)
_DEL_SESS = str(Scan.Service.DELETE_SESSION)
_CAP = str(Scan.Service.CAPTURE)
_BUILD = str(Scan.Service.BUILD)
_LIST_GROUPS = str(Waypoint.Service.LIST_GROUPS)
_LIST_MEMBERS = str(Waypoint.Service.LIST_GROUP_MEMBERS)


@pytest.fixture(autouse=True)
def _fast(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(steps, "_GRIPPER_SETTLE_S", 0.0)
    monkeypatch.setattr(steps, "_SETTLE_S", 0.0)


def _session(sid: int, label: str = "world_scan") -> ScanSessionRecord:
    return ScanSessionRecord(
        id=sid, robot_id=_BOT, session_id=f"s{sid}", created_at=_TS, label=label
    )


def _recon(verts: int = 1000) -> ReconstructionRecord:
    return ReconstructionRecord(
        id=1, session_row_id=7, robot_id=_BOT, created_at=_TS, blob_key="k",
        voxel_size=0.002, sdf_trunc=0.008, depth_trunc=0.5, icp_max_dist=0.02,
        n_scans=2, n_edges=1, vertex_count=verts, triangle_count=verts * 2,
        elapsed=1.0,
    )


def _scan_group(n_poses: int = 3) -> dict:
    grp = ListGroupsResponse(
        groups=[WaypointGroupRecord(id=2, robot_id=_BOT, name="search")]
    )
    members = ListGroupMembersResponse(
        waypoints=[
            WaypointRecord(
                id=i + 1, robot_id=_BOT, name=f"scan_{i}",
                joint_values=[float(i)] * 6, joint_names=[], created_at=_TS,
            )
            for i in range(n_poses)
        ]
    )
    return {_LIST_GROUPS: [grp], _LIST_MEMBERS: [members]}


def _script(
    n_poses: int = 3,
    sessions_before: list[ScanSessionRecord] | None = None,
    caps: list[CaptureResponse] | None = None,
    builds: list[BuildResponse] | None = None,
) -> dict:
    """world_scan 성공 스크립트 — 필요 부분만 override."""
    # capture: 기본 = pose 순서대로 scan_count 1..n
    caps = caps or [
        CaptureResponse(accepted=True, scan_count=i + 1) for i in range(n_poses)
    ]
    # build: 스윕 끝에 1번 (전체 재빌드 = 1회로 완전)
    builds = builds or [BuildResponse(accepted=True, reconstruction=_recon())]
    return {
        _GRIP: [SetGripperResponse()],
        _MOVE_J: [MoveJResponse()] * n_poses,
        _NEW: [NewSessionResponse(session=_session(7))],
        _LIST_SESS: [ListSessionsResponse(sessions=sessions_before or [_session(7)])],
        _DEL_SESS: [DeleteSessionResponse(ok=True)] * 10,
        _CAP: caps,
        _BUILD: builds,
        **_scan_group(n_poses),
    }


def _ctx(script: dict) -> FakeContext:
    return FakeContext(robots=[_BOT], specs={_BOT: _SPEC}, service_script=script)


def _module() -> WorldScanModule:
    class _Rt:
        def publish(self, k: str, e) -> None: ...  # noqa: ANN001
        async def call(self, *a, **kw): ...  # noqa: ANN002, ANN003, ANN201

    return WorldScanModule(_Rt(), {})  # type: ignore[arg-type]


# ─── happy path ───────────────────────────────────────────────────


async def test_sweep_capture_all_then_build_once():
    """조 열기 → 세션 → 3 pose 스윕: 이동 3 / 캡처 3 / **빌드 1 (끝에)**.

    빌드는 전체 재빌드라 마지막 1회면 완전 — pose 마다 안 함 (낭비/느림 방지).
    성장 UX 는 프론트 포인트클라우드 누적 몫 (빌드 없이)."""
    ctx = _ctx(_script(n_poses=3))
    await _module().scenario(ctx, voxel_size=0.002)

    assert len(ctx.calls(_GRIP)) == 1  # 관측 전 가동 조 open (§3.4)
    assert ctx.calls(_GRIP)[0]["req"].position_raw == _SPEC.gripper_open_raw
    assert len(ctx.calls(_NEW)) == 1
    assert len(ctx.calls(_MOVE_J)) == 3  # pose 당 1 이동
    assert len(ctx.calls(_CAP)) == 3
    assert len(ctx.calls(_BUILD)) == 1  # 스윕 끝에 딱 1번
    # 순서: 조 열기 → 세션 → 이동/캡처 … → (마지막) 빌드
    keys = ctx.keys()
    assert keys.index(_GRIP) < keys.index(_NEW) < keys.index(_MOVE_J)
    assert keys.index(_CAP) < keys.index(_BUILD)  # 캡처 다 끝난 뒤 빌드
    assert keys.index(_MOVE_J) < keys.index(_BUILD)


async def test_voxel_passes_through_to_build():
    """RunRequest.voxel_size → scan BUILD voxel_size 관통 (품질 셀렉터 backend 절반)."""
    ctx = _ctx(_script(n_poses=3))
    await _module().scenario(ctx, voxel_size=0.004)
    assert ctx.calls(_BUILD)[0]["req"].voxel_size == 0.004


# ─── 프루닝 ─────────────────────────────────────────────────────────


async def test_prunes_old_world_sessions_but_not_manual():
    """이전 world_scan 세션은 삭제, 수동 스캔 세션·새 세션은 보존 (latest-wins +
    git-tracked rdb bloat 방지)."""
    before = [_session(7), _session(3), _session(4), _session(9, label="manual")]
    ctx = _ctx(_script(n_poses=3, sessions_before=before))
    await _module().scenario(ctx, voxel_size=None)
    deleted = {c["req"].session_row_id for c in ctx.calls(_DEL_SESS)}
    assert deleted == {3, 4}  # 옛 world 세션만 (7=새 세션, 9=manual 보존)


# ─── 실패 = 침묵 없이 raise ─────────────────────────────────────────


async def test_capture_reject_raises():
    """캡처 거부 = TaskError raise (침묵 skip 아님 — §2.4 재발 방지)."""
    caps = [
        CaptureResponse(accepted=True, scan_count=1),
        CaptureResponse(accepted=False, message="camera 프레임 없음"),
    ]
    ctx = _ctx(_script(n_poses=3, caps=caps, builds=[]))
    with pytest.raises(TaskError, match="캡처 거부"):
        await _module().scenario(ctx, voxel_size=None)


async def test_build_failure_raises():
    """빌드 실패 = TaskError raise (정합 실패를 조용히 넘기지 않는다)."""
    builds = [BuildResponse(accepted=False, message="정합 발산")]
    ctx = _ctx(_script(n_poses=3, builds=builds))
    with pytest.raises(TaskError, match="빌드 실패"):
        await _module().scenario(ctx, voxel_size=None)


async def test_missing_scan_group_raises():
    """'search' 그룹 없음 = 명시 실패 (침묵 단일-뷰 폴백 금지 — 티칭 안내)."""
    script = _script(n_poses=3)
    script[_LIST_GROUPS] = [ListGroupsResponse(groups=[])]  # search 그룹 없음
    ctx = _ctx(script)
    with pytest.raises(TaskError, match="'search' waypoint 그룹 없음"):
        await _module().scenario(ctx, voxel_size=None)
