"""WaypointModule (@service wire) 검증 — in-process, hardware 불요.

robot-agnostic (host 당 1) — teach(rad+names 저장) / CRUD / group ordering(reorder) /
cascade invariant + **multi-robot 눈속임 방지** (backend.md §2.7.3):
6DOF so101 + 5DOF omx 를 같은 인스턴스로 — 이름 유일성은 robot 단위,
joints 캐시는 payload robot_id 키.
"""

from __future__ import annotations

from pathlib import Path

from framework.runtime.discovery import discover_services
from infra.database.sqlite import open_sqlite
from modules.motion.contract import TcpState
from modules.waypoint.contract import (
    AddToGroupRequest,
    CreateGroupRequest,
    DeleteWaypointRequest,
    ListGroupMembersRequest,
    ListGroupsRequest,
    ListWaypointsRequest,
    RemoveFromGroupRequest,
    RenameWaypointRequest,
    ReorderGroupRequest,
    TeachRequest,
    Waypoint,
)
from modules.waypoint.module import WaypointModule
from modules.waypoint.persistence.orm import Base
from modules.waypoint.persistence.repository import WaypointRepository

_SO101 = "so101_6dof_0"
_OMX = "omx_f_0"
_NAMES6 = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_NAMES5 = ["joint1", "joint2", "joint3", "joint4", "joint5"]


def _module(tmp_path: Path) -> tuple[WaypointModule, WaypointRepository]:
    engine, factory = open_sqlite(tmp_path / "wp.db")
    Base.metadata.create_all(engine)
    repo = WaypointRepository(factory)
    return WaypointModule(repository=repo), repo


def _feed(
    mod: WaypointModule,
    joints: list[float],
    robot_id: str = _SO101,
    names: list[str] | None = None,
) -> None:
    mod.on_tcp_state(
        TcpState(
            robot_id=robot_id,
            seq=0,
            timestamp_unix=0.0,
            position=(0.0, 0.0, 0.0),
            quaternion=(0.0, 0.0, 0.0, 1.0),
            joint_names=names or _NAMES6,
            joints=joints,
        )
    )


def test_service_wiring_discovers_all_keys(tmp_path: Path):
    # 전체 키 목록은 contract 미러라 잠그지 않는다 (서비스 추가마다 수정 유발).
    # 계약 = §2.7.3 acceptance 1: robot-agnostic 키에 {robot_id} 없음.
    mod, _ = _module(tmp_path)
    keys = {spec.wire_key for _m, spec in discover_services(mod)}
    assert Waypoint.Service.TEACH in keys  # discovery 자체가 도는지
    assert all("{robot_id}" not in k for k in keys)
    assert not hasattr(mod, "robot_id")


def test_teach_requires_joint_state(tmp_path: Path):
    mod, _ = _module(tmp_path)
    res = mod.teach(TeachRequest(robot_id=_SO101, name="home"))
    assert not res.accepted and "joint state" in res.message


def test_teach_stores_rad_and_names(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    res = mod.teach(TeachRequest(robot_id=_SO101, name="home"))
    assert res.accepted and res.waypoint is not None
    assert res.waypoint.joint_values == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # rad 그대로
    assert res.waypoint.joint_names == _NAMES6
    lst = mod.list_waypoints(ListWaypointsRequest(robot_id=_SO101)).waypoints
    assert len(lst) == 1 and lst[0].name == "home"


def test_teach_duplicate_name_rejected(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    assert mod.teach(TeachRequest(robot_id=_SO101, name="home")).accepted
    dup = mod.teach(TeachRequest(robot_id=_SO101, name="home"))
    assert not dup.accepted and "이미" in dup.message


def test_rename_and_delete(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    wp = mod.teach(TeachRequest(robot_id=_SO101, name="a")).waypoint
    assert wp is not None and wp.id is not None
    assert mod.rename(RenameWaypointRequest(waypoint_row_id=wp.id, name="b")).ok
    assert (
        mod.list_waypoints(ListWaypointsRequest(robot_id=_SO101)).waypoints[0].name
        == "b"
    )
    assert mod.delete(DeleteWaypointRequest(waypoint_row_id=wp.id)).ok
    assert mod.list_waypoints(ListWaypointsRequest(robot_id=_SO101)).waypoints == []


def test_rename_missing_waypoint_rejected(tmp_path: Path):
    # rename 의 대상 robot 은 row 에서 파생 — row 없으면 명확히 reject
    mod, _ = _module(tmp_path)
    res = mod.rename(RenameWaypointRequest(waypoint_row_id=999, name="x"))
    assert not res.ok and "없음" in res.message


def test_group_add_and_ordered_members(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    w1 = mod.teach(TeachRequest(robot_id=_SO101, name="left")).waypoint
    w2 = mod.teach(TeachRequest(robot_id=_SO101, name="right")).waypoint
    assert w1 and w2 and w1.id and w2.id
    g = mod.create_group(CreateGroupRequest(robot_id=_SO101, name="search")).group
    assert g and g.id
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w1.id))
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w2.id))
    members = mod.list_group_members(
        ListGroupMembersRequest(group_row_id=g.id)
    ).waypoints
    assert [m.name for m in members] == ["left", "right"]  # 추가 순 position


def test_reorder_group(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    w1 = mod.teach(TeachRequest(robot_id=_SO101, name="left")).waypoint
    w2 = mod.teach(TeachRequest(robot_id=_SO101, name="right")).waypoint
    assert w1 and w2 and w1.id and w2.id
    g = mod.create_group(CreateGroupRequest(robot_id=_SO101, name="s")).group
    assert g and g.id
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w1.id))
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w2.id))
    mod.reorder_group(
        ReorderGroupRequest(group_row_id=g.id, ordered_waypoint_row_ids=[w2.id, w1.id])
    )
    members = mod.list_group_members(
        ListGroupMembersRequest(group_row_id=g.id)
    ).waypoints
    assert [m.name for m in members] == ["right", "left"]


def test_remove_member(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    w1 = mod.teach(TeachRequest(robot_id=_SO101, name="left")).waypoint
    w2 = mod.teach(TeachRequest(robot_id=_SO101, name="right")).waypoint
    assert w1 and w2 and w1.id and w2.id
    g = mod.create_group(CreateGroupRequest(robot_id=_SO101, name="s")).group
    assert g and g.id
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w1.id))
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w2.id))
    mod.remove_from_group(
        RemoveFromGroupRequest(group_row_id=g.id, waypoint_row_id=w1.id)
    )
    members = mod.list_group_members(
        ListGroupMembersRequest(group_row_id=g.id)
    ).waypoints
    assert [m.name for m in members] == ["right"]


def test_delete_waypoint_cascades_membership(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    w1 = mod.teach(TeachRequest(robot_id=_SO101, name="left")).waypoint
    w2 = mod.teach(TeachRequest(robot_id=_SO101, name="right")).waypoint
    assert w1 and w2 and w1.id and w2.id
    g = mod.create_group(CreateGroupRequest(robot_id=_SO101, name="s")).group
    assert g and g.id
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w1.id))
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w2.id))
    # 라이브러리에서 삭제 → group 멤버십도 CASCADE 제거
    mod.delete(DeleteWaypointRequest(waypoint_row_id=w1.id))
    members = mod.list_group_members(
        ListGroupMembersRequest(group_row_id=g.id)
    ).waypoints
    assert [m.name for m in members] == ["right"]


# ──────────────── multi-robot 눈속임 방지 (§2.7.3 acceptance) ────────────────


def test_single_instance_serves_so101_and_omx_isolated(tmp_path: Path):
    """★ 리트머스 — 한 host-level 인스턴스가 6DOF so101 + 5DOF omx 동시 구동.

    같은 이름 teach 가 robot 별로 독립 (유일성 = robot 단위), joints 캐시가
    payload robot_id 로 분리 (DOF 6 vs 5), 목록/rename 유일성 검사가 안 샘.
    """
    mod, _ = _module(tmp_path)
    _feed(mod, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], _SO101, _NAMES6)
    _feed(mod, [1.1, 1.2, 1.3, 1.4, 1.5], _OMX, _NAMES5)

    # 같은 이름 "home" — robot 별 독립 (robot 단위 유일성)
    so = mod.teach(TeachRequest(robot_id=_SO101, name="home"))
    omx = mod.teach(TeachRequest(robot_id=_OMX, name="home"))
    assert so.accepted and omx.accepted
    assert so.waypoint is not None and omx.waypoint is not None
    # 각자 자기 robot 의 joints (DOF 6 vs 5)
    assert so.waypoint.joint_values == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert so.waypoint.joint_names == _NAMES6
    assert omx.waypoint.joint_values == [1.1, 1.2, 1.3, 1.4, 1.5]
    assert omx.waypoint.joint_names == _NAMES5

    # 목록 격리
    so_list = mod.list_waypoints(ListWaypointsRequest(robot_id=_SO101)).waypoints
    omx_list = mod.list_waypoints(ListWaypointsRequest(robot_id=_OMX)).waypoints
    assert [w.robot_id for w in so_list] == [_SO101]
    assert [w.robot_id for w in omx_list] == [_OMX]

    # rename 유일성도 robot 단위 — omx 의 "home" 을 "rest" 로, so101 "home" 과 무관
    assert omx.waypoint.id is not None
    assert mod.rename(
        RenameWaypointRequest(waypoint_row_id=omx.waypoint.id, name="rest")
    ).ok
    # so101 쪽에서 "rest" 는 여전히 사용 가능 (robot 단위 namespace)
    assert mod.teach(TeachRequest(robot_id=_SO101, name="rest")).accepted

    # group 도 robot 별 독립 — 같은 이름 group
    g_so = mod.create_group(CreateGroupRequest(robot_id=_SO101, name="scan"))
    g_omx = mod.create_group(CreateGroupRequest(robot_id=_OMX, name="scan"))
    assert g_so.accepted and g_omx.accepted
    so_groups = mod.list_groups(ListGroupsRequest(robot_id=_SO101)).groups
    assert [g.robot_id for g in so_groups] == [_SO101]
