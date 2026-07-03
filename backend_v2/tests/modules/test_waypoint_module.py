"""WaypointModule (@service wire) 검증 — in-process, hardware 불요.

teach(rad+names 저장) / CRUD / group ordering(reorder) / cascade invariant.
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

_ROBOT = "so101_6dof_0"
_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _module(tmp_path: Path) -> tuple[WaypointModule, WaypointRepository]:
    engine, factory = open_sqlite(tmp_path / "wp.db")
    Base.metadata.create_all(engine)
    repo = WaypointRepository(factory)
    return WaypointModule(robot_id=_ROBOT, repository=repo), repo


def _feed(mod: WaypointModule, joints: list[float]) -> None:
    mod.on_tcp_state(
        TcpState(
            robot_id=_ROBOT,
            seq=0,
            timestamp_unix=0.0,
            position=(0.0, 0.0, 0.0),
            quaternion=(0.0, 0.0, 0.0, 1.0),
            joint_names=_NAMES,
            joints=joints,
        )
    )


def test_service_wiring_discovers_all_keys(tmp_path: Path):
    mod, _ = _module(tmp_path)
    keys = {spec.wire_key for _m, spec in discover_services(mod)}
    assert keys == {
        Waypoint.Service.TEACH,
        Waypoint.Service.LIST,
        Waypoint.Service.RENAME,
        Waypoint.Service.DELETE,
        Waypoint.Service.CREATE_GROUP,
        Waypoint.Service.LIST_GROUPS,
        Waypoint.Service.DELETE_GROUP,
        Waypoint.Service.ADD_TO_GROUP,
        Waypoint.Service.REMOVE_FROM_GROUP,
        Waypoint.Service.REORDER_GROUP,
        Waypoint.Service.LIST_GROUP_MEMBERS,
    }


def test_teach_requires_joint_state(tmp_path: Path):
    mod, _ = _module(tmp_path)
    res = mod.teach(TeachRequest(name="home"))
    assert not res.accepted and "joint state" in res.message


def test_teach_stores_rad_and_names(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    res = mod.teach(TeachRequest(name="home"))
    assert res.accepted and res.waypoint is not None
    assert res.waypoint.joint_values == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # rad 그대로
    assert res.waypoint.joint_names == _NAMES
    lst = mod.list_waypoints(ListWaypointsRequest()).waypoints
    assert len(lst) == 1 and lst[0].name == "home"


def test_teach_duplicate_name_rejected(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    assert mod.teach(TeachRequest(name="home")).accepted
    dup = mod.teach(TeachRequest(name="home"))
    assert not dup.accepted and "이미" in dup.message


def test_rename_and_delete(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    wp = mod.teach(TeachRequest(name="a")).waypoint
    assert wp is not None and wp.id is not None
    assert mod.rename(RenameWaypointRequest(waypoint_row_id=wp.id, name="b")).ok
    assert mod.list_waypoints(ListWaypointsRequest()).waypoints[0].name == "b"
    assert mod.delete(DeleteWaypointRequest(waypoint_row_id=wp.id)).ok
    assert mod.list_waypoints(ListWaypointsRequest()).waypoints == []


def test_group_add_and_ordered_members(tmp_path: Path):
    mod, _ = _module(tmp_path)
    _feed(mod, [0.0] * 6)
    w1 = mod.teach(TeachRequest(name="left")).waypoint
    w2 = mod.teach(TeachRequest(name="right")).waypoint
    assert w1 and w2 and w1.id and w2.id
    g = mod.create_group(CreateGroupRequest(name="search")).group
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
    w1 = mod.teach(TeachRequest(name="left")).waypoint
    w2 = mod.teach(TeachRequest(name="right")).waypoint
    assert w1 and w2 and w1.id and w2.id
    g = mod.create_group(CreateGroupRequest(name="s")).group
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
    w1 = mod.teach(TeachRequest(name="left")).waypoint
    w2 = mod.teach(TeachRequest(name="right")).waypoint
    assert w1 and w2 and w1.id and w2.id
    g = mod.create_group(CreateGroupRequest(name="s")).group
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
    w1 = mod.teach(TeachRequest(name="left")).waypoint
    w2 = mod.teach(TeachRequest(name="right")).waypoint
    assert w1 and w2 and w1.id and w2.id
    g = mod.create_group(CreateGroupRequest(name="s")).group
    assert g and g.id
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w1.id))
    mod.add_to_group(AddToGroupRequest(group_row_id=g.id, waypoint_row_id=w2.id))
    # 라이브러리에서 삭제 → group 멤버십도 CASCADE 제거
    mod.delete(DeleteWaypointRequest(waypoint_row_id=w1.id))
    members = mod.list_group_members(
        ListGroupMembersRequest(group_row_id=g.id)
    ).waypoints
    assert [m.name for m in members] == ["right"]
