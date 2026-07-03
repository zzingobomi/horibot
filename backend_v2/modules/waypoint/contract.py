"""Waypoint domain — public contract surface.

Robot Asset Layer (Motion 위). Waypoint = 티칭한 joint 자세(rad). WaypointGroup =
목적별 묶음(ordered). Database-per-Module. 저장 단위 rad — Motion.TcpState.joints
계약을 그대로 소비 (raw encoder 는 Waypoint 가 모름, 계층 준수).
설계 docs/backend_v2.md §17.2.
"""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum

from pydantic import BaseModel, ConfigDict


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


# ─── records (DB row ↔ wire) ────────────────────────────────────────


class WaypointRecord(_Strict):
    id: int | None = None
    robot_id: str
    name: str
    joint_values: list[float]  # rad (Motion 계약 단위 = TcpState.joints)
    joint_names: list[str]  # parallel (TcpState.joint_names) — order-robust 매핑
    created_at: datetime


class WaypointGroupRecord(_Strict):
    id: int | None = None
    robot_id: str
    name: str


# ─── nested contract ────────────────────────────────────────────────


class Waypoint:
    class Service(StrEnum):
        # robot-agnostic (host 당 1, backend_v2.md §2.7) — 대상 robot 은
        # 생성/목록(teach/list/create_group/list_groups)은 req.robot_id, 나머지는
        # row id 에서 파생 (backend_v2.md §2.7.1).
        # waypoint CRUD
        TEACH = "srv/waypoint/teach"  # 현재 joint 로 저장
        LIST = "srv/waypoint/list"
        RENAME = "srv/waypoint/rename"
        DELETE = "srv/waypoint/delete"
        # group CRUD
        CREATE_GROUP = "srv/waypoint/create_group"
        LIST_GROUPS = "srv/waypoint/list_groups"
        DELETE_GROUP = "srv/waypoint/delete_group"
        # group membership (order 있는 join)
        ADD_TO_GROUP = "srv/waypoint/add_to_group"
        REMOVE_FROM_GROUP = "srv/waypoint/remove_from_group"
        REORDER_GROUP = "srv/waypoint/reorder_group"
        LIST_GROUP_MEMBERS = "srv/waypoint/list_group_members"


# ─── request / response ─────────────────────────────────────────────


class TeachRequest(BaseModel):
    robot_id: str
    name: str


class TeachResponse(BaseModel):
    accepted: bool
    waypoint: WaypointRecord | None = None
    message: str = ""


class ListWaypointsRequest(BaseModel):
    robot_id: str


class ListWaypointsResponse(BaseModel):
    waypoints: list[WaypointRecord]


class RenameWaypointRequest(BaseModel):
    waypoint_row_id: int
    name: str


class RenameWaypointResponse(BaseModel):
    ok: bool
    message: str = ""


class DeleteWaypointRequest(BaseModel):
    waypoint_row_id: int


class DeleteWaypointResponse(BaseModel):
    ok: bool


class CreateGroupRequest(BaseModel):
    robot_id: str
    name: str


class CreateGroupResponse(BaseModel):
    accepted: bool
    group: WaypointGroupRecord | None = None
    message: str = ""


class ListGroupsRequest(BaseModel):
    robot_id: str


class ListGroupsResponse(BaseModel):
    groups: list[WaypointGroupRecord]


class DeleteGroupRequest(BaseModel):
    group_row_id: int


class DeleteGroupResponse(BaseModel):
    ok: bool


class AddToGroupRequest(BaseModel):
    group_row_id: int
    waypoint_row_id: int


class AddToGroupResponse(BaseModel):
    ok: bool
    message: str = ""


class RemoveFromGroupRequest(BaseModel):
    group_row_id: int
    waypoint_row_id: int


class RemoveFromGroupResponse(BaseModel):
    ok: bool


class ReorderGroupRequest(BaseModel):
    group_row_id: int
    ordered_waypoint_row_ids: list[int]  # 새 순서 (position = index)


class ReorderGroupResponse(BaseModel):
    ok: bool
    message: str = ""


class ListGroupMembersRequest(BaseModel):
    group_row_id: int


class ListGroupMembersResponse(BaseModel):
    waypoints: list[WaypointRecord]  # position 순
