"""Waypoint domain — public contract surface.

Robot Asset Layer (Motion 위). Waypoint = 티칭한 joint 자세(rad). WaypointGroup =
목적별 묶음(ordered). Database-per-Module. 저장 단위 rad — Motion.TcpState.joints
계약을 그대로 소비 (raw encoder 는 Waypoint 가 모름, 계층 준수).
설계 docs/task_dsl_waypoint_port.md §2(D3~D8)·§4.
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
        # waypoint CRUD
        TEACH = "srv/waypoint/{robot_id}/teach"  # 현재 joint 로 저장
        LIST = "srv/waypoint/{robot_id}/list"
        RENAME = "srv/waypoint/{robot_id}/rename"
        DELETE = "srv/waypoint/{robot_id}/delete"
        # group CRUD
        CREATE_GROUP = "srv/waypoint/{robot_id}/create_group"
        LIST_GROUPS = "srv/waypoint/{robot_id}/list_groups"
        DELETE_GROUP = "srv/waypoint/{robot_id}/delete_group"
        # group membership (order 있는 join)
        ADD_TO_GROUP = "srv/waypoint/{robot_id}/add_to_group"
        REMOVE_FROM_GROUP = "srv/waypoint/{robot_id}/remove_from_group"
        REORDER_GROUP = "srv/waypoint/{robot_id}/reorder_group"
        LIST_GROUP_MEMBERS = "srv/waypoint/{robot_id}/list_group_members"


# ─── request / response ─────────────────────────────────────────────


class TeachRequest(BaseModel):
    name: str


class TeachResponse(BaseModel):
    accepted: bool
    waypoint: WaypointRecord | None = None
    message: str = ""


class ListWaypointsRequest(BaseModel):
    pass


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
    name: str


class CreateGroupResponse(BaseModel):
    accepted: bool
    group: WaypointGroupRecord | None = None
    message: str = ""


class ListGroupsRequest(BaseModel):
    pass


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
