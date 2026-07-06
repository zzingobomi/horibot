from __future__ import annotations

from datetime import datetime
from enum import StrEnum

from pydantic import BaseModel

from framework.contract.model import StrictModel


class Waypoint:
    class Service(StrEnum):
        # waypoint CRUD
        TEACH = "srv/waypoint/teach"
        LIST = "srv/waypoint/list"
        RENAME = "srv/waypoint/rename"
        DELETE = "srv/waypoint/delete"
        # group CRUD
        CREATE_GROUP = "srv/waypoint/create_group"
        LIST_GROUPS = "srv/waypoint/list_groups"
        DELETE_GROUP = "srv/waypoint/delete_group"
        # group membership
        ADD_TO_GROUP = "srv/waypoint/add_to_group"
        REMOVE_FROM_GROUP = "srv/waypoint/remove_from_group"
        REORDER_GROUP = "srv/waypoint/reorder_group"
        LIST_GROUP_MEMBERS = "srv/waypoint/list_group_members"


class WaypointRecord(StrictModel):
    id: int | None = None
    robot_id: str
    name: str
    joint_values: list[float]
    joint_names: list[str]
    created_at: datetime


class WaypointGroupRecord(StrictModel):
    id: int | None = None
    robot_id: str
    name: str


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
    ordered_waypoint_row_ids: list[int]


class ReorderGroupResponse(BaseModel):
    ok: bool
    message: str = ""


class ListGroupMembersRequest(BaseModel):
    group_row_id: int


class ListGroupMembersResponse(BaseModel):
    waypoints: list[WaypointRecord]
