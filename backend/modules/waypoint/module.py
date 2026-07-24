from __future__ import annotations

import logging
from datetime import UTC, datetime

from framework.contract.service import service
from framework.contract.subscriber import subscriber
from modules.motion.contract import Motion, TcpState

from .contract import (
    AddToGroupRequest,
    AddToGroupResponse,
    CreateGroupRequest,
    CreateGroupResponse,
    DeleteGroupRequest,
    DeleteGroupResponse,
    DeleteWaypointRequest,
    DeleteWaypointResponse,
    GetWaypointByNameRequest,
    GetWaypointByNameResponse,
    ListGroupMembersByNameRequest,
    ListGroupMembersByNameResponse,
    ListGroupMembersRequest,
    ListGroupMembersResponse,
    ListGroupsRequest,
    ListGroupsResponse,
    ListWaypointsRequest,
    ListWaypointsResponse,
    RemoveFromGroupRequest,
    RemoveFromGroupResponse,
    RenameWaypointRequest,
    RenameWaypointResponse,
    ReorderGroupRequest,
    ReorderGroupResponse,
    TeachRequest,
    TeachResponse,
    Waypoint,
    WaypointGroupRecord,
    WaypointRecord,
)
from .persistence.repository import WaypointRepository

logger = logging.getLogger(__name__)


class WaypointModule:
    def __init__(self, repository: WaypointRepository) -> None:
        self._repo = repository
        self._joints: dict[str, list[float]] = {}
        self._joint_names: dict[str, list[str]] = {}

    @subscriber(Motion.Stream.TCP_STATE)
    def on_tcp_state(self, state: TcpState) -> None:
        self._joints[state.robot_id] = list(state.joints)
        self._joint_names[state.robot_id] = list(state.joint_names)

    # ── waypoint CRUD ─────────────────────────────────────────
    @service(Waypoint.Service.TEACH)
    def teach(self, req: TeachRequest) -> TeachResponse:
        joints = self._joints.get(req.robot_id)
        names = self._joint_names.get(req.robot_id)
        if joints is None or names is None:
            return TeachResponse(
                accepted=False, message="joint state 아직 없음 (Motion TcpState 대기)"
            )
        name = req.name.strip()
        if not name:
            return TeachResponse(accepted=False, message="이름 비어있음")
        if self._repo.get_waypoint_by_name(req.robot_id, name) is not None:
            return TeachResponse(accepted=False, message=f"'{name}' 이름 이미 있음")
        rec = self._repo.insert_waypoint(
            WaypointRecord(
                robot_id=req.robot_id,
                name=name,
                joint_values=list(joints),
                joint_names=list(names),
                created_at=datetime.now(UTC),
            )
        )
        logger.info(
            "Waypoint teach '%s' (robot=%s, dof=%d)", name, req.robot_id, len(joints)
        )
        return TeachResponse(accepted=True, waypoint=rec)

    @service(Waypoint.Service.LIST)
    def list_waypoints(self, req: ListWaypointsRequest) -> ListWaypointsResponse:
        return ListWaypointsResponse(waypoints=self._repo.list_waypoints(req.robot_id))

    @service(Waypoint.Service.GET_WAYPOINT_BY_NAME)
    def get_waypoint_by_name(
        self, req: GetWaypointByNameRequest
    ) -> GetWaypointByNameResponse:
        return GetWaypointByNameResponse(
            waypoint=self._repo.get_waypoint_by_name(req.robot_id, req.name)
        )

    @service(Waypoint.Service.RENAME)
    def rename(self, req: RenameWaypointRequest) -> RenameWaypointResponse:
        name = req.name.strip()
        if not name:
            return RenameWaypointResponse(ok=False, message="이름 비어있음")
        # 대상 robot = waypoint row 소유자 (req 중복 채널 X) — 유일성은 그 robot 범위
        wp = self._repo.get_waypoint(req.waypoint_row_id)
        if wp is None:
            return RenameWaypointResponse(
                ok=False, message=f"waypoint {req.waypoint_row_id} 없음"
            )
        if self._repo.get_waypoint_by_name(wp.robot_id, name) is not None:
            return RenameWaypointResponse(ok=False, message=f"'{name}' 이름 이미 있음")
        try:
            self._repo.rename_waypoint(req.waypoint_row_id, name)
        except KeyError as e:
            return RenameWaypointResponse(ok=False, message=str(e))
        return RenameWaypointResponse(ok=True)

    @service(Waypoint.Service.DELETE)
    def delete(self, req: DeleteWaypointRequest) -> DeleteWaypointResponse:
        self._repo.delete_waypoint(req.waypoint_row_id)
        return DeleteWaypointResponse(ok=True)

    # ── group CRUD ────────────────────────────────────────────
    @service(Waypoint.Service.CREATE_GROUP)
    def create_group(self, req: CreateGroupRequest) -> CreateGroupResponse:
        name = req.name.strip()
        if not name:
            return CreateGroupResponse(accepted=False, message="이름 비어있음")
        if self._repo.get_group_by_name(req.robot_id, name) is not None:
            return CreateGroupResponse(
                accepted=False, message=f"'{name}' group 이미 있음"
            )
        g = self._repo.insert_group(
            WaypointGroupRecord(robot_id=req.robot_id, name=name)
        )
        return CreateGroupResponse(accepted=True, group=g)

    @service(Waypoint.Service.LIST_GROUPS)
    def list_groups(self, req: ListGroupsRequest) -> ListGroupsResponse:
        return ListGroupsResponse(groups=self._repo.list_groups(req.robot_id))

    @service(Waypoint.Service.DELETE_GROUP)
    def delete_group(self, req: DeleteGroupRequest) -> DeleteGroupResponse:
        self._repo.delete_group(req.group_row_id)
        return DeleteGroupResponse(ok=True)

    # ── group membership ──────────────────────────────────────
    @service(Waypoint.Service.ADD_TO_GROUP)
    def add_to_group(self, req: AddToGroupRequest) -> AddToGroupResponse:
        if self._repo.get_waypoint(req.waypoint_row_id) is None:
            return AddToGroupResponse(
                ok=False, message=f"waypoint {req.waypoint_row_id} 없음"
            )
        self._repo.add_member(req.group_row_id, req.waypoint_row_id)
        return AddToGroupResponse(ok=True)

    @service(Waypoint.Service.REMOVE_FROM_GROUP)
    def remove_from_group(self, req: RemoveFromGroupRequest) -> RemoveFromGroupResponse:
        self._repo.remove_member(req.group_row_id, req.waypoint_row_id)
        return RemoveFromGroupResponse(ok=True)

    @service(Waypoint.Service.REORDER_GROUP)
    def reorder_group(self, req: ReorderGroupRequest) -> ReorderGroupResponse:
        self._repo.reorder_group(req.group_row_id, req.ordered_waypoint_row_ids)
        return ReorderGroupResponse(ok=True)

    @service(Waypoint.Service.LIST_GROUP_MEMBERS)
    def list_group_members(
        self, req: ListGroupMembersRequest
    ) -> ListGroupMembersResponse:
        return ListGroupMembersResponse(
            waypoints=self._repo.list_group_members(req.group_row_id)
        )

    @service(Waypoint.Service.LIST_GROUP_MEMBERS_BY_NAME)
    def list_group_members_by_name(
        self, req: ListGroupMembersByNameRequest
    ) -> ListGroupMembersByNameResponse:
        group = self._repo.get_group_by_name(req.robot_id, req.name)
        if group is None or group.id is None:
            return ListGroupMembersByNameResponse(found=False, waypoints=[])
        return ListGroupMembersByNameResponse(
            found=True, waypoints=self._repo.list_group_members(group.id)
        )
