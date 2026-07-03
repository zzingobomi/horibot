"""WaypointModule — Robot Asset Layer (Motion 위).

티칭 = Motion.Stream.TCP_STATE(rad joints + names) 캐시 → 현재 값 저장. Waypoint 는
Motion 계약만 소비 — raw encoder / calibration / units 모름 (계층 준수, D6/D4).
Database-per-Module (WaypointRepository). PC 배치 (DB owner).

모든 핸들러 sync — cross-module call 없음 (CRUD + joint 캐시). runtime 불필요.
"""

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
    def __init__(self, robot_id: str, repository: WaypointRepository) -> None:
        self.robot_id = robot_id
        self._repo = repository
        # 최신 joint (rad) + names — Motion.TcpState 에서 캐시. teach 소스.
        self._joints: list[float] | None = None
        self._joint_names: list[str] | None = None

    # ── joint state 캐시 (Motion 계약 소비) ────────────────────
    @subscriber(Motion.Stream.TCP_STATE)
    def on_tcp_state(self, state: TcpState) -> None:
        if state.robot_id != self.robot_id:
            return
        self._joints = list(state.joints)
        self._joint_names = list(state.joint_names)

    # ── waypoint CRUD ─────────────────────────────────────────
    @service(Waypoint.Service.TEACH)
    def teach(self, req: TeachRequest) -> TeachResponse:
        if self._joints is None or self._joint_names is None:
            return TeachResponse(
                accepted=False, message="joint state 아직 없음 (Motion TcpState 대기)"
            )
        name = req.name.strip()
        if not name:
            return TeachResponse(accepted=False, message="이름 비어있음")
        if self._repo.get_waypoint_by_name(self.robot_id, name) is not None:
            return TeachResponse(accepted=False, message=f"'{name}' 이름 이미 있음")
        rec = self._repo.insert_waypoint(
            WaypointRecord(
                robot_id=self.robot_id,
                name=name,
                joint_values=list(self._joints),
                joint_names=list(self._joint_names),
                created_at=datetime.now(UTC),
            )
        )
        logger.info(
            "Waypoint teach '%s' (robot=%s, dof=%d)",
            name, self.robot_id, len(self._joints),
        )
        return TeachResponse(accepted=True, waypoint=rec)

    @service(Waypoint.Service.LIST)
    def list_waypoints(self, req: ListWaypointsRequest) -> ListWaypointsResponse:
        return ListWaypointsResponse(waypoints=self._repo.list_waypoints(self.robot_id))

    @service(Waypoint.Service.RENAME)
    def rename(self, req: RenameWaypointRequest) -> RenameWaypointResponse:
        name = req.name.strip()
        if not name:
            return RenameWaypointResponse(ok=False, message="이름 비어있음")
        if self._repo.get_waypoint_by_name(self.robot_id, name) is not None:
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
        if self._repo.get_group_by_name(self.robot_id, name) is not None:
            return CreateGroupResponse(accepted=False, message=f"'{name}' group 이미 있음")
        g = self._repo.insert_group(
            WaypointGroupRecord(robot_id=self.robot_id, name=name)
        )
        return CreateGroupResponse(accepted=True, group=g)

    @service(Waypoint.Service.LIST_GROUPS)
    def list_groups(self, req: ListGroupsRequest) -> ListGroupsResponse:
        return ListGroupsResponse(groups=self._repo.list_groups(self.robot_id))

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
    def remove_from_group(
        self, req: RemoveFromGroupRequest
    ) -> RemoveFromGroupResponse:
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
