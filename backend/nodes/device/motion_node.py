import time
import logging
import numpy as np

from core.transport.device_node import DeviceNode
from core.transport.topic_map import Service, Topic
from core.coords.joint_coordinates import JointCoordinates
from core.cache.joint_state_cache import JointStateCache
from core.coords.tool_coordinates import ToolCoordinates
from modules.motor.motor_config import load_motor_layout
from modules.kinematics.motion_modes import MotionModes
from modules.kinematics.registry import get_default_kinematics
from modules.kinematics.trajectory_runner import TrajectoryRunner
from modules.kinematics.motion_commands import (
    MotionCommand,
    MoveCCommand,
    MoveJCommand,
    MoveLCommand,
    MovePCommand
)
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.motor import MotorCmd, MotorCmdJoint, MotorSetProfileAllReq
from core.transport.messages.motion import (
    MotionTcpPose,
    MotionTrajState,
    MoveCReq,
    MoveJReq,
    MoveLReq,
    MovePReq,
    MoveTcpReq,
    TrajStatus,
)
from pydantic import BaseModel

logger = logging.getLogger(__name__)

# Tool offset — ToolCoordinates 싱글톤에서 로드 (별도 산출물 tool_offset.npz).
# EE frame 기준 (link5 +x 방향) 의 (실제 그리퍼 끝점 - URDF EE) 벡터. 의미:
#   실제 끝점 = URDF EE + R_be @ tool_offset_ee
# 외부 (detect, task) 가 받는 좌표는 obj 의 진짜 base 좌표 (frame 무관), motion
# service handler 가 cartesian 명령 진입 시점에만 단방향 변환 (target_urdf =
# target_user - tool_base). kinematics / 캘 / BA 는 URDF frame 그대로 (캘 reference
# frame 안정성 유지).
#
# 이력:
# - 2026-05-28 EE patch (urdf_patcher 가 tcp_joint origin 직접 수정) 시도
#   실패. detect 와 IK 양쪽 patched URDF 위에 도는 self-consistency 로 cancel out.
# - 임시로 LinkCoordinates ID=6 행을 tool_offset 으로 재해석 → 의미 충돌.
# - 별도 산출물 tool_offset.npz 신설로 정리 (이 commit).


class MotionNode(DeviceNode):
    def __init__(self, robot_id: str):
        super().__init__("motion_node", robot_id=robot_id)

        layout = load_motor_layout(robot_id)
        self._arm_cfgs = layout.arm
        self._arm_ids = [cfg.id for cfg in self._arm_cfgs]
        self._n_arm = len(self._arm_cfgs)

        self._motion = MotionModes()
        self._joint_cache = JointStateCache()
        self._joint_cache.subscribe(self)

        # arm_profile 가 motors.yaml 에 있으면 TrajectoryRunner 의 restore 기준값을
        # 그 값으로 — motor_node start baseline 과 동일. 없으면 constructor default
        # (Dynamixel OMX 가 기존 150/40 그대로 쓰던 자리).
        runner_kwargs: dict[str, int] = {}
        if layout.arm_profile is not None:
            runner_kwargs["default_profile_vel"] = layout.arm_profile.velocity
            runner_kwargs["default_profile_acc"] = layout.arm_profile.acceleration

        self._runner = TrajectoryRunner(
            n_arm=self._n_arm,
            set_profile=self._set_arm_profile,
            publish_cmd=self._publish_cmd,
            publish_state=self._publish_traj_state,
            move_tcp=self._motion.move_tcp,
            **runner_kwargs,
        )

        self.create_service(
            self.r(Service.MOTION_GET_TCP), EmptyData, MotionTcpPose, self._srv_get_tcp
        )
        self.create_service(
            self.r(Service.MOTION_MOVE_TCP), MoveTcpReq, EmptyData, self._srv_move_tcp
        )
        self.create_service(
            self.r(Service.MOTION_MOVE_J),
            MoveJReq,
            EmptyData,
            self._make_handler(MoveJCommand(self._arm_cfgs)),
        )
        # MoveL/C/P 는 cartesian 좌표 입출력 → tool_offset 변환 적용
        self.create_service(
            self.r(Service.MOTION_MOVE_L),
            MoveLReq,
            EmptyData,
            self._cartesian_handler_factory(MoveLCommand()),
        )
        self.create_service(
            self.r(Service.MOTION_MOVE_C),
            MoveCReq,
            EmptyData,
            self._cartesian_handler_factory(MoveCCommand()),
        )
        self.create_service(
            self.r(Service.MOTION_MOVE_P),
            MovePReq,
            EmptyData,
            self._cartesian_handler_factory(MovePCommand()),
        )
        self.create_service(
            self.r(Service.MOTION_STOP), EmptyData, EmptyData, self._srv_stop
        )

    # ─── Tool offset 변환 유틸 ─────────────────────────────────
    #
    # _tool_offset_base(angles): 현재 자세에서 (실제 끝점 - URDF EE) 의 base frame 벡터.
    # MoveJ 는 joint 명령이라 tool offset 무관. MoveL / MoveC / MoveP / MoveTCP /
    # GetTCP 같이 *cartesian 좌표* 를 입출력하는 명령에만 적용.

    def _tool_offset_base(self, angles: list[float]) -> np.ndarray:
        """현재 자세의 R_be 로 tool_offset_ee 를 base frame 으로 변환.

        반환: (3,) base frame 벡터. tool_offset.npz 비어있으면 [0,0,0].
        """
        tool_ee = ToolCoordinates().trans_m()
        if not np.any(tool_ee):
            return np.zeros(3, dtype=np.float64)
        R_be, _ = get_default_kinematics().fk_to_matrix(angles)
        return np.asarray(R_be) @ np.asarray(tool_ee)

    def _cartesian_handler_factory(self, cmd: MotionCommand):
        """MoveL / MoveC / MoveP — cartesian 좌표 입출력. tool_offset 변환 적용.

        입력 (req.data.position / via / end / waypoints) = user frame.
        내부 cmd.execute 에는 URDF frame 으로 변환해 전달 (= 입력 - tool_offset_base).
        시작점 tcp_pos 도 URDF frame (kinematics.fk 결과 그대로) 사용 → start/end 일관.

        cmd 내부는 dict API 유지 (typed_messaging.md §미해결 #4) — handler 가
        `{"data": {...}}` envelope dict 로 wrap 해 cmd 에 넘김.
        """
        def handler(
            req: ServiceRequest[BaseModel],
        ) -> ServiceResponse[EmptyData]:
            # cmd 내부 dict API 와 호환: envelope dict 재구성
            data_dict = req.data.model_dump()
            req_dict = {"data": data_dict}

            error = cmd.validate(req_dict)
            if error:
                return ServiceResponse(success=False, message=error, data=None)

            angles = self._joint_cache.get_joint_angles_rad(self._arm_cfgs)
            if angles is None:
                return ServiceResponse(
                    success=False, message="관절 상태 수신 전", data=None
                )

            try:
                tcp_pos_urdf = list(self._motion.get_tcp_pose(angles).position)
                tool_base = self._tool_offset_base(angles)
            except Exception as e:
                return ServiceResponse(
                    success=False, message=f"FK 오류: {e}", data=None
                )

            # user → URDF 변환 (in-place 수정 피하려고 dict copy)
            user_pos_before = data_dict.get("position")
            for key in ("position", "via", "end"):
                if key in data_dict and data_dict[key] is not None:
                    data_dict[key] = (
                        np.asarray(data_dict[key], dtype=float) - tool_base
                    ).tolist()
            if "waypoints" in data_dict and data_dict["waypoints"] is not None:
                data_dict["waypoints"] = [
                    (np.asarray(wp, dtype=float) - tool_base).tolist()
                    for wp in data_dict["waypoints"]
                ]
            req_urdf = {"data": data_dict}
            logger.info(
                "[tool_offset] %s tool_base=%s  user→urdf: %s → %s",
                cmd.label, np.round(tool_base, 4).tolist(),
                user_pos_before, data_dict.get("position"),
            )

            try:
                cmd.execute(req_urdf, angles, tcp_pos_urdf, self._runner)
                self.log("info", f"{cmd.label} 시작")
                return ServiceResponse(
                    success=True, message="ok", data=EmptyData()
                )
            except ValueError as e:
                return ServiceResponse(success=False, message=str(e), data=None)
            except Exception as e:
                logger.error(f"{cmd.label} execute 오류: {e}")
                return ServiceResponse(success=False, message=str(e), data=None)

        return handler

    def _make_handler(self, cmd: MotionCommand):
        """MoveJ 전용 (joint 명령 — tool offset 무관). cmd 는 dict API 그대로."""
        def handler(
            req: ServiceRequest[MoveJReq],
        ) -> ServiceResponse[EmptyData]:
            req_dict = {"data": req.data.model_dump()}

            error = cmd.validate(req_dict)
            if error:
                return ServiceResponse(success=False, message=error, data=None)

            angles = self._joint_cache.get_joint_angles_rad(self._arm_cfgs)
            if angles is None:
                return ServiceResponse(
                    success=False, message="관절 상태 수신 전", data=None
                )

            try:
                tcp_pos = list(self._motion.get_tcp_pose(angles).position)
            except Exception as e:
                return ServiceResponse(
                    success=False, message=f"FK 오류: {e}", data=None
                )

            try:
                cmd.execute(req_dict, angles, tcp_pos, self._runner)
                self.log("info", f"{cmd.label} 시작")
                return ServiceResponse(
                    success=True, message="ok", data=EmptyData()
                )
            except ValueError as e:
                return ServiceResponse(success=False, message=str(e), data=None)
            except Exception as e:
                logger.error(f"{cmd.label} execute 오류: {e}")
                return ServiceResponse(success=False, message=str(e), data=None)

        return handler

    # ─── Services ─────────────────────────────────────────────

    def _srv_get_tcp(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[MotionTcpPose]:
        """URDF EE pose 그대로 반환 (변환 X).

        detect 의 hand_eye 캘은 URDF EE 기준으로 풀려 있어 obj_in_base 계산 시
        t_be 가 *URDF EE 의 base 위치* 여야 obj 의 진짜 base 좌표 산출:
            obj_in_base = R_be @ obj_in_ee_urdf + t_be_urdf  = obj 의 진짜 base 좌표

        만약 t_be 를 user frame (URDF EE + tool_base) 으로 응답하면 detect 는
        (진짜 좌표 + tool_base) 라는 *잘못된* obj_in_base 도출 → 명령 시점에 motion
        핸들러가 -tool_base 빼봐야 cancel out 되어 효과 0 (실측 확인: 22:42).

        Tool offset 적용은 *motion 핸들러 단방향* — 명령 좌표 (실제 끝점이 가야 할
        진짜 base 좌표) 를 URDF target (= 명령 - tool_base) 으로 변환해 IK 호출.
        실제 끝점 = URDF EE + tool_base = 명령 위치 ✓
        """
        angles = self._joint_cache.get_joint_angles_rad(self._arm_cfgs)
        if angles is None:
            return ServiceResponse(
                success=False, message="관절 상태 수신 전", data=None
            )
        try:
            pose = self._motion.get_tcp_pose(angles)
            return ServiceResponse(
                success=True,
                message="ok",
                data=MotionTcpPose(
                    position=list(pose.position),
                    quaternion=list(pose.quaternion),
                ),
            )
        except Exception as e:
            return ServiceResponse(success=False, message=str(e), data=None)

    def _srv_move_tcp(
        self, req: ServiceRequest[MoveTcpReq]
    ) -> ServiceResponse[EmptyData]:
        """target_pos (user frame) → URDF frame → IK."""
        target_pos_user = req.data.position
        angles = self._joint_cache.get_joint_angles_rad(self._arm_cfgs)
        if angles is None:
            return ServiceResponse(
                success=False, message="관절 상태 수신 전", data=None
            )
        try:
            tool_base = self._tool_offset_base(angles)
            target_pos_urdf = (
                np.asarray(target_pos_user, dtype=float) - tool_base
            ).tolist()
            result = self._motion.move_tcp(target_pos_urdf, angles)
            if result is None:
                return ServiceResponse(
                    success=False, message="IK 수렴 실패", data=None
                )
            self._publish_cmd(result)
            return ServiceResponse(success=True, message="ok", data=EmptyData())
        except Exception as e:
            return ServiceResponse(success=False, message=str(e), data=None)

    def _srv_stop(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[EmptyData]:
        was_running = self._runner.is_running
        self._runner.stop()
        if was_running:
            self._publish_traj_state(TrajStatus.STOPPED, 0.0)
            self.log("info", "트래젝토리 중단")
        return ServiceResponse(success=True, message="ok", data=EmptyData())

    # ─── Internal ────────────────────────────────────────────

    def _publish_cmd(self, angles_rad: list[float]) -> None:
        coords = JointCoordinates()
        self.publish(
            self.r(Topic.MOTOR_CMD_JOINT),
            MotorCmd(
                timestamp=time.time(),
                joints=[
                    MotorCmdJoint(
                        id=cfg.id,
                        position=coords.urdf_to_motor(
                            angle,
                            cfg,
                            min_raw=cfg.limit_min,
                            max_raw=cfg.limit_max,
                        ),
                    )
                    for cfg, angle in zip(self._arm_cfgs, angles_rad)
                ],
            ),
        )

    def _publish_traj_state(self, status: TrajStatus, progress: float) -> None:
        self.publish(
            self.r(Topic.MOTION_STATE_TRAJ),
            MotionTrajState(
                status=status,
                progress=round(progress, 3),
                timestamp=time.time(),
            ),
        )

    def _set_arm_profile(self, velocity: int, acceleration: int) -> bool:
        res = self.call_service(
            self.r(Service.MOTOR_SET_PROFILE_ALL),
            MotorSetProfileAllReq(
                ids=self._arm_ids,
                velocity=velocity,
                acceleration=acceleration,
            ),
            EmptyData,
        )
        return res.success
