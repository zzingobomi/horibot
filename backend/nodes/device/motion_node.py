import logging
import time
from typing import cast

import numpy as np
from pydantic import BaseModel

from core.cache.joint_state_cache import JointStateCache
from core.coords.joint_coordinates import JointCoordinates
from core.coords.tool_coordinates import ToolCoordinates
from core.robot.robot_registry import RobotRegistry
from core.transport.device_node import DeviceNode
from modules.calibration.applier import CalibrationApplier
from modules.calibration.calibration_cache import CalibrationCache
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.motion import (
    JogJReq,
    JogTcpReq,
    MotionTcpPose,
    MotionTrajState,
    MoveCReq,
    MoveJReq,
    MoveLReq,
    MovePReq,
    ServoJReq,
    ServoTcpReq,
    TrajStatus,
)
from core.transport.messages.motor import MotorCmd, MotorCmdJoint, MotorSetProfileAllReq
from core.transport.topic_map import Service, Topic
from modules.kinematics.kinematics import Kinematics
from modules.kinematics.motion_commands import (
    JogJCommand,
    JogTcpCommand,
    MotionCommand,
    MoveCCommand,
    MoveJCommand,
    MoveLCommand,
    MovePCommand,
    ServoJCommand,
    ServoTcpCommand,
)
from modules.kinematics.motion_config import MotionConfig
from modules.kinematics.motion_modes import MotionModes
from modules.kinematics.trajectory_runner import TrajectoryRunner
from modules.motor.motor_config import load_motor_layout

logger = logging.getLogger(__name__)

# Tool offset — ToolCoordinates 싱글톤에서 로드 (별도 산출물 tool_offset.npz).
# EE frame 기준 (link5 +x 방향) 의 (실제 그리퍼 끝점 - URDF EE) 벡터. 의미:
#   실제 끝점 = URDF EE + R_be @ tool_offset_ee
# 외부 (detect, task) 가 받는 좌표는 obj 의 진짜 base 좌표 (frame 무관), motion
# service handler 가 cartesian 명령 진입 시점에만 단방향 변환 (target_urdf =
# target_user - tool_base). kinematics / 캘 / BA 는 URDF frame 그대로 (캘 reference
# frame 안정성 유지).


class MotionNode(DeviceNode):
    def __init__(self, robot_id: str):
        super().__init__("motion_node", robot_id=robot_id)

        layout = load_motor_layout(robot_id)
        self._arm_cfgs = layout.arm
        self._arm_ids = [cfg.id for cfg in self._arm_cfgs]
        self._n_arm = len(self._arm_cfgs)

        # robot_id 명시 — multi-enabled 환경 (host_mock 가 omx + robots.yaml default
        # 가 so101 같은 자리) 에서 default 가 self.robot_id 와 다르면 dof mismatch
        # / 잘못된 토픽 구독 됨. self.robot_id 로 통일.
        self._motion = MotionModes(robot_id=robot_id)
        self._joint_cache = JointStateCache()
        self._joint_cache.subscribe(self, robot_id=self.robot_id)
        # SpeedTcp 의 streamer 가 매 step Jacobian 풀 때 사용 — 같은 robot.
        self._kinematics = cast(Kinematics, RobotRegistry().get_kinematics(robot_id))

        # robot/<type>/motion.yaml SSOT — Ruckig 한계 (joint+cartesian).
        motion_cfg = cast(MotionConfig, RobotRegistry().get_motion_config(robot_id))

        missing = [
            cfg.name for cfg in self._arm_cfgs if cfg.name not in motion_cfg.joint_limits
        ]
        if missing:
            raise ValueError(
                f"motion.yaml: arm joint {missing} 의 joint_limits 누락 "
                f"(robot={robot_id})"
            )
        j_max_vel = [motion_cfg.joint_limits[cfg.name].max_velocity for cfg in self._arm_cfgs]
        j_max_acc = [motion_cfg.joint_limits[cfg.name].max_acceleration for cfg in self._arm_cfgs]
        j_max_jerk = [motion_cfg.joint_limits[cfg.name].max_jerk for cfg in self._arm_cfgs]

        # ServoTcpCommand 가 사용할 IK 콜백 — position-only / 6DOF 자동.
        def _solve_servo(position, quaternion, angles):
            return self._motion.servo_tcp(position, quaternion, angles)

        self._runner = TrajectoryRunner(
            n_arm=self._n_arm,
            joint_max_velocity=j_max_vel,
            joint_max_acceleration=j_max_acc,
            joint_max_jerk=j_max_jerk,
            cartesian_max_velocity=motion_cfg.cartesian_limits.max_trans_vel,
            cartesian_max_acceleration=motion_cfg.cartesian_limits.max_trans_acc,
            cartesian_max_jerk=motion_cfg.cartesian_limits.max_trans_jerk,
            release_profile=self._release_profile,
            restore_profile=self._restore_profile,
            publish_cmd=self._publish_cmd,
            publish_state=self._publish_traj_state,
            # Cartesian path 추종 IK = position-only (servo_tcp 의 6DOF 분기 안 씀).
            solve_ik=lambda pos, angles: self._motion.servo_tcp(pos, None, angles),
            get_joint_angles=self._get_joint_angles_for_streamer,
        )

        # ─── Service 등록 ───────────────────────────────────────
        self.create_service(
            self.r(Service.MOTION_GET_TCP),
            EmptyData,
            MotionTcpPose,
            self._srv_get_tcp,
        )
        self.create_service(
            self.r(Service.MOTION_MOVE_J),
            MoveJReq,
            EmptyData,
            self._make_handler(MoveJCommand(self._arm_cfgs)),
        )
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
        # ─── Servo (external target chase — RL / Vision / replay) ────
        # Caller 가 *자기가 계산한 절대 target* 보냄. server = direct IK + publish.
        # 산업 표준 UR `servoj/servoc` 의미. frontend Jog UI 자리는 별도
        # (Jog 계층). caller 자리 = 미래 RL / vision servo policy node.
        servo_tcp_cmd = ServoTcpCommand(_solve_servo, self._publish_cmd)
        self.create_service(
            self.r(Service.MOTION_SERVO_TCP),
            ServoTcpReq,
            EmptyData,
            self._cartesian_handler_factory(servo_tcp_cmd),
        )
        servo_j_cmd = ServoJCommand(self._publish_cmd, self._n_arm)
        self.create_service(
            self.r(Service.MOTION_SERVO_J),
            ServoJReq,
            EmptyData,
            self._make_servo_j_service_handler(servo_j_cmd),
        )

        # ─── Jog (human/manual velocity stream — frontend / gamepad) ──
        # Caller 가 velocity twist / joint vel 만 보냄. backend 가 fresh latch
        # + 실 측정 dt 로 적분 → IK + publish. SE(3) 적분 자리 SSOT = backend.
        # LeRobot delta-pose 패턴, cross-process safe (joint_offset 자리 backend
        # SSOT). docs/motion_taxonomy.md §Jog.
        def _get_current_arm_joints() -> list[float] | None:
            return self._joint_cache.get_joint_angles_rad(
                self._arm_cfgs, robot_id=self.robot_id
            )

        def _fk(arm_joints: list[float]) -> tuple[np.ndarray, np.ndarray]:
            pose = self._motion.get_tcp_pose(arm_joints)
            return np.asarray(pose.position), np.asarray(pose.quaternion)

        jog_j_cmd = JogJCommand(
            self._publish_cmd, _get_current_arm_joints, self._n_arm
        )
        jog_tcp_cmd = JogTcpCommand(
            _solve_servo,
            self._publish_cmd,
            _get_current_arm_joints,
            _fk,
            self._tool_offset_base,
        )
        self.create_service(
            self.r(Service.MOTION_JOG_J),
            JogJReq,
            EmptyData,
            self._make_jog_j_service_handler(jog_j_cmd),
        )
        self.create_service(
            self.r(Service.MOTION_JOG_TCP),
            JogTcpReq,
            EmptyData,
            self._make_jog_tcp_service_handler(jog_tcp_cmd),
        )
        # Topic streams — frontend Jog UI / gamepad 50Hz publish 자리.
        # bridge 가 frontend WS publish → Zenoh put → 본 subscriber 호출.
        self.create_subscriber(
            self.r(Topic.MOTION_JOG_J_STREAM),
            JogJReq,
            self._make_jog_j_topic_subscriber(jog_j_cmd),
        )
        self.create_subscriber(
            self.r(Topic.MOTION_JOG_TCP_STREAM),
            JogTcpReq,
            self._make_jog_tcp_topic_subscriber(jog_tcp_cmd),
        )
        self.create_service(
            self.r(Service.MOTION_STOP), EmptyData, EmptyData, self._srv_stop
        )

    # ─── Lifecycle (cross-process calibration apply) ───────────

    def start(self) -> None:
        """부팅 시 storage 에서 active calibration fetch + 자기 process 의 runtime
        객체 (Coordinates 3종 + PybulletKinematics) 에 apply 후 heartbeat 시작.

        분산 자리 — calibration_node 가 PC, motion_node 는 모터 Pi. PC 의
        calibration_node 는 자기 process 객체만 mutate 함 → 모터 Pi 의
        PybulletKinematics 는 그 push 못 받음. 본 메서드가 *cross-process leg* —
        모터 Pi process 에서 직접 storage 호출해 자기 객체 init (docs/storage_layer.md §7).

        storage 미연결 자리는 `fetch_active` 안의 `load_active_blocking` 가 무한 retry —
        PC 가 늦게 떠도 자동 연결. storage 의 응답 found=false (첫 부팅 robot) 자리는
        default empty offsets/identity → PybulletKinematics 가 unpatched URDF 로 init.
        """
        # DeviceNode 는 robot_id 필수 — BaseNode 의 Optional 타입을 좁힘.
        assert self.robot_id is not None, "DeviceNode 의 robot_id 누락"
        cache = CalibrationCache()
        snapshot = cache.fetch_active(self.robot_id)
        CalibrationApplier.apply(self.robot_id, snapshot)
        super().start()

    # ─── Tool offset 변환 유틸 ─────────────────────────────────
    #
    # _tool_offset_base(angles): 현재 자세에서 (실제 끝점 - URDF EE) 의 base frame 벡터.
    # MoveJ 는 joint 명령이라 tool offset 무관. MoveL / MoveC / MoveP / ServoTcp /
    # GetTCP 같이 *cartesian 좌표* 를 입출력하는 명령에만 적용.

    def _tool_offset_base(self, angles: list[float]) -> np.ndarray:
        """현재 자세의 R_be 로 tool_offset_ee 를 base frame 으로 변환.

        반환: (3,) base frame 벡터. tool_offset.npz 비어있으면 [0,0,0].
        """
        tool_ee = ToolCoordinates().trans_m()
        if not np.any(tool_ee):
            return np.zeros(3, dtype=np.float64)
        R_be, _ = self._kinematics.fk_to_matrix(angles)
        return np.asarray(R_be) @ np.asarray(tool_ee)

    def _dispatch_cartesian(
        self, cmd: MotionCommand, data_dict: dict, *, verbose: bool
    ) -> str | None:
        """MoveL / MoveC / MoveP / ServoTcp 공통 chain.

        - validate → angles fetch → FK + tool_offset → user frame → URDF frame
          변환 → cmd.execute. 반환: 에러 메시지 (실패) 또는 None (성공).
        - `verbose=True` (service caller — 단발) → tool_offset 변환 log + 시작 log.
          `verbose=False` (topic stream — 50Hz) → log 생략 (스팸 방지).
        """
        req_dict = {"data": data_dict}
        error = cmd.validate(req_dict)
        if error:
            return error

        angles = self._joint_cache.get_joint_angles_rad(
            self._arm_cfgs, robot_id=self.robot_id
        )
        if angles is None:
            return "관절 상태 수신 전"

        try:
            tcp_pos_urdf = list(self._motion.get_tcp_pose(angles).position)
            tool_base = self._tool_offset_base(angles)
        except Exception as e:
            return f"FK 오류: {e}"

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
        if verbose:
            logger.info(
                "[tool_offset] %s tool_base=%s  user→urdf: %s → %s",
                cmd.label, np.round(tool_base, 4).tolist(),
                user_pos_before, data_dict.get("position"),
            )

        try:
            cmd.execute(req_urdf, angles, tcp_pos_urdf, self._runner)
            if verbose:
                self.log("info", f"{cmd.label} 시작")
            return None
        except ValueError as e:
            return str(e)
        except Exception as e:
            logger.error(f"{cmd.label} execute 오류: {e}")
            return str(e)

    def _cartesian_handler_factory(self, cmd: MotionCommand):
        """Service handler for MoveL / MoveC / MoveP / ServoTcp."""

        def handler(
            req: ServiceRequest[BaseModel],
        ) -> ServiceResponse[EmptyData]:
            err = self._dispatch_cartesian(
                cmd, req.data.model_dump(), verbose=True
            )
            if err is not None:
                return ServiceResponse(success=False, message=err, data=None)
            return ServiceResponse(success=True, message="ok", data=EmptyData())

        return handler

    def _make_servo_j_service_handler(self, cmd: MotionCommand):
        """ServoJ service handler — 단발 절대 joint target (RL replay 자리)."""

        def handler(
            req: ServiceRequest[ServoJReq],
        ) -> ServiceResponse[EmptyData]:
            req_dict = {"data": req.data.model_dump()}
            error = cmd.validate(req_dict)
            if error:
                return ServiceResponse(success=False, message=error, data=None)
            try:
                cmd.execute(req_dict, [], [], self._runner)
                return ServiceResponse(success=True, message="ok", data=EmptyData())
            except ValueError as e:
                return ServiceResponse(success=False, message=str(e), data=None)
            except Exception as e:
                logger.error(f"ServoJ 오류: {e}")
                return ServiceResponse(success=False, message=str(e), data=None)

        return handler

    def _make_jog_j_service_handler(self, cmd: MotionCommand):
        """JogJ service handler — 단발 velocity (자동화 tool / test 호출)."""

        def handler(
            req: ServiceRequest[JogJReq],
        ) -> ServiceResponse[EmptyData]:
            req_dict = {"data": req.data.model_dump()}
            error = cmd.validate(req_dict)
            if error:
                return ServiceResponse(success=False, message=error, data=None)
            try:
                cmd.execute(req_dict, [], [], self._runner)
                return ServiceResponse(success=True, message="ok", data=EmptyData())
            except ValueError as e:
                return ServiceResponse(success=False, message=str(e), data=None)
            except Exception as e:
                logger.error(f"JogJ 오류: {e}")
                return ServiceResponse(success=False, message=str(e), data=None)

        return handler

    def _make_jog_tcp_service_handler(self, cmd: MotionCommand):
        """JogTcp service handler — 단발 twist (자동화 tool / test 호출)."""

        def handler(
            req: ServiceRequest[JogTcpReq],
        ) -> ServiceResponse[EmptyData]:
            req_dict = {"data": req.data.model_dump()}
            error = cmd.validate(req_dict)
            if error:
                return ServiceResponse(success=False, message=error, data=None)
            try:
                cmd.execute(req_dict, [], [], self._runner)
                return ServiceResponse(success=True, message="ok", data=EmptyData())
            except ValueError as e:
                return ServiceResponse(success=False, message=str(e), data=None)
            except Exception as e:
                logger.error(f"JogTcp 오류: {e}")
                return ServiceResponse(success=False, message=str(e), data=None)

        return handler

    def _make_jog_j_topic_subscriber(self, cmd: MotionCommand):
        """JogJ topic subscriber — 50Hz hot path, throttled error log."""
        last_err_ts = [0.0]
        last_err_msg = [""]

        def callback(req: BaseModel) -> None:
            req_dict = {"data": req.model_dump()}
            error = cmd.validate(req_dict)
            if error is None:
                try:
                    cmd.execute(req_dict, [], [], self._runner)
                    return
                except ValueError as e:
                    error = str(e)
                except Exception as e:
                    logger.error(f"JogJ stream execute 오류: {e}")
                    error = str(e)
            now = time.time()
            if now - last_err_ts[0] > 1.0 or error != last_err_msg[0]:
                logger.warning(f"JogJ stream 오류: {error}")
                last_err_ts[0] = now
                last_err_msg[0] = error

        return callback

    def _make_jog_tcp_topic_subscriber(self, cmd: MotionCommand):
        """JogTcp topic subscriber — 50Hz hot path. tool_offset 자리는 command 가
        매 cycle 내부 처리 (실 끝점 적분 자리)."""
        last_err_ts = [0.0]
        last_err_msg = [""]

        def callback(req: BaseModel) -> None:
            req_dict = {"data": req.model_dump()}
            error = cmd.validate(req_dict)
            if error is None:
                try:
                    cmd.execute(req_dict, [], [], self._runner)
                    return
                except ValueError as e:
                    error = str(e)
                except Exception as e:
                    logger.error(f"JogTcp stream execute 오류: {e}")
                    error = str(e)
            now = time.time()
            if now - last_err_ts[0] > 1.0 or error != last_err_msg[0]:
                logger.warning(f"JogTcp stream 오류: {error}")
                last_err_ts[0] = now
                last_err_msg[0] = error

        return callback

    def _make_handler(self, cmd: MotionCommand):
        """MoveJ 전용 (joint 명령 — tool offset 무관)."""

        def handler(
            req: ServiceRequest[MoveJReq],
        ) -> ServiceResponse[EmptyData]:
            req_dict = {"data": req.data.model_dump()}

            error = cmd.validate(req_dict)
            if error:
                return ServiceResponse(success=False, message=error, data=None)

            angles = self._joint_cache.get_joint_angles_rad(self._arm_cfgs, robot_id=self.robot_id)
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
        t_be 가 *URDF EE 의 base 위치* 여야 obj 의 진짜 base 좌표 산출.

        Tool offset 적용은 *motion 핸들러 단방향* — 명령 좌표 (실제 끝점이 가야 할
        진짜 base 좌표) 를 URDF target (= 명령 - tool_base) 으로 변환해 IK 호출.
        실제 끝점 = URDF EE + tool_base = 명령 위치 ✓
        """
        angles = self._joint_cache.get_joint_angles_rad(self._arm_cfgs, robot_id=self.robot_id)
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

    def _get_joint_angles_for_streamer(self) -> list[float] | None:
        """TrajectoryRunner cartesian path 자리 초기 자세 자리. None = joint state 미수신."""
        return self._joint_cache.get_joint_angles_rad(self._arm_cfgs, robot_id=self.robot_id)

    def _release_profile(self) -> bool:
        """raw 0,0 → motor cap 해제 (Ruckig 가 직접 trajectory shape 만든다)."""
        res = self.call_service(
            self.r(Service.MOTOR_SET_PROFILE_ALL),
            MotorSetProfileAllReq(
                ids=self._arm_ids,
                velocity=0,
                acceleration=0,
            ),
            EmptyData,
        )
        return res.success

    def _restore_profile(self) -> bool:
        """각 모터의 motors.yaml `profile` (dps SSOT) 복원 — moveJ/L 종료 시."""
        res = self.call_service(
            self.r(Service.MOTOR_SET_PROFILE_ALL),
            MotorSetProfileAllReq(
                ids=self._arm_ids,
                restore_defaults=True,
            ),
            EmptyData,
        )
        return res.success
