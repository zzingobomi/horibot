import time
import logging
import threading

from core.transport.device_node import DeviceNode
from core.transport.topic_map import Topic, Service
from core.units import raw_to_deg
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.motor import (
    MotorCmd,
    MotorConfigItem,
    MotorEnableReq,
    MotorEnableRes,
    MotorGetConfigRes,
    MotorGripperReq,
    MotorJoint,
    MotorJointState,
    MotorLimit,
    MotorRebootReq,
    MotorSetProfileAllReq,
    MotorSetProfileReq,
)
from core.robot.robot_registry import RobotRegistry
from modules.motor.motor_config import load_motor_layout
from modules.motor.backend import MotorCommError

logger = logging.getLogger(__name__)

STATE_PUBLISH_HZ = 50  # 초당 상태 발행 횟수. SpeedTcp closed-loop Jacobian
# (50Hz _velocity_loop) 가 stale encoder reading 안 보도록 같은 rate 로 매칭.
# Feetech bulk read 가 7 motor (SO-101) 자리 ~수 ms 수준 → 50Hz 감당 가능.
# 부하 시 driver log 에 read latency 경고 — 그 자리는 20Hz 로 환원 검토.

# Gripper open/close raw position
GRIPPER_OPEN_RAW = 2600
GRIPPER_CLOSE_RAW = 1800  # current 제한이 있으므로 여유있게
GRIPPER_CURRENT_DEFAULT = 200  # mA, 기본 파지력


class MotorNode(DeviceNode):
    def __init__(self, robot_id: str):
        super().__init__("motor_node", robot_id=robot_id)

        layout = load_motor_layout(robot_id)
        self._layout = layout
        self.port = layout.port.get()
        self.motor_cfgs = layout.motors
        self._gripper_cfg = layout.gripper
        self._arm_cfgs = layout.arm
        # RobotRegistry factory 경유 — robots.yaml 의 motor_backend 따라
        # DynamixelBackend / FeetechBackend 자동 분기. 두 backend 모두
        # MotorBackend Protocol + legacy aliases 만족 (motor_node 호출 그대로).
        self.driver = RobotRegistry().get_motor_backend(robot_id)
        self.connected = False
        self.torque_enabled = False

        self._state_thread: threading.Thread | None = None

        self.create_subscriber(
            self.r(Topic.MOTOR_CMD_JOINT), MotorCmd, self._on_cmd_joint
        )
        self.create_service(
            self.r(Service.MOTOR_ENABLE), MotorEnableReq, MotorEnableRes, self._srv_enable
        )
        self.create_service(
            self.r(Service.MOTOR_REBOOT), MotorRebootReq, EmptyData, self._srv_reboot
        )
        self.create_service(
            self.r(Service.MOTOR_SET_PROFILE),
            MotorSetProfileReq,
            EmptyData,
            self._srv_set_profile,
        )
        self.create_service(
            self.r(Service.MOTOR_SET_PROFILE_ALL),
            MotorSetProfileAllReq,
            EmptyData,
            self._srv_set_profile_all,
        )
        self.create_service(
            self.r(Service.MOTOR_GET_CONFIG),
            EmptyData,
            MotorGetConfigRes,
            self._srv_get_config,
        )
        self.create_service(
            self.r(Service.MOTOR_GRIPPER), MotorGripperReq, EmptyData, self._srv_gripper
        )

    # ─── Lifecycle ───────────────────────────────────────────

    def start(self) -> None:
        try:
            self.driver.connect()
            self.connected = True
        except MotorCommError as e:
            self.connected = False
            self.torque_enabled = False
            self.log("error", f"Motor backend 연결 실패 ({self.port}): {e}")

        if self.connected:
            self._apply_position_pid()
            self.driver.torque_enable_all()
            self._apply_profiles()
            self.torque_enabled = True
            self.log("info", f"모터 노드 시작 ({self.port})")

        super().start()

        self._state_thread = threading.Thread(
            target=self._state_loop,
            name="motor-state",
            daemon=True,
        )
        self._state_thread.start()

    def stop(self) -> None:
        super().stop()
        if self.connected:
            self.driver.disconnect()

    def _apply_profiles(self, target_ids: list[int] | None = None) -> None:
        """모터별 motors.yaml `profile` 적용 — slider/teleop slam 방지 baseline.

        TrajectoryRunner 가 moveJ/L/C/P 진입 시 release (raw 0,0) 로 풀고 종료 시
        이 함수로 복원. dps 단위 → adapter 가 vendor 별 raw 변환.

        target_ids=None → 모든 모터. profile 이 None 인 모터는 skip (SDK default 유지).
        """
        targets = (
            [cfg for cfg in self.motor_cfgs if cfg.id in target_ids]
            if target_ids is not None
            else self.motor_cfgs
        )
        vel_map: dict[int, float] = {}
        acc_map: dict[int, float] = {}
        for cfg in targets:
            if cfg.profile is None:
                continue
            vel_map[cfg.id] = cfg.profile.velocity_dps
            acc_map[cfg.id] = cfg.profile.acceleration_dpss
        if not vel_map:
            return
        try:
            self.driver.write_profile_accelerations_dpss(acc_map)
            self.driver.write_profile_velocities_dps(vel_map)
            self.log(
                "info",
                f"모터 profile 적용: {len(vel_map)}개 "
                f"(motors.yaml `profile` per-motor dps SSOT)",
            )
        except Exception as e:
            logger.warning(f"motor profile 설정 실패: {e}")

    def _apply_position_pid(self) -> None:
        for cfg in self.motor_cfgs:
            if cfg.pid_p is None and cfg.pid_i is None and cfg.pid_d is None:
                continue
            try:
                self.driver.set_position_pid(
                    cfg.id, p=cfg.pid_p, i=cfg.pid_i, d=cfg.pid_d
                )
                self.log(
                    "info",
                    f"모터 {cfg.id}({cfg.name}) PID 적용 "
                    f"P={cfg.pid_p} I={cfg.pid_i} D={cfg.pid_d}",
                )
            except Exception as e:
                logger.error(f"모터 {cfg.id} PID 적용 실패: {e}")

    # ─── Publishers ──────────────────────────────────────────

    def _state_loop(self) -> None:
        interval = 1.0 / STATE_PUBLISH_HZ
        while self._running:
            if self.connected:
                self._publish_state()
            time.sleep(interval)

    def _publish_state(self) -> None:
        try:
            positions = self.driver.get_present_positions()
            loads = self.driver.get_present_loads()
            joints: list[MotorJoint] = []
            for cfg in self.motor_cfgs:
                raw = positions.get(cfg.id)
                if raw is None:
                    logger.warning(f"모터 {cfg.id}({cfg.name}) 위치 읽기 실패")
                    continue
                joints.append(
                    MotorJoint(
                        id=cfg.id,
                        name=cfg.name,
                        position=raw,
                        degree=raw_to_deg(raw),
                        velocity=0.0,
                        torque=0.0,
                        load=loads.get(cfg.id, 0),
                    )
                )
            self.publish(
                self.r(Topic.MOTOR_STATE_JOINT),
                MotorJointState(timestamp=time.time(), joints=joints),
            )
        except Exception as e:
            logger.error(f"상태 발행 오류: {e}")

    # ─── Subscribers ─────────────────────────────────────────

    def _on_cmd_joint(self, cmd: MotorCmd) -> None:
        if not self.connected:
            return
        try:
            positions = {j.id: int(j.position) for j in cmd.joints}
            if positions:
                self.driver.set_goal_positions_sync(positions)
        except Exception as e:
            logger.error(f"joint 명령 처리 오류: {e}")

    # ─── Services ────────────────────────────────────────────

    def _srv_enable(
        self, req: ServiceRequest[MotorEnableReq]
    ) -> ServiceResponse[MotorEnableRes]:
        enable = req.data.enable
        try:
            if enable:
                self.driver.torque_enable_all()
            else:
                self.driver.torque_disable_all()
            self.torque_enabled = enable
            self.log("info", f"토크 {'ON' if enable else 'OFF'}")
            return ServiceResponse(
                success=True, message="ok", data=MotorEnableRes(enable=enable)
            )
        except Exception as e:
            return ServiceResponse(success=False, message=str(e), data=None)

    def _srv_reboot(
        self, req: ServiceRequest[MotorRebootReq]
    ) -> ServiceResponse[EmptyData]:
        motor_id = req.data.id
        try:
            if motor_id:
                self.driver.reboot(motor_id)
            else:
                for mid in self.driver.motor_ids:
                    self.driver.reboot(mid)
            # reboot은 profile_velocity/acceleration도 리셋(=0). per-motor default 복원.
            target_ids = None if motor_id is None else [motor_id]
            self._apply_profiles(target_ids)
            return ServiceResponse(success=True, message="ok", data=EmptyData())
        except Exception as e:
            return ServiceResponse(success=False, message=str(e), data=None)

    def _srv_set_profile(
        self, req: ServiceRequest[MotorSetProfileReq]
    ) -> ServiceResponse[EmptyData]:
        d = req.data
        try:
            if d.velocity is not None:
                self.driver.set_profile_velocity(d.id, int(d.velocity))
            if d.acceleration is not None:
                self.driver.set_profile_acceleration(d.id, int(d.acceleration))
            return ServiceResponse(success=True, message="ok", data=EmptyData())
        except Exception as e:
            return ServiceResponse(success=False, message=str(e), data=None)

    def _srv_set_profile_all(
        self, req: ServiceRequest[MotorSetProfileAllReq]
    ) -> ServiceResponse[EmptyData]:
        d = req.data
        target_ids = d.ids if d.ids is not None else list(self.driver.motor_ids)
        try:
            if d.restore_defaults:
                # 각 모터의 motors.yaml `profile` 적용 (dps SSOT).
                self._apply_profiles(target_ids)
            else:
                # 단일 raw 값 일괄 적용 (release sentinel 0,0 / 임시 override).
                vel_map = {mid: d.velocity for mid in target_ids}
                acc_map = {mid: d.acceleration for mid in target_ids}
                self.driver.set_profile_accelerations_sync(acc_map)
                self.driver.set_profile_velocities_sync(vel_map)
            return ServiceResponse(success=True, message="ok", data=EmptyData())
        except Exception as e:
            return ServiceResponse(success=False, message=str(e), data=None)

    def _srv_get_config(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[MotorGetConfigRes]:
        motors = [
            MotorConfigItem(
                id=cfg.id,
                name=cfg.name,
                model=cfg.model,
                mode=cfg.mode,
                kind=cfg.kind.value,
                home=cfg.home,
                limit=MotorLimit(min=cfg.limit_min, max=cfg.limit_max),
            )
            for cfg in self.motor_cfgs
        ]
        return ServiceResponse(
            success=True,
            message="ok",
            data=MotorGetConfigRes(
                motors=motors, torque_enabled=self.torque_enabled
            ),
        )

    def _srv_gripper(
        self, req: ServiceRequest[MotorGripperReq]
    ) -> ServiceResponse[EmptyData]:
        d = req.data
        # 객체별 셋업 (paper_cup vs cube 등) 에서 raw position override 가능.
        # None 이면 default (open=2600 / close=1800).
        if d.position is not None:
            raw = int(d.position)
        else:
            raw = GRIPPER_OPEN_RAW if d.action == "open" else GRIPPER_CLOSE_RAW

        self.driver.set_goal_current(self._gripper_cfg.id, int(d.current))
        self.driver.set_goal_position(self._gripper_cfg.id, raw)

        return ServiceResponse(success=True, message="ok", data=EmptyData())
