import time
import logging
import threading

from core.transport.base_node import BaseNode
from core.transport.topic_map import Topic, Service
from core.units import raw_to_deg
from core.common import GRIPPER_ID
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
from modules.motor.motor_config import load_motor_config
from modules.motor.adapters.dynamixel_backend import DynamixelBackend
from modules.motor.backend import MotorCommError

logger = logging.getLogger(__name__)

STATE_PUBLISH_HZ = 20  # 초당 상태 발행 횟수

# Gripper 관련 상수
GRIPPER_OPEN_RAW = 2600
GRIPPER_CLOSE_RAW = 1800  # current 제한이 있으므로 여유있게
GRIPPER_CURRENT_DEFAULT = 200  # mA, 기본 파지력

# Gripper 부드러운 동작 — start() 시 한 번 설정해서 영구 적용.
# Dynamixel default = 0 (= 최대 속도로 즉시 = "휙"). >0이면 trapezoidal ramp.
# profile_velocity 단위: 0.229 rpm (XL 시리즈) → 80 ≈ 18 rpm = full stroke ~1.5s
# profile_acceleration 단위: 214.577 rpm/s²
GRIPPER_PROFILE_VELOCITY = 80
GRIPPER_PROFILE_ACCELERATION = 30


class MotorNode(BaseNode):
    def __init__(self):
        super().__init__("motor_node")

        port_cfg, motors = load_motor_config()
        self.port = port_cfg.get()
        self.motor_cfgs = motors
        self.driver = DynamixelBackend(self.port, motors)
        self.connected = False
        self.torque_enabled = False

        self._state_thread: threading.Thread | None = None

        self.create_subscriber(Topic.MOTOR_CMD_JOINT, MotorCmd, self._on_cmd_joint)
        self.create_service(
            Service.MOTOR_ENABLE, MotorEnableReq, MotorEnableRes, self._srv_enable
        )
        self.create_service(
            Service.MOTOR_REBOOT, MotorRebootReq, EmptyData, self._srv_reboot
        )
        self.create_service(
            Service.MOTOR_SET_PROFILE,
            MotorSetProfileReq,
            EmptyData,
            self._srv_set_profile,
        )
        self.create_service(
            Service.MOTOR_SET_PROFILE_ALL,
            MotorSetProfileAllReq,
            EmptyData,
            self._srv_set_profile_all,
        )
        self.create_service(
            Service.MOTOR_GET_CONFIG, EmptyData, MotorGetConfigRes, self._srv_get_config
        )
        self.create_service(
            Service.MOTOR_GRIPPER, MotorGripperReq, EmptyData, self._srv_gripper
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
            self._apply_gripper_smooth_profile()
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

    def _apply_gripper_smooth_profile(self) -> None:
        try:
            self.driver.set_profile_velocity(
                GRIPPER_ID, GRIPPER_PROFILE_VELOCITY
            )
            self.driver.set_profile_acceleration(
                GRIPPER_ID, GRIPPER_PROFILE_ACCELERATION
            )
            logger.info(
                "그리퍼 부드러운 profile 적용: vel=%d acc=%d",
                GRIPPER_PROFILE_VELOCITY,
                GRIPPER_PROFILE_ACCELERATION,
            )
        except Exception as e:
            logger.warning(f"그리퍼 profile 설정 실패: {e}")

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
                Topic.MOTOR_STATE_JOINT,
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
            # reboot은 그리퍼 profile_velocity/acceleration도 리셋(=0).
            # 부드러운 동작 default 복원.
            if motor_id is None or motor_id == GRIPPER_ID:
                self._apply_gripper_smooth_profile()
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

        self.driver.set_goal_current(GRIPPER_ID, int(d.current))
        self.driver.set_goal_position(GRIPPER_ID, raw)

        return ServiceResponse(success=True, message="ok", data=EmptyData())
