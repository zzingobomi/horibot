"""하드웨어 없이 motor topic/service 를 충족시키는 mock 노드 — frontend UX 검증용.

motors.yaml 의 모터 list 만 읽고 (port 연결 시도 X), MOTOR_CMD_JOINT 가 들어오면
internal position 을 즉시 갱신, MOTOR_STATE_JOINT 를 20Hz 로 발행한다.
trajectory 보간은 TrajectoryRunner 가 100Hz 로 step publish 해 주므로 mock 이
즉시 따라가도 UI 에선 부드러워 보임.

services 는 모두 success no-op — 실 hardware action 없음. 검증은 실 하드웨어에서.
"""

import logging
import threading
import time

from core.transport.base_node import BaseNode
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
from core.transport.topic_map import Service, Topic
from core.common import GRIPPER_ID
from core.units import raw_to_deg
from modules.motor.motor_config import load_motor_config

logger = logging.getLogger(__name__)

STATE_PUBLISH_HZ = 20
GRIPPER_OPEN_RAW = 2600
GRIPPER_CLOSE_RAW = 1800


class MockMotorNode(BaseNode):
    def __init__(self, robot_id: str | None = None):
        # heartbeat node name = real motor_node 와 동일 — frontend 가 mock/real
        # 무관 동일 lookup 가능 (CLAUDE.md "mock 노드는 contract 만 충족" 정합).
        super().__init__("motor_node", robot_id=robot_id)

        _port, self.motor_cfgs = load_motor_config(robot_id)
        # 초기 raw position = motors.yaml home (URDF 의 home pose 에 자연스럽게 매칭)
        self._positions: dict[int, int] = {cfg.id: int(cfg.home) for cfg in self.motor_cfgs}
        self._lock = threading.Lock()
        self.torque_enabled = True  # mock 은 항상 on (UI 가 "Torque ON" 으로 보이게)

        self._state_thread: threading.Thread | None = None

        self.create_subscriber(
            self.r(Topic.MOTOR_CMD_JOINT), MotorCmd, self._on_cmd_joint
        )
        self.create_service(
            self.r(Service.MOTOR_ENABLE),
            MotorEnableReq,
            MotorEnableRes,
            self._srv_enable,
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
            self.r(Service.MOTOR_GRIPPER),
            MotorGripperReq,
            EmptyData,
            self._srv_gripper,
        )

    # ─── Lifecycle ───────────────────────────────────────────

    def start(self) -> None:
        super().start()
        self.log("info", f"mock motor 노드 시작 (motors={len(self.motor_cfgs)})")
        self._state_thread = threading.Thread(
            target=self._state_loop,
            name="mock-motor-state",
            daemon=True,
        )
        self._state_thread.start()

    # ─── Publisher ───────────────────────────────────────────

    def _state_loop(self) -> None:
        interval = 1.0 / STATE_PUBLISH_HZ
        while self._running:
            self._publish_state()
            time.sleep(interval)

    def _publish_state(self) -> None:
        with self._lock:
            snapshot = dict(self._positions)
        joints = [
            MotorJoint(
                id=cfg.id,
                name=cfg.name,
                position=snapshot.get(cfg.id, int(cfg.home)),
                degree=raw_to_deg(snapshot.get(cfg.id, int(cfg.home))),
                velocity=0.0,
                torque=0.0,
                load=0,
            )
            for cfg in self.motor_cfgs
        ]
        self.publish(
            self.r(Topic.MOTOR_STATE_JOINT),
            MotorJointState(timestamp=time.time(), joints=joints),
        )

    # ─── Subscriber ──────────────────────────────────────────

    def _on_cmd_joint(self, cmd: MotorCmd) -> None:
        with self._lock:
            for j in cmd.joints:
                self._positions[j.id] = int(j.position)

    # ─── Services (모두 success no-op, gripper 만 internal position 갱신) ───

    def _srv_enable(
        self, req: ServiceRequest[MotorEnableReq]
    ) -> ServiceResponse[MotorEnableRes]:
        self.torque_enabled = bool(req.data.enable)
        return ServiceResponse(
            success=True,
            message="mock ok",
            data=MotorEnableRes(enable=self.torque_enabled),
        )

    def _srv_reboot(
        self, _req: ServiceRequest[MotorRebootReq]
    ) -> ServiceResponse[EmptyData]:
        return ServiceResponse(success=True, message="mock ok", data=EmptyData())

    def _srv_set_profile(
        self, _req: ServiceRequest[MotorSetProfileReq]
    ) -> ServiceResponse[EmptyData]:
        return ServiceResponse(success=True, message="mock ok", data=EmptyData())

    def _srv_set_profile_all(
        self, _req: ServiceRequest[MotorSetProfileAllReq]
    ) -> ServiceResponse[EmptyData]:
        return ServiceResponse(success=True, message="mock ok", data=EmptyData())

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
            message="mock ok",
            data=MotorGetConfigRes(motors=motors, torque_enabled=self.torque_enabled),
        )

    def _srv_gripper(
        self, req: ServiceRequest[MotorGripperReq]
    ) -> ServiceResponse[EmptyData]:
        if req.data.position is not None:
            raw = int(req.data.position)
        else:
            raw = GRIPPER_OPEN_RAW if req.data.action == "open" else GRIPPER_CLOSE_RAW
        with self._lock:
            self._positions[GRIPPER_ID] = raw
        return ServiceResponse(success=True, message="mock ok", data=EmptyData())
