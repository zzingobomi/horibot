import time
import logging
import threading

from core.base_node import BaseNode
from core.topic_map import Topic, Service
from core.units import raw_to_deg
from core.common import GRIPPER_ID
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

        self.create_subscriber(Topic.MOTOR_CMD_JOINT, self._on_cmd_joint)
        self.create_service(Service.MOTOR_ENABLE, self._srv_enable)
        self.create_service(Service.MOTOR_REBOOT, self._srv_reboot)
        self.create_service(Service.MOTOR_SET_PROFILE, self._srv_set_profile)
        self.create_service(Service.MOTOR_SET_PROFILE_ALL,
                            self._srv_set_profile_all)
        self.create_service(Service.MOTOR_GET_CONFIG, self._srv_get_config)
        self.create_service(Service.MOTOR_GRIPPER, self._srv_gripper)

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
            joints = []
            for cfg in self.motor_cfgs:
                raw = positions.get(cfg.id)
                if raw is None:
                    logger.warning(f"모터 {cfg.id}({cfg.name}) 위치 읽기 실패")
                    continue
                joints.append(
                    {
                        "id": cfg.id,
                        "name": cfg.name,
                        "position": raw,
                        "degree": raw_to_deg(raw),
                        "velocity": 0.0,
                        "torque": 0.0,
                        "load": loads.get(cfg.id, 0),
                    }
                )
            self.publish(
                Topic.MOTOR_STATE_JOINT,
                {
                    "timestamp": time.time(),
                    "joints": joints,
                },
            )
        except Exception as e:
            logger.error(f"상태 발행 오류: {e}")

    # ─── Subscribers ─────────────────────────────────────────

    def _on_cmd_joint(self, data: dict) -> None:
        if not self.connected:
            return
        try:
            joints = data.get("joints", [])
            positions = {
                j["id"]: int(j["position"])
                for j in joints
                if "id" in j and "position" in j
            }
            if positions:
                self.driver.set_goal_positions_sync(positions)
        except Exception as e:
            logger.error(f"joint 명령 처리 오류: {e}")

    # ─── Services ────────────────────────────────────────────

    def _srv_enable(self, req: dict) -> dict:
        enable: bool = req.get("data", {}).get("enable", True)
        try:
            if enable:
                self.driver.torque_enable_all()
            else:
                self.driver.torque_disable_all()
            self.torque_enabled = enable
            self.log("info", f"토크 {'ON' if enable else 'OFF'}")
            return {"success": True, "message": "ok", "data": {"enable": enable}}
        except Exception as e:
            return {"success": False, "message": str(e), "data": {}}

    def _srv_reboot(self, req: dict) -> dict:
        motor_id: int | None = req.get("data", {}).get("id")
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
            return {"success": True, "message": "ok", "data": {}}
        except Exception as e:
            return {"success": False, "message": str(e), "data": {}}

    def _srv_set_profile(self, req: dict) -> dict:
        data = req.get("data", {})
        motor_id = data.get("id")
        velocity = data.get("velocity")
        acceleration = data.get("acceleration")
        try:
            if motor_id and velocity is not None:
                self.driver.set_profile_velocity(motor_id, int(velocity))
            if motor_id and acceleration is not None:
                self.driver.set_profile_acceleration(
                    motor_id, int(acceleration))
            return {"success": True, "message": "ok", "data": {}}
        except Exception as e:
            return {"success": False, "message": str(e), "data": {}}

    def _srv_set_profile_all(self, req: dict) -> dict:
        data = req.get("data", {})
        target_ids = data.get("ids", self.driver.motor_ids)
        velocity = int(data.get("velocity", 0))
        acceleration = int(data.get("acceleration", 0))

        try:
            vel_map = {mid: velocity for mid in target_ids}
            acc_map = {mid: acceleration for mid in target_ids}
            self.driver.set_profile_accelerations_sync(acc_map)
            self.driver.set_profile_velocities_sync(vel_map)
            return {"success": True, "message": "ok", "data": {}}
        except Exception as e:
            return {"success": False, "message": str(e), "data": {}}

    def _srv_get_config(self, req: dict) -> dict:
        configs = [
            {
                "id": cfg.id,
                "name": cfg.name,
                "model": cfg.model,
                "mode": cfg.mode,
                "home": cfg.home,
                "limit": {"min": cfg.limit_min, "max": cfg.limit_max},
            }
            for cfg in self.motor_cfgs
        ]
        return {
            "success": True,
            "message": "ok",
            "data": {"motors": configs, "torque_enabled": self.torque_enabled},
        }

    def _srv_gripper(self, req: dict) -> dict:
        data = req.get("data", {})
        action = data.get("action", "open")
        current = int(data.get("current", GRIPPER_CURRENT_DEFAULT))
        # 객체별 셋업 (self-play 의 paper_cup vs cube 등) 에서 raw position
        # override 가능. None 이면 default (open=2600 / close=1800).
        position_override = data.get("position")

        if action not in ("open", "close"):
            return {
                "success": False,
                "message": f"action은 open/close만 허용: {action}",
                "data": {},
            }

        if position_override is not None:
            raw = int(position_override)
        else:
            raw = GRIPPER_OPEN_RAW if action == "open" else GRIPPER_CLOSE_RAW

        self.driver.set_goal_current(GRIPPER_ID, current)
        self.driver.set_goal_position(GRIPPER_ID, raw)

        return {"success": True, "message": "ok", "data": {}}
