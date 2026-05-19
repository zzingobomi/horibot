import time
import logging
import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, Result

from core.base_node import BaseNode
from core.topic_map import Service, Topic
from core.joint_state_cache import JointStateCache
from core.common import GRIPPER_ID
from core.units import rad_to_raw
from core.joint_offsets import get_joint_offset
from modules.dynamixel.motor_config import MotorConfig, load_motor_config
from modules.kinematics.motion_modes import MotionModes
from modules.kinematics.trajectory_runner import TrajectoryRunner
from modules.kinematics.motion_commands import (
    MotionCommand,
    MoveCCommand,
    MoveJCommand,
    MoveLCommand,
    MovePCommand
)

logger = logging.getLogger(__name__)


class MotionNode(BaseNode):
    def __init__(self):
        super().__init__("motion_node")

        _, self._motor_cfgs = load_motor_config()
        self._arm_cfgs: list[MotorConfig] = [
            m for m in self._motor_cfgs if m.id != GRIPPER_ID
        ]
        self._arm_ids = [cfg.id for cfg in self._arm_cfgs]
        self._n_arm = len(self._arm_cfgs)

        self._motion = MotionModes()
        self._cache = JointStateCache()
        self._cache.subscribe(self)

        self._runner = TrajectoryRunner(
            n_arm=self._n_arm,
            set_profile=self._set_arm_profile,
            publish_cmd=self._publish_cmd,
            publish_state=self._publish_traj_state,
            move_tcp=self._motion.move_tcp,
        )

        self.create_service(Service.MOTION_GET_TCP,  self._srv_get_tcp)
        self.create_service(Service.MOTION_MOVE_TCP, self._srv_move_tcp)
        self.create_service(Service.MOTION_MOVE_J,
                            self._make_handler(MoveJCommand(self._arm_cfgs)))
        self.create_service(Service.MOTION_MOVE_L,
                            self._make_handler(MoveLCommand()))
        self.create_service(Service.MOTION_MOVE_C,
                            self._make_handler(MoveCCommand()))
        self.create_service(Service.MOTION_MOVE_P,
                            self._make_handler(MovePCommand()))
        self.create_service(Service.MOTION_STOP,     self._srv_stop)

    def _make_handler(self, cmd: MotionCommand):
        def handler(req: dict) -> dict:
            # 1. 요청 검증
            error = cmd.validate(req)
            if error:
                return {"success": False, "message": error, "data": {}}

            # 2. 관절 상태
            angles = self._cache.get_joint_angles_rad(self._arm_cfgs)
            if angles is None:
                return {"success": False, "message": "관절 상태 수신 전", "data": {}}

            # 3. 현재 TCP
            try:
                tcp_pos = list(self._motion.get_tcp_pose(angles).position)
            except Exception as e:
                return {"success": False, "message": f"FK 오류: {e}", "data": {}}

            # 4. 실행
            try:
                cmd.execute(req, angles, tcp_pos, self._runner)
                self.log("info", f"{cmd.label} 시작")
                return {"success": True, "message": "ok", "data": {}}
            except ValueError as e:
                return {"success": False, "message": str(e), "data": {}}
            except Exception as e:
                logger.error(f"{cmd.label} execute 오류: {e}")
                return {"success": False, "message": str(e), "data": {}}

        return handler

    # ─── Services ─────────────────────────────────────────────

    def _srv_get_tcp(self, req: dict) -> dict:
        angles = self._cache.get_joint_angles_rad(self._arm_cfgs)
        if angles is None:
            return {"success": False, "message": "관절 상태 수신 전", "data": {}}
        try:
            pose = self._motion.get_tcp_pose(angles)
            return {"success": True, "message": "ok",
                    "data": {"position": pose.position, "quaternion": pose.quaternion}}
        except Exception as e:
            return {"success": False, "message": str(e), "data": {}}

    def _srv_move_tcp(self, req: dict) -> dict:
        target_pos = req.get("data", {}).get("position")
        if target_pos is None:
            return {"success": False, "message": "position 필요", "data": {}}
        angles = self._cache.get_joint_angles_rad(self._arm_cfgs)
        if angles is None:
            return {"success": False, "message": "관절 상태 수신 전", "data": {}}
        try:
            result = self._motion.move_tcp(target_pos, angles)
            if result is None:
                return {"success": False, "message": "IK 수렴 실패", "data": {}}
            self._publish_cmd(result)
            return {"success": True, "message": "ok", "data": {}}
        except Exception as e:
            return {"success": False, "message": str(e), "data": {}}

    def _srv_stop(self, req: dict) -> dict:
        was_running = self._runner.is_running
        self._runner.stop()
        if was_running:
            self._publish_traj_state("stopped", 0.0)
            self.log("info", "트래젝토리 중단")
        return {"success": True, "message": "ok", "data": {}}

    # ─── Internal ────────────────────────────────────────────

    def _publish_cmd(self, angles_rad: list[float]) -> None:
        self.publish(Topic.MOTOR_CMD_JOINT, {
            "timestamp": time.time(),
            "joints": [
                {
                    "id":       cfg.id,
                    "position": rad_to_raw(
                        angle,
                        reverse=cfg.reverse,
                        min_raw=cfg.limit_min,
                        max_raw=cfg.limit_max,
                        offset_rad=get_joint_offset(idx),
                    ),
                }
                for idx, (cfg, angle) in enumerate(
                    zip(self._arm_cfgs, angles_rad)
                )
            ],
        })

    def _publish_traj_state(self, status: str, progress: float) -> None:
        self.publish(Topic.MOTION_STATE_TRAJ, {
            "status":    status,
            "progress":  round(progress, 3),
            "timestamp": time.time(),
        })

    def _set_arm_profile(self, velocity: int, acceleration: int) -> bool:
        res = self.call_service(
            Service.MOTOR_SET_PROFILE_ALL,
            {
                "ids": self._arm_ids,
                "velocity": velocity,
                "acceleration": acceleration
            },
        )
        return res.get("success", False)
