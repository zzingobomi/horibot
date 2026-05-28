import time
import logging
import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, Result

from core.base_node import BaseNode
from core.topic_map import Service, Topic
from core.joint_coordinates import JointCoordinates
from core.joint_state_cache import JointStateCache
from core.link_coordinates import LinkCoordinates
from core.common import GRIPPER_ID
from modules.dynamixel.motor_config import MotorConfig, load_motor_config
from modules.kinematics.motion_modes import MotionModes
from modules.kinematics.solver import PybulletSolver
from modules.kinematics.trajectory_runner import TrajectoryRunner
from modules.kinematics.motion_commands import (
    MotionCommand,
    MoveCCommand,
    MoveJCommand,
    MoveLCommand,
    MovePCommand
)

logger = logging.getLogger(__name__)

# Tool offset 의미 — LinkCoordinates ID=6 행을 *URDF patch 가 아닌 tool_offset 으로*
# 재해석 (urdf_patcher 는 ID=6 무시, patched URDF 에 안 적용). EE frame 기준 (link5
# +x 방향 등) 의 (실제 그리퍼 끝점 - URDF EE) 벡터. 의미:
#   실제 끝점 = URDF EE + R_be @ tool_offset_ee
# 외부 (detect, task) 가 보는 좌표는 *진짜 끝점* (user frame), motion service handler
# 가 진입/응답 시점에 URDF frame ↔ user frame 변환. solver / 캘 / BA 는 URDF frame
# 그대로 (캘 reference frame 안정성 유지).
#
# 이력: 2026-05-28 EE patch (urdf_patcher 가 end_effector_joint origin 직접 수정) 시도
# 실패. detect 와 IK 양쪽 patched URDF 위에 도는 self-consistency 로 cancel out 됨.
# 그 fix 가 이 tool_offset 변환 (한쪽만 적용 → cancel 회피).
TOOL_OFFSET_LINK_ID = 6


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
        # MoveL/C/P 는 cartesian 좌표 입출력 → tool_offset 변환 적용
        self.create_service(Service.MOTION_MOVE_L,
                            self._cartesian_handler_factory(MoveLCommand()))
        self.create_service(Service.MOTION_MOVE_C,
                            self._cartesian_handler_factory(MoveCCommand()))
        self.create_service(Service.MOTION_MOVE_P,
                            self._cartesian_handler_factory(MovePCommand()))
        self.create_service(Service.MOTION_STOP,     self._srv_stop)

    # ─── Tool offset 변환 유틸 ─────────────────────────────────
    #
    # _tool_offset_base(angles): 현재 자세에서 (실제 끝점 - URDF EE) 의 base frame 벡터.
    # MoveJ 는 joint 명령이라 tool offset 무관. MoveL / MoveC / MoveP / MoveTCP /
    # GetTCP 같이 *cartesian 좌표* 를 입출력하는 명령에만 적용.

    def _tool_offset_base(self, angles: list[float]) -> np.ndarray:
        """현재 자세의 R_be 로 tool_offset_ee 를 base frame 으로 변환.

        반환: (3,) base frame 벡터. LinkCoordinates ID=6 비어있으면 [0,0,0].
        """
        tool_ee = LinkCoordinates().get_trans(TOOL_OFFSET_LINK_ID)
        if not np.any(tool_ee):
            return np.zeros(3, dtype=np.float64)
        R_be, _ = PybulletSolver().fk_to_matrix(angles)
        return np.asarray(R_be) @ np.asarray(tool_ee)

    def _cartesian_handler_factory(self, cmd: MotionCommand):
        """MoveL / MoveC / MoveP — cartesian 좌표 입출력. tool_offset 변환 적용.

        입력 (req["data"]["position"/"via"/"end"/"waypoints"]) = user frame.
        내부 cmd.execute 에는 URDF frame 으로 변환해 전달 (= 입력 - tool_offset_base).
        시작점 tcp_pos 도 URDF frame (solver.fk 결과 그대로) 사용 → start/end 일관.
        """
        def handler(req: dict) -> dict:
            error = cmd.validate(req)
            if error:
                return {"success": False, "message": error, "data": {}}

            angles = self._cache.get_joint_angles_rad(self._arm_cfgs)
            if angles is None:
                return {"success": False, "message": "관절 상태 수신 전", "data": {}}

            try:
                tcp_pos_urdf = list(self._motion.get_tcp_pose(angles).position)
                tool_base = self._tool_offset_base(angles)
            except Exception as e:
                return {"success": False, "message": f"FK 오류: {e}", "data": {}}

            # user → URDF 변환 (in-place 수정 피하려고 dict copy)
            data = dict(req.get("data", {}) or {})
            user_pos_before = data.get("position")
            for key in ("position", "via", "end"):
                if key in data and data[key] is not None:
                    data[key] = (np.asarray(data[key], dtype=float) - tool_base).tolist()
            if "waypoints" in data and data["waypoints"] is not None:
                data["waypoints"] = [
                    (np.asarray(wp, dtype=float) - tool_base).tolist()
                    for wp in data["waypoints"]
                ]
            req_urdf = {**req, "data": data}
            logger.info(
                "[tool_offset] %s tool_base=%s  user→urdf: %s → %s",
                cmd.label, np.round(tool_base, 4).tolist(),
                user_pos_before, data.get("position"),
            )

            try:
                cmd.execute(req_urdf, angles, tcp_pos_urdf, self._runner)
                self.log("info", f"{cmd.label} 시작")
                return {"success": True, "message": "ok", "data": {}}
            except ValueError as e:
                return {"success": False, "message": str(e), "data": {}}
            except Exception as e:
                logger.error(f"{cmd.label} execute 오류: {e}")
                return {"success": False, "message": str(e), "data": {}}

        return handler

    def _make_handler(self, cmd: MotionCommand):
        """MoveJ 전용 (joint 명령 — tool offset 무관)."""
        def handler(req: dict) -> dict:
            error = cmd.validate(req)
            if error:
                return {"success": False, "message": error, "data": {}}

            angles = self._cache.get_joint_angles_rad(self._arm_cfgs)
            if angles is None:
                return {"success": False, "message": "관절 상태 수신 전", "data": {}}

            try:
                tcp_pos = list(self._motion.get_tcp_pose(angles).position)
            except Exception as e:
                return {"success": False, "message": f"FK 오류: {e}", "data": {}}

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
        """target_pos (user frame) → URDF frame → IK."""
        target_pos_user = req.get("data", {}).get("position")
        if target_pos_user is None:
            return {"success": False, "message": "position 필요", "data": {}}
        angles = self._cache.get_joint_angles_rad(self._arm_cfgs)
        if angles is None:
            return {"success": False, "message": "관절 상태 수신 전", "data": {}}
        try:
            tool_base = self._tool_offset_base(angles)
            target_pos_urdf = (
                np.asarray(target_pos_user, dtype=float) - tool_base
            ).tolist()
            result = self._motion.move_tcp(target_pos_urdf, angles)
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
        coords = JointCoordinates()
        self.publish(Topic.MOTOR_CMD_JOINT, {
            "timestamp": time.time(),
            "joints": [
                {
                    "id":       cfg.id,
                    "position": coords.urdf_to_motor(
                        angle,
                        cfg,
                        min_raw=cfg.limit_min,
                        max_raw=cfg.limit_max,
                    ),
                }
                for cfg, angle in zip(self._arm_cfgs, angles_rad)
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
