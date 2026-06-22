import time
from abc import ABC, abstractmethod
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from core.units import deg_to_rad
from modules.kinematics.kinematics import Position3, Quaternion
from modules.kinematics.trajectory_runner import (
    ArcPath,
    LinearPath,
    SplinePath,
    TrajectoryRunner,
)

# Servo (target chase) — caller 가 절대 target → planner 우회 IK + direct publish.
# 의존성 주입: motion_modes.servo_tcp (6DOF) + cmd publish.
ServoSolveFn = Callable[
    [Position3, Quaternion | None, list[float]], list[float] | None
]
PublishCmdFn = Callable[[list[float]], None]
# Jog 의 ref latch 자리 — backend 가 자기 process joint_cache 의 URDF rad
# 자리 받음. caller (motion_node) 가 wrap.
GetCurrentJointsFn = Callable[[], list[float] | None]
# Jog 의 fk 자리 — joint URDF rad → (position, quaternion). motion_modes.get_tcp_pose
# 가 SagCorrectedKinematics 통해 sag 보정된 FK.
FkFn = Callable[[list[float]], tuple[np.ndarray, np.ndarray]]

# Jog 의 idle 자리 — 마지막 publish 후 본 시간 넘으면 다음 publish 자리에서
# joint_cache 로 fresh latch. button up 후 모터 settled 자리에 다시 hold 시
# 인코더 - ref 누적 drift 차단. (frontend setInterval 20ms × ~10 cycle 분)
JOG_IDLE_RESET_S = 0.2


class MotionCommand(ABC):
    @abstractmethod
    def validate(self, req: dict) -> str | None:
        ...

    @abstractmethod
    def execute(
        self,
        req:     dict,
        angles:  list[float],
        tcp_pos: list[float],
        runner:  TrajectoryRunner,
    ) -> None:
        ...

    @property
    def label(self) -> str:
        return self.__class__.__name__.replace("Command", "")


class MoveJCommand(MotionCommand):
    def __init__(self, arm_cfgs) -> None:
        self._arm_cfgs = arm_cfgs

    def validate(self, req: dict) -> str | None:
        if not req.get("data", {}).get("joints"):
            return "joints 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        target_by_id = {
            int(j["id"]): float(j["degree"])
            for j in req["data"]["joints"]
        }
        target_angles = [
            deg_to_rad(target_by_id.get(cfg.id, 0.0))
            for cfg in self._arm_cfgs
        ]
        runner.run_joint(angles, target_angles)


class MoveLCommand(MotionCommand):
    def validate(self, req: dict) -> str | None:
        if req.get("data", {}).get("position") is None:
            return "position 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        start = np.array(tcp_pos, dtype=float)
        end = np.array(req["data"]["position"], dtype=float)
        runner.run_cartesian(LinearPath(start, end), angles)


class MoveCCommand(MotionCommand):
    def validate(self, req: dict) -> str | None:
        data = req.get("data", {})
        if data.get("via") is None or data.get("end") is None:
            return "via, end 모두 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        data = req["data"]
        p1 = np.array(tcp_pos,       dtype=float)
        p2 = np.array(data["via"],   dtype=float)
        p3 = np.array(data["end"],   dtype=float)
        runner.run_cartesian(ArcPath(p1, p2, p3), angles)


class MovePCommand(MotionCommand):
    def validate(self, req: dict) -> str | None:
        wps = req.get("data", {}).get("waypoints", [])
        if len(wps) < 2:
            return "waypoints 최소 2개 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        wps = req["data"]["waypoints"]
        all_pts = np.array([tcp_pos] + [list(wp) for wp in wps], dtype=float)
        runner.run_cartesian(SplinePath(all_pts), angles)


class ServoTcpCommand(MotionCommand):
    """절대 TCP target → planner 우회 직접 IK + publish (chase 패턴).

    caller (gamepad/외부 컨트롤러) 가 빠른 rate (50Hz+) 로 갱신 시 자연스러운 추종.
    Ruckig 트래젝토리 없음 — 1 step / 1 publish.
    Trajectory 가 돌고 있으면 stop (servo 가 trajectory 가로채는 의미).

    `quaternion=None` → position-only IK (5DOF / 6DOF orientation 무시).
    """

    def __init__(
        self,
        solve_servo: ServoSolveFn,
        publish_cmd: PublishCmdFn,
    ) -> None:
        self._solve_servo = solve_servo
        self._publish_cmd = publish_cmd

    def validate(self, req: dict) -> str | None:
        data = req.get("data", {})
        if data.get("position") is None:
            return "position 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        data = req["data"]
        position = tuple(data["position"])  # type: ignore[assignment]
        quat_raw = data.get("quaternion")
        quaternion = tuple(quat_raw) if quat_raw is not None else None  # type: ignore[assignment]

        # Trajectory 가 돌고 있으면 stop — servo 가 새 chase 자리 진입.
        if runner.is_running:
            runner.stop()

        result = self._solve_servo(position, quaternion, angles)  # type: ignore[arg-type]
        if result is None:
            raise ValueError("IK 수렴 실패")
        self._publish_cmd(result)


class ServoJCommand(MotionCommand):
    """Servo (target chase) — 절대 joint target → planner 우회 직접 publish.

    Caller (RL replay / external trajectory player) 가 *자기가 계산한* 절대
    joint URDF rad target 자리 보냄. server = direct publish (IK 불요,
    joint-space). UR `servoj` / KUKA RSI joint 자리 정석.
    Trajectory 가 돌고 있으면 stop.
    """

    def __init__(
        self,
        publish_cmd: PublishCmdFn,
        n_arm: int,
    ) -> None:
        self._publish_cmd = publish_cmd
        self._n_arm = n_arm

    def validate(self, req: dict) -> str | None:
        positions = req.get("data", {}).get("positions")
        if positions is None:
            return "positions 필요"
        if len(positions) != self._n_arm:
            return f"positions 길이 {len(positions)} != arm dof {self._n_arm}"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        positions = list(req["data"]["positions"])
        if runner.is_running:
            runner.stop()
        self._publish_cmd(positions)


class JogJCommand(MotionCommand):
    """Jog (human/manual velocity) — joint-space velocity input.

    caller (frontend Jog UI / gamepad pendant) 가 50Hz 로 *velocity 만* 보냄.
    backend 가:
      1. 첫 publish 또는 JOG_IDLE_RESET_S 이상 끊긴 자리 → joint_cache 에서
         fresh latch (URDF rad, joint_offset 적용). 인코더 - ref 누적 drift 차단.
      2. 실 측정 dt 로 `ref += velocity * dt` → publish_cmd(ref).

    LeRobot delta-pose 패턴의 joint-space 등가 — IK 불요, Ruckig 안 끼는
    direct publish. 모터 trapezoidal profile 이 추종.

    Trajectory 가 돌고 있으면 stop.
    Stateful — motion_node 가 단일 인스턴스 보유.
    """

    def __init__(
        self,
        publish_cmd: PublishCmdFn,
        get_current_joints: GetCurrentJointsFn,
        n_arm: int,
    ) -> None:
        self._publish_cmd = publish_cmd
        self._get_current_joints = get_current_joints
        self._n_arm = n_arm
        self._last_cmd: list[float] | None = None
        self._last_publish_t = 0.0

    def validate(self, req: dict) -> str | None:
        velocities = req.get("data", {}).get("velocities")
        if velocities is None:
            return "velocities 필요"
        if len(velocities) != self._n_arm:
            return (
                f"velocities 길이 {len(velocities)} != arm dof {self._n_arm}"
            )
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        velocities = list(req["data"]["velocities"])
        now = time.time()

        if runner.is_running:
            runner.stop()
            self._last_cmd = None

        idle_too_long = (
            self._last_cmd is None
            or (now - self._last_publish_t) > JOG_IDLE_RESET_S
        )

        if idle_too_long:
            current = self._get_current_joints()
            if current is None:
                raise ValueError("joint_cache 비어있음 — motor state 수신 전")
            if len(current) != self._n_arm:
                raise ValueError(
                    f"joint_cache dof {len(current)} != arm dof {self._n_arm}"
                )
            self._last_cmd = list(current)
            self._last_publish_t = now
            self._publish_cmd(list(self._last_cmd))
            return

        assert self._last_cmd is not None
        dt = now - self._last_publish_t
        for i in range(self._n_arm):
            self._last_cmd[i] += velocities[i] * dt
        self._last_publish_t = now
        self._publish_cmd(list(self._last_cmd))


class JogTcpCommand(MotionCommand):
    """Jog (human/manual velocity) — Cartesian twist input + SE(3) 적분.

    Caller (frontend Jog UI / gamepad) 는 *twist (linear + angular + frame) 만*
    보냄. backend 가:
      1. 첫 publish 또는 JOG_IDLE_RESET_S 이상 끊긴 자리 → joint_cache → fk 로
         URDF EE pose fresh latch.
      2. 실 측정 dt 로 SE(3) 적분:
         - linear, frame="base": pos_real += linear * dt
         - linear, frame="tcp" : pos_real += R(quat) @ linear * dt
         - angular: scipy Rotation.from_rotvec(angular * dt), base 자리 premultiply,
           tcp 자리 postmultiply.
      3. IK target = pos_real, quat_real → IK.
      4. publish_cmd(joint_target).

    모든 caller 가 같은 wire — SE(3) 적분 SSOT = backend (frontend Three.js /
    Python gamepad 자리 중복 회피).
    Trajectory 가 돌고 있으면 stop.
    Stateful — motion_node 가 단일 인스턴스 보유.
    """

    def __init__(
        self,
        solve_servo: ServoSolveFn,
        publish_cmd: PublishCmdFn,
        get_current_joints: GetCurrentJointsFn,
        fk: FkFn,
    ) -> None:
        self._solve_servo = solve_servo
        self._publish_cmd = publish_cmd
        self._get_current_joints = get_current_joints
        self._fk = fk
        # ref state — URDF EE pose.
        self._last_pos: np.ndarray | None = None  # (3,) m
        self._last_quat: np.ndarray | None = None  # (4,) [x,y,z,w]
        self._last_publish_t = 0.0

    def validate(self, req: dict) -> str | None:
        data = req.get("data", {})
        if data.get("linear") is None or data.get("angular") is None:
            return "linear, angular 모두 필요"
        if len(data["linear"]) != 3 or len(data["angular"]) != 3:
            return "linear / angular 길이 3"
        frame = data.get("frame", "base")
        if frame not in ("base", "tcp"):
            return f"frame='{frame}' invalid — 'base' 또는 'tcp'"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        data = req["data"]
        linear = np.asarray(data["linear"], dtype=float)
        angular = np.asarray(data["angular"], dtype=float)
        frame = data.get("frame", "base")
        now = time.time()

        if runner.is_running:
            runner.stop()
            self._last_pos = None
            self._last_quat = None

        idle_too_long = (
            self._last_pos is None
            or self._last_quat is None
            or (now - self._last_publish_t) > JOG_IDLE_RESET_S
        )

        current_joints = self._get_current_joints()
        if current_joints is None:
            raise ValueError("joint_cache 비어있음 — motor state 수신 전")

        if idle_too_long:
            # Fresh latch — fk → URDF EE pose.
            pos_urdf, quat_urdf = self._fk(current_joints)
            last_pos = np.asarray(pos_urdf, dtype=float)
            last_quat = np.asarray(quat_urdf, dtype=float)
            self._last_pos = last_pos
            self._last_quat = last_quat
            self._last_publish_t = now
            # 첫 publish — 적분 없이 latched ref 그대로 (현재 위치).
            result = self._solve_servo(
                tuple(last_pos.tolist()),
                tuple(last_quat.tolist()),
                current_joints,
            )
            if result is None:
                raise ValueError("IK 수렴 실패 (fresh latch)")
            self._publish_cmd(result)
            return

        # SE(3) 적분 — 실 측정 dt. (idle_too_long=False 분기 → last_pos/quat 보장).
        assert self._last_pos is not None and self._last_quat is not None
        prev_pos = self._last_pos
        prev_quat = self._last_quat
        dt = now - self._last_publish_t
        new_pos = prev_pos
        new_quat = prev_quat

        # Linear
        if np.any(linear):
            if frame == "base":
                new_pos = prev_pos + linear * dt
            else:  # tcp frame: world_lin = R(ref_quat) @ linear
                R = Rotation.from_quat(prev_quat)
                new_pos = prev_pos + R.apply(linear) * dt

        # Angular
        ang_mag = float(np.linalg.norm(angular))
        if ang_mag > 1e-9:
            delta_R = Rotation.from_rotvec(angular * dt)
            cur_R = Rotation.from_quat(prev_quat)
            if frame == "base":
                new_R = delta_R * cur_R  # world rotation = premultiply
            else:
                new_R = cur_R * delta_R  # tcp rotation = postmultiply
            new_quat = new_R.as_quat()

        # IK + publish — URDF target = ref pose 그대로.
        result = self._solve_servo(
            tuple(new_pos.tolist()),
            tuple(new_quat.tolist()),
            current_joints,
        )
        if result is None:
            # IK 실패 — ref 자리 *적분 전 값* 유지 (reach 한계 자리에 누적 X).
            # motor 는 마지막 valid target 머무름 자리 (자연 보호).
            raise ValueError("IK 수렴 실패")
        # commit — IK 성공 자리에만.
        self._last_pos = new_pos
        self._last_quat = new_quat
        self._last_publish_t = now
        self._publish_cmd(result)
