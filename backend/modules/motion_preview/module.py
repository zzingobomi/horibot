"""MotionPreviewModule — plan-only 궤적 미리보기 (POC).

motion 을 거치지 않는다 (wire/런타임/코드 미접촉). motion 의 kinematics 와
TrajectoryRunner 를 **라이브러리로 import** 재사용할 뿐 — scan 이
build_calibrated_kinematics 를 소비하는 것과 같은 공용 패턴. IK/Ruckig 를 새로
구현하면 SSOT 이중화라 금지.

핵심 트릭: TrajectoryRunner 는 콜백 DI 라, `publish_cmd` 를 "모터로 발행" 대신
"리스트에 수집" 으로 바꿔 끼우면 그대로 궤적 프레임 생성기가 된다. `solve_ik` 는
자기 kinematics, profile 토글은 no-op (실 모터 없음). 실행부(모터)와 공유하는 게
없어 motion 런타임과 완전히 격리된다.

**robot-agnostic** — host 당 1 인스턴스. 자기 kinematics 를 robot_id 별로 lazy
빌드/캐시 (bundle=None = 무보정 ideal URDF — POC 1단계. 2단계에서 calibration
Mirror 를 붙이면 motion 과 같은 보정 경로가 되어 프리뷰가 실 경로를 대변).
"""

from __future__ import annotations

import asyncio
import logging
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.motion.contract import TrajStatus
from modules.motion.kinematics import Kinematics, Position3, Quaternion
from modules.motion.kinematics_builder import build_calibrated_kinematics
from modules.motion.trajectory_runner import LinearPath, TrajectoryRunner
from modules.motor.layout import MotorSpec

from .contract import (
    MotionPreview,
    PlanPreviewRequest,
    PlanPreviewResponse,
    PreviewMode,
    PreviewPoseTarget,
)

logger = logging.getLogger(__name__)

# 궤적 실시간 수집 상한 — 정상 이동은 수 초. 초과 = 러너가 종료 신호를 못 준 것
# (방어). 서비스 timeout(60s)보다 짧게 잡아 여기서 먼저 손절.
_COLLECT_TIMEOUT_S = 55.0


@dataclass(frozen=True)
class PreviewRobotSpec:
    """robot 별 정적 config — resolve 가 robots.yaml/URDF/motion.yaml 에서 투영.

    wire contract 아님 (constructor dep). callable 을 들고 있어 dataclass
    (scan 의 ScanRobotSpec / motion deps 와 같은 role). motion 과 같은 값을 쓰되
    주입 출처만 preview 전용 — motion 인스턴스를 참조하지 않는다.
    """

    kinematics_factory: Callable[[Path], Kinematics]
    urdf_path: Path
    arm_specs: list[MotorSpec]  # motors.yaml 순 (gripper 제외)
    joint_max_velocity: list[float]
    joint_max_acceleration: list[float]
    joint_max_jerk: list[float]
    cartesian_max_velocity: float
    cartesian_max_acceleration: float
    cartesian_max_jerk: float


def _rpy_to_quat(rpy_deg: tuple[float, float, float]) -> Quaternion:
    """intrinsic XYZ 오일러(도) → quaternion [x,y,z,w]. contract 규약 SSOT."""
    q = Rotation.from_euler("XYZ", rpy_deg, degrees=True).as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _diagnose_and_log(
    kin: Kinematics,
    pos: Position3,
    quat: Quaternion | None,
    seed: list[float],
    *,
    robot_id: str,
    mode: PreviewMode,
    frames_n: int,
) -> str:
    """도달 불가 사유를 분해해 로그로 남기고 사용자 메시지를 반환 (관측성 안전망).

    "6축인데 왜 안 되나" 를 매번 답하려면 실패가 어느 종류인지 데이터로 남아야
    한다 (motion jog_tcp 진단 패턴). 목표를 position-only IK vs 자세 포함 IK 로
    각각 찔러 원인을 가른다:
      - 위치 IK 실패          → workspace 밖 (진짜 도달 불가)
      - 위치 OK, 자세 IK 실패 → orientation blocker (자세 제약 — '위치만'이면 도달)
      - 둘 다 OK (=끝점 OK)    → MoveL 직선 경로 중간 실패 (path)
    """
    pos_only_ok = kin.ik(pos, None, seed) is not None
    with_ori_ok = quat is None or kin.ik(pos, quat, seed) is not None
    if not pos_only_ok:
        reason = "위치가 도달 범위 밖 (workspace 초과 — 진짜 도달 불가)"
    elif not with_ori_ok:
        reason = (
            "위치는 되나 그 자세(orientation)로는 IK 불가 — 자세 제약 "
            "('위치만' 모드면 도달 가능)"
        )
    else:
        reason = (
            "끝점은 도달 가능하나 직선 경로 중간에서 IK 불가 "
            "(MoveJ(pose) 는 가능할 수 있음)"
        )
    logger.info(
        "preview INFEASIBLE robot=%s mode=%s use_ori=%s | %s | "
        "pos=%s quat=%s pos_only_ok=%s with_ori_ok=%s frames=%d",
        robot_id or "?",
        mode.value,
        quat is not None,
        reason,
        [round(float(v), 4) for v in pos],
        None if quat is None else [round(float(v), 4) for v in quat],
        pos_only_ok,
        with_ori_ok,
        frames_n,
    )
    return reason


def plan_trajectory(
    kin: Kinematics,
    spec: PreviewRobotSpec,
    start_joints: list[float],
    target: PreviewPoseTarget,
    mode: PreviewMode,
    use_orientation: bool = True,
    robot_id: str = "",
) -> PlanPreviewResponse:
    """궤적을 실시간 수집해 관절 프레임 + TCP 트레이스로 반환 (blocking).

    TrajectoryRunner 를 수집 sink 로 재사용 — motion 의 move_l/move_j(pose) 실행부와
    같은 planner·IK·seed 연쇄라 프레임이 실 경로와 동일. IK 실패(도달 불가)만
    feasible=False; 구성 플립(wrist flip)은 프레임에 그대로 남아 애니메이션으로 보임
    (Viewer — 거부/분석 안 함). 호출자가 to_thread 책임 (runner 스레드 + IK blocking).

    use_orientation: True = RPY 를 목표 자세로 (MoveL=현재→목표 slerp, MoveJ=그 자세
      도달). False = position-only (자세 자유 — IK 가 seed 근처 자세를 알아서). motion
      PoseTarget.quaternion None/set 을 그대로 노출 (모션 계약 무변경, §2×2 자세 축).
    """
    quat = _rpy_to_quat(target.rpy_deg) if use_orientation else None
    pos = target.position
    joint_names = [s.name for s in spec.arm_specs]

    frames: list[list[float]] = []
    done_ev = threading.Event()
    status_box: dict[str, TrajStatus] = {}

    def _collect(angles: list[float]) -> None:
        frames.append(list(angles))

    def _on_state(status: TrajStatus, _progress: float) -> None:
        if status in (TrajStatus.DONE, TrajStatus.FAILED, TrajStatus.STOPPED):
            status_box["s"] = status
            done_ev.set()

    def _solve_ik(
        p: Position3, q: Quaternion | None, seed: list[float]
    ) -> list[float] | None:
        return kin.ik(p, q, seed)

    runner = TrajectoryRunner(
        n_arm=len(spec.arm_specs),
        joint_max_velocity=spec.joint_max_velocity,
        joint_max_acceleration=spec.joint_max_acceleration,
        joint_max_jerk=spec.joint_max_jerk,
        cartesian_max_velocity=spec.cartesian_max_velocity,
        cartesian_max_acceleration=spec.cartesian_max_acceleration,
        cartesian_max_jerk=spec.cartesian_max_jerk,
        release_profile=lambda: True,  # 실 모터 없음 — no-op
        restore_profile=lambda: True,
        publish_cmd=_collect,  # 모터 대신 수집
        publish_state=_on_state,
        solve_ik=_solve_ik,
        get_joint_angles=lambda: start_joints,
    )

    if mode == PreviewMode.MOVE_J_POSE:
        # 목표 pose IK 1회 → 관절 보간. 도달 불가면 프레임 0 (모션 없음).
        sol = kin.ik(pos, quat, start_joints)
        if sol is None:
            return PlanPreviewResponse(
                feasible=False,
                joint_names=joint_names,
                frames=[],
                tcp_trace=[],
                fail_at_sample=0,
                message=_diagnose_and_log(
                    kin, pos, quat, start_joints,
                    robot_id=robot_id, mode=mode, frames_n=0,
                ),
            )
        runner.run_joint(start_joints, sol)
    else:  # MOVE_L — TCP 직선. 현재 자세 → 목표 자세 slerp (motion move_l 동형).
        start_pos, start_quat = kin.fk(start_joints)
        path = LinearPath(
            np.asarray(start_pos, dtype=float),
            np.asarray(pos, dtype=float),
        )
        runner.run_cartesian(path, start_joints, quat, start_quat)

    if not done_ev.wait(timeout=_COLLECT_TIMEOUT_S):
        runner.stop()
        logger.warning("preview 궤적 수집 timeout — 부분 프레임 반환")
    runner.stop()

    status = status_box.get("s", TrajStatus.FAILED)
    feasible = status == TrajStatus.DONE
    # 프레임별 FK → TCP 트레이스 (두 모드 통일 — MoveL 은 직선, MoveJ 는 곡선).
    tcp_trace = [
        tuple(float(v) for v in kin.fk(f)[0]) for f in frames
    ]
    fail_at = None if feasible else len(frames)
    msg = (
        ""
        if feasible
        else _diagnose_and_log(
            kin, pos, quat, start_joints,
            robot_id=robot_id, mode=mode, frames_n=len(frames),
        )
    )
    return PlanPreviewResponse(
        feasible=feasible,
        joint_names=joint_names,
        frames=frames,
        tcp_trace=tcp_trace,  # type: ignore[arg-type]
        fail_at_sample=fail_at,
        message=msg,
    )


class MotionPreviewModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        robots: dict[str, PreviewRobotSpec],
    ) -> None:
        self.runtime = runtime
        self._robots = robots
        # robot_id → kinematics (lazy — 첫 요청 시 빌드/캐시. PyBullet init 은 무거움).
        self._kin: dict[str, Kinematics] = {}
        self._kin_lock = asyncio.Lock()

    async def start(self) -> None:
        # 상주 자원 없음 — kinematics 는 첫 plan 요청 시 lazy 빌드.
        pass

    async def stop(self) -> None:
        for kin in self._kin.values():
            try:
                kin.close()
            except Exception:  # noqa: BLE001 — 종료 정리, best-effort
                pass
        self._kin.clear()

    @service(MotionPreview.Service.PLAN)
    async def plan(self, req: PlanPreviewRequest) -> PlanPreviewResponse:
        """plan-only 미리보기 — 로봇 안 움직임. 궤적 프레임 + TCP 트레이스 반환.

        blocking (kinematics 빌드 + runner 실시간 수집 + IK) → to_thread 로 event
        loop 를 안 막는다 (async 계약)."""
        spec = self._robots.get(req.robot_id)
        if spec is None:
            raise RuntimeError(
                f"preview 미지원 robot: {req.robot_id!r} "
                f"(motion 한계/URDF 미설정 — robots.yaml/motion.yaml 확인)"
            )
        dof = len(spec.arm_specs)
        if len(req.start_joints) != dof:
            raise RuntimeError(
                f"start_joints dof 불일치 ({len(req.start_joints)} != {dof})"
            )
        kin = await self._get_kinematics(req.robot_id, spec)
        result = await asyncio.to_thread(
            plan_trajectory,
            kin,
            spec,
            list(req.start_joints),
            req.target,
            req.mode,
            req.use_orientation,
            req.robot_id,
        )
        # 클릭 1번 = preview 로그 1줄 (성공/실패 모두) — 어느 모드를 눌렀고 결과가
        # 무엇인지 매칭 가능하게. 실패는 plan_trajectory 가 INFEASIBLE 상세도 별도로
        # 남긴다 (pos_only/with_ori 분해). 성공은 이 줄이 유일한 preview 로그.
        logger.info(
            "preview robot=%s mode=%s use_ori=%s → %s frames=%d",
            req.robot_id,
            req.mode.value,
            req.use_orientation,
            "OK" if result.feasible else "INFEASIBLE",
            len(result.frames),
        )
        return result

    async def _get_kinematics(
        self, robot_id: str, spec: PreviewRobotSpec
    ) -> Kinematics:
        async with self._kin_lock:
            kin = self._kin.get(robot_id)
            if kin is not None:
                return kin
            kin = await asyncio.to_thread(self._build_kinematics, robot_id, spec)
            self._kin[robot_id] = kin
            return kin

    def _build_kinematics(
        self, robot_id: str, spec: PreviewRobotSpec
    ) -> Kinematics:
        """무보정 ideal URDF kinematics 빌드 (blocking — POC 1단계, bundle=None).

        공용 빌더 재사용 (motion/scan 과 같은 SSOT) — 2단계에서 calibration Mirror
        의 bundle 을 넘기면 motion 과 같은 보정 경로가 된다 (여기만 바뀜)."""
        built = build_calibrated_kinematics(
            spec.urdf_path, robot_id, spec.arm_specs, None, spec.kinematics_factory
        )
        built.kinematics.initialize()
        logger.info("preview kinematics 빌드 robot=%s (무보정)", robot_id)
        return built.kinematics
