from __future__ import annotations

import asyncio
import logging
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    SnapshotBundleRequest,
)
from modules.detector.contract import (
    DetectOrientedResponse,
    DetectRequest,
    Detector,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    ResolveReachableRequest,
    ResolveReachableResponse,
    TcpPose,
)
from modules.tasks.core.context import TaskContext
from modules.tasks.core.step import step
from modules.waypoint.contract import WaypointRecord

from .. import servo
from . import primitives
from .primitives import (
    _TOP_K,
    _VIEW_MATCH_RADIUS_M,
    _nearest_within,
    _xy_dist,
    transit,
)

logger = logging.getLogger(__name__)

_OBSERVE_SETTLE_S = 0.3  # MoveJ 후 카메라 진동 안정화 대기 시간

# close 관측용 카메라 거리 후보 (가까운 거리 우선, 실패 시 fallback)
_OBSERVE_CAM_DIST_M = (0.13, 0.18)

# close 관측 카메라 고각 후보 (높은 각도부터 시도)
_OBSERVE_ELEV_DEG = (90, 75, 60, 45)

_OBSERVE_AZIM_N = 12  # 관측 자세 생성을 위한 방위 방향 개수

# close 관측 결과 신뢰 기준:
# 보이는 물체 영역이 coarse 대비 부족하면 잘못된 시점으로 판단하고 폴백
_OBSERVE_POINTS_TRUST_RATIO = 0.7


@step(title="접근·관측")
async def approach_observe(
    ctx: TaskContext,
    robot_id: str,
    coarse_cands: list[OrientedDetection],
    prompt: str,
    home: WaypointRecord,
) -> tuple[list[OrientedDetection], list[float], bool]:
    """coarse 탐색 결과를 가까운 관측으로 정밀화해 파지 계획용 후보를 만든다.

    가장 높은 score의 후보를 선택하고, 해당 대상을 잘 볼 수 있는 관측 자세로 이동한 뒤
    close detection을 수행한다. close 관측이 성공하면 대상 후보만 정밀 결과로 교체하고,
    주변 후보는 coarse 결과를 유지한다.

    Returns:
        (candidates, observe_joints, close_success)
        - candidates: plan_pick에 사용할 후보 목록
        - observe_joints: close 관측에 사용한 로봇 자세
        - close_success: close 관측 결과를 신뢰할 수 있는지 여부
    """
    if not coarse_cands:
        return coarse_cands, list(home.joint_values), False
    cfg = primitives._SERVO_CFG
    target = max(coarse_cands, key=lambda c: c.score)
    point = servo.grasp_point(target, target, cfg, None)
    t_ee_cam = await _hand_eye(ctx, robot_id)
    if t_ee_cam is None:
        logger.warning(
            "approach_observe(%s): hand-eye 캘 없음 — close 관측 불가, coarse "
            "폴백 (캘 후 재시도)",
            prompt,
        )
        return coarse_cands, list(home.joint_values), False

    # 관측 거리 후보를 가까운 순서로 시도하고, 도달 가능한 look pose를 찾는다.
    res: ResolveReachableResponse | None = None
    dist_m = _OBSERVE_CAM_DIST_M[0]
    n_poses = 0
    for dist_m in _OBSERVE_CAM_DIST_M:
        poses = _camera_look_poses(point, t_ee_cam, dist_m)
        n_poses = len(poses)
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=[[p] for p in poses], path_from=list(home.joint_values)
            ),
            ResolveReachableResponse,
            robot_id=robot_id,
        )
        if res.index >= 0:
            break
        logger.info(
            "approach_observe(%s): 카메라 %.0fcm %d후보 전멸 — 다음 단",
            prompt,
            dist_m * 100.0,
            n_poses,
        )
    if res is None or res.index < 0:
        logger.warning(
            "approach_observe(%s): 관측 자세 도달 불가 (카메라 거리 %s 전부 "
            "%d후보 전멸: %s) — coarse 관측 유지, 계획은 멀리서 + yaw 격자 (폴백)",
            prompt,
            _OBSERVE_CAM_DIST_M,
            n_poses,
            "" if res is None else res.message,
        )
        return coarse_cands, list(home.joint_values), False
    elev = _OBSERVE_ELEV_DEG[res.index // _OBSERVE_AZIM_N]
    azim = 360 * (res.index % _OBSERVE_AZIM_N) // _OBSERVE_AZIM_N
    logger.info(
        "approach_observe(%s): look-pose 채택 — 카메라 %.0fcm 고각 %d° 방위 %d° "
        "(후보 %d/%d)",
        prompt,
        dist_m * 100.0,
        elev,
        azim,
        res.index,
        n_poses,
    )
    look_joints = res.solutions[0]

    # close 관측 자세로 이동
    await transit(
        ctx,
        robot_id,
        look_joints,
        home,
        floor_z=_observe_floor(coarse_cands),
    )
    await asyncio.sleep(_OBSERVE_SETTLE_S)

    # close 관측으로 대상을 다시 확인하고, 결과가 유효한지 검증
    t0 = time.monotonic()
    det = await ctx.call(
        Detector.Service.DETECT_ORIENTED,
        DetectRequest(robot_id=robot_id, prompts=[prompt], top_k=_TOP_K),
        DetectOrientedResponse,
    )
    accurate = _nearest_within(det.candidates, target.position, _VIEW_MATCH_RADIUS_M)
    if accurate is None:
        logger.warning(
            "approach_observe(%s): close 관측 실패 (타깃 근방 후보 없음) — "
            "coarse 유지 + yaw 격자 (폴백)",
            prompt,
        )
        return coarse_cands, look_joints, False

    n_close = len(accurate.points or [])
    n_coarse = len(target.points or [])
    # 가까이 봤지만 품질이 낮으면 coarse 결과로 폴백
    if n_coarse and n_close < n_coarse * _OBSERVE_POINTS_TRUST_RATIO:
        logger.warning(
            "approach_observe(%s): close 관측 불신 (물체 points %d < coarse %d × "
            "%.1f — 시점 불량 의심) — coarse 유지 + yaw 격자 (폴백)",
            prompt,
            n_close,
            n_coarse,
            _OBSERVE_POINTS_TRUST_RATIO,
        )
        return coarse_cands, look_joints, False
    logger.info(
        "approach_observe(%s): close 관측 → pos=(%.3f,%.3f) base_z=%.3f "
        "(coarse=(%.3f,%.3f), 물체 points %d/%d, %.1fs)",
        prompt,
        accurate.position[0],
        accurate.position[1],
        accurate.base_z,
        target.position[0],
        target.position[1],
        n_close,
        n_coarse,
        time.monotonic() - t0,
    )
    # close 관측 대상만 교체하고, 주변 후보는 coarse 정보 유지
    neighbors = [
        c
        for c in coarse_cands
        if _xy_dist(c.position, accurate.position) > _VIEW_MATCH_RADIUS_M
    ]
    return [accurate, *neighbors], look_joints, True


def _camera_look_poses(
    obj: tuple[float, float, float],
    t_ee_cam: np.ndarray,
    dist_m: float,
) -> list[TcpPose]:
    """물체를 바라보는 카메라 관측 자세 후보를 생성하고 TCP 자세로 변환한다.

    물체 중심 기준 여러 look pose를 생성하고, hand-eye 변환으로 로봇 관절 해석이 가능한
    TCP 후보로 변환한다. 후보는 높은 고각부터 순서대로 생성한다.
    """
    p = np.asarray(obj, dtype=float)
    t_cam_ee = np.linalg.inv(t_ee_cam)
    out: list[TcpPose] = []
    for elev_deg in _OBSERVE_ELEV_DEG:
        el = math.radians(elev_deg)
        for k in range(_OBSERVE_AZIM_N):
            az = 2.0 * math.pi * k / _OBSERVE_AZIM_N
            direction = np.array(
                [
                    math.cos(el) * math.cos(az),
                    math.cos(el) * math.sin(az),
                    math.sin(el),
                ]
            )
            campos = p + dist_m * direction
            z_cam = (p - campos) / dist_m
            if abs(z_cam[2]) > 0.999:
                x_cam = np.array([math.cos(az), math.sin(az), 0.0])
            else:
                x_cam = np.cross([0.0, 0.0, -1.0], z_cam)
                x_cam = x_cam / np.linalg.norm(x_cam)
            y_cam = np.cross(z_cam, x_cam)
            t_base_cam = np.eye(4)
            t_base_cam[:3, 0] = x_cam
            t_base_cam[:3, 1] = y_cam
            t_base_cam[:3, 2] = z_cam
            t_base_cam[:3, 3] = campos
            t_base_ee = t_base_cam @ t_cam_ee
            qx, qy, qz, qw = Rotation.from_matrix(t_base_ee[:3, :3]).as_quat()
            out.append(
                TcpPose(
                    position=(
                        float(t_base_ee[0, 3]),
                        float(t_base_ee[1, 3]),
                        float(t_base_ee[2, 3]),
                    ),
                    quaternion=(float(qx), float(qy), float(qz), float(qw)),
                )
            )
    return out


def _observe_floor(cands: list[OrientedDetection]) -> float:
    """관측 transit 충돌 방지를 위한 보수적 floor gate를 계산한다."""
    return min(c.base_z for c in cands) - 0.005


async def _hand_eye(ctx: TaskContext, robot_id: str) -> np.ndarray | None:
    bundle = await ctx.call(
        Calibration.Service.SNAPSHOT_BUNDLE,
        SnapshotBundleRequest(robot_id=robot_id),
        CalibrationBundle,
    )
    if bundle.hand_eye is None:
        return None
    t = np.eye(4)
    t[:3, :3] = np.asarray(bundle.hand_eye.result_data.R_cam2gripper, dtype=float)
    t[:3, 3] = np.asarray(
        bundle.hand_eye.result_data.t_cam2gripper, dtype=float
    ).reshape(3)
    return t
