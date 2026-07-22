"""접근·관측 — 파지/적치 계획을 '멀리서(스윕)'가 아니라 '가까이서' 세우기 위한
앞단 (2026-07-21 재구조, docs/pnp_scenario_rework.md §3.2 "가족 이사").

스윕 관측은 카메라 31-33cm 라 ~40mm 오차 → yaw 를 못 믿어 312 가족으로 헤지(느림
+ 겨우 닿는 자세 채택 위험). 물체 가까이(카메라 ~13cm = servo.py 실측 최적 대역,
base 관측 편차 5-12mm)서 다시 관측하면 정확도가 좋아진다 — 그 정확 관측으로
plan_pick/plan_place 가 돈다.

**look-pose 생성 = 카메라 중심** (2026-07-21 밤 재설계 — 실증 자세가 규정):
관측은 카메라 문제인데 옛 구현은 파지 가족(TCP 중심, 접근축이 물체 관통 +
후방 standoff)에서 파생 — "TCP 는 물체 옆에 두고 손목을 꺾어 카메라만 물체를
보는" 자세류를 통째로 놓쳐, 되는 자세가 있는데도 IK 전멸/극단 tilt 채택이 났다
(실증: TCP (0.192,−0.078,0.041), 카메라 11.3cm·고각 55° = 최적 뷰 — 파지 가족
밖). 지금은 **카메라를 물체 위 반구(고각×방위×거리)에 직접 배치**하고 hand-eye
역변환으로 TCP 를 도출한다. 방위/roll 은 시점 품질과 무관하지만 도달성엔 결정적.

**servo 는 안 건드린다** — 이 step 은 servo *앞에서* 더 좋은 입력을 만들 뿐.
실패는 침묵하지 않되 치명적이지 않다: 관측 자세 도달 불가/close 관측 0/시점
불량(points 급감)이면 **coarse 관측으로 폴백**(경고 로그 + 계획은 예전처럼
멀리서). 회귀 아님 — 07-19 까지 돌던 경로로 degrade.
"""

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
    FuseOrientedRequest,
    FuseOrientedResponse,
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

_OBSERVE_SETTLE_S = 0.3  # MoveJ 후 카메라 진동 정착 (검출 품질)
# 관측 프레임 수. 1 = 가까이서 한 번(카메라 14cm 편차 5-12mm 로 이미 coarse 대비
# 충분). 다중프레임 융합(노이즈↓)은 실물 첫 런 데이터로 이득 보이면 올린다 —
# servo tick 루프가 하강 중 재관측·refit 로 위치는 이미 계속 보정. ⚠ 실물 튜닝점.
_OBSERVE_FRAMES = 1
# 카메라-물체 거리 사다리 — 13cm(실측 최적 대역, 실증 자세 11.3cm)부터, 전멸 시
# 18cm 한 단 (coarse 31cm 보단 훨씬 가까움 — "가장자리는 close 포기" 반려).
_OBSERVE_CAM_DIST_M = (0.13, 0.18)
# 내려다보는 고각 사다리 (수직 우선). 하한 45° — 그 아래는 옆면 뷰라 관측이
# coarse 보다 나빠진다 (실측: 고각 ~30/45° 대 측면뷰 points ratio 0.31/0.55,
# 실증 좋은 뷰 = 55°). ⚠ 실물 튜닝점.
_OBSERVE_ELEV_DEG = (90, 75, 60, 45)
_OBSERVE_AZIM_N = 12  # 방위 그리드 — 시점 품질과 무관한 순수 도달성 축
# close 관측 새니티 — close 물체 점수가 coarse(31cm) 대비 이 비율 미만이면 시점
# 불량(옆면/부분만 봄)으로 보고 불신 → coarse 폴백. 같은 voxel 다운샘플이라
# 점수 ∝ 보이는 표면적: 좋은 시점은 같은 면을 더 가까이 봐 비율 ≥1 이 정상,
# 실사고는 0.31 (397/1296). ⚠ 실물 튜닝점 (n=1 사고 기반, 여유 2배로 시작).
_OBSERVE_POINTS_TRUST_RATIO = 0.7


def _camera_look_poses(
    obj: tuple[float, float, float],
    t_ee_cam: np.ndarray,
    dist_m: float,
) -> list[TcpPose]:
    """물체 위 반구의 카메라 자세 → hand-eye 역변환으로 TCP 후보 (선호순).

    카메라: 위치 = obj + d·(고각, 방위), 광축(+z_cam) = 물체 응시, roll =
    이미지 수평(cam x 를 월드 수평으로). 순서 = 고각 사다리(수직 먼저) × 방위.
    TCP = T_base_cam @ inv(T_ee_cam) — TCP 가 어디를 향하든 무관, 카메라 프레이밍
    이 유일한 제약 (관측 자세의 정의)."""
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
            z_cam = (p - campos) / dist_m  # 광축 = 물체 응시
            if abs(z_cam[2]) > 0.999:  # 수직 특이 — 방위각으로 roll 지정
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
    """관측 transit 의 바닥 게이트 평면 — 검출 최저 base_z − 여유 (plan.py 의
    cluster floor 산정과 동일 발상 — cm-급 육안 충돌 차단용, mm 정밀 아님)."""
    return min(c.base_z for c in cands) - 0.005


async def _hand_eye(ctx: TaskContext, robot_id: str) -> np.ndarray | None:
    """활성 hand-eye (T_ee_cam, 4x4) — 없으면 None (카메라 배치 불가 = 폴백)."""
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


@step(title="접근·관측")
async def approach_observe(
    ctx: TaskContext,
    robot_id: str,
    coarse_cands: list[OrientedDetection],
    prompt: str,
    home: WaypointRecord,
) -> tuple[list[OrientedDetection], list[float], bool]:
    """coarse 후보 → 최고 score 대상 위 관측 자세로 이동 → 정확 관측.

    반환 = (계획용 후보 리스트, 관측 자세 joints, **close: 정확 관측 성공 여부**).
    계획 리스트 = [정확 관측, *coarse 이웃] (이웃은 plan 의 장애물/바닥 문맥 유지 —
    타깃만 정확본으로 교체). 관측 실패(자세 도달 불가/close 후보 0) 시 (coarse_cands,
    joints, **False**) = 폴백(멀리서 계획, yaw 격자 유지). close=True 면 호출부가
    관측 yaw 를 믿어 파지 yaw 격자를 끈다 (plan_pick trust_yaw)."""
    if not coarse_cands:
        return coarse_cands, list(home.joint_values), False  # plan_pick 이 0 처리
    cfg = primitives._SERVO_CFG
    target = max(coarse_cands, key=lambda c: c.score)
    # 카메라가 응시할 점 = 정제된 파지점 (servo 와 동일 정의 — 물체 상면 중심대).
    point = servo.grasp_point(target, target, cfg, None)
    # hand-eye — 카메라 배치의 필수 입력. 없으면 (미캘 robot) close 관측 불가.
    t_ee_cam = await _hand_eye(ctx, robot_id)
    if t_ee_cam is None:
        logger.warning(
            "approach_observe(%s): hand-eye 캘 없음 — close 관측 불가, coarse "
            "폴백 (캘 후 재시도)", prompt,
        )
        return coarse_cands, list(home.joint_values), False
    # 거리 사다리 — 최적(13cm)부터, 전멸 시 한 단 멀리(18cm) 재시도.
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
            prompt, dist_m * 100.0, n_poses,
        )
    if res is None or res.index < 0:
        logger.warning(
            "approach_observe(%s): 관측 자세 도달 불가 (카메라 거리 %s 전부 "
            "%d후보 전멸: %s) — coarse 관측 유지, 계획은 멀리서 + yaw 격자 (폴백)",
            prompt, _OBSERVE_CAM_DIST_M, n_poses,
            "" if res is None else res.message,
        )
        return coarse_cands, list(home.joint_values), False
    elev = _OBSERVE_ELEV_DEG[res.index // _OBSERVE_AZIM_N]
    azim = 360 * (res.index % _OBSERVE_AZIM_N) // _OBSERVE_AZIM_N
    logger.info(
        "approach_observe(%s): look-pose 채택 — 카메라 %.0fcm 고각 %d° 방위 %d° "
        "(후보 %d/%d)", prompt, dist_m * 100.0, elev, azim, res.index, n_poses,
    )
    look_joints = res.solutions[0]
    # 계획 이동 — 현재 자세에서 look 자세로 직접 (home 왕복 강등, §12).
    # 폴백(home 경유)은 위 resolve 의 path_from=home 게이트가 사전 증명.
    await transit(
        ctx, robot_id, look_joints, home,
        floor_z=_observe_floor(coarse_cands),
    )
    await asyncio.sleep(_OBSERVE_SETTLE_S)

    t0 = time.monotonic()
    seen: list[OrientedDetection] = []
    for _ in range(_OBSERVE_FRAMES):
        det = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=robot_id, prompts=[prompt], top_k=_TOP_K),
            DetectOrientedResponse,
        )
        near = _nearest_within(det.candidates, target.position, _VIEW_MATCH_RADIUS_M)
        if near is not None:
            seen.append(near)
    if not seen:
        logger.warning(
            "approach_observe(%s): close 관측 0프레임 (타깃 근방 후보 없음) — "
            "coarse 유지 + yaw 격자 (폴백)", prompt,
        )
        return coarse_cands, look_joints, False

    accurate = await _fuse(ctx, seen, target.position)
    # close-vs-coarse 새니티 — 가까이 갔는데 물체 점수가 급감하면 시점 불량
    # (옆면/부분 관측). 그 관측을 믿고 계획하면 coarse 보다 나쁘다 (2026-07-21
    # −60° 실사고: points ⅓토막 + base_z 26mm 이동을 "성공"으로 채택). 침묵
    # 품질저하 금지 — 불신 사유를 로그로 남기고 정직하게 폴백.
    n_close = len(accurate.points or [])
    n_coarse = len(target.points or [])
    if n_coarse and n_close < n_coarse * _OBSERVE_POINTS_TRUST_RATIO:
        logger.warning(
            "approach_observe(%s): close 관측 불신 (물체 points %d < coarse %d × "
            "%.1f — 시점 불량 의심) — coarse 유지 + yaw 격자 (폴백)",
            prompt, n_close, n_coarse, _OBSERVE_POINTS_TRUST_RATIO,
        )
        return coarse_cands, look_joints, False
    logger.info(
        "approach_observe(%s): close 관측 %d/%d프레임 → pos=(%.3f,%.3f) base_z=%.3f "
        "(coarse=(%.3f,%.3f), 물체 points %d/%d, %.1fs)",
        prompt, len(seen), _OBSERVE_FRAMES,
        accurate.position[0], accurate.position[1], accurate.base_z,
        target.position[0], target.position[1], n_close, n_coarse,
        time.monotonic() - t0,
    )
    # 타깃 클러스터는 정확본으로 교체, 이웃(다른 물체)은 문맥 유지.
    neighbors = [
        c for c in coarse_cands
        if _xy_dist(c.position, accurate.position) > _VIEW_MATCH_RADIUS_M
    ]
    return [accurate, *neighbors], look_joints, True


async def _fuse(
    ctx: TaskContext, seen: list[OrientedDetection], anchor: tuple[float, float, float]
) -> OrientedDetection:
    """관측 프레임 융합 → 타깃 군집. 2프레임 미만/군집 없음이면 최신 단독(침묵 X)."""
    if len(seen) < 2:
        return seen[-1]
    res = await ctx.call(
        Detector.Service.FUSE_ORIENTED,
        FuseOrientedRequest(candidates=list(seen)),
        FuseOrientedResponse,
    )
    near = _nearest_within(res.candidates, anchor, _VIEW_MATCH_RADIUS_M)
    if near is None:
        logger.info("approach_observe: 융합 군집 없음 (%d프레임) — 최신 단독", len(seen))
        return seen[-1]
    return near
