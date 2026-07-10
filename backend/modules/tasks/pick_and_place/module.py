import asyncio
import logging
import math
import time

from scipy.spatial.transform import Rotation

from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.detector.contract import Detector, DetectRequest, DetectOrientedResponse
from modules.motion.contract import (
    Motion,
    MoveJPoseRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    SelectReachableRequest,
    SelectReachableResponse,
    TcpPose,
)
from modules.motor.contract import Motor, SetGripperRequest, SetGripperResponse

from .contract import PickAndPlace, RunRequest, RunResponse

logger = logging.getLogger(__name__)

_ROBOT = "so101_6dof_0"  # 테스트 고정 — shape 굳으면 RunRequest 필드로 승격

# TODO:
# 더 가면 (지금은 안 감)
# 탐색 자체를 없애는 길도 있습니다 — 부팅 시 1회 FK 샘플링으로 (r, z) → 최소 가용 tilt 테이블을 캐시하면 probe 0회.
# 제가 오프라인에서 만든 워크스페이스 지도가 바로 그 원형입니다.
# 다만 지금 단계엔 배치 서비스만으로 충분하고, 테이블은 나중에 상수 실측 쌓이면.

# ── 그리퍼/전략 상수 (URDF mesh 실측 2026-07-09) ──
# TCP 프레임 규약 (so101_6dof.urdf 실측): x_tool=approach(손끝 방향),
# y_tool=조 벌림축(+y 쪽 = 파란 고정 조), TCP 원점 = 손끝 평면 (z_fixed -0.104).
_TCP_TO_FIXED_JAW_M = 0.0079  # TCP → 고정 조 안쪽 면 (+y_tool 방향, mesh 실측)
_FIXED_JAW_CLEAR_M = 0.005  # 하강 중 고정 조 vs 큐브 옆면 여유 (0.5cm)
_APPROACH_CLEAR_M = 0.06  # pre-grasp: 큐브 윗면 위 접근 높이
_FINGER_TABLE_CLEAR_M = 0.008  # 손끝 vs 테이블 여유
_GRIPPER_OPEN_RAW = 3186  # motors.yaml max (+100° full open)
_GRIPPER_CLOSE_RAW = 1935  # motors.yaml min (-10° — 큐브 클램프는 torque stall)
_GRIPPER_SETTLE_S = 1.2  # SET_GRIPPER 는 즉시 반환 — 조 이동 대기 (90°/s profile)
# top-down 파지 자세: 툴 x(approach)→base -z(수직 하향), y(조 축)→base +y.
# yaw=0 이면 조가 base Y 로 벌어짐 → Rz(grasp_yaw) 곱하면 조가 OBB 짧은 변을
# 가로질러 묾 (grasp_yaw = 긴 변 방향, detector geometry 규약).
_TOPDOWN = Rotation.from_matrix([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])


class PickAndPlaceModule:
    def __init__(self, runtime: ModuleRuntime) -> None:
        self.runtime = runtime

    async def start(self) -> None:
        logger.info("PickAndPlaceModule start")

    async def stop(self) -> None:
        logger.info("PickAndPlaceModule stop")

    @service(PickAndPlace.Service.RUN)
    async def run(self, req: RunRequest) -> RunResponse:
        # 1) 검출 — Top-K OBB 후보 (오버레이/디버그 PNG 는 detector 가 publish)
        t0 = time.perf_counter()
        result = await self.runtime.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=_ROBOT,
                          prompt="white small cube box", top_k=5),
            DetectOrientedResponse,
            timeout=10.0,
        )
        logger.info(
            "found=%s n=%d (detect %.2fs)",
            result.found, len(result.candidates), time.perf_counter() - t0,
        )
        for i, d in enumerate(result.candidates):
            logger.info(
                "cand[%d]: score=%.2f top_z=%.3f base_z=%.3f h=%.3f "
                "footprint=%.0fx%.0fmm",
                i, d.score, d.position[2], d.base_z, d.height,
                d.footprint[0] * 1000, d.footprint[1] * 1000,
            )

        # 2) 후보 선택 — 기하 prior 는 소비자 책임 (§17.5). 테스트: 높이 1.5~15cm.
        # (2cm 하한은 실측 2.3cm 큐브가 depth 노이즈로 경계 탈락 — 느슨하게. 상한은
        # 테이블 모서리 ring 에 낮은 바닥이 섞여 height 부풀 때 잡는 안전망.)
        cands = [c for c in result.candidates if 0.015 <= c.height <= 0.15]
        if not cands:
            logger.info("파지 후보 없음 (height prior 통과 0) — 위 cand 로그 확인")
            return RunResponse()
        c = max(cands, key=lambda d: d.score)
        x, y, top_z = c.position
        short = c.footprint[1]

        # 3) grasp 계획 (인라인 — 테스트 단계)
        # 수직 approach 는 이 팔이 높은 z 에서 못 냄 — FK 샘플링(2026-07-09) 결과
        # tilt<15° 는 z≤0.038 에서만 가능 (관절 리밋). 그래서 조 축(y_tool)은 수평
        # 유지(파지 성립 조건)한 채 approach 만 조 축 둘레로 기울인 후보를 tilt
        # 오름차순 probe: MOVE_J_POSE/MOVE_L 의 IK reject = 모션 0 이라 안전.
        grasp_z = max(top_z - c.height * 0.5, c.base_z + _FINGER_TABLE_CLEAR_M)
        pre_z = top_z + _APPROACH_CLEAR_M
        logger.info(
            "grasp plan: cube xy=(%.3f,%.3f) grasp_z=%.3f yaw=%.1fdeg short=%.0fmm",
            x, y, grasp_z, math.degrees(c.grasp_yaw), short * 1000,
        )

        # 4) 실행 — move 서비스는 trajectory 완료까지 블록. 후보 3축:
        #    tilt(작은 것부터) × yaw 가족 × 180° flip(조 대칭 — 파지 등가).
        #    yaw 가족 = grasp_yaw(짧은 변 물기) + 90°(긴 변 물기) — 정사각형 근처
        #    footprint(예: 23x22mm)는 minAreaRect yaw 가 노이즈 임의값이라 한 yaw
        #    강제가 도달성 전멸을 만듦 (2026-07-09 yaw=84° 사건). 둘 다 유효 파지.
        long_side = c.footprint[0]
        yaw_options = (
            (c.grasp_yaw, short),  # 조가 짧은 변을 가로질러 묾
            (c.grasp_yaw + math.pi / 2, long_side),  # 90° 돌려 긴 변을 묾
        )
        # 후보 = 순수 데이터 생성 (판정 아님). 각 후보: (라벨, gx, gy, quat).
        # 단일 가동 조 보정: 파란(고정) 조는 모터로 안 움직임 — TCP 를 큐브 중심이
        # 아니라 고정 조 안쪽 면이 (물 변 + 5mm) 에 오는 자리로 조 축(+y_tool,
        # tilt 무관 수평) 방향 횡이동.
        cands2 = []
        for tilt_deg, (yaw_base, across), flip in (
            (t, yo, f)
            for t in (0, 15, -15, 25, -25, 30, -30, 35, -35, 40, -40)
            for yo in yaw_options
            for f in (0.0, math.pi)
        ):
            rot = (
                Rotation.from_euler("z", yaw_base + flip)
                * _TOPDOWN
                * Rotation.from_euler("y", math.radians(tilt_deg))
            )
            qx, qy, qz, qw = (float(v) for v in rot.as_quat())
            quat = (qx, qy, qz, qw)
            lateral = across / 2 + _FIXED_JAW_CLEAR_M - _TCP_TO_FIXED_JAW_M
            off = rot.apply([0.0, lateral, 0.0])
            label = (
                f"tilt={tilt_deg:+d} yaw={math.degrees(yaw_base):.0f} "
                f"flip={math.degrees(flip):.0f}"
            )
            cands2.append((label, x + float(off[0]), y + float(off[1]), quat))

        # 판정 = motion 배치 IK 1회 (모션 0, in-process seed 연쇄 + early-exit
        # + deepening — motion select_reachable 로그에 소요 시간).
        # 그룹 = [pre, grasp] — 같은 자세로 접근+파지 둘 다 풀려야 실행 가능.
        t1 = time.perf_counter()
        sel = await self.runtime.call(
            Motion.Service.SELECT_REACHABLE,
            SelectReachableRequest(groups=[
                [
                    TcpPose(position=(gx, gy, pre_z), quaternion=quat),
                    TcpPose(position=(gx, gy, grasp_z), quaternion=quat),
                ]
                for _, gx, gy, quat in cands2
            ]),
            SelectReachableResponse, robot_id=_ROBOT, timeout=60.0,
        )
        if sel.index < 0:
            logger.warning("모든 approach 후보 IK 불가 — 파지 포기 (%s)", sel.message)
            return RunResponse()
        label, gx, gy, quat = cands2[sel.index]
        logger.info("approach 확정: %s (plan %.2fs)", label, time.perf_counter() - t1)

        # 실행 — 직선 5수 (move 서비스는 trajectory 완료까지 블록)
        res_pre = await self.runtime.call(
            Motion.Service.MOVE_J_POSE,
            MoveJPoseRequest(target_position=(gx, gy, pre_z),
                             target_quaternion=quat),
            MoveJResponse, robot_id=_ROBOT, timeout=30.0,
        )
        if not res_pre.accepted:
            logger.warning("pre-grasp 실패(판정 후 불일치): %s", res_pre.message)
            return RunResponse()
        # 그리퍼 열기 — 하강 전에 활짝 (가동 조가 큐브 반대편 안 치게)
        await self.runtime.call(
            Motor.Service.SET_GRIPPER,
            SetGripperRequest(position_raw=_GRIPPER_OPEN_RAW),
            SetGripperResponse, robot_id=_ROBOT, timeout=10.0,
        )
        await asyncio.sleep(_GRIPPER_SETTLE_S)
        # 자세 고정 하강 — MoveL 은 경로 전 구간 IK 사전 검증 (안 풀리면 모션 0)
        res_down = await self.runtime.call(
            Motion.Service.MOVE_L,
            MoveLRequest(target_position=(gx, gy, grasp_z),
                         target_quaternion=quat),
            MoveLResponse, robot_id=_ROBOT, timeout=30.0,
        )
        if not res_down.accepted:
            logger.warning("하강 실패(판정 후 불일치): %s", res_down.message)
            return RunResponse()
        # 닫기 — 가동 조가 큐브를 고정 조까지 ~5mm 밀며 클램프
        await self.runtime.call(
            Motor.Service.SET_GRIPPER,
            SetGripperRequest(position_raw=_GRIPPER_CLOSE_RAW),
            SetGripperResponse, robot_id=_ROBOT, timeout=10.0,
        )
        await asyncio.sleep(_GRIPPER_SETTLE_S)
        # 자세 고정 수직 상승 (들어올림)
        await self.runtime.call(
            Motion.Service.MOVE_L,
            MoveLRequest(target_position=(gx, gy, pre_z),
                         target_quaternion=quat),
            MoveLResponse, robot_id=_ROBOT, timeout=30.0,
        )
        return RunResponse()
