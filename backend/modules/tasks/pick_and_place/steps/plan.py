"""계획 (모션 0 판정) — 스윕 누적 후보 → 집기 servo 진입 계획 / 놓기 후보.

순서 규약 (2026-07-13): 물리 파지 **전에** 집기·놓기 도달성을 모두 검증한다 —
놓을 곳이 도달 불가면 아무것도 집기 전에 실패 (쥔 채 멈춤 corrupt 방지).
놓기 계획의 held 기하는 coarse 관측 (단일 뷰 height 과소 가능 — release 가
수 mm 낮아질 수 있으나 상자 삽입은 관대. 정밀화는 실물 데이터 후 판단).
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from modules.detector.contract import OrientedDetection
from modules.motion.contract import (
    Motion,
    ResolveReachableRequest,
    ResolveReachableResponse,
    TcpPose,
)
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import (
    DetectionNotFound,
    NoReachableGrasp,
    TaskError,
)
from modules.tasks.core.step import step
from modules.waypoint.contract import WaypointRecord

from .. import geometry, servo
from ..antipodal import _JAW_OPEN_MAX_M
from ..geometry import PlaceCandidate, Vec3
from . import primitives
from .primitives import _VIEW_MATCH_RADIUS_M, _fmt, _xy_dist

logger = logging.getLogger(__name__)

# 바닥 충돌 게이트 평면을 검출 base_z 보다 살짝 내리는 버퍼 — 게이트는 cm-급
# 육안 충돌용, mm-급 여유는 geometry clearance 상수 책임.
_FLOOR_GATE_MARGIN_M = 0.005

# 이웃 장애물 수집 반경 — 이 안의 다른 검출 군집 점군을 계획 resolve 의 충돌
# 게이트에 장애물로 넣는다 (이보다 먼 물체는 접근과 무관).
_NEIGHBOR_RADIUS_M = 0.15

# 집기 계획이 resolve 를 시도하는 검출 후보 수 상한 — 오염 뷰 하나에 태스크가
# 죽지 않게 다음 후보로 넘어가되, 전멸 뷰당 resolve ~40s 라 폭주 방지 상한.
# 건강한 뷰는 첫 budget 에서 조기 성공 (수 초).
_PLAN_TRY_MAX = 4

# 후보 시도 순서용 base_z 문턱 — base_z 는 뷰마다 다른 걸 잰다 (top-view =
# 보이는 band 하단 ≈ 윗면 / 옆면 뷰 = 실제 바닥 근처. 2026-07-16 실물 확인).
# top-view 는 obs XY 가 윗면 centroid = 파지 XY 그 자체라 계획 진입이 깨끗
# → 먼저 시도. 낮은 base_z 뷰는 기하 정보(실 바닥)는 좋지만 위치에 뷰별 FK
# 오차가 섞여 후순위. 기각 아님 — 최종 심판은 resolve.
_BASE_Z_PLAUSIBLE_MIN_M = -0.01
# 위쪽 상한 — flying-pixel 트레일이 검출을 공중으로 들어올린 오염의 방어선
# (1차 방어 = detector _body_z_band 질량 앵커, 2026-07-17 실물: 오염 spot
# base_z=+0.156~0.175 가 score 상위로 resolve ~55s×2 낭비 / 큐브 +0.044 →
# servo 허공 목표). 적치로 상자 위에 놓인 물체(base_z ≈ 상자 top ~0.03)는
# 정상 통과해야 하므로 gross floater 만 거르는 성긴 상한.
_BASE_Z_PLAUSIBLE_MAX_M = 0.08
# 적치 spot 하한이 pick(-0.01)보다 깊은 이유: 적치 상자는 크고 멀티뷰 바닥이
# 실제로 -0.02 대까지 관측된다 (2026-07-17 로그: 실상자 -0.007~-0.024) —
# pick 문턱을 그대로 쓰면 건강한 상자 뷰 전부가 후순위로 밀린다.
_PLACE_BASE_Z_MIN_M = -0.04

# pick 후보 score 하한 — 이 밑은 실행 후보에서 **제외** (후순위가 아니라 컷).
# 도달성 우선 순회가 저신뢰 오검출까지 내려가면 엉뚱한 물체를 집으러 간다
# (2026-07-17 실물: 진짜 큐브(0.76/0.49) 전멸 → score 0.31 오검출(로봇 옆
# 흰 어댑터) 채택 → 사용자 STOP). 실측 분리: 진짜 큐브 min 0.49 / 오검출 max
# 0.44. 전 후보 미달이면 명시 실패 — 오동작보다 정직한 실패가 낫다.
# **pick 전용** — place spot 은 진짜 상자가 score 0.34 로 채택된 실측(06:18)이
# 있어 하한을 걸면 정상 run 이 죽는다 (적치는 오검출 비용도 낮음).
_PICK_SCORE_MIN = 0.45
# pick 후보 폭 상한 — 조가 물리적으로 못 무는 물체는 score 와 무관하게 후보가
# 아니다 (2026-07-17 실물: 손에 든 큐브 전멸 후 score 0.68 짜리 footprint
# 116mm blob 을 "small cube" 로 채택, lateral 47mm 계획으로 "완전 다른 데"
# 주행 — antipodal 쌍 필터(_JAW_OPEN_MAX_M)는 쓰레기 점군 안 우연 쌍으로
# 우회됨). 문턱 = 조 개구 SSOT + 관측 번짐 여유 (실측: 실물 20mm 가 depth
# 번짐으로 33mm 로 측정된 전례 — 진짜 큐브를 컷하지 않게 +15mm).
_PICK_WIDTH_BLEED_M = 0.015
_PICK_MAX_WIDTH_M = _JAW_OPEN_MAX_M + _PICK_WIDTH_BLEED_M

# 로봇 베이스 점유 제외 반경 — 이 안의 pick 후보는 score 무관 **제외** (컷).
# 로봇 위치는 크로스캘로 아는 세계(robots.yaml base_pose)다 — 2026-07-19 22:26
# 실물: OMX 흰 원형 모터가 "white small round cube" 로 score 0.57 을 받아
# 통계 컷(0.45)을 정면 돌파, 로봇이 OMX 베이스를 집으러 감 (07-17 오검출
# 사고의 강화판 — score 재튜닝은 다음 조명에서 또 뚫리는 땜빵). 실측 분리:
# 오검출↔OMX base 거리 7.8cm / 정상 큐브 최근접 21.2cm → 13cm 컷.
# ⚠ 베이스 XY 만 (팔 링크는 미커버 — 팔이 뻗은 자세로 오검출되면 이 게이트
# 밖. 실물 재발 시 다음 단계 = FK 로 링크 점유 영역 제외).
_ROBOT_BASE_EXCLUDE_M = 0.13


@dataclass(frozen=True, slots=True)
class ServoPlan:
    """plan_pick 산출 — servo 루프의 시작 조건 (전부 coarse 관측 기준 초기값).

    rung0_joints: resolve 가 반환한 첫 standoff 의 IK 해 (실행부 재계산 없음).
    grasp_point0/grasp_tcp0: coarse 기준 초기 파지 지점/TCP — 루프가 매 tick
    관측으로 갱신하므로 이 값은 진입용 + 마커 표시용.
    """

    coarse: OrientedDetection
    family: servo.GraspFamily
    rung0_joints: list[float]
    grasp_point0: Vec3
    grasp_tcp0: Vec3
    lateral0: float
    # 실 바닥 추정 (클러스터 min base_z − margin) — servo 루프의 파지 z 하한
    # guard (납작한 물체에서 grip_below_top 이 테이블을 뚫는 것 방지).
    floor_z: float | None = None
    # 채택된 진입 standoff 사다리 (None = cfg 기본). 기본 사다리 전멸 시 낮은
    # 진입으로 폴백한 결과를 servo 가 그대로 실행해야 한다 ("판정 사다리 ==
    # 실행 사다리" — resolve 의 판정 해 == 실행 해 원칙과 동일).
    standoffs: tuple[float, ...] | None = None


# 진입 사다리 폴백 라운드 (2026-07-21 감사 — docs/pnp_scenario_rework.md §8.5).
# 01:28 "312가족 전멸" 위치 재판정: 실제로는 68가족이 파지 가능했는데 진입
# rung(8cm)이 같은 자세를 못 만들어 전부 매장 (SO-101 수직류는 높은 z 에서
# 자세 불가). 진입 가능 최고 standoff 실측: 5cm 23가족(최적 관측 대역 그대로) /
# 3cm 12 / 2cm(midstop급, 수직 tilt0 포함) 16. 낮은 진입 = 보정 창 축소
# 트레이드오프 — approach_observe close 관측(5-12mm)이 시작 정확도로 메운다.
_ENTRY_LADDERS: tuple[tuple[float, ...], ...] = ((0.05,), (0.03,), (0.02,))


def _fmt_ladder(entry: tuple[float, ...]) -> str:
    """진입 사다리 로그 표기 (cm) — 명명 헬퍼인 이유: 프리뷰 정적 인덱서가
    step 본문의 인라인 join 을 `<동적>` 노이즈 행으로 잡는다 (_join_msgs 동형)."""
    return "/".join(f"{s * 100:.0f}" for s in entry)


def _join_round_msgs(msgs: list[str]) -> str:
    """라운드별 전멸 사유 병합 — 명명 헬퍼 (프리뷰 노이즈 회피, _fmt_ladder 동형)."""
    return "; ".join(msgs)


def servo_ladder_groups(
    coarse: OrientedDetection,
    cfg: servo.ServoConfig,
    floor_z: float | None = None,
    *,
    yaw_grid: bool = True,
    standoffs: tuple[float, ...] | None = None,
) -> tuple[list[list[TcpPose]], list[tuple[servo.GraspFamily, Vec3, Vec3, float]]]:
    """coarse 관측 → resolve 후보 그룹 ([standoff 사다리…, 파지] × 가족) + 메타.

    plan_pick 과 sim 게이트 테스트(test_motion — 실 URDF IK 로 이 그룹이 진짜
    풀리는지)가 공유하는 그룹 구성 SSOT. 가족 = 절대 yaw 격자 (§11 —
    servo.grasp_families). **width 물리 게이트**: 그 yaw 방향 관측 폭이 조
    개구를 넘는 가족은 물 수 없다 — 여기서 제외 (직사각 물체의 긴 변 물기가
    자연 기각되는 자리, 옛 aspect 문턱의 물리 기반 대체).
    standoffs: 진입 사다리 override (None = cfg 기본) — _ENTRY_LADDERS 폴백."""
    entry = cfg.standoffs if standoffs is None else standoffs
    families = servo.grasp_families(coarse, yaw_grid=yaw_grid)
    groups: list[list[TcpPose]] = []
    metas: list[tuple[servo.GraspFamily, Vec3, Vec3, float]] = []
    g_point0 = servo.grasp_point(coarse, coarse, cfg, floor_z)
    width_dropped = 0
    for fam in families:
        width = servo.width_along(
            coarse.points, fam.jaw_axis, fallback_m=coarse.footprint[1]
        )
        if width > _PICK_MAX_WIDTH_M:
            width_dropped += 1
            continue
        lateral = servo.lateral_offset(width)
        g_tcp0 = servo.grasp_tcp(g_point0, fam, lateral, cfg.engage_m)
        poses = [
            TcpPose(
                position=servo.standoff(g_tcp0, fam, s), quaternion=fam.quat
            )
            for s in entry
        ]
        poses.append(TcpPose(position=g_tcp0, quaternion=fam.quat))
        groups.append(poses)
        metas.append((fam, g_point0, g_tcp0, lateral))
    if width_dropped:
        logger.info(
            "servo_ladder_groups: 개구 초과 yaw 가족 %d/%d 제외 "
            "(관측 폭 > %.0fmm)",
            width_dropped, len(families), _PICK_MAX_WIDTH_M * 1000.0,
        )
    return groups, metas


def _fail_histogram(
    metas: list[tuple[servo.GraspFamily, Vec3, Vec3, float]],
    group_failures: list[str],
    top_n: int = 8,
) -> str:
    """전멸 사유 히스토그램 (tilt×사유) — §11 관측성, 실물 디버깅 1차 데이터.

    "어느 자세 축이 어느 게이트에서 죽었나"가 로그만으로 갈린다 (2026-07-20
    "전멸인데 원인 모름" 재발 방지). 모듈 레벨 함수인 이유 = 시나리오 프리뷰
    정적 인덱서가 step 본문의 `.join` 등을 `<동적>` 행으로 만드는 것 방지
    (pick._notify_grasp 전례)."""
    hist: dict[str, int] = {}
    for (fam, _, _, _), why in zip(metas, group_failures):
        key = f"tilt{fam.tilt_deg:+d}/{why or '?'}"
        hist[key] = hist.get(key, 0) + 1
    top = sorted(hist.items(), key=lambda kv: -kv[1])[:top_n]
    return ", ".join(f"{k}×{v}" for k, v in top)


@step(title="집기 계획")
async def plan_pick(
    ctx: TaskContext,
    robot_id: str,
    prompt: str,
    home: WaypointRecord,
    cands: list[OrientedDetection],
    *,
    exclude_xy: list[tuple[float, float]] | None = None,
    trust_yaw: bool = False,
) -> ServoPlan:
    """스윕 누적 후보(cands — detect step 산출) → servo 접근 계획 (모션 0) →
    ServoPlan. 검출은 detect step 몫 (2026-07-19 스윕 통합 — 계획은 순수 판정).

    **도달성 우선 선택 (2026-07-16)**: score 1등에 커밋하지 않는다 — 스윕 뷰 간
    검출 위치는 FK 계통 오차로 1.5~3.3cm 어긋나며 (detector FUSE_ORIENTED
    docstring), 오염 뷰(예: base_z 가 테이블 아래로 3cm 눌린 관측)는 resolve 가
    정당하게 전멸시킨다. score 1등이 그 오염 뷰면 태스크 전체가 죽는 실사고
    (2026-07-16: 같은 큐브의 건강 뷰 4가족 통과, score 1등 오염 뷰 전멸).
    → score 내림차순으로 후보마다 resolve, 첫 성공 채택 (plan_place 2026-07-14
    와 동일 원칙). 전 후보 전멸 = 후보별 사유 포함 명시 실패 (맹목 파지 금지).

    자세 가족(조 축 2 × flip 2 × tilt 13)마다 [standoff 사다리…, 파지] 를 한
    그룹으로 resolve — 게이트: 끝점 IK + 바닥 + 그리퍼(벌림)↔**이웃** 점군
    충돌 + home→rung0 관절 경로 + 사다리 구간 직선(linear). 파지 대상 자신의
    점군은 장애물이 아니다 (engage 겹침은 의도 — resolve 호출부 주석).
    """
    if not cands:
        raise DetectionNotFound(prompt, candidates=0, reason="검출 0건")
    cfg = primitives._SERVO_CFG
    # 구조 컷 ⓪ — 로봇 베이스 점유 영역 (exclude_xy = robots.yaml base_pose,
    # _ROBOT_BASE_EXCLUDE_M 주석 = 실사고/실측 근거). **타깃 후보에서만** 제외
    # — 이웃 장애물/바닥 클러스터(_neighbor_points 등)는 원본 cands 그대로
    # (로봇이 거기 실재하므로 장애물로는 오히려 유효한 관측).
    in_robot = [
        c for c in cands
        if any(
            _xy_dist(c.position, (bx, by, 0.0)) <= _ROBOT_BASE_EXCLUDE_M
            for bx, by in (exclude_xy or [])
        )
    ]
    if in_robot:
        logger.info(
            "plan_pick(%s): 로봇 베이스 영역 후보 %d개 제외 (%s — 반경 %.0fcm, "
            "로봇은 집을 물체가 아니다)",
            prompt, len(in_robot),
            [f"({c.position[0]:.2f},{c.position[1]:.2f}) s={c.score:.2f}"
             for c in in_robot],
            _ROBOT_BASE_EXCLUDE_M * 100,
        )
    targets = [c for c in cands if c not in in_robot]
    if not targets:
        raise DetectionNotFound(
            prompt,
            candidates=len(cands),
            reason=(
                f"검출 {len(cands)}건 전부 로봇 베이스 점유 영역 안 (반경 "
                f"{_ROBOT_BASE_EXCLUDE_M * 100:.0f}cm) — 물체를 로봇에서 떨어진 "
                "작업 영역에 두고 다시 실행하세요"
            ),
        )
    # 신뢰 컷 2종 — 저신뢰 score / 조 개구 초과 폭. 어느 쪽이든 순회 폴백
    # 대상조차 아님 (상수 주석 — 엉뚱한 물체 파지 실사고 2건). 컷은 침묵하지
    # 않는다.
    oversize = [c for c in targets if c.footprint[1] > _PICK_MAX_WIDTH_M]
    if oversize:
        logger.info(
            "plan_pick(%s): 개구 초과 후보 %d개 제외 (짧은 변 %s > %.0fmm — "
            "조가 못 무는 크기)",
            prompt, len(oversize),
            [f"{c.footprint[1] * 1000:.0f}mm" for c in oversize],
            _PICK_MAX_WIDTH_M * 1000,
        )
    sized = [c for c in targets if c.footprint[1] <= _PICK_MAX_WIDTH_M]
    trusted = [c for c in sized if c.score >= _PICK_SCORE_MIN]
    if len(trusted) < len(sized):
        logger.info(
            "plan_pick(%s): 저신뢰 후보 %d개 제외 (score < %.2f — 오검출 방지)",
            prompt, len(sized) - len(trusted), _PICK_SCORE_MIN,
        )
    if not trusted:
        raise DetectionNotFound(
            prompt,
            candidates=len(cands),
            reason=(
                f"검출 {len(cands)}건 전부 신뢰 컷 미달 (score < "
                f"{_PICK_SCORE_MIN} 저신뢰 {len(sized) - len(trusted)}건 / "
                f"조 개구 {_PICK_MAX_WIDTH_M * 1000:.0f}mm 초과 "
                f"{len(oversize)}건). 물체 위치/조명 확인 후 다시 실행하세요"
            ),
        )
    # 물리 타당(base_z 가 설치면 근방 대역 안) 후보 먼저, 그 안에서 score
    # 내림차순 — 불가능 기하에 resolve ~40s 를 먼저 태우지 않는다 (2026-07-16:
    # score 1등이 base_z=-0.021 오염 뷰라 건강 뷰 도달인데 1분 소모+전멸 보고 /
    # 2026-07-17: 공중 부양 오염 뷰 — 상한 추가).
    ordered = sorted(
        trusted,
        key=lambda c: (
            c.base_z < _BASE_Z_PLAUSIBLE_MIN_M
            or c.base_z > _BASE_Z_PLAUSIBLE_MAX_M,
            -c.score,
        ),
    )[:_PLAN_TRY_MAX]
    failures: list[str] = []
    for rank, coarse in enumerate(ordered):
        neighbors = _neighbor_points(cands, coarse)
        # 바닥 평면 = 같은 물체를 본 뷰들(클러스터) base_z 의 최솟값 — 단일
        # top-view 의 base_z 는 바닥이 아니라 ≈윗면이라, 그걸 floor 로 쓰면
        # 윗면 근처 가짜 바닥이 생겨 깊은 파지가 계획에서 전멸한다. 옆면을 본
        # 뷰의 base_z 가 실 바닥에 가장 가깝다 (min 이 그 뷰를 고른다).
        cluster_base = [
            c.base_z for c in cands
            if _xy_dist(c.position, coarse.position) <= _VIEW_MATCH_RADIUS_M
        ]
        floor_z = min(cluster_base) - _FLOOR_GATE_MARGIN_M
        # resolve 라운드 (§11 + 2026-07-21 적응 진입 사다리): 가족 = tilt 사다리
        # × yaw × flip (선호순). yaw = trust_yaw(가까이 정확 관측) 면 면정렬
        # 2개만, 아니면(coarse 폴백) 절대 격자 전체. 라운드 = 기본 사다리 →
        # _ENTRY_LADDERS (낮은 진입) — 진입 rung 이 자세를 못 만들어 파지 가능
        # 가족이 매장되는 것 방지 (68가족 매장 감사, _ENTRY_LADDERS 주석).
        # 해석적 IK 라 라운드당 전멸도 수 s 확정.
        res = None
        groups: list = []
        metas: list = []
        entry: tuple[float, ...] = cfg.standoffs
        round_msgs: list[str] = []
        t0 = time.monotonic()
        for entry in (cfg.standoffs, *_ENTRY_LADDERS):
            groups, metas = servo_ladder_groups(
                coarse, cfg, floor_z, yaw_grid=not trust_yaw, standoffs=entry
            )
            # 장애물 = **이웃 점군만** — 파지 대상 자신의 점군은 넣지 않는다
            # (2026-07-17 오후 실사고): engage(조를 물체 쪽으로 밀어넣어 물기)
            # 설계상 grasp 자세의 조↔대상 겹침은 **의도된 것**인데, 자기
            # 점군을 장애물로 검사하면 그 겹침이 기각된다 (실측 침투 -3.6~
            # -9.6mm = engage 겹침 — 기각/통과를 가른 건 "카메라가 어느 면을
            # 봤나"). 대상 보호 = antipodal 구성 + servo + 파지 판정
            # (MoveIt ACM 의 조작 대상 충돌 허용 등가).
            res = await ctx.call(
                Motion.Service.RESOLVE_REACHABLE,
                ResolveReachableRequest(
                    groups=groups,
                    floor_z=floor_z,
                    linear=True,
                    obstacle_points=list(neighbors),
                    gripper_open=True,
                    path_from=list(home.joint_values),
                ),
                ResolveReachableResponse,
                robot_id=robot_id,
            )
            if res.index >= 0:
                break
            round_msgs.append(
                f"진입 {_fmt_ladder(entry)}cm: "
                f"{len(groups)}가족 전멸 — {res.message}"
            )
            logger.info(
                "plan_pick(%s): 진입 사다리 %s 전멸 — 낮은 진입 재시도 | "
                "사유 상위: %s",
                prompt, entry, _fail_histogram(metas, res.group_failures),
            )
        resolve_s = time.monotonic() - t0
        if res is None or res.index < 0:
            msg = (
                f"후보{rank}(score {coarse.score:.2f} "
                f"pos={_fmt(coarse.position)} base_z={coarse.base_z:+.3f}): "
                f"진입 사다리 {1 + len(_ENTRY_LADDERS)}단 전멸 ({resolve_s:.1f}s)"
                f" — {_join_round_msgs(round_msgs)}"
            )
            failures.append(msg)
            logger.info("plan_pick(%s): %s — 다음 후보 시도", prompt, msg)
            continue
        fam, g_point0, g_tcp0, lateral = metas[res.index]
        logger.info(
            "plan_pick(%s): 후보%d/%d 채택 (score %.2f) — 가족 %d/%d %s, "
            "grasp0=%s lateral=%.1fmm 진입=%scm (resolve %.1fs)",
            prompt, rank, len(ordered), coarse.score, res.index,
            len(groups), fam.label, _fmt(g_tcp0), lateral * 1000.0,
            _fmt_ladder(entry), resolve_s,
        )
        return ServoPlan(
            coarse=coarse,
            family=fam,
            rung0_joints=res.solutions[0],
            grasp_point0=g_point0,
            grasp_tcp0=g_tcp0,
            lateral0=lateral,
            floor_z=floor_z,
            standoffs=tuple(entry),
        )
    raise NoReachableGrasp(
        f"servo 접근 — 검출 후보 {len(ordered)}개 전부 전멸:\n  "
        + "\n  ".join(failures)
    )


def _neighbor_points(
    cands: list[OrientedDetection], coarse: OrientedDetection
) -> list[Vec3]:
    """타깃 아닌 이웃 후보의 점군 — 계획 resolve 충돌 게이트의 장애물.

    같은 prompt 로 잡힌 다른 물체 군집 (매치 반경 밖 ~ _NEIGHBOR_RADIUS_M 안).
    다른 prompt 의 물체는 지금 관측 채널이 없다 — 미관측 장애물은 실물 몫."""
    out: list[Vec3] = []
    for c in cands:
        d = _xy_dist(c.position, coarse.position)
        if d <= _VIEW_MATCH_RADIUS_M or d > _NEIGHBOR_RADIUS_M:
            continue
        out.extend(c.points or [])
    return out


def _fuse_place_center(spots: list[OrientedDetection]) -> OrientedDetection | None:
    """스윕에서 모인 같은 상자 검출들을 융합 → 안정된 중심 (2026-07-18 실물 버그).

    상자는 **정적**이라 여러 search pose 관측이 한 자리에 몰린다. 그런데 단일
    검출 중심은 부분-림 관측으로 pose 마다 2~3cm 흔들려(실측 dump: 같은 상자가
    X 0.276→0.298 / Y 0.085→0.117), 7cm 짜리 상자에선 그 오차가 곧 **모서리 적치**
    였다 (pick 은 closed-loop servo 로 잡지만 place 는 open-loop 라 coarse 오차를
    그대로 물려받음). closed-loop place 는 물체를 든 채라 상자가 가려 어렵다 —
    대신 **집기 전 스윕의 가림 없는 관측들을 융합**해 occlusion 을 아예 회피한다.

    plausible base_z 검출을 XY 클러스터링 → 최대 score 군집의 score-가중 평균
    중심을 **대표(최고 score) 검출에 덮어** 반환 (산포 √N 감소). footprint/yaw/
    base_z/points 는 대표 실검출 값 유지 — 위치만 융합 (기하 fabricate 최소).
    군집이 1개(융합 이득 없음)면 None → 호출부가 기존 단일-spot 경로 유지."""
    good = [
        s
        for s in spots
        if _PLACE_BASE_Z_MIN_M <= s.base_z <= _BASE_Z_PLAUSIBLE_MAX_M
    ]
    if len(good) < 2:
        return None
    clusters: list[list[OrientedDetection]] = []
    for s in sorted(good, key=lambda d: -d.score):
        for cl in clusters:
            if _xy_dist(s.position, cl[0].position) <= _VIEW_MATCH_RADIUS_M:
                cl.append(s)
                break
        else:
            clusters.append([s])
    best = max(clusters, key=lambda cl: sum(d.score for d in cl))
    if len(best) < 2:
        return None  # 대표 뷰가 하나뿐 = 융합할 이웃 없음
    wsum = sum(d.score for d in best) or 1.0
    fx = sum(d.position[0] * d.score for d in best) / wsum
    fy = sum(d.position[1] * d.score for d in best) / wsum
    rep = max(best, key=lambda d: d.score)
    return rep.model_copy(update={"position": (fx, fy, rep.position[2])})


@step(title="놓기 계획")
async def plan_place(
    ctx: TaskContext,
    robot_id: str,
    prompt: str,
    *,
    home: WaypointRecord,
    spots: list[OrientedDetection],
) -> tuple[PlaceCandidate, list[float]]:
    """관측 spot(spots — detect/approach 산출) 게이트 판정 (모션 0) → (적치 후보,
    pre 관절해).

    **도달성 우선 선택 (2026-07-14)**: 점수 1등에 무조건 커밋하지 않는다 — spot
    을 점수순으로 돌며 팔이 실제로 닿는 첫 spot 채택. spot 마다 yaw 두 가족 순차
    (① 상자 방위 정렬 ② 전멸 시 자유 — 삐딱하게라도 놓는 게 task 실패보다 낫다).
    **놓기 자리 = 상자 정중앙 위** (2026-07-21 단순화 — 물건 폭/높이 무시,
    geometry._place_candidates TODO). 집기 계획과 독립 (held/lateral 의존 제거)
    이라 집기 전에 미리 계획 가능."""
    if not spots:
        raise TaskError(
            f"'{prompt}' 적치 대상 검출 0건 — 물체 배치/조명 확인 후 다시 "
            "실행하세요"
        )
    # 물리 타당(base_z 대역 안) spot 먼저 — plan_pick 과 같은 원칙 (기각 아님,
    # 최종 심판은 resolve). 2026-07-17 실물: 공중 부양 오염 spot(base_z=+0.156
    # ~0.175)이 score 상위를 차지해 spot 당 정렬+자유 resolve ~55s 를 먼저
    # 태우고, 최악 런은 plan_place 에만 3.5분 소모 후 실패.
    ranked = sorted(
        spots,
        key=lambda s: (
            s.base_z < _PLACE_BASE_Z_MIN_M
            or s.base_z > _BASE_Z_PLAUSIBLE_MAX_M,
            -s.score,
        ),
    )
    # 상자 중심 융합 (정적 상자 = 스윕 관측 몰림). 융합 중심을 **먼저** 시도하고
    # 도달 불가/실패 시 기존 단일-spot 순회로 폴백 (안전 — 동작 후퇴 없음).
    fused = _fuse_place_center(spots)
    if fused is not None:
        best_single = ranked[0]
        logger.info(
            "plan_place(%s): 상자 중심 융합 — 단일 best (%.3f,%.3f) → 융합 "
            "(%.3f,%.3f), 이동 %.1fcm (모서리 적치 방지)",
            prompt, best_single.position[0], best_single.position[1],
            fused.position[0], fused.position[1],
            _xy_dist(fused.position, best_single.position) * 100.0,
        )
        ranked = [fused, *ranked]
    for spot in ranked:
        for family, pplan in (
            ("정렬", geometry.plan_place(spot)),
            ("자유", geometry.plan_place_free(spot)),
        ):
            got = await resolve_place(
                ctx, robot_id, pplan,
                floor_z=spot.base_z - _FLOOR_GATE_MARGIN_M,
                home=home,
            )
            if got is not None:
                idx, sols = got
                logger.info(
                    "plan_place(%s): spot 채택 score=%.2f base_z=%.3fm "
                    "pos=(%.3f,%.3f) — %s yaw %s (후보 %d건 중)",
                    prompt, spot.score, spot.base_z, spot.position[0],
                    spot.position[1], pplan[idx].label, family, len(ranked),
                )
                return pplan[idx], sols[0]
            logger.info(
                "plan_place(%s): spot score=%.2f pos=(%.3f,%.3f) %s yaw %d후보 "
                "전멸 — %s", prompt, spot.score, spot.position[0],
                spot.position[1], family, len(pplan),
                "자유 yaw 폴백" if family == "정렬" else "다음 spot",
            )
    raise NoReachableGrasp(
        f"놓을 자리 도달 불가 — '{prompt}' 후보 {len(ranked)}건 모두 팔이 닿지 "
        "않습니다 (정렬+자유 yaw 전부 시도 — workspace 밖이거나 주변이 막힘). "
        "상자를 로봇 쪽으로 옮기거나 주변 장애물을 치운 뒤 다시 실행하세요"
    )


@step(title="적치 후보 선별")
async def resolve_place(
    ctx: TaskContext,
    robot_id: str,
    plan: list[PlaceCandidate],
    *,
    floor_z: float,
    home: WaypointRecord,
) -> tuple[int, list[list[float]]] | None:
    """한 spot 의 적치 후보 게이트 판정 (위치→자세→바닥→home→pre 관절 경로→
    pre↔place 직선) — 모션 0. 닿는 그룹 있으면 (index, solutions), 없으면 None.

    None = 이 spot 은 도달 불가 (부정 데이터 — 호출부가 다음 spot 으로 폴백).
    최종 실패 판정(모든 spot 소진)은 호출부 plan_place 가 raise."""
    t0 = time.monotonic()
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=geometry.place_ik_groups(plan),
            floor_z=floor_z,
            linear=True,
            path_from=list(home.joint_values),
        ),
        ResolveReachableResponse,
        robot_id=robot_id,
    )
    resolve_s = time.monotonic() - t0
    if res.index < 0:
        logger.info("resolve_place: 도달 불가 (%.1fs) — %s", resolve_s, res.message)
        return None
    logger.info(
        "resolve_place: group %d — %s (%.1fs)",
        res.index, plan[res.index].label, resolve_s,
    )
    return res.index, res.solutions
