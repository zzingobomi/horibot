"""servo 집기 실행 (closed-loop 본체) — look-then-move 루프 + commit + 판정.

    rung0 진입 (계획 이동 transit — home 왕복 강등, docs/motion.md §12) →
    tick 루프 (정지 관측 → gate → 상대 보정 MoveL → 수렴 시 하강) → commit
    (2단 하강 + 재앵커) → close → 파지 판정 (재시도) → 후퇴 → 슬립 판정 →
    (적치 미동반 시) home

순수 계산·실측 근거·상태 전이 = servo.py (SSOT), trace = servo_trace.py.
계획(가족/사다리 구성)은 plan.py — 여기는 실행과 실패 대응만.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Callable
from dataclasses import replace

import numpy as np

from framework.transport.protocol import RemoteError
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
    MoveJRequest,
    MoveJResponse,
    PoseTarget,
    ResolveReachableRequest,
    ResolveReachableResponse,
    TcpSnapshotRequest,
    TcpState,
)
from modules.motor.contract import JointState, Motor, ReadStateRequest
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import GraspFailed, ServoFailed
from modules.tasks.core.step import step
from modules.waypoint.contract import WaypointRecord

from .. import servo
from ..geometry import Quat, Vec3
from ..servo_trace import ServoTrace
from . import primitives
from .plan import ServoPlan, servo_ladder_groups
from .primitives import (
    _TOP_K,
    _VIEW_MATCH_RADIUS_M,
    _fmt,
    _log_reached_tcp,
    _move_j_joints,
    _move_l,
    _nearest_within,
    close_gripper,
    go_home,
    open_gripper,
    transit,
    verify_grasp,
)

logger = logging.getLogger(__name__)


def _effective_cfg(
    cfg: servo.ServoConfig, plan: ServoPlan
) -> servo.ServoConfig:
    """plan 이 채택한 진입 사다리를 실행 cfg 에 반영 — 판정 사다리 == 실행 사다리.

    기본 사다리 전멸 시 plan 이 낮은 진입(_ENTRY_LADDERS)으로 폴백한다 (2026-07-21
    68가족 매장 감사). eps 는 사다리 길이에 맞춰 뒤에서부터 (마지막 rung 이 항상
    최엄격 — 짧은 사다리 = 최종 관측 rung 만 남는 것과 동형)."""
    if plan.standoffs is None or tuple(plan.standoffs) == tuple(cfg.standoffs):
        return cfg
    n = len(plan.standoffs)
    eps = cfg.eps_descend_m[-n:] if n <= len(cfg.eps_descend_m) else (
        cfg.eps_descend_m + (cfg.eps_descend_m[-1],) * (n - len(cfg.eps_descend_m))
    )
    logger.info(
        "servo_pick: 진입 사다리 %s cm (기본 %s 전멸 폴백) — eps %s",
        [round(s * 100) for s in plan.standoffs],
        [round(s * 100) for s in cfg.standoffs],
        list(eps),
    )
    return replace(cfg, standoffs=tuple(plan.standoffs), eps_descend_m=eps)


@step(title="servo 집기")
async def servo_pick(
    ctx: TaskContext,
    robot_id: str,
    plan: ServoPlan,
    prompt: str,
    home: WaypointRecord,
    on_grasp: Callable[[Vec3, servo.GraspFamily], None] | None = None,
    *,
    end_home: bool = True,
) -> None:
    """closed-loop 파지 실행 — rung0 진입(계획 이동) → tick 루프 → commit →
    close → 판정(재시도) → 후퇴 → 판정 → (end_home 시) home.

    end_home=False = 적치가 이어질 때 — 쥔 채 home 왕복(최장 스윙)을 없애고
    execute_place 의 운반 transit 이 withdraw 자세에서 바로 적치 접근을 계획
    (home 허브 강등, docs/motion.md §12). 실패/취소 경로는 영향 없음 —
    on_abort STOP 은 기존 그대로.

    루프 계약 (servo.py docstring = SSOT):
    - 관측은 **정지 상태** 에서만 (이동 완료 → settle → DETECT_ORIENTED).
    - 명령은 관측한 그 tick 의 TCP 기준 상대 목표 → common-mode FK 상쇄.
    - 모든 실패에 정의된 동작 (decide_tick) — 크래시/무한대기 없음, 사유는
      ServoFailed 메시지 + trace 에 남는다.
    - trace: 매 tick JSONL + 종료 summary (debug/servo_pick/<ts>/ —
      실패 재구성이 하드웨어 없이 가능해야 한다는 요구의 구현).
    - on_grasp: 채택 관측이 파지점을 갱신할 때마다 (파지점, 현재 가족) 호출
      (호출자=module 이 마커 스트림 재발행 — 계획 시점 마커가 실행 내내 고정
      표시되던 UI 구멍, 2026-07-17 사용자 리포트. 가족 동봉 = 파지 방향
      화살표/조 축 바의 실시간 소스, 2026-07-19).
    """
    cfg = _effective_cfg(primitives._SERVO_CFG, plan)
    trace = ServoTrace(prompt, robot_id)
    state = servo.ServoState()  # tick/rung 카운터 — decide_tick 의 입력
    run = servo.TrackState(  # 관측 추적 + 파지 기하 (전이 근거 = servo.py 주석)
        fam=plan.family,
        expected_xy=plan.coarse.position,
        g_tcp=plan.grasp_tcp0,
        g_point=plan.grasp_point0,
        lateral=plan.lateral0,
        fallback_width_m=plan.coarse.footprint[1],
        floor_z=plan.floor_z,
    )
    comp = servo.PlantComp()  # 명령-실측 잔차 보상 (근거 = servo.py docstring)
    tcp: TcpState | None = None
    summary: dict = {"result": "unknown", "family": run.fam.label}
    floor_suspect = False  # 하강 프로파일의 바닥 접촉 의심 (진단 — summary 로)
    midstop_resid_mm: list[float] | None = None  # 마지막 commit 재앵커 잔차

    try:
        # rung0 진입 — 현재(관측) 자세에서 직접 계획 (home 왕복 강등, §12).
        # 충돌 모델 = plan_pick resolve 게이트와 동일 (바닥 + 이웃 점군 +
        # 조 벌림). 폴백(home 경유)은 게이트 ④ path_from=home 이 사전 증명.
        await transit(
            ctx, robot_id, plan.rung0_joints, home,
            floor_z=plan.floor_z,
            obstacle_points=list(plan.neighbors),
            gripper_open=True,
        )
        await open_gripper(ctx, robot_id)

        while True:  # attempt 루프 (close 후 EMPTY 재시도)
            committed = False
            while not committed:  # tick 루프
                await asyncio.sleep(cfg.settle_s)
                det = await ctx.call(
                    Detector.Service.DETECT_ORIENTED,
                    DetectRequest(
                        robot_id=robot_id, prompts=[prompt], top_k=_TOP_K
                    ),
                    DetectOrientedResponse,
                )
                tcp = await ctx.call(
                    Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
                    robot_id=robot_id,
                )
                comp.observe(tcp.position)
                gate = servo.gate_observation(
                    det.candidates, run.expected_xy, run.last, cfg,
                    match_radius_m=(
                        cfg.reacquire_radius_m if run.reacquiring else None
                    ),
                )
                if gate.obs is None:
                    # 재앵커 — 연속 도약-기각 2건이 상호 일관하면 그쪽이
                    # 다수결 진실 (servo.TrackState.consider_reanchor 주석).
                    re = run.consider_reanchor(gate.rejected, cfg)
                    if re is not None:
                        logger.info(
                            "servo: 재앵커 — 연속 기각 2건 상호 일관 (%s)",
                            gate.reason,
                        )
                        await _trace_emit(trace, {
                            "phase": "reanchor",
                            "tick": state.ticks,
                            "reason": gate.reason,
                            "obs": _obs_record(re),
                        })
                        gate = servo.GateResult(re, "")
                lateral_err: float | None = None
                axial_err = 0.0
                fused = None
                if gate.obs is not None:
                    if run.reacquiring:
                        # 물체가 움직인 뒤 첫 재획득 — 회전했을 수 있으니 조
                        # 각도(가족 yaw)도 새 관측으로 재유도 (servo.refit_family
                        # docstring — 옛 각도 스큐 close 가 재튕김 만든 실사고)
                        new_fam = servo.refit_family(run.fam, gate.obs)
                        if new_fam is not None:
                            logger.info(
                                "servo: 재획득 yaw 재유도 — obs yaw %.1f° 로 "
                                "가족 회전 (%s)",
                                math.degrees(gate.obs.grasp_yaw),
                                run.fam.label,
                            )
                            await _trace_emit(trace, {
                                "phase": "refit",
                                "tick": state.ticks,
                                "action": "family_refit",
                                "obs_yaw_deg": round(
                                    math.degrees(gate.obs.grasp_yaw), 1
                                ),
                            })
                            run.fam = new_fam
                    run.reacquiring = False  # 재획득 완료 — 반경 원복
                    run.note_accept(gate.obs)
                    fused = await _fuse_recent(
                        ctx, run.accepted[-cfg.fuse_last_k:], gate.obs
                    )
                    run.update_grasp(gate.obs, fused, cfg)
                    _notify_grasp(on_grasp, run.g_point, run.fam)  # 마커 재발행
                    target_so = servo.standoff(
                        run.g_tcp, run.fam, cfg.standoffs[state.rung]
                    )
                    delta = (
                        target_so[0] - tcp.position[0],
                        target_so[1] - tcp.position[1],
                        target_so[2] - tcp.position[2],
                    )
                    lateral_err, axial_err = servo.split_error(delta, run.fam)

                decision = servo.decide_tick(state, gate, lateral_err, cfg)
                logger.info(
                    "servo tick %d rung=%d(%.0fmm) 관측=%s lat=%s ax=%.1fmm → "
                    "%s (%s)",
                    state.ticks, state.rung,
                    cfg.standoffs[state.rung] * 1000.0,
                    "채택" if gate.obs is not None else f"기각[{gate.reason}]",
                    f"{lateral_err * 1000:.1f}mm" if lateral_err is not None
                    else "-",
                    axial_err * 1000.0, decision.action, decision.reason,
                )
                await _trace_emit(trace, {
                    "phase": "tick",
                    "tick": state.ticks,
                    "rung": state.rung,
                    "standoff_m": cfg.standoffs[state.rung],
                    "gate_reason": gate.reason,
                    "observation": _obs_record(gate.obs),
                    "fused": _obs_record(fused),
                    "candidates_n": len(det.candidates),
                    "lateral_mm": (
                        round(lateral_err * 1000, 2)
                        if lateral_err is not None else None
                    ),
                    "axial_mm": round(axial_err * 1000, 2),
                    "grasp_tcp": [round(v, 4) for v in run.g_tcp],
                    "lateral_offset_mm": round(run.lateral * 1000, 2),
                    "comp_mm": comp.mm,
                    "tcp_position": [round(v, 4) for v in tcp.position],
                    "tcp_joints": [round(v, 4) for v in tcp.joints],
                    "action": decision.action,
                    "reason": decision.reason,
                })

                if decision.action == "hold":
                    continue
                if decision.action == "abort":
                    raise ServoFailed(decision.reason, ticks=state.ticks)
                if decision.action in ("correct", "descend"):
                    cmd = comp.apply(servo.standoff(
                        run.g_tcp, run.fam, cfg.standoffs[state.rung]
                    ))
                    try:
                        await _servo_move(
                            ctx, robot_id, cmd, run.fam.quat, trace
                        )
                    except ServoFailed:
                        # 오염 관측이 만든 허공 목표는 IK 거부가 옳다 — 관측
                        # 불신 + 재관측으로 계속 (2026-07-17 실물: 거부 1회에
                        # 태스크 전체 중단 사고). 연속 2회 = 재관측도 같은
                        # 목표를 재생산 = **관측이 진실인데 가족이 그 자리서
                        # 불가** (07-17 저녁 실물: 헛집음이 큐브를 3cm 밀어
                        # r≈0.32 경계로 — 재유도 가족 standoff 가 IK 밖, 정확한
                        # 관측 2연속을 '불신'만 하다 사망) → 그 위치 기준 가족
                        # 재-resolve 로 계속, 그것도 전멸이면 진짜 경계 → abort.
                        run.move_fails += 1
                        if run.move_fails >= 2:
                            if await _replan_family(
                                ctx, robot_id, run, state, cfg, tcp, trace
                            ):
                                run.move_fails = 0
                                continue
                            raise
                        run.distrust_last()
                        logger.warning(
                            "servo 이동 거부 (%d/2) — 관측 불신, 재관측으로 "
                            "계속: cmd=%s", run.move_fails, _fmt(cmd),
                        )
                        await _trace_emit(trace, {
                            "phase": "move",
                            "action": "rejected_hold",
                            "tick": state.ticks,
                            "move_fails": run.move_fails,
                            "cmd": [round(v, 4) for v in cmd],
                        })
                        continue
                    run.move_fails = 0
                    comp.commanded(cmd)
                    continue
                committed = True  # commit

            # ── commit + 최종 하강 (blind — 재앵커/touch-up 포함) ──
            resid_mm, suspect = await _commit_descend(
                ctx, robot_id, run, state, comp, cfg, trace, tcp
            )
            midstop_resid_mm = resid_mm
            floor_suspect = floor_suspect or suspect
            await close_gripper(ctx, robot_id)
            # ⚠ 파지 판정 2회(close 직후/withdraw 후)는 servo_pick 본문에 인라인
            # 유지 — 헬퍼로 빼면 preview 정적 인덱서가 step 트리에서 verify_grasp
            # 를 잃어 breakpoint 표면이 사라진다 (test_task_preview 안전 잠금이
            # 2026-07-19 추출 시도를 실제로 잡음).
            try:
                grip = await verify_grasp(
                    ctx, robot_id, phase="close 직후",
                    grasp_label=run.fam.label,
                )
                await _trace_emit(trace, {
                    "phase": "close", "tick": state.ticks, "action": "held",
                    **grip,
                })
                # ── 후퇴 + 슬립 재판정 — 이송 자격 검증 (attempt 루프 안:
                # 놓침/슬립 모두 재시도 대상). 후퇴 감속 — 잡은 직후 가속이
                # 얕은 파지를 흔들어 빼는 실사고 (2026-07-17).
                await _withdraw_with_fallback(ctx, robot_id, run, plan,
                                              state, cfg, trace)
                grip2 = await verify_grasp(
                    ctx, robot_id, phase="withdraw 후",
                    grasp_label=run.fam.label,
                )
                # 슬립 = close 시점 gap 대비 유지율 미달 (상대 비교 — 물체
                # 크기 무관). **실패 아님 — 경고 + 계속** (2026-07-17 실물:
                # 공중에 뜨는 순간 자중이 조 곡면을 타고 끝 홈까지 밀리는
                # 결정적 현상 — gap 216→35 두 번 재현, 끝 홈에서 load 288 로
                # 안정적으로 매달림. 내려놓고 재시도해봤자 똑같이 미끄러지고,
                # 조 끝 파지도 이송·상자 적치는 된다. 근본 = 조 마찰 패드
                # [하드웨어]). verify(load OR gap)가 "물고 있음"은 이미 보장.
                slip = (
                    grip2["gap_raw"]
                    < cfg.slip_retention * grip["gap_raw"]
                )
                await _trace_emit(trace, {
                    "phase": "withdraw", "tick": state.ticks,
                    "action": "slip" if slip else "held",
                    "close_gap_raw": grip["gap_raw"], **grip2,
                })
                if slip:
                    logger.warning(
                        "servo: withdraw 슬립 (gap %d→%d, load %s) — 조 끝 "
                        "파지로 이송 계속. 반복되면 조 접촉면 마찰 패드 권장",
                        grip["gap_raw"], grip2["gap_raw"], grip2["load_raw"],
                    )
            except GraspFailed as e:
                run.close_attempts += 1
                await _trace_emit(trace, {
                    "phase": "grasp_retry",
                    "tick": state.ticks,
                    "action": "retry",
                    "reason": str(e),
                    "attempt": run.close_attempts,
                })
                if run.close_attempts >= cfg.close_attempts:
                    raise
                logger.warning(
                    "servo: 파지 실패(%s) — 재시도 %d/%d (후퇴, 재관측)",
                    e, run.close_attempts, cfg.close_attempts,
                )
                await open_gripper(ctx, robot_id)
                await _retreat_for_retry(ctx, robot_id, run, comp, cfg)
                # 재시도 rung = 사다리 마지막 (재관측 자리 = _retreat_for_retry
                # 의 [-1] 과 짝). rung=1 하드코딩은 적응 1단 사다리(plan
                # _ENTRY_LADDERS 폴백)에서 standoffs[1] IndexError = 재시도
                # 무력화 (pnp_scenario_rework §8.6 — 07-21 [-1] sweep 이 놓친
                # 형제 자리, 회귀 테스트 잠금).
                state = servo.ServoState(rung=len(cfg.standoffs) - 1)
                continue
            break  # 파지 + 이송 자격 통과

        if end_home:  # 적치 미동반 종료 — 알려진 휴식 자세로 (place 는 운반 transit)
            await go_home(ctx, robot_id, home)
        summary.update({
            "result": "success",
            "close_attempts": run.close_attempts + 1,
            "final_grasp_tcp": [round(v, 4) for v in run.g_tcp],
            "midstop_resid_mm": midstop_resid_mm,
            "floor_contact_suspect": floor_suspect,
            "error_history_mm": [
                round(e, 1) for e in state.error_history_mm
            ],
        })
    except BaseException as e:
        summary.update({
            "result": "cancelled" if isinstance(e, asyncio.CancelledError)
            else "failed",
            "error": f"{type(e).__name__}: {e}",
            "close_attempts": run.close_attempts,
            "midstop_resid_mm": midstop_resid_mm,
            "floor_contact_suspect": floor_suspect,
            "error_history_mm": [
                round(er, 1) for er in state.error_history_mm
            ],
        })
        raise
    finally:
        try:
            await asyncio.to_thread(trace.finish, summary)
            logger.info("servo trace: %s (%s)", trace.dir, summary["result"])
        except Exception:
            logger.exception("servo trace summary 기록 실패")


async def _commit_descend(
    ctx: TaskContext,
    robot_id: str,
    run: servo.TrackState,
    state: servo.ServoState,
    comp: servo.PlantComp,
    cfg: servo.ServoConfig,
    trace: ServoTrace,
    tcp: TcpState | None,
) -> tuple[list[float] | None, bool]:
    """commit 국면 — 마지막 관측(run.g_tcp)으로 blind 최종 접근 (handoff §4).

    2단 하강 (2026-07-17 스틱션 release 스침 대응) — midstop 에서 하강방향
    재안착 후 **그 순간의 FK 실측 잔차**로 마지막 구간 재앵커 (근거·분기별
    무해성 = servo.midstop_sequence/reanchor docstring). midstop 경로가 어떤
    이유로든 실패하면 단발 하강 폴백 — 이 수정으로 IK 사망 경로가 새로 생기지
    않는다. 하강 중 FK z/load 프로파일 + touch-up + 도달 로깅까지 — 끝나면
    조가 파지 위치에 있다. 반환 = (midstop 재앵커 잔차 mm, 바닥 접촉 의심)."""
    blind_m = (
        math.dist(run.g_tcp, tuple(tcp.position))
        if tcp is not None else cfg.standoffs[state.rung]
    )
    logger.info(
        "servo commit: rung=%d blind=%.1fmm grasp_tcp=%s (%s)",
        state.rung, blind_m * 1000.0, _fmt(run.g_tcp), run.fam.label,
    )
    await _trace_emit(trace, {
        "phase": "commit",
        "tick": state.ticks,
        "rung": state.rung,
        "blind_mm": round(blind_m * 1000, 1),
        "grasp_tcp": [round(v, 4) for v in run.g_tcp],
        "comp_mm": comp.mm,
        "action": "commit",
        "reason": f"blind {blind_m * 1000:.1f}mm 최종 접근",
    })
    midstop_resid_mm: list[float] | None = None
    profile: list[dict] = []
    t_descent0 = time.monotonic()
    grasp_cmd = comp.apply(run.g_tcp)  # 폴백/미사용(midstop off) 기본값
    seq = servo.midstop_sequence(run.g_tcp, run.fam, cfg)
    if seq:
        try:
            for i, wp in enumerate(seq):
                await _descend_profiled(
                    ctx, robot_id, comp.apply(wp), run.fam.quat,
                    cfg=cfg, seg=f"midstop{i}", t0=t_descent0,
                    samples=profile,
                )
            await asyncio.sleep(cfg.commit_settle_s)
            snap = await ctx.call(
                Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(),
                TcpState, robot_id=robot_id,
            )
            resid, grasp_cmd = servo.reanchor(
                run.g_tcp, comp.apply(seq[-1]), snap.position,
                cfg.commit_residual_max_m,
            )
            midstop_resid_mm = [round(v * 1000, 1) for v in resid]
            logger.info(
                "commit midstop: 실측 잔차 %s mm → 최종 %s",
                midstop_resid_mm, _fmt(grasp_cmd),
            )
            await _trace_emit(trace, {
                "phase": "commit",
                "action": "midstop_reanchor",
                "tick": state.ticks,
                "measured": [round(v, 4) for v in snap.position],
                "resid_mm": midstop_resid_mm,
                "cmd": [round(v, 4) for v in grasp_cmd],
            })
        except asyncio.CancelledError:
            raise
        except Exception as e_m:
            grasp_cmd = comp.apply(run.g_tcp)
            logger.warning(
                "commit midstop 실패 (%s) — 단발 하강 폴백: %s",
                e_m, _fmt(grasp_cmd),
            )
            await _trace_emit(trace, {
                "phase": "commit",
                "action": "midstop_skipped",
                "tick": state.ticks,
                "reason": str(e_m),
                "cmd": [round(v, 4) for v in grasp_cmd],
            })
    await _descend_profiled(
        ctx, robot_id, grasp_cmd, run.fam.quat,
        cfg=cfg, seg="final", t0=t_descent0, samples=profile,
    )
    comp.commanded(grasp_cmd)
    suspect = False
    if profile:
        suspect = servo.descent_suspect(
            profile, ctx.spec(robot_id).gripper_index,
            cfg.descent_load_suspect_raw,
        )
        await _trace_emit(trace, {
            "phase": "commit",
            "action": "descent_profile",
            "tick": state.ticks,
            "floor_contact_suspect": suspect,
            "samples": profile,
        })
    await _touch_up(ctx, robot_id, run, grasp_cmd, comp, cfg,
                    trace, state.ticks)
    await _log_reached_tcp(
        ctx, robot_id, expected=run.g_tcp, phase="servo grasp 도달"
    )
    return midstop_resid_mm, suspect


async def _withdraw_with_fallback(
    ctx: TaskContext,
    robot_id: str,
    run: servo.TrackState,
    plan: ServoPlan,
    state: servo.ServoState,
    cfg: servo.ServoConfig,
    trace: ServoTrace,
) -> None:
    """쥔 채 후퇴 (withdraw standoff 로 감속 MoveL) — 실패 시 rung0 관절해 폴백.

    **쥔 이후의 이동 실패는 task 를 죽일 수 없다** (2026-07-17 저녁 실물:
    HELD·부하 320 직후 withdraw 사전검증 거부 → 쥔 채 사망. withdraw 목표는
    servo 가 보정한 새 위치 기준이라 계획이 검증한 적 없는 자세 — 경계에서
    자세 IK 가 안 풀릴 수 있다). 폴백 = 계획이 증명한 rung0 관절해 (IK 재풀이
    0 — retreat/재플랜과 동일 원칙: 알려진 해가 있으면 그 해로)."""
    try:
        await _move_l(
            ctx, robot_id,
            servo.standoff(
                run.g_tcp, run.fam, cfg.withdraw_standoff_m
            ),
            run.fam.quat,
            speed_scale=cfg.gentle_speed_scale,
        )
    except asyncio.CancelledError:
        raise
    except Exception as e_w:
        logger.warning(
            "withdraw MoveL 실패 (%s) — 계획 rung0 관절해 MoveJ "
            "폴백 (쥔 채 계속)", e_w,
        )
        await _trace_emit(trace, {
            "phase": "withdraw",
            "action": "fallback_rung0",
            "tick": state.ticks,
            "reason": str(e_w),
        })
        await _move_j_joints(ctx, robot_id, plan.rung0_joints)


async def _replan_family(
    ctx: TaskContext,
    robot_id: str,
    run: servo.TrackState,
    state: servo.ServoState,
    cfg: servo.ServoConfig,
    tcp: TcpState | None,
    trace: ServoTrace,
) -> bool:
    """이동 거부 연속 시 최후 수단 — 현 관측(run.last) 위치 기준 가족 전체
    재-resolve, 통과 가족으로 교체 + rung0 재진입 (plan_pick 채택과 동일 경로).

    전제: 거부가 2연속이면 재관측이 같은 목표를 재생산한 것 = 관측은 진실이고
    **기존 가족이 그 자리에서 IK 불가** (2026-07-17 저녁 실물: 헛집음이 큐브를
    3cm 밀어 경계로 — yaw 재유도만으로는 같은 tilt 가족의 standoff 가 계속
    도달 밖). 전멸/관측 없음 = False — 호출부가 기존 명시 실패 경로 유지."""
    if run.last is None:
        return False
    t0 = time.monotonic()
    # plan_pick 과 동일한 단일 resolve (§11 — 절대 yaw 격자, 2단 폐지).
    groups, metas = servo_ladder_groups(run.last, cfg, run.floor_z)
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=groups,
            floor_z=run.floor_z,
            linear=True,
            path_from=list(tcp.joints) if tcp is not None else None,
        ),
        ResolveReachableResponse,
        robot_id=robot_id,
    )
    if res.index < 0:
        logger.warning(
            "servo 재플랜: 가족 %d개 전멸 (%.1fs) — 물체가 도달 "
            "범위 밖으로 밀려난 것으로 판단: %s",
            len(groups), time.monotonic() - t0, res.message,
        )
        return False
    n_groups = len(groups)
    fam, g_point0, g_tcp0, lateral = metas[res.index]
    logger.info(
        "servo 재플랜: 가족 %d/%d 채택 (%s) — 밀린 물체 위치 기준 rung0 재진입 "
        "(%.1fs)", res.index, n_groups, fam.label, time.monotonic() - t0,
    )
    await _trace_emit(trace, {
        "phase": "replan",
        "tick": state.ticks,
        "action": "refamily",
        "family": fam.label,
        "grasp_tcp": [round(v, 4) for v in g_tcp0],
    })
    run.fam = fam
    run.g_point = g_point0
    run.g_tcp = g_tcp0
    run.lateral = lateral
    run.widths.clear()  # 조 축이 바뀌었을 수 있음 — 폭 이력 무효
    state.rung = 0
    state.corrections = 0
    state.misses = 0
    await _move_j_joints(ctx, robot_id, res.solutions[0])
    return True


def _notify_grasp(
    cb: Callable[[Vec3, servo.GraspFamily], None] | None,
    p: Vec3,
    fam: servo.GraspFamily,
) -> None:
    """파지점 갱신 통지 — 모듈 레벨 간접호출인 이유: 시나리오 프리뷰의 정적
    인덱서가 파라미터 직접 호출(`on_grasp(...)`)을 못 풀어 step 트리에 `<동적>`
    노이즈 행을 만든다 (2026-07-17 preview 계약 테스트로 포착). 마커 통지는
    step 이 아니므로 트리 미표시가 올바른 의미론. fam = 현재 파지 가족 (방향
    시각화 소스 — refit/재플랜 반영)."""
    if cb is not None:
        cb(p, fam)


async def _trace_emit(trace: ServoTrace, record: dict) -> None:
    """trace tick 기록 — blocking 파일 I/O 는 to_thread, 실패는 로깅만 (관측이
    실행을 죽이면 안 됨)."""
    try:
        await asyncio.to_thread(trace.emit, record)
    except Exception:
        logger.exception("servo trace 기록 실패 (실행 영향 없음)")


def _obs_record(obs: OrientedDetection | None) -> dict | None:
    """trace 용 관측 요약 — 원시 depth/mask 는 detector 덤프에 (timestamp 교차참조)."""
    if obs is None:
        return None
    return {
        "position": [round(v, 4) for v in obs.position],
        "base_z": round(obs.base_z, 4),
        "height_mm": round(obs.height * 1000, 1),
        "score": round(obs.score, 3),
        "grasp_yaw_deg": round(math.degrees(obs.grasp_yaw), 1),
        "footprint_mm": [round(v * 1000, 1) for v in obs.footprint],
        "points_n": len(obs.points or []),
    }


async def _fuse_recent(
    ctx: TaskContext, recent: list[OrientedDetection], latest: OrientedDetection
) -> OrientedDetection:
    """최근 채택 관측 융합 → 타깃 군집 (z/height/폭 안정화). 융합 불가/군집 없음
    이면 latest 그대로 (침묵 아님 — 로그)."""
    if len(recent) < 2:
        return latest
    res = await ctx.call(
        Detector.Service.FUSE_ORIENTED,
        FuseOrientedRequest(candidates=list(recent)),
        FuseOrientedResponse,
    )
    near = _nearest_within(res.candidates, latest.position, _VIEW_MATCH_RADIUS_M)
    if near is None:
        logger.info("servo: 융합 군집 없음 (%d 관측) — 최신 관측 단독 사용",
                    len(recent))
        return latest
    return near


async def _touch_up(
    ctx: TaskContext,
    robot_id: str,
    run: servo.TrackState,
    grasp_cmd: Vec3,
    comp: servo.PlantComp,
    cfg: servo.ServoConfig,
    trace: ServoTrace,
    tick: int,
) -> None:
    """blind 도달 touch-up — 카메라가 가려진 마지막 ~5cm 는 comp 측정 시점
    (standoff)과 자세·부하가 달라 잔차가 남는다 (2026-07-16 실물: lateral
    1.1mm 수렴인데 EMPTY — 잔여 미달이 조 끝 nip/밀어냄). 관측 불가 구간이라
    FK(엔코더) 잔차로 재보정, 상한 2회."""
    for _ in range(2):
        snap = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
            robot_id=robot_id,
        )
        resid = np.asarray(run.g_tcp) - np.asarray(snap.position)
        resid_norm = float(np.linalg.norm(resid))
        if resid_norm <= 0.003:
            break
        grasp_cmd = (
            float(grasp_cmd[0] + resid[0]),
            float(grasp_cmd[1] + resid[1]),
            float(grasp_cmd[2] + resid[2]),
        )
        logger.info(
            "servo touch-up: FK 잔차 %.1fmm → 재보정 %s",
            resid_norm * 1000.0, _fmt(grasp_cmd),
        )
        await _trace_emit(trace, {
            "phase": "touchup",
            "tick": tick,
            "resid_mm": [round(float(v) * 1000, 1) for v in resid],
            "cmd": [round(v, 4) for v in grasp_cmd],
        })
        await _move_l(
            ctx, robot_id, grasp_cmd, run.fam.quat,
            speed_scale=cfg.gentle_speed_scale,
        )
        comp.commanded(grasp_cmd)


async def _descend_profiled(
    ctx: TaskContext,
    robot_id: str,
    position: Vec3,
    quat: Quat,
    *,
    cfg: servo.ServoConfig,
    seg: str,
    t0: float,
    samples: list[dict],
) -> None:
    """하강 이동 + 진행 중 FK z/관절 load 프로파일 (진단 전용 — 제어 무관).

    이동을 task 로 띄우고 완료까지 descent_sample_hz 로 폴링 — release 가
    하강의 **어느 시점**에 났는지 / 바닥 접촉 load 지문이 trace 에 남는다.
    이동이 즉시 끝나면(=mock) 샘플 0 (첫 폴링 전에 1 tick 양보) — 스크립트
    테스트의 응답 소비를 오염하지 않는 결정성 계약."""
    move = asyncio.ensure_future(_move_l(
        ctx, robot_id, position, quat, speed_scale=cfg.gentle_speed_scale,
    ))
    await asyncio.sleep(0)  # mock 즉시완료 이동은 아래 루프 진입 전에 done
    try:
        while cfg.descent_sample_hz > 0 and not move.done():
            await _descent_sample(ctx, robot_id, seg=seg, t0=t0, samples=samples)
            if move.done():
                break
            await asyncio.sleep(1.0 / cfg.descent_sample_hz)
        await move
    except BaseException:
        # 취소/샘플 예외가 이동 task 를 고아로 남기지 않는다 (이후 STOP 은
        # on_abort 몫 — 여기선 회수만).
        if not move.done():
            move.cancel()
            try:
                await move
            except BaseException:
                pass
        raise


async def _descent_sample(
    ctx: TaskContext,
    robot_id: str,
    *,
    seg: str,
    t0: float,
    samples: list[dict],
) -> None:
    """프로파일 샘플 1건 — FK z + 관절 load. wire 실패는 프로파일 구멍일 뿐
    이동에 영향 없음 (RemoteError/Timeout 만 — 그 외는 버그라 그대로 전파)."""
    try:
        snap = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
            robot_id=robot_id,
        )
        state = await ctx.call(
            Motor.Service.READ_STATE, ReadStateRequest(), JointState,
            robot_id=robot_id,
        )
    except (RemoteError, TimeoutError):
        return
    samples.append({
        "t_ms": round((time.monotonic() - t0) * 1000),
        "seg": seg,
        "z": round(float(snap.position[2]), 4),
        "loads": list(state.loads_raw) if state.loads_raw else None,
    })


async def _retreat_for_retry(
    ctx: TaskContext,
    robot_id: str,
    run: servo.TrackState,
    comp: servo.PlantComp,
    cfg: servo.ServoConfig,
) -> None:
    """파지 재시도 후퇴 — 마지막 rung standoff 로 물러나 재관측 준비. 관측 이력
    리셋 + 재획득 반경 확대는 run.reset_for_retry (근거 = servo.py 주석).
    [-1] = 사다리 길이 무관 (적응 진입 1-rung 사다리에서 [1] 은 범위 밖)."""
    cmd = comp.apply(servo.standoff(run.g_tcp, run.fam, cfg.standoffs[-1]))
    await _move_l(  # 물체 옆에서 시작하는 후퇴 — 감속 (재밀침 방지)
        ctx, robot_id, cmd, run.fam.quat,
        speed_scale=cfg.gentle_speed_scale,
    )
    comp.commanded(cmd)
    run.reset_for_retry()


async def _servo_move(
    ctx: TaskContext,
    robot_id: str,
    position: Vec3,
    quat: Quat,
    trace: ServoTrace,
) -> None:
    """servo 보정/하강 이동 — MoveL(직선, 자세 고정) 우선, 거부 시 MoveJ 폴백
    1회 (관절 보간 — 목표 동일. 짧은 구간이라 스윙 미미), 둘 다 실패 = ServoFailed.

    실패를 침묵으로 넘기면 루프가 "명령한 증분은 항상 실행된다" 를 가정하게 된다
    (handoff §2 표) — 여기서 명시적으로 끊는다.
    """
    try:
        await _move_l(ctx, robot_id, position, quat)
        return
    except asyncio.CancelledError:
        raise
    except Exception as e_l:
        logger.warning(
            "servo 이동 MoveL 거부 (%s) — MoveJ 폴백: %s", _fmt(position), e_l
        )
        await _trace_emit(trace, {
            "phase": "move",
            "action": "movel_rejected",
            "reason": str(e_l),
            "target": [round(v, 4) for v in position],
        })
        try:
            await ctx.call(
                Motion.Service.MOVE_J,
                MoveJRequest(
                    target=PoseTarget(
                        kind="pose", position=position, quaternion=quat
                    )
                ),
                MoveJResponse,
                robot_id=robot_id,
            )
            return
        except asyncio.CancelledError:
            raise
        except Exception as e_j:
            raise ServoFailed(
                f"servo 이동 실패 — MoveL({e_l}) / MoveJ 폴백({e_j}). "
                "목표가 workspace 경계일 수 있습니다 — 물체를 로봇 쪽으로 "
                "옮긴 뒤 다시 실행하세요"
            ) from e_j
