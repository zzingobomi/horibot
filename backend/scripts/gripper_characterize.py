"""gripper_characterize — 실물 그리퍼의 "물림/빈 파지" 경계 raw 값을 먼저 뽑는다.

    uv run --no-sync python scripts/gripper_characterize.py \
        --robot so101_6dof_0 [--deploy pc] [--settle 1.5]

왜: 파지 판정(steps.verify_grasp)은 "close 명령 후 실제 도달 그리퍼 위치가 close 에서
얼마나 벌어졌나"로 물림을 본다. 그 경계(gripper_held_threshold_raw)를 지금은 close+15%
휴리스틱으로 추정하는데, **실물 값은 그리퍼·물체마다 다르다.** 이 스크립트로 본 PnP 전에
빈 close / 물체 물고 close 의 실제 raw + 부하를 2분 만에 찍어, 임계값을 추측이 아니라
데이터로 정한다 (docs/logging.md / CLAUDE.md "작업 방식: sim 소진 + 관측성 안전망").

동작: 이미 떠 있는 **분산 backend 에 Zenoh 클라이언트로 접속** (motor 모듈은 Pi 에서
돎 — 이 스크립트는 SET_GRIPPER/READ_STATE 만 호출하는 씬 클라이언트, 자기 HW 안 엶).
open → close(빈손) → [물체 삽입 프롬프트] → close(물림) 순으로 READ_STATE 를 찍는다.

주의: 실 deployment(pc 등)로 붙으면 같은 LAN 의 실 robot 그리퍼가 실제로 움직인다.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
from pathlib import Path

BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from apps.main import load_configs  # noqa: E402
from framework.runtime.app import Runtime  # noqa: E402
from framework.transport.protocol import RemoteError  # noqa: E402
from modules.motor.contract import (  # noqa: E402
    JointState,
    Motor,
    MotorKind,
    ReadStateRequest,
    SetGripperRequest,
    SetGripperResponse,
    SetTorqueRequest,
    SetTorqueResponse,
)

logger = logging.getLogger("gripper_characterize")


def _gripper_spec(robots: dict, robot_id: str) -> tuple[int, int, int]:
    """robot config 에서 (gripper_index, open_raw, close_raw) — resolve.py 와 동일 규약
    (open=limit_max, close=limit_min)."""
    robot = robots.get(robot_id)
    if robot is None:
        raise SystemExit(f"robot {robot_id!r} 가 robots.yaml 에 없음")
    grip = next((m for m in robot.motors if m.kind == MotorKind.GRIPPER), None)
    if grip is None:
        raise SystemExit(f"robot {robot_id!r} 에 gripper 모터 없음")
    return robot.motors.index(grip), grip.limit_max, grip.limit_min


async def _set_gripper(mr, robot_id: str, raw: int) -> None:  # noqa: ANN001
    await mr.call(
        Motor.Service.SET_GRIPPER, SetGripperRequest(position_raw=raw),
        SetGripperResponse, robot_id=robot_id, timeout=10.0,
    )


async def _read(mr, robot_id: str, gi: int) -> tuple[int, int | None]:  # noqa: ANN001
    st = await mr.call(
        Motor.Service.READ_STATE, ReadStateRequest(), JointState,
        robot_id=robot_id, timeout=10.0,
    )
    load = st.loads_raw[gi] if st.loads_raw is not None and gi < len(st.loads_raw) else None
    return st.positions_raw[gi], load


async def _measure(
    mr, robot_id: str, gi: int, close_raw: int, settle: float, label: str  # noqa: ANN001
) -> tuple[int, int | None]:
    await _set_gripper(mr, robot_id, close_raw)
    await asyncio.sleep(settle)
    achieved, load = await _read(mr, robot_id, gi)
    gap = abs(achieved - close_raw)
    print(
        f"[{label}] close 명령={close_raw} → 도달={achieved} "
        f"(gap={gap}, 부하={load})",
        flush=True,
    )
    return achieved, load


async def _run(args: argparse.Namespace) -> int:
    deploy, robots = load_configs(args.deploy)
    gi, open_raw, close_raw = _gripper_spec(robots, args.robot)
    print(
        f"robot={args.robot} gripper_index={gi} open_raw={open_raw} close_raw={close_raw}",
        flush=True,
    )

    from infra.transport.zenoh import ZenohTransport

    transport = ZenohTransport(deploy.zenoh)
    mr = Runtime(transport).module_runtime  # start() 안 함 — 순수 클라이언트 call 표면
    try:
        # 그리퍼가 물체를 잡고 stall 하려면 토크 ON 필요.
        await mr.call(
            Motor.Service.SET_TORQUE, SetTorqueRequest(enabled=True),
            SetTorqueResponse, robot_id=args.robot, timeout=10.0,
        )
        # 1) 열림 기준
        await _set_gripper(mr, args.robot, open_raw)
        await asyncio.sleep(args.settle)
        opened, _ = await _read(mr, args.robot, gi)
        print(f"[열림] open 명령={open_raw} → 도달={opened}", flush=True)

        # 2) 빈손 close (물체 없이 완전히 닫힘)
        empty_raw, empty_load = await _measure(
            mr, args.robot, gi, close_raw, args.settle, "빈손"
        )

        # 3) 물체 물고 close
        await _set_gripper(mr, args.robot, open_raw)
        await asyncio.sleep(args.settle)
        input("\n>>> 그리퍼 사이에 대상 물체를 넣고 Enter (Ctrl+C 취소) ...")
        held_raw, held_load = await _measure(
            mr, args.robot, gi, close_raw, args.settle, "물림"
        )

        # 4) 임계값 제안 — 빈손과 물림 사이 중간(gap 기준)이 안전한 경계.
        empty_gap = abs(empty_raw - close_raw)
        held_gap = abs(held_raw - close_raw)
        print("\n===== 결과 =====", flush=True)
        print(f"빈손 gap={empty_gap} (부하 {empty_load}) / 물림 gap={held_gap} "
              f"(부하 {held_load})", flush=True)
        if held_gap <= empty_gap:
            print(
                "⚠️ 물림 gap 이 빈손보다 크지 않음 — 위치 신호로 구분 불가.\n"
                "   물체가 너무 얇거나 토크 부족/과부하 차단 의심. 부하 신호 확인 필요.",
                flush=True,
            )
        else:
            mid_gap = (empty_gap + held_gap) // 2
            suggested = (
                close_raw + mid_gap if open_raw > close_raw else close_raw - mid_gap
            )
            heuristic = close_raw + round((open_raw - close_raw) * 0.15)
            print(
                f"제안 held_threshold_raw ≈ {suggested} (빈손·물림 중간)\n"
                f"현재 15% 휴리스틱 = {heuristic}\n"
                f"→ 이 값을 robot config/TaskRobotSpec 튜닝 근거로 사용",
                flush=True,
            )
        return 0
    except RemoteError as e:
        print(f"서비스 오류: {e.type_name}: {e.message}", flush=True)
        return 1
    finally:
        transport.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="실물 그리퍼 물림 경계 특성화")
    parser.add_argument("--robot", default="so101_6dof_0", help="대상 robot id")
    parser.add_argument("--deploy", default="pc", help="deployment yaml (zenoh 접속 설정)")
    parser.add_argument("--settle", type=float, default=1.5, help="명령 후 정착 대기(s)")
    args = parser.parse_args()
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    try:
        raise SystemExit(asyncio.run(_run(args)))
    except KeyboardInterrupt:
        print("\n취소됨", flush=True)
        raise SystemExit(130) from None


if __name__ == "__main__":
    main()
