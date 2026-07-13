"""run_task — task 트리거 서비스를 터미널에서 호출 + 진행 관찰 (frontend 없이).

    uv run --no-sync python scripts/run_task.py srv/pick_and_place/run \
        --param pick_object="white cube" [--param place_object="red box"] \
        [--deploy mock] [--timeout 120] [--stop-key srv/pick_and_place/stop]

동작: deployment 를 **in-process 부팅** (bridge 제외 — :8000 미점유, 유령 backend
사고 방지) → 인자로 받은 트리거 키를 그대로 호출 → 키의 module 세그먼트로
`stream/<ns>/*/state|trace` 를 wildcard 구독해 stdout 으로 실시간 → 최종 status 가
종료 코드 (SUCCESS=0, 그 외 1). Ctrl+C/timeout = STOP 서비스 (모터 정지) 후 종료.

registry 의존 없음 (2026-07-13) — task 시작 = 그냥 서비스 호출이고, 개발자는 자기
contract 키를 안다. --param 값은 클라이언트에서 검증하지 않는다 — 관용 모델로
보내고 **서비스의 RunRequest 가 검증** (원래 SSOT). 틀리면 RemoteError 사유 출력.
값은 JSON 우선 해석 ("5"→int, "true"→bool), 실패 시 문자열.

주의: mock 은 zenoh multicast off 라 LAN 안전. pc 등 실 deployment 로 쓸 땐 같은
LAN 의 실 robot 에 모션이 나감 — 실행 전 확인.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict

# Repo imports (script standalone) — backend 를 path 에.
BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from apps.main import build_runtime, load_configs  # noqa: E402
from framework.contract.publisher import decode_event  # noqa: E402
from framework.transport.protocol import RemoteError  # noqa: E402
from modules.tasks.core.contract import TaskState, TaskStatus, TaskTrace  # noqa: E402

logger = logging.getLogger("run_task")

_FINAL = (TaskStatus.SUCCESS, TaskStatus.FAILED, TaskStatus.STOPPED)


class _LooseModel(BaseModel):
    """관용 wire 모델 — 검증은 서비스 쪽 (RunRequest 가 SSOT)."""

    model_config = ConfigDict(extra="allow")


class _RunRes(_LooseModel):
    accepted: bool = True  # 트리거 응답 규약 모양 (accepted/message) — 관용 디코드
    message: str = ""


class _CtrlRes(_LooseModel):
    ok: bool = False
    message: str = ""


def _parse_key(key: str) -> str:
    """`srv/<ns>/<name>` → ns. 스트림 wildcard 구독과 STOP 파생의 기준."""
    parts = key.split("/")
    if len(parts) < 3 or parts[0] != "srv":
        raise SystemExit(f"트리거 키 형식은 srv/<task>/<name>: {key!r}")
    return parts[1]


def _parse_params(pairs: list[str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for p in pairs:
        if "=" not in p:
            raise SystemExit(f"--param 형식은 key=value: {p!r}")
        k, v = p.split("=", 1)
        try:
            out[k.strip()] = json.loads(v)  # "5"→int, "true"→bool, '"x"'→str
        except json.JSONDecodeError:
            out[k.strip()] = v  # 평문 문자열
    return out


async def _run(args: argparse.Namespace) -> int:
    ns = _parse_key(args.key)
    stop_key = args.stop_key or f"srv/{ns}/stop"

    deploy, robots = load_configs(args.deploy)
    # bridge 제외 — :8000 바인딩 회피 (유령 backend 가 full-boot pytest 를 hang 시킨
    # 사고 기록). task 실행에 bridge 는 불필요.
    deploy.modules = [m for m in deploy.modules if m.name != "bridge"]

    from infra.transport.zenoh import ZenohTransport

    transport = ZenohTransport(deploy.zenoh)
    runtime = build_runtime(deploy, robots, transport)
    exit_code = 1
    try:
        await runtime.start()

        final = asyncio.Event()
        loop = asyncio.get_running_loop()
        last_status: list[TaskState] = []
        printed: set[tuple[int, str]] = set()

        def on_state(payload: bytes) -> None:
            st = decode_event(TaskState, payload)
            last_status.append(st)
            line = f"[state] {st.status.value}"
            if st.current_name:
                line += f"  @{st.current_name}"
            if st.error:
                line += f"  — {st.error}"
            print(line, flush=True)
            if st.status in _FINAL:
                loop.call_soon_threadsafe(final.set)

        def on_trace(payload: bytes) -> None:
            tr = decode_event(TaskTrace, payload)
            for i, e in enumerate(tr.entries):
                key = (i, e.status)
                if key in printed:
                    continue
                printed.add(key)
                detail = f"  ({e.detail})" if e.detail else ""
                indent = "  " * e.depth  # 중첩 step — depth 들여쓰기
                title = f" [{e.title}]" if e.title else ""
                print(
                    f"[trace] {e.status:<9} {indent}{e.name}{title}{detail}",
                    flush=True,
                )

        # robot 목록 불필요 — {robot_id} 자리를 wildcard 로 구독.
        handles = [
            transport.subscribe(f"stream/{ns}/*/state", on_state),
            transport.subscribe(f"stream/{ns}/*/trace", on_trace),
        ]

        try:
            res = await runtime.module_runtime.call(
                args.key, _LooseModel(**_parse_params(args.param)), _RunRes,
                timeout=10.0,
            )
        except RemoteError as e:
            # param 검증 실패 포함 — 서비스(RunRequest)가 SSOT, 사유 그대로 표출.
            print(f"트리거 거부: {e.type_name}: {e.message}", flush=True)
            return 1
        if not res.accepted:
            print(f"RUN 거부: {res.message}", flush=True)
            return 1
        print(f"RUN accepted — {args.key}", flush=True)

        try:
            await asyncio.wait_for(final.wait(), timeout=args.timeout)
        except TimeoutError:
            print(f"timeout {args.timeout}s — STOP 송신 ({stop_key})", flush=True)
            await runtime.module_runtime.call(
                stop_key, _LooseModel(), _CtrlRes, timeout=10.0
            )
            await asyncio.wait_for(final.wait(), timeout=15.0)
        except (KeyboardInterrupt, asyncio.CancelledError):
            print(f"중단 요청 — STOP 송신 ({stop_key})", flush=True)
            await runtime.module_runtime.call(
                stop_key, _LooseModel(), _CtrlRes, timeout=10.0
            )
            await asyncio.wait_for(final.wait(), timeout=15.0)

        status = last_status[-1].status if last_status else None
        exit_code = 0 if status == TaskStatus.SUCCESS else 1
        print(f"최종: {status.value if status else '?'} (exit {exit_code})", flush=True)
        for h in handles:
            try:
                h.undeclare()
            except Exception:
                pass
        return exit_code
    finally:
        await runtime.stop()
        transport.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="task 트리거 터미널 실행기")
    parser.add_argument("key", help="트리거 서비스 키 (예: srv/pick_and_place/run)")
    parser.add_argument(
        "--param", action="append", default=[], help="key=value (반복 가능, 값 JSON 우선)"
    )
    parser.add_argument(
        "--stop-key", default="", help="중단 서비스 키 (기본: srv/<task>/stop)"
    )
    parser.add_argument("--deploy", default="mock", help="deployment yaml 이름")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()
    # Windows 콘솔 cp949 — 한글/대시 출력 깨짐 방지 (utf-8 강제).
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    raise SystemExit(asyncio.run(_run(args)))


if __name__ == "__main__":
    main()
