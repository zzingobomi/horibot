"""run_task — task 를 터미널에서 실행 (frontend 없이 작성 루프 완결).

    uv run --no-sync python scripts/run_task.py pick_and_place \
        --param pick_object="white cube" [--param place_object="red box"] \
        [--deploy mock] [--timeout 120]

동작: deployment 를 **in-process 부팅** (bridge 제외 — :8000 미점유, 유령 backend
사고 방지) → GET /tasks 등가 (core registry) 에서 task 조회 → RUN 호출 →
STATE/TRACE 를 stdout 으로 실시간 → 최종 status 가 종료 코드 (SUCCESS=0, 그 외 1).
Ctrl+C = STOP 서비스 (모터 정지) 후 종료. finally 에서 runtime/transport 정리 보장.

주의: mock 은 zenoh multicast off 라 LAN 안전. pc 등 실 deployment 로 쓸 땐 같은
LAN 의 실 robot 에 모션이 나감 — 실행 전 확인.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
from pathlib import Path

from pydantic import BaseModel, ConfigDict

# Repo imports (script standalone) — backend 를 path 에.
BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from apps.main import build_runtime, load_configs  # noqa: E402
from framework.contract.publisher import decode_event  # noqa: E402
from modules.tasks.core.metadata import TaskMetadata, task_infos  # noqa: E402
from modules.tasks.core.contract import TaskState, TaskStatus, TaskTrace  # noqa: E402

logger = logging.getLogger("run_task")

_FINAL = (TaskStatus.SUCCESS, TaskStatus.FAILED, TaskStatus.STOPPED)


class _RunRes(BaseModel):
    """task 모듈 RunResponse 규약 모양 (accepted/message) — CLI 는 관용 디코드."""

    model_config = ConfigDict(extra="allow")
    accepted: bool = False
    message: str = ""


class _CtrlRes(BaseModel):
    model_config = ConfigDict(extra="allow")
    ok: bool = False
    message: str = ""


class _EmptyReq(BaseModel):
    model_config = ConfigDict(extra="allow")


def _find_task(name: str) -> TaskMetadata:
    metas = {m.name: m for m in task_infos()}
    if name not in metas:
        raise SystemExit(
            f"task '{name}' 없음 — 등록: {sorted(metas) or '(deployment 에 task 모듈 없음)'}"
        )
    return metas[name]


def _parse_params(pairs: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for p in pairs:
        if "=" not in p:
            raise SystemExit(f"--param 형식은 key=value: {p!r}")
        k, v = p.split("=", 1)
        out[k.strip()] = v
    return out


async def _run(args: argparse.Namespace) -> int:
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

        meta = _find_task(args.task)
        ns = str(meta.run).split("/")[1]  # "srv/<task>/run" → 표준 표면 규약
        req = meta.params_model(**_parse_params(args.param))

        final = asyncio.Event()
        loop = asyncio.get_running_loop()
        last_status: list[TaskState] = []
        printed: set[tuple[int, str]] = set()

        def on_state(payload: bytes) -> None:
            st = decode_event(TaskState, payload)
            last_status.append(st)
            line = f"[state] {st.status.value}"
            if st.current_label:
                line += f"  @{st.current_label}"
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
                print(f"[trace] {e.status:<9} {e.label} <{e.kind}>{detail}", flush=True)

        handles = [
            transport.subscribe(f"stream/{ns}/{rid}/state", on_state)
            for rid in meta.robots
        ] + [
            transport.subscribe(f"stream/{ns}/{rid}/trace", on_trace)
            for rid in meta.robots
        ]

        res = await runtime.module_runtime.call(meta.run, req, _RunRes, timeout=10.0)
        if not res.accepted:
            print(f"RUN 거부: {res.message}", flush=True)
            return 1
        print(f"RUN accepted — task={meta.name} robots={meta.robots}", flush=True)

        try:
            await asyncio.wait_for(final.wait(), timeout=args.timeout)
        except TimeoutError:
            print(f"timeout {args.timeout}s — STOP 송신", flush=True)
            await runtime.module_runtime.call(
                f"srv/{ns}/stop", _EmptyReq(), _CtrlRes, timeout=10.0
            )
            await asyncio.wait_for(final.wait(), timeout=15.0)
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("중단 요청 — STOP 송신", flush=True)
            await runtime.module_runtime.call(
                f"srv/{ns}/stop", _EmptyReq(), _CtrlRes, timeout=10.0
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
    parser = argparse.ArgumentParser(description="task 터미널 실행기")
    parser.add_argument("task", help="task 이름 (예: pick_and_place)")
    parser.add_argument(
        "--param", action="append", default=[], help="key=value (반복 가능)"
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
