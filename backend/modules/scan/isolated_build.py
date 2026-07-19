"""빌드 프로세스 격리 실행기 — Open3D 가 backend(=bridge) 프로세스를 굶기는 것 차단.

배경 (2026-07-19 실물 확정 — build_world off 대조 실험): 백그라운드 빌드가
backend 프로세스 안 스레드에서 돌면 Open3D(OMP 전 코어)가 같은 프로세스의
asyncio 루프 — bridge WS 릴레이 — 를 굶겨 20Hz 로봇 스트림이 몰아서 도착
→ 3D 뷰 로봇이 버벅인다. 스윕 중 검출도 같은 경쟁으로 ~12s 느려졌다 (16:11
실측). 빌드는 best-effort 백그라운드 계약이라 **늦어도 무해** — 격리 + 낮은
우선순위로 "경쟁 시에만 양보"시킨다 (유휴 코어에선 사실상 풀스피드).

구조:
- 워커 = ProcessPoolExecutor(max_workers=1, spawn) 상주 (open3d import ~2s 는
  첫 빌드 1회). 우선순위 = psutil BELOW_NORMAL (실패해도 격리만으로 태반 해결
  — 경고 없이 진행하되 debug 로그).
- progress + build.py 진단 로그(pair fitness/corr — 오늘 정합 사고를 잡은 그
  로그)는 Manager Queue 로 부모에 릴레이 — 관측성 무손실.
- 워커 사망(BrokenProcessPool) = 풀 재생성 후 예외 전파 — 호출부(scan module)
  기존 실패 경로(BuildResponse accepted=False)가 처리. 다음 빌드는 새 풀.
"""

from __future__ import annotations

import asyncio
import logging
import multiprocessing
import multiprocessing.managers
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from typing import Any, Callable

from .build import BuildResult, BuildScanInput

logger = logging.getLogger(__name__)

_LOG_KIND = "__log__"
_DONE = None  # 릴레이 종료 sentinel

_executor: ProcessPoolExecutor | None = None
_manager: multiprocessing.managers.SyncManager | None = None


def _init_worker() -> None:
    """워커 프로세스 초기화 — OS 우선순위 낮춤 (경쟁 시에만 양보)."""
    try:
        import psutil

        p = psutil.Process()
        if hasattr(psutil, "BELOW_NORMAL_PRIORITY_CLASS"):  # Windows
            p.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
        else:  # POSIX
            p.nice(10)
    except Exception:  # 우선순위 실패 = 격리만으로 진행 (치명 아님)
        logging.getLogger(__name__).debug("워커 우선순위 낮춤 실패 — 격리만 적용")


def _ensure_pool() -> tuple[ProcessPoolExecutor, Any]:
    global _executor, _manager
    if _executor is None:
        ctx = multiprocessing.get_context("spawn")  # fork 금지 (o3d/스레드 안전)
        _executor = ProcessPoolExecutor(
            max_workers=1, mp_context=ctx, initializer=_init_worker
        )
    if _manager is None:
        _manager = multiprocessing.Manager()
    return _executor, _manager


def _reset_pool() -> None:
    global _executor
    if _executor is not None:
        _executor.shutdown(wait=False, cancel_futures=True)
        _executor = None


def shutdown() -> None:
    """워커/매니저 전체 종료 — ScanModule.stop / 테스트 정리 (유령 프로세스
    방지: 검증용 backend 는 세션 안에서 반드시 kill 규약의 프로세스판)."""
    global _manager
    _reset_pool()
    if _manager is not None:
        _manager.shutdown()
        _manager = None


def _run_build_in_worker(
    scans: list[BuildScanInput], kwargs: dict, q: Any
) -> BuildResult:
    """워커 프로세스 본체 — build_mesh 실행 + progress/진단 로그를 큐로.

    build.py 의 logger 레코드(pair fitness/corr 진단)를 QueueHandler 로 부모에
    올린다 — 격리 때문에 관측성이 죽으면 안 된다 (2026-07-19 정합 사고를 그
    로그가 잡았다)."""
    import logging as worker_logging
    from logging.handlers import QueueHandler

    from . import build as recon

    class _TaggedQueueHandler(QueueHandler):
        def enqueue(self, record: worker_logging.LogRecord) -> None:
            # put_nowait — 무한 큐라 가득 참 없음 (stub _QueueLike 표면도 이것)
            self.queue.put_nowait(
                (_LOG_KIND, record.levelno, self.format(record))
            )

    build_logger = worker_logging.getLogger("modules.scan.build")
    handler = _TaggedQueueHandler(q)
    handler.setFormatter(worker_logging.Formatter("%(message)s"))
    build_logger.addHandler(handler)
    build_logger.setLevel(worker_logging.INFO)
    try:
        def progress(stage: str, percent: float, message: str) -> None:
            try:
                q.put(("progress", stage, percent, message))
            except Exception:
                pass  # 릴레이 실패가 빌드를 죽이지 않는다

        return recon.build_mesh(scans, progress=progress, **kwargs)
    finally:
        build_logger.removeHandler(handler)
        try:
            q.put(_DONE)
        except Exception:
            pass


async def run_isolated(
    scans: list[BuildScanInput],
    kwargs: dict,
    on_progress: Callable[[str, float, str], None],
) -> BuildResult:
    """격리 빌드 1회 — 완료까지 progress/로그 릴레이. 예외는 그대로 전파
    (BrokenProcessPool 은 풀 재생성 후 전파 — 다음 빌드가 새 풀로 재시도)."""
    executor, manager = _ensure_pool()
    q = manager.Queue()
    loop = asyncio.get_running_loop()
    fut = loop.run_in_executor(executor, _run_build_in_worker, scans, kwargs, q)
    relay = asyncio.create_task(_relay(q, on_progress))
    try:
        return await fut
    except BrokenProcessPool:
        _reset_pool()
        raise
    finally:
        try:
            q.put(_DONE)  # 워커가 sentinel 못 넣고 죽은 경우 릴레이 탈출 보장
        except Exception:
            pass
        await relay


async def _relay(q: Any, on_progress: Callable[[str, float, str], None]) -> None:
    """워커 큐 → 부모: progress publish + build 진단 로그 재기록."""
    while True:
        try:
            item = await asyncio.to_thread(q.get)
        except Exception:
            return  # manager 종료 등 — 릴레이만 조용히 끝냄 (빌드 결과와 무관)
        if item is _DONE:
            return
        try:
            if item[0] == _LOG_KIND:
                logging.getLogger("modules.scan.build").log(
                    item[1], "%s", item[2]
                )
            elif item[0] == "progress":
                on_progress(item[1], item[2], item[3])
        except Exception:
            logger.exception("빌드 릴레이 처리 실패 (빌드 영향 없음)")
