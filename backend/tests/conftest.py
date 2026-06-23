"""공용 pytest fixture / helper — LAN 격리 + spawned process 정리.

두 가지 helper + 두 가지 안전망:

1. `isolated_zenoh_config()` — test process 의 zenoh peer config. multicast OFF
   + localhost TCP only. 같은 LAN 의 실 robot pi backend 가 떠있어도 reach X.
   host_mock / host_*_sim yaml 의 zenoh config 와 동일한 격리.

2. `spawn_backend(host)` — backend subprocess. `.venv/Scripts/python.exe` 직접
   호출 (uv run wrapper 우회) → child 단일 process 라 `proc.terminate()` 가
   정상 작동. testing_strategy.md §5 anchor — uv subprocess 좀비 trauma fix.

3. spawn 한 process 는 module-level `_SPAWNED_PROCS` 에 추가. atexit 가 pytest
   종료 시 살아있는 것만 종료 → 사용자 production backend 와 격리 (자기가 띄운
   것만 정리). port-based kill 안 함 — production 의 같은 port 가 같이 죽는 사고
   방지.

4. fixture finalize 에서 `proc.terminate()` / `proc.wait()` / `proc.kill()` 3단
   순차 호출. atexit 는 KeyboardInterrupt / 예외로 fixture finalize 못 도달했을
   때 마지막 안전망.
"""

from __future__ import annotations

import atexit
import os
import subprocess
import sys
from pathlib import Path

import zenoh

BACKEND_ROOT = Path(__file__).resolve().parents[1]
VENV_PYTHON = (
    BACKEND_ROOT / ".venv" / "Scripts" / "python.exe"
    if sys.platform == "win32"
    else BACKEND_ROOT / ".venv" / "bin" / "python"
)

# host_mock / host_pc_sim yaml 의 listen endpoint 와 동일.
LOCALHOST_ENDPOINT = "tcp/127.0.0.1:7447"

# atexit 가 정리할 자기 spawn list — 사용자 production backend 안 건드림.
_SPAWNED_PROCS: list[subprocess.Popen] = []


def isolated_zenoh_config() -> zenoh.Config:
    """LAN 격리 zenoh config — multicast OFF + localhost TCP only.

    test process 의 zenoh peer 가 같은 LAN 의 실 robot pi backend 와 격리.
    backend (host_mock / host_pc_sim) 의 listen endpoint 와 1:1 매칭.
    """
    cfg = zenoh.Config()
    cfg.insert_json5("mode", '"peer"')
    cfg.insert_json5("scouting/multicast/enabled", "false")
    cfg.insert_json5("connect/endpoints", f'["{LOCALHOST_ENDPOINT}"]')
    cfg.insert_json5("listen/endpoints", "[]")
    return cfg


def spawn_backend(
    host: str, env_extra: dict[str, str] | None = None
) -> subprocess.Popen:
    """venv python 직접 호출 — uv run wrapper 우회 (좀비 방지).

    `uv run python main.py` 는 uv 가 한 layer 더라 `proc.terminate()` 가 uv 만
    죽이고 child python 이 잔여 process 로 남는 경우 있음. venv python 직접
    호출하면 child 단일 process → terminate 정상.

    띄운 process 는 `_SPAWNED_PROCS` 에 추가 — atexit 가 module fixture
    finalize 누락 시 마지막 안전망으로 정리. 사용자 production backend (다른
    pid) 는 절대 건드리지 않음.
    """
    if not VENV_PYTHON.exists():
        raise RuntimeError(
            f".venv python 없음: {VENV_PYTHON}. backend/ 에서 `uv sync` 먼저."
        )
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    if env_extra:
        env.update(env_extra)
    proc = subprocess.Popen(
        [str(VENV_PYTHON), "main.py", "--host", host],
        cwd=str(BACKEND_ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=env,
    )
    _SPAWNED_PROCS.append(proc)
    return proc


def _atexit_cleanup() -> None:
    """pytest 종료 시 — 자기가 띄운 process 만 정리. production 안 건드림."""
    for proc in _SPAWNED_PROCS:
        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        except Exception:
            # atexit 안 — exception 무시 (다른 proc 도 정리해야).
            pass


atexit.register(_atexit_cleanup)
