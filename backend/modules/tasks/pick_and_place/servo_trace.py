"""servo 파지 run 단위 trace — "로그와 debug 폴더만 보고 실패를 재구성" 이 기준
(docs/closed_loop_grasp_handoff.md §6 — 선택이 아니라 필수 요구사항).

run 마다 `debug/servo_pick/<UTC timestamp>/` 폴더:
- `trace.jsonl` — tick 마다 1줄 append (crash 나도 그때까지의 tick 은 남는다):
  timestamp / tick / rung / 관측 원출력(위치·score·점군 수·게이트 사유) / 융합 기하 /
  lateral·axial 오차 / 판정(action+reason) / 명령 목표 / 그 tick 의 TCP(위치+관절).
- `summary.json` — 종료 시 1회: 결과(성공/실패+단계), commit 지점(rung/blind 거리/
  마지막 관측), 시도 횟수, 오차 이력. 실패해도 finally 로 반드시 쓴다.

raw 관측(depth/mask/color/points)은 detector 가 같은 PC 의 `debug/detect/<세션>/`
에 순번으로 이미 남긴다 — trace 의 timestamp 로 교차참조 (중복 저장 안 함).

파일 I/O 는 전부 sync 내부 구현 — async 호출부(steps)가 asyncio.to_thread 로 감싼다
(async 계약 — blocking 호출 이벤트 루프 밖).
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

_TRACE_ROOT = Path("debug") / "servo_pick"


class ServoTrace:
    """run 1회의 trace 기록기 — emit(tick)/finish(summary). 기록 실패는 로깅만
    (관측이 실행을 죽이면 안 됨 — 호출부가 try/except)."""

    def __init__(self, prompt: str, robot_id: str) -> None:
        self.dir = _TRACE_ROOT / time.strftime("%Y%m%d_%H%M%S")
        self._prompt = prompt
        self._robot_id = robot_id
        self._t0 = time.time()
        self._ticks = 0
        self._events: list[dict[str, Any]] = []  # summary 용 요약 누적

    def emit(self, record: dict[str, Any]) -> None:
        """tick 1건 append (sync — 호출부가 to_thread)."""
        self.dir.mkdir(parents=True, exist_ok=True)
        record = {
            "t": round(time.time(), 3),
            "elapsed_s": round(time.time() - self._t0, 2),
            **record,
        }
        self._ticks += 1
        self._events.append(
            {k: record.get(k) for k in ("tick", "phase", "action", "reason")}
        )
        with (self.dir / "trace.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

    def finish(self, summary: dict[str, Any]) -> None:
        """종료 요약 1회 (sync — 호출부가 to_thread). 성공/실패/중단 전부."""
        self.dir.mkdir(parents=True, exist_ok=True)
        out = {
            "prompt": self._prompt,
            "robot_id": self._robot_id,
            "started_unix": round(self._t0, 3),
            "duration_s": round(time.time() - self._t0, 2),
            "tick_count": self._ticks,
            **summary,
            "events": self._events,
        }
        (self.dir / "summary.json").write_text(
            json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8"
        )
