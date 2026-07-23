"""handover run 단위 trace — "trace 와 debug 폴더만 보고 실패를 재구성" 이 기준
(servo_trace.py 규약 계승 — omx 실물 데이터가 0 이므로 첫 런의 유일한 진단 소스,
docs/omx_handover_prep.md §6).

run 마다 `debug/handover/<UTC timestamp>/` 폴더:
- `trace.jsonl` — 이벤트마다 1줄 append (crash 나도 그때까지는 남는다):
  observe 검출 원출력(끝점/길이/yaw/점수) / z=0 역투영 입출력 / 파지점 선택
  근거 / resolve 채택·전멸(그룹별 기각 사유 zip) / refine 관측·보정량 /
  held 판정(gap/load 원값) / 제시·수취 계획 / 충돌 게이트 판정.
- `summary.json` — 종료 시 1회: 결과/실패 사유(사유+다음행동)/노브 스냅샷.
  실패해도 finally 로 반드시 쓴다.

raw 관측(color/mask/평면 투영 오버레이)은 detector 가 `debug/detect/<세션>/` 에
이미 남긴다 — timestamp 교차참조 (중복 저장 안 함).

파일 I/O 는 sync — async 호출부가 asyncio.to_thread 로 감싼다.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

_TRACE_ROOT = Path("debug") / "handover"


class HandoverTrace:
    """run 1회의 trace 기록기 — emit(event)/finish(summary). 기록 실패는 로깅만
    (관측이 실행을 죽이면 안 됨 — 호출부가 try/except)."""

    def __init__(self, pick_object: str) -> None:
        self.dir = _TRACE_ROOT / time.strftime("%Y%m%d_%H%M%S")
        self._pick_object = pick_object
        self._t0 = time.time()
        self._events: list[dict[str, Any]] = []  # summary 용 요약 누적

    def emit(self, record: dict[str, Any]) -> None:
        """이벤트 1건 append (sync — 호출부가 to_thread)."""
        self.dir.mkdir(parents=True, exist_ok=True)
        record = {
            "t": round(time.time(), 3),
            "elapsed_s": round(time.time() - self._t0, 2),
            **record,
        }
        self._events.append(
            {k: record.get(k) for k in ("phase", "event", "reason")}
        )
        with (self.dir / "trace.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")

    def finish(self, summary: dict[str, Any]) -> None:
        """종료 요약 1회 (sync — 호출부가 to_thread). 성공/실패/중단 전부."""
        self.dir.mkdir(parents=True, exist_ok=True)
        out = {
            "pick_object": self._pick_object,
            "started_unix": round(self._t0, 3),
            "duration_s": round(time.time() - self._t0, 2),
            **summary,
            "events": self._events,
        }
        (self.dir / "summary.json").write_text(
            json.dumps(out, ensure_ascii=False, indent=2, default=str),
            encoding="utf-8",
        )
