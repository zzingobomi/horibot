"""LogCollector — 여러 host 의 로그를 중앙 한 파일로 모으는 모듈 (PC 에만 배치).

계약 없음 (bridge 와 동류 — 순수 인프라인데도 "정확히 한 host 에서만 떠야" 해서
런타임 인프라가 아니라 모듈로 두고 deployment yaml 로 배치한다, docs/logging.md §1).
`log/**` 를 raw 구독해 수신한 줄을 **그대로(verbatim)** 일단위 rotation 파일에 append.

레코드는 발행 시점에 이미 자기완결(host 포함, docs/logging.md §2)이라 collector 는
키 파싱·host 재추출 안 하고 "받아서 append" 만 하는 dumb 소비자다.

sink = 지금은 파일 하나. DB 로도 남기려면 여기 write 대상 한 곳만 바꾼다 (§4).
"""

from __future__ import annotations

import logging
import os
from logging.handlers import TimedRotatingFileHandler
from pathlib import Path

from framework.transport.protocol import Handle, RawTransport

logger = logging.getLogger(__name__)

# logs/ = 중앙 로그 전용 (docs/logging.md §3). debug/ 아티팩트와 분리 — 라이프사이클이
# 다르다: debug/ = 자주 비우는 scratch, logs/ = 14일 rolling 으로 보존하는 기록.
# 섞으면 debug scratch 를 비우다 로그 히스토리까지 날아간다. gitignore 됨.
_LOG_DIR = Path("logs")
_BASE_FILENAME = "horibot.log"
_BACKUP_DAYS = 14


def _dated_namer(default_name: str) -> str:
    """rotation 파일명을 `horibot-YYYY-MM-DD.log` 로 (기본은 `horibot.log.YYYY-MM-DD`)."""
    dirpath, fname = os.path.split(default_name)
    # fname = "horibot.log.2026-07-15" → base="horibot", date="2026-07-15"
    base, _ext, date = fname.split(".")
    return os.path.join(dirpath, f"{base}-{date}.log")


class LogCollectorModule:
    def __init__(self, transport: RawTransport, log_dir: Path = _LOG_DIR) -> None:
        self._transport = transport
        self._log_dir = log_dir
        self._sub: Handle | None = None
        self._file_handler: TimedRotatingFileHandler | None = None
        # 파일 쓰기 전용 격리 로거 — propagate=False 라 root(=발행 핸들러)로 안 샌다.
        # collector 가 파일 쓰며 낸 로그가 다시 발행→수신→기록 되는 루프를 원천 차단.
        self._sink = logging.getLogger("horibot.logcollector.sink")

    def start(self) -> None:
        self._log_dir.mkdir(parents=True, exist_ok=True)
        handler = TimedRotatingFileHandler(
            filename=str(self._log_dir / _BASE_FILENAME),
            when="midnight",
            backupCount=_BACKUP_DAYS,
            encoding="utf-8",
        )
        handler.suffix = "%Y-%m-%d"
        handler.namer = _dated_namer
        # 수신 줄은 이미 완성형 — 접두 조립 없이 그대로 기록 (verbatim).
        handler.setFormatter(logging.Formatter("%(message)s"))

        self._file_handler = handler
        self._sink.handlers = [handler]
        self._sink.setLevel(logging.INFO)
        self._sink.propagate = False

        self._sub = self._transport.subscribe("log/**", self._on_line)
        logger.info(
            "LogCollector 시작 — log/** 구독 → %s",
            self._log_dir / _BASE_FILENAME,
        )

    def _on_line(self, payload: bytes) -> None:
        # 수신 줄을 그대로 append. 디코드 실패해도 서비스 무사.
        try:
            line = payload.decode("utf-8", errors="replace")
        except Exception:
            return
        # 격리 sink 로 기록 → propagate=False 라 발행 핸들러로 안 되돌아감.
        self._sink.info(line)

    def stop(self) -> None:
        if self._sub is not None:
            try:
                self._sub.undeclare()
            except Exception:
                pass
            self._sub = None
        self._sink.handlers = []
        if self._file_handler is not None:
            self._file_handler.close()
            self._file_handler = None
