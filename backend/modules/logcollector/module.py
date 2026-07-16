from __future__ import annotations

import logging
import os
import time
from datetime import datetime
from logging.handlers import TimedRotatingFileHandler
from pathlib import Path

from framework.transport.protocol import Handle, RawTransport

logger = logging.getLogger(__name__)


_LOG_DIR = Path("logs")
_PREFIX = "horibot"
_BACKUP_DAYS = 14


class DatedMidnightHandler(TimedRotatingFileHandler):
    def __init__(self, log_dir: Path, prefix: str, backup_count: int) -> None:
        self._dir = log_dir
        self._prefix = prefix
        self._backup = backup_count
        log_dir.mkdir(parents=True, exist_ok=True)
        super().__init__(
            filename=str(self._dated_path()),
            when="midnight",
            backupCount=backup_count,
            encoding="utf-8",
        )

    def _dated_path(self) -> Path:
        return self._dir / f"{self._prefix}-{datetime.now().strftime('%Y-%m-%d')}.log"

    def doRollover(self) -> None:  # noqa: N802 (stdlib override 이름)
        if self.stream:
            self.stream.close()
            self.stream = None  # type: ignore[assignment]
        self.baseFilename = os.path.abspath(str(self._dated_path()))
        self.stream = self._open()
        current = int(time.time())
        nxt = self.computeRollover(current)
        while nxt <= current:
            nxt += self.interval
        self.rolloverAt = nxt
        self._cleanup_old()

    def _cleanup_old(self) -> None:
        if self._backup <= 0:
            return
        files = sorted(self._dir.glob(f"{self._prefix}-*.log"))
        for f in files[: -self._backup]:
            try:
                f.unlink()
            except OSError:
                pass


class LogCollectorModule:
    def __init__(self, transport: RawTransport, log_dir: Path = _LOG_DIR) -> None:
        self._transport = transport
        self._log_dir = log_dir
        self._sub: Handle | None = None
        self._file_handler: TimedRotatingFileHandler | None = None
        self._sink = logging.getLogger("horibot.logcollector.sink")

    def start(self) -> None:
        self._log_dir.mkdir(parents=True, exist_ok=True)
        handler = DatedMidnightHandler(self._log_dir, _PREFIX, _BACKUP_DAYS)
        handler.setFormatter(logging.Formatter("%(message)s"))

        self._file_handler = handler
        self._sink.handlers = [handler]
        self._sink.setLevel(logging.INFO)
        self._sink.propagate = False

        self._sub = self._transport.subscribe("log/**", self._on_line)
        logger.info(
            "LogCollector 시작 — log/** 구독 → %s",
            handler.baseFilename,
        )

    def _on_line(self, payload: bytes) -> None:
        try:
            line = payload.decode("utf-8", errors="replace")
        except Exception:
            return
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
