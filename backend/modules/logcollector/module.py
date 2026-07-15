from __future__ import annotations

import logging
import os
from logging.handlers import TimedRotatingFileHandler
from pathlib import Path

from framework.transport.protocol import Handle, RawTransport

logger = logging.getLogger(__name__)


_LOG_DIR = Path("logs")
_BASE_FILENAME = "horibot.log"
_BACKUP_DAYS = 14


def _dated_namer(default_name: str) -> str:
    dirpath, fname = os.path.split(default_name)
    base, _ext, date = fname.split(".")
    return os.path.join(dirpath, f"{base}-{date}.log")


class LogCollectorModule:
    def __init__(self, transport: RawTransport, log_dir: Path = _LOG_DIR) -> None:
        self._transport = transport
        self._log_dir = log_dir
        self._sub: Handle | None = None
        self._file_handler: TimedRotatingFileHandler | None = None
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
