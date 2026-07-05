"""LlmModule — 자연어 pick-and-place 명령 파서 (§17 NL PnP). PC 배치, robot-agnostic.

host 당 1 (§2.7) — 무거운 모델(Qwen)을 1회 로드, PARSE_COMMAND 마다 텍스트 → (pick,
place). 파싱은 robot 무관 (robot_id 불필요). 모델은 LlmBackend adapter 뒤 (§17.1).
DetectorModule 동형 — 백그라운드 preload(boot 안 막음) + to_thread inference(event loop
안 막음). frontend PromptPanel 이 PARSE_COMMAND → (pick,place) → Task.RUN 중계.
"""

from __future__ import annotations

import asyncio
import logging

from framework.contract.service import service
from framework.runtime.api import ModuleRuntime

from .contract import (
    Llm,
    ParseCommandRequest,
    ParseCommandResponse,
    ParsedPickPlace,
)
from .drivers.protocol import LlmBackend

logger = logging.getLogger(__name__)


class LlmModule:
    def __init__(self, runtime: ModuleRuntime, backend: LlmBackend) -> None:
        self.runtime = runtime
        self._backend = backend
        self._preload_task: asyncio.Task[None] | None = None

    async def start(self) -> None:
        logger.info("LlmModule start (host-level)")
        # 백그라운드 preload — boot 를 막지 않는다 (모델 다운로드/로드 수십 초~수 분).
        # 실패해도 첫 parse 가 lazy 재시도. GDINO 와 공유 load-lock 으로 race 차단.
        self._preload_task = asyncio.create_task(self._preload())

    async def _preload(self) -> None:
        try:
            await asyncio.to_thread(self._backend.preload)
        except Exception:
            logger.exception("LLM backend preload 실패 — 첫 parse 시 재시도")

    async def stop(self) -> None:
        logger.info("LlmModule stop (host-level)")
        if self._preload_task is not None:
            self._preload_task.cancel()
            self._preload_task = None

    @service(Llm.Service.PARSE_COMMAND)
    async def parse_command(self, req: ParseCommandRequest) -> ParseCommandResponse:
        text = req.text.strip()
        if not text:
            return ParseCommandResponse(ok=False, message="명령이 비어있음")
        # blocking 추론(GPU) → to_thread (event loop 안 막음). GDINO 동형.
        parsed = await asyncio.to_thread(self._backend.parse, text)
        if parsed is None:
            return ParseCommandResponse(
                ok=False, message=f"'{text}' 파싱 실패 — 다시 말해주세요"
            )
        return ParseCommandResponse(
            ok=True,
            parsed=ParsedPickPlace(
                pick_object=parsed.pick, place_object=parsed.place
            ),
        )
