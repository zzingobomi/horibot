"""LlmModule PARSE_COMMAND + Qwen JSON 파싱 단위테스트 (실 모델 0).

의미(뒤집으면 회귀):
  - mock backend → ok + parsed(pick/place) 전달
  - 빈 명령 → ok=False, 모델 호출 안 함 (guard)
  - backend None → ok=False (파싱 실패 전파)
  - start() 백그라운드 preload 호출
  - parse_json_response: 평문 JSON / markdown fence / JSON 없음 / pick 없음 / place null
    (drivers/parse.py 경량 모듈 — qwen.py 를 import 하면 transformers ~20s 를 문다)
"""

from __future__ import annotations

from typing import Any

from modules.llm.contract import ParseCommandRequest
from modules.llm.drivers.mock import MockLlmBackend
from modules.llm.drivers.parse import parse_json_response
from modules.llm.module import LlmModule


class _FakeRuntime:
    def publish(self, wire_key, event) -> None:  # noqa: ANN001
        pass

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None) -> Any:  # noqa: ANN001
        raise AssertionError("LLM 은 runtime.call 안 함")


async def test_parse_command_mock_returns_parsed():
    mod = LlmModule(_FakeRuntime(), MockLlmBackend(pick="white cube", place="blue box"))
    res = await mod.parse_command(ParseCommandRequest(text="흰 큐브를 파란 상자에 둬"))
    assert res.ok, res.message
    assert res.parsed is not None
    assert res.parsed.pick_object == "white cube"
    assert res.parsed.place_object == "blue box"


async def test_parse_command_empty_skips_backend():
    class _Recording:
        def __init__(self) -> None:
            self.called = False

        def parse(self, text: str):  # noqa: ANN202
            self.called = True
            return None

        def preload(self) -> None:
            pass

    backend = _Recording()
    mod = LlmModule(_FakeRuntime(), backend)
    res = await mod.parse_command(ParseCommandRequest(text="   "))
    assert not res.ok
    assert not backend.called  # 빈 명령이면 모델 호출 안 함 (guard)


async def test_parse_command_backend_none_not_ok():
    class _NoneBackend:
        def parse(self, text: str):  # noqa: ANN202
            return None

        def preload(self) -> None:
            pass

    mod = LlmModule(_FakeRuntime(), _NoneBackend())
    res = await mod.parse_command(ParseCommandRequest(text="횡설수설"))
    assert not res.ok
    assert res.parsed is None


async def test_start_triggers_preload():
    class _Recording:
        def __init__(self) -> None:
            self.preloaded = False

        def parse(self, text: str):  # noqa: ANN202
            return None

        def preload(self) -> None:
            self.preloaded = True

    backend = _Recording()
    mod = LlmModule(_FakeRuntime(), backend)
    await mod.start()
    assert mod._preload_task is not None
    await mod._preload_task
    assert backend.preloaded
    await mod.stop()


# ─── Qwen JSON 파싱 (순수 함수 — 모델/torch 로드 없이) ────────────────


def test_parse_json_response_plain():
    p = parse_json_response('{"pick": "white cube", "place": "blue box"}')
    assert p is not None and p.pick == "white cube" and p.place == "blue box"


def test_parse_json_response_markdown_fence():
    p = parse_json_response('```json\n{"pick": "red ball", "place": null}\n```')
    assert p is not None and p.pick == "red ball" and p.place is None


def test_parse_json_response_no_json_returns_none():
    assert parse_json_response("I cannot parse this command.") is None


def test_parse_json_response_missing_pick_returns_none():
    assert parse_json_response('{"place": "blue box"}') is None
