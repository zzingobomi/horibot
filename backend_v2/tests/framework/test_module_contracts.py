"""build_module_contracts — module attribution 열거 (contract_graph_viewer.md §5.1).

build_snapshot 이 버리는 module 별 attribution + publish/subscribe 방향을 이 primitive
가 보존하는지 격리 검증 (fake module 로 — apps/mock 무관). 검증 축:
- attribution: 어느 module 이 무엇을 serve/publish/subscribe (방향 보존)
- dedup: per-robot 인스턴스 (같은 class) 는 한 ModuleContract 로
- robot_scoped: wire_key 의 {robot_id} placeholder 유무 정확
- empty module (contract 0) 도 열거 (framework 는 editorialize X — node 제외는 apps)
- 정렬: module_id / 각 wire_key 그룹
"""

from __future__ import annotations

from pydantic import BaseModel

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.snapshot import build_module_contracts


class _Req(BaseModel):
    pass


class _Res(BaseModel):
    ok: bool


class _Evt(BaseModel):
    robot_id: str


@publishes(
    ("stream/alpha/{robot_id}/out", _Evt),
    ("event/alpha/{robot_id}/blip", _Evt),
)
class _AlphaModule:
    """robot-scoped — service + publish 2 + subscribe 1."""

    @service("srv/alpha/{robot_id}/do")
    def do(self, req: _Req) -> _Res:
        return _Res(ok=True)

    @subscriber("stream/beta/{robot_id}/feed")
    def on_feed(self, evt: _Evt) -> None: ...


class _GlobalModule:
    """robot-agnostic — {robot_id} placeholder 없음 → robot_scoped False."""

    @service("srv/global/ping")
    def ping(self, req: _Req) -> _Res:
        return _Res(ok=True)


class _EmptyModule:
    """contract 0 (Bridge 같은 relay) — 열거는 되되 empty tuple."""


def test_attribution_direction_preserved():
    [alpha] = [c for c in build_module_contracts([_AlphaModule()])]
    assert alpha.module_id == "_AlphaModule"
    # @service → services (owner)
    assert alpha.services == ("srv/alpha/{robot_id}/do",)
    # @publishes → publishes (output), sorted
    assert alpha.publishes == (
        "event/alpha/{robot_id}/blip",
        "stream/alpha/{robot_id}/out",
    )
    # @subscriber → subscribes (input) — publish 와 반대 방향
    assert alpha.subscribes == ("stream/beta/{robot_id}/feed",)


def test_robot_scoped_flag():
    contracts = {c.module_id: c for c in build_module_contracts([_AlphaModule(), _GlobalModule()])}
    assert contracts["_AlphaModule"].robot_scoped is True
    assert contracts["_GlobalModule"].robot_scoped is False
    # global service 는 {robot_id} 없는 raw key
    assert contracts["_GlobalModule"].services == ("srv/global/ping",)


def test_per_robot_instances_dedup_by_class():
    # 같은 class 두 인스턴스 (per-robot) → template 동일 → 한 ModuleContract
    contracts = build_module_contracts([_AlphaModule(), _AlphaModule()])
    assert [c.module_id for c in contracts] == ["_AlphaModule"]


def test_empty_module_enumerated():
    [empty] = build_module_contracts([_EmptyModule()])
    assert empty.module_id == "_EmptyModule"
    assert empty.services == ()
    assert empty.publishes == ()
    assert empty.subscribes == ()
    assert empty.robot_scoped is False


def test_sorted_by_module_id():
    # 입력 순서 무관 — module_id 알파벳 정렬 (안정 diff)
    contracts = build_module_contracts([_GlobalModule(), _EmptyModule(), _AlphaModule()])
    assert [c.module_id for c in contracts] == [
        "_AlphaModule",
        "_EmptyModule",
        "_GlobalModule",
    ]
