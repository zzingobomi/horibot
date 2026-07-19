"""정적 프리뷰 테스트 — build_preview 는 소스만 읽는다 (실행 0).

의미 (뒤집으면 회귀): 프리뷰가 step 본문/서비스를 실행 / if·loop 표시 누락 /
동적 호출 구멍 침묵 삭제 / 지역 메서드(ctx.call 류)·builtin 이 <동적> 노이즈로
범람 / 재귀 시나리오에서 무한 전개 / 소스 없는 step 에서 크래시 /
pick_and_place 실 시나리오 트리 변질 (breakpoint 대상 목록이 곧 이 트리).
"""

from __future__ import annotations

import textwrap

from pydantic import BaseModel

from modules.tasks.core.preview import DYNAMIC_NAME, build_preview
from modules.tasks.core.step import is_step, step, step_meta
from modules.tasks.pick_and_place.module import PickAndPlaceModule


# ─── 픽스처 step 들 (도메인 없음) ─────────────────────────────────────
#
# 본문에 AssertionError 를 심는다 — 프리뷰가 본문을 한 줄이라도 실행하면 즉사.
# ("실행 0" 이 프리뷰의 존재 이유라 모든 픽스처가 이 형태.)


@step
async def leaf() -> None:
    raise AssertionError("프리뷰가 step 본문을 실행함")


@step(title="아이")
async def child() -> None:
    raise AssertionError("프리뷰가 step 본문을 실행함")


@step
async def straight() -> None:
    await leaf()
    await child()


@step
async def brancher(flag: bool) -> None:
    if flag:
        await leaf()
    else:
        await child()


@step
async def looper(n: int) -> None:
    for _ in range(n):
        await leaf()
    while n > 0:
        await child()


@step
async def dyn_host() -> None:
    fn = leaf  # 지역 변수에 담아 호출 — 정적으로 대상 미상
    await fn()


@step
async def local_noise() -> None:
    # 지역 객체 메서드/builtin — step 일 수 없는 호출들이 <동적> 노이즈가 되면
    # 트리가 잠긴다 (ctx.call 이 대표 사례).
    items: list[int] = []
    items.append(len(sorted([3, 1])))
    await leaf()


@step
async def rec() -> None:
    await rec()  # self-recursion — 프리뷰는 표시 후 끊어야 (무한 전개 금지)


class _ClassTask:
    """메서드 step — self.x 해석 (getattr_static, 인스턴스 바인딩)."""

    @step
    async def outer(self) -> None:
        await self.inner()

    @step
    async def inner(self) -> None:
        raise AssertionError("프리뷰가 step 본문을 실행함")


def _rows(entries: list) -> list[tuple[str, int]]:
    return [(e.name, e.depth) for e in entries]


# ─── 구조 추출 ────────────────────────────────────────────────────────


def test_straight_children_flat_with_depth():
    entries = build_preview(straight)
    assert _rows(entries) == [("straight", 0), ("leaf", 1), ("child", 1)]
    assert entries[2].title == "아이"  # @step(title=) 이 프리뷰에도 전달


def test_branch_marks_conditional_without_resolving():
    entries = build_preview(brancher)
    assert _rows(entries) == [("brancher", 0), ("leaf", 1), ("child", 1)]
    # 양쪽 분기 모두 존재 + 조건부 표시 — 어느 쪽 탈지는 해석 안 함
    assert entries[1].conditional and entries[2].conditional


def test_loop_marks_repeated_without_counting():
    entries = build_preview(looper)
    assert _rows(entries) == [("looper", 0), ("leaf", 1), ("child", 1)]
    assert entries[1].repeated and entries[2].repeated
    assert not entries[1].conditional


def test_dynamic_call_leaves_visible_hole():
    entries = build_preview(dyn_host)
    assert _rows(entries) == [("dyn_host", 0), (DYNAMIC_NAME, 1)]
    assert entries[1].dynamic
    assert "fn" in entries[1].title  # 어떤 호출인지 소스 조각으로 보여줌


def test_local_methods_and_builtins_are_not_noise():
    """지역 객체 메서드(append)/builtin(len/sorted) 은 <동적> 이 아니다 —
    ctx.call 류가 전부 구멍으로 찍히면 트리가 노이즈에 잠긴다."""
    entries = build_preview(local_noise)
    assert _rows(entries) == [("local_noise", 0), ("leaf", 1)]


def test_recursion_marked_and_terminates():
    entries = build_preview(rec)
    assert _rows(entries) == [("rec", 0), ("rec", 1)]
    assert entries[1].recursive and not entries[0].recursive


def test_method_steps_resolve_via_self():
    entries = build_preview(_ClassTask().outer)
    assert _rows(entries) == [("outer", 0), ("inner", 1)]


def test_source_unavailable_marked_not_crash():
    ns: dict = {}
    exec(  # noqa: S102 — 소스 없는 함수 재현 (interactive/생성 코드 등가)
        textwrap.dedent(
            """
            async def ghost():
                ...
            """
        ),
        ns,
    )
    ghost = step(ns["ghost"])
    entries = build_preview(ghost)
    assert _rows(entries) == [("ghost", 0)]
    assert entries[0].unavailable


def test_step_marker_metadata():
    assert is_step(leaf) and not is_step(_rows)
    assert step_meta(child) == ("child", "아이")
    assert step_meta(_rows) == ("_rows", "")


# ─── 실 시나리오 (pick_and_place) — breakpoint 대상 목록 잠금 ─────────


class _DeadRuntime:
    """호출/발행 즉사 stub — 프리뷰가 wire 를 한 번이라도 타면 테스트가 죽는다."""

    def publish(self, wire_key: str, event: BaseModel) -> None:
        raise AssertionError(f"프리뷰 중 publish 발생: {wire_key}")

    async def call(self, *a, **kw):  # noqa: ANN002, ANN003, ANN201
        raise AssertionError("프리뷰 중 서비스 call 발생")


def test_pick_and_place_scenario_tree():
    """실 시나리오의 정적 트리 — **골격과 오염 불변식만** 잠근다. 전체 step
    목록의 exact 잠금은 시나리오 편집마다 깨지는 구현 미러였다 (2026-07-17
    하루 2회 파손 → 완화). 뒤집으면 잡히는 것: phase 골격 붕괴/순서 역전,
    안전 이중 파지판정 소실, <동적> 노이즈 범람, 소스 미보유 step, 프리뷰 중
    wire 실행 (_DeadRuntime)."""
    mod = PickAndPlaceModule(_DeadRuntime(), {})  # type: ignore[arg-type]
    entries = build_preview(mod.scenario)

    # phase 골격 (depth 0) — 시나리오의 뼈대 순서 (2026-07-19 스윕 통합:
    # detect 가 시나리오 직속 step 으로 승격 — pick/place/world 한 스윕)
    top = [e.name for e in entries if e.depth == 0]
    assert top == [
        "home_waypoint", "detect", "plan_pick", "plan_place", "servo_pick",
        "execute_place",
    ]
    by_top = {e.name: e for e in entries if e.depth == 0}
    # place 경로는 `if place_object:` 안 — 조건부 표시, pick 은 무조건
    assert by_top["plan_place"].conditional and by_top["execute_place"].conditional
    assert not by_top["plan_pick"].conditional
    # 안전 구조: 파지 판정은 최소 2곳 (close 직후 + withdraw 후 — 놓침/슬립 포착)
    names = [e.name for e in entries]
    assert names.count("verify_grasp") >= 2
    for required in ("close_gripper", "open_gripper", "retreat", "go_home"):
        assert required in names, f"{required} step 이 트리에서 사라짐"
    # 동적 구멍은 소수만 (현재 = plan_pick 후보 순회 resolve). trace emit 류
    # 노이즈가 <동적> 으로 범람하면 여기서 깨진다 (회귀 잠금).
    dynamic = [e for e in entries if e.dynamic]
    assert 0 < len(dynamic) <= 2 and all(e.depth >= 1 for e in dynamic)
    assert all(not e.unavailable for e in entries)
