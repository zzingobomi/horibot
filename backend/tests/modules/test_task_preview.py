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
    """실 시나리오의 정적 트리 — 이 목록이 곧 실행 전 breakpoint/run_to 대상.
    시나리오 구조를 바꾸면 이 테스트도 같이 바뀌는 게 맞다 (계약 잠금)."""
    mod = PickAndPlaceModule(_DeadRuntime(), {})  # type: ignore[arg-type]
    entries = build_preview(mod.scenario)

    assert _rows(entries) == [
        ("home_waypoint", 0),
        ("plan_pick", 0),
        ("detect", 1),  # search 스윕 = 찾기(coarse)만
        ("observe_and_plan_grasp", 1),
        ("go_home", 2),  # close 뷰 간 이동 = home 경유 (§10.4-4)
        ("try_plan_grasp", 2),  # close 뷰 후 파지 성립 검사 (adaptive 루프)
        ("fuse_target", 3),
        ("plan_place", 0),
        ("detect", 1),
        ("resolve_place", 1),
        ("execute_pick", 0),
        ("go_home", 1),
        ("pre_grasp", 1),
        ("open_gripper", 1),
        ("advance", 1),
        ("close_gripper", 1),
        ("withdraw", 1),
        ("go_home", 1),
        ("execute_place", 0),
        ("pre_place", 1),
        ("insert", 1),
        ("release", 1),
        ("retreat", 1),
        ("go_home", 1),
    ]
    by_row = {(e.name, e.depth): e for e in entries}
    # place 경로는 `if place_object:` / `if drop is not None:` 안 — 조건부 표시
    assert by_row[("plan_place", 0)].conditional
    assert by_row[("execute_place", 0)].conditional
    assert not by_row[("plan_pick", 0)].conditional
    # 동적 구멍/소스 불가 없음 — 이름 직접 호출 스타일이라 트리가 깨끗해야
    assert all(not e.dynamic and not e.unavailable for e in entries)
