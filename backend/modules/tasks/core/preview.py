"""정적 프리뷰 — 시나리오 소스를 읽어 @step 호출 구조를 **실행 없이** 추출.

프리뷰는 "실행 시뮬레이션"이 아니라 **코드 구조 인덱싱**이다 (2026-07-14 확정).
얻으려는 건 step 본문의 계산 결과가 아니라 "이 step 이 저 step 을 부른다"는
호출 관계뿐이고, 그 관계는 소스 텍스트에 이미 적혀 있다 — 본문을 돌릴 이유가
없으니 로봇 모션/detection/DB/무거운 순수 연산이 탈 자리도, 모킹할 대상도 없다.
개발자 추가 규약 0: @step 만 붙이면 프리뷰가 따라온다 (dry-run/가짜 응답/
ctx 게이팅 계열은 전부 이 전제 위반이라 기각된 역사 — docs/task.md).

보장하는 것 / 안 하는 것:
  - 보여주는 건 "어떤 step 이 있고 어떻게 중첩되는가"라는 **정적 구조**.
    if 분기 선택/loop 횟수는 실행 전엔 모른다 — 해석하지 않고 conditional/
    repeated **표시만** 한다 (실행 경로 보장은 요구사항에서 제외 — 그 순간
    정적 읽기의 약점이 결함이 아니게 된다).
  - 정적으로 대상을 못 푸는 호출(지역 변수/getattr/첨자 디스패치)은 <동적>
    노드로 자리를 남긴다 — 구멍이 조용히 사라지지 않는다. 실행 중엔 기존
    런타임 trace 가 실제 진입을 정확한 이름으로 채운다 (두 경로는 완전 분리 —
    @step 선언만 공유).

step 판정 = 호출 **문법이 아니라 resolve 된 대상의 @step 표식** (step.is_step).
await 여부/호출 형태로 거르지 않는다 — step 은 개발자가 지정하는 것이지
프레임워크가 문법을 강제하는 게 아니다. 이름 해석은 inspect.getattr_static —
프리뷰 중 property/descriptor 실행 금지 (실행 0 이 프리뷰의 존재 이유).

문서화된 한계 (전부 "표시하고 넘어감" — 침묵 누락 아님):
  - 지역 변수/콜백/자료구조에 담아 부르는 step → <동적> (subtree 없음).
  - 중첩 def/lambda 안의 호출 → 정의는 실행이 아니므로 추적 안 함.
  - 지역 객체 메서드 호출(ctx.call/list.append 류)은 step 일 수 없다고 보고
    조용히 무시 — 전부 <동적> 처리하면 트리가 노이즈에 잠긴다.
  - 재귀는 루트(시나리오)와 step 안으로만 — step 아닌 일반 함수 내부는 안
    들어간다 (들어가면 stdlib 까지 기어들어감). 일반 헬퍼 뒤에 숨은 step 은
    못 본다 — 계층 규약(시나리오→step→ctx.call, docs/task.md)상 step 은 직접
    부르는 게 정상이라 실 코드에선 안 걸린다.
"""

from __future__ import annotations

import ast
import builtins
import inspect
import textwrap
import types
from dataclasses import dataclass
from typing import Any

from .contract import PreviewEntry
from .step import is_step, step_meta

DYNAMIC_NAME = "<동적>"

_SNIPPET_MAX = 40


def build_preview(fn: Any) -> list[PreviewEntry]:
    """루트(시나리오 — step 여부 무관)부터 정적 호출 트리를 preorder flat 으로.

    루트가 step 이면 depth 0 entry 로 포함, 아니면(모듈 scenario 메서드 등)
    자식 step 들이 depth 0. 본문은 어떤 경우에도 실행하지 않는다.
    """
    out: list[PreviewEntry] = []
    owner = getattr(fn, "__self__", None)  # bound method → self 해석용
    if is_step(fn):
        _emit(fn, owner, depth=0, path=(), out=out, conditional=False, repeated=False)
        return out
    root = _identity(fn)
    try:
        fn_def = _parse_def(root)
    except (OSError, TypeError, SyntaxError):
        return [PreviewEntry(name=step_meta(fn)[0], unavailable=True)]
    _emit_children(fn_def, root.__globals__, owner, depth=0, path=(root,), out=out)
    return out


# ─── 재귀 방출 ────────────────────────────────────────────────────────


def _emit(
    fn: Any,
    owner: Any,
    *,
    depth: int,
    path: tuple[Any, ...],
    out: list[PreviewEntry],
    conditional: bool,
    repeated: bool,
) -> None:
    ident = _identity(fn)
    name, title = step_meta(fn)
    entry = PreviewEntry(
        name=name, title=title, depth=depth,
        conditional=conditional, repeated=repeated,
    )
    if ident in path:  # 순환 (self/상호 재귀) — 무한 전개 대신 표시하고 끊는다
        entry.recursive = True
        out.append(entry)
        return
    try:
        fn_def = _parse_def(ident)
    except (OSError, TypeError, SyntaxError):
        entry.unavailable = True  # exec/생성 함수 등 소스 없음 — 자식 미상
        out.append(entry)
        return
    out.append(entry)
    _emit_children(
        fn_def, ident.__globals__, owner,
        depth=depth + 1, path=(*path, ident), out=out,
    )


def _emit_children(
    fn_def: ast.FunctionDef | ast.AsyncFunctionDef,
    globalns: dict[str, Any],
    owner: Any,
    *,
    depth: int,
    path: tuple[Any, ...],
    out: list[PreviewEntry],
) -> None:
    for site in _collect_sites(fn_def):
        kind, target, child_owner = _resolve_call(site.call.func, globalns, owner)
        if kind == "resolved" and is_step(target):
            _emit(
                target, child_owner, depth=depth, path=path, out=out,
                conditional=site.in_if, repeated=site.in_loop,
            )
        elif kind == "dynamic":
            out.append(PreviewEntry(
                name=DYNAMIC_NAME, title=_snippet(site.call.func), depth=depth,
                dynamic=True, conditional=site.in_if, repeated=site.in_loop,
            ))


def _identity(fn: Any) -> Any:
    """순환 판정/소스 접근 키 — bound method·@step wrapper 를 원본 함수로."""
    fn = getattr(fn, "__func__", fn)  # bound method 는 접근마다 새 객체
    return inspect.unwrap(fn)  # functools.wraps 의 __wrapped__ 체인 추적


def _parse_def(fn: Any) -> ast.FunctionDef | ast.AsyncFunctionDef:
    src = textwrap.dedent(inspect.getsource(fn))
    node = ast.parse(src).body[0]
    if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
        raise TypeError(f"함수 정의가 아님: {getattr(fn, '__name__', fn)!r}")
    return node


# ─── 호출 지점 수집 (제어 흐름은 풀지 않고 표시만) ────────────────────


@dataclass(frozen=True)
class _Site:
    call: ast.Call
    in_if: bool
    in_loop: bool


def _collect_sites(
    fn_def: ast.FunctionDef | ast.AsyncFunctionDef,
) -> list[_Site]:
    """본문의 모든 호출 지점 — 소스 순서(≈실행 순서), if/loop 조상 플래그 포함."""
    sites: list[_Site] = []

    def block(stmts: list[ast.stmt], in_if: bool, in_loop: bool) -> None:
        for s in stmts:
            visit(s, in_if, in_loop)

    def visit(node: ast.AST | None, in_if: bool, in_loop: bool) -> None:
        if node is None:
            return
        if isinstance(
            node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)
        ):
            return  # 정의는 실행이 아니다 — 콜백/중첩 def 내부 미추적 (한계 명시)
        if isinstance(node, ast.Call):
            visit(node.func, in_if, in_loop)  # 평가 순서: func → args → 호출
            for a in node.args:
                visit(a, in_if, in_loop)
            for kw in node.keywords:
                visit(kw.value, in_if, in_loop)
            sites.append(_Site(node, in_if, in_loop))
            return
        if isinstance(node, ast.If):
            visit(node.test, in_if, in_loop)  # test 는 항상 평가 — 조건부 아님
            block(node.body, True, in_loop)
            block(node.orelse, True, in_loop)
            return
        if isinstance(node, ast.IfExp):
            visit(node.test, in_if, in_loop)
            visit(node.body, True, in_loop)
            visit(node.orelse, True, in_loop)
            return
        if isinstance(node, (ast.For, ast.AsyncFor)):
            visit(node.iter, in_if, in_loop)  # iterable 은 1회 평가
            block(node.body, in_if, True)
            block(node.orelse, in_if, in_loop)
            return
        if isinstance(node, ast.While):
            visit(node.test, in_if, True)  # 반복 평가
            block(node.body, in_if, True)
            block(node.orelse, in_if, in_loop)
            return
        if isinstance(node, ast.Match):
            visit(node.subject, in_if, in_loop)
            for case in node.cases:
                visit(case.guard, True, in_loop)
                block(case.body, True, in_loop)
            return
        if isinstance(node, (ast.Try, ast.TryStar)):
            block(node.body, in_if, in_loop)
            for h in node.handlers:
                block(h.body, True, in_loop)  # 예외 시에만 — 조건부
            block(node.orelse, in_if, in_loop)
            block(node.finalbody, in_if, in_loop)
            return
        if isinstance(node, (ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)):
            for gen in node.generators:
                visit(gen.iter, in_if, in_loop)
                for cond in gen.ifs:
                    visit(cond, in_if, True)
            if isinstance(node, ast.DictComp):
                visit(node.key, in_if, True)
                visit(node.value, in_if, True)
            else:
                visit(node.elt, in_if, True)  # comprehension 몸통 = 반복
            return
        for child in ast.iter_child_nodes(node):
            visit(child, in_if, in_loop)

    block(fn_def.body, False, False)  # 데코레이터/시그니처 제외 — 본문만
    return sites


# ─── 이름 해석 (실행 0 — getattr_static) ─────────────────────────────


def _resolve_call(
    func: ast.expr, globalns: dict[str, Any], owner: Any
) -> tuple[str, Any, Any]:
    """호출 대상 정적 해석 → (kind, target, child_owner).

    kind: "resolved" = 대상 확정 (step 여부는 호출측이 표식으로 판정) /
    "local" = 지역 객체 메서드 (ctx.call 류 — step 일 수 없다고 보고 무시) /
    "dynamic" = 정적으로 못 푼 호출 (<동적> 표시 대상).
    """
    parts = _name_chain(func)
    if parts is None:
        return "dynamic", None, None  # getattr(...)()/x[k]()/(lambda)() 등
    base, *rest = parts
    if base == "self":
        if owner is None:
            return "dynamic", None, None  # unbound 문맥 — 침묵 누락 대신 구멍 표시
        obj: Any = owner
    elif base in globalns:
        obj = globalns[base]
    elif hasattr(builtins, base):
        obj = getattr(builtins, base)
    elif rest:
        return "local", None, None  # 지역 객체 메서드 — step 아님으로 간주
    else:
        return "dynamic", None, None  # 지역 변수 직접 호출 fn(...) — 대상 미상

    container: Any = None
    for attr in rest:
        container = obj
        try:
            obj = inspect.getattr_static(obj, attr)  # descriptor/property 미실행
        except AttributeError:
            return "dynamic", None, None  # base 는 풀렸는데 attr 없음 — 구멍 표시
        obj = _unwrap_static(obj)

    if isinstance(obj, types.MethodType):
        return "resolved", obj, obj.__self__
    if container is not None and not isinstance(container, (types.ModuleType, type)):
        return "resolved", obj, container  # 인스턴스에서 찾은 함수 — self 해석용
    return "resolved", obj, None


def _name_chain(func: ast.expr) -> list[str] | None:
    """`a.b.c` 형태 호출 대상 → ["a","b","c"]. Name/Attribute 체인 밖이면 None."""
    parts: list[str] = []
    node = func
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
        return list(reversed(parts))
    return None


def _unwrap_static(obj: Any) -> Any:
    """getattr_static 은 descriptor 를 안 푼다 — staticmethod/classmethod 만 수동."""
    if isinstance(obj, (staticmethod, classmethod)):
        return obj.__func__
    return obj


def _snippet(expr: ast.expr) -> str:
    """<동적> 노드의 title — 어떤 호출인지 사용자가 알 수 있게 소스 조각."""
    try:
        text = ast.unparse(expr)
    except Exception:
        return ""
    return text if len(text) <= _SNIPPET_MAX else text[: _SNIPPET_MAX - 1] + "…"
