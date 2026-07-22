"""관절 공간 경로 계획 — RRT-Connect + shortcut smoothing (순수 계산).

home 허브 강등의 계산부 (docs/motion.md §12): "긴 이동은 home 경유" 라는 옛
실행 계약을 "임의 시작→목표의 충돌 없는 직접 경로" 로 바꾸는 플래너.
collision_fn 주입 (PyBullet 무관) — fast 단위테스트 대상 (합성 충돌장으로
결정적 재현). PyBullet 결선은 module.plan_path (기존 충돌 세계 재사용).

알고리즘 = RRT-Connect (Kuffner & LaValle 2000, 양방향 트리 + greedy connect)
— MoveIt/OMPL 의 사실상 기본 플래너와 같은 계보. cuRobo/OMPL 대신 이걸 직접
얹은 근거 = docs/motion.md §12 (신규 의존성 0 + 충돌검사 SSOT 유지).

결정성: rng = 고정 seed 시퀀스 — 같은 입력 = 같은 결과 (adapters/pybullet.py
IK restart 의 "복권 금지" 정책과 동일. 2026-07-09 복권 버그 재발 방지).

비용 모델: 지배 항 = collision_fn 호출 수 (PyBullet 질의 ~0.1-0.5ms). 자유
공간 이동은 직선 fast-path 가 수십 회로 끝난다 — RRT 는 직선이 막힌 경우만.
전체 예산 = max_checks (Pi 배치 대비 폭주 방지 — 소진 시 명시 실패, 호출자가
home 경유로 폴백하는 계약이라 안전).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

CollisionFn = Callable[[list[float]], bool]

# 엣지 충돌 검사 해상도 — 최대 관절 이동 기준 (module._JOINT_PATH_STEP_RAD 와
# 같은 5°: SO-101 링크 끝 이동 ~수 mm 해상도, 충돌만 검사라 촘촘해도 싸다).
STEP_RAD = math.radians(5.0)
# 트리 확장 스텝 — 샘플 방향으로 한 번에 뻗는 최대 관절 이동.
_EXTEND_RAD = math.radians(30.0)
# seed 당 샘플 상한 / seed 시퀀스 (결정적) / shortcut 시도 횟수
_MAX_SAMPLES = 600
_SEEDS = (0, 1, 2)
_SHORTCUT_ITERS = 80
# 전체 collision_fn 호출 예산 — 소진 = 명시 실패 (호출자 폴백 계약)
_MAX_CHECKS = 20000
# connect greedy 루프 안전 상한 (이론상 d/extend 로 종료하지만 방어)
_CONNECT_CAP = 200


@dataclass(frozen=True, slots=True)
class PlanResult:
    """계획 산출 — path=None 이면 reason 이 사유 (호출자 로그/폴백 판단).

    path 는 [start, ..., goal] 전체 (성공 시). checks = collision_fn 호출 수
    (관측성 — Pi 배치 성능 계측의 1차 데이터, module 이 로그로 노출)."""

    path: list[list[float]] | None
    reason: str
    direct: bool  # 직선이 그대로 자유 — RRT 없이 통과
    checks: int


class _Budget:
    """collision_fn 호출 계수 + 예산 — 소진 시 _BudgetExhausted."""

    __slots__ = ("fn", "checks", "limit")

    def __init__(self, fn: CollisionFn, limit: int) -> None:
        self.fn = fn
        self.checks = 0
        self.limit = limit

    def __call__(self, q: list[float]) -> bool:
        if self.checks >= self.limit:
            raise _BudgetExhausted
        self.checks += 1
        return self.fn(q)


class _BudgetExhausted(Exception):
    pass


def _maxabs(a: Sequence[float], b: Sequence[float]) -> float:
    return max(abs(x - y) for x, y in zip(a, b))


def _edge_free(
    a: Sequence[float], b: Sequence[float], coll: CollisionFn, step_rad: float
) -> bool:
    """a→b 관절 보간 엣지 충돌 검사 — a 는 이미 검증됐다고 가정, b 포함."""
    qa = np.asarray(a, dtype=float)
    qb = np.asarray(b, dtype=float)
    n = max(1, int(math.ceil(float(np.max(np.abs(qb - qa))) / step_rad)))
    for k in range(1, n + 1):
        q = [float(v) for v in qa + (qb - qa) * (k / n)]
        if coll(q):
            return False
    return True


class _Tree:
    __slots__ = ("nodes", "parents")

    def __init__(self, root: list[float]) -> None:
        self.nodes: list[list[float]] = [list(root)]
        self.parents: list[int] = [-1]

    def nearest(self, q: Sequence[float]) -> int:
        arr = np.asarray(self.nodes, dtype=float)
        d = ((arr - np.asarray(q, dtype=float)) ** 2).sum(axis=1)
        return int(np.argmin(d))

    def path_to_root(self, i: int) -> list[list[float]]:
        out: list[list[float]] = []
        while i != -1:
            out.append(self.nodes[i])
            i = self.parents[i]
        return out  # [node_i, ..., root]


def _extend(
    tree: _Tree,
    q_target: list[float],
    coll: CollisionFn,
    step_rad: float,
    extend_rad: float,
) -> int | None:
    """nearest 에서 q_target 방향으로 extend_rad 만큼 뻗기 — 엣지 자유면 추가."""
    i = tree.nearest(q_target)
    qn = tree.nodes[i]
    d = _maxabs(q_target, qn)
    if d <= extend_rad:
        q_new = list(q_target)
    else:
        f = extend_rad / d
        q_new = [a + (b - a) * f for a, b in zip(qn, q_target)]
    if not _edge_free(qn, q_new, coll, step_rad):
        return None
    tree.nodes.append(q_new)
    tree.parents.append(i)
    return len(tree.nodes) - 1


def _connect(
    tree: _Tree,
    q_target: list[float],
    coll: CollisionFn,
    step_rad: float,
    extend_rad: float,
) -> int | None:
    """q_target 까지 greedy 연속 extend — 도달하면 그 노드 index, 막히면 None."""
    for _ in range(_CONNECT_CAP):
        idx = _extend(tree, q_target, coll, step_rad, extend_rad)
        if idx is None:
            return None
        if _maxabs(tree.nodes[idx], q_target) <= 1e-9:
            return idx
    return None


def _rrt_connect(
    start: list[float],
    goal: list[float],
    limits: Sequence[tuple[float, float]],
    coll: CollisionFn,
    rng: np.random.Generator,
    step_rad: float,
    extend_rad: float,
    max_samples: int,
) -> list[list[float]] | None:
    ta, tb = _Tree(start), _Tree(goal)
    a_is_start = True
    lo = np.asarray([pair[0] for pair in limits], dtype=float)
    hi = np.asarray([pair[1] for pair in limits], dtype=float)
    for _ in range(max_samples):
        q_rand = [float(v) for v in rng.uniform(lo, hi)]
        idx_a = _extend(ta, q_rand, coll, step_rad, extend_rad)
        if idx_a is not None:
            idx_b = _connect(tb, ta.nodes[idx_a], coll, step_rad, extend_rad)
            if idx_b is not None:
                seg_a = ta.path_to_root(idx_a)[::-1]  # ta_root … meet
                seg_b = tb.path_to_root(idx_b)  # meet … tb_root
                full = seg_a + seg_b[1:]  # meet 중복 제거 (connect 는 exact 도달)
                return full if a_is_start else full[::-1]
        ta, tb = tb, ta
        a_is_start = not a_is_start
    return None


def _shortcut(
    path: list[list[float]],
    coll: CollisionFn,
    rng: np.random.Generator,
    step_rad: float,
    iters: int,
) -> list[list[float]]:
    """무작위 (i,j) 쌍 직선 대체 — RRT 특유의 갈지자 경로를 다듬는다 (표준 후처리)."""
    path = [list(q) for q in path]
    for _ in range(iters):
        if len(path) <= 2:
            break
        i = int(rng.integers(0, len(path) - 2))
        j = int(rng.integers(i + 2, len(path)))
        if _edge_free(path[i], path[j], coll, step_rad):
            path = path[: i + 1] + path[j:]
    return path


def plan_joint_path(
    start: Sequence[float],
    goal: Sequence[float],
    limits: Sequence[tuple[float, float]],
    collision_fn: CollisionFn,
    *,
    step_rad: float = STEP_RAD,
    extend_rad: float = _EXTEND_RAD,
    max_samples: int = _MAX_SAMPLES,
    seeds: Sequence[int] = _SEEDS,
    shortcut_iters: int = _SHORTCUT_ITERS,
    max_checks: int = _MAX_CHECKS,
) -> PlanResult:
    """start→goal 충돌 없는 관절 경로 — 직선 fast-path → RRT-Connect → shortcut.

    반환 path = [start, ..., goal] (양 끝 포함). 실패 = path None + reason
    (시작/목표 침투 · 탐색 실패 · 예산 소진 — 전부 부정 데이터, 호출자가 home
    경유 폴백하는 계약이라 예외가 아니다)."""
    s = [float(v) for v in start]
    g = [float(v) for v in goal]
    if not (len(s) == len(g) == len(limits)):
        return PlanResult(None, f"dof 불일치 (start {len(s)} / goal {len(g)} / "
                                f"limits {len(limits)})", False, 0)
    coll = _Budget(collision_fn, max_checks)
    try:
        if coll(s):
            return PlanResult(
                None, "시작 자세가 충돌 모델과 침투", False, coll.checks
            )
        if coll(g):
            return PlanResult(
                None, "목표 자세가 충돌 모델과 침투", False, coll.checks
            )
        if _edge_free(s, g, coll, step_rad):
            return PlanResult([s, g], "", True, coll.checks)
        for seed in seeds:
            rng = np.random.default_rng(seed)
            path = _rrt_connect(
                s, g, limits, coll, rng, step_rad, extend_rad, max_samples
            )
            if path is not None:
                path = _shortcut(path, coll, rng, step_rad, shortcut_iters)
                return PlanResult(path, "", False, coll.checks)
        return PlanResult(
            None,
            f"경로 탐색 실패 (seed {len(list(seeds))}회 × {max_samples} 샘플)",
            False, coll.checks,
        )
    except _BudgetExhausted:
        return PlanResult(
            None, f"충돌검사 예산 소진 ({max_checks}회)", False, coll.checks
        )
