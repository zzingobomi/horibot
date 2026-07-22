"""planner.plan_joint_path 순수 test — fast loop (PyBullet 없음, 합성 충돌장).

의미 있는 검증 (통과용 X): 이 플래너가 실행 경로의 안전을 직접 책임진다
(home 허브 강등 후 transit 이 이 경로를 그대로 MoveJ 한다). 뒤집으면 회귀:
직선 자유인데 RRT 를 도는 낭비 / 벽 반대편 목표를 "직선 막힘 = 실패" 로
포기 / 반환 경로에 충돌 엣지가 섞임 / 시작·목표 침투가 침묵 통과 / 같은
입력이 호출마다 다른 경로 (복권 — 2026-07-09 IK 사고 클래스) / 리밋 밖 샘플.
"""

from __future__ import annotations

import math

from modules.motion.planner import STEP_RAD, _edge_free, plan_joint_path

# 2-DOF 합성 세계: q0 ∈ [0,1] 구간 (0.4,0.6) 에 벽 — 단 q1 > 0.5 로 들면 통과
# ("문턱을 넘어 우회"가 유일한 해 — RRT 가 실제로 탐색해야 풀림).
_LIMITS = [(0.0, 1.0), (0.0, 1.0)]


def _wall(q: list[float]) -> bool:
    return 0.4 < q[0] < 0.6 and q[1] <= 0.5


def _free(q: list[float]) -> bool:
    return False


def _assert_path_valid(path, start, goal, collision_fn):
    """경로 계약 전체 검증 — 끝점 일치 + 전 엣지 무충돌 (미세 재검사) + 리밋 안."""
    assert path[0] == list(start) and path[-1] == list(goal)
    for a, b in zip(path, path[1:]):
        # 반환 경로를 더 촘촘한 해상도로 독립 재검사 — 플래너 내부 검사에
        # 기대지 않는다 (검사기 버그가 자기 경로를 통과시키는 순환 차단).
        assert _edge_free(a, b, collision_fn, STEP_RAD / 4), f"충돌 엣지 {a}→{b}"
    for q in path:
        for v, (lo, hi) in zip(q, _LIMITS):
            assert lo - 1e-9 <= v <= hi + 1e-9


def test_direct_free_line_returns_two_points():
    """직선 자유 = fast-path — RRT 탐색 없이 [start, goal] + direct 표시."""
    r = plan_joint_path([0.1, 0.1], [0.9, 0.1], _LIMITS, _free)
    assert r.path == [[0.1, 0.1], [0.9, 0.1]]
    assert r.direct
    # fast-path 비용 = 직선 샘플 수준 (수십 회) — RRT 폭주가 아니어야 함
    assert r.checks < 50


def test_wall_detour_found_and_valid():
    """직선이 벽에 막힌 목표 — 우회 경로를 찾고 전 구간이 실제로 무충돌."""
    start, goal = [0.1, 0.1], [0.9, 0.1]
    r = plan_joint_path(start, goal, _LIMITS, _wall)
    assert r.path is not None, r.reason
    assert not r.direct
    _assert_path_valid(r.path, start, goal, _wall)


def test_deterministic_same_input_same_path():
    """복권 금지 — 같은 입력 두 번 = 같은 경로 (고정 seed 시퀀스)."""
    a = plan_joint_path([0.1, 0.1], [0.9, 0.1], _LIMITS, _wall)
    b = plan_joint_path([0.1, 0.1], [0.9, 0.1], _LIMITS, _wall)
    assert a.path == b.path


def test_shortcut_keeps_path_small():
    """shortcut 후 경로가 갈지자 원본이 아니라 소수 waypoint 로 정리."""
    r = plan_joint_path([0.1, 0.1], [0.9, 0.1], _LIMITS, _wall)
    assert r.path is not None and len(r.path) <= 12


def test_goal_in_collision_is_explicit_negative():
    r = plan_joint_path([0.1, 0.1], [0.5, 0.2], _LIMITS, _wall)
    assert r.path is None and "목표" in r.reason


def test_start_in_collision_is_explicit_negative():
    r = plan_joint_path([0.5, 0.2], [0.1, 0.1], _LIMITS, _wall)
    assert r.path is None and "시작" in r.reason


def test_fully_blocked_reports_search_failure():
    """우회 자체가 불가능한 완전 벽 — 명시 실패 (무한 루프/예외 아님)."""

    def full_wall(q: list[float]) -> bool:
        return 0.4 < q[0] < 0.6  # q1 어디로도 못 넘는 벽

    r = plan_joint_path(
        [0.1, 0.1], [0.9, 0.1], _LIMITS, full_wall,
        max_samples=80, seeds=(0,),
    )
    assert r.path is None and r.reason


def test_check_budget_exhaustion_is_explicit():
    """충돌검사 예산 소진 = 명시 실패 (Pi 폭주 방지 계약)."""

    def full_wall(q: list[float]) -> bool:
        return 0.4 < q[0] < 0.6

    r = plan_joint_path(
        [0.1, 0.1], [0.9, 0.1], _LIMITS, full_wall, max_checks=100,
    )
    assert r.path is None and "예산" in r.reason and r.checks <= 100


def test_dof_mismatch_is_explicit():
    r = plan_joint_path([0.1], [0.9, 0.1], _LIMITS, _free)
    assert r.path is None and "dof" in r.reason


def test_same_start_goal_trivial():
    r = plan_joint_path([0.3, 0.3], [0.3, 0.3], _LIMITS, _free)
    assert r.path is not None and r.direct


def test_edge_free_resolution():
    """엣지 검사 해상도 — step 보다 얇은 벽도 잡는다 (검사 간격 계약)."""
    thin_lo, thin_hi = 0.50, 0.50 + math.degrees(0)  # 자리표시
    del thin_lo, thin_hi

    def thin_wall(q: list[float]) -> bool:
        return 0.499 < q[0] < 0.5001 + STEP_RAD / 2  # step/2 두께

    assert not _edge_free([0.0, 0.0], [1.0, 0.0], thin_wall, STEP_RAD / 4)
