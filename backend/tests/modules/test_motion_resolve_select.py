"""resolve_reachable 채택 정책(_pick_by_residual) 순수 test — fast loop.

의미 있는 검증 (통과용 X): §2.1 헛집음의 직접 원인이 "잔차 큰(겨우 닿는) 자세를
선호 순서만 보고 채택"한 것이었다. 이 정책이 그 order-inversion 을 잡는지 —
성공/실패 실측값(0.16mm / 7.35mm)을 그대로 심어 결정적으로 재현한다. 구멍 하나 =
테스트 하나: (a) 앞선 mediocre 건너뛰고 뒤의 good 채택, (b) good 없으면 잔차 최소,
(c) 둘 다 good 이면 선호 순서 유지(early-exit 동치), (d) 빈 입력 = None.
"""

from __future__ import annotations

from modules.motion.module import _RESOLVE_RESIDUAL_GOOD_MM, _pick_by_residual

# 해 내용(관절값)은 채택 로직과 무관 — 어느 그룹의 해가 반환되는지만 검증하므로
# 구분 가능한 sentinel 로 충분.
_S0: list[list[float]] = [[0.0]]
_S1: list[list[float]] = [[1.0]]
_S2: list[list[float]] = [[2.0]]


def test_prefers_good_over_earlier_mediocre():
    """§2.1 재발 방지 — 선호 순서상 앞선 '겨우 닿는'(잔차 > GOOD) 그룹을 건너뛰고
    뒤의 '깨끗이 닿는'(잔차 ≤ GOOD) 그룹을 채택. 실측: 헛집음 7.35mm / 성공 0.16mm."""
    mediocre, good = 7.35, 0.16
    assert good <= _RESOLVE_RESIDUAL_GOOD_MM < mediocre  # 전제 고정(상수 바뀌면 알림)
    picked = _pick_by_residual([(0, mediocre, _S0), (1, good, _S1)])
    assert picked is not None
    gi, resid, sols = picked
    assert gi == 1 and sols is _S1 and resid == good  # 앞선 0(mediocre) 아님


def test_best_of_mediocre_when_none_good():
    """GOOD 이 하나도 없으면(겨우 닿는 것만) 잔차 최소를 채택 — 침묵 실패가 아니라
    최선을 잡되 선호 순서보다 잔차 우선."""
    picked = _pick_by_residual([(0, 8.0, _S0), (1, 5.0, _S1), (2, 9.0, _S2)])
    assert picked is not None
    gi, resid, sols = picked
    assert gi == 1 and sols is _S1 and resid == 5.0  # 5.0 이 최소


def test_preserves_preference_among_good():
    """둘 다 GOOD 이면 선호 순서(먼저 온 것) 유지 — grasp-safe 한 것들 사이에선
    호출자 선호(예: tilt 사다리)를 잔차로 뒤집지 않는다. _scan early-exit 과 동치."""
    picked = _pick_by_residual([(0, 0.5, _S0), (1, 0.1, _S1)])
    assert picked is not None
    gi, _resid, sols = picked
    assert gi == 0 and sols is _S0  # 0.1 < 0.5 이지만 둘 다 GOOD → 선호 첫 그룹


def test_empty_is_none():
    """통과 그룹 0 = None (호출부 _scan 이 전멸 사유 메시지로 분기)."""
    assert _pick_by_residual([]) is None


def test_single_good():
    picked = _pick_by_residual([(3, 0.4, _S0)])
    assert picked is not None and picked[0] == 3 and picked[2] is _S0


def test_single_mediocre_still_picked():
    """겨우 닿는 것 하나뿐이어도 채택(기각 아님) — 하드리밋(10mm)은 kin 몫."""
    picked = _pick_by_residual([(5, 9.9, _S2)])
    assert picked is not None and picked[0] == 5 and picked[2] is _S2
