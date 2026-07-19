"""drivers/prompts.py 순수 함수 잠금 — GDINO 합동 추론의 유일한 신규 판단
(phrase→prompt 귀속)을 torch 없이 검증 (2026-07-19 멀티 프롬프트 일반화).

의미 (뒤집으면 회귀): 오귀속 = task 가 box 를 cube 로 집으러 간다 / 모호 label
을 아무 prompt 에 배정 = 같은 사고의 침묵판 / 전체 top-k = 한 prompt 독식 시
다른 prompt 후보 전멸.
"""

from __future__ import annotations

from modules.detector.drivers import prompts as p

_BOX = (0.0, 0.0, 1.0, 1.0)


def test_build_joint_text_period_convention():
    # GDINO phrase 규약 — strip + 끝마침표 정리 후 ". " 연결 + 종결 마침표.
    assert p.build_joint_text(["white cube", "box."]) == "white cube. box."
    assert p.build_joint_text([" red ball "]) == "red ball."
    assert p.build_joint_text(["a", "", "b"]) == "a. b."


def test_match_prompt_exact_and_case():
    ps = ["white cube", "blue box"]
    assert p.match_prompt("white cube", ps) == "white cube"
    assert p.match_prompt("White Cube", ps) == "white cube"  # 정규화 일치


def test_match_prompt_partial_span():
    # GDINO label 은 요청 phrase 의 부분 스팬일 수 있다 ("white cube" → "cube")
    ps = ["white cube", "blue box"]
    assert p.match_prompt("cube", ps) == "white cube"
    assert p.match_prompt("box", ps) == "blue box"


def test_match_prompt_ambiguous_or_unknown_is_none():
    ps = ["white cube", "white box"]
    # "white" 는 두 prompt 에 다 포함 — 모호 = None (오귀속 방지)
    assert p.match_prompt("white", ps) is None
    # 무관 label / 빈 label = None
    assert p.match_prompt("banana", ps) is None
    assert p.match_prompt("  ", ps) is None


def test_match_prompt_token_overlap_tiebreak():
    # 포함이 아니라 토큰 겹침으로만 풀리는 경우 (합성 스팬) — 겹침 최대가 유일
    # 하면 그쪽, 동률이면 None.
    ps = ["small white cube", "large blue box"]
    assert p.match_prompt("white cube thing", ps) == "small white cube"
    assert p.match_prompt("cube box", ps) is None  # 1:1 동률 — 모호


def test_top_k_per_prompt_no_starvation():
    triples = [
        (_BOX, 0.9, "cube"), (_BOX, 0.8, "cube"), (_BOX, 0.7, "cube"),
        (_BOX, 0.3, "box"),
    ]
    out = p.top_k_per_prompt(triples, top_k=2)
    # cube 는 2개로 잘리고 box 는 낮은 score 여도 생존 (per-prompt 의미)
    assert [(s, q) for _, s, q in out] == [
        (0.9, "cube"), (0.8, "cube"), (0.3, "box")
    ]
