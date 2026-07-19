"""멀티 프롬프트 순수 헬퍼 — 합동 쿼리 텍스트 조립 + phrase→prompt 귀속.

torch-free 인 이유: gdino.py 는 module-top 에서 torch/transformers 를 끌고 와
fast 테스트가 못 건드린다 — 귀속 로직(합동 추론의 유일한 신규 판단)은 여기
분리해 단위테스트로 잠근다 (2026-07-19 멀티 프롬프트 일반화).

GDINO 합동 쿼리 규약: 마침표로 phrase 를 잇는다 ("white cube. box."). 후처리가
box 마다 매칭 phrase(label)를 돌려주는데, label 은 요청 phrase 의 **부분 스팬**
일 수 있다 ("white cube" 요청에 "cube" label) — 귀속은 정규화 후 완전일치 →
포함 → 토큰 겹침 순으로 푼다. 못 풀면 None (버림) — 엉뚱한 prompt 로 찍는
것보다 후보 하나 잃는 게 낫다 (오귀속 = task 가 box 를 cube 로 집으러 간다).
"""

from __future__ import annotations

from collections.abc import Sequence

from .protocol import Bbox


def build_joint_text(prompts: Sequence[str]) -> str:
    """합동 쿼리 텍스트 — GDINO phrase 구분 규약(마침표) 로 조립.

    단독 경로(gdino.detect_boxes)와 같은 정규화(strip + 끝 마침표 제거)만 —
    lowercase 강제 등 추가 변형은 단독/합동 score 비교 가능성을 깨므로 안 한다."""
    return ". ".join(p.strip().rstrip(".") for p in prompts if p.strip()) + "."


def match_prompt(label: str, prompts: Sequence[str]) -> str | None:
    """GDINO 매칭 phrase(label) → 요청 prompt 귀속. 실패 = None (버림).

    순서: ① 정규화 완전일치 ② 포함 (label⊂prompt 또는 prompt⊂label) 이
    유일할 때 ③ 토큰 겹침 최대 (동률이면 모호 — None). label 이 두 prompt 의
    합성 스팬("cube box" 등)으로 오면 ②/③ 이 모호 판정해 버린다."""
    lab = label.strip().lower()
    if not lab:
        return None
    norm = [(p, p.strip().lower()) for p in prompts if p.strip()]
    for p, n in norm:
        if lab == n:
            return p
    contains = [p for p, n in norm if lab in n or n in lab]
    if len(contains) == 1:
        return contains[0]
    lab_tokens = set(lab.split())
    best: str | None = None
    best_overlap = 0
    tie = False
    for p, n in norm:
        overlap = len(lab_tokens & set(n.split()))
        if overlap > best_overlap:
            best, best_overlap, tie = p, overlap, False
        elif overlap == best_overlap and overlap > 0:
            tie = True
    if best_overlap == 0 or tie:
        return None
    return best


def top_k_per_prompt(
    triples: Sequence[tuple[Bbox, float, str]], top_k: int
) -> list[tuple[Bbox, float, str]]:
    """**프롬프트별** score 내림차순 Top-K (contract top_k 의미) — 전체 top-k
    로 자르면 한 prompt 가 상위를 독식할 때 다른 prompt 후보가 전멸한다."""
    by_prompt: dict[str, list[tuple[Bbox, float, str]]] = {}
    for t in triples:
        by_prompt.setdefault(t[2], []).append(t)
    out: list[tuple[Bbox, float, str]] = []
    for items in by_prompt.values():
        items.sort(key=lambda t: t[1], reverse=True)
        out.extend(items[: max(1, top_k)])
    out.sort(key=lambda t: t[1], reverse=True)
    return out
