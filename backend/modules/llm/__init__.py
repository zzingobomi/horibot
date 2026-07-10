"""LLM domain — 자연어 pick-and-place 명령 파서 (§17 NL PnP).

한국어/영어 명령 → (pick_object, place_object) 영어 구조화. GroundingDINO 가 영어
prompt 만 잘 먹으므로 LLM 이 번역 + 의도 추출. 구현체(Qwen)는 adapter 뒤 (§17.1).
detector 모듈과 동형 (host-level robot-agnostic ML 모듈, 공유 transformers load-lock).
"""
