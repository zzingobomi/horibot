"""Detector 모듈 — `Detect Object` (Day-1 primitive).

물체 검출 → base frame 3D 좌표. 인터페이스(계약)는 Day-1, 구현체(Grounding DINO 등)는
adapter 뒤 (backend.md §17.1 "인터페이스 ≠ 구현"). base-투영 수학은
projection.py (결정적, 모델/하드웨어 무관 — 회사 단위테스트 가능).
"""
