# grasp_verify — object-centric 파지 재설계 sim 검증 스크립트

[docs/grasp_redesign_journey.md](../../../docs/grasp_redesign_journey.md) **§10** 의
검증 근거. 하드웨어 0 — PyBullet 카메라로 물리적으로 정직한 부분 점군(가려진 면=점 없음)
을 렌더하고, 실 캘 kinematics(`horibot.db` active bundle + `build_calibrated_kinematics`)
로 도달성·FK 를 재현해, "일반 형상을 멀티뷰로 관측→표면 antipodal→reachable-orientation
파지" 가 viable/robust 한지 **깨지는 케이스(구/원뿔/L자 concave) + 노이즈 + 마스크 bleed +
outlier + clutter** 로 adversarial 검증한다.

## 실행

```powershell
cd backend
uv run --no-sync python scripts/grasp_verify/<name>.py
# 한글 출력 깨지면: $env:PYTHONIOENCODING='utf-8' 먼저
```

경로는 `Path(__file__).resolve().parents[3]` 로 repo 루트를 잡으므로 머신 무관(집/회사 동일).
`horibot.db` 의 `so101_6dof_0` active 캘 번들을 읽는다(git tracked).

## 스크립트 ↔ §10.3 검증 항목

| 스크립트 | §10.3 | 무엇을 보이나 (핵심 결과) |
| --- | --- | --- |
| `shape_generality.py` | A | 단일뷰 충분성은 prismatic 한정 (box/원기둥 OK, 구 조폭 −5mm 헛집음) |
| `verify_antipodal_multiview.py` | B | 표면 antipodal: 단일뷰 0쌍(box 포함), 3뷰 154~405쌍 → 멀티뷰 필수 |
| `verify_reachable_multiview.py` | C | 실 IK 로 닿는 뷰만 융합해도 방위 180~300° 커버 → antipodal 생존 |
| `verify_grasp_execution.py` | D | 접촉쌍→단일조 TCP→tilt 스윕 IK+바닥충돌: 3위치×3형상 실행가능 |
| `comprehensive_verify.py` | D | workspace 12위치×4형상(concave 포함) + 그리퍼-물체 충돌 = 48/48 |
| `verify_grasp_exec_noise.py` | E | σ1mm+bleed10%+outlier2% 에도 실행 5/5 |
| `outlier_phantom.py` | F | 실물 phantom(base_z −0.23m) 재현 = 2-percentile bottom 이 범인, z-gap 군집이 수정 |
| `mask_bleed.py` | F | 인접 책상 bleed 는 (물체가 책상 위라) 견고 — phantom 은 아래-outlier 가 원인임을 대조 |
| `verify_motion_stopping.py` | G,H | 정지: 2~4뷰면 파지 성립 / 이동: naive 관절보간 뷰 사이 floor/obj 충돌 잦음 |
| `verify_clutter.py` | I | 이웃 3.5cm 파지O / 빽빽 2.8cm 3면 파지X(충돌게이트 fail-safe) |
| `verify_production_pipeline.py` | §10.4 구현 | **프로덕션 모듈 코드** end-to-end (z-gap→adaptive 뷰→antipodal→plan_grasp→게이트): 3위치×4형상×클린/노이즈 24/24, 전 케이스 2뷰 정지. antipodal 폭 하한 8mm 근거(4mm=노이즈 가짜 쌍)가 여기서 나옴 |

## 주의

- 검증/프로토타입 코드다 — 프로덕션 아님. 확정 설계·현재 코드 keep/replace/add 는
  journey.md §10.4/§10.5.
- antipodal 선택기·접촉쌍→TCP 변환은 여기 프로토타입이 있으니 구현 시 출발점으로.
- 하드웨어에서만 판명될 것(파지 물리 안정성/실 depth 재질 실패/실 마스크 품질)은 §10.6.
