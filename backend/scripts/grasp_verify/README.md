# grasp_verify — 파지 진단 스크립트 (실 debug 데이터)

파지 실패를 **어제 저장한 실 데이터**(`backend/debug/detect/{세션}/`)로 정량 분석하는
스크립트. 하드웨어 0 — 실 base-frame 점군 + 실 캘 kinematics + **프로덕션 모듈 코드**로
재현한다. 근거·결론은 [docs/grasping.md](../../../docs/grasping.md) §2(근본원인)·§6.

> 이전의 sim 검증군(shape_generality / verify_* 등 11개)은 2026-07-15 제거됐다 —
> 기하는 검증했으나 실 병목(D405 저텍스처 depth 편향)은 모델 불가라 다 초록인데 실물은
> 전패했다(false confidence). 결론은 grasping.md §1.3 에 박제, 원문은 git history 로 복원 가능.

실행 (backend/ 에서):

```powershell
$env:PYTHONPATH="."; $env:PYTHONIOENCODING="utf-8"
uv run --no-sync python scripts/grasp_verify/code_vs_data.py [세션ID]
```

| 스크립트 | 역할 |
| --- | --- |
| `analyze_ply.py` | 세션별 PLY z 분포/extent/centroid — blue box vs 큐브 대조(누가 뜨나) |
| `reproduce_grasp.py` | 실 PLY 에 프로덕션 `object_metrics`+`antipodal` 을 돌려 로봇이 계산한 파지점 재현 |
| `code_vs_data.py` | **결정적 테스트 A/B/C** — 편향 실데이터 / 편향제거 / 합성 클린 큐브에 동일 코드를 돌려 "코드 로직 vs 입력 depth" 를 가름 (grasping.md §2.1) |

인자 없으면 세션 `20260714_233959`(어제 마지막 실패 run) 기본. 다른 세션은 첫 인자로 지정.
