# Hand-Eye σ Floor 진단 — SO-101 + D405 (2026-06-21)

> SO-101 6DOF + Intel RealSense D405 eye-in-hand 의 σ floor 진단 종합. **현재 best**
> effective σ_R 0.801° / σ_t 7.53mm (DB run_id=2, id=6 active, 25 cap drop 9).
> 사용자 목표 0.5° / 5mm 를 못 뚫는 이유 + 가능한 다음 step.

## 1. 핵심 결론

| 항목 | 결과 |
|---|---|
| Algorithmic floor | **σ_R 0.801° / σ_t 7.53mm** (effective σ) |
| Hardware floor | STS3215 backlash ±0.87° → σ_R 0.5° 목표 **impossible** |
| Algorithm 으로 짜낼 여지 | **없음** (5 axis cross-check + Stage E reject) |
| σ_t 5mm 진입 가능 path | Hardware fix (알루미늄 보드 가장 가성비) |

## 2. σ Dual Metric ([[project-calibration-sigma-dual-metric]])

| Metric | 정의 | 우리 best |
|---|---|---|
| **effective σ** ([`measure_effective_sigma`](../backend/scripts/calibrate_offline.py)) | BA fit 적용 후 모든 capture 의 board_in_base 의 std. *accuracy* (commit 결정 기준) | **σ_R 0.801° / σ_t 7.53mm** |
| **Jacobian σ** ([`run_ba_stage`](../backend/scripts/calibrate_offline.py)) | `(JᵀJ)⁻¹·σ²` 의 handeye block trace. *parameter confidence* (BA solver 의 self-reported uncertainty) | σ_R 5.20° / σ_t 7.33mm |

DB schema 자리 두 metric 분리 컬럼:
- `calibration_results.sigma_rot` / `sigma_t` — Jacobian σ
- `calibration_results.effective_sigma_rot` / `effective_sigma_t` — effective σ

[commit_results](../backend/scripts/calibrate_offline.py) 가 hand_eye row INSERT 시 둘 다 박음.

## 3. 진단 시도 (2026-06-21)

### 3.1 cv2 PARK seed BA (`calibrate_validate_opencv.py` + `calibrate_cv2_seed.py` — 2026-06-21 제거, git history 에서 복원 가능)

cv2 4 method (TSAI/PARK/HORAUD/DANIILIDIS) 가 자기들끼리 매우 tight cluster (|t|=85-87mm, ΔR<1°/Δt<2mm). 우리 BA stage-A 는 |t|=91.58mm, 9°/22mm 벗어남 (44% outlier) — 처음엔 *BA bug 의심* 했음.

근데 stage D + drop 9 적용한 BA 를 cv2 PARK seed vs 우리 TSAI seed × IRLS on/off 4 config 비교:

| config | \|t\| | effective σ_R | effective σ_t | outlier |
|---|---|---|---|---|
| D_park_irls_off | 89.82mm | — | — | 14/25 |
| D_park_irls_on | 89.11mm | 0.801° | 7.53mm | 8/25 |
| D_ours_irls_off | 89.82mm | — | — | 14/25 |
| D_ours_irls_on | 89.11mm | 0.801° | 7.53mm | 8/25 |

→ **seed 무관 같은 basin 수렴**. joint/link/sag 추정치도 100% 일치. **BA globally identifiable, single basin**. stage-A 의 9°/22mm 벗어남은 bug 가 아니라 6 DOF 가 joint/link offset 흡수 못해서 생기는 정상 거리.

### 3.2 Bayesian MCMC NUTS 4 chain (`calibrate_mcmc.py` — 2026-06-21 제거, git history 에서 복원 가능)

NumPyro + jaxlie. cv2 5 method 각각의 결과로 4 chain dispersed init.

- Wall time 33.2s (4 chain, 200 warmup + 500 samples)
- **R̂ = 1.0023 < 1.01 threshold**
- chain mean handeye t (mm) 4 chain 다 같음: `[-66.98, -5.36, -57.72]`
- chain mean handeye R euler (deg) 4 chain 다 같음: `[+65.06, +3.34, +87.75]`
- Posterior σ_R 0.034° / σ_t 0.16mm (한 데이터셋 내 credible width)

→ **UNIMODAL 결정적 확정**. LM wrong basin 가설 reject. cv2_seed + MCMC 둘 다 globally identifiable 확인.

### 3.3 Stage E (depth-augmented) 재시도

cal_v3.json 의 Stage E (full 28 cap, drop 6) train 3.79px, LOOCV 9.74, σ_R 4.36°/σ_t 6.15mm — LOOCV/train 2.57× RED 만 보고 reject 했지만 LOOCV 절대값은 D 와 같음. drop 9 적용 안 했음.

**Stage E + drop 9 + default HUBER 시도**:

| Stage | reproj | LOOCV | Jacobian σ_R / σ_t | **Effective σ_R / σ_t** |
|---|---|---|---|---|
| D (commit) | 4.50px | 8.03 | 5.20° / 7.33mm | **0.801° / 7.53mm** |
| **E (depth)** | **3.04px** | 8.11 | **3.50° / 4.92mm** | **0.828° / 7.62mm** |

Jacobian σ 는 떨어졌으나 **effective σ 사실상 동일** (E 가 약간 더 나쁨). LOOCV/train 2.67× RED 가 정확히 잡은 *parameter overfitting*. depth 가 BA solver confidence 만 ↑, data consistency 개선 X. **Stage E reject 확정**.

### 3.4 5 axis cross-check ([`handeye_sigma_floor_so101.md`](handeye_sigma_floor_so101.md))

| Axis | 결과 |
|---|---|
| 1. Community benchmark | LeRobot SO-101 + STS3215 setup 에 0.5°/5mm 달성 reported 사례 **0건**. STS3215 backlash **±0.87°** (실측) — 우리 σ_R 0.801° 와 같은 차수 → 0.5° **hardware impossible** |
| 2. Observability | `observability_params.py` 가 실제로는 없음 (CLAUDE.md 의 spec 과 다름). 측정 인프라 구축 필요 — 시도 안 함 |
| 3. PnP rms | 25 cap mean 0.176px, max 0.381px — *sub-pixel*. corner detection noise 자리 floor 아님 |
| 4. Stage E rehab | 위 3.3 — reject |
| 5. 보드 거리 / corner pixel | 25 cap mean 보드 z=22.7cm, 25mm square → 71.5 pixel. tilt mean 36° (30-70° 권장 19/25). 모두 정상 |

### 3.5 Kalib neural hand-eye (NO-GO)

[Kalib](https://github.com/robotflow-initiative/Kalib) (arXiv 2408.10562):
- **6 DOF only** (handeye 만). 우리 11+ DOF (handeye + joint + link + sag) 보다 *구조적으로 weaker model*
- SpaTracker continuous video 필수 — 우리 34 sparse cap 안 됨, 새 video 캡처 필요
- Reported "0.5°/3mm" 은 simulation only (RFUniverse, Franka, perfect URDF). real-world metric 은 mask IoU, σ 비교 불가
- Windows 비호환 (Ubuntu + CUDA 11.8 + Python 3.10 + 22GB VRAM)
- Best case σ_t 3mm 달성 확률 **5%**, realistic σ_t 8-15mm (우리 7.53 보다 나쁨) **55%**

## 4. BA Degeneracy — joint_offset vs link_offset Trade-off

현재 fit 의 의심 항목:
- **J3 offset = +6.57° + J3 link [-6.43, -4.69, 0] mm** — 둘 다 크고 OUTLIER_RATE_RED 초과
- J5 offset = -5.28° 도 비슷

수학적으로 *joint_offset(J3) Δθ* 와 *link_offset(J3) 의 R/t 6 DOF* 가 **동일한 EE 위치를 만들 수 있음** (frame chain 의 다른 자리에서 같은 효과). BA 의 prior 강도 (joint 1°, link_r 0.2°, link_t 1mm) 가 어느 쪽으로 흡수할지 결정. 둘 중 진짜 mechanical origin 이 무엇인지 BA 결과만으론 판별 불가.

**중요 — 적용 메커니즘**:

| 산출물 | 적용 자리 |
|---|---|
| `joint_offset` | [`JointCoordinates.motor_to_urdf`](../backend/core/coords/joint_coordinates.py) raw↔rad 변환 양쪽 가산 |
| `link_offset` | [`PybulletKinematics.apply_link_offsets`](../backend/modules/kinematics/adapters/pybullet_kinematics.py) → in-memory URDF patch → tempfile → `loadURDF`. **모든 fk/ik 가 patched URDF 사용** |
| `sag_k` | [`SagCorrectedKinematics`](../backend/modules/kinematics/adapters/sag_corrected.py) Decorator 양방향 |

즉 BA 가 J3 = 6.57° + J3 link [-6, -4]mm 으로 *분산 fit* 한 결과가 둘 다 robot motion 에 적용 중. **수학적으로 동등한 EE 위치 만들지만 어느 쪽이 진짜인지는 BA 가 모름**.

### Caliper 검증 protocol (5분 work)

1. J3 motor 회전축에서 J4 motor 회전축까지 caliper 측정 (직선거리 + orientation)
2. URDF 의 J3→J4 link origin 값과 비교
3. 판정:
   - 측정값 = URDF + ~7mm → **link offset 진짜**, joint_offset 6.57° 는 BA artifact
   - 측정값 = URDF (차이 거의 없음) → **servo zero offset 진짜**, link_offset 은 BA artifact

caliper 결과의 가치:
- *추가 URDF patch X* (이미 BA + LinkCoordinates 가 적용 중)
- *진단 용도* — 어느 mechanism 이 진짜인지 가름
- 다음 step:
  - 진짜 link → 그대로 (BA 가 이미 처리)
  - 진짜 servo zero → J3 motor manual home 다시 + 캘 재실행 (link_offset 가짜 fit 안 들어감 → 깔끔)

## 5. 남은 Hardware Fix 옵션

| 옵션 | 사용자 work | 코드 work | 예상 σ 개선 |
|---|---|---|---|
| **1. 알루미늄/아크릴 ChArUco 보드** ($15) | 새 보드 + 캡처 한 번 | 0 | **σ_t 1-3mm ↓** (가성비 ★★★) |
| 2. STS3215 backlash CW/CCW characterization | 새 캡처 50-120장 + 새 protocol | 1-2주 (directional joint_offset BA 추가) | σ_R 0.3° ↓ |
| 3. 3D-print link caliper 측정 + URDF default 수정 | 측정 1시간 | 1-2시간 (URDF patch) | σ minor |
| 4. 목표 조정 | — | — | 현 σ 가 SO-101+D405 tier 정상 영역 (LRBO2/PLOS 논문 0.16°-1°) — 받아들이기 |

## 6. 시도된 path summary (anti-pattern)

- 옵션 cycling 의 위험성 — 사용자가 1 일에 6번 "다시" 외치며 4 agent + 5 axis + Stage E + MCMC 다 던지고 결국 *hardware floor* 확정. 다음 캘 trauma 발생 시 본 문서 anchor 로 시작하면 같은 옵션 또 검토 안 해도 됨.
- Algorithmic optimum 확정에는 BA single-basin 증명 (cv2 multi-seed + MCMC 4 chain R̂) 이 충분 — 추가 algorithm 시도 전에 본 진단 먼저 돌릴 것.

## 관련 문서

- [hand_eye_extended_ba.md](hand_eye_extended_ba.md) — 확장 BA + 물리 sag (OMX 시대 진단, σ floor 1.5°→0.65°/7.94mm)
- [handeye_robust_irls_plan.md](handeye_robust_irls_plan.md) — IRLS+Huber plan
- [handeye_ux_solver_v3_plan.md](handeye_ux_solver_v3_plan.md) — Hand-Eye UX + Solver v3
- [calibration_workflow.md](calibration_workflow.md) — 캡처 절차
- [calibration_apply_flow.md](calibration_apply_flow.md) — 4종 산출물 적용 메커니즘
