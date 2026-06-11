# Hand-Eye 캘 Trauma 영구 해결 plan — IRLS + Huber + Posterior σ

다음 세션 진입 anchor. 사용자 trauma 사이클의 *수학적* root cause 진단 + 영구 fix paradigm + 1주 sprint plan.

## 1. 배경 — 사용자 trauma 사이클

```
narrow_sigma_good 경고 → 자세 추가 → outlier 도입 → σ 악화
→ 더 자세 추가 → 더 outlier → 반복 (수 개월째)
```

**구체 사례 (2026-06-11 세션)**:

| 단계 | σ_rot | σ_t | n장 | verdict |
|---|---|---|---|---|
| 어제 commit (2026-06-10) | 0.367° | 2.94mm | 8 | narrow_sigma_good |
| 오늘 RESET 후 새 8장 | 0.511° | 3.7mm | 8 | narrow_sigma_good |
| 추가 4장 (사용자 손 변주) | 0.708° | 3.7mm | 12 | narrow_sigma_good |
| #8/9/10 제거 시뮬 | 0.525° | 3.74mm | 9 | narrow_sigma_good |
| 어제 백업 복원 + 재계산 | 0.367° | 2.94mm | 8 | narrow_sigma_good |

자세 추가 = σ 악화 패턴 박제. narrow_sigma_good 메시지가 사용자를 *추가 캡처* 로 trigger.

## 2. 사용자 hardware 제약 (영구 fix 조건)

- ChArUco 보드 **1장** (수직 거치, 보드 ↔ omx_f base 23cm)
- omx_f (OpenMANIPULATOR-X 커스텀, 5DOF, reach ~25cm)
- D405 카메라 eye-in-hand
- 추가 hardware 주문 / 인쇄 *불가* (AMS 고장으로 multi-color 3D 프린팅 불가)
- SO-101 도착 시 D405 양도 + omx_f 는 USB 카메라 → 캘 또 해야

**fix 는 알고리즘만 변경**. 캘판 1장 + 거치대 + ChArUco 인프라 그대로 살림. setup-independent (SO-101 환경에서 그대로 작동).

## 3. 진짜 root cause (수학적)

기존 BA: unweighted L2

```
X* = argmin_X  Σ_i  r_i(X)²
```

한 outlier 자세 i 의 `r_i²` 가 dominant → `X*` 가 그 자세 쪽으로 끌려가 다른 자세들의 σ 까지 오염. **outlier influence 가 bound 안 됨** = 자세 추가가 정보 추가 ≠ outlier 도입 가능성 추가 = σ 폭주 위험.

→ trauma 사이클이 알고리즘 외부 (사용자 행동) 에서 발생하는 게 아니라 **알고리즘 내부의 수학적 결함**.

## 4. Fix paradigm — IRLS + Huber + Posterior σ

리서치 결과 ([Hydra arxiv:2504.20584](https://arxiv.org/abs/2504.20584), [Nguyen & Pham arxiv:1706.03498](https://arxiv.org/abs/1706.03498)) 기반.

### 4.1 IRLS + Huber loss

```
X* = argmin_X  Σ_i  w_i(r_i) · r_i²
w_κ(r) = min(1, κ / |r|),  κ = 1.345 · σ̂
σ̂ = MAD(residuals) / 0.6745  (robust scale estimate)
```

iter:
- iter 0: w_i = 1 (현재 동작)
- 각 iter: σ̂ 측정 → κ 계산 → w_i 재계산 → BA re-solve
- 수렴 (~5 iter, |X_t − X_{t-1}| < ε)

**수학적 보장**: max influence ≤ κ. outlier 의 weight 가 자동으로 1/|r_i| 로 떨어짐. **새 자세 추가가 σ_X 를 늘릴 상한 존재**.

→ outlier 도입 trauma 사이클이 알고리즘 내부에서 *닫힘*.

### 4.2 Closed-form posterior σ_X

scipy `least_squares` 의 `result.jac` (Jacobian) 사용:

```
Σ_X = (Jᵀ · Σ_obs⁻¹ · J)⁻¹     # parameter covariance
σ_X = diag(Σ_X)^0.5             # per-parameter σ
```

**Cramer-Rao lower bound** — 현재 자세 셋의 정보량 한계. 새 자세 r 의 *기대 information gain*:

```
δI_r = J_rᵀ · Σ_r⁻¹ · J_r
```

`δI_r → 0` 면 새 자세가 기존과 redundant → 추가해도 σ_X 안 줄어듦. UI 가 즉시 "추가 의미 없음" 표시.

### 4.3 LOOCV (Leave-One-Out Cross-Validation)

self-consistency σ 의 함정 해소:
- K 자세 → K번 BA 재호출 (각각 i번째 자세 빼고)
- `σ_X^LOOCV = std(X_i)` — 자세 1개 빠질 때 X 가 얼마나 흔들리는지

비교:
- σ_X^posterior 와 σ_X^LOOCV 일치 → trustworthy
- σ_X^posterior 작은데 σ_X^LOOCV 큼 → outlier 존재 → 어느 자세인지 visualization

## 5. UI / UX 변화

기존:
- `narrow_sigma_good` 경고 → "자세 추가하세요" 압박
- 사용자가 자세 추가 → outlier 도입 → σ 악화

신규:
- 캘 안정성 패널 (3색):
  - 🟢 σ_X^posterior < 0.5° AND σ_X^LOOCV < 0.5° AND |posterior − LOOCV| < 0.1° → "충분, 추가 X"
  - 🟡 둘 다 큼 → "추가 도움 됨"
  - 🔴 posterior 작은데 LOOCV 큼 → "outlier 의심, 자세별 weight 확인"
- 자세별 weight `w_i` 표시 (`w_i < 0.5` 빨강) → 사용자가 어느 자세 drop 할지 시각 결정
- `narrow_sigma_good` 자체는 *경고* 아니라 *정보* (캡처 추가 trigger 안 됨)

## 6. 1주 sprint plan

| Day | 작업 | 검증 |
|---|---|---|
| 1-2 | IRLS + Huber outer loop 추가 ([bundle_adjust.py](../backend/modules/calibration/bundle_adjust.py) 의 `bundle_adjust_hand_eye_extended` / `_physical_sag` 에) | unit test: 어제 8장 + 가상 outlier 1장 → outlier 의 w_i → 0 + σ_X 변동 없음 |
| 3 | closed-form `σ_X = (Jᵀ J)⁻¹` extract + `CALIB_HANDEYE_SIGMA` payload 확장 (`posterior_sigma_X` 필드 추가) | unit test: 어제 8장 → posterior_sigma_X 값 sanity check |
| 4 | LOOCV 워커 — `compute_with_diagnostics` 끝에 K번 BA 재호출. K=10 ~ 30 이면 <30s | unit test: 어제 8장 LOOCV → σ_X^LOOCV 값 + σ_X^posterior 비교 |
| 5 | frontend 자세별 weight w_i + LOOCV ΔX_i 시각화 패널 — 기존 `HandeyeSigmaState` payload 활용 | manual test: UI 에 weight 표시 + 빨강 자세 식별 가능 |
| 6-7 | 실 hardware 검증 — 의도적 outlier 자세 (보드 부분 가림 / 큰 tilt / 광량 부족) 1-2개 섞어 캡처. IRLS 가 자동 downweight 하는지 + σ_X 변동 없는지 확인 | trauma 사이클 reproduction 시도 (자세 추가 폭격) — σ 악화 패턴 안 나오는지 |

## 7. 다음 세션 첫 액션 (코드 entry point)

`backend/modules/calibration/bundle_adjust.py` 의 세 함수가 IRLS 변환 대상:

```python
bundle_adjust_hand_eye                # standard (9 자유도)
bundle_adjust_hand_eye_extended       # +link_offset (20 자유도)
bundle_adjust_hand_eye_physical_sag   # +sag_k (22 자유도)
```

세 함수 모두 scipy `least_squares` 호출. IRLS outer loop = `least_squares` 를 N번 호출하면서 residual weight 매번 재계산.

`hand_eye.py` 의 `compute_with_diagnostics` 가 진입점:
- 현재 multi-seed BA → outlier 자동 제거 → 깨끗한 set 재BA
- 신규 multi-seed IRLS BA → 자세별 weight return → 사후 σ_X + LOOCV

기존 outlier 자동 제거 + 다양성 가드 → **단순 weight-based 로 통합** (사용자에게 표시만, 자동 제거 X — 사용자 결정).

## 8. 검증 데이터 (이미 보유)

- `robot/instances/omx_f_0/calibration/handeye_poses.npz` — 어제 8장 (σ 0.367°)
- `robot/instances/omx_f_0/calibration/.history/20260610T223814_pre-commit/` — 어제 캘 직전 백업 (4종 offset + intrinsic + hand_eye + handeye_poses)
- 검증 시뮬 코드 패턴 (`backend/` 디렉토리에서):

```python
# uv run python -c "..."  또는 << PYEOF
import sys; sys.path.insert(0, '.')
import numpy as np
from pathlib import Path
from modules.calibration.hand_eye import HandEyeCalibration, Pose
from modules.kinematics.registry import get_default_kinematics
from modules.motor.motor_config import MotorConfig, MotorKind
import yaml

d = np.load(str(Path('../robot/instances/omx_f_0/calibration/handeye_poses.npz')))
# ... handeye_poses load → HandEyeCalibration() → compute_with_diagnostics → 결과 비교
```

위 시뮬 코드 + 가상 outlier 자세 1-2장 추가로 *모든 검증* 가능. 백엔드 띄울 필요 X, 사용자 UI 액션 필요 X.

## 9. reference

- **Hydra (IRLS + Huber)** — [Esposito et al. 2025 arxiv:2504.20584](https://arxiv.org/abs/2504.20584), [`lbr-stack/roboreg`](https://github.com/lbr-stack/roboreg)
- **Closed-form covariance** — [Nguyen & Pham 2017 arxiv:1706.03498](https://arxiv.org/abs/1706.03498)
- **Uncertainty-aware Bayesian** — [Ulrich & Hillemann TRO 2023](https://ieeexplore.ieee.org/document/10310118/)
- **Gauss-Helmert 2025** — [Čolaković-Bencerić TRO 2025](https://ieeexplore.ieee.org/document/10916510/)
- **OpenCV broken methods warning** — [opencv/opencv#24871](https://github.com/opencv/opencv/issues/24871) — 본 stack 의 PARK seed 채택 근거
- **Zivid 산업 표준 trauma 회피** — [residuals doc](https://support.zivid.com/academy/applications/hand-eye/hand-eye-calibration-residuals.html), [troubleshooting](https://support.zivid.com/en/latest/camera/support/unsatisfactory-hand-eye-calibration-results-no-infield.html)

## 10. 검토 완료한 다른 방향 (선택 안 한 이유)

| 방향 | 이유 |
|---|---|
| aprilcube (multi-face cube) | AMS 고장 → multi-color 인쇄 불가 |
| 2-board L자 | 캘판 추가 주문 부담 |
| Kalib (foundation model) | GPU 의존 + paradigm shift 큼 |
| EasyHeC++ (URDF mesh) | OMX_F 커스텀 변형 → URDF mesh 정확도 의존 위험 |
| Continuous video BA | workflow 큰 변경 + Ceres-level solver 필요 |
| NBV (Fisher info) | 자세 추천만 개선 — outlier 도입 trauma 안 풂 |
| Pinpoint / TCP probe | hand-eye 자체는 못 풂 |
| Single-shot deep learning | 정확도 수 cm / 수 도 — 부족 |

IRLS + Huber + LOOCV 만이 **(1) 추가 hardware 0 + (2) trauma 양 축 (outlier influence / stop criterion) 동시 해결 + (3) 1주 sprint 가능 + (4) SO-101 환경에 그대로 작동** 조건 모두 만족.

## 11. 메타

세션 끝 시점 (2026-06-11 ~ 06-12):
- 코드 변경 다 revert (`git checkout HEAD -- backend/`) — 깨끗한 baseline (a9fc583 상태)
- NPZ 파일 = 어제 백업 복원본 (σ 0.367°)
- 다음 세션 진입 시 본 문서 + CLAUDE.md 만 읽으면 즉시 작업 가능
