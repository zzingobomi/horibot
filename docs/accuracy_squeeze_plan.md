# Accuracy Squeeze Plan

> DIY 3D프린트 + XL430/XL330 + 5DOF 라는 하드웨어 한계 안에서 **TCP 절대 정확도를 짜낼 수 있는 만큼 짜내는 것**이 목표.
>
> 현재 캘 4종 + 물리 sag 모델로 도달한 floor: **σ_rot 0.647° / σ_t 7.77mm**
> ([hand_eye_extended_ba.md](hand_eye_extended_ba.md)). 이 위에서 더 짜낸다.
>
> **2026-05-28 이력**: 시스템 commit 버그 발견 + fix (§1.6 참조). link/sag commit 이
> 과거 cumulative 가산이라 매번 누적 손상되던 것을 overwrite 로 영구 fix.
> disk σ 회복됨 (19.5mm → 7.77mm).

---

## 1. 즉시 과제 — 큐브 grasp 갭 진단

### 1.1 증상

pick_and_place 가 20mm 큐브를 "옆면 중간" 이 아닌 **상단** 에서 집음.

### 1.2 코드 의도는 맞음

```
detect       → position = (x, y, top_z),  _meta = {base_z, height}
GraspPolicy  → grasp_z = base_z + height * 0.5
```

[step_executor.py:246-254](../backend/modules/task/step_executor.py#L246-L254)
([_grounded_detect](../backend/modules/task/step_executor.py#L240)),
[step_executor.py:369](../backend/modules/task/step_executor.py#L369)
([_grasp_policy](../backend/modules/task/step_executor.py#L352)).

20mm 큐브면 `0 + 20 * 0.5 = 10mm` 가 옆면 가운데. 코드는 맞게 돼있음.

그런데 실제 결과가 위쪽 → **셋 중 하나가 거짓말 중**.

### 1.3 의심 셋

| | 거짓말 위치 | 결과 |
|--|--|--|
| **A** | 측정 height 가 실제(20mm)보다 작게 잡힘 | grasp_z 가 floor 쪽으로 끌려 내려가서 상대적으로 윗부분 접촉으로 보임 |
| **B** | base_z(floor) 가 실제보다 높게 잡힘 (책상 위로 떠 있음) | grasp_z 가 큐브 상단 근처로 올라감 |
| **C** | URDF 의 `tcp` link 가 실제 그리퍼 끝점 (핑거 닫혔을 때 만나는 점) 이 아니라 더 위 (손목 근처) | 명령은 옆면 가운데지만 실제 그리퍼 끝점은 위쪽 |

A, B 는 perception 문제. C 는 모든 task 공통의 **TCP 프레임 정의** 문제 — 즉 grasp 만이 아니라 미래 모든 task 가 같은 오프셋만큼 어긋남.

### 1.4 진단 — 코드 변경 0, 5분

큐브 하나 + 자 + pick_and_place 1회 시도.

기존 로그가 그대로 사용 가능:

1. **Detect 로그** ([step_executor.py:240-244](../backend/modules/task/step_executor.py#L240-L244))
   ```
   GroundedDetect 성공: conf=... base=(x, y, top_z)
   ```
2. **GraspPolicy 로그** ([step_executor.py:371-374](../backend/modules/task/step_executor.py#L371-L374))
   ```
   GraspPolicy base_z=... height=... → grasp_z=...
   ```

자로 잰 실측치와 비교해 표 채움:

| 항목 | 코드 값 | 자 실측치 | 차이 |
|--|--|--|--|
| height | (log) | 0.020 | → A 여부 |
| base_z | (log) | 책상 표면 z | → B 여부 |
| grasp_z | (log) | 그리퍼 끝점이 실제로 닿은 z | → C 여부 |

해석:
- height 가 5\~15mm 로 작게 나옴 → **A** (depth 샘플링이 책상 픽셀로 끌려감)
- base_z 가 큐브 두께만큼 떠있음 → **B** (ring 픽셀이 객체 옆면/그림자 포함)
- 위 둘 다 정상인데 grasp_z 명령과 실제 그리퍼 끝점 위치가 어긋남 → **C** (TCP 프레임 = EE link 위치가 실제 그리퍼 끝점 아님)

> ⚠️ default 트랩: `_meta.base_z`/`_meta.height` 가 detector 응답에 없으면 0.0 으로 떨어짐
> ([step_executor.py:251-253](../backend/modules/task/step_executor.py#L251-L253)). 둘 다 0.0 으로 찍히면 detector 가 안 채우는 거 — 진단 이전에 그 버그부터 잡아야 함.

### 1.5 케이스별 fix 방향

- **A**: depth 샘플링 개선. 현재 bbox 안 percentile 25 → percentile 더 위로 / segmentation mask 로 객체 픽셀만 / bbox erosion 으로 책상 leak 제거.
- **B**: ring pad 공식 조정 (객체 그림자/옆면 미포함). depth gradient outlier 거르기.
- **C**: URDF 의 `tcp_joint` xyz (현재 92mm) 가 실제 그리퍼 끝점 (핑거 닫혔을 때 만나는 점) 과 mm 단위로 안 맞는 것. link_offset 캘 자유도에 `tcp_joint` 추가해서 BA 가 같이 풀게 확장 (§ 3 참조).

### 1.6 시스템 버그 — BA commit cumulative 누적 손상 (이력: 2026-05-28 발견 + fix)

**발견 경위.** 사용자가 painful 한 캘 작업으로 σ_t 7.94mm 도달 한 게 첫 commit
직후 잠깐의 정답이었고, 이후 commit 들이 link/sag 를 cumulative 누적시키며 disk 가
실제 σ_t 19.5mm 까지 악화돼 있었음. 사용자는 σ 모니터링 안 한 채로 진행해서 인지 못 함.

**원인.** semantics 불일치:
- `bundle_adjust_hand_eye_physical_sag` 의 `x0` 가 link_t / sag_k 를 **0 으로 초기화** +
  내부 `fk_chain` 이 original URDF 기준 → BA 출력은 **absolute total** 값
- 그러나 `_srv_handeye_commit` 의 link/sag 가 `commit_offsets` 통해 **cumulative 가산**
  (`existing + delta`) — joint_offset semantics 를 그대로 적용한 잘못된 디자인
- → 매 commit 마다 disk = optimal × N 누적

`joint_offsets` 는 정상 — ja 가 `motor_to_urdf` 통해 이미 disk joint_offset 적용된
상태로 BA 에 들어가서 BA offset 은 진짜 delta. cumulative 가산 정합.

**검증 (2026-05-28).** 41 자세에서 두 가설 비교:
- H1 (TOTAL, link_t=BA): σ_t 7.770mm — BA 자체 보고와 0.0000mm 차이 ✓
- H2 (CUMULATIVE, link_t=disk+BA): σ_t 33.608mm — 25.8mm 차이 ✗
- → BA 출력은 absolute total. commit cumulative 가산은 버그.

**Fix 적용 (2026-05-28).**
- `LinkCoordinates.commit_offsets` / `SagCoordinates.commit_offsets` → **overwrite** 로 변경
  (참조: [link_coordinates.py](../backend/core/coords/link_coordinates.py),
  [sag_coordinates.py](../backend/core/coords/sag_coordinates.py))
- `_srv_handeye_commit` 변수명 / 주석 명확화
  ([calibration_node.py](../backend/nodes/application/calibration_node.py) `_srv_handeye_commit`)
- 모듈 문서 갱신: `link_offsets.py`, `sag_offsets.py`, `hand_eye.py:compute_with_diagnostics`,
  `bundle_adjust.py:bundle_adjust_hand_eye_extended`
- 프론트엔드 라벨 갱신 — "delta" → "절대 보정값" (참조: HandEyeResults.tsx)

`merge_delta` 유틸은 io 모듈에 남아있음 — 분석/실험 용도로만 사용, commit 흐름에서는
호출 안 됨.

**디스크 상태 (2026-05-28).**
- σ_t 19.535mm → 7.770mm, σ_rot 2.079° → 0.647° 회복
- 사용자가 painful 작업으로 도달했던 7.94mm 가 진짜 system floor 임을 재확인

**향후 사용:** UI 의 COMPUTE → COMMIT 흐름 정상 사용 가능. 매 commit 이 disk 를
absolute 정답으로 덮어씀, 누적 손상 없음.

---

## 2. TCP 정확도 = 모든 task 의 공통 인프라

어떤 task 든 결국 **"base 프레임의 특정 지점에 도구 작용점을 정확히 가져다 둔다"** 가 본질. TCP 정확도는 task 별 코드가 아니라 캘리브레이션 layer 가 책임.

OMX_F 의 URDF 는 `tcp` link 를 link5 에서 92mm 떨어진 지점에 박아두고
([omx_f.urdf:13-17](../robot/urdf/omx_f/omx_f.urdf#L13-L17)),
그 위치가 그리퍼 끝점 (핑거 닫혔을 때 만나는 점) 을 노린 점임 — 즉 **URDF 의도 자체가 "TCP 프레임 = 그리퍼 끝점"**.
Hand-eye 는 이미 이 정의 위에서 풀려있으므로 별도 tool offset 산출물 불필요.

```
base
 └─ link 0..5     (joint_offset + link_offset + sag 보정)
     └─ tcp = 그리퍼 끝점 (핑거 닫혔을 때 만나는 점)
              ↑
              link5→tcp 의 92mm 자체도 보정 대상
              (link_offset 자유도 확장으로 BA 가 풀게)
```

이 구조의 장점:
- 캘 산출물 늘리지 않음 — 기존 link_offset BA 의 자유도만 확장
- 모든 task 가 동일 정확도 baseline 위에 얹힘
- "EE 프레임 = 그리퍼 끝점" 의 의미적 일관성 유지

> 도구 갈아끼움(디스펜서, 펜 등) 가능성을 가정하면 산업 관행처럼 hand_eye 분리 + 별도 tool offset 산출물이 유리하지만, OMX_F 는 gripper 영구 부착 + DIY 환경에서 도구 swap 비현실적 → 분리 안 함. 미래에 도구 추가 필요해지면 그때 별도 산출물 도입 검토.

---

## 3. DIY 환경에서 정확도 짜내기 — 전략 프레임워크

### 3.1 출발점: 완전 분해는 불가능

DIY 환경에는 오차원인이 복합적으로 얽혀있음:

- Dynamixel 백래시 + 인코더 양자화 + 자세별 토크 변형 (XL430/XL330 hobby 급)
- 3D 프린트 휨 + 조립 mm 단위 오차
- sag (현재 J2/J3 만 모델링, 잔재 다수)
- 케이블 텐션, 마운트 회전, thermal drift, ...

이걸 산업 환경처럼 **하나씩 분해해서 따로 캘** 하려면 CMM / 레이저 트래커 / 정밀 지그 필요. **DIY 에선 불가**.

→ 전략을 바꿔야 함. 다음 세 가지 인사이트.

### 3.2 인사이트 1 — 카메라가 너의 지그다

D405 가 산업용 측정기 역할을 함. depth 정확도 mm 단위 + RGB 코너 검출 sub-pixel.
산업에서 CMM 가 하는 일을 DIY 에선 **카메라가 함**.

이미 부분적으로 하고 있음 — extended BA + 물리 sag 가 카메라 데이터로 캘 짜낸 것
([hand_eye_extended_ba.md](hand_eye_extended_ba.md)). 이걸 더 깊이 쓰는 게 다음 단계.

### 3.3 인사이트 2 — 물리 모델 + **잔차 학습** (empirical residual) — **실측 후 폐기**

**2026-05-28 update**: 41 자세에서 LOO RBF 시험 결과 효과 없음 (§4 #3 참조).
정상 baseline σ_t 7.77mm 에서 hold-out σ_t 8.33mm 로 오히려 악화, 잔차 ↔
joint angle 상관 모두 \|corr\|<0.3. BA 가 이미 다 짜냈다는 의미.
**이 인사이트는 폐기**. mm 미만 정확도 필요시 §3.4 visual servoing 으로.

(아래는 시도 전 설계 기록 — 참고용)

캘 4종은 **구조적 보정** — 각 자유도가 물리적 의미 가짐 (joint zero, link geom, sag).
**기존 캘 4종은 그대로 frozen, 절대 재캘 안 함** (`joint_offset.npz` / `link_offset.npz` /
`sag.npz` / `hand_eye.npz` 모두 그대로). 그 위에 **5번째 산출물 `residual.npz`** 만 추가.
잔차를 통째로 학습시킴:

```
명령 (q1..q5)
    ↓
[기존 캘 4종 적용 — 그대로]
    - joint_offset 차감
    - link_offset patched URDF
    - sag 보정
    - hand_eye matrix
    ↓
Kinematics.fk(q) = 물리모델 예측 EE 위치
    ↓
+ learned_residual(q)   ← 신규 (residual.npz)
    ↓
= 실제 EE 예측 위치 (더 정확)

학습 phase (오프라인):
  - 자세 200~500개 캡처
  - 잔차 = (카메라가 본 실제) - (Kinematics.fk 예측)
  - smooth regressor (GP / 작은 NN / RBF) 로 q-space 보간
  - residual.npz 로 저장
IK 도 역방향으로 합성
```

**무엇이 흡수되나** — 모델링 안 한 sag 잔재 (J4/J5), 백래시 평균값, 3D 프린트
휨의 자세 의존, 케이블 텐션 패턴, **"분리해서 못 잡는 모든 자세 의존 오차"**.

**데이터 양** — 자세 200\~500개로 충분 (sub-cm residual smooth 가정 시).
자동으로 워크스페이스 격자 돌면서 캡처, BA 와 같은 인프라 재사용.

**왜 RL 이 아닌가** — 이건 **지도 학습** 문제. 정답(카메라 측정)이 모든 샘플에
있음. RL 의 exploration / credit assignment / 시퀀셜 결정 부담 없음. RL 쓰면
sample efficiency 가 100\~1000배 나빠짐 — DIY 데이터 양으로 못 함. 정답이 직접
주어지는데 trial-and-error 하면 안 됨.

#### 잔차 학습의 실패 모드 vs BA 자유도 확장의 실패 모드 — 헷갈리지 말 것

자유도 늘리는 것 = 위험, 이라는 직관은 valid 하지만 두 가지가 다른 메커니즘으로
망함. 같이 묶으면 잘못된 방어책을 씀.

| | BA 자유도 확장 (§4 #2 같은 것) | 잔차 학습 (§4 #3) |
|--|--|--|
| 무엇을 fit | 물리 파라미터 (joint/link/hand-eye) **동시** | 물리 모델은 **frozen**, 잔차만 |
| 파라미터끼리 경쟁? | **예** — link 줄이고 hand-eye 늘리면 같은 EE → gauge freedom | **아니오** — 물리 파라미터 안 건드림. leftover 회귀 |
| 주된 실패 모드 | **비물리값에 정착** ([extended_ba §5](hand_eye_extended_ba.md)). σ는 작지만 generalize 안 함 | **overfit** + **extrapolation** (학습 분포 밖에서 신뢰 X) |
| 방어책 | regularization sweep + hold-out σ + 파라미터 sanity check | smoothing prior (GP 자체가 smooth) + train/test split + repeatability 와 비교 (이 floor 아래로 fit 시키지 않기) |

→ 결론: 잔차 학습이 BA 의 gauge freedom 문제를 **재현하지 않음**. 단 자기 고유의
실패 모드가 있으니 별도 방어 (특히 workspace coverage + repeatability 한계 인지)
필요. 학습 후 σ 가 repeatability floor 근처면 더 fit 시키지 말기 — 그 이상은
noise overfit.

### 3.4 인사이트 3 — Closed-loop 으로 마무리 (필요한 task 에 한해)

§3.2 + §3.3 다 해도 absolute floor 는 **모터 repeatability**. XL430/XL330
hobby 급은 보통 absolute 정확도보다 repeatability 가 나음 — 같은 명령 두 번 →
같은 자리 (편향은 있어도 분산은 작음).

→ **편향(bias)은 (캘 + 잔차)가 잡고, repeatability 가 hard floor.**

그 floor 보다 더 가야 하는 task 는 **open-loop 으로 풀 수 없음**. 어떤 캘도
못 해결. → **visual servoing** 으로 마지막 1cm 만 카메라 보면서 close-loop.
산업 로봇도 이렇게 함.

**pick task 적용 예:**
- 거친 접근 (cm 단위): 캘 + 잔차로 TCP 갖다 댐
- 최종 descent (mm 단위): 카메라가 객체 보면서 EE 위치 보정
- → RL 아님. 고전 PBVS/IBVS 기법으로 충분

### 3.5 RL 의 위치 — 언제 의미 있나

| 풀려는 문제 | 적합한 도구 |
|--|--|
| "캘 4종 위에 잔차 잡기" | **지도 학습** (정답 = 카메라 측정) |
| "잡기 자체 성공률 올리기" (목표 정확도 부족 보완) | **Visual servoing** (closed-loop) |
| "스킬 자체를 학습" (흔들리는 물체 잡기, 새 도구 사용 적응) | **RL 의미 있음** |

RL 이 빛나는 데는 **시퀀셜 결정 + sparse reward + 모델 없음** 인 상황.
OMX_F 정확도 문제는 셋 다 아님 → RL 부적합.

---

## 4. Squeeze 축 — 4종 캘 위에 더 짜낼 곳

현재 잡혀있는 것:

| 산출물 | 어디서 적용 | 상태 |
|--|--|--|
| intrinsic | `cv2.undistortPoints` | ✓ |
| hand_eye | Detector + Frontend PC layer | ✓ |
| joint_offset | raw↔rad 변환 양쪽 | ✓ |
| link_offset | URDF patch (PyBullet 로드) | ✓ |
| sag_offset | Kinematics fk/ik 양방향 | J2/J3 만 |

위에 더 들어갈 후보들. **효과 큰 × 비용 작은** 순:

| # | 후보 | 무엇 | 효과 추정 | 비용 |
|--|--|--|--|--|
| 1 | **repeatability floor 측정** | 같은 명령 N회 → 분산 측정. 모든 squeeze 의 baseline (이 floor 보다 잘 짜내려는 시도는 무의미) | (측정만) | 30분 |
| 2 | **link_offset BA 자유도 확장 → `tcp_joint` xyz 포함** | URDF 의 92mm 가 실제 그리퍼 끝점 (핑거 닫혔을 때 만나는 점) 과 안 맞는 부분을 BA 가 같이 풀게. 별도 산출물 추가 X, 기존 link_offset 캘 흐름 안에서 처리. 진단이 C 로 나오면 1순위 fix. **적용 layer 변경 필요**: 현재 [urdf_patcher.py](../backend/core/coords/urdf_patcher.py) 의 `_default_joint_id_map` 은 joint1\~5 만 알기 때문에 `tcp_joint` 도 처리하도록 확장 필요 + `LinkOffsets` 자료 구조에 fixed joint 표현 추가. **⚠️ 자유도 추가 = gauge freedom 위험 ([extended_ba §5](hand_eye_extended_ba.md))** — hand-eye t 와 swap 가능. regularization 재튜닝 + hold-out + sanity check 필수 | mm 단위 (C 결판 시) | 1\~2시간 (BA + LinkOffsets + patcher map + reg sweep + hold-out) |
| 3 | ~~**empirical residual 학습 layer**~~ — **실측 후 효과 없음 확정** | 2026-05-28 [analyze_residuals.py](../backend/scripts/analyze_residuals.py) 로 LOO RBF 시험. **올바른 baseline σ_t 7.77mm 에서 hold-out σ_t 8.33mm (악화)**. 잔차 ↔ joint angle 상관도 모두 \|corr\|<0.3 — 자세 의존 시그널 없음. BA 가 이미 다 짜냄. **이 후보는 폐기.** mm 미만 정확도 필요시 §3.4 visual servoing 으로. | 효과 없음 (실측 확정) | — |
| 4 | **백래시 방향성 보정** | XL430/XL330 둘 다 backlash 존재. 같은 각도여도 CW 접근 vs CCW 접근에 raw 다름. direction-dependent offset | sub-degree, 끝단 mm 단위 | 측정 1시간 + raw 변환 layer 패치 (#3 의 잔차 학습이 흡수할 수도 — 중복 검증 필요) |
| 5 | **sag 모델 J4/J5 확장** | 현재 J2/J3 만. J4(wrist roll) 자세에 따라 J5 처짐 모멘트 바뀜 | sub-mm \~ mm | 데이터 캡처 + 모델 확장 (#3 잔차 학습이 흡수할 수도) |
| 6 | **D405 depth bias** | per-pixel / per-distance systematic bias. plane fit 으로 추출 → lookup table. detection / ICP / TSDF 모두 영향 | mm 단위 (특히 ICP/grasp) | 측정 1시간 + 보정 layer |
| 7 | **thermal drift** | XL430/XL330 발열 후 zero drift. 워밍업 protocol + cold/warm 캘 비교 | sub-degree | 측정 1시간 |
| 8 | **BA 자세 분포 개선** | 현재 BA 자세가 워크스페이스 cover 충분한지. residual 의 spatial pattern 보면 즉시 진단 | 모서리 자세에서 mm 단위 | 분석 30분 |
| 9 | **static settle 보장** | trajectory 끝 후 settle 시간 부족하면 잔진동 중 측정으로 노이즈. 이미 잡혀있을 수도 | 노이즈 floor | 확인 30분 |
| 10 | **Visual servoing (closed-loop)** (§3.4) | open-loop 한계(repeatability floor) 아래로 가야 하는 task 에만 적용. 마지막 1cm 카메라 보면서 보정. RL 아님 — PBVS/IBVS 고전 기법 | task 의존 (sub-mm 가능) | task 별로 끼움. pick descent 가 첫 적용 후보 |

> #4, #5 는 #3 의 잔차 학습이 결과적으로 흡수할 수 있음. #3 적용 후 잔차 패턴이 "방향 의존" 또는 "J4/J5 자세 의존" 으로 명확히 남으면 그때 별도 모델로 분리. 안 남으면 그냥 #3 안에 두고 진행. **분리 캘은 측정으로 정당화될 때만**.

---

## 5. 권장 진행 순서

> 2026-05-28 갱신. 잔차 학습 (§4 #3) 은 실측으로 효과 없음 확인되어 폐기.
> commit 버그 (§1.6) 는 영구 fix 완료. 정상 system floor σ_t 7.77mm.

1. **§1 큐브 진단** — 어차피 막힌 일. 표 한 줄 채워서 A/B/C 결판.
2. **repeatability 측정** — § 1 진단 돌리는 김에 같은 명령 N회 반복으로 분산도 같이 잡음. **이 값이 모든 후속 squeeze 의 hard floor**.
3. **진단이 C 면** §4 #2 (link_offset BA 자유도 확장). **A/B 면** depth 샘플링 / ring 픽셀 추정 개선. 끝나고 σ 재측정.
4. **구조적 squeeze 후보들** — §4 #4 (백래시), #5 (sag J4/J5), #6 (D405 depth bias), #7 (thermal), #8 (BA 자세 분포). 잔차 학습 폐기됐으니 이 구조적 후보들로 직접 짜내야 함. 측정 후 효과 큰 것부터.
5. **mm 미만 정확도 필요 task** — visual servoing (§3.4, §4 #10) 으로 closed-loop. open-loop 의 floor (~7.77mm) 아래로 내려가는 유일한 길.
6. 이후로는 **데이터 driven** — 매번 새 캘/보정 적용 후 σ 측정. 가장 큰 잔차를 만드는 다음 후보부터.

---

## 6. 작업 원칙

- **측정 없이 다음 캘 만들지 말기.** "할 수 있는 것" 말고 "**현재 가장 큰 잔차를 만드는 것**" 잡기.
- **물리 모델 vs 잔차 학습의 분업**: 물리적 의미 있고 측정으로 따로 분리되는 것만 별도 캘로. 분리 안 되는 잔차는 무리해서 모델링 말고 §3.3 잔차 학습 layer 가 흡수. 새 분리 캘은 "잔차 패턴이 그렇게 생겼다" 는 측정 근거 있을 때만.
- 캘 산출물 늘릴 때는 항상 **무엇을 보정 / 어디서 적용 / COMMIT 후 어디까지 자동 반영** 셋 다 [calibration_apply_flow.md](calibration_apply_flow.md) 표에 추가.
- 새 보정 도입 시 BA 잔차 / σ_rot / σ_t 가 실제로 떨어지는지 검증. 안 떨어지면 그 보정은 의미 없는 거 — 코드에 박지 말고 빼기.
- **자유도 늘리는 보정에는 항상 hold-out + sanity check** (§3.3 박스 참조). σ 떨어졌어도 (a) 학습 안 한 자세에서도 떨어지는가, (b) 파라미터 값이 물리적으로 합리적인가 둘 다 통과해야 채택. [extended_ba §5, §12](hand_eye_extended_ba.md) 의 교훈.
- **Open-loop floor 인정**: 모터 repeatability 가 open-loop 의 hard floor. 그 아래 필요한 정확도는 closed-loop (§3.4) 으로만 도달 가능. 캘 추가로 못 뚫는 벽.
- 5DOF 제약 인지: 도구 축이 직선인 작업(gripper, 노즐)은 자유, 도구 회전 자유도 필요한 작업은 reachable workspace 좁아짐.
- **RL 은 정확도 짜내기에 부적합** (§3.5). 정확도는 지도 학습 / closed-loop 영역. RL 은 스킬 학습용.

---

## 7. 관련 문서

- [hand_eye_extended_ba.md](hand_eye_extended_ba.md) — 현재 floor 0.647°/7.77mm 도달 과정
- [calibration_apply_flow.md](calibration_apply_flow.md) — 4종 산출물의 적용 메커니즘
- [calibration_workflow.md](calibration_workflow.md) — 캡처 절차 + 결과 해석
- [hardware.md](hardware.md) — 모터/링크/3D프린트 토폴로지
- [pick_and_place_walkthrough.md](pick_and_place_walkthrough.md) — 현재 task 흐름
