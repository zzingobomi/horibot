# Random Palletizing — Design Document

ideas.md 의 "Palletizing" 항목이 별도 design 문서로 승격된 본 문서. 박스 spec / 선결 prerequisite / 학습 트랙 (A 휴리스틱 / B 정석 / C iterative sim2real RL) / 운영 시나리오 / 시각화 / curriculum 모두 여기서 다룬다.

## 한 줄 / 왜

- **한 줄**: 사이즈 가변 직육면체 5-10개 + **고정 위치 팔레트**. 매 cycle 마다 카메라로 (a) 다음에 집을 박스 + (b) 어떤 orientation 으로 + (c) 팔레트 어디에 둘지를 동적으로 결정해서 쌓음.
- **왜**: 산업용 팔레타이저 흉내. 단일 픽앤플레이스의 N회 반복이 아니라 **이전 placement 가 다음 placement 의 가능 공간을 바꾸는** closed-loop 상태 추론이 핵심. 큐브 대신 **직육면체 (가로/세로/높이 가변)** 로 가면 orientation 결정이 정책에 추가돼서 진짜 팔레타이저 사고에 가까워짐. 지금까지 만든 자산 (detection / TSDF / GraspPolicy / TaskRunner) 거의 다 한 task 에 끌어와서 통합 데모로도 좋음.

## 학습 트랙 — 세 트랙 병행 (이 문서의 핵심 framing)

- **Track A (휴리스틱)**: 빠르게 baseline 띄움. 어디서 깨지는지 정량 측정 (cycle fail rate, IK fail, stability fail, packing efficiency).
- **Track B (정석)**: 산업/학계 정식 stack. A 의 실패 patterns 를 정석 기법이 어떻게 잡는지 + 얼마나 gain 나는지 정량 비교.
- **Track C (iterative sim2real RL)** ⭐ 메인 방향: 실 환경 노이즈를 sim 에 박고 RL 학습 → real → 로그 → sim 보정 → 반복. 결과 효과 + 학습 효과 둘 다 노림.

**왜 세 트랙 다 가는가:**

- C 단독으로 가면 결과 검증할 baseline 이 없음. RL 이 잘 학습됐는지 휴리스틱과 비교해야 알 수 있음
- A 단독으로 가면 study value 가 휴리스틱 구현 1편으로 좁아짐
- B 의 정석 stack 은 C 의 reward design / domain randomization 에 직접 활용됨 (B.7 안정성 정식 → reward 의 stability term, B.10 reachability → action space 의 mask)
- 정석 트랙만 가면 진척 안 보여서 도중 포기 위험. 휴리스틱 → 실패 측정 → 정석 도입 → gain 측정 → DIY 한계 측정의 4-step 학습이 study value 의 핵심

**2-layer 평가**: sim (MuJoCo, 노이즈 0) 에서 알고리즘 ranking + real (σ_t 7.94mm 노이즈) 에서 robust성 ranking. **sim/real gap 자체가 학습 output** — DIY 의 진짜 한계가 어디서 알고리즘 한계와 분리되는지 정량으로.

## 박스 사이즈 spec

- 모든 변 ∈ **[30, 50]mm**
- 각 박스에 **≥1개 변은 ≤40mm** (그리퍼 개구 호환)
- 5-10개 박스, 사이즈 mix
- 어떤 면을 아래로 하든 contact 변이 ≥30mm → 5층 누적 ~78% 성공률 유지

## 확정 출력 셋 (3D 프린트)

10개 / 7종 / **white 단색** / chamfer 없음 / 윗면 마킹 없음 / infill 15-20% PLA:

| dim (mm) | 분류                 | 목표 수량 | 출력 완료 | 역할                                             |
| -------- | -------------------- | --------- | --------- | ------------------------------------------------ |
| 40×40×40 | 큐브                 | 2         | ✅ 2      | gripper-safe base, identifiability anchor        |
| 30×30×30 | 작은 큐브            | 2         | ✅ 2      | top layer 마무리                                 |
| 50×40×30 | full rect (3-face)   | 2         | ✅ 2      | **face 선택 정책 메인 시험대**                   |
| 40×35×30 | full rect (3-face)   | 1         | 0         | 중간 rect, 50×40×30 와 footprint 구분            |
| 50×50×40 | square base + 다른 H | 1         | ✅ 1      | 큰 footprint, top-down은 40 변만 grip            |
| 50×30×30 | pillar               | 1         | 0         | 5층 누적 빠르게 채움 / 옆으로 눕히면 long ground |
| 35×35×35 | 중간 큐브            | 1         | 0         | identifiability 채움                             |

- **White 단색의 detector 함의**: 색 기반 instance separation 불가 → **depth-aware segmentation 필수**. D405 active stereo 가 white surface 에서도 depth 잘 잡히므로 depth discontinuity + Open3D plane segmentation 으로 분리. prerequisite 1번 (Detector mask) 설계 시 SAM2 RGB-only 가 아니라 depth-gradient 기반 조합.
- **운영 규칙**: 소스 영역에 박스 배치 시 박스 간 최소 5-10mm 간격 유지 (붙어있으면 한 박스로 segment 위험).
- **Chamfer/마킹 없음 영향 없음**: 큐브 4종은 4-fold symmetry 로 yaw 무의미, rect 3종은 2-fold self-symmetry 라 180° ambiguity 가 placement 동등 → marking 으로 안 깨도 정책 무관.

## 선결 — 회전된 박스 grasp pipeline (트랙 무관 prerequisite)

5DOF + J5 wrist roll 로 **운동학적으론 가능**, 소프트웨어가 4군데에서 orientation 정보 끊고 있음. 7개 wire-up:

1. **Detector** — depth connected component → cluster 내부 decomposition → 박스 단위 oriented bbox + dim 식별 + yaw 추출. white 단색 + touching 허용 환경 대응:
   - **분리 단서 3종**: (a) depth step (다른 height 박스 touching), (b) silhouette 꺾임 (다른 yaw touching), (c) RGB micro-shadow at seam (3D 프린트 모서리 imperfection)
   - **Known-dim hypothesis matching**: 분리 단서로도 못 푸는 worst case (같은 height + 같은 yaw + perfect align) 는 footprint 를 알려진 7종 dim set 분해 후보 enumerate. 다중 hypothesis 면 옆면 depth profile 로 disambiguate.
   - **분해 실패 시 reject + 사용자 호출** (운영 fallback)
   - 180° 대칭 ambiguity 는 rect 박스 자체 2-fold self-symmetry 라 placement 동등 → 별도 해소 불필요
2. **Step DSL** — `Position3` → `Pose6` (또는 quaternion 필드 추가). `GraspPolicyStep` / `GroundedDetectStep` 출력 확장
3. **Motion API** — [motion_modes.servo_tcp](../backend/modules/kinematics/motion_modes.py) 가 이미 quaternion 받음 (motion_taxonomy.md Phase 1 에서 6DOF 화). `ServoTcpReq.quaternion` 으로 전달.
4. **Grasp candidate enumerator** — (top-down × yaw / side × yaw) family enumerate + **J5 closest-arc 선택** (4-fold 대칭 중 wrist 안 도는 쪽)
5. **Reachability filter** — PyBullet IK + self-collision 사전 검증
6. **Orientation-lock Cartesian descent** — hover→grip 중 wrist 흔들림 방지. `move_l` orientation interpolation 확인 / 필요 시 `move_lockori` 추가
7. **Placement-aware pick yaw** — pick yaw 는 place yaw 의 함수 (J5 transit cost + manipulability). pick/place 를 한 plan 으로 묶는 구조

**5DOF 한계 명확화** — 임의 6DoF 불가, 도달 가능한 건 **1-parameter family** (top-down × yaw, side × yaw):

- upright + table yaw: ✓ J5 로 보상
- 옆으로 누운 박스: 새 top 면 dim ≤40mm 일 때만 top-down, 아니면 side 접근인데 워크스페이스 가장자리 unreachable 많음
- 기울어짐 / 다른 박스 밑: reject 정책

**이 prerequisite 없이는 세 트랙 다 yaw=0 큐브로 격하** — 알고리즘 정교화 ROI 무의미.

## Feasibility 검토 (목표 ≥5층)

결론: **조건부 가능**. 큐브 ≥30mm + per-layer re-grounding 이면 데모 수준 동작 기대.

큐브 사이즈 vs 5층 누적 성공률 (σ_t 7.94mm 가우시안, CoM 이 아래 큐브 contact polygon 안에 들어와야 정적 안정):

| 큐브 변 | 허용 오차 (반변) | 단일 placement 실패율 | 5층 누적 성공률    |
| ------- | ---------------- | --------------------- | ------------------ |
| 20mm    | ±10mm            | ~16%                  | ~50% (코인플립)    |
| 30mm    | ±15mm            | ~6%                   | ~78% (가끔 무너짐) |
| 40mm    | ±20mm            | ~1.2%                 | ~95% (안정)        |

⇒ **20mm 는 5층 데모로 부적합. ≥30mm 필수, sweet spot 30mm**.

## 누적 오차 신화 — per-layer re-grounding 이 아키텍처 단일 결정점 (세 트랙 공통)

- **Feed-forward (재검출 X)**: 5층 후 누적 ~30mm → 거의 항상 무너짐
- **매 cycle 카메라로 stack top 재검출** → 각 layer error 독립 → 5층 후 총 오차 = σ_t (~8mm)
- 구현: height-map 을 매 placement 직전 capture 로 refresh. 필요시 전용 **observation pose** (stack 잘 보이는 자세)

---

## Track A — 휴리스틱 트랙 (빠른 baseline)

**sub-problem 5개**:

1. **박스 enumerate + 치수 측정** — `SearchAndDetect` 에 `enumerate_all=True`. TSDF / 포인트클라우드 스캔 → 박스별 segmentation → Open3D oriented bbox (yaw 포함, prerequisite 1번 활용)
2. **팔레트 상태 모델** — **height map** (격자별 stack 높이). `pallet_origin_xyz` + `pallet_size` 하드코딩
3. **Selection + orientation policy** — **큰 거부터 + flat 면 아래로** (산업 흔한 휴리스틱). 안정성 + 공간 효율
4. **Placement policy** — **greedy Bottom-Left-Fill** on height map. "지지면 N% + 도달 가능 + 수직 접근 가능" 한 가장 낮은 위치
5. **Stacking 정확도 보정** — placement 후 visual check, 안 맞으면 미세 조정 또는 fail. Per-layer re-grounding 으로 다음 cycle 에 보정

**운영 시나리오 v1 — Track A 결정사항** (sub-problem 의 구체화):

- **Setup**: 사람이 박스 10개를 source 영역에 흩어놓음. **박스끼리 안 겹침만 보장하면 자유** — yaw 자유, 어떤 면이 위든 자유. flat-down 강제 룰은 X (평지 단독 안착 = 물리적 자동, 박스가 비스듬히 정지 못 함). face 선택의 자연 랜덤성도 사람 셋업에서 확보.
- **Layout**: 좌측 source / 우측 palette 분리 — J1 yaw 회전만으로 transit, 두 영역 사이 공간 위로 EE 미경유 (carrying 중 다른 박스 위 가로지름 X). 정확한 좌표는 셋업 시점 PyBullet reach mask 로 결정. 사용자가 책상 마킹 후 좌표를 yaml 에 입력 → IK 가 obs pose 자동 계산.
- **Observation — 자세 2개 박아둠** (cycle 마다 동일 자세 재사용):
  - **Source obs pose**: source center 위 **angled view** (top-down 강제 X — 5DOF + 짧은 정면거리에서 운동학 빡빡. hand-eye extrinsic 정확하면 perspective re-project 으로 정보 손실 없음)
  - **Palette obs pose**: stack 옆 **angled side view**. stack 1층 (50mm) ~ 5층 (250mm) 모두 같은 자세 1개로 보임. **위에서 보면 5층 reach 초과** (√(150²+80²+450²)=480mm > 380mm) 라 side view 필수.
  - 매 cycle **single-shot depth + RGB 한 장씩**. cuboid + flat-down + 안 겹침 → 박스 pose 4DoF (x, y, z, yaw) 로 reduce, footprint(W,L) + height(H) + yaw 한 장으로 추출 완전. **TSDF / multi-view rebuild 매 cycle 불필요** (그건 자유 형상 / 클러터 시나리오용).
- **Detect**: depth connected component → cluster decomposition (위 "선결 prerequisite #1" 항목). worst case 만 옆 view 1개 추가 fallback.
- **Decision — Track A 휴리스틱**:
  1. Selection: 큰 footprint 박스부터 (tie-break: footprint area)
  2. Face: 가장 큰 face 아래 + **그리퍼 호환 (위에서 잡을 변 ≤40mm) constraint** — 큰 면 아래 정책이 그리퍼 호환과 충돌 가능 (예: 50×50×40 박스를 50×40 면 아래로 두면 위에서 잡을 변 = 50 ✗)
  3. Place 위치: BLF on height map, 첫 박스는 좌하단 모서리
  4. Place yaw: axis-align (yaw=0) + Selection 시점 reachability pre-flight (PyBullet IK + self-collision)
- **Cycle time 추정** (한 박스): Observation 3-5초 + Decision <1초 + Pick 5-8초 + Place 5-8초 = **13-22초**. 10개 → **2.5-4분 전체 데모**.

**예상 실패 케이스 (이게 학습 자료)** — Track B/C 의 motivation 이 됨:

- BLF 가 5DOF unreachable spot 선택 → IK fail
- Greedy 가 "다음 박스 들어갈 자리 막는" placement → 후반 cycle fit 실패
- 안정성 평가 = 지지면 ratio 만 → CoM 분석 없어서 가끔 무너짐
- yaw 회전 박스의 4-fold 대칭 중 안 좋은 J5 각 선택 (prerequisite 있어도 정책이 약하면)
- 직육면체 face 선택 정책이 약함 — "큰 면 아래" 가 stability optimal 이 아닌 경우 잡지 못함
- 치수 측정 1-2mm 오차 → 가시적 흔들림 (ICP refinement 없음)

**측정 항목** (Track A 의 실 output): cycle fail rate / IK fail rate / topple rate / packing utilization %. **이 정량 데이터 자체가 Track B/C 의 동기와 평가 기준**

---

## Track B — 정석 트랙 (proper 학습 stack)

각 항목 옆 → Track A 의 어느 실패 patterns 를 잡는지 mapping. C 와의 직접 연결도 표기.

- **B.1 문제 정형화** — Online 3D BPP 의 state representation (height map / EMS / corner points / extreme points 비교) / action / reward / 제약 정식 작성. 부분관측이면 POMDP. paper exercise 지만 안 하면 다 surface. **→ Track C 의 state/action 정형화에 직접 사용**
- **B.2 Exact methods** — MILP 모델링 (Tsai/Chen 류 변수/제약), **CP-SAT (Google OR-Tools — BPP 에 의외로 강함)**, branch-and-bound. n=5/7/10 시간 폭발 시점 측정 → 복잡도 직관. → 모든 Track A 실패의 upper bound reference (sim, noise=0)
- **B.3 정식 constructive heuristics** — **Extreme Points (Crainic et al. 2008)**, DBLF, skyline algorithm, maximal-rectangles. Track A 의 BLF 보다 일반적인 후보 enumeration. → "다음 박스 자리 막힘"
- **B.4 메타휴리스틱** — GA (box-ordering chromosome + crossover), Tabu Search neighborhood, SA, GRASP. **Bortfeldt-Gehring** 의 classic palletizing reference
- **B.5 Search 기반** — MCTS (UCB1/PUCT exploration constant, rollout policy), beam search width tuning. **Lookahead depth 가 greedy 대비 어디서 break-even** 인지 측정. → "다음 자리 막힘", 직육면체 face 선택. **→ Track C 의 search-augmented policy 옵션**
- **B.6 DRL** — **Zhou et al. AAAI 2021 PackNet** replicate (constrained action space + feasibility mask + PPO). Zhao 2022 PCT, Attend2Pack, TAP-Net. **→ Track C 의 직계 선조 — 모듈은 그대로 C 의 RL 학습 단계에서 재활용**
- **B.7 안정성 정식** — CoM support polygon (정역학), **Stewart-Trinkle equilibrium** (friction cone 포함), force closure vs form closure 구분. PyBullet sim 과 정역학 cross-validate. Learned stability classifier (CNN on rendered scene) 옵션. → "지지면 ratio 만으로는 무너짐". **→ Track C 의 reward shaping (stability term) 직접 입력**
- **B.8 Grasp planning 정식** — **Force closure metric (Ferrari-Canny)**, antipodal grasp computation, 6DoF grasp synthesis (AnyGrasp / Contact-GraspNet). **5DOF reachable grasp manifold 를 Jacobian rank / manipulability ellipsoid 로 derive** — 5DOF 의 어떤 grasp 가 가능한지 정식으로. → "yaw 4-fold 안 좋은 선택". **→ Track C 의 action space refinement**
- **B.9 Object pose 정식** — **ICP variants**: point-to-point / point-to-plane / **GICP (Generalized)** / Colored ICP / **GoICP (globally optimal)**. Oriented bbox + symmetry resolution 정식 알고리즘. 6DoF DL (FoundationPose, Megapose) 와 oriented bbox + Z-up assumption 의 trade-off 측정 — 5DOF 한정 marginal value 정량. → "치수 측정 오차". **→ Track C 의 perception noise model 보정 입력**
- **B.10 Motion planning** — Cartesian L-move with orientation lock (SLERP), **CBiRRT (Constrained Bi-RRT)** — orientation manifold constrained sampling, TrajOpt / CHOMP / STOMP optimization-based. Manipulability-aware planning. → IK fail rate, "비스듬한 접근 모서리 닿음". **→ Track C 의 action space mask**

---

## Track C — Iterative sim2real RL (메인 방향)

DIY 의 노이즈 (캘 σ_t 7.94mm, dim 측정 오차, placement bias 등) 를 sim 에 박고 RL 학습 → real 적용 → 로그 분석 → sim 보정 → 반복하는 **iterative real-to-sim-to-real** 사이클. Track A 의 BLF 가 noise-blind 라면, C 는 noise 분포 안에서 robust 정책을 학습. 학계 용어로는 *system identification + domain randomization + sim2real gap closure*. Track B.6 (Zhou 2021 PackNet, Zhao 2022 PCT) 의 직계 확장이지만 **sim2real gap closure 가 main 작업**.

**왜 메인 방향**:

- **결과 효과**: noise-injected sim 안에서 RL > 휴리스틱 BLF (BLF 는 가장자리 빠듯하게 놓다가 σ_t 노이즈에 떨어짐, RL 은 안쪽 마진 두는 robust 행동 학습)
- **학습 효과**: 산업 sim2real pipeline 의 표준 flow (NVIDIA Isaac / OpenAI ADR / Google Robotics 의 standard cycle) 직접 경험
- **DIY 비유**: 사이드 프로젝트라도 회사에서 React 쓰면 React 직접 써봐야 학습 효과 있음. 결과/ROI 만 보면 BLF 가 가성비 좋지만 industry-standard pipeline 경험 자체가 study output 의 핵심

### Pipeline — 7단계 iterative loop

```
[1] 실 환경 노이즈 측정 → 수치화 (system identification)
    - 이미 있음: 캘 σ_t 7.94mm, sag 모델
    - 신규 측정: dim 측정 오차 분포 (footprint W/L/H 별)
                placement target vs actual 분포
                IK fail 패턴, 자세별 bias

[2] MuJoCo env 에 노이즈 박기 (domain randomization)
    - URDF → MJCF 변환 (OMX_F)
    - 박스 7종 spec 그대로 reconstruction
    - DR: mass / friction / dim / camera pose / depth / actuation / 그리퍼 timing

[3] RL 학습 (PPO + Stable-Baselines3)
    - Action: object-centric (어느 박스 / grid cell / yaw)
    - Reward: stack height + stability + IK fail penalty + packing bonus
    - Episode: 박스 10개 다 쌓거나 무너지면 종료

[4] real 적용
    - 1차 iter 항상 깨짐 (예상됨, 이게 학습 자료)

[5] 로그 수집 (★ 인프라 핵심)
    - intended placement vs actual placement
    - 어느 layer 에서 무너졌는지
    - 어떤 자세에서 IK fail
    - detector dim 오차 분포
    - cause attribution (decision / motion / perception / actuation)

[6] 분석 → sim noise model 보정 결정 (residual analysis)
    - gap pattern → noise 추가/조정 매핑
    - 예: 자세 5 에서 placement bias 12mm → sim actuation noise 에 자세 의존 bias 추가
    - 예: dim 측정이 heavy-tail → sim dim noise 분포 변경

[7] [2] → [3] → [4] → [5] → [6] 반복
```

### 핵심 인프라

**측정 인프라 (system identification)**:

- 이미 있음: 캘 σ_t (확장 BA + sag 모델), [hand_eye_extended_ba.md](hand_eye_extended_ba.md) 참조
- 신규: dim 측정 분포 측정 task (cube 1개 → obs pose 1개 → dim 100회 측정 → 분포). placement target vs actual 측정 task (단순 pick-and-place 100회 반복)

**로그 인프라 (sim2real gap closure)**:

- Topic: `omx/palletizer/cycle_log` (cycle 끝나면 1회 publish)
- 페이로드: intended_placement, actual_placement, layer_index, success/fail, cause_attribution, timing breakdown
- Persistence: `scan_<id>.npz` 와 동일 패턴으로 `cycle_<id>.npz` 저장 ([../backend/modules/pointcloud/scan_io.py](../backend/modules/pointcloud/scan_io.py) 패턴 재사용)

**분석 → sim 보정 매핑 (residual analysis)**:

- gap pattern 분류기 (수동 시작 → 패턴 누적되면 자동화 검토)
- 매 iter 마다 변경된 noise 항목 changelog (별도 iter log 파일로 관리)

### Sim 환경 (MuJoCo)

**선택 이유**: 산업/연구 표준 (DeepMind / Google Robotics / NVIDIA Isaac 등 거의 다 MuJoCo 계열). PyBullet 으로 가면 인프라 재사용 효율은 좋지만 산업 flow 경험 가치 손실. 더 가면 NVIDIA Isaac Lab (GPU 병렬 환경 수천 개 동시 학습) 옵션 — 단 setup 무겁고 학습 곡선 가파라서 MuJoCo 가 첫발로 합리적.

**URDF → MJCF 변환**:

- [../robot/urdf/omx_f/](../robot/urdf/omx_f/) 의 URDF 를 MJCF 로 변환 (mujoco 의 `urdf2mjcf` 또는 수동)
- **link_offset 패치된 URDF 그대로 사용** (캘 일관성)
- sag 모델은 [4] real 적용 시점에 PyBullet solver 가 처리 (sim 에는 sag 처음엔 안 박음, 2차 iter 이후 검토)

**박스 7종 setup**: 박스 spec 표 그대로 MJCF box body 로 reconstruction. White 단색 / chamfer 없음 / 3D print warping ±0.2mm.

**Domain randomization 항목**:

- Mass: ±5% (3D print infill 변동)
- Friction: μ ∈ [0.3, 0.7] (PLA-PLA / PLA-종이 등)
- Dim noise: 각 변 ±1mm (3D print 정확도)
- 카메라 pose 오차: σ_t 7.94mm, σ_rot 0.65° (extended BA 측정값)
- Depth noise: D405 spec sheet 기반 (제조사 ~1% at 0.5m)
- Actuation 오차: σ_t 8mm 가우시안 (1차), 자세 의존 bias (2차 iter 이후)
- 그리퍼 release timing: ±20ms

### RL 설계

**Action space — 후보 3개**:

- (a) Joint-level (qdot) — sim2real physics fidelity 차이로 깨짐, 후보 아님
- (b) Object-centric (어느 박스 / grid cell / yaw) — sim2real transfer 안정성 ↑, 학습 가치는 packing decision 으로 좁아짐. **1차 iter 채택**
- (c) Cartesian pose + IK 위임 — 중간, 5DOF reachable manifold 도 같이 학습 필요. 2차 iter 이후 도전

**Reward**:

- Per-step: 박스 placement 후 stack height delta × +1
- Per-step penalty: IK fail × −5, collision × −2
- Episode end: 박스 다 쌓음 × +20, 무너짐 × −10, packing efficiency bonus × score

**Algorithm**:

- Stable-Baselines3 PPO (산업 / 연구 표준 baseline)
- VecEnv 으로 병렬 환경 (16-64)
- 학습 step: 1차 iter ~ 5M step (PPO 기준 packing 류 paper 평균)

**Episode**:

- 박스 10개 source 영역 random pose → 다 쌓거나 무너지면 종료
- 무너짐 정의: 어느 박스든 z 가 layer 평균보다 1cm 아래로 떨어짐

### 첫 iter MVP

**최소 setup**:

- MuJoCo env + 박스 7종 + 팔레트
- 노이즈: 가우시안 σ_t 8mm 만 (다른 noise 는 2차 이후)
- RL: PPO 1M step
- real: 박스 10개 → 5층 시도 → cycle 마다 placement actual / 무너짐 layer 만 로그

**성공 기준 (1차 iter)**:

- sim 에서 5층 누적 50% 이상
- real 에서 1개 박스라도 placement 성공
- 로그 인프라 가동 (10 cycle 데이터 수집)

**예상 실패 모드 (학습 자료)**:

- sim 잘 됐는데 real 0층 — sim/real action mapping 깨짐 (action space 재검토)
- real placement bias 가 σ_t 8mm 보다 큼 — sim noise model 부족 (자세 의존 bias / actuation drift 추가)
- detector dim 오차가 ±1mm 보다 큼 — perception noise model 추가

### Iter cycle 운영

**다음 iter trigger**:

- 현재 iter 의 real 5층 성공률 변화가 std error 안으로 들어옴 (수렴)
- 또는 새로운 gap pattern 발견 (예측 못 한 실패 모드)

**Iter 간 변경 항목 changelog**:

- 별도 파일 `docs/random_palletizing_iter_log.md` (또는 본 문서 부록) 에 매 iter 의 (a) 추가/조정된 noise (b) 변경된 reward term (c) 변경된 action space 누적 기록

### Track A 와의 관계

- A 는 baseline, C 는 iterative 학습 트랙
- 같은 sim/real 에서 동일 metric (cycle fail rate / topple rate / packing %) 으로 비교
- A 가 결과 책임 (가성비 baseline 으로 항상 가동), C 가 학습 + 진단 책임
- A 의 성능 천장이 시스템의 deterministic 천장. C 가 그 위를 얼마나 깨는지가 RL gain
- 단 C 의 1차 iter 결과는 A 보다 못 나올 가능성 높음 (예상됨, iter 반복으로 따라잡음)

---

## 비교 metric — 세 트랙 평가 차원

- 공간 효율 (utilization %) — B.2 exact / C DRL 가장 빛남
- 단일 cycle stability fail rate — B.7 안정성 정식 / C noise-aware policy 가장 빛남
- 5층 누적 topple rate
- IK fail rate (reachability) — B.10 가장 빛남
- Cycle time (planning + 실행)
- **알고리즘 결정 시간 vs 실행 오차 break-down** — sim/real gap, **DIY 한계의 정량적 답**
- **sim2real gap 자체** (Track C 고유) — iter 마다 줄어드는지

## Step 구조

**`PalletizeStep` primitive 신규**. 1 step 안에 cycle loop 캡슐화 + **트랙 선택 옵션** (`policy="heuristic"|"ep"|"mcts"|"milp"|"rl"`). LLM orchestrator 는 `[PalletizeStep(max_boxes=10, policy="heuristic")]` 만 짜면 됨.

## 3D 시각화 — 실시간 world model 레이어 (세 트랙 공통)

박스 사이즈가 작은 집합이라 [Workspace3D](../frontend/src/pages/Workspace3D.tsx) 안에 three.js `<boxGeometry>` 로 그릴 수 있음. URDF / PointCloudLayer / MeshLayer 옆에 **PalletizerLayer** 추가.

- **토픽**: `omx/palletizer/state` (cycle 마다, ~5Hz)
  ```
  {
    boxes: [
      {id, dims:[L,W,H], pose:[x,y,z,qx,qy,qz,qw],
       state:"source"|"held"|"placed", color?}
    ],
    pallet: {origin:[x,y,z], size:[W,D]},
    next_placement?: {pose, dims, score?, policy?}   // 정책 후보 + 점수 (트랙 비교 시각화)
  }
  ```
- **프론트 레이어**: 팔레트 outline / `source`·`placed`·`held` 박스별 색 / `next_placement` dashed wireframe + score + policy 라벨
- **Identity tracking**: dims similarity + 위치 근접으로 cycle 간 매칭. SLAM 불필요
- **추가 효용**: (a) 측정/정책 버그 모션 전 catch, (b) **세 트랙 비교 visual evidence**, (c) 데모 가치 ↑

## DIY 에서 진짜 발목 잡을 항목들 (세 트랙 공통)

1. **Release dynamics** — 그리퍼 너무 높이서 열면 떨어지는 충격으로 stack 흔듦. **Dynamixel current spike 로 contact 검출 후 open**. XL430 OK, XL330 노이즈 ↑
2. **5DOF 접근 각도** — top-down 은 base-arm 평면 안에서만 정확. **팔레트를 base 정면에 셋업** + IK reachability check (Track B.10 의 응용)
3. **5DOF 수직 reach** — 5층 × 40mm + 팔레트 + hover ≈ 300mm. OMX_F reach 380mm. **셋업 시점 reach mask** 사전 enumerate (격자 × 높이 × yaw bin)
4. **Gripper 개구 + orientation** — gripper max ~40mm. prerequisite candidate enumerator + B.8 grasp manifold 결합
5. **Stack top 가림** — eye-in-hand 라 observation pose 비용 감수

## 권장 진행 순서 (curriculum)

1. **선결 prerequisite — 회전 박스 grasp pipeline** (7개 wire-up) — 세 트랙 다 시작 전 필수
2. **Track A 베이스라인 + 측정 인프라** — 휴리스틱 BLF + sim/real 측정 hook (실패 patterns 분류 + 통계). **Track C 의 로그 인프라가 여기서 같이 깔림** (`cycle_log` topic + persistence)
3. **회전 큐브 baseline** — Track A 로 30mm 큐브 3층 (feed-forward), σ_t 실측 + 무너짐 patterns 데이터 → Track C 의 system identification 1차 input
4. **Per-layer re-grounding** — 5층 도전 (Track A 유지)
5. **직육면체 도입 (Track A)** — 가변 dim + face 선택. 실패 patterns 확장 측정
6. **Track C iter 1** — MuJoCo env + 가우시안 노이즈 + PPO 1M step → real → 로그 → gap 측정. **A 의 baseline 과 동시 가동 비교**
7. **Track B 점진 도입 — C 의 reward / DR 에 input 으로**:
   - **B.1 정형화** (paper exercise) — Track C 의 state/action 정형화에 직접 사용
   - **B.7 안정성 (정역학 + sim)** → C 의 reward 의 stability term
   - **B.10 reachability-aware motion** → C 의 action space mask
   - **B.3 EP/DBLF** → Track A baseline 강화 (C 와의 비교 기준)
   - **B.5 MCTS/beam** → C 와 hybrid 가능성 (search-augmented policy)
   - **B.2 MILP / CP-SAT** — n=5/7/10 시간 측정, exact baseline 확보 (sim 안에서만)
   - **B.8 Grasp 정식** — yaw 4-fold 선택 quality, C 의 action space refinement
   - **B.9 ICP refinement** — 치수 정확도 gain, C 의 perception noise model 보정
8. **Track C iter 2+** — B 의 inputs 반영 후 noise model / reward / action space 재학습. sim2real gap 축소 측정
9. **Sim2real gap 분석** — 각 iter 의 gap pattern 분류 + 정량 변화. **DIY 한계의 정량적 답** — 이게 study output 의 메인

## 양보 못 하는 두 제약

**(a) 모든 변 ≥30mm** **(b) per-layer visual re-grounding**

## 리스크 / 트레이드오프

- **치수 측정 오차** — 1-2mm → 가시 흔들림. B.9 ICP refinement 로 잡힘
- **Cycle time** — observation pose + re-detection + planning. Track A ~10-15초, B.2 MILP / B.5 MCTS 추가 비용 측정 필요
- **Track C 의 sim2real gap closure 가 발산 가능성** — 매 iter 마다 noise model 추가하다 보면 over-fitting. iter changelog 로 관리하고 ablation (특정 noise 제거 후 재학습) 으로 검증
- **Track C 1차 iter 결과가 휴리스틱보다 못함** — 예상됨, 받아들이고 시작. iter 반복으로 따라잡음
- **모든 Track B 항목 = 정석 implementation 자체에 학습 시간** — paper 1-2개 읽고 구현 단위. study 가 목적이라 이게 비용이 아니라 곧 output

## 의존성

- **공통**: 포인트클라우드 / TSDF 인프라 (있음). 객체 enumerate 모드 (recipe 화 가능). PalletizeStep. observation pose 정의. Current-spike contact detect. 셋업 reach mask 사전 계산
- **선결**: 회전 박스 wire-up 6개 (Detector cluster decomp / Motion API quaternion / Grasp enumerator + J5 closest-arc / Reach filter / Orientation-lock descent / Placement-aware pick yaw). `Position3` → `Pose6` typed schema + Step DSL 토대는 [step_dsl.md](step_dsl.md) 에서 완성됨. 나머지 6개 wire-up 은 Palletizing 본 작업 범위.
- **Track A**: greedy BLF + height map + 측정 hook (~수백 줄)
- **Track B**: B.1-B.10 각각 수일~수주. PyBullet sim env (B.6, B.7 공유). OR-Tools (B.2). Open3D ICP 변종 (B.9). 모듈별 paper reference 별도 정리 가치
- **Track C**: MuJoCo + URDF→MJCF + Stable-Baselines3 + cycle_log 인프라 (~수천 줄, 가장 큰 의존성). 1차 iter 의 noise model 은 단순 가우시안만 → 2차 iter 부터 B 와 측정 데이터로 정교화
