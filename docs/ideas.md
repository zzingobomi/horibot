# Ideas

새 기능/방향 아이디어 모음. roadmap.md 는 "할 거"이고 여기는 "할까 말까 — 검토 대기" 버킷. 채택되면 roadmap 으로 옮기거나 별도 design 문서로 승격.

각 항목 포맷:

- **한 줄**: 뭘 하자는 거
- **왜**: 동기 / 지금 못 하는 것
- **어떻게**: 대략적 구현 스케치
- **리스크 / 트레이드오프**
- **의존성**: 선행으로 깔려야 하는 인프라

---

## Palletizing — 다양한 크기 직육면체 쌓기 (★ 유력)

- **한 줄**: 사이즈 가변 직육면체 5-10개 + **고정 위치 팔레트**. 매 cycle 마다 카메라로 (a) 다음에 집을 박스 + (b) 어떤 orientation 으로 + (c) 팔레트 어디에 둘지를 동적으로 결정해서 쌓음.
- **왜**: 산업용 팔레타이저 흉내. 단일 픽앤플레이스의 N회 반복이 아니라 **이전 placement 가 다음 placement 의 가능 공간을 바꾸는** closed-loop 상태 추론이 핵심. 큐브 대신 **직육면체 (가로/세로/높이 가변)** 로 가면 orientation 결정이 정책에 추가돼서 진짜 팔레타이저 사고에 가까워짐. 지금까지 만든 자산 (detection / TSDF / GraspPolicy / TaskRunner) 거의 다 한 task 에 끌어와서 통합 데모로도 좋음.

- **학습 전략 — 두 트랙 병행 (이 섹션의 핵심 framing)**:

  - **Track A (휴리스틱)**: 빠르게 baseline 띄움. 어디서 깨지는지 정량 측정 (cycle fail rate, IK fail, stability fail, packing efficiency).
  - **Track B (정석)**: 산업/학계 정식 stack. A 의 실패 patterns 를 정석 기법이 어떻게 잡는지 + 얼마나 gain 나는지 정량 비교.
  - **이유**: 정석 트랙만 가면 진척 안 보여서 도중 포기 위험. 휴리스틱 → 실패 측정 → 정석 도입 → gain 측정 → DIY 한계 측정의 4-step 학습이 study value 의 핵심.
  - **2-layer 평가**: sim (PyBullet, 노이즈 0) 에서 알고리즘 ranking + real (σ_t 7.94mm 노이즈) 에서 robust성 ranking. **sim/real gap 자체가 학습 output** — DIY 의 진짜 한계가 어디서 알고리즘 한계와 분리되는지 정량으로.

- **박스 사이즈 spec**:

  - 모든 변 ∈ **[30, 50]mm**
  - 각 박스에 **≥1개 변은 ≤40mm** (그리퍼 개구 호환)
  - 5-10개 박스, 사이즈 mix
  - 어떤 면을 아래로 하든 contact 변이 ≥30mm → 5층 누적 ~78% 성공률 유지

- **확정 출력 셋 (3D 프린트)** — 10개 / 7종 / **white 단색** / chamfer 없음 / 윗면 마킹 없음 / infill 15-20% PLA:

  | dim (mm) | 분류                 | 목표 수량 | 출력 완료 | 역할                                             |
  | -------- | -------------------- | --------- | --------- | ------------------------------------------------ |
  | 40×40×40 | 큐브                 | 2         | 1         | gripper-safe base, identifiability anchor        |
  | 30×30×30 | 작은 큐브            | 2         | 1         | top layer 마무리                                 |
  | 50×40×30 | full rect (3-face)   | 2         | 0         | **face 선택 정책 메인 시험대**                   |
  | 40×35×30 | full rect (3-face)   | 1         | 0         | 중간 rect, 50×40×30 와 footprint 구분            |
  | 50×50×40 | square base + 다른 H | 1         | 0         | 큰 footprint, top-down은 40 변만 grip            |
  | 50×30×30 | pillar               | 1         | 0         | 5층 누적 빠르게 채움 / 옆으로 눕히면 long ground |
  | 35×35×35 | 중간 큐브            | 1         | 0         | identifiability 채움                             |

  - **White 단색의 detector 함의**: 색 기반 instance separation 불가 → **depth-aware segmentation 필수**. D405 active stereo 가 white surface 에서도 depth 잘 잡히므로 depth discontinuity + Open3D plane segmentation 으로 분리. prerequisite 1번 (Detector mask) 설계 시 SAM2 RGB-only 가 아니라 depth-gradient 기반 조합.
  - **운영 규칙**: 소스 영역에 박스 배치 시 박스 간 최소 5-10mm 간격 유지 (붙어있으면 한 박스로 segment 위험).
  - **Chamfer/마킹 없음 영향 없음**: 큐브 4종은 4-fold symmetry 로 yaw 무의미, rect 3종은 2-fold self-symmetry 라 180° ambiguity 가 placement 동등 → marking 으로 안 깨도 정책 무관.

- **선결 — 회전된 박스 grasp pipeline (트랙 무관 prerequisite)**:

  - 5DOF + J5 wrist roll 로 **운동학적으론 가능**, 소프트웨어가 4군데에서 orientation 정보 끊고 있음. 7개 wire-up:
    1. **Detector** — depth connected component → cluster 내부 decomposition → 박스 단위 oriented bbox + dim 식별 + yaw 추출. white 단색 + touching 허용 환경 대응:
       - **분리 단서 3종**: (a) depth step (다른 height 박스 touching), (b) silhouette 꺾임 (다른 yaw touching), (c) RGB micro-shadow at seam (3D 프린트 모서리 imperfection)
       - **Known-dim hypothesis matching**: 분리 단서로도 못 푸는 worst case (같은 height + 같은 yaw + perfect align) 는 footprint 를 알려진 7종 dim set 분해 후보 enumerate. 다중 hypothesis 면 옆면 depth profile 로 disambiguate.
       - **분해 실패 시 reject + 사용자 호출** (운영 fallback)
       - 180° 대칭 ambiguity 는 rect 박스 자체 2-fold self-symmetry 라 placement 동등 → 별도 해소 불필요
    2. **Step DSL** — `Position3` → `Pose6` (또는 quaternion 필드 추가). `GraspPolicyStep` / `GroundedDetectStep` 출력 확장
    3. **Motion API** — [motion_modes.move_tcp](../backend/modules/kinematics/motion_modes.py) 가 quaternion 받게 (solver `ik()` 는 이미 받음 — wrapper 만 None 으로 끔)
    4. **Grasp candidate enumerator** — (top-down × yaw / side × yaw) family enumerate + **J5 closest-arc 선택** (4-fold 대칭 중 wrist 안 도는 쪽)
    5. **Reachability filter** — PyBullet IK + self-collision 사전 검증
    6. **Orientation-lock Cartesian descent** — hover→grip 중 wrist 흔들림 방지. `move_l` orientation interpolation 확인 / 필요 시 `move_lockori` 추가
    7. **Placement-aware pick yaw** — pick yaw 는 place yaw 의 함수 (J5 transit cost + manipulability). pick/place 를 한 plan 으로 묶는 구조
  - **5DOF 한계 명확화** — 임의 6DoF 불가, 도달 가능한 건 **1-parameter family** (top-down × yaw, side × yaw):
    - upright + table yaw: ✓ J5 로 보상
    - 옆으로 누운 박스: 새 top 면 dim ≤40mm 일 때만 top-down, 아니면 side 접근인데 워크스페이스 가장자리 unreachable 많음
    - 기울어짐 / 다른 박스 밑: reject 정책
  - **이 prerequisite 없이는 두 트랙 다 yaw=0 큐브로 격하** — 알고리즘 정교화 ROI 무의미

- **Feasibility 검토 (목표 ≥5층)** — 결론: **조건부 가능**. 큐브 ≥30mm + per-layer re-grounding 이면 데모 수준 동작 기대.

  큐브 사이즈 vs 5층 누적 성공률 (σ_t 7.94mm 가우시안, CoM 이 아래 큐브 contact polygon 안에 들어와야 정적 안정):

  | 큐브 변 | 허용 오차 (반변) | 단일 placement 실패율 | 5층 누적 성공률    |
  | ------- | ---------------- | --------------------- | ------------------ |
  | 20mm    | ±10mm            | ~16%                  | ~50% (코인플립)    |
  | 30mm    | ±15mm            | ~6%                   | ~78% (가끔 무너짐) |
  | 40mm    | ±20mm            | ~1.2%                 | ~95% (안정)        |

  ⇒ **20mm 는 5층 데모로 부적합. ≥30mm 필수, sweet spot 30mm**.

- **누적 오차 신화 — per-layer re-grounding 이 아키텍처 단일 결정점** (두 트랙 공통):
  - **Feed-forward (재검출 X)**: 5층 후 누적 ~30mm → 거의 항상 무너짐
  - **매 cycle 카메라로 stack top 재검출** → 각 layer error 독립 → 5층 후 총 오차 = σ_t (~8mm)
  - 구현: height-map 을 매 placement 직전 capture 로 refresh. 필요시 전용 **observation pose** (stack 잘 보이는 자세)

---

### Track A — 휴리스틱 트랙 (빠른 baseline)

- **sub-problem 5개**:

  1. **박스 enumerate + 치수 측정** — `SearchAndDetect` 에 `enumerate_all=True`. TSDF / 포인트클라우드 스캔 → 박스별 segmentation → Open3D oriented bbox (yaw 포함, prerequisite 1번 활용)
  2. **팔레트 상태 모델** — **height map** (격자별 stack 높이). `pallet_origin_xyz` + `pallet_size` 하드코딩
  3. **Selection + orientation policy** — **큰 거부터 + flat 면 아래로** (산업 흔한 휴리스틱). 안정성 + 공간 효율
  4. **Placement policy** — **greedy Bottom-Left-Fill** on height map. "지지면 N% + 도달 가능 + 수직 접근 가능" 한 가장 낮은 위치
  5. **Stacking 정확도 보정** — placement 후 visual check, 안 맞으면 미세 조정 또는 fail. Per-layer re-grounding 으로 다음 cycle 에 보정

- **운영 시나리오 v1 — Track A 결정사항** (sub-problem 의 구체화):

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

- **예상 실패 케이스 (이게 학습 자료)** — Track B 의 motivation 이 됨:

  - BLF 가 5DOF unreachable spot 선택 → IK fail
  - Greedy 가 "다음 박스 들어갈 자리 막는" placement → 후반 cycle fit 실패
  - 안정성 평가 = 지지면 ratio 만 → CoM 분석 없어서 가끔 무너짐
  - yaw 회전 박스의 4-fold 대칭 중 안 좋은 J5 각 선택 (prerequisite 있어도 정책이 약하면)
  - 직육면체 face 선택 정책이 약함 — "큰 면 아래" 가 stability optimal 이 아닌 경우 잡지 못함
  - 치수 측정 1-2mm 오차 → 가시적 흔들림 (ICP refinement 없음)

- **측정 항목** (Track A 의 실 output): cycle fail rate / IK fail rate / topple rate / packing utilization %. **이 정량 데이터 자체가 Track B 의 동기와 평가 기준**

---

### Track B — 정석 트랙 (proper 학습 stack)

각 항목 옆 → Track A 의 어느 실패 patterns 를 잡는지 mapping.

- **B.1 문제 정형화** — Online 3D BPP 의 state representation (height map / EMS / corner points / extreme points 비교) / action / reward / 제약 정식 작성. 부분관측이면 POMDP. paper exercise 지만 안 하면 다 surface
- **B.2 Exact methods** — MILP 모델링 (Tsai/Chen 류 변수/제약), **CP-SAT (Google OR-Tools — BPP 에 의외로 강함)**, branch-and-bound. n=5/7/10 시간 폭발 시점 측정 → 복잡도 직관. → 모든 Track A 실패의 upper bound reference
- **B.3 정식 constructive heuristics** — **Extreme Points (Crainic et al. 2008)**, DBLF, skyline algorithm, maximal-rectangles. Track A 의 BLF 보다 일반적인 후보 enumeration. → "다음 박스 자리 막힘"
- **B.4 메타휴리스틱** — GA (box-ordering chromosome + crossover), Tabu Search neighborhood, SA, GRASP. **Bortfeldt-Gehring** 의 classic palletizing reference
- **B.5 Search 기반** — MCTS (UCB1/PUCT exploration constant, rollout policy), beam search width tuning. **Lookahead depth 가 greedy 대비 어디서 break-even** 인지 측정. → "다음 자리 막힘", 직육면체 face 선택
- **B.6 DRL** — **Zhou et al. AAAI 2021 PackNet** replicate (constrained action space + feasibility mask + PPO). Zhao 2022 PCT, Attend2Pack, TAP-Net. **PyBullet sim env 구축 자체가 학습 가치**. → 위 다 통합 학습
- **B.7 안정성 정식** — CoM support polygon (정역학), **Stewart-Trinkle equilibrium** (friction cone 포함), force closure vs form closure 구분. PyBullet sim 과 정역학 cross-validate. Learned stability classifier (CNN on rendered scene) 옵션. → "지지면 ratio 만으로는 무너짐"
- **B.8 Grasp planning 정식** — **Force closure metric (Ferrari-Canny)**, antipodal grasp computation, 6DoF grasp synthesis (AnyGrasp / Contact-GraspNet). **5DOF reachable grasp manifold 를 Jacobian rank / manipulability ellipsoid 로 derive** — 5DOF 의 어떤 grasp 가 가능한지 정식으로. → "yaw 4-fold 안 좋은 선택"
- **B.9 Object pose 정식** — **ICP variants**: point-to-point / point-to-plane / **GICP (Generalized)** / Colored ICP / **GoICP (globally optimal)**. Oriented bbox + symmetry resolution 정식 알고리즘. 6DoF DL (FoundationPose, Megapose) 와 oriented bbox + Z-up assumption 의 trade-off 측정 — 5DOF 한정 marginal value 정량. → "치수 측정 오차"
- **B.10 Motion planning** — Cartesian L-move with orientation lock (SLERP), **CBiRRT (Constrained Bi-RRT)** — orientation manifold constrained sampling, TrajOpt / CHOMP / STOMP optimization-based. Manipulability-aware planning. → IK fail rate, "비스듬한 접근 모서리 닿음"

---

- **비교 metric — 두 트랙 평가 차원**:

  - 공간 효율 (utilization %) — B.2 exact / B.6 DRL 가장 빛남
  - 단일 cycle stability fail rate — B.7 안정성 정식 가장 빛남
  - 5층 누적 topple rate
  - IK fail rate (reachability) — B.10 가장 빛남
  - Cycle time (planning + 실행)
  - **알고리즘 결정 시간 vs 실행 오차 break-down** — sim/real gap, **DIY 한계의 정량적 답**

- **Step 구조**: **`PalletizeStep` primitive 신규**. 1 step 안에 cycle loop 캡슐화 + **트랙 선택 옵션** (`policy="heuristic"|"ep"|"mcts"|"milp"|"drl"`). LLM orchestrator 는 `[PalletizeStep(max_boxes=10, policy="heuristic")]` 만 짜면 됨.

- **3D 시각화 — 실시간 world model 레이어** (두 트랙 공통): 박스 사이즈가 작은 집합이라 [Workspace3D](../frontend/src/pages/Workspace3D.tsx) 안에 three.js `<boxGeometry>` 로 그릴 수 있음. URDF / PointCloudLayer / MeshLayer 옆에 **PalletizerLayer** 추가.

  - **토픽**: `omx/palletizer/state` (cycle 마다, ~5Hz)
    ```
    {
      boxes: [
        {id, dims:[L,W,H], pose:[x,y,z,qx,qy,qz,qw],
         state:"source"|"held"|"placed", color?}
      ],
      pallet: {origin:[x,y,z], size:[W,D]},
      next_placement?: {pose, dims, score?, policy?}   // 정책 후보 + 점수 (Track A vs B 비교 시각화)
    }
    ```
  - **프론트 레이어**: 팔레트 outline / `source`·`placed`·`held` 박스별 색 / `next_placement` dashed wireframe + score + policy 라벨
  - **Identity tracking**: dims similarity + 위치 근접으로 cycle 간 매칭. SLAM 불필요
  - **추가 효용**: (a) 측정/정책 버그 모션 전 catch, (b) **두 트랙 비교 visual evidence**, (c) 데모 가치 ↑

- **DIY 에서 진짜 발목 잡을 항목들** (두 트랙 공통):

  1. **Release dynamics** — 그리퍼 너무 높이서 열면 떨어지는 충격으로 stack 흔듦. **Dynamixel current spike 로 contact 검출 후 open**. XL430 OK, XL330 노이즈 ↑
  2. **5DOF 접근 각도** — top-down 은 base-arm 평면 안에서만 정확. **팔레트를 base 정면에 셋업** + IK reachability check (Track B.10 의 응용)
  3. **5DOF 수직 reach** — 5층 × 40mm + 팔레트 + hover ≈ 300mm. OMX_F reach 380mm. **셋업 시점 reach mask** 사전 enumerate (격자 × 높이 × yaw bin)
  4. **Gripper 개구 + orientation** — gripper max ~40mm. prerequisite candidate enumerator + B.8 grasp manifold 결합
  5. **Stack top 가림** — eye-in-hand 라 observation pose 비용 감수

- **권장 진행 순서** (curriculum):

  1. **선결 prerequisite — 회전 박스 grasp pipeline** (7개 wire-up) — 둘 다 시작 전 필수
  2. **Track A 베이스라인 + 측정 인프라** — 휴리스틱 BLF + sim/real 측정 hook (실패 patterns 분류 + 통계)
  3. **회전 큐브 baseline** — Track A 로 30mm 큐브 3층 (feed-forward), σ_t 실측 + 무너짐 patterns 데이터
  4. **Per-layer re-grounding** — 5층 도전 (Track A 유지)
  5. **직육면체 도입 (Track A)** — 가변 dim + face 선택. 실패 patterns 확장 측정
  6. **Track B 점진 도입 — 작은 것부터, 매 항목 후 Track A 와 sim/real ranking**:
     - **B.1 정형화** (paper exercise) — 후속 다 의존
     - **B.3 EP/DBLF** → Track A BLF 와 비교
     - **B.7 안정성 (정역학 + PyBullet)** → "지지면 ratio" 와 비교
     - **B.5 MCTS/beam** → greedy 와 비교
     - **B.10 reachability-aware motion** → IK fail rate 비교
     - **B.2 MILP / CP-SAT** — n=5/7/10 시간 측정, exact baseline 확보
     - **B.8 Grasp 정식** — yaw 4-fold 선택 quality
     - **B.9 ICP refinement** — 치수 정확도 gain
     - **B.6 DRL** — 가장 후순위, prerequisite (sim env + reward + 학습) 큼
  7. **Sim2real gap 분석** — 각 알고리즘이 σ_t 7.94mm 노이즈에 얼마나 robust 한지. **DIY 한계의 정량적 답** — 이게 study output 의 메인

- **리스크 / 트레이드오프**:

  - **치수 측정 오차** — 1-2mm → 가시 흔들림. B.9 ICP refinement 로 잡힘
  - **Cycle time** — observation pose + re-detection + planning. Track A ~10-15초, B.2 MILP / B.5 MCTS 추가 비용 측정 필요
  - **DRL prerequisite 비용** — sim env / reward design / 1-2일 학습. 후순위 이유
  - **Track B 모든 항목 = 정석 implementation 자체에 학습 시간** — paper 1-2개 읽고 구현 단위. study 가 목적이라 이게 비용이 아니라 곧 output

- **양보 못 하는 두 제약**: **(a) 모든 변 ≥30mm** **(b) per-layer visual re-grounding**

- **의존성**:
  - **공통**: 포인트클라우드 / TSDF 인프라 (있음). SearchAndDetect enumerate 모드. PalletizeStep. observation pose 정의. Current-spike contact detect. 셋업 reach mask 사전 계산
  - **선결**: prerequisite 7개 wire-up. 그중 #2 (`Position3` → `Pose6`) + Step DSL 전반 정리는 별도 entry **Step DSL 레고화** 로 격상됨 — Palletizing 본격 시작 전에 그 작업 완료 필요. 나머지 6개 wire-up (Detector / Motion API / Grasp enumerator / Reach filter / Descent / Pick yaw) 도 Step DSL 레고화 작업 안에 흡수해서 같이 처리.
  - **Track A**: greedy BLF + height map + 측정 hook (~수백 줄)
  - **Track B**: B.1-B.10 각각 수일~수주. PyBullet sim env (B.6, B.7 공유). OR-Tools (B.2). Open3D ICP 변종 (B.9). 모듈별 paper reference 별도 정리 가치

---

## Step DSL 레고화 (★ 유력, Palletizing/LLM 양쪽 prereq)

- **한 줄**: 현 step 들이 너무 거대 + 데이터 자유 dict + 동적 흐름 표현 X 라 **조립이 안 됨**. 작은 primitive + typed schema + 흐름 제어 step 으로 리팩토링.

- **왜**: LLM 무관하게 **Python 코드로 새 task 짤 때부터 building block 재사용 안 됨**. [pick_and_place.py](../backend/modules/task/tasks/pick_and_place.py) 의 pick 단계 (hover→descent→grip→lift) 가 매크로 안에 묻혀있어 Palletizing 의 pick 에 그대로 재사용 X. 새 task 마다 비슷한 큰 매크로 또 만드는 패턴 반복. **task-agnostic 인프라 부채**. Palletizing 의 "선결 prerequisite — 회전 박스 grasp pipeline" 7개 wire-up 중 #2 (`Position3` → `Pose6`) 가 이 부채를 정통으로 건드림 → 두 작업 묶어서 한 번에 손봄.

- **근본 원인 3개**:

  1. **Step 이 너무 거대** — `SearchAndDetectStep` 안에 "여러 자세 순회 + 검출 + 첫 성공 break" 한 덩어리. 그중 하나만 못 씀 (search pose 이동 + 다른 검출, 다른 break 조건 등).
  2. **데이터가 자유 dict** — `context.data: dict` 에 키 이름 의존. step A 의 `output_key` 가 step B 의 `input_key` 와 type 맞는지 run-time 까지 모름. 새 task 짤 때 무슨 키가 어떤 형태인지 외워야 함.
  3. **동적 흐름 표현 수단 없음** — "검출 결과 보고 다음 step 결정" 같은 분기/반복이 `list[Step]` 에 표현 X. 매크로로 우회 → #1 의 원인.

- **작업 방향**:

  | 문제            | 해결                                                                                                                       |
  | --------------- | -------------------------------------------------------------------------------------------------------------------------- |
  | 1. step 이 거대 | 작은 primitive 분해 (예: `SearchAndDetectStep` → `MoveToPoseStep` + `GroundedDetectStep`)                                  |
  | 2. dict 자유    | typed schema (`Position3` / `Pose6` / `Detection` 클래스). step 입출력 타입 명시. plan-time 에 reference / type validation |
  | 3. 동적 흐름 X  | `ForEachStep` / `BreakOnSuccessStep` 같은 흐름 제어 step 추가. TaskRunner 가 runtime 에 unroll                             |
  | 매크로 보존     | 자주 쓰는 조합은 **recipe 함수** 로 유지. 내부적으로 primitive 조합 반환. 편의 + 기존 task regression                      |

- **작업 범위 (~4-5주)**:

  1. `Pose6` / `Detection` / `ObjectMeta` 등 typed schema 정의 + `TaskContext` 가 typed accessor 노출
  2. 매크로 step 분해 (primitive + 흐름 제어 step). 기존 11개 step 마이그레이션
  3. **회전 박스 wire-up 7개 (Palletizing prerequisite) 같이** — `Position3` → `Pose6` 확장과 통합. Detector segmentation + cluster decomposition / Motion API quaternion / Grasp candidate enumerator + J5 closest-arc / Reachability filter / Orientation-lock descent / Placement-aware pick yaw
  4. 기존 task ([pick_and_place](../backend/modules/task/tasks/pick_and_place.py)) regression — 새 구조로 동등 동작
  5. 매크로 → recipe 함수 (편의용, 내부는 primitive 조합)

- **LLM 도입 시점에 추가 작업** (지금은 안 함):

  - 흐름 제어 step 의 LLM 노출 + system prompt few-shot
  - JSON schema 자동 추출 (typed schema 로부터 dataclass reflection)
  - LLM orchestrator 섹션의 "4축" 중 **축 1 (Control flow 위치)** / **축 4 (비용 모델)** 본격 결정. 본 작업에서 **축 2 (typed schema)** / **축 3 (primitive 입도)** 는 일부 진행.

- **리스크 / 트레이드오프**:

  - 기존 11개 step 마이그레이션 — TaskRunner / Workspace3D 시각화 / 토픽 스키마 영향. regression suite 깔아둬야 함
  - 흐름 제어 step (`ForEachStep`) 가 LLM 무관해도 도입 정당한가? Python 코드면 `for` 가 자연스럽지만, TaskRunner 가 step list 만 받는 구조 + UI 시각화 (`omx/task/state`) + 진행 추적 위해 step 단위 명시 필요 → 도입 정당

- **의존성**:
  - Palletizing 의 "선결 prerequisite — 회전 박스 grasp pipeline" 의 #2 (`Position3` → `Pose6`) 를 이 작업에 흡수. **Palletizing 본격 시작 전에 이 entry 가 먼저 완료** 되어야 함
  - LLM orchestrator 섹션의 "결정 보류 — 4축" 중 축 1, 축 4 는 LLM 도입 시점에 결정. 이 작업에서는 축 2, 축 3 부분 진행

---

## LLM task orchestrator (★ 유력)

- **한 줄**: 자연어 task 요청 → Local LLM 이 step list 를 JSON 으로 생성 → 기존 TaskRunner 가 실행.
- **왜**: 지금 [pick_and_place.py](../backend/modules/task/tasks/pick_and_place.py) 같은 task 는 `(pick_object, place_object)` 2-슬롯 템플릿이라 "큐브 다 박스에" (loop), "빨간 건 X, 파란 건 Y" (분기), "책상 정리해줘" (목표 추상화), 이종 task 연결 등이 안 됨. step primitive 는 이미 [step_types.py](../backend/modules/task/step_types.py) 에 11개 있고 충분히 표현력 있는데, **조합을 사람이 코드로 짜야** 한다는 게 병목.
- **어떻게**:

  1. system prompt 에 11개 primitive 문법 + few-shot 2-3개 박기 (pick_and_place 시퀀스 그대로 정답 예시).
  2. JSON list → Step[] 디시리얼라이저 (~30줄, `type` literal 을 dispatch key 로).
  3. [prompt_parser.py](../backend/modules/llm/prompt_parser.py) 옆에 `task_planner.py` — 같은 모델/lock 재사용, 출력 스키마만 다름.
  4. TaskRunner 진입점 하나 추가 — 자연어 prompt → planner → Task → 실행.
  5. **Dry-run preview UI** — 프론트 `PromptPanel` 에 LLM 이 짠 step list 먼저 띄우고 사용자 confirm 해야 실행. 환각 1번이 충돌로 이어지는 거 막음.

- **Step primitive 입도 재검토 (선결 과제)**:

  - 현재 step 들 중 `SearchAndDetectStep` 은 "search pose 순회 + grounded_detect + 첫 성공 break" 를 하드코딩한 **매크로**. LLM 입장에선 호출은 쉽지만 **조합 자유도 없음** — search 하면서 detect 외 다른 행동 끼우거나, 다른 break 조건 쓰거나 못함.
  - 동적 step 조합을 진짜로 하려면 search pose 이동 / detect / loop / break-on-success 가 **별개 primitive** 여야 함. 현재 `list[Step]` 은 flat — 분기/루프 표현 수단 자체가 없어서 매크로 step 으로 우회 중인 상태.
  - 두 방향:
    - **순수 primitive + control flow** (`MoveToSearchPoseStep`, `GroundedDetectStep`, `ForEachStep`, `BreakOnSuccessStep`, `IfStep`): 표현력 ↑, 새 task 패턴마다 클래스 추가 X. 단점은 LLM 이 루프 조건/탈출/context 키 일관성 잘못 짜면 무한루프나 빈 context 참조로 task 전체 깨짐 — 환각 면적 ↑.
    - **매크로 위주 유지**: 환각 면적 작음, 검증된 시퀀스 보존. 새 패턴마다 클래스 추가 필요해서 확장성 ↓, LLM 은 그냥 매크로 dispatcher 역할.
  - **권장: 두 층 공존**. (a) primitive (`MoveToSearchPoseStep`, `GroundedDetectStep`, `ForEachStep`, `BreakOnSuccessStep`) 를 LLM 에 노출 + (b) 검증된 composite "recipe" step (`SearchAndDetectStep`, `PalletizeStep` 등) 도 유지. LLM 은 단순/잘 알려진 task 는 recipe 호출, 새로운 조합이 필요할 땐 primitive 직접 조합. system prompt 에 "가능하면 recipe 우선, 안 맞으면 primitive 조합" 가이드.
  - **선결 검증**: 기존 매크로 (`SearchAndDetectStep`) 가 primitive 조합 (`ForEach(search_poses) { GroundedDetect; BreakOnSuccess }`) 으로 동일하게 재현되는지 regression. 둘 다 유지하려면 의미 일치가 전제.

- **결정 보류 — primitive 깎기 전 픽스해야 할 4개 축** (다음 세션에서 토론):

  네 축은 직교 — 각각 독립으로 결정되지만 합쳐서 step DSL 의 전체 모양을 정함. 결정 없이 "일단 primitive 클래스 1개 짜보자" 들어가면 그게 default 가 돼서 후회. 4개 축에 결론 박힌 후 코드 시작.

  **축 1. Control flow 가 어디에 사는가 — 가장 큰 fork**:

  - (A) **step DSL 안** — LLM 이 `ForEachStep` / `IfStep` / `BreakOnSuccessStep` 포함된 plan 을 1회 emit, TaskRunner 가 그대로 실행. LLM 호출 1번.
  - (B) **planner 재호출 loop — ReAct 식** — LLM 이 짧은 sequence 만 emit → 실행 → 결과 (context, observation) 보고 다음 sequence emit. step DSL 은 sequential only, 분기/루프는 "다음 emit" 으로 표현.
  - (C) **하이브리드** — sequence + 명시적 `ReplanStep` 으로 일정 지점에서만 planner 재호출.
  - 트레이드오프: (A) 표현력 ↑ + 1회 호출이지만 환각 면적 ↑ + "루프 어디서 잘못됐나" 디버깅 지옥. (B) 환각 회복 쉬움 + 작은 모델 OK + environment 가 변하는 task 에 강함, 단 LLM latency 누적 + planner-runner protocol 새로 정의. (C) 균형 but 체크포인트 위치 결정이 새 골치.
  - **이걸 먼저 정해야 나머지 축 풀림**: (A) 면 `ForEach`/`If`/`Break` primitive 자체를 설계해야 함. (B) 면 control flow primitive 가 아예 불필요해지고 대신 "어떤 시점에 planner 다시 부를지" 의 runner ↔ planner 프로토콜이 핵심.

  **축 2. Context 타입화 수준**:

  - 현재 [TaskContext.data: dict](../backend/modules/task/step_types.py#L191-L204) — 자유분방. `MoveTCPStep.position_key` 가 가리키는 값이 실제로 `Position3` 인지 run-time 까진 모름. fragility 의 원천.
  - 옵션:
    - (a) **typed schema** — `Position3` / `Pose6` / `Detection` / `ObjectMeta` 등 타입 정의 + 각 step 의 입출력 키 타입 선언. plan-time 에 reference + type validation.
    - (b) **dict 유지 + LLM 출력만 schema validate** — context 흐름은 그대로 자유.
    - (c) **Pydantic 전구간** — 데이터 흐름 자체를 타입 객체.
  - 트레이드오프: (a)/(c) 가 LLM 환각의 큰 부분 (없는 키 / 타입 mismatch) 을 run-time 전에 reject 가능. 비용 = 타입 정의 유지 + 새 데이터 추가 시 schema 등록 절차.
  - 답이 어느 정도 (a) 방향으로 정해진 축이지만, **어디까지 타입 박을지 (모든 키 vs hot path 만)** 는 결정 필요.

  **축 3. Primitive 입도 기준선 — "verb 1개" 의 정의**:

  - 현 step 입도 제각각: `MoveTCPStep` (한 동작), `SearchAndDetectStep` (순회 + 검출 + break), `GraspPolicyStep` (policy 계산만, motion X), `PalletizeStep` (전체 cycle). 일관 기준 없이 case-by-case 로 추가돼옴.
  - 기준 후보:
    - (a) "한 번의 motion 또는 한 번의 perception 또는 한 번의 context write — 셋 중 하나만"
    - (b) "외부 부수효과 (motor 명령 / 카메라 capture) 가 1회 이하"
    - (c) "기대 실행 시간 ≤ N초"
    - 또는 위 조합.
  - 트레이드오프: 엄격한 기준 = primitive 수 ↑ + LLM 이 짜야 할 sequence 길이 ↑ + 환각 면적 ↑. 느슨한 기준 = 지금처럼 case-by-case 매크로화 재발 (= 이 섹션의 우려 그대로).
  - 결정되면 기존 step (`SearchAndDetect` / `GraspPolicy` / `PlacePolicy` / `Palletize`) 의 재분류 또는 분해 가이드가 자동으로 따라옴.

  **축 4. 새 primitive 추가 비용 모델**:

  - primitive 1개 추가할 때 필요한 작업: (i) 클래스, (ii) executor handler, (iii) LLM system prompt schema, (iv) few-shot 예시, (v) 조합 regression — 어디까지 자동 / 어디까지 손?
  - 비용 모델이 안 좋으면 "이번 한 번만 매크로로" 패턴이 반복돼서 결국 매크로화로 회귀. 즉 축 3 의 결정이 무력화됨.
  - 결정 항목: schema 자동 생성 (dataclass → JSON schema reflection 가능한가?), few-shot 자동/수동, regression suite 구조.

  **토론 시작점 — 현 코드 앵커**:

  - [step_types.py](../backend/modules/task/step_types.py) — 현재 11개 step 정의 + `TaskContext`. 입도 비교 baseline.
  - [step_executor.py](../backend/modules/task/step_executor.py) — step → handler dispatch. 축 3 결정 시 "어디까지 쪼개야 자연스러운지" 가 코드에서 보임.
  - [task_runner.py](../backend/modules/task/task_runner.py) — 현재 sequential 실행만, control flow 없음. 축 1 옵션 (A) 가면 손볼 게 큼, (B) 면 planner 호출 hook 만 추가.
  - [tasks/pick_and_place.py](../backend/modules/task/tasks/pick_and_place.py) — 현 task DSL 의 가장 정직한 사용 예. 축 2/3 의 "지금 얼마나 자유분방한지" 가 보임.
  - [prompt_parser.py](../backend/modules/llm/prompt_parser.py) — 이미 굴러가는 LLM 인프라 (모델 lock / preload / JSON 파싱 / fallback). planner 는 이 위에 얹음.

- **리스크 / 트레이드오프**:
  - Qwen2.5-1.5B 는 2-슬롯 추출은 잘 하지만 15-step JSON 의 `input_key`/`output_key` 일관 체이닝 (e.g. `SearchAndDetect.output_key="pick"` ↔ `GraspPolicy.input_key="pick"`) 은 환각 잦을 가능성. 안 되면 Qwen2.5-3B / Phi-3.5-mini 로 올림 (RTX 3060 가능).
  - 환각 방지는 `type` literal whitelist + 참조 검증 (없는 키 참조 시 reject + 재호출).
  - **매크로 vs primitive 입도** 결정이 환각 면적 / 표현력 직접 트레이드오프 — primitive 만 가면 LLM 이 루프/탈출 잘못 짜서 무한루프, 매크로만 가면 모든 새 패턴이 코드 추가.
- **의존성**: 이미 [prompt_parser.py](../backend/modules/llm/prompt_parser.py) 인프라 (모델 로드/lock/preload/JSON 파싱/fallback) 굴러감. 추가 작업 적음.

---

## Auto-scanning (Next-Best-View)

- **한 줄**: TSDF 캡처 자세를 사람이 클릭하지 않고 로봇이 알아서 다음 best view 로 이동 → 캡처 반복.
- **왜**: 지금 scan 자세는 사용자가 매번 수동 지정. 워크스페이스 전체 mesh 빌드가 노동집약.
- **어떻게**:
  1. 현재까지 빌드된 partial TSDF 에서 **information gain** 큰 viewpoint 산출 (frontier voxel / unseen surface 비율 기반).
  2. PyBullet 으로 후보 자세 IK 검증 + self-collision check.
  3. `POINTCLOUD_CAPTURE` 자동 트리거 → 충분히 수렴할 때까지 loop.
- **리스크 / 트레이드오프**:
  - NBV scoring 이 너무 단순하면 같은 자리 맴돌거나, 너무 정교하면 계산 비싸짐.
  - 워크스페이스 reach 한계 (5DOF) 로 도달 불가 자세가 많을 수 있음.
- **의존성**: 이미 [tsdf_pipeline.md](tsdf_pipeline.md) + [pointcloud_node.py](../backend/nodes/pointcloud_node.py) 인프라 다 있음. NBV scorer 만 추가.

---

## Kinesthetic teaching (drag-to-record)

- **한 줄**: XL430/330 을 current-control 모드로 풀어서 사람이 손으로 팔 끌고 다니면 trajectory 녹화, replay 가능.
- **왜**: (a) 자체로 데모 인터페이스. (b) 미래 imitation learning (Diffusion Policy / ACT / VLA) 의 **데이터 수집 인프라** 로 자연스럽게 연결.
- **어떻게**:
  1. Dynamixel current-based control 모드 진입 + 중력 보상 토크 미세 출력.
  2. 모터 상태 + 카메라 frame + gripper state 를 sync 해서 episode 로 저장 (.npz 또는 zarr).
  3. Replay 는 기존 TrajectoryRunner 재사용.
- **리스크 / 트레이드오프**:
  - XL330 토크 약해서 중력 보상 정밀도 한계. 어깨 무거우면 처짐.
  - Episode label/메타데이터 스키마 결정 필요.
- **의존성**: Dynamixel current control 모드 진입 검증. 중력 보상 모델 (URDF 기반 RNE) 구현.

---

## 6DoF Grasp prediction (AnyGrasp / Contact-GraspNet)

- **한 줄**: 포인트클라우드 → 6DoF grasp pose 예측 모델로 "centroid + top-down" 가정을 대체.
- **왜**: 지금 detector 는 centroid + Z=0 평면 가정 + 옆면 정책으로 grasp 위치 산출. 누운 병, 손잡이, 클러터, 미학습 객체엔 무력.
- **어떻게**:
  1. 라이브 포인트클라우드 stream → AnyGrasp / Contact-GraspNet inference.
  2. 6DoF grasp 후보들 중 reachable + collision-free 한 거 선택 (PyBullet 검증).
  3. `GraspPolicyStep` 을 정책 기반 → 모델 기반으로 교체 또는 병행.
- **리스크 / 트레이드오프**:
  - 모델 크기 / 추론 비용 (RTX 3060 가능한지 확인 필요).
  - σ_t 7.94mm 천장 때문에 작은 손잡이/얇은 부품은 여전히 안 됨. 박스/병/도구 같은 관대한 grasp 이 현실적.
- **의존성**: 포인트클라우드 pipeline 이미 굴러감. 모델 라이선스 + 가중치 확인.

---

## Closed-loop visual servoing

- **한 줄**: 마지막 approach 단계에서 카메라 픽셀 오차를 직접 줄이는 방향으로 in-loop 보정.
- **왜**: σ_t 7.94mm 천장은 캘 정확도 한계인데, visual servoing 은 캘에 의존 안 하고 픽셀 오차를 minimize 하므로 **하드웨어 천장을 우회**할 수 있음. 지금은 open-loop (detect → plan → 실행).
- **어떻게**:
  1. Approach 단계에서 30Hz 정도로 target object 재검출 → 픽셀 오차 → Image-Based Visual Servoing (IBVS) Jacobian → joint vel command.
  2. PyBullet IK / Ruckig 위에 얹는 게 아니라 별도 reactive controller.
- **리스크 / 트레이드오프**:
  - 5DOF + jerk-limited 트레이지 위에서 reactive 제어 안정성 튜닝 어려울 수 있음.
  - 검출 latency / FOV 한계 (eye-in-hand 라 가까이 가면 target 시야 밖).
- **의존성**: 안정적 30Hz+ 검출 (현재 5fps stream 으론 부족).

---

## Drawing / 필기 (와일드카드)

- **한 줄**: 그리퍼에 펜 물리고 입력 텍스트/스케치를 평면에 그리기.
- **왜**: 새 motion 도메인 — 지금까지 다 점-to-점 manipulation 인데 이건 연속 contact-rich. 데모 영상 보기 좋음, ROI 는 fun.
- **어떻게**:
  1. 펜 그리퍼 어태치먼트 + 종이 평면 캘리브레이션 (3-point touch).
  2. 입력 텍스트 → stroke path (vector font / SVG) → MoveL waypoints.
  3. Z 압력은 펜 스프링 / 수동 force compliance.
- **리스크 / 트레이드오프**:
  - σ_t 7.94mm 천장 안에서 필기 가독성 확인 필요 (한글 < 영어 < 도형).
  - contact force control 없으니 종이/펜에 따라 결과 편차.
- **의존성**: 없음. 단순히 새 task 추가.

---

## 미래 방향: Imitation Learning (Diffusion Policy / ACT / VLA)

- **한 줄**: (image stream, language, action) 페어 데이터셋으로 end-to-end 정책 학습. step 스크립트 자체 제거.
- **왜**: 트렌드. 하지만 첫 발로 적합하지 않음.
- **어떻게**: ACT / Diffusion Policy 부터 — 50M~100M, 자기 로봇만 학습, 데모 30-50개로 시작 가능. 작동하면 OpenVLA-OFT / π0-FAST 같은 VLA fine-tuning 으로 확장.
- **리스크 / 트레이드오프**:
  - VLA 들은 Franka/UR5/ALOHA pretrain 이라 OMX_F (5DOF) 는 분포 밖. fine-tune 필수.
  - 5DOF action head 차원 mismatch — 모델 수술 필요.
  - 추론 latency (OpenVLA-7B 기준 ~150ms/액션) 와 100Hz 제어 결합 설계.
  - **실제 병목은 모델 선택이 아니라 데모 수집 인프라**. 그래서 kinesthetic teaching 이 선행.
- **의존성**: kinesthetic teaching (위) 가 먼저 깔려야 함. 그 위에서 자연스럽게 ACT → VLA.
