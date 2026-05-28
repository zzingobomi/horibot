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
- **왜**: 산업용 팔레타이저 흉내. 단일 픽앤플레이스의 N회 반복이 아니라 **이전 placement 가 다음 placement 의 가능 공간을 바꾸는** closed-loop 상태 추론이 핵심. 큐브 (변 다 같음) 대신 **직육면체 (가로/세로/높이 가변)** 로 가면 orientation 결정이 정책에 추가돼서 진짜 팔레타이저 사고에 가까워짐. 지금까지 만든 자산(detection / TSDF / GraspPolicy / TaskRunner) 거의 다 한 task 에 끌어와서 통합 데모로도 좋음.
- **박스 사이즈 spec**:
  - 모든 변 ∈ **[30, 50]mm**
  - 각 박스에 **≥1개 변은 ≤40mm** (그리퍼 개구 호환 — 잡을 수 있는 방향 최소 1개 보장)
  - 5-10개 박스, 사이즈 mix
  - 어떤 면을 아래로 하든 contact 변이 ≥30mm → 5층 누적 ~78% 성공률 유지
- **어떻게** — sub-problem 5개:
  1. **박스 enumerate + 치수 측정** — `SearchAndDetect` 에 `enumerate_all=True` 옵션 추가해서 모든 박스 bbox 반환. 가로/세로/높이 다 알아야 하므로 TSDF / 포인트클라우드 한 번 스캔 후 박스별 segmentation + Open3D oriented bbox 추출.
  2. **팔레트 상태 모델** — **height map** (격자별 현재 stack 높이). 팔레트가 **고정 위치**라 `pallet_origin_xyz` + `pallet_size` 하드코딩, height-map 좌표계 즉시 확정. cycle 마다 박스 검출만 다시.
  3. **Selection + orientation policy** — 산업 표준 = **큰 거부터 + flat 하게 (제일 큰 면을 아래로)**. 안정성 + 공간 효율. 작은 거는 그 위에 LIFO.
  4. **Placement policy** — 2D bin packing 단순화. 각 박스의 선택된 orientation 의 footprint 에 대해 height map 위에서 "지지면 충분 (baseline 의 N% 이상이 같은 높이 위) + reach 가능 + 수직 접근 가능" 한 가장 낮은 위치를 greedy bottom-left-fill.
  5. **Stacking 정확도 보정** — placement 후 visual check step. 안 맞으면 미세 조정 또는 fail.
- **Step 구조**: **`PalletizeStep` primitive 신규**. 1 step 안에 위 5개 로직 캡슐화 + cycle loop. LLM orchestrator 와 직교 — orchestrator 는 "박스들 팔레트에 쌓아" → `[PalletizeStep(max_boxes=10)]` 만 짜면 됨 (50-step 시퀀스를 LLM 이 한 번에 짜면 환각 위험 큼).

- **3D 시각화 — 실시간 world model 레이어**: 박스 사이즈가 작은 집합이라 카메라로 인식한 직육면체를 [Workspace3D](../frontend/src/pages/Workspace3D.tsx) 안에 three.js `<boxGeometry>` 로 그대로 그릴 수 있음. 기존 URDF / PointCloudLayer / MeshLayer 옆에 **PalletizerLayer** 1개 추가.

  - **토픽**: `omx/palletizer/state` (cycle 마다 publish, ~5Hz)
    ```
    {
      boxes: [
        {id, dims:[L,W,H], pose:[x,y,z,qx,qy,qz,qw],
         state:"source"|"held"|"placed", color?}
      ],
      pallet: {origin:[x,y,z], size:[W,D]},
      next_placement?: {pose, dims}   // 정책이 검토 중인 후보 (dashed wireframe)
    }
    ```
  - **프론트 레이어**:
    - 팔레트: 평면 outline
    - `source` 박스: 작업대 위 (검출 결과 위치), 색 A
    - `placed` 박스: 팔레트 위, 색 B, cycle 진행하며 누적
    - `held` 박스: 현재 EE 에 부착 (TCP 행렬 곱 transform), 색 C — pick/place 도중 어디 있는지 보임
    - `next_placement`: dashed wireframe — 정책이 왜 거기로 가는지 시각화
  - **Identity tracking**: 5-10개 + 사이즈 mix 라 **dims similarity + 위치 근접** 으로 cycle 간 매칭하면 충분. SLAM 까진 불필요. `placed` 상태 진입 후엔 카메라 가림 무관 위치 고정.
  - **추가 효용**: (a) 측정/정책 버그를 모션 전에 시각적 catch, (b) 데모 가치 ↑ (인식→추론→실행 한 화면).
- **Feasibility 검토 (목표 ≥5층)** — 결론: **조건부 가능**. 큐브 ≥30mm + per-layer re-grounding 아키텍처면 데모 수준 동작 기대.

  큐브 사이즈 vs 5층 누적 성공률 (σ_t 7.94mm 가우시안 가정, CoM이 아래 큐브 contact polygon 안에 들어와야 정적 안정):

  | 큐브 변 | 허용 오차 (반변) | 단일 placement 실패율 | 5층 누적 성공률 |
  |---|---|---|---|
  | 20mm | ±10mm | ~16% | ~50% (코인플립) |
  | 30mm | ±15mm | ~6% | ~78% (가끔 무너짐) |
  | 40mm | ±20mm | ~1.2% | ~95% (안정) |

  ⇒ **20mm 는 5층 데모로 부적합. ≥30mm 필수, sweet spot 30mm** (gripper 최대 개구 ~40mm 와 호환).

- **누적 오차 신화 — per-layer re-grounding 이 아키텍처 단일 결정점**:
  - **Feed-forward (재검출 X)**: 5층 후 누적 ~30mm → 거의 항상 무너짐.
  - **매 cycle 카메라로 stack top 재검출** → 각 layer placement error 가 **독립** → 5층 후 총 오차 = σ_t (~8mm) 수준. **이게 5층 가능/불가능을 가르는 단일 아키텍처 결정**.
  - 구현: height-map 을 매 placement 직전 카메라 capture 로 refresh. 필요시 전용 **observation pose** (stack 잘 보이는 자세) 한 번 들리고 placement.

- **DIY 에서 진짜 발목 잡을 항목들**:
  1. **Release dynamics** — 그리퍼 너무 높이서 열면 떨어지는 충격으로 stack 흔듦. 해법: Dynamixel **current spike** 로 contact 검출 후 open. XL430 OK, XL330 은 노이즈 ↑.
  2. **5DOF 접근 각도** — top-down 은 base-arm 평면 안에서만 정확. off-axis spot 은 비스듬한 접근 → 모서리부터 닿아 회전 모멘트. 대응: **팔레트를 base 정면에 셋업** (고정 위치라 한 번만 잡으면 됨), IK reachability check 로 수직 접근 불가 spot 은 placement 후보에서 제외.
  3. **5DOF 수직 reach** — 5층 × 40mm = 200mm + 팔레트 base + hover ≈ 300mm. OMX_F reach 380mm 안이지만 빠듯. 팔레트 고정이라 **셋업 시점 1회 reach 전수 검증** 가능 (모든 격자 × 가능 높이에 대해 IK 풀어두고 placement 후보 마스크 만들어두기).
  4. **Gripper 개구 + orientation 선택** — OMX gripper 최대 ~40mm. spec 상 각 박스에 ≤40mm 변이 ≥1개 보장돼 있으므로 **잡을 방향만 옳게 선택**하면 항상 가능. 정책이 grip orientation 도 같이 결정해야 함.
  5. **Stack top 가림** — eye-in-hand 라 자세에 따라 stack 부분 가려짐. observation pose 비용 감수.

- **권장 진행 순서**:
  1. **Sanity check** — 동일 30mm 큐브 3층, feed-forward 만. σ_t 실측 + 무너짐 패턴 확인. 한 시간 안에 데이터.
  2. **Per-layer re-grounding 도입** — height-map cycle refresh + observation pose. 동일 큐브 5층 도전.
  3. **Release dynamics 튜닝** — current-spike contact detect.
  4. **직육면체 (가변 dim) 도입** — selection (큰 거부터 + flat) + orientation 결정 + placement (greedy BLF) policy.

- **리스크 / 트레이드오프** (위에서 다룬 것 외 잔여):
  - **치수 측정 오차** — 1-2mm 틀리면 가시적 흔들림. TSDF 스캔 oriented bbox 정밀도 검증 필요.
  - **Cycle time** — observation pose 들리는 비용 + per-layer re-detection 으로 한 cycle 10-15초 예상. 5층 데모 = 1분 정도. 받아들일 만함.

- **양보 못 하는 두 제약** (요약): **(a) 모든 변 ≥30mm** **(b) per-layer visual re-grounding**.

- **의존성**: 포인트클라우드 / TSDF 인프라 (있음). SearchAndDetect enumerate 모드 추가. 신규 모듈 = height-map + palletizer (selection / orientation / placement policy) + PalletizeStep + observation pose 정의 + current-spike contact detect + 셋업 시점 reach 마스크 사전 계산.

---

## LLM task orchestrator (★ 유력)

- **한 줄**: 자연어 task 요청 → Local LLM 이 step list 를 JSON 으로 생성 → 기존 TaskRunner 가 실행.
- **왜**: 지금 [pick_and_place.py](../backend/modules/task/tasks/pick_and_place.py) 같은 task 는 `(pick_object, place_object)` 2-슬롯 템플릿이라 "큐브 다 박스에" (loop), "빨간 건 X, 파란 건 Y" (분기), "책상 정리해줘" (목표 추상화), 이종 task 연결 등이 안 됨. step primitive 는 이미 [step_types.py](../backend/modules/task/step_types.py) 에 11개 있고 충분히 표현력 있는데, **조합을 사람이 코드로 짜야** 한다는 게 병목.
- **어떻게**:
  1. system prompt 에 11개 primitive 문법 + few-shot 2-3개 박기 (pick_and_place 시퀀스 그대로 정답 예시).
  2. JSON list → Step[] 디시리얼라이저 (~30줄, `type` literal 을 dispatch key 로).
  3. [prompt_parser.py](../backend/modules/llm/prompt_parser.py) 옆에 `task_planner.py` — 같은 모델/lock 재사용, 출력 스키마만 다름.
  4. TaskRunner 진입점 하나 추가 — 자연어 prompt → planner → Task → 실행.
  5. (필요시) `ForEach` / `If` primitive 추가하면 표현력 점프.
  6. **Dry-run preview UI** — 프론트 `PromptPanel` 에 LLM 이 짠 step list 먼저 띄우고 사용자 confirm 해야 실행. 환각 1번이 충돌로 이어지는 거 막음.
- **리스크 / 트레이드오프**:
  - Qwen2.5-1.5B 는 2-슬롯 추출은 잘 하지만 15-step JSON 의 `input_key`/`output_key` 일관 체이닝 (e.g. `SearchAndDetect.output_key="pick"` ↔ `GraspPolicy.input_key="pick"`) 은 환각 잦을 가능성. 안 되면 Qwen2.5-3B / Phi-3.5-mini 로 올림 (RTX 3060 가능).
  - 환각 방지는 `type` literal whitelist + 참조 검증 (없는 키 참조 시 reject + 재호출).
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
