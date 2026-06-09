# Ideas

새 기능/방향 아이디어 모음. roadmap.md 는 "할 거"이고 여기는 "할까 말까 — 검토 대기" 버킷. 채택되면 roadmap 으로 옮기거나 별도 design 문서로 승격.

각 항목 포맷:

- **한 줄**: 뭘 하자는 거
- **왜**: 동기 / 지금 못 하는 것
- **어떻게**: 대략적 구현 스케치
- **리스크 / 트레이드오프**
- **의존성**: 선행으로 깔려야 하는 인프라

---

## Palletizing — 다양한 크기 직육면체 쌓기

승격됨 — 별도 design 문서로 분리: [docs/random_palletizing.md](random_palletizing.md)

박스 spec / 선결 prerequisite / Track A (휴리스틱) / Track B (정석) / Track C (iterative sim2real RL) / curriculum 전부 거기서 다룬다.

---

## LLM task orchestrator (★ 유력)

- **한 줄**: 자연어 task 요청 → Local LLM 이 step list 를 JSON 으로 생성 → 기존 TaskRunner 가 실행.

- **왜**: 지금 [pick_and_place.py](../backend/modules/task/tasks/pick_and_place.py) 같은 task 는 `(pick_object, place_object)` 2-슬롯 템플릿이라 "큐브 다 박스에" (loop), "빨간 건 X, 파란 건 Y" (분기), "책상 정리해줘" (목표 추상화), 이종 task 연결 등이 안 됨. **Step DSL lego refactor 완료** ([step_dsl.md](step_dsl.md)) 로 primitive + recipe + control flow (`ForEach`/`Try`/`BreakIf`) 다 정리돼서 LLM 이 조립할 토대 준비됨. 남은 일은 **plumbing** (LLM 출력 → Task 변환 + schema 자동 추출 + dry-run preview).

- **어떻게** (작업 list):

  1. system prompt — primitive + recipe 카탈로그 (`step_dsl.md` 의 표 그대로 활용) + few-shot 2-3개 (pick_and_place 정답 plan)
  2. dataclass → JSON schema 자동 reflection (~50줄). [step.py](../backend/modules/task/step.py) 의 `step_to_dict` 가 출력 schema 이미 정의 — 그 역방향 deserializer
  3. Slot 참조 JSON 표기 결정 (`{"$ref": "step-xxx"}` 같은) + plan-time 검증 (잘못된 step_id 참조 reject)
  4. [prompt_parser.py](../backend/modules/llm/prompt_parser.py) 옆에 `task_planner.py` — 같은 모델/lock 재사용, 출력 스키마만 다름
  5. TaskNode 진입점 추가 — 자연어 prompt → planner → Task → 실행
  6. **Dry-run preview UI** — [PromptPanel](../frontend/src/components/panels/PromptPanel.tsx) 에 LLM 이 짠 step list 먼저 띄우고 사용자 confirm 해야 실행. 환각 1번이 충돌로 이어지는 거 막음

- **결정 정리** (Step DSL refactor 로 사실상 정리됨):

  | 축                        | 옛 상태                                                | 현재                                                                                             |
  | ------------------------- | ------------------------------------------------------ | ------------------------------------------------------------------------------------------------ |
  | 1. Control flow 위치      | (A) step DSL 안 / (B) planner 재호출 / (C) hybrid 보류 | ✅ **(A) 선택** — `ForEach`/`BreakIf`/`Try` 가 step DSL 안 first-class                           |
  | 2. Context 타입화         | dict 유지 / typed / Pydantic 미정                      | ✅ **typed schema** — Position3/Pose6/Detection + Slot[T]                                        |
  | 3. Primitive 입도         | 기준 미정                                              | ✅ **primitive + recipe 두 층** — [step_dsl.md](step_dsl.md) §10 참조                            |
  | 4. 새 primitive 비용 모델 | schema reflection 가능한가 미정                        | ⏳ **LLM 도입 시점에 결정** — dataclass `fields()` reflection 으로 자동 생성 가능 여부 본격 점검 |

- **리스크 / 트레이드오프**:

  - Qwen2.5-1.5B 는 2-슬롯 추출은 잘 하지만 multi-step plan 의 Slot 참조 일관성은 환각 잦을 가능성. 안 되면 Qwen2.5-3B / Phi-3.5-mini 로 올림 (RTX 3060 가능). 다만 typed Slot 으로 plan-time 검증 → 환각 _수용 가능 한도_ 낮춤.
  - 환각 방지는 `type` literal whitelist + Slot 참조 검증 (없는 step_id 참조 시 reject + 재호출)
  - LLM 이 recipe 우선 호출 / 새 조합엔 primitive 분해 — system prompt 가이드로 처리

- **의존성**: 이미 [prompt_parser.py](../backend/modules/llm/prompt_parser.py) 인프라 (모델 로드/lock/preload/JSON 파싱/fallback) 굴러감. Step DSL 토대 완성. 추가 plumbing 약 ~400줄 수준.

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
- **의존성**: 이미 [tsdf_pipeline.md](tsdf_pipeline.md) + [pointcloud_node.py](../backend/nodes/application/pointcloud_node.py) 인프라 다 있음. NBV scorer 만 추가.

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
