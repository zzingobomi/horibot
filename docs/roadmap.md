# Roadmap

진행 중/예정 작업 기록용. 이미 구현 완료된 항목은 git log + 관련 docs/ 문서로 가니까 여기는 **미래 작업**만.

> 현재 active 작업: [self_play_pick.md](self_play_pick.md) — self-play pick 루프 설계 진행 중 (WIP).
>
> 피벗 메모: TCP 절대 정확도 측정 방향(옛 AccuracyTest 패널)은 폐기. pick 실패의 root metric 은 "TCP 절대 정확도"가 아니라 "pick 성공률"이고, 정밀 캘리브레이션/peg/물리 모델은 DIY 환경(사용자 개입 최소화 제약)에서 ROI 낮음. self-play 로그 누적 → residual 보정 → (필요 시) bandit/RL 순서로 진화.

---

## Grounded Detection — 다음 세션 우선

> Phase 2 frontend shell + backend wiring + Grounding DINO Swin-B (lazy preload + cu130) 까지 완성. 실측에서 정확도 문제 1건과 정리 잔재 몇 개.

### 1. (최우선) Mesh raycast 로 detection 3D 좌표 보정

**증상**: 카메라 피드 상 bbox 는 차키 위에 정확히 떨어지고 confidence 56%인데, base frame 환산 결과가 `(0.153, -0.085, -0.020)` — Z 가 작업대(z=0) 평면 아래 20mm. 3D 씬에서도 분홍 sphere 마커가 그리드 밑으로 떨어짐.

**원인 후보** (누적):
- bbox 영역 depth median 이 작업대 가장자리 픽셀까지 포함해 노이즈
- 카메라 ↔ 작업대 비스듬한 각도에서 depth 오차가 ray 따라 증폭 → base Z 음수 방향
- hand_eye σ_t 7.94mm 잔류
- (별도) TCP Z 도 -0.0052m 로 살짝 음수 — base frame z=0 정의 자체가 작업대 표면이 아닐 가능성

**해법**: bbox 중심 픽셀 ray ↔ 빌드된 TSDF mesh 교점 (Open3D `RaycastingScene`). depth median + unproject 대신 mesh 가 ground truth. design.md 의 "Stage 0 = depth-only" 가정이 깨졌으므로 Stage 1 보조 수단을 1차로 끌어옴.

**결정 필요**:
- mesh 선택: 자동 latest (`mesh_<session>.ply` 중 mtime 최신) vs 사용자 명시 (UI 드롭다운)
- mesh 없을 때 fallback: 지금 depth median 유지 vs 명시적 fail
- mesh 좌표계: TSDF 빌드 결과는 이미 base 프레임이므로 ray 도 base 프레임으로 변환 후 raycast
- bbox 중심 1픽셀 ray vs bbox 영역 N픽셀 ray voting (mesh 가 작은 결손/노이즈 있을 때)

**코드 위치**: [backend/nodes/detector_node.py](../backend/nodes/detector_node.py) `_handle_grounded_detect`. 새 모듈 `modules/perception/mesh_raycast.py` 추출 권장 (DetectorNode 가 mesh 관리 책임 안 지게).

**검증**: 차키 마커가 mesh 표면(작업대 + 차키 본체) 위에 박히는지. 그 다음 task `pick_named_object` 흘려서 그리퍼가 실제로 차키를 잡는지.

### 2. 잔재 lint 정리 (병행 가능)

phase 2 작업 중 만난 사전 dirty 상태 — phase 2 무관이라 그대로 두고 옴.

- [backend/nodes/motion_node.py](../backend/nodes/motion_node.py) 미사용 import 5개 (numpy, ruckig 심볼 4개) — ruff F401, `--fix` 한 번이면 끝
- [backend/modules/task/step_executor.py](../backend/modules/task/step_executor.py) `_move_tcp` Optional/object narrowing 3건 — pyright reportArgumentType. `step.position` None 가드 + `context.get()` cast 추가 필요

### 3. 안전/UX 보강 (mesh raycast 후 자연스러움)

- **confidence threshold 가드** — 현재 box_threshold=0.3 미달 시 fail, 통과 후엔 score 그대로. task 실행 전 e.g. score < 0.5 면 stop 옵션
- **워크스페이스 경계 검증** — base x/y/z 가 작업 영역 밖이면 fail (mesh raycast 도입 시 ray 가 mesh 안 닿으면 None → 이걸로 자연스럽게 처리)
- **PromptPanel 마커 reset** — task 끝나도 분홍 sphere 잔존. 새 prompt 입력 또는 명시 clear 버튼
- **bridge timeout 검토** — cu130 GPU 잡혔으니 ~1-3초로 떨어질 것. 현재 60초인데 너무 길면 UX (실패 응답이 늦게 옴) — preload 끝난 뒤엔 5~10초 정도로 줄여도 무방

### 4. (보류) Stage 1 — 시야 각도 / Multi-view

- 차키가 위에서 거의 막대기로 보이는 케이스. Grounding DINO confidence 떨어지면 카메라를 다른 각도로 이동해 재시도. design.md Stage 1 (A) Multi-view 캡처.
- 또는 mesh 의 canonical view 렌더링 (design.md Stage 1 (B)).

### 5. (보류) 한국어 prompt

현재 영어만. 번역 layer 추가하면 됨 — 우선순위 낮음.

---

## TSDF / PointCloud

- (관찰 단계) Mesh 품질이 캘 σ_t 7.94mm 영향을 얼마나 받는지 — `voxel_size=2mm`, `sdf_trunc=10mm` 기본값에서 두께/이중벽 양상 확인. 영향 크면 voxel size 키우거나 ICP `icp_max_dist` 조정.
- 자세 수 vs mesh 품질 trade-off — 10자세 vs 20자세에서 fragment 개수, hole 개수 비교.
- (보류) Colored ICP — point-to-plane으로 부족할 때만. JPEG 압축이 텍스처에 노이즈 줄 수 있어 현재 미채택.
- (보류) Mesh smoothing / hole filling — Open3D `filter_smooth_taubin`, `fill_holes`. 1차 결과 보고.

### End-to-end 정확도 평가 시나리오 (집에서 실행 예정)

**목적**: "사용자가 mesh에서 클릭한 점 ↔ EE가 실제 도착한 물리 위치" 오차 = 이 시스템이 사용자에게 줄 수 있는 정확도. TSDF 단독 평가가 아니라 mesh + 캘 + IK + sag 전체 스택의 end-to-end 평가.

**셋업**:
- 20mm XYZ 캘리브레이션 큐브 1개 (MakerWorld) — 캘리퍼스로 실측해서 ground truth 갱신.
- (선택) Pointer tool 1개 — 그리퍼가 grasp하는 기둥(10×10×30mm) + 30~40mm 원뿔 끝. 하드웨어 교체 X. tool 출력 후 "grasp center → tip" 거리 캘리퍼스로 재서 TCP offset에 박음.
- 또는 단순히 그리퍼 닫고 finger 사이 중심점을 TCP로 사용 (정확도 살짝↓).

**절차**:
1. 큐브 책상 위에 놓고 TSDF scan.
2. Workspace3D에서 mesh 표시, 큐브 꼭짓점/feature 클릭.
3. 클릭 좌표로 `move_l` (Z+30~50mm hover 먼저 → 천천히 descent → 표면 1~2mm 위 정지).
4. 카메라/캘리퍼스로 needle 끝과 실제 큐브 feature 사이 X/Y/Z 오차 측정, 점별 기록.

**측정 포인트 (큐브 옮기지 않고 한 자리에서)**:
- 윗면 꼭짓점 4 + 변 중점 4 + 중심 1 = 9점 (모두 위에서 접근 가능).
- **바닥 4 꼭짓점은 버린다** — 그리퍼가 측면으로 접근 못 함.
- Z 다양화 원하면 50mm 큐브 추가 1개로 Z 두 단계 확보.

**분리 가능한 오차**:
- `클릭점 - 큐브 GT` = TSDF + hand_eye 오차 (mesh가 진실에서 얼마 떨어짐)
- `EE 도달점 - 클릭점` = IK + sag + joint/link_offset 오차 (명령한 곳에 얼마나 갔나)
- `EE 도달점 - 큐브 GT` = end-to-end 오차 (사용자 체감 숫자)
- 9점 평균 = systematic shift, 분산 = random/local error

**실행 전 선행 작업** (TSDF 1차 결과 보고 나서 착수):
- [MeshLayer.tsx](frontend/src/components/workspace3d/3d/MeshLayer.tsx)에 raycaster onClick → 좌표 추출 → "이 점으로 move_l" UI.
- Pointer tool TCP offset 처리 (사용 시).
- Safe approach 시퀀스 (hover → descent → stop above surface) 헬퍼.

**유의사항**:
- 큐브 실측해서 ground truth 갱신 안 하면 프린터 인쇄 오차(elephant foot 등 ±0.1~0.3mm)가 TSDF 오차로 잘못 잡힘.
- 한 자리 한 자세 평가 — 작업공간 전체 정확도는 별도. 첫 숫자 보고 나서 다중 위치 평가 여부 결정.

## Calibration

- 현재 σ_rot 0.65° / σ_t 7.94mm 달성. 더 내리려면:
  - D405 마운트 강성 — XL330 wrist 그룹 끝에 카메라 매달려있는 구조라 작은 sag 잔존 가능.
  - Joint encoder 분해능 — XL430의 4096 분해능이 본질적 floor.
  - 두 가지 다 H/W 변경 필요 → 소프트 측면에서는 사실상 saturate 상태로 보고 TSDF 결과로 검증.

## 분산 운영

- (보류) 모터 Pi의 motion + motor 통합 latency 측정 — 현재 100Hz `MOTOR_CMD_JOINT` 큐 모니터링은 없음. 명령 누락 시 trajectory가 끊겨야 정상.
- (보류) Zenoh peer 발견 실패 케이스 — 멀티캐스트 차단 환경 디버깅 절차 정리.
