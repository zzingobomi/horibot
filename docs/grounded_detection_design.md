# Grounded Detection 설계

> Open-vocabulary detection을 활용한 자연어 기반 픽업 시스템 설계 문서.
> 사용자가 영어 텍스트로 물체를 묘사하면 그것을 찾아서 잡아오는 파이프라인.

## 목표

```
사용자: "black car key" 입력 → 로봇이 그것을 찾아서 정해진 위치에 옮김.
```

기존 detection 흐름은 COCO 80 클래스로 사전학습된 YOLO 기반이라 우리 워크스페이스 물체(차키, 피규어, 부품 등)를 거의 알아보지 못함. 이를 open-vocabulary detection으로 대체.

## 핵심 결정사항

| 항목 | 결정 | 근거 |
|------|------|------|
| Detection 모델 | **Grounding DINO** (Swin-B) | 속성 묘사("검은색") 잘 이해, 자연어 grounding SOTA |
| 입력 언어 | 영어만 | 번역 단계 제거, 단순성 |
| 동작 흐름 | one-shot | 입력 1번 → inference 1번 → 잡기 1번 |
| 가정 | 시야 안에 물체 있음 | 첫 단계 단순화 (스캔/탐색은 추후) |
| 2D → 3D 변환 | bbox 영역 depth median → unproject | D405 RGBD 활용, 평면 가정 안 함 |
| Grasp 자세 | top-down 고정 yaw | 가장 단순, 작은 물체에 충분 |
| PLACE 좌표 | 하드코딩 | 추후 ROI 기반으로 확장 |
| 노드 | `DetectorNode` (기존 이름 재사용, 내용은 재작성) | 익숙한 이름 유지 |
| Task 연동 | 새 step + 시나리오 | 기존 task 시스템 패턴 유지 |

## 모델 선택 — 왜 Grounding DINO?

후보 3개 비교:

|  | YOLO-World | OWLv2 | **Grounding DINO** |
|---|---|---|---|
| prompt 스타일 | 단어 ("cup") | phrase ("red cup") | 자연어 ("the black key on table") |
| 속성 이해 | 약함 | 보통 | **강함** |
| 속도 | 30+ FPS | 5~10 | 1~3 |
| 무게 | ~50MB | ~600MB | ~700MB |
| 정확도 | 중간 | 중간~높음 | **높음** |
| 희귀 물체 | 약함 | 보통 | **강함** |

선택 근거:

1. **속성 묘사가 핵심** — "검은색 차키"의 "검은색"이 중요 단서. Grounding DINO만이 자연어 속성을 detection에 제대로 반영.
2. **속도는 비핵심** — one-shot이라 1~3 FPS도 충분.
3. **VRAM 여유** — RTX 3060 12GB에서 Swin-B 무리 없음 (실행 시 4~5GB).
4. **희귀 물체 강함** — 차키/피규어/부품은 COCO에 없음. Grounding DINO 학습 데이터가 훨씬 다양.

## 아키텍처

```
[Frontend]
   Input "black car key" + [Run]
        ↓ task 호출 (prompt 파라미터)

[TaskNode] — 기존 task 시스템 활용
   pick_named_object 시나리오:
   ┌─────────────────────────────────────────┐
   │ 1. GroundedDetectStep(prompt) → 3D 좌표 │
   │ 2. MoveTCPStep (좌표 위로 접근)         │
   │ 3. MoveTCPStep (내려가기)               │
   │ 4. GripperStep (close)                  │
   │ 5. MoveTCPStep (들어올리기)             │
   │ 6. MoveTCPStep (PLACE 좌표, 하드코딩)   │
   │ 7. GripperStep (open)                   │
   │ 8. HomeStep                             │
   └─────────────────────────────────────────┘
        ↓ 1번 step에서 서비스 호출

[DetectorNode] — 새로 짤 노드
   Service: omx/perception/grounded_detect
   Input:   {"prompt": "black car key"}
   Output:  {"success", "position": [x,y,z], "confidence", "bbox"}

   내부 흐름:
     1. RGB + depth 한 쌍 캡처
     2. Grounding DINO inference → 2D bbox + score
     3. bbox 영역 depth median → 카메라 프레임 3D 점
     4. intrinsic으로 unproject
     5. hand_eye matrix 곱 → 베이스 프레임 좌표
```

### 좌표 변환 체인

```
2D bbox (image px)
   ↓ bbox 영역 depth median 추출
depth value (mm or m)
   ↓ intrinsic (fx, fy, cx, cy)로 unproject
3D 점 (카메라 frame)
   ↓ hand_eye matrix 적용
3D 점 (베이스 frame)
   ↓ TaskNode가 이걸 받아서 MoveTCP
실제 위치
```

## 진화 경로 — 지금 짜는 게 버려지지 않음

지금 구조는 토대. 더 복잡한 작업으로 확장 시에도 유지됨.

```
                Stage 0     Stage 1        Stage 2       Stage 3

Detection       2D ─────── 2D ──────────── 2D ────────── 3D (2D 보조)
3D 변환        depth ── depth + multi ── pointcloud ── 3D mask
Grasp          top-down ── top-down ──── 6DoF 예측 ── 6DoF

                "위 잡기"   "각도 문제"   "어떻게 잡지"   "복잡한 장면"
```

### Stage 0 (지금 짜는 것)

- 2D detection + depth + top-down
- 테이블 위 분리된 물체 잡기

### Stage 1 — 시야 각도 문제 해결

**문제**: "위에서 본 차키"는 막대기처럼 보여 detection 실패. 2D 모델은 canonical view에 학습된 만큼, 비정상 view에 약함.

**해결책** (Grounding DINO 자체는 그대로 유지):

- **(A) Multi-view 캡처** — 여러 각도에서 detection 후 confidence fusion
- **(B) TSDF mesh의 canonical view 렌더링** — 이미 mesh 있으니 거기서 "옆에서 본 모습" 렌더링 → 그 이미지에 detection
- **(C) Active perception** — confidence 낮으면 카메라 각도 능동 변경

### Stage 2 — 6DoF grasp pose 추가

- 2D detection은 "어디" 만 답
- 잡힌 영역의 pointcloud → GraspNet / AnyGrasp → "어떻게 잡을지" 자세 예측
- Detection 그대로, grasp 모듈만 추가

### Stage 3 — 3D segmentation 도입

- 산업 bin picking 같은 정말 복잡한 장면에서만 필요
- 이때조차 2D는 보조로 남아있는 경우 많음 (mature 하니까)

### 핵심

각 Stage가 **독립적으로 추가**됨. 지금 짜는 Motion 코드, 좌표 변환 코드, Task 구조는 Stage 3까지 그대로 살아남음. "교체"가 아니라 "추가" 모델.

## 우리 방식 vs 진짜 3D Segmentation — 근본 차이

|  | **우리 방식** | **3D Segmentation** |
|---|---|---|
| 입력 | 사진 (2D 픽셀) | 점/메시 (3D 구조) |
| 출력 | 이미지 위 사각형 | 3D 공간의 voxel 집합 |
| 모델이 "이해"하는 것 | 픽셀의 색/패턴 | 3D 모양/표면 |
| Depth의 역할 | 단순 조회 ("이 픽셀의 거리") | 모델의 입력 ("이 모양을 봐") |

**비유**:
- **우리 방식** — 눈 감고 사진 만지는 사람한테 "여기 컵 있어"라고 손가락으로 가리키는 거. AI는 사진만 봤고, 깊이는 우리가 자로 잰 거.
- **3D seg** — AI가 직접 3D 공간을 만져보고 "여기부터 여기까지가 컵"이라고 판단.

**우리 방식이 충분한 경우** (현재 use case):
- 테이블 위 떨어져 있는 물체들
- 위에서 잡기만 하면 됨
- 중심 좌표만 알면 됨

**우리 방식이 약한 경우** (Stage 3 영역):
- 겹친 물체, 가려진 면
- 모양 정보가 필요한 grasp planning

## 미정 사항

구현 단계에서 결정할 것들:

1. **Task step 시그니처** — 기존 `DetectStep` 재사용 vs 새 step (`GroundedDetectStep`)
2. **시나리오 구조** — `pick_named_object` 이름/위치, 어디에 정의
3. **PLACE 좌표** — task 파라미터로 받기 vs 시나리오에 하드코딩
4. **안전 처리** — confidence threshold, 못 찾을 때 동작, z 제한, 워크스페이스 경계
5. **프론트 UX** — input 위치, 결과 시각화 (bbox 오버레이?)
6. **Depth 스트리밍 정책** — 평소 켜두기 vs on-demand
7. **번역 단계** — 추후 한국어 입력 지원 시 (지금은 영어 only로 단순화)
8. **모델 로드 타이밍** — 노드 시작 시 vs 첫 호출 시 (lazy)

## 참고

- 라이브 detection이 안 될 때 mesh 기반으로 fallback하는 패턴은 [tsdf_pipeline.md](tsdf_pipeline.md) 참조
- 캘리브레이션 산출물의 적용 경로는 [calibration_apply_flow.md](calibration_apply_flow.md) 참조
- 기존 task 시스템 (DetectStep, MoveTCPStep 등)은 [backend/modules/task/](../backend/modules/task/) 참조
