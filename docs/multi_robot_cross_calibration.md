# Multi-Robot Cross-Calibration — Design Note

두 robot 의 base 사이 상대 transform (`T_robot_a_robot_b`) 을 구하는 캘리브레이션. SO-101 도착 후 omx_f + so101 bimanual 운용의 prerequisite.

본 문서는 **논의 정리 + 구현 plan** 이며 결정 못 박은 자리는 그대로 open question 으로 표기.

**관련 문서**:
- [multi_robot_architecture.md](multi_robot_architecture.md) — multi-robot 일반화 architecture (이 cross-cal 은 §5 calibration 의 6번째 종 후보)
- [calibration_workflow.md](calibration_workflow.md) — 기존 5종 캘 절차
- [calibration_apply_flow.md](calibration_apply_flow.md) — 5종 캘 적용 메커니즘
- [hand_eye_extended_ba.md](hand_eye_extended_ba.md) — hand-eye BA (cross-cal 의 prerequisite 인 hand-eye 정확도 기반)

---

## 1. 배경

### 1.1 왜 필요한가

bimanual / cooperative task (예: omx 가 병 잡고 so101 가 뚜껑 따기, handoff, 두 팔 동시 grasp) 자리에서 두 robot 이 같은 world frame 을 공유해야 함:
- 충돌 회피 (한 robot 의 EE 가 다른 robot 의 workspace 어디 있는지)
- planning 초기값 (대략 어디로 가야 상대 robot/물체 시야에 들어오는지)
- vision 못 쓰는 자리에서 절대 좌표 의존 작업
- frontend 시각화 정합 (`RobotLayer` 가 N robot 을 같은 world frame 에 띄움)

현재 [`robots.yaml::base_pose`](../robot/robots.yaml) 는 hand-measured nominal 값 placeholder. multi-task 일반 운용엔 부족.

### 1.2 카메라 setup (N=2 시점)

- **omx_f**: USB cam (eye-in-hand, RGB only)
- **so101**: D405 (eye-in-hand, RGBD)

둘 다 hand-eye 카메라 보유 → vision-based cross-cal 양쪽 모두 가능.

---

## 2. 산업 표준 — 3 tier

| Tier | 정밀도 | 도구 | 자리 |
|---|---|---|---|
| 1 | sub-100μm | Laser tracker (Leica AT960 등, $100K+) + precision base plate + dowel pin | 자동차/항공 (BIW, 항공기 동체) |
| 2 | 0.1–0.5mm | TCP touch / sphere calibration (vendor SDK: ABB MultiMove, KUKA RoboTeam, FANUC Coord Motion) | 일반 제조 |
| 3 | 0.5–2mm | Vision + fiducial (ChArUco, ArUco, AprilTag — Photoneo / Zivid / OpenCV / MoveIt2 `easy_handeye2`) | 코봇 / 유연 셀 / 연구 |

**우리 자리**: 책상 + C-clamp + 학습 setup. Tier 1/2 의 precision mounting / 외부 metrology 없음. **Tier 3 (vision + fiducial) 이 사실상 산업 표준 자리**.

`easy_handeye2` (ROS2) / `hand_eye_calibration` (Photoneo) 등이 우리가 짜고 있는 hand-eye 인프라와 같은 자리를 차지하는 도구.

---

## 3. 두 방안 + cross-check

### 3.1 방안 A — Shared fiducial (대칭)

두 robot 다 같은 ChArUco 보드를 본다.

```
omx_base ← omx_ee ← omx_cam ← board → so101_cam → so101_ee → so101_base
```

수식:
- omx 자세별: `T_omxbase_board_i = T_omxbase_ee_i · T_ee_cam(omx hand-eye) · T_cam_board(detect)`
- so101 자세별: `T_so101base_board_j = T_so101base_ee_j · T_ee_cam(so101 hand-eye) · T_cam_board(detect)`
- 보드 고정 → 모든 자세에서 같은 값이어야 → 평균
- `T_omx_so101 = T_omx_board · T_so101_board⁻¹`

**Prerequisite**: 두 robot 다 hand-eye 캘 완료.

**에러 source**: 7개 link (양쪽 4개 + 보드 공유).

### 3.2 방안 B — so101 가 omx EE 마커 본다 (비대칭)

omx EE/flange 에 ArUco 마커 1장 (CAD 로 마운트 위치 알려진).

```
so101_base → so101_ee → so101_cam → marker ← omx_ee ← omx_base
```

수식:
- so101 쪽: `T_so101base_marker = T_so101base_ee · T_ee_cam(so101 hand-eye) · T_cam_marker(detect)`
- omx 쪽: `T_omxbase_marker = T_omxbase_ee · T_ee_marker(CAD 상수)`
- `T_so101_omx = T_so101base_marker · T_omxbase_marker⁻¹` (여러 자세 평균)

**Prerequisite**: so101 hand-eye 캘 완료. **omx hand-eye 안 끝났어도 시작 가능**.

**에러 source**: 5개 link (so101 측 3 + omx 측 2). 방안 A 보다 짧음.

**장점**:
- so101 D405 hand-eye 이미 σ_rot 0.65° / σ_t 7.94mm 자리 — 강한 link 만 체인에 들어감
- omx USB cam hand-eye (아직 미구축) 의 정확도가 D405 보다 노이지할 가능성 — 약한 link 회피
- bottle-opening 같은 "so101=vision, omx=blind" task 의 역할 분담과 방향 일치

### 3.3 권장: 둘 다 + cross-check (redundant measurement)

- 방안 A 로 estimate `T_A`
- 방안 B 로 estimate `T_B`
- 1mm / 0.3° 안에서 일치하면 cal 신뢰 OK
- 불일치 시 어느 link 가 깨졌는지 진단 (omx hand-eye? marker mount offset? FK?)

산업 자리에서도 "redundant measurement" 가 신뢰성 확보 표준 절차.

---

## 4. Workflow

### 4.1 Mount 먼저 → world frame 정의 나중

**잘못된 mental model**: "(0, 0.4) 자리에 omx 를 정확히 둬야 한다"
**올바른 흐름**:

1. 두 robot 을 책상에 **편한 자리** 에 C-clamp (mounting 정밀도 신경 X)
2. 그 다음 두 robot 의 *상대 transform* 측정
3. 둘 중 하나 (예: so101) 를 "world origin" 으로 임의 지정 → 다른 하나의 base_pose 가 측정값에서 derive

이 흐름이면 mount 정확도 != base_pose 정확도. mount 는 cm 단위로 대충, base_pose 는 cal 결과로 mm 단위.

### 4.2 캘리브레이션 순서 (중요 — 에러 체인)

```
1. 각 robot individual cal (5종, 기존)
   joint_offset → link_offset → sag_offset    (FK 정확도)
   → intrinsic                                  (카메라)
   → hand_eye                                   (FK + intrinsic 둘 다 사용)

2. Cross-cal (6번째 종, 신규)
   → individual cal 결과 전부 입력으로 받음
   → T_omx_so101 산출, npz 저장
   → 보드 치움
```

**왜 순서 강제?** joint_offset 틀리면 → FK 틀림 → hand-eye 틀림 → cross-cal 틀림. 에러가 체인 따라 누적. individual cal 안 끝난 robot 으로 cross-cal 돌리면 결과 못 믿음.

### 4.3 캘판 위치 규칙

| 시점 | 보드 상태 |
|---|---|
| 한 캘 세션 *동안* | **고정** (마스킹 테이프로 네 꼭짓점) — AX=XB / cross-cal 수식이 "보드 frame 일정" 전제 |
| 세션 *사이* | **자유롭게 옮김** — 각 세션은 독립 |
| 캘 끝난 후 | **치움** — 일상 동작 시 보드 불필요 (산출물 npz 만 메모리 로드) |

**Cross-cal 세션 한정**: so101 캡처 → 보드 그대로 → omx 캡처 (순차 OK, 동시 캡처 불필요. 보드만 안 움직이면).

### 4.4 Re-cal 트리거

- C-clamp 풀거나 robot 위치/yaw 변경
- robot 충격 (책상 부딪힘 등) 받았을 가능성
- 그리퍼 / 카메라 마운트 교체 (hand-eye 영향 → cross-cal 도 재실행)
- 산업 자리에선 daily/weekly drift verify routine 도 — study 자리엔 over-spec

---

## 5. Hand-measure path (저정밀 / 단기 fallback)

vision cross-cal 인프라 구축 전까지의 임시 path.

### 5.1 정확도 ceiling

- 약 **3–6mm + 1–2°** — frontend 시각화 / 충돌회피 정도 OK, 일반 multi-task 엔 부족
- 한계 이유: URDF `base_link` origin 은 robot 몸체 *내부* 가상 점 — 캘리퍼 jaw 못 댐. fiducial point 식별 자체가 노이지.

### 5.2 J1 축 표면 투영 방법

URDF base origin 이 보통 **J1 축 + 마운팅 표면 교점** 자리 (convention). 그 점을 책상에 mark 해야 측정 가능:

- **(a) CAD/spec 기반**: 베이스가 원기둥이고 J1 이 중심이면 → 외곽선 따라 마스킹테이프 4점 표시 → 대각선 그어 교점 = 중심
- **(b) J1 회전 trace**: EE 에 펜/포인터 부착 → J1 만 천천히 회전 → 펜이 그리는 원호의 중심 = J1 축 (compass 거꾸로)

### 5.3 측정 절차

1. so101 mark = origin (0, 0, 0)
2. omx mark 까지 dx, dy 를 줄자/캘리퍼로 잼
3. yaw 는 각도기 — 각 robot 의 "J1=0 일 때 팔 뻗는 방향 = +X" 정렬해 두 forward arrow 사이 각도 측정
4. Z 는 두 robot 책상 위에 있으면 0 (mounting 표면 일치 가정)
5. `robots.yaml::base_pose` 에 박음

### 5.4 한계

- Step 5.2 의 mark 정확도 ±2–5mm
- Step 5.3 측정 ±0.5–1mm
- 합쳐서 3–6mm + 1–2°
- yaw 측정이 특히 노이지 — 각도기 정확도 + forward 정의 모호성

vision cross-cal 인프라 생기면 즉시 deprecate.

---

## 6. 저장 위치 (open question)

| 옵션 | 자리 | 장단 |
|---|---|---|
| A | `robot/instances/<robot_id>/calibration/peer_<other_id>.npz` | 기존 instance 별 5종 캘 구조와 일관. 하지만 어느 robot 쪽에 저장할지 비대칭 (방안 B 면 so101 자연, 방안 A 면 둘 다 가능) |
| B | `robot/pairs/<robot_a>_<robot_b>/cross_cal.npz` | "관계" 가 robot 한 대의 속성이 아니라는 점 반영. 새 폴더 구조 필요 |

[multi_robot_architecture.md](multi_robot_architecture.md) §5 (calibration 분리) 에 6번째 종 포함시키며 결정.

---

## 7. 후속 구현 작업 (so101 도착 후)

so101 hardware 도착 + 6DOF mod + Feetech adapter 등록 완료된 후 (so101_6dof_plan.md 의 prerequisite) 시작:

1. **Cross-cal capture UI** (frontend) — `RobotCalibrateMode` 에 6번째 탭 추가. 보드 detect 시각화 + 자세별 캡처 카운터 + 양쪽 robot 진행 표시
2. **Cross-cal solver** (backend) — 새 module `backend/modules/calibration/cross_calibration.py`. 방안 A / B 양쪽 풀고 cross-check. Open3D / scipy.optimize 사용
3. **Storage 결정** — §6 옵션 중 택1, multi_robot_architecture.md §5 업데이트
4. **Topic / service contract** — `horibot/calibration/srv/cross_capture` / `cross_solve` 같은 신규 키, api_contract.py 등재
5. **omx hand-eye 신규 캘** — USB cam intrinsic + hand-eye 캡처 파이프라인 동작 확인 (방안 A 의 prerequisite)
6. **omx EE 마커 마운트 디자인** — 3D print 마운트 + 마커 위치 CAD 측정 (방안 B 의 prerequisite)
7. **`base_pose` 의 SSOT 갱신 메커니즘** — yaml hand-measured 값 vs cross-cal npz 값의 우선순위 / merge 룰

---

## 8. 핵심 take-away

- **Cross-cal 도 일종의 캘**. 기존 5종 캘과 같은 npz 산출물 패턴.
- **Mount 정확도 != base_pose 정확도** — 정확도는 cal 이 만듦, mount 는 그냥 어디든.
- **보드는 도구**, 영구 설치 X. 세션 동안만 고정.
- **순서 강제**: individual cal (FK → camera → hand-eye) → cross-cal. 약한 link 가 체인 전체 정확도 결정.
- **우리 setup 의 산업 표준 자리 = Tier 3 (vision + fiducial)**. 캘리퍼 hand-measure 는 fallback 일 뿐 일반 multi-task 엔 부족.
- **redundant measurement** (방안 A + B cross-check) 가 신뢰성 확보의 표준 절차.
