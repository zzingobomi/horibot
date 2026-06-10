# Calibration Workflow

캘리브레이션 페이지의 Hand-Eye 탭을 사용하는 절차와 결과 해석 가이드. **무엇이 어떻게 적용되는가**는 [calibration_apply_flow.md](calibration_apply_flow.md), **BA 자유도/알고리즘**은 [hand_eye_extended_ba.md](hand_eye_extended_ba.md) 참조.

---

## 1. Capture → Compute → Commit 절차

좌측 카메라 피드 위에 라이브 체커보드 코너 오버레이가 자동 표시되어 자세 평가가 실시간으로 됨.

1. (필요 시) **Capture 카드 [리셋]** — 누적 포즈 비움 (백엔드 재시작 불필요).
2. 자세 잡기 (Move TCP / 토크 OFF 후 수동). 라이브 오버레이가 초록색이면 검출 OK.
3. **[캡처]** — 프레임 캡처 + 체커보드 검출 + PnP + 포즈 추가. 검출 실패면 사유 표시되고 포즈 미추가.
4. 8~10자세 반복 (자세 다양성 가이드 ↓).
5. **Compute 카드 [COMPUTE]** — BA 모드 선택 (`physical_sag` 기본, `extended`, `standard`) + per-pose 잔차 + method 비교 출력. **파일 저장 X** (미리보기만).
6. 결과 해석 (§ 결과 해석 가이드). outlier 포즈는 Capture 리스트의 휴지통(`#<id>` 클릭)으로 삭제 후 다시 COMPUTE — Pose ID는 안정 ID라 삭제해도 인덱스 시프트 없음.
7. 만족스러우면 **Commit 카드 [COMMIT]** — `hand_eye.npz` + (BA 모드에 따라) `joint_offsets.npz` / `link_offsets.npz` / `sag_offsets.npz` 저장.
8. (선택) **Validate 카드** — 저장된 .npz 또는 최근 COMPUTE 결과로 T_target←base 흩어짐 σ_rot/σ_t 측정.

### COMMIT 후 재시작 필요 여부

[calibration_apply_flow.md § 0](calibration_apply_flow.md) 표 참조. 요약:

| 산출물       | 즉시 반영 | 재시작 필요               |
| ------------ | --------- | ------------------------- |
| hand_eye     | O         | DetectorNode 재시작 필요  |
| joint_offset | O         | 불필요                    |
| link_offset  | O (mem)   | **백엔드 재시작 필요** (PyBullet URDF는 부팅 시 1회 로드) |
| sag_offset   | O         | 불필요                    |

---

## 2. 자세 다양성 가이드

5DOF 한계 안에서 최대한 다양하게:

- **joint 1 base yaw** — 좌우 회전 (월드 yaw)
- **joint 4 wrist pitch** — 위아래 끄덕임
- **joint 5 wrist roll** — 비틀기
- 셋을 골고루 섞기. 한 축만 위주로 돌리면 TSAI 회전 추정이 부정확.
- 체커보드는 화면 중앙 가깝게, 너무 비스듬한 각도(<30°)는 PnP 정확도 떨어짐.
- 매 자세 캡처 직전 로봇 완전 정지 (모터 명령 전송 후 ~0.5s 대기).

---

## 3. 결과 해석 가이드

COMPUTE / Validate 결과를 보고 어떤 조치를 취할지 판단하는 룰. 색 임계값은 [HandEyeResults.tsx](../frontend/src/components/calibration/HandEyeResults.tsx)에 박혀 있음.

### 색 임계값

| 항목                       | 의미                                                                                                                                           | 초록 (좋음)  | 노랑 (경계)   | 빨강 (나쁨)   |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- | ------------ | ------------- | ------------- |
| **σ_rot**                  | T_target←base 회전 분산. 캡처한 모든 포즈에서 본 체커보드를 base 프레임으로 환산했을 때 얼마나 흩어지나. 체커보드는 안 움직였으니 이상적이면 0 | <0.5°        | <1.5°         | ≥1.5°         |
| **σ_t**                    | 위 위치 버전 (mm)                                                                                                                              | <5           | <15           | ≥15           |
| **PARK / DANIILIDIS Δrot** | TSAI 대비 다른 알고리즘 결과의 차이. 같은 입력을 세 가지 다른 수학으로 풀어서 합의 정도 → 입력 self-consistency 척도                           | <1°          | <3°           | ≥3°           |
| **per-pose drot / dt**     | 각 포즈가 평균(또는 첫 포즈) 대비 벗어난 양. outlier 식별                                                                                      | <0.5° / <5mm | <1.5° / <15mm | ≥1.5° / ≥15mm |

### 진단 룰 — 읽는 순서: PARK Δrot → per-pose → σ

1. **PARK Δrot 노랑/빨강** (≥1°) → 알고리즘 자체 문제 아니라 **입력 포즈에 outlier가 섞여 있음**. PARK이 TSAI보다 outlier에 민감해 가장 먼저 빨강이 됨. per-pose 표에서 빨강 행 식별 → 삭제(#`id`) → 재 COMPUTE.
2. **PARK ≤1°인데 σ_rot 빨강** (≥1.5°) → outlier는 정리됐지만 **시스템 전반 오차**. 자세 다양성 부족 가능 → joint 1/4/5 분포 점검 후 추가 캡처. 그래도 안 떨어지면 [hand_eye_extended_ba.md](hand_eye_extended_ba.md) 참조해서 BA mode 변경 (`extended` / `physical_sag`).
3. **σ_rot 초록 (<0.5°) + σ_t 초록 (<5mm)** → 캘 품질 충분. COMMIT. TSDF/ICP에 사용 OK.

### 액션 플레이북

| 상황                                | 조치                                                                      |
| ----------------------------------- | ------------------------------------------------------------------------- |
| per-pose에 빨강 1~3개 (나머진 깨끗) | 빨강 포즈 삭제 → COMPUTE 재실행                                           |
| per-pose에 빨강/노랑이 절반 이상    | 캡처 절차 문제 (로봇 정지 안 함 / 체커보드 가림 / 비스듬). 리셋 후 재캡처 |
| PARK 노랑, σ_rot 경계               | 자세 다양성 부족 가능 → joint 1/4/5 분포 점검 후 추가 캡처                |
| 모든 게 깨끗한데 σ_rot ~ 1° 정체    | BA mode `standard` → `extended` → `physical_sag` 단계적으로 시도          |
| Validate σ가 Compute σ보다 큼       | 정상 (Validate는 평균 대비 흩어짐, Compute는 첫 포즈 대비)                |

> **TSDF 목표치**: σ_rot < 1° / σ_t < 10mm. 현재 달성치는 BA `physical_sag`로 σ_rot **0.65°** / σ_t **7.94mm** ([hand_eye_extended_ba.md § 16](hand_eye_extended_ba.md)).

---

## 4. Intrinsic

[robot/calibration/intrinsic.npz](../robot/calibration/intrinsic.npz) — D405 color 1280x720, **factory seed 기반** (`seed_d405_intrinsic_if_missing`이 카메라 노드 기동 시 채움).

- camera_matrix: fx=649.75, fy=648.10, cx=632.67, cy=359.60
- dist_coeffs: [-0.0525, 0.0596, -0.000246, 0.000545, -0.0198]
- rms_error=0.0 — 재캘리브 잔차가 아니라 factory seed라서 0.

D405의 color stream 공장 캘리브는 일반적으로 정확하므로 별도 재캘리브는 보류. UI에서 Intrinsic 탭으로 재캘 가능하지만 현재 권장하지 않음.

---

## 5. Calibration Board (ChArUco)

Hand-Eye / Intrinsic 캡처에 사용하는 ChArUco 보드.

### Pattern spec (OpenCV 입력)

```yaml
Pattern:        ChArUco
Rows:           5
Columns:        7
Square Length:  25 mm     # OpenCV `squareLength` — 실측치 사용 (아래 주의)
Marker Length:  18 mm     # OpenCV `markerLength` (≈ square × 72%)
Dictionary:     DICT_4X4
```

내부 코너 = (5-1) × (7-1) = **24개** / pose. 마커 17개.

### 물리적 사양

- 보드 외곽: 200 × 150 mm (패턴 175 × 125 mm + 여백)
- 재료: 포맥스 5T (PVC foam, white)
- 표면: PP 유포지 + 무광코팅 (수분/조명 반사 무관)
- 모서리: 라운드 처리 (안전, 캘 영향 0)

선정 근거: OMX 5DOF 자유도 제약 + 책상 55×34cm 환경 ([hardware.md § 작업대](hardware.md)) 에서 "작은 보드 + 다양한 pose" 가 "큰 보드 + 적은 pose" 보다 유리. 6×8 (35 코너) 도 후보였으나 OMX 도달 영역 위주로 5×7 선택. SO-101 (6DOF) 도 같은 보드 공용 가능 (D405 stays 가정 — OMX swap 후 USB UVC 시나리오는 [hardware.md § 카메라](hardware.md) 참조).

### 재제작 정보

- **PDF 생성**: calib.io Pattern Generator
  - Target Type: ChArUco
  - Board Width 200 / Height 150
  - Rows 5 / Columns 7 / Checker Width 25
  - Dictionary DICT_4X4 / Start Id 0
- **합지**: 출력스토리 견적 의뢰 → 포맥스 5T 무광. 참고 단가 16,280원 + 배송비 (2026-06)
- **견적 의뢰 시 명시**: "카메라 캘리브레이션용, square 25mm 치수 정확도 중요" (자동 fit-to-page 방지)

### 사용 시 주의

- **PDF 설계치(25mm) ≠ 실측치 가능** — 합지 시 인쇄 스케일 ±1% 오차 흔함
- 받은 보드는 캘리퍼스로 square 실측 → **실측치를 OpenCV `squareLength` 에 입력** (PDF 설계치 X)
- 실측 결과 (2026-06-10, 벌니어 캘리퍼스): **25 mm** — 벌니어 정밀도 (±0.05mm) 내 PDF 설계치 일치. [board.py:37](../backend/modules/calibration/board.py#L37) `SQUARE_LENGTH_M = 0.025` 그대로 유지. (square 0.05mm 오차 → 작업거리 250mm 환산 0.5mm pose error, σ_t <10mm 목표 대비 무시 가능)
