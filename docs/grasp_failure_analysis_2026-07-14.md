# 파지 실패 정밀 분석 — 2026-07-14 실물 세션 (내일 분석용 dossier)

> **목적**: 오늘(2026-07-14 밤) 실 SO-101 + D405 로 pick&place 를 여러 번 굴렸으나
> 한 번도 파지 성공 못 함. 시도/수정/실패를 전부 박제해 **내일 이 문서 + `backend/debug/`
> 압축본으로 정밀 분석**한다. 결론 아님 — **가설과 검증 계획**이 핵심. 해소되면 내용은
> [perception.md](perception.md) / [backend.md](backend.md) 로 접어 넣고 이 파일 삭제.

---

## 0. 세팅

- robot: `so101_6dof_0` (SO-101 6DOF, Feetech STS3215, D405 **손목 장착** eye-in-hand)
- task: `white small round cube` 집어 `blue box` 위에 놓기
- 흐름: search 스윕(찾기) → close 멀티뷰 관측·융합 → antipodal 파지 → 놓기
- 좌표계: base frame. **z=0 이 물리적으로 어디인지 = 최우선 확인 대상 (§4-A)**

---

## 1. 오늘의 시도 연대기 (무엇을 바꿨고 무엇이 실패했나)

| # | 바꾼 것 | 결과 |
| --- | --- | --- |
| 1 | (시작) 놓기 타깃 점수-only 선택 | 놓기 실패 — 점수 1등이 선반 위 통(도달 불가), 닿는 테이블 박스 버림 |
| 2 | **놓기 도달성 폴백** (점수순 spot 루프, resolve_place non-raising) + 놓기 top-down 재설계 | 놓기 여전히 실패 — top-down tilt ±30 이 SO-101 사각지대(§3.2)에 꽂혀 IK 전멸 |
| 3 | 놓기 **tilt 파지와 동일 도달 범위(0~±90)** 로 되돌림 | 놓기 여전히 실패 — yaw 2방향(정렬)만이라 자세 그물 성겨 "위치 통과/자세 전멸" |
| 4 | 놓기 **yaw 정렬4 + 자유8 가족** + place tilt 사다리 성기게 + RESOLVE timeout 60→120s | 놓기 계획 **성립**(자유 yaw tilt+45 채택) — 여기서부터 놓기는 계획됨 |
| 5 | **멀티뷰 융합 정합**: naive vstack → 중심차+평균 앵커 (뷰별 1.5~3.3cm 어긋남) | 파지 얼룩(50×64mm→가짜 w=31mm 허공 파지) 완화, 그래도 **큐브 끝 스침** |
| 6 | **찾기/집기 분리**: search 스윕으로 파지 판정하던 조기종료 제거, 파지는 close 뷰만 융합 | **여전히 못 집음 — 큐브 위 허공을 스치듯 집음** (이 문서의 실패) |

**부수 사건**: 세션 중 **PC 2회 하드다운**. 원인 = ① 유령 중복 backend(`apps.main` 2개
= CPU 경합, resolve 74s) ② 저가 PSU(Aone Storm 600LF)가 RTX 3060 스파이크 못 버팀.
임시 조치 `nvidia-smi -pl 120`(170→120W). 근거 = Kernel-Power 41 (Bugcheck=0 +
PowerButton=0 = 순수 전원 손실, 소프트 크래시 아님). → 파워 교체 권장.

---

## 2. 코드 변경 목록 (오늘, 전부 test 초록 387 passed)

- [tasks/pick_and_place/geometry.py](../backend/modules/tasks/pick_and_place/geometry.py)
  - `plan_place` / `plan_place_free`: 놓기 자세 = 파지 tilt 도달 범위 + yaw 정렬4/자유8 가족
  - `_PLACE_TILTS_DEG`(0/±30/±45/±60) — 놓기 전용 성긴 사다리 (perf)
- [tasks/pick_and_place/steps.py](../backend/modules/tasks/pick_and_place/steps.py)
  - `plan_place`: 도달성 우선 spot 폴백 + 정렬→자유 yaw 가족 폴백
  - `resolve_place`: non-raising (None 반환, 최종 실패는 호출부)
  - **`observe_and_plan_grasp`: search 시드/조기종료 제거 → close 뷰 관측만 융합** ★핵심
  - detect/resolve 에 elapsed 초 로그
- [detector/geometry.py](../backend/modules/detector/geometry.py)
  - `align_and_merge_views`: 뷰 중심차 평행이동 + **평균 앵커** 정합 (ICP 는 상보 면 height 붕괴로 기각)
- [detector/module.py](../backend/modules/detector/module.py)
  - 디버그 덤프: `debug/detect/{세션시각}/` 순번 PNG + 후보/융합 점군 PLY + 메트릭 txt
- [motion/contract.py](../backend/modules/motion/contract.py): RESOLVE_REACHABLE timeout 60→120s

---

## 3. 오늘 마지막 실패 run 정밀 데이터 (`debug/detect/20260714_233959/`)

**증상 (사용자 관찰)**: "큐브 위에 스치듯이 그리퍼가 허공을 집었어" — 옆이 아니라 **위/허공**.

### 3.1 파지는 "성립"했다 (기하는 이제 정상으로 보임)
- close 뷰 4개 누적 → `group 64 채택 — pair2 tilt=+30 w=26mm (쌍 10)`
- **w=26mm = 실제 큐브(≈25mm)와 일치** → 예전 얼룩(w=31mm) 문제 아님. 크기·쌍은 정상.

### 3.2 그런데 뷰별 위치가 여전히 크게 흩어진다 (close 뷰인데도!)
실제 큐브(≈(0.27, 0.125)) 관측 (search 2 + close 4):

| dump | 종류 | pos (x,y) | base_z | footprint |
| --- | --- | --- | --- | --- |
| 0001 | search | (0.269, 0.138) | 0.033 | 30×21 |
| 0002 | search | (0.277, 0.121) | 0.020 | 28×23 |
| 0004 | close1 | (0.264, 0.123) | 0.023 | 25×14 |
| 0006 | close2 | (0.278, 0.117) | 0.016 | 34×23 |
| 0008 | close3 | (0.268, 0.138) | 0.016 | 22×21 |
| 0010 | close4 | (0.276, 0.119) | 0.015 | 24×21 |

- **close 4뷰만**: 중심 xy std **5.7 / 8.2mm**, base_z **15~23mm** 편차.
- close 뷰로 좁혔는데도 산포가 안 줄었다 = **거리 문제 아니라 자세별 FK 오차**(예측대로).

### 3.3 융합 점군이 아직 스미어 (`0011_fuse`)
- 실제 큐브(≈25mm)인데 융합 후 **X 38mm × Y 47mm × Z 29mm**, centroid (0.273, 0.124, **0.029**).
- 중심차+평균 앵커로도 **~40mm 스미어** 잔존 — 부분 관측들이 정확히 registration 안 됨.

### 3.4 z 가 수상하다 (★ "허공을 집었다" 와 직결)
- 융합 큐브 z 범위 **17~45mm**, 파지 mid z ≈ 29mm.
- 테이블 위 2.5cm 큐브라면 바닥≈0, top≈25mm 여야 정상인데 **top 이 45mm, 바닥이 16mm**.
- 즉 검출이 큐브를 **실제보다 ~16mm 높게** 본다? → 팔이 그만큼 높은 데서 조를 닫음
  = **큐브 위 허공**. (단, base z=0 이 실제 어디인지 모름 — §4-A 가 먼저.)

### 3.5 놓기 (참고 — 이번엔 계획 성립)
- blue box 후보2 (0.230, -0.101) score 0.83 → 정렬 yaw 28 전멸 → **자유 yaw tilt+45 채택**.
- 도달성 폴백 + 자유 yaw 가족이 제대로 동작. 놓기 계획 로직은 OK.

### 3.6 성능 (두 번째 큰 문제 — 거의 못 써먹을 속도)
- detect 스윕(3자세): **9.0~9.2s**
- 파지 resolve(26그룹): **17.5s**(실패) / 6.4s(성공)
- 놓기 resolve: 정렬 28그룹 **17.6s** + 자유 29그룹 **22.8s**
- **1회 시도 ≈ 수 분.** resolve ~0.7s/group 이 지배적. (2 backend 경합 아님 — 단일에서도 이 속도.)

---

## 4. 내일 분석 계획 (debug 폴더로 하나씩)

### 4-A. **[최우선] base z=0 이 물리적으로 어디인가**
"허공을 집었다" 의 z 가설 검증. 방법:
1. 자로 실제 테이블 표면·큐브 top 높이를 robot base 기준 측정 (또는 알려진 base_pose 로 계산).
2. 검출 base_z(≈16mm) 와 비교 → 계통 z 오프셋이면 그게 "위에서 닫힘" 의 직접 원인.
3. hand_eye/cross-cal 의 z 성분 점검 (z bias 면 모든 검출이 위/아래로 시프트).
→ **오프셋이면 최소 노력으로 큰 개선.** 아니면 H1(절대정확도)로.

### 4-B. FK/백래시 절대오차 정량 (repeatability)
1. 한 자세에서 큐브 검출 **N회 연속**(안 움직이고) → 산포 = 검출/depth 노이즈.
2. 딴 데 갔다 **같은 자세 복귀** 후 검출 → 산포 = 백래시 repeatability.
3. **다른 자세들**에서 검출(=오늘 §3.2, xy 14~21mm) → 자세별 FK 오차.
→ 1≪3 이면 "절대정확도 바닥" 확정. (오늘 데이터로 3은 이미 큼.)

### 4-C. 파지 목표 vs 실제 큐브 위치
- 로그의 채택 파지 pos(pair2 mid) 와 실제 큐브 위치(자 측정) 비교 → 빗나간 **방향·크기**.
- xy 만 빗나갔나, z 도인가 → 4-A/4-B 중 무엇이 지배적인지.

### 4-D. PLY 육안 (MeshLab/CloudCompare)
- `000N_det_*_c0.ply` (단일 뷰, 깨끗한 25mm?) vs `00NN_fuse_*_c0.ply` (융합, 38×47mm 스미어?).
- 스미어가 뷰 간 평행 오프셋(FK)인지, 회전 어긋남인지 → registration 전략 결정.

---

## 5. 가설 (순위 + 오늘 증거)

| # | 가설 | 지지 증거 | 반증/미확인 |
| --- | --- | --- | --- |
| H1 | **절대 FK/백래시 정확도(~1-2cm) ≈ 큐브(2.5cm)** → open-loop 구조적 한계 | 뷰 xy 14~21mm 산포, close 뷰로도 안 줄음, 융합 40mm 스미어 | 정량 repeatability(4-B) 미실시 |
| H2 | **z 계통 오프셋** (검출이 ~16mm 높게) → 위에서 닫힘 | "허공을 집음", 융합 z 17~45mm, base_z 일관되게 15~33mm | base z=0 실제 위치 미확인(4-A) |
| H3 | 융합 스미어(40mm)가 antipodal mid 를 흐림 | 융합 38×47mm | w=26mm 는 정상 — 크기는 맞음 |
| H4 | eye-in-hand 미활용 (검출 자세 ≠ 파지 자세 → FK 오차 안 상쇄) | 손목 캠인데 far 검출/다른 자세 파지 | — |

**H1·H2·H4 는 서로 연결** — 전부 "자세별 FK 오차를 open-loop 이 못 이김". H3 은 이제 부차.

---

## 6. 유력 수정 방향 (진단 후 착수 — 오늘은 구현 안 함)

1. **eye-in-hand 공통오차 상쇄** (H1/H4 정답 후보): 타깃 근처 **집기 접근 자세에서 검출**하고
   그 자세 그대로 하강 → 검출·실행이 같은 FK bias 공유해 **상쇄**. 손목 D405 라 자연스러움.
   look-then-move / 짧은 open-loop 하강. **표준 해법.**
2. **z 오프셋 보정** (H2 면): 검출 base_z 를 실측 테이블 z 로 캘/보정. 값싼 즉효 가능.
3. **registration 개선** (H3): 겹침 큰 뷰만 ICP, 또는 단일 최적 뷰 OBB 채택 (박스 한정).
4. **캘 재점검**: hand_eye z/전체 σ 실측 재확인 (문서상 σ_t 7.5mm floor — [calibration.md](calibration.md)).
5. **성능**: resolve group 수 축소/조기중단, IK 예산 튜닝, 검출 스윕 자세 수 축소. (별 트랙)

---

## 7. debug 폴더 읽는 법 (`backend/debug/detect/{세션시각}/`)

- **세션마다 폴더 1개**, 매 검출/융합 호출이 4자리 순번으로 쌓임 (overwrite 없음).
- `{NNNN}_det_{prompt}.png` — SAM mask 오버레이 + bbox + OBB + yaw (세그멘테이션 육안).
- `{NNNN}_det_{prompt}.txt` — 후보별 score/base_z/height/pos/footprint/points (수치).
- `{NNNN}_det_{prompt}_c{i}.ply` — 후보 i 의 base 점군 (voxel 다운샘플, m). 단일 뷰.
- `{NNNN}_fuse_{prompt}_c{i}.ply` — 융합(멀티뷰 병합) 점군. **단일 뷰 PLY 와 나란히 비교** = 스미어 진단.
- 순번이 곧 시간순 = detect 스윕 → close 뷰들 → fuse → (다음 물체) 흐름 그대로 재생.
- PLY 좌표 = base frame (m). MeshLab/CloudCompare 로 열어 크기·z·정합 확인.

---

## 8. 한 줄 요약

크기·yaw·놓기 계획은 이제 정상. **남은 건 "큐브가 base 어디에 있냐"가 뷰마다 xy ~2cm /
z ~2cm 틀리는 것** (자세별 FK/백래시, close 뷰로도 안 줄고 융합으로도 못 지움). 내일
**4-A(z 실측) → 4-B(repeatability) → 4-C(목표 vs 실제)** 순으로 H1/H2 를 가르고, 답이면
**eye-in-hand 공통오차 상쇄**로 간다. 기하 파라미터는 더 만지지 않는다.
