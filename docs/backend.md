# backend — 아키텍처 spec (framework + Module catalog + Task 방향)

> 본 문서 = backend 의 **아키텍처 SSOT 단일 문서**. 구성: §1–§14 framework spec
> (8 라운드 토론 2026-06-25 + 이후 정정), **§16 Module catalog** (옛
> backend_modules.md 통합, 2026-07-03), **§17 Task-first 운영 원칙 + Task/PnP 설계**
> (옛 task_dsl_waypoint_port.md 통합). §2.7 에 robot-scoped/agnostic + robot_id 라우팅
> 최종 규칙 (옛 robot_agnostic_module_refactor.md 통합).
>
> **진행 status / 다음 작업 = 바로 아래 [진행 status] 부** (2026-07-11 구 backend.md 통합 — 세션 handoff 는 항상 그 부를 갱신).


---
---

<!-- ═══════════ [통합 원문] backend.md — 진행 status + 세션 handoff (항상 이 부를 갱신) ═══════════ -->

# backend — 진행 status + 다음 세션 handoff

> 새 세션이 **바로 이어서 작업**할 수 있게 박은 status. 아키텍처 SSOT =
> [backend.md](backend.md) (framework §1–§14 + Module catalog §16 + Task-first §17).
> 본 문서 = "지금 어디까지 됐고 다음 뭐 할지" 만 — 설계 결정은 여기 안 둠.

## 현재 상태 (2026-07-16)

**framework + 전 Module 가동 + Task 아키텍처 확정 + 집기 = closed-loop(servo)
look-then-move 로 전면 전환 구현 완료 (실물 검증 대기 — 집).** open-loop 멀티뷰
antipodal 집기는 실물 0/N 로 대체됨 (아래 2026-07-16 부).

### 2026-07-16 — 집기 closed-loop(servo) 전환 (구현 + sim 검증 완료, 실물 대기)

배경 = [grasp_debug_handoff_2026-07-16.md](grasp_debug_handoff_2026-07-16.md)
(post-mortem: 기구학 절대정확도 ~1-2cm ≈ 큐브 2.5cm → 정적 인식 후처리로는 불가) +
[closed_loop_grasp_handoff.md](closed_loop_grasp_handoff.md) (구현 사고 가이드).

- **착수 전 offline 검증** (`scripts/grasp_verify/closed_loop_feasibility.py`,
  실물 2026-07-15 debug 세션 재분석): ① base 관측 편차가 카메라 거리에 비례
  **r=0.95** (14-17cm 에서 5-12mm / 31-33cm 에서 ~40mm) = "가까이서 loop 를 닫으면
  오차가 준다" 전제 실측 성립. ② cam-frame centroid 산출 건강 (기하 거리와 ≤5mm).
  ③ mask 오검출로 455mm 튄 뷰 실존 → tick gate 필수의 근거. ④ hand_eye 기하:
  카메라가 TCP 를 ~5° 오차로 응시, TCP-파지점 5cm 에서 카메라-물체 14.3cm =
  최적 측정 대역 → **파지 직전까지 servo 가능**.
- **구조**: 이산 look-then-move (정지 관측 → tick gate → 관측한 그 tick 의 TCP
  기준 상대 목표 MoveL → 수렴 시 standoff 사다리 8→5cm 하강 (12cm+ 는 실 URDF
  IK 로 자세 고정 사다리 전멸 — test_motion servo ladder sim 이 잡음) → 마지막 관측으로
  blind commit → close → 판정(EMPTY 재시도 1) → 후퇴 → 판정). 연속 velocity servo
  기각: 검출 ~1s/tick + **이동 중엔 카메라 frame ↔ TCP snapshot 쌍이 깨짐**
  (detector 가 검출 시점 TCP 를 따로 읽는 계약). 코드 정본 =
  `modules/tasks/pick_and_place/servo.py` (순수 계산 + 상태 전이) + `steps.py`
  `servo_pick`/`plan_pick` + `servo_trace.py` (run 당 `debug/servo_pick/<ts>/`
  tick JSONL + summary — 로그만으로 실패 재구성이 요구사항). 파라미터 SSOT =
  `servo.ServoConfig` (전부 실물 첫 런 데이터로 튜닝하는 knob).
- **대체/삭제**: `observe_and_plan_grasp`/`try_plan_grasp`/`fuse_target`/
  `execute_pick`(advance/withdraw 포함) 삭제, `geometry.view_directions`/
  `view_pose_groups` 삭제 (뷰 이동 자체가 servo 로 대체 — git 복원 가능).
  `antipodal.py`/`plan_grasp` 는 production 소비자 없음 상태로 유지
  (grasp_verify 진단 자산 + test_motion resolve sim 이 소비). **놓기는 open-loop
  유지** (상자 적치는 12.8mm 급 오차에 관대). wire 계약 무변경 (RUN req/스트림
  전부 그대로 — frontend regen 불요, 페이지 그대로 동작).
- **sim 으로 증명된 것**: servo 상태 전이 전수 (test_servo 19) + 시나리오 배선
  (test_pick_and_place — 오검출 hold/연속 소실 결단/수렴 실패 중단/이동 거부
  폴백/EMPTY 재시도/trace 기록/놓기 선검증 불변식). 전 스위트 422 passed. mock
  run_task = 부팅+트리거+명시 실패 경로 확인 (mock DB 에 waypoint/hand_eye 없음).
- **실물에서 남는 미지수 (정직)**: ① 근접(14cm)에서 GDINO 가 큐브를 계속 잡는가
  + 벌린 그리퍼가 mask 를 오염시키는가 (gate 가 걸러주지만 miss 연발이면 commit
  일찍 끊김) ② 상대 명령의 common-mode 상쇄가 실제로 남기는 잔차 크기 (offline
  추정 ~5mm, blind 5cm 구간의 FK drift 미포함) ③ ServoConfig 임계값 전부
  (eps/capture/jump/min_points — trace 데이터로 1회 보정 전환) ④ tilt 가족에서
  카메라가 물체를 보는 각도 (수직 가족이 기각되고 tilt 45+ 채택 시 시야 기하).
  실물 절차: 어제와 동일 (frontend or `run_task srv/pick_and_place/run`) — 실패
  시 `debug/servo_pick/<ts>/trace.jsonl` + `debug/detect/<세션>/` timestamp 교차.

### 2026-07-14 (4) — 실물 pick&place 디버깅 (집, 실 SO-101+D405)

pick&place 실물 첫 굴림 세션. 여러 실패를 debug 덤프(신규)로 뿌리까지 추적 —
아래 순서로 벗겨짐 (각각 회귀 test 잠금):
- **놓기 재설계**: ① 타깃 = 점수-only → **닿는 첫 spot 폴백**(선반 위 통 버리고 테이블
  박스) ② 자세 = top-down 강제 폐기 → 파지와 같은 tilt 도달 띠(사각지대 §3.2) ③ yaw
  2방향 → **정렬 4 + 자유 8 가족**(2방향은 위치 닿아도 자세 전멸). place tilt 사다리
  성기게(perf) + RESOLVE timeout 60→120s.
- **멀티뷰 융합 정합**: naive vstack → **뷰 중심차 평행이동 + 평균 앵커**(뷰마다 검출
  1.5~3.3cm 어긋남 = 백래시 FK 오차). ICP 는 상보 면서 height 붕괴로 기각.
- **디버그 덤프 강화**: `debug/detect/{세션}/` 순번 PNG + 후보/융합 점군 PLY + 메트릭
  txt (overwrite 1장 폐기). base_z/얼룩/스미어 육안 판별 — 위 진단 전부 이걸로.
- **찾기/집기 분리 (핵심, 사용자 지적)**: `observe_and_plan_grasp` 가 **search 스윕
  (멀리서 찾기) 관측으로 파지까지 판정**하던 조기 종료를 제거. search = coarse 위치만,
  **파지는 close 뷰 관측만 융합**해 세운다 (스윕 시드 배제). 멀티뷰를 넣은 목적 복원.

**열린 근본 (다음 세션 최우선)**: 팔 절대정확도(백래시/FK ~1-2cm) ≈ 큐브 크기(2.5cm)
라 open-loop 파지가 구조적으로 아슬. Z 안정 + XY 3cm 산포가 증거. **진짜 답 = eye-in-hand
공통오차 상쇄**(검출을 집기 접근 자세에서 → 검출·실행이 같은 FK bias 공유해 상쇄) — D405
손목 장착이라 자연스러움. 확인용 repeatability 체크(같은 자세 ×N vs 다른 자세) 먼저.
부수: `gripper_held_threshold_raw` 소비자 0(허공 물어도 진행) — motor 계약 확장 필요.

**HW 사고**: pick&place 중 PC 2회 하드다운. 원인 = ① 유령 중복 backend(`apps.main` 2개
= CPU 경합 74s) ② 저가 PSU(Aone Storm 600LF)가 GPU 스파이크 못 버팀. 임시 `nvidia-smi
-pl 120`(RTX 3060 170→120W). 진단 근거 = Kernel-Power 41 (Bugcheck=0 + PowerButton=0).

### 2026-07-14 (3) — §10 대전환 구현: 일반 형상 + adaptive 멀티뷰 + 표면 antipodal

설계 SSOT = [grasping.md](grasping.md) **§1**(아키텍처 — keep/replace/add 그대로 구현
완료). 아래 (2) 부의 "단일 사선 뷰 / 고정 궤도 / footprint
파지 / height 게이트" 서술은 이 부로 **대체**됨 — (2) 의 Phase 1 게이트 골격과
object-centric detect 는 유지.

**REPLACE 구현:**
- detector `object_metrics_from_points`: 2-percentile bottom → **z-gap 군집**
  (`_body_bottom_z` — top 에서 5mm 빈 틈 전까지가 몸통, §10.3-F). 실물 #1
  phantom(base_z −0.23m) 클래스의 근본 수정 — 아래-outlier 5% 회귀 테스트 잠금.
- task `plan_grasp`: footprint(prismatic 추측) 파지 → **관측 표면 antipodal**
  (`tasks/pick_and_place/antipodal.py` — open3d 법선, 조 축 수평 필터, 중심
  파지 선호 정렬 + dedupe + 상한 12쌍) × tilt 0~±90 × 조 축 flip. **폭 하한
  8mm** — 프로토타입 4mm 는 노이즈 σ1mm 의 4σ 라 단일 뷰 edge 가짜 쌍이 생김
  (프로덕션 파이프라인 재검증에서 발견, 스코프 물체 ~2cm 라 8mm 안전).
- `target_view_poses` 고정 궤도 6점 → **adaptive 뷰 탐색축** (`view_directions`
  반경 0.16/0.13 × 고도 55/40/70 × 방위 spread-first 12방향 + `view_pose_groups`
  roll 6변형 — resolve 그룹으로 묶어 첫 가용 roll 채택).
- `observe_target` 궤도 순회 + `require_plausible_height` **폐기** →
  **`observe_and_plan_grasp` adaptive 루프**: 검색 스윕 관측 = 공짜 멀티뷰 시드
  → 융합 → antipodal → resolve, **서면 멈춤** (§10.3-G — sim 에서 대부분 2뷰).
  안 서면 다음 뷰 (도달 상한 6), 끝까지 안 서면 **"안전 파지 불가" 명시 실패**
  (NoReachableGrasp — 맹목 파지 금지, §10.4-3).

**ADD 구현 (motion 게이트 확장 — RESOLVE_REACHABLE, frontend 미노출):**
- `Kinematics.set_obstacle_points`/`obstacle_collision` (pybullet: 6mm voxel →
  3mm 구 + 침투 2mm 임계, gripper_open=URDF 상한 벌림 후 원위치 복원 — sag
  wrapper forward 포함).
- 게이트 ③b **장애물 점군 충돌** (`obstacle_points`+`gripper_open` — 그리퍼가
  물체/이웃 점군 침투 시 기각) / ④ **관절 보간 경로** (`path_from` — home→첫
  해 경로의 self/floor/obstacle, §10.4-4 naive MoveJ 금지의 계획 시점 강제.
  실행부는 실제로 home 경유 MoveJ) / ⑤ 기존 linear. 전멸 message 에 게이트별
  기각 수. 장애물 lifecycle = 판정 동안만 (잔존 오염 회귀 테스트).
- task: 뷰 이동/파지/적치 resolve 전부 `path_from=home` + 파지는
  `obstacle_points=융합 점군+이웃(0.15m 내 다른 군집)` + `gripper_open=True`.
  이웃 = 같은 prompt 의 다른 검출 군집만 (미관측 장애물은 실물 몫 — §10.6).

**검증:** ruff/pyright 0 · fast pytest 381 PASS (신규: antipodal/뷰수식 12 ·
z-gap 회귀 · resolve 장애물/경로 게이트 sim 2 · adaptive 관측 시나리오 6) ·
**프로덕션 파이프라인 sim end-to-end** (당시 `verify_production_pipeline.py` — 실 캘
kinematics + 물리 렌더 부분 점군으로 실제 모듈 코드 실행; **2026-07-15 제거, git history 복원
가능** — sim 이 실 depth 편향을 못 담아 false confidence 였음, grasping.md §6):
3위치 × 4형상(box/눕힌원기둥/구/L자) × 클린/노이즈(σ1mm+outlier3%) = **24/24
파지 성립, 대부분 2뷰 정지**, z-gap bottom 오차 ≤10mm (구는 바닥 미관측 정직 오차).

**다음 세션 (집, 실물):** ① `home` waypoint 티칭 (필수) ② 실패 위치(0.275,0.208)
재시도 — adaptive 관측 + antipodal 채택 확인 ③ D405 근접(0.13~0.16m) depth 품질
/ 뷰 도달 수 / 융합 점군 밀도 ④ §10.6 실물-only (파지 물리 안정성 / 재질 dropout
/ 실 마스크 품질) ⑤ 로드맵 3 (공중 파지). 실물에서 과민하면 조정할 임계:
antipodal 폭 하한 8mm / obstacle 침투 2mm / MoveL jump 0.35rad.

### 2026-07-14 (2) — object-centric 파지 재설계 Phase 1·2 구현 (grasping.md §1)

> ⚠️ **부분 대체 (위 (3) 부):** 이 부의 "자동 뷰 6개 궤도 / observe_target /
> require_plausible_height / height prior" 서술은 §10 대전환으로 교체됐다.
> Phase 1 (reachable-orientation + resolve 게이트 골격 + grasp-frame 동작) 과
> object-centric detect (floor ring 폐기 / FUSE_ORIENTED) 는 유효.

설계 SSOT = [grasping.md](grasping.md). 구현 규율대로
**§7 미검증 항목을 시뮬로 먼저 짚고** (캘 적용 kinematics 재현 — FK 시연관절
sub-mm 일치 anchor 재확인) 로드맵 1·2 를 구현. 전부 시뮬/유닛 검증 완료, **실물
검증은 집 (아래 체크리스트)**.

**§7 시뮬 선검증 결과 (구현 전):**
- 실물 실패 케이스(큐브 0.275,0.208) 재현 — 현 top-down 가족 실질 전멸 확인.
- 확장 가족(tilt 0~±90 + 접근축 pre)에서 **tilt 30~60 base쪽 후보가 pre+grasp
  끝점 + pre→grasp 직선 전 구간 IK 통과**, 인접 샘플 joint jump ≤5°/cm (플립
  없음 → jump 게이트 임계 20°/샘플 근거).
- motors.yaml home(2048)은 joint3 URDF limit 밖 — **home 경유는 티칭 waypoint
  `home`** 으로 확정 (motors.yaml home 재사용 금지).
- pybullet 바닥 평면 침투 검사 의미론 확인 (base 링크 상시접촉 없음, 재호출/
  self_collision 오염 없음).

**Phase 1 — reachable-orientation 파지 + resolve 게이트 파이프라인:**
- `tasks/pick_and_place/geometry.py`: (파지) top-down 강제 폐기 — tilt 0~±90 전체
  probe (조 축 수평 유지 불변), **pre = grasp 의 접근축(툴 x) 후방** (월드 +z
  폐기). 파지 후보 13tilt×2yaw×2flip=52.
  - **놓기 (2026-07-14 재설계 + 같은 날 실물 2회 정정)**: 파지와 **딱 두 가지만
    다르다** — ① antipodal 옆면 물기 없음(상자 가운데로, lateral 오프셋만 재사용)
    ② yaw 는 쌍 방향이 아니라 **상자 방위(spot.grasp_yaw)** 기준. 나머지는 파지와
    같은 폭으로 뿌린다 — 좁힌 게 두 번 다 실물 전멸의 원인:
    - 정정 1: tilt 소각(±30) 제한 → SO-101 top-down±40° 사각지대(§3.2)에 전멸.
      tilt 는 파지와 동일 0~±90° (수직 먼저 = 선호, 도달 판정은 resolve).
    - 정정 2: yaw 2방향(0/90°) → "위치 통과 26/26, **자세 IK 실패 26**" — 지점은
      닿는데 방향 그물이 성겨 자세를 못 찾음. 집기가 되는 이유 = antipodal 쌍이
      yaw 를 수십 개 공급(쌍10×flip2×tilt13=260 vs 놓기 26). 180° flip 은 위치
      등가지만 조·롤이 달라 **IK 가 다른 별개 자세**. → **yaw 두 가족**: 정렬
      (상자 방위 0/180/90/270°, 13tilt×4=52) 우선, 전멸 시 자유(30° 격자 나머지
      8방향, 13tilt×8=104) 폴백 — 도달이 정렬(선호)을 이긴다 (`plan_place` /
      `plan_place_free`, steps 가족 루프).
    타깃 선택도 점수-only 커밋이 아니라 **닿는 첫 spot** 채택(`steps.plan_place`
    점수순 spot 루프 + `resolve_place` non-raising) — 점수 최고가 workspace 밖
    (예: 선반 위)이면 닿는 대안으로 폴백해 "집었는데 못 놓는" 제거. RESOLVE_REACHABLE
    timeout 60→120s (전멸 가족은 전 그룹 풀예산 IK — 최악이 성공보다 느림 + 유령
    중복 backend CPU 경합 74s 실측). perf: place tilt 사다리는 성기게
    (0/±30/±45/±60 — 15° 랑간·±75/90 제거, 후보 52+104→28+56) + task 로그에
    resolve/detect elapsed 초 기록.
  - **멀티뷰 융합 정합 (2026-07-14 심야, 허공 파지 사고)**: 뷰(관측 자세)마다 검출
    base 좌표가 **계통적으로 1.5~3.3cm** 어긋난다 (3 run 재현 — STS3215 백래시
    ±0.87°/sag FK 오차가 손목 구성마다 다르게 투영, 캘 σ 7.5mm 밖). naive vstack
    융합이 25mm 큐브를 50×64mm 얼룩으로 만들어 가짜 antipodal 쌍(w=31mm)이 허공을
    물었다 (디버그 PLY 로 확정 — debug/detect/ 세션 폴더). 수정 =
    `detector/geometry.align_and_merge_views`: 멤버별 **중심차 평행이동** 정렬
    (ref=medoid, 검출 position 이 자기 점군 centroid 라 bias 를 그대로 담음 →
    중심차 = bias 추정치). **ICP 는 기각** — 상보 면 관측(윗면 뷰+옆면 뷰)은
    겹침이 작아 point-to-point ICP 가 면을 끌어당겨 height 붕괴
    (test_fuse_oriented_merges_views 가 잡음). 알려진 갭 (다음 작업 후보):
    ① **파지 성공 검증 미배선** — `TaskRobotSpec.gripper_held_threshold_raw` 가
    정의만 되고 소비자 0 (SET_GRIPPER 는 쓰기 전용, 도달 위치 readback 계약 없음)
    → 허공을 물어도 task 가 태연히 놓기까지 진행. motor 계약 확장 필요
    (SET_GRIPPER 응답에 settle 후 실측 raw 동봉 or 스냅샷 서비스 신설).
    ② 동일 물체 2개가 3cm 내 인접 시 중심 정렬이 phantom 병합 (grasp 은 ref 실물을
    조준하므로 물리적으론 잡히나 이웃이 장애물 목록에서 빠짐).
- `motion` RESOLVE_REACHABLE 재설계 (frontend 미노출이라 wire 자유):
  cheap→expensive 게이트 ①position-only 스크린(예산5) → ②자세 IK deepening
  (10/40/full — 2026-07-09 벤치 계승) → ③`floor_z` 바닥 평면 충돌 (신규
  `Kinematics.floor_collision`, base쪽 고정링크 제외) → ④`linear` 직선 경로
  샘플 IK + jump. **응답에 `solutions`(관절 해) 동봉 — 실행부 IK 재계산 제거.**
  전멸 message 에 게이트별 기각 수 (사유 침묵 금지).
- MoveL 사전검증을 `_linear_path_blocker` 로 공용화 + **jump_threshold 등가
  게이트 추가** (§8 "거의 필수급") + to_thread 화.
- steps/시나리오: `home_waypoint`(없으면 모션 0 실패+티칭 안내) → plan →
  execute_pick = home→pre(관절해)→open→**진입(MoveL 접근축)**→close→**후퇴(역방향)**
  →home, execute_place = pre→삽입→release→후퇴→home. step 이름 descend/lift →
  advance/withdraw/insert (breakpoint 이름도 변경 주의).
- **Phase 1 시뮬 종합**: 실패 케이스가 tilt+45 후보로 0.24s 에 풀림 (pre 해 FK
  오차 1.45mm), workspace 밖은 0.13s 사유 있는 전멸, 바닥 게이트 동작.

**Phase 2 — object-centric detect + 자동 멀티뷰 융합:**
- detector: **floor ring 추정 폐기** (`projection.floor_z_and_height` /
  `object_top_center_base` / `z_cam_from_depth_bbox` 삭제) — position/base_z/
  height 전부 물체 자기 점군(`base_points_from_mask`)에서
  (`geometry.object_metrics_from_points`, z 2/98 percentile). **base_z 의미
  변경: 주변 바닥 → 물체 아랫면.** 단일 뷰 height 는 구조적 과소 (정직) —
  판정은 융합 후.
- `OrientedDetection.points` (voxel 3mm 다운샘플 물체 점군, 상한 2048) — 서비스
  응답 전용, DETECTIONS_ORIENTED 스트림에선 strip (mask bitmap 결정과 같은 근거).
  contract regen 완료 (fixture+contract.ts, diff = points 한 필드).
- 신규 `DETECTOR.FUSE_ORIENTED` (robot-agnostic 순수 계산): XY 군집
  (`cluster_indices_by_xy`) → 점군 vstack → 기하 재계산. 합성 큐브 테스트로
  "단일뷰 height≈0 → 융합 후 2.3cm 실측 복원" 잠금.
- task: `plan_pick` = detect 스윕 → `select_target_by_score`(**height prior
  폐기** — 스윕 단계 판정 무의미) → **`observe_target`** (hand_eye 로 타깃 중심
  자동 뷰 6개 계산 `target_view_poses` — 반경 0.16m/고도 55·35°/base쪽 방위±40°,
  MoveJ 거부 뷰는 스킵·비 IK 원격실패는 전파, 도달 상한 3) → **`fuse_target`**
  (FUSE 호출 + `require_plausible_height` — height 판정은 여기서만, 사유에 관측
  수 포함). place 는 멀티뷰 불요 (place_z 는 spot 윗면 + held.height).
- floor 는 planner 충돌 평면으로만 강등: `floor_z = base_z − 5mm 버퍼`.

**검증**: backend ruff/pyright 0 · full pytest **374 PASS** (신규: resolve 게이트
sim 2 / geometry 접근축·뷰수식 / observe·fuse 단위 5 / FUSE 융합 2 / object-centric
합성 점군 4) · mock 부팅→contract regen→**kill 확인** · frontend vitest 159 ·
lint(기존 경고 1)·build green.

**다음 세션 (집, 실물):** ① `home` waypoint 티칭 (필수 — 없으면 task 가 안내
메시지로 실패) ② 큐브 실패 위치(0.275,0.208) 재시도 — tilt 후보 채택 + 진입/후퇴
동작 확인 ③ 멀티뷰: 자동 뷰 도달 수 / D405 근접(0.16m) depth 품질 / 융합 height
안정성 ④ grasping.md §6 "sim 한계" 목록 (antipodal 부분 점군 내성,
실제 파지 성공) ⑤ 로드맵 3 (손에 들고 공중 검증). MoveL jump 게이트가 실물에서
과민하면 임계(0.35rad/cm)를 실측으로 조정.

**① 미리보기(#1) 설계 확정 + 구현 — "실행 시뮬레이션"이 아니라 "코드 구조 인덱싱".**
어제 다섯 번 갈아엎은 지점의 틀 전환 (사용자 설계 수렴): 트리를 얻으려고 본문을
돌릴 이유가 없다 — "이 step 이 저 step 을 부른다"는 호출 관계는 **소스에 이미
적혀 있다.** 실행 경로 보장을 요구에서 제외(분기 선택/loop 횟수는 실행 전 미지 —
받아들임)하는 순간 정적 읽기의 약점이 결함이 아니게 됨. dry-run/가짜응답/ctx
게이팅 계열은 전부 "개발자 추가 규약 0" 전제 위반이라 기각 확정.

- **@step 에 정적 표식** (`step.py` — `__is_step__`/`__step_name__`/`__step_title__`
  + `is_step`/`step_meta`): 런타임 게이트/trace 경로 무변경, functools.wraps 의
  `__wrapped__` 로 원본 소스 접근.
- **`tasks/core/preview.py` `build_preview(fn)`**: 소스 AST 에서 호출 지점 수집 →
  이름 해석(**inspect.getattr_static — 프리뷰 중 property/descriptor 실행 금지**,
  실행 0 보장) → resolve 된 대상의 @step 표식으로 판정 (await/호출 문법 무관 —
  개발자가 step 을 지정하지 문법을 강제 안 함). if/match/loop 는 **풀지 않고
  conditional/repeated 표시만**. 못 푼 호출(지역 변수/getattr/첨자)= `<동적>` 노드
  (title 에 호출식 — 구멍이 침묵으로 안 사라짐), 지역 객체 메서드(ctx.call 류)/
  builtin 은 무시(노이즈 차단), 재귀 = recursive 표시 후 절단, 소스 없음 =
  unavailable. wire = **TraceEntry 동형 preorder flat + depth** (`PreviewEntry`,
  트리 표현은 UI 몫 — 프리뷰↔trace 가 같은 렌더 공유).
- **계약**: core `PreviewEntry/PreviewRequest/PreviewResponse`(공용 조작판 모양) +
  pick_and_place `Service.PREVIEW` + FRONTEND_EXPOSED (services 47→48, regen 완료).
- **프론트 TaskProgressPanel**: trace 비면 프리뷰가 그 자리 (breakpoint 미리 박기
  dot + 조건부/반복/재귀/소스없음 배지 + `<동적>` 자리 표식 + 실패 사유·재시도).
  실행 시작 = trace 가 실제 진입으로 자연 치환.
- **run 밖 breakpoint 토글도 STATE 발행** (runner idle 스냅샷 + 모듈 TASK_ROBOTS
  fallback) — 프리뷰에서 미리 박은 bp 가 침묵하지 않게.
- 한계 (전부 표시하고 넘어감 — preview.py docstring): 콜백/자료구조로 넘긴 step,
  중첩 def/lambda 내부, step 아닌 일반 헬퍼 뒤에 숨은 step (계층 규약상 비정상).

**② TaskRunner 콜백 → 데코레이터 DX 통일 (B안 — 선행 확인 통과).** 선행 확인
결과: framework 는 @service/@subscriber(함수 attr 태깅 + dir 스캔) 와 **Mirror
(descriptor + `__set_name__` + 인스턴스 `__dict__` lazy state + on_change 이름
지연 bind)** 두 결을 이미 씀 — B안(descriptor 분리)이 Mirror 와 정확히 같은 결이라
채택. runner 혼자 새 패턴을 들고 오는 게 아님.

- **TaskRunner = 선언부 descriptor** (`task = TaskRunner()` 클래스 변수 +
  `@task.on_state`/`@task.on_trace` — 이름만 저장, `__get__` 이 getattr 로 bound
  해석 → MRO 라 서브클래스 override 승리). **TaskRunnerState = 본체** (기존 엔진
  그대로 — _run/_breakpoints/게이트 + 생성자 콜백 유지 = 단독 조립/headless 표면,
  wire 무지 불변). 인스턴스별 `__dict__["_taskrunner_<name>"]` 격리 — 모듈 2개가
  run/breakpoint 공유하는 사고 차단 (Mirror/MirrorState 분리와 같은 이유, 회귀
  테스트 잠금). `_publish_markers` 는 runner 이벤트가 아니라 시나리오가 도메인
  데이터로 직접 부르는 발행 — 훅으로 안 옮김 (runner 를 도메인에 노출하게 됨).

검증: backend ruff·pyright 0 / fast pytest **362 PASS** (신규: preview 12 +
descriptor 4 + module 2, contract_export 카운트 47→48 갱신) / mock 부팅 →
contract.json fixture+gen:types regen (services 48) → **backend kill 확인** /
frontend lint(기존 경고 1)·build·vitest **159 PASS** (패널 프리뷰 테스트 4 신규).

### 2026-07-13 (밤/3) — PnP 실물 개밥먹기: 구현분 + **열린 문제 4건 (다음 세션 최우선)**

so101_6dof_0 + D405 실 로봇(`--host pc`)에서 pick_and_place 를 처음 돌린 세션.
아래 구현분은 **lint/type/unit 초록이나 실물 미검증** (motion/sim 테스트는 Zenoh
multicast 로 실 로봇에 broadcast 될 수 있어 이 환경에선 안 돌림 — 사용자 실물 확인 몫).

**이번에 구현한 것:**
- **#2 검색 스윕 + plan/execute 순서 재설계** — `detect` 가 waypoint `search` 그룹
  자세를 전부 돌며 후보 **누적**(첫 자세서 안 멈춤) → `select_pick_target` 이 누적
  전체에서 최고 score 선택 (옛 `SearchWaypointGroup`+`SelectTarget` 원리 포팅, git
  `e44acfd:modules/task/tasks/pick_and_place.py` 참고). 시나리오는 **집기·놓기 도달성을
  모두 계획·검증한 뒤** 실행 (`plan_pick`/`plan_place` → `execute_pick`/`execute_place`)
  — 놓을 곳 IK 불가면 집기 전에 실패해 물체 쥔 채 멈추는 corrupt 방지.
- **#3 MoveL 자세 = slerp** (UR/ABB/MoveIt 식: quaternion=목표, 현재→목표 보간;
  계약 필드 불변) + **cartesian EMA 저역통과 제거** (seeded IK 를 직접 명령, MoveJ 동형
  — 옛 alpha=0.1 EMA 가 Ruckig 프로파일을 지연시켜 이동 끝 3~4.5° 잔차→snap = 비매끄러움,
  로그 진단으로 확인) + MoveL 진단 로그(dt/관절 step/ori 총각).
- **task↔robot 바인딩 = `srv/pick_and_place/list_robots`** 서비스 (frontend
  `useTaskRobots` 가 조회 — 하드코딩 `TASK_ROBOT_ID` 제거).
- **미리보기(#1) = 제거, TODO 주석만** (열린문제 4 참조).
- 진단 로그: `detect` 가 후보별 `height/base_z/top/pos/prior통과` 를 찍음.

**열린 문제 (오늘 제대로 못 푼 것 — 다음 세션 우선순위 순):**

> **2026-07-14 갱신 — #1/#2 설계 확정 + 시뮬 재현으로 원인 규명.** detection/grasp 을
> **object-centric(물체 점군 + 멀티뷰, floor 제거) + reachable-orientation 파지**로 재설계 확정.
> **파지 SSOT = [grasping.md](grasping.md)** (설계/근본원인/해결/검증/히스토리 통합 —
> 다른 세션에 이 문서 하나 주면 파악 가능). perception.md
> 는 요약 포인터.
> - **#1** = floor 뺄셈 폐기로 해소 (phantom·height 노이즈 클래스 소멸).
> - **#2 정정**: "IK 오판" 아님 — **시뮬 캘 적용 재현(FK sub-mm 일치)으로 IK/솔버 정상 확인**,
>   `plan_grasp` 의 **top-down 강제**가 범인 (그 리치에서 top-down 자세 자체가 도달 불가, 비스듬한
>   접근은 됨). 로드맵 step 1 = grasp 자세를 reachable-orientation 으로 (IK 솔버는 안 건드림).
> 아래 원문은 그 논의의 출발점(실물 로그 근거)이라 보존.

1. **검출 height/base_z 측정이 부정확 — 근본 원인 조사 필요 (threshold 낮추지 말 것).**
   실물 로그: 같은 "white small round cube" 를 search 자세마다 **height 0.5cm ↔ 1.5cm**
   로 제각각 재고(뷰 간 편차 큼), 매번 **base_z −0.23m(책상보다 23cm 아래) / height 19cm
   짜리 phantom 후보**가 하나씩 낌. 즉 depth→base frame 투영(`detector/module.py`
   `object_top_center_base` / `floor_z_and_height`)이 **불안정한 height 를 산출**.
   → **height prior 하한(0.015)을 낮춰 덮는 건 band-aid (사용자 반려).** 조사할 것:
   D405 depth 품질(거리/각도 — search 자세가 D405 최소거리~7cm 를 어기나?), base_z(주변
   바닥 ring) 추정 방식, hand_eye/intrinsic/TCP 투영 정확도, phantom(base_z −0.23m)이
   나오는 픽셀이 어디인지. **작은 물체 높이를 안정적으로 재는 게 목표** — 임계 조정 아님.
   **더 근본 (사용자 지적):** height prior(하한/상한) 라는 하드코딩 크기 창(window) 존재
   자체가 잘못된 설계다 — 물체 크기는 매번 다른데(임의 물체를 집어야 함) 고정 창을
   두는 건 "제대로 인식 못 하니 크기로 걸러내자"는 crutch. 제대로 된 per-object 3D
   size/pose 추정이면 그 창이 필요 없다. 지금 prior 는 (a) 작은 물체 하한 탈락 + (b)
   phantom(−0.23m) 걸러내기 두 역할인데, phantom 은 **크기로 필터할 게 아니라 그 잘못된
   depth 투영을 소스에서 고칠 perception 버그**다. 방향 = 크기 무관 안정 인식 → prior 제거.

2. **resolve_grasp 가 물리적으로 도달 가능한 pose 를 "불가"로 오판 (solver vs 현실).**
   큐브 (0.275,0.208) 에서 44개 접근 후보 전부 IK 불가 → 실패했는데, 사용자가 **토크오프로
   그 자리에 팔을 실제로 갖다 댐** (화면 관절 J1..J6 = −1.07/2.26/−1.82/0.81/0.61/0.64rad,
   TCP≈(0.241,0.179,−0.033)). **그 관절값은 URDF joint limit 안에 전부 있음**(확인함:
   joint1±1.5708 / joint2[−0.17,2.62] / joint3[−3.05,−0.087] / joint4·5±1.518 /
   joint6[0,3.14]). 즉 **단순 joint-limit-too-tight 아님.** 미조사 근본원인 후보: (a)
   self-collision 오판(OMX link6↔link7 전례 [[project_omx_urdf_link6_link7_penetration]]),
   (b) pre-grasp(6cm 위+tilt) pose 가 범인, (c) 파지 자세(조 축 수평+tilt) 조합이 현재
   토크오프 자세와 다른 관절해를 요구해 거기서 한계/충돌, (d) IK seed/budget.
   → **필요한 것: `resolve_reachable`(motion/module.py)에 후보별 어느 pose(pre/grasp)가
   왜(관절리밋/self-collision/수렴실패) 깨졌는지 로깅** → 근본 짚고 그걸 고침(물체 옮기라
   금지). 원칙: [[feedback_verify_solver_not_reality]] — 도구가 "불가"래도 물리가 "가능"이면
   도구를 의심.

3. **#3 MoveL 매끄러움(EMA 제거) 실물 미검증.** 진단은 로그 기반, 수정(직접 명령)은
   실 로봇에서 안 돌려봄. EMA 없이 특이점 근처 IK 튐이 실측되면 **lag 필터 재도입이 아니라
   IK 연속성을 고치는 게 정석.** 또 MoveL 루프 dt 6~28ms 지터(목표 20, Windows sleep) —
   EMA 제거 후에도 남는 별개 문제(중요도 낮음, 필요시 sleep 정밀도).

4. ~~**미리보기(#1) 설계 미정.**~~ → **해결 (2026-07-14 — 위 항목 참조).** "실행
   보장" 요구를 "존재하는 구조 표시"로 완화하니 정적 소스 읽기로 충분해짐 —
   실행/모킹 0, `tasks/core/preview.py`. (당시 dry-run+가짜응답 `_preview_responders`
   는 "개발자가 프리뷰용 보일러플레이트 쓰는 건 나쁜 DX" 반려로 제거된 상태였음.)

**이번 세션 진행 방식 회고 (assistant 가 오늘 반복한 잘못 — 다음 세션 먼저 읽고 반대로 할 것):**
사용자가 하루 종일 "생각이란게 없어?", "정석이야?", "묻지마", "내 코드 아니라고 그러지마",
"문서에 적으랬지" 를 반복하게 만든 원인. 코드를 많이 뱉는 게 잘하는 게 아니다.

1. **코드 전에 생각 안 함 — 증상에 땜빵만 내밈.** 검출 height 가 틀리면 height prior
   하한을 낮추자 했고(→ 사용자: "제대로 인식을 못 하는 게 문제지 크기 창이 왜 있냐"),
   MoveL 이 안 부드러우면 EMA alpha 0.1→0.5 를 내밈(→ "그게 정석이야?"). **근본(왜 depth
   투영이 틀리나 / 왜 EMA 라는 lag 필터가 있나)을 먼저 파야 함.** 임계·필터·계수 조정 =
   전부 band-aid.
2. **"정석이야?" 를 방어로 받음.** 이건 "멈추고 근본·표준을 찾으라"는 신호다. 방어·정당화
   금지.
3. **도구를 믿고 현실을 무시.** resolve_grasp "도달 불가" 를 믿고 "물체를 옮겨라" 했는데,
   사용자가 토크오프로 팔을 실제로 그 자리에 갖다 댐. 도구가 "불가"래도 물리 증거가
   "가능"이면 **도구를 의심** ([[feedback_verify_solver_not_reality]] — 이미 있는 교훈인데 또 어김).
4. **책임 회피.** "이건 내 코드 아니다 / 오늘 코드 아니다" 를 반복 → 사용자 폭발. 누가
   짰든 지금 같이 고친다.
5. **설계 확정 없이 코드를 왔다갔다.** 미리보기 하나를 하루에 dry-run→정의목록→AST→선언형
   →제거 로 다섯 번 갈아엎음. 트레이드오프는 **먼저 사유로 좁히고** 코드는 확정 후 한 번.
6. **프레임워크가 이미 하는 걸 손으로 재발명.** `@step` 이 trace/게이트를 자동 제공하는데
   `_preview_responders` 로 서비스별 가짜응답을 손코딩 = @step 취지 역행 + 나쁜 DX.
7. **묻지 말라는데 계속 물음.** 근본을 사유로 정한 뒤 결정은 스스로, 진짜 사용자만 아는
   분기만 물을 것 ([[dont-ask-too-many]] [[feedback_wait_for_explicit_implement]]).
8. **사용자 지시를 문자 그대로 안 들음.** "문서에 적어" 라 했는데 메모리에 적는 등. 지시
   대상·위치를 그대로 따를 것.

**앞으로 수정: (a) 코드 전에 근본원인·정석을 사유로 먼저 → 확정 후 한 번 구현, (b) 임계/
필터/계수 조정 같은 땜빵 금지, (c) solver 보다 물리 증거, (d) 책임 회피·변명 반복 금지,
(e) 지시를 문자 그대로.** (동일 회고 = 메모리 [[think-root-cause-not-bandaid]].)

### 2026-07-13 (밤/2) — MoveJ 통합 (MoveJ_pose 흡수, target discriminated union)

**진단 (사용자↔GPT 토론):** `MOVE_J`(관절값) / `MOVE_J_POSE`(pose→IK) 두 서비스는
산업 로봇 관습(UR `movej(q|pose)` — 한 명령, 인자 타입으로 분기)과 어긋남. 판별
기준은 **planner 동일성**: MoveJ vs MoveL 은 planner 가 달라(관절 보간 vs Cartesian
직선) 못 합치지만, JointTarget vs PoseTarget 은 planner 같고 **목표 표현만** 달라
같은 MoveJ 안이 논리적. (근거는 "산업표준이라서"가 아니라 계약 의미 — "MoveJ =
관절 보간으로 이동" 이면 target 표현은 입력 방식일 뿐.)

- **contract**: `MOVE_J_POSE` 서비스 + `MoveJPoseRequest` 삭제. `MoveTarget =
  Annotated[JointTarget | PoseTarget, Field(discriminator="kind")]`. `MoveJRequest
  {target: MoveTarget}` / `MoveLRequest {target: PoseTarget}` (직선은 pose 만 —
  joint 직선 무의미). `tool_offset` → **`tcp_offset`** 개명 + **PoseTarget 로 이동**
  (제어점 선택 = "어느 점이 target 에 닿나" = 도달 명세(Reach Spec)의 일부, goal 정의.
  seed_joint/redundancy 같은 solver hint 는 "어떻게 푸나" 라 target 아님 — 생기면 별도
  options 로. 판별 규칙: **다른 데 서나(→Target) vs 같은 데 다르게 가나(→options)**).
- **handler**: `move_j` 가 `match req.target` (JointTarget→직접 / PoseTarget→IK).
  `move_j_pose` 삭제. tcp_offset 보정은 `_corrected_target_pos` 공유 헬퍼로
  MoveJ/MoveL 공용 (제어점 보정은 planner 무관). MoveL 도 PoseTarget 이라 tcp_offset
  일관 지원.
- **호출부**: steps.py `_move_j_pose`(→MOVE_J+PoseTarget)/`_move_l`(→PoseTarget),
  frontend WaypointPanel(`{target:{kind:"joint",joints}}`), 관련 테스트 전부.
- **kind 는 required 판별자** (default 없음) — 와이어 decode 가 kind 로 arm 선택, 타입도
  정직하게 required (프론트 누락 footgun 방지).

검증: backend `test_motion` **14 PASS**(sim — 실 IK: MoveJ joint/pose + MoveL + tcp_offset
경로) + task/contract 비-sim **59 PASS** + ruff·pyright 0 / contract regen(MoveJPoseRequest
소멸, JointTarget/PoseTarget 추가) / frontend tsc·lint·vitest(WaypointPanel+regen) 통과.

### 2026-07-13 (밤) — STEP_RESULT/step_note/ctx.record 제거 + task-owned 마커 통로(B)

**진단 (사용자):** `step_note`/`ctx.record`/`STEP_RESULT` 가 설계 논의 없이 얹힌
채널이었음 → 우선 걷어내고, 정말 필요하면 설계부터. 로그가 필요하면 표준
`logging.getLogger(__name__)` 로.

- **제거**: `core/step.py` `step_note`+`_CURRENT_ENTRY`+`RunLink.emit_result`,
  `core/context.py` `record`+`dump_value`, `core/runner.py` `on_result`/
  `_notify_result`/`_Link.emit_result`, `core/contract.py` `TaskStepResult`,
  `pick_and_place` STEP_RESULT publish/`_publish_result`, `FRONTEND_EXPOSED`
  STEP_RESULT, 프론트 `TaskResultsOverlay`(+test). runner 콜백은 이제 `on_state/
  on_trace` 둘. `TraceEntry.detail` 은 **실패 사유 전용**으로 축소 (성공 요약 안 채움).
  steps.py 의 검출 수/선별 그룹 요약은 `logger.info` 로.
- **검출 시각화는 무관/무영향**: 카메라 패널 AABB/OBB/세그는 detector 가 자기
  스트림(DETECTIONS/DETECTIONS_ORIENTED)으로 직접 발행 (task 무관). ctx.record
  제거와 별개 — 그대로 동작.
- **task 고유 계획 마커(파지/적치)는 (B)로 재설계**: "시각화 데이터는 그걸 계산한
  쪽이 소유"(detector 동형). pick_and_place 가 자기 typed 스트림 `MARKERS`
  (`stream/pick_and_place/{robot_id}/markers`, payload `TaskMarker{label,position}`
  + `TaskMarkers{robot_id,seq,timestamp_unix,markers[]}`) 선언·발행. 모듈이
  scenario 에서 pick→grasp/place→drop 지점을 스냅샷(latest-wins)으로 publish,
  프론트 `TaskMarkersOverlay` 가 구독(STATE RUNNING 전이 시 clear). 범용
  프레임워크 미신설 — "통로가 있다"까지만, 중복 생기면 그때 헬퍼 (task-first).
  네이밍: `...Update`(델타로 오독) 대신 `TaskMarkers`(스냅샷).

검증: backend `test_task_context/test_task_runner/test_pick_and_place/
test_contract_export/test_contract_draft` **63 PASS** + ruff·pyright 0 / contract
regen (STEP_RESULT/TaskStepResult 소멸, MARKERS/TaskMarker(s) 추가 — fixture+
contract.ts 재생성, gen-contract byte-동형 vitest 통과) / frontend tsc·lint(기존
경고 1)·scene/task/detection vitest **42 PASS**. **실물 마커 표시 검증 = 다음
hardware 세션.**

### 2026-07-13 (후반) — runner wire 절단 + 등록 의식 전폐 + ctx 단일 표면

**핵심 진단 (사용자):** "runner 가 wire 를 발행하는 한, 계약이 runner 의 사정이
된다" — 하루 종일 반복된 계약↔프레임워크 엮임(entry=/params=/task_surface/
TASK_INFO/streams=/TaskMeta)의 근본 원인. 수술 = runner 의 wire 절단.

- **TaskRunner = wire 무지 범용 감독기**: runtime/키/robot payload/seq/fan-out
  전부 제거. 변화는 생성자 **콜백 3개**(on_state/on_trace/on_result — 전부 선택,
  안 달면 headless)로 통지 — RunState 스냅샷/TraceEntry/record 값. 콜백 예외는
  삼키고 로그. (listener 객체/EventEmitter/TaskHost 계층 검토 후 기각 — 콜백이
  "내부 전이 지점이 미지의 외부에 닿는" 최소 형식. 알림 코드 소유는 이미 모듈,
  호출 지점은 전이가 일어나는 runner 내부에만 존재 가능.)
- **진행 발행 = 모듈 소유**: 모듈이 runner 콜백에 자기 발행 메서드를 담
  (TaskState/TaskTrace/TaskStepResult 조립 + seq + robot fan-out — 자기 계약
  키로). payload 규약은 core/contract.py — 공용 task UI 의 전제.
- **등록 의식 전폐**: register_task/TaskMetadata/registry/@task 데코레이터/
  task_surface 생성기/**GET /tasks(TaskInfo/TasksResponse) 삭제**. task 의 정보
  채널 = 계약이 유일 (frontend 는 gen:types 로 키를 정적으로 앎). robot 바인딩/
  표시 문구 = frontend task 전용 페이지 소유 상수 (pickAndPlaceTask.ts —
  useTasks/useTaskRobotId 삭제). 조작판(stop/pause/...) wire 노출도 **모듈 결정**
  — 계약에 명시하고 핸들러 손코드 (runner API 를 코드에서 직접 불러도 됨).
- **ctx 단일 호출 표면 (RobotHandle 삭제)**: 표면이 둘이면 "어느 쪽으로 부르지?"
  가 오용을 낳음 (detector 를 robot.call 로 부른 실사고). `ctx.call(key, req,
  res, robot_id=)` 하나 — robot-scoped 키는 robot_id= (호출마다 **참여 명부
  검증** — 선언 밖 robot 명령 즉시 에러 = on_abort STOP 커버리지 보장), agnostic
  은 req 필드 (§2.7. agnostic 키에 robot_id= 주면 fail-fast). `ctx.spec(robot_id)`
  물리값. steps 시그니처 = (ctx, robot_id: str, ...) — robot 은 리터럴 id.
- **병렬 = all-stop 의미론으로 지원** (회귀 테스트 잠금): gather 가지들 —
  pause/breakpoint 시 전 가지가 각자 다음 경계에서 hold (공유 게이트 — gdb
  all-stop 등가), cancel 가지 전파, depth ContextVar 가지별 독립. 미장착(additive):
  가지 단위 step_once, current_name 복수 표시, UI robot 트랙.
- **run_task.py = 키 직접**: `run_task.py srv/pick_and_place/run --param k=v` —
  registry 의존 소멸, param 검증은 서비스(RunRequest)가 SSOT (오류 사유 그대로
  출력), 스트림은 키 ns + wildcard 구독 (robots 목록 불필요).

검증: backend pytest full **348 PASS** (병렬 all-stop/취소 전파/ctx 검증/headless/
콜백 예외 격리 신규) / ruff·pyright 0 / run_task 워크스루 (중첩 trace + 서버측
param 거부 확인) / frontend vitest 160 + lint·build + **e2e 2/2** / contract
regen (TaskInfo/TasksResponse 소멸 확인). 유령 서버 정리 확인.

### 2026-07-13 (전반) — @step 개편: 저자 지정 step + 거부=raise + contract timeout 선언

**설계 (사용자↔GPT 교차 토론 수렴 — docs/task.md 상단 정본 개정):** step 을
프레임워크 강제 primitive 에서 **저자가 @step 으로 지정하는 함수 단위**로 전환.
계층 = 시나리오(step 나열) → @step 함수(raw service call — **contract 가 SDK**,
client/미러 래퍼 계층 기각) → `ctx.call` → 서비스.

- `core/step.py` 신설 — `@step` / `@step(title="집기")`. **name(식별자 —
  breakpoint/run_to 키) = 함수 이름** (override 파라미터 없음 — 함수 이름이 이미
  안정 식별자), **title = UI 표시 문구** (분리 — 문구 바뀌어도 name 안정). 필드
  네이밍 확정 (2026-07-13 후반 토론): 식별자를 `label` 이 아니라 `name` 으로 —
  "이 step 을 식별하는 안정적 이름" 의미가 코드/wire/frontend 전체에서 일관 (label
  은 관례상 표시 텍스트라 혼동). wire: `TraceEntry.name/title`,
  `TaskState.current_name/current_title`, `RunToRequest/ToggleBreakpointRequest.name`.
  (`TaskStepResult.label` 은 ctx.record 키로 별개 개념 — 그대로 유지.) **중첩 허용**: compound step (pick/place) 안의 자식
  step 이 trace 에 depth 로 찍힘 (wire = flat 리스트 + `TraceEntry.depth` 하나,
  트리는 UI 들여쓰기). 게이트는 모든 진입점 = step_once 는 step-into (over 버튼
  없음 — run_to 가 커버). 링크는 ContextVar (runner._supervise 가 bind — 병렬
  확장 안전). run 밖 호출 = 게이트 없이 본문만 (step 함수가 그냥 async 함수로
  테스트됨). `step_note()` 로 실행 중 detail ("3개 후보") 보존.
- **거부 = raise (motion/motor contract 개정, wire 가시 변경)**: MOVE_J/
  MOVE_J_POSE/MOVE_L 거부 → `MotionRejected` raise (응답에서 accepted/message
  제거 — 빈 응답), SET_GRIPPER ok 필드 제거. 기준 확정: **예외 = 기술적 실패
  (서비스가 raise → RemoteError), 데이터 = 부정적 유효 결과** (검출 0개,
  RESOLVE_REACHABLE -1 유지 — 치명 판정은 step 이 NoReachableGrasp 로). task 에
  accepted 체크 코드가 존재하지 않는 게 계약.
- **SELECT_REACHABLE → RESOLVE_REACHABLE rename** (2026-07-13 후반): 계약이
  "순서 = 선호 힌트(best-effort), 가용 그룹 하나 반환" — 엄격 first 보장이 아님
  (deepening 이 속도와 맞바꾼 문서화된 트레이드오프). "Select" 는 motion 이 선택
  정책을 소유하는 듯 오독, "First" 는 구현이 보장 않는 것을 약속 → Resolve.
  배치 자체는 motion 이 정본 (기구학 판정 = motion 만의 지식 + in-process batch
  성능 — 2026-07-09 원격 probe 10s 사고. 정책 = 후보·순서·전멸 판정은 task 유지).
  frontend 미노출 서비스라 regen 불필요. FIRST/ALL/BEST 확장은 소비자 생길 때.
- **timeout = contract 선언**: `framework.contract.service.declare_service_timeouts`
  — 각 contract.py 가 자기 서비스 기본 timeout 선언 (motion 60s / detector 30s),
  `runtime.call(timeout=None)` 이 template 키로 해석. 상충 재선언 = fail-fast.
- **RobotHandle 축소**: primitive 메서드 (detect_oriented/move_l/gripper 등) 전부
  삭제 — robot_id 주입 `call` + `spec`/`require_spec()` 만. core 의 도메인 import
  는 Motion.STOP 하나 (안전 의무). **on_abort = 참여 robot 전원 STOP** (moved
  추적 폐기 — 보수적 안전). `ctx.wait` 삭제 (step 안 asyncio.sleep).
  `FakeContext` 재설계 — ScriptedRuntime (서비스 키별 응답/예외 스크립트).
- **pick_and_place 재작성 (표준형 갱신)**: module.py `_scenario` = `steps.pick` →
  `steps.place` 두 줄. steps.py 신설 — 한글 title 붙은 @step 14개 (compound
  pick/place + detect/select/모션/gripper 원자 step + plain 헬퍼 _move_l 등).
  settle(1.2s)/raw 값은 step 파일 소유. step 재사용은 실제 필요가 생길 때 공용화 (어휘집
  선축조 금지).
- **frontend**: contract regen (fixture+contract.ts — TraceEntry.name/title/depth,
  TaskState.current_name/current_title, RunTo/ToggleBreakpoint.name, 빈 Move 응답),
  TaskProgressPanel 이 `{ name }` 으로 run_to/breakpoint 호출, WaypointPanel move_j 를 res.success
  기반으로 (accepted 소멸), TaskProgressPanel depth 들여쓰기 + title 표시
  (접기/펼치기는 후속 polish).

검증 (2026-07-13 전부 실행): backend pytest full **346 PASS** (신규: @step 게이트
/depth/중첩 실패 경로/step-into/title/timeout 해석 등) / ruff·pyright 0 (기존
calibration 3건도 타입 주석으로 해소) / frontend vitest **160 PASS** + lint(기존
경고 2 외 0)·build 통과 / run_task mock 워크스루 — 중첩 trace(`pick[집기] >
detect[검출]`)·step_note·실패 사유 조립·STOP 경로 실물 확인 (mock 은 캘 없어
detect 0건 자연 실패 = 실패 경로 검증). **실물 완주 검증 = 다음 hardware 세션.**

다음 후보: ① 실물 pick(+place) 검증 ② 두 번째 task 개밥먹기 (사용자 직접 작성 —
표면 확정판으로) ③ trace 접기/펼치기 + 병렬 robot 트랙 UI polish ④ 가지 단위
step_once (병렬 디버거 세밀 제어).

### 2026-07-12 — Task 아키텍처 확정 + 옛 DSL 통삭제 + Pick&Place 본 계약 승격

**설계 (사용자 논의 수렴 — docs/task.md 개정판이 정본):** task = 당당한 모듈
(자기 contract/서비스/구독/발행 소유), 프레임워크는 `modules/tasks/core/` 의
**부품** (상속/자동배선/@task 데코레이터 전부 기각 — 조합만):

- `TaskRunner` — 실행 생명주기만: start(fire-and-monitor)/cancel(**in-flight
  await 즉시 끊김** — 옛 runner 의 step-경계 stop 결함 해소)/pause/step_once/
  run_to/toggle_breakpoint(**label 기준**), 예외→FAILED+사유 조립, STATE/TRACE/
  STEP_RESULT 발행 (키는 모듈이 `streams=` 로 주입). robot 은 id 문자열만 앎.
- `TaskContext`/`RobotHandle` — 도메인 접근: `ctx.robot("so101_6dof_0")` handle 로
  primitive (detect_oriented/select_reachable/move_j_pose/move_l/gripper — timeout/
  wire/typed 예외 내장), escape hatch 2종 (robot.call = robot-scoped 주입 /
  ctx.call = robot 무관), **모션 보낸 robot 추적 → on_abort 시 그 robot 에만
  Motion.STOP**. 시나리오 규칙은 둘뿐: "ctx 받는 async 함수" + "실패는 raise".
- `core/contract.py` — STATE/TRACE(TREE 폐기 대체)/STEP_RESULT payload 규약
  (파일명이 contract 인 이유 = contract_export 가 정의 클래스만 카탈로그).
- `TaskMetadata` + registry — GET /tasks (param 스펙은 **typed RunRequest 에서
  자동 파생** — 손 목록 금지), bridge 는 tasks_provider 로 요청 시점 평가.
- `FakeContext` — 실 ctx 상속+동일 시그니처 override (pyright 드리프트 방지),
  시나리오 로직을 wire 없이 검증.

**pick_and_place 본 계약 승격** (modules/tasks/pick_and_place — task 모듈 표준형
= 다음 task 의 레퍼런스): contract.py (표준 표면 7 서비스 + 3 스트림, typed
`RunRequest{pick_object, place_object=""}`, 디버거 키는 선언한 task 만 생성) /
module.py (핸들러 one-liner 위임 + `_scenario` — 2026-07-09 실기 검증 시퀀스) /
geometry.py (순수 함수 — tilt×yaw×flip 후보·고정 조 횡보정·height prior 이식 +
**plan_place 신설**: place_object 검출 대상 상면 적치, 파지 lateral 재사용).
gripper raw 는 resolve 의 motors.yaml 투영(TaskRobotSpec) 재사용 — 하드코딩 소멸.

**삭제:** modules/task (DSL 전체 — Step/Slot/TaskRunner/registry), PREVIEW/TREE
계약, frontend /tasks 페이지·PromptPanel. **frontend:** /tasks/pick_and_place
전용 페이지 (task 별 페이지 원칙) + PickAndPlacePanel (파싱→typed 폼→실행/중지)
+ TaskProgressPanel (TRACE 기반, breakpoint/run_to=label) + TaskResultsOverlay
(STEP_RESULT label 키, 새 run RUNNING 전이 시 clear). **CLI:** scripts/run_task.py
(in-process mock 부팅, bridge 제외 — 터미널 작성 루프).

검증 (2026-07-12 전부 실행): backend pytest full **336 PASS** (신규: runner 게이트/
cancel-in-flight/on_abort 대상 정밀/FakeContext 시나리오/geometry 경계 52개) /
ruff·pyright 신규 0 (calibration 기존 3건 별도) / frontend tsc·lint 0 + vitest
**160 PASS** / Playwright e2e (mock, headed) **pick_and_place 2/2** — 실행→trace→
FAILED 사유 표시, breakpoint→PAUSED hold→STOP 탈출. mock 완주는 불가 (in-memory
DB 라 캘 없음 → detector 후보 0 → 자연 실패 = 실패 경로 검증). **실물 완주 검증
(place 분기 포함) = 다음 hardware 세션.**

다음 후보: ① 실물 pick(+place 신설분) 검증 ② handover task (사용자 개밥먹기 —
robot 2대 ctx 실행 모델 + 모듈 간 robot lease 그때 설계) ③ `@step` 데코레이터
(관측 단위 span — 논의만 됨, 골격 굳은 뒤).

### 2026-07-07 — liveliness + Mirror 활성 (부팅 순서 종속성 근본 제거)

분산에서 PC(calibration) 늦게 뜨면 motion 이 **무보정으로 조용히 영원히 운전**하던
설계 구멍을 프레임워크 레벨에서 제거 (정본 = backend.md §3.3 배너 + anchor #2/#9/#23):

- **L1 Transport**: `declare_liveliness` / `subscribe_liveliness` (zenoh liveliness
  token, history=True). 4전이 실검증 (사전존재/undeclare/재선언/세션 crash) →
  `test_transport.py::test_liveliness_presence_lifecycle`
- **L2 Runtime**: service 등록 시 같은 key 로 token **자동 선언** (Mirror 가 구독)
  (모듈이 "나 떴어요" publish 하는 손 컨벤션 금지 — 부팅 순서 = distribution 문제 =
  framework 책임)
- **L3 Mirror 완성**: owner liveliness 구독 (늦은 부팅/재시작 자동 refetch 수렴) +
  `@mirror.on_change(old,new)` 훅 (실제 값 변경 전이만 발화) + refetch 직렬화.
  → `test_mirror.py::test_mirror_converges_when_owner_boots_later`
- **L4 Motion = 첫 Mirror consumer**: start() blocking fetch 삭제 → mirror peek.
  없음→값 = runner idle 때 **live 적용** / 값→값′ = `calibration_stale` 표시만
  ("변경은 재부팅" 유지). `TcpState.calibration_applied/stale` 상시 표면화 +
  frontend LivePointCloudPanel "robot FK" 배지.
  → `test_motion_calibration.py::test_motion_converges_when_calibration_owner_boots_later`
- 같은 날 오전: bridge WS **CONNECTING 창 service 프레임 silent drop** 수정 (버퍼→
  open flush, `bridge.test.ts` 회귀 가드) — tasks e2e 파싱 실패의 근본 원인이었음.

검증 (전부 실행 확인, 2026-07-03):

| 층 | 결과 |
|---|---|
| backend pytest | **212 PASS** (모듈별 so101 6DOF + omx 5DOF multi-robot 눈속임 방지 포함) |
| ruff / pyright | 0 / 0 |
| frontend vitest / lint / tsc | **47 PASS** / 0 / 새 에러 0 (pre-existing jest-dom 2건만 — [[project-frontend-v2-build-prexisting-fail]]) |
| **Playwright e2e (headed)** | **14/14** — jog 50Hz full wire / calibrate `CALIB_SIM_BOARD=1` capture over-wire / scan 세션+캡처 / waypoint 티칭+group / contract-graph 9노드 |
| mock 실부팅 | 전 Module host-level/scoped 정상 add+start, 에러 0 |

**집 하드웨어 검증 (2026-07-02)**: frontend → backend wire → 실 SO-101 **TCP jog**
동작 확인 (C2 transport + JogTcp→IK→feetech + 토크 enable).

| 영역 | 상태 |
|---|---|
| framework (contract/runtime/transport/persistence/storage/Mirror/liveliness) | ✅ (Mirror 활성 2026-07-07 — 첫 consumer Motion.calibration, spec §3.3 + anchor #23) |
| infra (zenoh / sqlite·postgres / fs·minio) + 루트 alembic | ✅ |
| motor (mock + 실 feetech) / camera (mock + realsense_d405) / camera_decoded | ✅ (실 feetech TCP jog 검증됨. realsense·PID/profile 미검증 — 아래) |
| motion — D1 kinematics(dof=6) / D2 MoveJ+TCP state / D3 Jog / **MoveL v1 + await-complete 완료 계약** | ✅ (spec §17.3) |
| calibration — persistence/capture/preview/factory-seed + offline 분석 흐름 | ✅ (capture 는 sim-image — 실 D405 미검증) |
| detector — `Detect Object` (mock backend, 투영 수학 단위검증) | ✅ (GDINO 실 모델 = 슬라이스 3, 집) |
| scene3d / scan (TSDF build 포함) / waypoint | ✅ |
| bridge (WS relay + MJPEG + HTTP + /contract.json + /contract/graph) + frontend contract gen | ✅ |
| **robot-agnostic 스코프 리팩터** (detector·calibration·scan·scene3d·waypoint → host당 1) | ✅ (2026-07-03 — 규칙은 spec §2.7, 아래 히스토리) |
| **task 프레임워크 (tasks/core) + pick_and_place task 모듈** | ✅ (2026-07-12 — 위 항목. 실물 완주 검증 대기) |
| Gamepad Module | 미착수 |

**검증 명령** (cwd 반드시 `backend/`):
```bash
cd backend
uv run --no-sync pytest -q                          # 265 passed (~75s)
uv run --no-sync ruff check . && uv run --no-sync pyright
uv run --no-sync python -m apps.main --host mock    # 실 boot (:8000)
# frontend: cd frontend && pnpm vitest run && pnpm lint
# e2e: mock backend(CALIB_SIM_BOARD=1) + pnpm dev(:5174) 띄우고 pnpm test:e2e (headed)
```

> **⚠️ 검증 게이트가 몇 분씩 hang 하면 → 유령 `apps.main` 확인** (2026-07-07 사고, [[project-verify-hang-stale-backend]]).
> 이전 세션이 `--host mock` 으로 띄운 backend 를 kill 안 하고 남기면 그게 :8000 을
> 계속 점유한다. 그 상태에서 pytest 를 돌리면 full-boot fixture 의 bridge 가 실패하고
> **pytest 요약은 찍히지만 프로세스가 종료되지 않는다** (`| tail` 버퍼링이라 출력조차
> 안 보임). 원인은 "테스트가 느림" 이 아니라 좀비 프로세스. 클린 상태 실측 = 전체
> 게이트 ~86s.
> - **구조 fix 완료** (재발해도 hang 대신 명확한 에러로 실패): bridge 소켓 pre-bind
>   (uvicorn `sys.exit(1)`→`RuntimeError("bind 실패")`), `Runtime.start` 실패 시
>   started 모듈 역순 rollback, 테스트 전부 `bridge_port=0` (ephemeral — 실 backend
>   와 공존). 회귀 가드 = `test_runtime::test_start_failure_stops_already_started_modules`
>   + `test_bridge::test_start_port_conflict_raises_clear_error`.
> - **운영 수칙**: 검증/실행용으로 띄운 backend 는 그 세션 안에서 반드시 kill.
>   장시간 무출력이면 프로세스 트리에 `apps.main`/`pytest` 잔존부터 확인
>   (`Get-NetTCPConnection -LocalPort 8000 -State Listen`).

## 아키텍처 불변식 (절대 어기지 말 것 — 포팅 시 [[feedback-port-keep-v2-arch]])

- **레이어링**: `modules/` 는 `apps/` import 금지. 다른 모듈 contract import 는 OK.
- **role 격리 (lazy registry)**: `apps/registry.py` = name→"path:Class" string lazy import,
  `apps/resolve.py` = branch 안 lazy import. eager import 금지 (test_boot subprocess 검증).
- **scope + robot_id 라우팅 = spec §2.7** — robot-scoped 4 (motor/camera/camera_decoded/
  motion) 외 전부 robot-agnostic. robot_id 는 키(주소) 또는 req 필드(파생 규칙) —
  Bridge 자동주입 금지. 새 모듈/서비스 추가 시 §2.7.1 3갈래부터.
- **raw↔rad = Motion 책임**. MotorDriver 는 순수 raw.
- **contract.py 컨벤션**: nested `Service`/`Stream`/`Event` StrEnum. stream/event payload
  에 `robot_id`+`seq`+`timestamp_unix` (spec §16.6). Stream key 는 채널 정의 모듈 contract 에.
- **Bridge = relay only** (spec §16.6) — domain logic 0.
- Motion = pi_motor 배치 (100Hz 명령 network 안 넘게). dof = arm only.
- **안전 수치 임의 금지**: limit=motors.yaml(실측), 속도=motion.yaml. 새 값 필요하면
  사용자에게 꺼내 보여줄 것, 추측 X.
- 테스트는 통과용 X — 실제 동작/invariant + spec ref docstring ([[feedback-meaningful-tests]], spec §15).

## 다음 작업 후보

1. **detector 슬라이스 3 — 실 GDINO backend** (`Detect Object` 의 구현체). 현재
   `apps/resolve.py::_detector_backend` 가 mock 만 배선 (real = NotImplementedError —
   그 메시지가 진입점).
   **2026-07-03 착수 — pyproject 까지만 완료 (uncommitted)**: `pc` 그룹에
   `transformers>=4.45,<5` + `accelerate` + `pillow` + `torch==2.11.0`
   (cu130 uv.sources/index, 옛 backend 동일 판). `uv sync` 아직 안 돌림.
   **transformers 상한 판단 (사용자 지적)**: 옛 backend 의 `<4.57` 핀 근거
   (meta tensor + `.to(device)` 깨짐) 는 **Qwen LLM 로드에서 관측**된 것 —
   GDINO 단독으론 분리 검증된 적 없고 v2 는 LLM 없음. 옛 결과를 근거로 인용하면
   거짓 권위 → `<5` 만 남김 (v5 의 `AutoModelForZeroShotObjectDetection` 제거는
   API 존재 문제라 확실). **smoke 때 최신 4.x 로 preload 검증 — 깨지면 그때
   실측 근거로 핀**. 남은 구현 순서:
   1. `modules/detector/gdino.py` 신규 — 옛 `backend/modules/detector/grounded_detector.py`
      포팅 (계약 = `detect(img, prompt) -> (bbox, score) | None`). **별도 파일** =
      torch/transformers import 를 mock 배치에서 격리 (motor/camera drivers 패턴 동형).
      load lock + transformers module-top import 유지.
   2. `backend.py` Protocol 에 `preload()` 추가 (Mock no-op).
   3. `module.py` — `start()` background preload (`asyncio.to_thread`) + `detect()` 의
      backend 호출도 `to_thread` (blocking 추론 → async 계약).
   4. `resolve.py::_detector_backend` real branch 배선 + `pc.yaml` 에 detector 추가.
   5. 테스트 (preload 배선 + real resolve 회귀) → `uv sync` → pytest/ruff/pyright →
      실 모델 로드 smoke (최신 4.x 에서 preload OK 확인 — 위 상한 판단의 검증 자리.
      깨지면 에러 실측 후 상한 핀 + 주석에 v2 측정 결과 기록).
   preload race 판단: 옛 race 는 LLM+GDINO **두** `from_pretrained` 동시 실행 전제 —
   v2 는 transformers 모델이 GDINO 하나라 전제가 구조적으로 없음. reproduction script
   ([perception.md](perception.md)) 는 두 번째 transformers
   소비자(LLM 포팅) 등장 시점의 프로토콜. 지금은 load lock + 단일 preload 경로로 보장.
   모델 로드/배선/mock 대비 회귀는 회사 가능, **검출 정확도는 집 하드웨어**.
   frontend 노출 필요 시 `FRONTEND_EXPOSED` 에 `Detector.Service.DETECT` 추가 + regen.
2. **PnP task (task-first — spec §17)** — ② 필요 primitive 정의 → task #1 을 async 함수 +
   디버거로. Day-1 primitive 중 MoveL·Detect Object(계약) 완료, 남은 것: Gripper 서비스 /
   VerifyGrasp / async runner+디버거 (spec §17.4) / detection Top-K+기하 prior (§17.5 —
   detector 슬라이스 3 과 자연 병행).
3. **Motion boot consumer** — Motion.start() 가 `snapshot_bundle` 읽어 kinematics build
   (link_offset patched URDF + joint/sag). calibration bundle wire 는 살아있음 — 미배선.
4. **offline BA 이월** — `calibrate_offline.py`(1722 LOC 5-stage BA) + `fk_chain.py` v2
   재배선 → 실 horibot.db run 으로 σ regression. capture→finalize 는 완성, BA 만 남음.
   (σ 0.818 재현 불가는 port 버그 아님 — 미기록 drop set, [[project-offline-ba-port-faithful]].)
5. **집 하드웨어 검증** — 아래 미검증 목록.

## 하드웨어 미검증 (집에서)

- `realsense_d405.py` (pipeline/align) — 아직 실 통신 안 해봄. 실 D405 intrinsic /
  ChArUco 캘 정확도 / scan TSDF 실물.
- feetech PID/profile write — motors.yaml `pid`/`profile` 가 실 모터 미적용 (driver 가
  EEPROM default 사용). 모션 느리거나 진동 시 wire (EEPROM write-once 주의).
- joint jog / cartesian MoveL 실물 / detector GDINO 실 모델 + preload race (reproduction
  script 먼저 — [[llm-preload-race]]).

## follow-up (blocking 아님)

- frontend framework store 의 agnostic 서비스 캐시가 robot 간 공유 (마지막 응답 wins) —
  패널이 robot 변경 시 refetch 라 기능 문제 아님. robot 별 캐시 분리는 실사용 시점에.
- omx `enabled: false` 라 mock fleet 투영 제외 — multi-robot **실부팅** 검증은 두 robot
  enabled 배포가 생기는 시점 (unit 층은 눈속임 방지 테스트가 커버).
- Playwright e2e CI 화 시 `CALIB_SIM_BOARD=1` backend 기동을 webServer 에 포함 (sim-board
  capture 테스트 skip 방지).
- latent (해당 step 진입 시): color+depth stream 페어링 (독립 seq) / Mirror refetch
  coalescing (consumer 등장 시) / Minio 예외·list semantics (Phase 3).

## 히스토리 (요지 — 상세는 git log)

- **2026-07-03 robot-agnostic 리팩터**: detector 구현 중 드리프트 발견 (설계 =
  robot-agnostic 인데 calibration 발 robot-scoped 가 복사 전파, 근거 없는 드리프트) →
  사용자 결정 "설계대로 되돌린다" → §2.7 라우팅 규칙 확정 (Bridge 자동주입 폐기 과정
  포함 — spec §2.7.3 폐기안) → calibration (최난도, 패턴 증명) → scan/scene3d/waypoint
  적용 → 전 층 검증 (위 표). mock 초기 자세 버그 fix (`MotorSpec.initial_raw` clamp —
  joint3 영점이 limit 밖) + so101 home/rest waypoint DB 삽입 동반.
- **2026-07-03 task-first 재정의**: "DSL 먼저" 폐기 → spec §17. 첫 task = 단팔 PnP.
  waypoint 모듈 (Robot Asset Layer 첫 자산) backend+frontend 완료.
- **2026-07-02 Calibration Step E 풀스택** + C2 (frontend 적응) TCP jog 실물 검증.
  CalibrationBundle = boot-time config 재분류 → Mirror consumer 0 (deferred).
- **2026-07-01 contract gen 파이프라인** (`/contract.json` EXPORT + gen-contract.mjs) +
  contract graph viewer (`/contract/graph` + React Flow).
- 상세 캘 도메인 결정 = [calibration.md](calibration.md),
  frontend = [frontend.md](frontend.md), framework 결정 history =
  [backend.md](backend.md).


---
---

# 부록 — 통합 원문 (2026-07-11 문서 다이어트)

> 아래 문서들을 본 문서 부록으로 병합 (원문 그대로):
> - `backend.md`


---
---

<!-- ═══════════ [통합 원문] backend.md ═══════════ -->

# Pick & Grasp / Motion 디버깅 handoff (2026-07-07~08 세션)

작은 흰 큐브 pick&place 가 계속 못 집던 문제를 파고든 세션 기록 + 남은 문제/버그 +
다음 세션 할 일. **모든 변경은 working tree 에 있고 커밋 안 됨** (사용자 검토 후 결정).

---

## 0. 한 줄 요약

`pick_and_place` 가 큐브를 못 집는 문제를 파다가 **접근 primitive 를 MoveL(Cartesian
직선) → MoveJPose(관절 공간) 로 전면 재설계** + IK 어댑터 **multi-restart** + **pinch
offset** 도입까지 왔음. 한 번 집혔지만 **운**이고 아직 신뢰성 없음. detection 이 가끔
엉뚱한 데 찍히고, 모션이 급하고, verify_grasp 가 별개 버그로 실패함.

---

## 1. 이번 세션 진행 히스토리 (어떻게 여기까지 왔나)

1. **시작**: `ApproachAlongTool` 의 MoveL 이 `MotionFailed: MoveL failed` — trajectory
   FAILED. 원인 = cartesian loop 의 IK 실패 (`MoveL IK 실패 | s=2.2cm`).
2. **오진 1 (내가)**: "SO-101 워크스페이스 한계로 도달 불가" 라고 docstring("FK 20만
   샘플, 수직 z≤0.05") 을 그대로 읊음. → 사용자 반박: "J2 150° 까지 가는데 왜 불가?"
3. **진짜 원인**: 실제 보정 kinematics 로 heavy sampling(seed 400) 해보니 — **위치는
   전부 도달**(position-only IK OK), 문제는 **PyBullet `calculateInverseKinematics`
   가 seed 1개짜리 local 솔버라 존재하는 해를 놓침**. straight-down 14° 자세도 seed
   400 개면 나옴. → **reach 문제 아님, IK 솔버 robustness 문제.**
4. **핵심 재정의 (사용자 주도)**: 목표는 "그리퍼를 -Z 수직으로" 가 아니라 "**큐브를
   잘 집는다**". 필요없는 자세 제약을 걸어 도달 가능한 위치의 IK 를 스스로 죽이고
   있었음. + MoveL(자세 고정 직선)은 **높이마다 도달 자세가 변하는** SO-101 에서
   구조적으로 실패 (heavy sampling 으로 확인 — 같은 자세로 z 2.3cm 만 OK, 4.3cm+ FAIL).
5. **재설계 (사용자 주도)**: "MoveL 에 갇히지 말고 다양한 primitive 써라. lift 도 월드
   Z 상승만이 답이냐?" → **pick 전체를 관절 공간 config 전이로**. approach/grasp/lift
   전부 `MoveJPose`(pose→IK→run_joint). lift = "위 config 로 복귀 = 자연 상승".
6. **단일 jaw 발견 (사용자)**: 그리퍼가 좌우 대칭이 아님 — **파란 jaw 고정, 검은 jaw
   (joint7) 만 회전**해 닫힘. 물체는 고정 jaw 쪽으로 눌려 잡힘. → tcp ≠ 파지점 →
   **pinch offset** 필요 (tool-frame, 그리퍼 상수, 큐브 무관).
7. **detection projection fix**: bbox 중심 픽셀 × 윗면 depth **불일치** → 파지 x/y 가
   카메라 쪽 모서리로 편향. → **윗면 픽셀 3D centroid** 로 교체.
8. **gripper URDF 시각화**: 프론트가 arm(TcpState)만 받아 gripper 안 움직임 → motion 이
   gripper rad 를 TcpState 별도 필드로 report.
9. **결과**: 한 번 집힘 — 근데 **운** (사용자: "운좋게 집은거야, 디텍팅도 가끔 이상한데
   찍힘"). 아직 미해결.

**교훈 (내 반복 실수)**: 검증 전에 "불가/미완/워크스페이스 한계" 를 단정함. 진단 도구
(IK 솔버) 가 "불가" 라 해도 물리 증거(손으로 도달)가 상충하면 **도구(seed 수)를 의심**.
[[feedback_verify_solver_not_reality]]

---

## 2. 구현된 것 (파일별, 전부 uncommitted)

### Motion
- `backend/modules/motion/adapters/pybullet.py` — **IK multi-restart**. seeded 1회
  실패 시 random restart 24회 후 seed 에 가장 가까운 해 선택 (motion 연속성). single-seed
  local IK 가 존재하는 해 놓치는 것 방지. `_ik_from_seed` 로 분리.
- `backend/modules/motion/contract.py` — `MOVE_J_POSE` 서비스 + `MoveJPoseRequest`
  (target_position, optional target_quaternion, optional **tool_offset**). TcpState 에
  `gripper_joint_name`/`gripper_rad` 필드 추가.
- `backend/modules/motion/module.py` — `move_j_pose` 핸들러 (pose→IK(현재자세 seed,
  multi-restart)→run_joint). **tool_offset**: IK(target)→자세 R→`target - R·offset`
  재-IK (파지점을 target 에 맞춤, 검증 0.5mm). gripper rad report (units SSOT).

### Task
- `backend/modules/task/steps.py` — `MoveToPose` step (MOVE_J_POSE 호출, optional
  tool_offset). 기존 `MoveTCP`(MoveL)는 남겨둠 (안 쓰임, Cartesian 필요시용).
- `backend/modules/task/tasks/pick_and_place.py` — approach/grasp/lift/place 전부
  `MoveToPose` 로. `ApproachAlongTool`/`RetreatAlongTool` 삭제. **PINCH_OFFSET =
  (0.0, -0.015, 0.0)** (rough URDF 추정, grasp/place 에 적용, 튜닝 필요).

### Detector
- `backend/modules/detector/projection.py` — `object_top_center_base` (윗면 픽셀
  3D centroid). 기존 z_cam_from_depth_bbox/unproject_to_base 는 test 만 씀.
- `backend/modules/detector/module.py` — `object_top_center_base` 로 파지 x/y 산출.

### Frontend
- `frontend/src/api/generated/contract.ts` — TcpState gripper 필드 (offline 재생성).
- `frontend/src/components/scene/RobotLayer.tsx` — gripper joint 를 arm 뒤 append.

### Tests (통과)
- multi-restart 회귀 (`test_kinematics.py`), projection centroid (`test_detector_projection.py`),
  MoveJPose stack 도달 (`test_motion.py`), pick 이 MoveL 안 씀 (`test_pick_and_place.py`),
  gripper report (`test_motion.py`), RobotLayer gripper append.

---

## 3. 열린 문제 / 버그 (다음 세션 우선순위)

### ★ P1 — 모션이 급하고 안 스무스함 (사용자 2026-07-08 관찰)
> "moveJ 가 너무 빨리 움직여. 서치 후 다음 포인트 이동도 급해. 집으러 갈 때도. 빠른 게
> 중요한 게 아냐, 스무스한 느낌이 없어."

- 후보 원인:
  - `run_joint` 의 Ruckig joint max **velocity/accel/jerk** 가 큼 (robot motion.yaml).
    profile 값 확인 — 급가감속.
  - 각 step 이 **독립 MoveJ + 끝에서 정지** → start-stop-start (blending 없음).
  - position-only IK 가 **자세를 매번 자유롭게** 잡아 config 가 크게 튀면 큰 관절 이동
    = 휙 도는 느낌. (seed 근처 해 선택하지만 여전할 수 있음)
- 조사 시작점: `backend/modules/motion/trajectory_runner.py` `run_joint`/`_joint_loop`
  + robot motion.yaml 의 joint profile + MoveJPose 가 넘기는 속도 한계.

### ★ P2 — detection 이 가끔 엉뚱한 데 찍힘
- 사용자: "디텍팅이 이상한데 찍히기도 했어." grasp 정확도의 뿌리 — 여기가 틀리면 나머지
  다 무의미.
- 조사: projection fix(`object_top_center_base`) 의 top-band 선택이 노이즈/테이블에
  민감한지, 아니면 GDINO bbox 자체가 가끔 오검출인지 분리 필요. depth top-percentile
  band(0.010m) 튜닝 여지. `backend/modules/detector/projection.py` + `module.py`.

### P3 — verify_grasp "gripper 상태 미수신"
- grasp 물리 성공해도 `VerifyGrasp` 가 실패 (task 모듈이 gripper raw 캐시 못 함).
- 확인된 것: 프레임워크는 robot-scoped 구독을 wildcard(`stream/motor/*/raw_state`)로
  등록([app.py:280](../backend/framework/runtime/app.py)) → task 모듈이 RAW_STATE
  받아야 정상. scan 모듈은 같은 패턴으로 잘 됨. gripper_index=`r.motors.index(grip)`=6
  (7모터), positions_raw 7개면 유효.
- **미확정**: 왜 캐시가 None 인가. 라이브 로그 필요 — `_on_motor_raw` 가 실제 호출되는지,
  cross-machine(PC task ← 모터 Pi RAW_STATE) 이 도착하는지, robot_id 매칭 되는지.
  `backend/modules/task/module.py:66` `_on_motor_raw`.

### P4 — grasp 신뢰성 (아직 운)
- **pinch offset 값이 rough 추정** `(0,-0.015,0)` — 실측 필요. 큐브 문 자세(토크오프)
  에서 tcp pose vs 큐브 base 차이 = 정확한 tool-frame offset. 미스 방향이 튜닝 정보.
- **jaw-yaw 정렬 미구현** — detector 가 큐브 yaw 를 안 줌. top-face PCA 로 yaw 뽑아
  그리퍼 roll 을 큐브 면에 맞추면 회전된 큐브도 평평한 면 파지. 단 자세 제약이라
  도달성은 cube 90° 대칭 + 후보 search 로 처리 (motion 에 grasp-pose search 필요).
  **작은 큐브가 pinch offset 만으로 잡히면 불필요할 수도 — 테스트가 판단 근거.**

---

## 4. 검증된 사실 (다음 세션이 재유도 말 것)

- **reach 는 문제 아님**: position-only IK 가 큐브 위 z 2~16cm 전부 도달.
- **MoveL(자세 고정 직선)은 SO-101 pick 에 구조적으로 부적합**: 같은 자세로 z 2.3cm 만
  도달, 4.3cm+ 는 seed 400 개로도 FAIL. 높이마다 도달 자세가 바뀜. → 관절 이동만이 답.
- **single-seed PyBullet IK 는 존재하는 해를 놓침**: 14° 자세가 seed 몇 개론 FAIL,
  400개론 OK. → multi-restart 필수 (구현됨).
- **그리퍼 = 단일 가동 jaw** (파란 고정 / 검은 joint7 회전). 파지점 ≠ 기하 중앙 ≠ tcp.
- **tcp 는 gripper_center 에서 +X 0.04m** (URDF tcp_joint). fingertip ≈ tcp (접근축),
  파지 중심은 tcp 에서 고정 jaw 쪽(tool -Y).
- **tool_offset 메커니즘 검증**: pinch point 가 target 에 0.5mm 정렬 (재-IK 1회 근사).

## 5. 설계 결정 (박힌 것)

- pick 접근/파지/승강 = **MoveJPose(관절 공간)**, MoveL 아님. 월드축(Z-lift 포함) 명령 X.
- IK 어댑터 = multi-restart (seeded 우선 + 실패시 restart, seed 최근접 해).
- pinch offset = **그리퍼 상수(tool-frame), 큐브 치수 하드코딩 금지**. jaw 정렬 = 검출
  기하(cube yaw). 둘 다 큐브 크기 독립.
- gripper 관절 상태 = motion 이 TcpState 별도 필드로 report (arm `joints` 에 안 섞음 —
  waypoint 가 `.joints` 를 arm/IK 벡터로 소비하므로 섞으면 MoveJ dof 깨짐).

## 6. 배포 노트

- 변경 노드: **PC** (task, detector) + **모터 Pi** (motion, IK). 카메라 Pi 변경 없음.
- frontend: contract.ts + RobotLayer (재빌드). contract.ts 는 이미 offline 재생성됨.
- `MOVE_J_POSE` 는 frontend 미노출 (task 가 backend 에서 호출).

---

*세션 중 반복된 사용자 피드백: 검증 전 문제 단정 금지 / 한 primitive·한 축에 갇히지 말
것 / 목표(잘 집기)를 놓지 말 것 / 성급히 메모리·"미완" 선언 말 것.*


---
---

<!-- ═══════════ 여기부터 아키텍처 spec 본문 ═══════════ -->

## 1. 개요

framework 의 목표 = **"같은 코드가 어디 배치되든 그대로 동작하게 만들기"**.

분산 시스템의 mechanical plumbing (topic string / serialize / subscriber routing / late-join snapshot / cache wiring / Zenoh queryable·subscriber 등록) 을 framework 가 흡수하고, 개발자는 domain 의 business intent 만 짠다.

단 *React / Redux / MobX 식 reactive state framework* 가 아님. Owner 쪽은 명시적 (`repo.save() + publish(Event)`), Reader 쪽만 framework primitive 로 흡수. 이 비대칭이 **현재 spec 의 가장 중요한 line**.

## 2. 핵심 원칙

### 2.1 Distribution is runtime concern

Module 코드는 자기가 같은 process / 다른 process / 다른 장비 어디서 도는지 모름. 같은 코드를 한 process 에 다 띄우든 Pi/PC/NAS 로 분산하든 동일 동작 — Zenoh same-session in-routing 이 같은 process 자리 처리, 다른 session 사이는 wire 통과. 배치는 deployment yaml 의 결정.

### 2.2 Framework 는 mechanical plumbing 만 흡수

흡수하는 것:
- contract key (service / event / stream) 관리
- payload serialize / deserialize
- subscriber registry + dispatch
- Zenoh queryable / subscriber 등록 + dispatch
- Reader 의 late-join snapshot fill
- Reader 의 event subscription wiring
- Reader 의 local cache management
- service contract 자동 generate (frontend `contract.ts`)

흡수하지 않는 것:
- domain logic (BA / IRLS / Ruckig / IK / TSDF 등)
- `repo.save()` 호출 — domain 이 "저장한다" 라는 의도 표현
- `publish(Event)` 호출 — domain 이 "사건이 발생했다" 라는 의도 표현
- DB schema (각 Module 의 SQLAlchemy class)
- Migration (각 Module 의 Alembic)

### 2.3 Owner / Reader 비대칭

같은 cross-module state read 문제도 두 쪽이 다름.

**Owner** (예: CalibrationModule) = 자기 상태 변경의 *의미* 를 안다. `repo.save() + publish(DomainEvent)` 명시적. framework 가 mutation tracking 으로 자동 event 생성 X — *DB update ≠ domain event*. 같은 row update 가 어떤 때는 ACTIVATE, 어떤 때는 그저 metadata 수정. 의미는 Owner 만 결정.

**Reader** (예: MotionModule) = 다른 Module 의 *현재 상태* 가 필요할 뿐. snapshot 가져오기 / event subscribe / cache update 는 mechanical. framework 가 흡수.

### 2.4 Database-per-Module

각 도메인 Module 이 자기 영속성 owner. 통합 Storage Module 없음 — *centralization 이 풀려 했던 문제 (cross-module 동기화) 의 진짜 답은 Reader primitive*. Storage Module 의 다른 motivation 들 (migration owner / DB dep 격리) 도 자연 해결 (§9 참조).

### 2.5 DIP — Framework Protocol vs Infra impl

framework 는 *기술 모름*. Protocol (`Repository`, `ObjectStore`, `Transport`) 만 정의.
infra/ 가 실 impl (`PostgresRepository`, `MinioObjectStore`, `ZenohTransport`).
Module 은 Protocol 만 의존.

motivation 두 개:
- **test mock** — pytest 시 in-memory transport + sqlite `:memory:` 박아서 framework 자체 검증.
- **import boundary** — Module 에 `import zenoh` / `import sqlalchemy` 안 새는 보장.

"미래 Zenoh → ROS2 갈아끼우기" 같은 자유도 motivation 은 over-engineering reflex. 박지 말 것.

### 2.6 한 사람 capacity 안

Phoenix / Django / Spring Boot 급 풀 framework 짜는 것 한 사람 무리. 단 우리 도메인 (calibration / scan / reconstruction / task) 패턴 좁고 반복적 — **2 패턴 (active-toggle + broadcast / append-only event)** 추출하면 한 사람 capacity 안.

NestJS / Spring 정도 + 분산 transport 흡수 정도. React / Redux / Apollo cache 수준 X.

### 2.7 Module scope — robot-scoped / robot-agnostic + robot_id 라우팅 (최종)

> 2026-07-03 확정 — 구현 드리프트 (calibration/scan/scene3d/waypoint 가 robot-scoped 로
> 잘못 구현) 정정 완료. 본 절 = scope + robot_id 라우팅의 잠긴 규칙. 폐기안 (Bridge
> 자동주입 / 생성 scope 메타데이터) 다시 꺼내지 말 것 — §2.7.3.

Module 두 종류. **기준 = "Module 이 robot 의 runtime state / 물리 자원을 소유하는가"**.

| 종류 | Module (구현 = 설계) | 인스턴스 |
|---|---|---|
| **robot-scoped (4)** | MotorDriver / CameraDriver / CameraDecoded / Motion | per-robot (Module type × robot_id) |
| **robot-agnostic** | Calibration / Detector / Scene3D / Scan / Waypoint / Bridge (+ 미래 Task / Gamepad) | host 당 1 |

- robot-scoped = *물리 자원 owner* (Feetech handle / RealSense handle / robot kinematics state). 자원은 robot 별 분리되어야 자연.
- robot-agnostic = *작업 / orchestration*. robot_id 는 매 service request 의 인자 (req 안 field). DB 의 `robot_id` column 으로 multi-tenant.

기존 backend 의 `DeviceNode` (per-robot) / `ApplicationNode` (host 당 1 + `enabled_robot_ids` dict) 패턴과 본질 동일 — 새 spec 의 차이는 ApplicationNode 의 `dict[robot_id, _state]` boilerplate 가 Repository 의 robot_id parameter 로 흡수.

#### 2.7.1 robot_id 라우팅 — "robot_id 는 두 개다"

같은 이름이지만 위치에 따라 **다른 레이어**의 것:

| | 키 안의 `{robot_id}` | body 안의 `robot_id` |
|---|---|---|
| 정체 | 어느 인스턴스로 라우팅할지 = **주소** | req 모델의 **필드** (`DetectRequest.robot_id`) |
| 책임 | **전송 계층** — Bridge/framework 가 키 확장 | **서비스 API** — 호출자가 req 에 넣음 (타입 강제) |

규칙 (기계적 3갈래 — 전부 구조적으로 갈림, 런타임 추론/메타데이터 0):

1. **robot-scoped 서비스** — 키에 `{robot_id}`. framework 가 `self.robot_id` 로 확장
   (`_register_service` — **scoped 판정의 SSOT**). caller 는 `robot_id=` kwarg.
2. **robot-agnostic + 로봇 대상** — 키에 placeholder 없음, **req 에 `robot_id` 필드**.
   호출자가 넣고 pydantic/TS 가 강제. 단 **다른 식별자(run_id / session_row_id /
   result_id / waypoint_row_id)로 robot 특정 가능하면 DB row 에서 파생** — req 에
   중복 robot_id 채널을 안 만든다 ("run A 에 robot B 캡처" 불일치 원천 차단).
3. **global** — req 에 robot_id 필드 자체가 없음 → 아무 데도 안 들어감 (구조적).

**stream/event 는 서비스와 성격이 다름 — 키에 `{robot_id}` 유지.** framework 가
payload 의 robot_id 로 확장(publish) / wildcard 구독(subscribe) → host-level 모듈도
robot-scoped 스트림을 그대로 발행/구독 (예: 호스트 1개 calibration 의 preview).
따라서 **`robot_scoped` 판정 = service 키만** (publish/subscribe 는 판정에 안 씀 —
snapshot.py `ModuleContract.robot_scoped`).

**Bridge = 순수 transport** — 키 확장(라우팅)만. domain body 는 손대지 않는다.
의도된 대가: 서비스가 scoped↔agnostic 바뀌면 call site 편집 필요 (robot_id 가
options↔req 이동) — **컴파일타임에 잡히는 기계적 수정**. "frontend 0 수정" 목표는
Bridge 에 서비스 의미를 넣는 비용이라 포기 (책임 분리 우선).

#### 2.7.2 robot-agnostic 모듈의 구현 패턴

- **runtime state → `dict[robot_id]`** (모듈 소유): 최신 frame / raw / preview on-off /
  seq. 실행 중에만 존재, 대부분 0~1 sparse.
- **config → resolve 가 robots.yaml 에서 lean 투영 주입** (모듈이 SSOT 복사·재보유 X).
  모듈별 필요만 — 스펙트럼: Calibration=`CalibrationRobotSpec`(motor_ids+has_camera) /
  Scan=`ScanRobotSpec`(kinematics+arm_specs, dataclass) / Scene3D=`robot_ids` 멤버십만
  (enabled+rgbd) / Waypoint·Detector=**0**. 투영 class 는 module.py 소유 (wire 아님) —
  bridge 의 RobotInfo 변환과 동형 (내부 config → module dep 는 apps 책임).
- `@subscriber` 는 framework wildcard → `payload.robot_id` 로 dict 캐시 (fleet 밖
  robot 은 skip).
- deployment yaml 에 `robots:` 없음 → `resolve_host_deps` 배선.

#### 2.7.3 acceptance (기능 검증 아니라 아키텍처 검증)

1. host-level (`self.robot_id` 없음)  2. robot-specific 정보 소유권 한 곳 (robots.yaml
SSOT, 모듈 복사 X)  3. runtime state ↔ config 명확 분리  4. **★ 새 로봇 추가 시 모듈
코드 0 수정** (진짜 리트머스).
- **눈속임 방지 테스트**: 단일 host-level 인스턴스로 **so101(6DOF) AND omx(5DOF)** 둘 다
  구동 (`test_single_instance_serves_so101_and_omx_isolated` 패턴 — 한쪽 하드코딩
  잔재는 다른쪽 경로에서 터짐). 한 robot 만 green = 기능 검증일 뿐.
- **폐기안 (다시 꺼내지 말 것)**: ① 생성 메타데이터 `robot_id_body_services`
  (contract 파생 목록 — SSOT 중복) ② Bridge 휴리스틱 자동주입 (agnostic vs global
  런타임 구분 불가 → "지금 global 없으니까" 타협 필요). 근거: 키의 robot_id(주소)와
  body 의 robot_id(req 필드)는 다른 레이어 — 자동주입 자체가 무근거.

#### Scope 결정 자리 — yaml primary, constructor 계약 검증

**scope 결정 주체 = deployment yaml**. 같은 Module class 가 host 별 다른 scope 가질 수 있음. constructor 는 그저 *계약 검증*.

```yaml
pc:
  modules:
    - module: CalibrationModule         # robots: 없음 → host-scoped 1 인스턴스
    - module: TaskModule
    - module: Bridge

pi_motor:
  modules:
    - module: MotorModule               # robots: 박힘 → per-robot N 인스턴스
      robots: [omx_f_0]
    - module: MotionModule
      robots: [omx_f_0]
```

framework 부팅 흐름:
```python
if "robots" in module_cfg:
    # 계약: __init__ 에 robot_id parameter 박혀있어야
    assert "robot_id" in inspect.signature(cls.__init__).parameters
    for rid in module_cfg["robots"]:
        instances.append(cls(robot_id=rid, ...))
else:
    # 계약: __init__ 에 robot_id parameter 박혀있으면 안 됨
    assert "robot_id" not in inspect.signature(cls.__init__).parameters
    instances.append(cls(...))
```

**규칙 표현**:
- ❌ "Module 이 robot_id 받으면 robot-scoped" (direction 반대)
- ✅ "robot-scoped 로 배치하려면 constructor 가 robot_id 받아야 한다"

차이 — 미래에 robot_id 받지만 scope 아닌 Module 가능 (예: `FleetMonitor(robot_id_filter=...)`). yaml 이 primary 이면 그 자리 자연 흡수.

base class / `@robot_scoped` 데코 박지 않음 — Module = plain class 유지 (§3 의 데코 인플레이션 회피).

## 3. 4 framework primitive

framework 가 제공하는 1급 시민 4 개. 이외 surface 박지 않음.

### 3.0 Contract key — 세 원칙

framework 의 contract key (service path / event topic / stream topic) 가 따르는 세 원칙 (hard rule):

**1. Explicit — 사람이 지정**

- 개발자가 key string 의 값 자체 명시
- 정의 = `contract.py` 의 nested `StrEnum` (string 정의 유일 위치)
- 모든 use site (service handler / subscriber / publisher / Mirror / caller) 가 key 를 직접 박음 — implicit lookup (예: class attribute / method `@service` spec lookup) 박지 X
- auto-derive (class name → topic regex) 박지 X

**2. Typed — class / enum (raw str X)**

- raw string 참조 박지 X (typo 차단)
- 모든 use site 가 typed identifier — `StrEnum value` / `event class` / `type hint`

**3. service 가리키는 방법 = 항상 `Service.X` enum 하나**

- method reference (`Module.method`) 박지 X — 박으면 service 가리키는 방법이 두 개 (enum + method ref) 가 됨
- Mirror / `runtime.call` / `@subscriber` / publish 모두 동일 패턴

**원칙 정합 = 다음 형태** (module 별 nested class + contract.py 통합):

```python
# modules/calibration/contract.py — 외부 Public Surface (Service / Event key + Pydantic payload)
from enum import StrEnum
from pydantic import BaseModel

class Calibration:
    class Service(StrEnum):
        ACTIVATE         = "srv/calibration/activate"
        SNAPSHOT_BUNDLE  = "srv/calibration/snapshot_bundle"

    class Event(StrEnum):
        ACTIVATED  = "event/calibration/activated"
        COMMITTED  = "event/calibration/committed"

# payload (event / req / res / bundle) — pure Pydantic data, key 정보 박지 X
class CalibrationActivated(BaseModel):
    robot_id: str
    bundle_id: int
# ... (req/res/bundle 자체 같은 파일 안)
```

```python
@service(Calibration.Service.ACTIVATE)                                  # handler
@subscriber(Calibration.Event.ACTIVATED)                                # subscriber
runtime.publish(Calibration.Event.ACTIVATED, event)                     # publisher
runtime.call(Calibration.Service.SNAPSHOT_BUNDLE, req, ResCls, ...)     # caller
Mirror(snapshot_service=Calibration.Service.SNAPSHOT_BUNDLE,
       change_topic=Calibration.Event.ACTIVATED, value_cls=Bundle, ...) # Reader (5 인자 모두 explicit)
```

상세 = §3.1 (service) / §3.2 (event) / §3.3 (Mirror) / §3.7 (ModuleRuntime).

**Nested class 패턴 — `Module.Service` / `Module.Event` / `Module.Stream`**:

- 도메인 별 단일 entry point — `Calibration.Service.X` / `Calibration.Event.X` 가 한 묶음
- 읽기 자체 자연어 — "Calibration 의 Service ACTIVATE", "Camera 의 Stream JPEG"
- IDE 자동완성: `Calibration.` → `Service` / `Event` 가지 자동 보임
- 도메인 격리 + module self-containment 정합 (§2.4 / §7.2)
- 새 종류 추가 = nested class 1개 (예: Camera 에 `Stream` 가지) — class 이름 prefix 반복 X

**contract.py = "Public Surface"** (외부 module 이 import 박는 모든 것):

| contract.py 안 | 이유 |
|---|---|
| ✅ `Module.Service` / `Module.Event` / `Module.Stream` (nested StrEnum) | 외부에서 `@subscriber` / `runtime.call` / Mirror 에 사용 |
| ✅ Event payload Pydantic class | `@subscriber` type hint / Mirror `change_event_cls` |
| ✅ Service Request / Response Pydantic class | caller 가 인자로 박음 |
| ✅ Bundle / Value Pydantic class (Mirror value_cls) | Mirror 의 cache type |
| ❌ SQLAlchemy ORM (`models.py`) | 영속성 internal — Repository 안에서만 |
| ❌ Repository / Business logic (`service.py`) | module.py 안에서만 |
| ❌ Module class (entry) | framework Runtime 만 instantiate |

기준 한 줄 = **"다른 module 이 이걸 import 박는가"**. 답이 yes 면 contract.py.

**진화 path** — 첫 박을 때 `contract.py` 단일 파일. 비대해지면 (예: 1000 줄+) `contract/` 패키지로:

```
contract/
  __init__.py     # re-export (외부 import path 자체 안 바뀜)
  keys.py
  events.py
  services.py
```

외부 module 의 import 자체 동일 (`from modules.X.contract import ...`) — 내부만 refactor.

**경로 convention** — 첫 chunk 가 통신 purpose 분리:

| prefix | 의미 | 형태 | nested class |
|---|---|---|---|
| `srv/` | request/response RPC | `srv/<module>/<verb>` / `srv/<module>/{robot_id}/<verb>` | `Module.Service` |
| `event/` | 상태 변화 notification (broadcast) | `event/<module>/<name>` / `event/<module>/{robot_id}/<name>` | `Module.Event` |
| `stream/` | 고빈도 raw 데이터 (camera / depth / pointcloud) | `stream/<module>/{robot_id}/<kind>` | `Module.Stream` |

`horibot/` prefix 박지 X — broker 단일 project, namespace 분리 motivation 약함. purpose 분리가 진짜 가치 (debugging / wildcard scope / Zenoh declare 명확).

### 3.1 `@service` — RPC handler

Service key 는 **사람이 explicit 지정** + **typed identifier (nested StrEnum)** — raw string 박지 X (§3.0 의 세 원칙).

```python
# modules/calibration/contract.py — string 정의 (유일)
from enum import StrEnum

class Calibration:
    class Service(StrEnum):
        ACTIVATE         = "srv/calibration/activate"
        SNAPSHOT_BUNDLE  = "srv/calibration/snapshot_bundle"


# modules/calibration/module.py
from .contract import Calibration

class CalibrationModule:
    @service(Calibration.Service.ACTIVATE)
    def activate(self, req: ActivateRequest) -> ActivateResponse:
        result = self._repo.get(req.result_id)
        if result is None:
            raise NotFound(f"result {req.result_id} 없음")    # exception propagation
        ...
        return ActivateResponse(ok=True)
```

- `req_cls` / `res_cls` = handler 의 type hint 에서 자동 추출.
- Service key = `@service` 인자의 StrEnum value — raw string 아님.
- framework Runtime 이 ZenohTransport 위에 service queryable 등록 (key = enum value).
- 같은 process caller = Zenoh same-session in-routing.
- 다른 process caller = Zenoh between-session.

**Caller — key + req + res_cls (모두 explicit)**:

```python
from modules.calibration.contract import Calibration, ActivateRequest, ActivateResponse

class OtherModule:
    async def do(self):
        try:
            result = await self.runtime.call(
                Calibration.Service.ACTIVATE,                  # service key
                ActivateRequest(result_id=10),                 # req
                ActivateResponse,                              # res_cls (return type narrow)
            )
        except RemoteError as e:
            if e.type == "NotFound": ...
        except TimeoutError:
            ...
```

framework 전체 단 하나의 규칙: service 가리키는 방법 = 항상 `Module.Service.X`. method reference 박지 X — Mirror / call / publish / subscribe 모두 같은 패턴.

`res_cls` 명시 — caller 가 받을 return type narrow + framework 가 wire payload decode 시 cls 직접 사용 (spec lookup indirection 없음).

**Robot-scoped service** — key 안 `{robot_id}` placeholder:

```python
class Motion:
    class Service(StrEnum):
        MOVE_L  = "srv/motion/{robot_id}/move_l"
        MOVE_J  = "srv/motion/{robot_id}/move_j"

class MotionModule:
    @service(Motion.Service.MOVE_L)
    def move_l(self, req: MoveLRequest) -> MoveLResponse: ...

# 호출 — caller 가 key + req + res_cls + robot_id 명시
await self.runtime.call(
    Motion.Service.MOVE_L, req, MoveLResponse, robot_id="omx_f_0",
)
```

framework register 시점 — Module instance 의 `self.robot_id` 로 placeholder 자동 substitute. caller 시점 — `robot_id=` 인자로 substitute.

**Error contract — exception propagation, envelope X**:

- 성공 path = `ServiceResponse[T]` (Pydantic generic) — 항상 valid `T`. caller 가 `res.success` 체크 박지 않음.
- handler exception → framework 가 type name + message 만 wire 통과 (traceback 박지 X).
- caller 측에서 `RemoteError(type=<name>, message=<msg>)` raise. caller 가 `except RemoteError as e: if e.type == "...":` 패턴 또는 generic catch.
- 같은 exception class 의 client-side 자동 raise (예: `NotFound` 실 class) 는 박지 않음 — Phase B detail.
- timeout = `Transport.call(timeout=5.0)` exceeded → `TimeoutError` raise.

이유 — Python 자연 = exception. caller 가 매 호출 `if not res.success: ...` envelope check 박는 자체 boilerplate. type-safe success path + exception path 분리가 정직.

### 3.2 `@subscriber` + `publish` — Domain event broadcast

Event 도 §3.0 원칙 정합 — key 가 publisher / subscriber 양쪽에서 직접 박힘. Event class = pure Pydantic data (key 정보 박지 X — separation of concerns).

```python
# modules/calibration/contract.py — Service + Event key + payload class 한 묶음
from enum import StrEnum
from pydantic import BaseModel

class Calibration:
    class Service(StrEnum):
        ACTIVATE         = "srv/calibration/activate"
        SNAPSHOT_BUNDLE  = "srv/calibration/snapshot_bundle"

    class Event(StrEnum):
        ACTIVATED  = "event/calibration/activated"
        COMMITTED  = "event/calibration/committed"

class CalibrationActivated(BaseModel):
    robot_id: str
    bundle_id: int

class CalibrationCommitted(BaseModel):
    robot_id: str
    bundle_id: int
```

**Owner 측 publish — key 첫 인자, event instance 두 번째**:

```python
class CalibrationModule:
    @service(Calibration.Service.ACTIVATE)
    def activate(self, req):
        result = self.repo.get(req.result_id)
        result.activate()
        self.repo.save(result)
        self.runtime.publish(
            Calibration.Event.ACTIVATED,
            CalibrationActivated(robot_id=req.robot_id, bundle_id=result.bundle_id),
        )
```

domain logic 바로 다음 줄에 어떤 event key 로 publish 하는지 보임.

**Subscriber 측 — `@subscriber(key)` factory + type hint 로 decode**:

```python
class AuditModule:
    @subscriber(Calibration.Event.ACTIVATED)
    def on_calibration_activated(self, event: CalibrationActivated):
        self.log_audit(event)
```

- event key = `@subscriber` 인자 (nested StrEnum value)
- event class = type hint (framework 가 payload decode)

**`@publishes` class decorator — (key, event_cls) pair self-declare**:

self-doc + contract.ts auto-generate 용. 실 publish 강제 X — declare 안 된 pair 도 publish 동작.

```python
@publishes(
    (Calibration.Event.ACTIVATED, CalibrationActivated),
    (Calibration.Event.COMMITTED, CalibrationCommitted),
)
class CalibrationModule:
    ...
```

**Robot-scoped event** — topic 안 `{robot_id}` placeholder:

```python
class Motion:
    class Event(StrEnum):
        COMPLETED = "event/motion/{robot_id}/completed"

class MoveCompleted(BaseModel):
    robot_id: str
    ...

# publish — event.robot_id 가 placeholder substitute
self.runtime.publish(
    Motion.Event.COMPLETED,
    MoveCompleted(robot_id=self.robot_id, ...),
)

# subscribe — framework 가 placeholder → Zenoh wildcard `*` substitute
@subscriber(Motion.Event.COMPLETED)
def on_completed(self, event: MoveCompleted):
    ...
```

framework 자동: publish 시점에 `event.robot_id` 로 substitute, subscribe 는 transport wildcard 로 substitute 후 payload 의 `robot_id` 로 self-filter (Mirror 도 동일).

### 3.3 `Mirror[T]` — Cross-module state read

> ✅ **STATUS: 활성 (2026-07-07) — 첫 consumer = MotionModule.calibration.** 옛 deferred (2026-07-02, consumer 0) 해제. 근거: "boot-query 1회" 는 분산 부팅 순서 종속성 (PC 늦으면 motion 이 무보정으로 영원히 운전 — silent degradation) 을 만들었다. Mirror 가 **liveliness** (owner 의 snapshot service 생존을 transport 가 관측 — anchor #23) 로 완성되어: owner 늦은 부팅 / 재시작 (죽어있는 동안 데이터 변경, event 영영 안 옴) 전부 자동 refetch 수렴. `on_change(old, new)` 훅 (`@mirror.on_change` decorator, 값이 실제로 바뀐 전이만 발화) 으로 consumer 반응. Motion 정책: 없음→값 = runner idle 때 live 적용 / 값→값′ = `calibration_stale` 표시만 (변경은 재부팅 유지). 상태는 `TcpState.calibration_applied/stale` 로 상시 표면화.

가장 중요한 primitive. Reader 쪽 boilerplate (snapshot fill / subscribe / cache) 흡수.

```python
from modules.calibration.contract import Calibration, CalibrationActivated, CalibrationBundle, SnapshotRequest

class MotionModule:
    calibration: Mirror[CalibrationBundle] = Mirror(
        snapshot_service=Calibration.Service.SNAPSHOT_BUNDLE,         # service key
        snapshot_req=lambda self: SnapshotRequest(robot_id=self.robot_id),  # req factory
        change_topic=Calibration.Event.ACTIVATED,                     # event key
        value_cls=CalibrationBundle,                                  # snapshot res_cls + cache type
        change_event_cls=CalibrationActivated,                        # event class (decode)
    )

    @service(Motion.Service.MOVE_L)
    def move_l(self, req):
        cal = self.calibration.value           # 매 호출 fresh cache read
        urdf_joints = [j + cal.joint_offsets[i] for i, j in enumerate(joints)]
        tf = cal.hand_eye                       # sub-field access — consumer 책임
        ...
```

Mirror 의 5 인자:
- `snapshot_service` = service key (StrEnum value). framework 가 호출할 RPC.
- `snapshot_req` = req factory `Callable[[self], BaseModel]`. Module instance 박힌 후 호출 — `self.robot_id` 등 활용 가능. robot-agnostic Reader 면 `lambda self: SnapshotRequest()`.
- `change_topic` = event key (StrEnum value). subscribe 할 topic.
- `value_cls` = snapshot response type = cache 의 T. `Mirror[T]` 의 T 자체.
- `change_event_cls` = event class. change_topic payload decode 시 type.

framework 전체 단 하나 규칙 (service = `Module.Service.X`, event = `Module.Event.X`) 정합 — Mirror 도 method reference 박지 X.

framework 자동:
1. Module start 시 `runtime.call(snapshot_service, snapshot_req(self), value_cls)` → local cache fill (단 fail OK, §3.3.1 참조).
2. `change_topic` subscribe → 받으면 cache refetch.
3. `self.calibration.value` access = cache read.
4. Module stop 시 subscription unregister.

Owner 쪽은 standard service + event 박는 것만, Mirror 가 wiring.

**명시적 mapping** (5 인자 모두 key + Pydantic class) 가 정직. method reference / class attribute lookup 0. framework 전체 *service 가리키는 방법 = 항상 `Module.Service.X`* 한 패턴.

#### 3.3.1 Startup ordering — empty + fallback fetch

```python
Module.start():
    ① state = INITIALIZING
    ② event_buffer = []
    ③ subscribe(change_event):
          if state == INITIALIZING: event_buffer.append(event)
          else: cache = fetch_snapshot()      # event 받음 → 다음 snapshot fetch 로 갱신
    ④ snapshot try (background, non-blocking):
          success → cache = result
          fail (Owner 안 떠 있음) → cache = None
    ⑤ buffer replay:
          if any event in buffer: cache = fetch_snapshot()  (Owner 가 그 사이 떴을 수 있음)
    ⑥ state = READY
```

- **blocking retry 박지 않음** — Owner 가 안 떠 있어도 Reader Module 의 start 가 영원히 block 되면 안 됨 (분산 partition tolerance).
- **race 차단 — buffer + replay** — subscribe 시점부터 받은 event 를 INITIALIZING 동안 buffer. snapshot 적용 후 buffer 가 비어있지 않으면 fresh fetch (가장 단순한 구현, 마지막 변경값으로 수렴).
- snapshot 실패 후에도 *첫 change event* 가 fallback fetch trigger — 결국 fresh cache 도달.

#### 3.3.2 Value access — `.value` 매 access fresh + `is_ready` flag

```python
class Mirror[T]:
    _cache: T | None = None
    _initialized: bool = False
    
    @property
    def is_ready(self) -> bool:
        return self._initialized       # snapshot/event 한 번이라도 받았나
    
    @property
    def value(self) -> T:
        if not self._initialized:
            raise NotReady(f"Mirror[{T.__name__}] 아직 snapshot/event 못 받음")
        return self._cache
```

**계약**:
- `self.calibration.value` 매 access 가 **fresh cache read**. consumer 가 local variable 에 capture 박지 X (stale 위험).
- `is_ready=False` 자리는 application 책임 — `if not self.calibration.is_ready: raise/return error`. *"값이 empty domain value"* (예: `bundle.hand_eye == identity`) 와 *"아직 안 받음"* 자리 명확 분리.
- **`.value` 는 partially updated state 노출 X** — Mirror update 가 *event callback thread* 에서 일어남, service handler 가 다른 thread 에서 access. 두 access 사이 race window 가 partial state (예: cache 만 새값, initialized 옛값) 보이면 안 됨. 구현 (lock / atomic reference swap / RCU / actor model) 은 자유 — 운영 model 바뀌면 함께 진화.

#### 3.3.3 Bundle 단위 — sub-field 분리 박지 않음

Mirror[T] 의 T = **도메인의 atomic 단위 (Bundle)**. 같은 BA / 같은 commit 이 만든 산출물은 한 type 으로 묶음 — sub-field 별로 4-5 개 Mirror 박지 않음.

예 — Calibration:
```python
class CalibrationBundle(BaseModel):
    joint_offsets: list[float]
    link_offsets:  list[LinkOffset]
    sag_offsets:   list[float]
    hand_eye:      Transform4x4
    intrinsic:     CameraIntrinsic
    commit_time:   datetime
    bundle_id:     int
```

이유:
- BA atomic = 한 ResultBundle. 사용자가 "joint_offset 만 commit" 박지 X — BA 가 동시 산출.
- 4-5 sub-field 별 별도 Mirror 박는 자체 *기존 backend implementation detail (4 종 npz 파일 분리) 의 매몰*. 도메인 의도 X.
- Bundle size 작음 (수 KB). 모든 consumer 가 전체 받아도 transport 비용 무관.
- consumer 가 sub-field 별 access 자체 책임 — `cal.hand_eye`, `cal.joint_offsets[i]`.

#### 3.3.4 Effective apply — framework 안 박힘, consumer 책임

> ⚠️ **이 절의 calibration 예제는 SUPERSEDED (2026-07-02).** 아래 `link_offsets 변경 → _rebuild_kinematics` 런타임 재로드는 **실제 calibration 에서 제거됨** — Bundle 은 boot-time config 라 Motion 은 start() 에서 1회 build 하고 런타임 rebuild 하지 않는다 ([calibration.md §10.2](calibration.md): "Mirror 니까 실시간이어야 한다" 는 아키텍처적 연역이었고 실제 트리거가 없었음). 아래는 *만약* control-correctness-state consumer 가 있었다면 effective-apply 를 framework 가 아니라 consumer 가 처리한다는 **패턴 illustration** 으로만 유지.

Mirror cache 갱신 = framework 자동. 단 *effective apply* (architectural side-effect) 는 consumer 책임 — framework 가 `@on_mirror_change` 같은 magic 데코 박지 X, 그저 **일반 `@subscriber(ChangeEvent)`** 박아 자기 도메인 처리.

```python
class MotionModule:
    calibration: Mirror[CalibrationBundle]   # 위 Mirror(...) 선언과 동일 instance

    @subscriber(Calibration.Event.ACTIVATED)             # Mirror 와 같은 event key
    def on_calibration_change(self, event: CalibrationActivated):
        # Mirror cache 는 framework 가 갱신함
        # 단 PyBullet kinematics 는 부팅 1회 load — 재로드는 consumer 책임
        if event.changed.contains("link_offsets"):
            self._rebuild_kinematics(self.calibration.value)
            # trajectory 실행 중이면 안전 timing 대기 후 rebuild — consumer 도메인 책임
        # joint_offsets / sag_offsets / hand_eye = 매 access fresh, rebuild 불필요
```

framework 가 *graceful restart / rebuild* 자체 처리하지 않음 — Module 이 자기 architectural side-effect 알아 처리. trajectory 중단 timing, queue drain 등 도메인 정책.

#### 3.3.5 동기화 패턴 — invalidate+refetch only (push update 박지 X)

Mirror 의 cache 갱신 방식 두 후보 패턴:

| 패턴 | event 역할 | Mirror 동작 |
|---|---|---|
| **Push Update** | event payload = 최신 상태 그 자체 | event 받으면 cache = event payload (서비스 호출 X) |
| **Invalidate + Refetch** (현재) | event = 변경 알림 (notification) | event 받으면 snapshot service 재호출 → cache 갱신 |

**현재 spec = Invalidate + Refetch 단일**. push update 패턴 박지 X.

이유:

1. **Bundle atomic invariant 보존** (§3.3.3) — Bundle 은 한 BA / 한 commit 의 atomic 산출물. push update 박으면 event payload 가 진실 source 가 되어 snapshot 과 diverge 위험. snapshot 호출이 항상 최신 보장.

2. **Mirror 의 진짜 use case = 다른 Module 의 event 가 trigger** — Owner 가 자기 전체 상태 모를 수 있음:
   ```
   CameraModule → publish(CameraIntrinsicChanged)
       ↓
   CalibrationModule 의 Mirror 가 event 받음
       ↓
   "Calibration 이 영향 받음" → snapshot 호출
       ↓
   최신 CalibrationBundle (intrinsic + extrinsic + sag 합산) cache
   ```
   여기서 event = trigger 신호일 뿐 payload 아님. push update 불가능 — event publisher 가 전체 Bundle 모름.

3. **same-module event 도 invalidate+refetch 로 통일** — `CalibrationActivated` 처럼 Owner 가 자기 Bundle 알 때도 같은 path. 두 갈래 비대칭 회피. wasted RPC cost 작음 (Bundle 수 KB, 변경 빈도 낮음 — calibration 은 BA 시점만).

4. **push update 필요하면 Mirror 안 박고 `@subscriber` 직접** — event 가 최신 상태 그 자체면 framework 흡수 가치 작음. `@subscriber(Module.Event.X) def on(self, e): self._cache = e` 박으면 충분. Mirror 의 진짜 가치 = "Notification + auto Refetch" 흡수, push update 패턴은 이 가치 안 만족.

새 use case 발견 시 본 결정 재검토. 단 첫 박힘은 invalidate+refetch 단일 path.

### 3.4 Transport (Zenoh 단일)

**Transport 의 의미** — Zenoh 추상화 객체가 아니라, framework 가 Module 에게 *허용한 통신 어휘 그 자체*. 4 surface (publish / subscribe / call / register_service) 외 통신 박지 X — Module 짤 때 첫 질문이 "Zenoh 로 어떻게 보내지?" 가 아니라 "이건 4 어휘 중 어떤 거지?" 가 되도록 강제. 결과 = 모든 Module 의 통신 모양이 균일. Module 코드에 `import zenoh` 절대 안 나옴 (import boundary §2.5) — 이건 "Zenoh 갈아끼우기" 가 목적이 아니라 **"Module 이 4 어휘 밖으로 못 나가게 막는 차단막"**.

Module 코드는 transport object 본 적 없음. `self.runtime.publish` / `self.runtime.call` 만 호출 (`ModuleRuntime` Protocol — §3.7). `@subscriber` 는 데코레이터로 framework 가 wire — Module 코드 직접 subscribe 호출 X.

framework Runtime 이 transport 를 hold:

- **ZenohTransport** (infra/transport/zenoh.py) — Zenoh session + `put` / `declare_subscriber` / `declare_queryable`. 같은 process / 다른 process 동일 어휘.

**Wire encoding — Pydantic + msgpack layered (DIP)**:

```
Module                  Pydantic                  Transport
─────────               ────────                  ─────────
event instance          schema validation         msgpack bytes
   │                       │                          │
   ├─ runtime.publish ─→ model_dump() ─→ msgspec ─→ transport.publish
       (key, event)       (dict)         .encode      (str(key), bytes)

   ◀── decode_event ───── model_validate ◀── msgspec ◀── subscriber callback
                          (instance)         .decode     (bytes)
```

- **Pydantic** = schema validation + Python ↔ dict 변환. Module 코드는 도메인 의도만 표현.
- **msgspec.msgpack** = wire serialization (transport boundary). Module 코드는 모름.
- Module 은 Pydantic 만 알고, Transport boundary 가 msgpack 처리 — DIP 정합.
- native `bytes` field pass-through — JSON 의 base64 33% overhead 회피. camera JPEG / depth zstd / pointcloud 에서 영향 큼.

("Wire encoding" 의 *wire* 어휘는 transport boundary 의 raw byte 의미 — §3.0 의 contract key 와 다른 layer.)

```python
# framework/contract/publisher.py
import msgspec

def encode_event(event: BaseModel) -> bytes:
    return msgspec.msgpack.encode(event.model_dump())

def decode_event(event_cls: type[T], payload: bytes) -> T:
    return event_cls.model_validate(msgspec.msgpack.decode(payload))
```

key lookup helper 박지 X — key 가 use site (publish 첫 인자 / `@subscriber` 인자) 에 직접 박혀있어 추가 lookup 불필요.

같은 process 안 Module 간 호출도 Zenoh same-session 통과 — `session.put` → in-session routing → subscriber callback. wire 0 (TCP/UDP 안 거침), application boundary 의 Pydantic encode/decode + Python ↔ Rust ZBytes copy 만 비용.

**LocalTransport (process-local `dict[key] → callback` direct dispatch) 박지 않음.** 측정 결과 ([backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py)):

| Payload | Zenoh same-session | LocalTransport 가 절감 |
|---|---|---|
| Pydantic small (32B) | 3.5us | ~3.5us, 무관 |
| JPEG 200KB × 30Hz | 52us = 1.5ms/sec | 무관 |
| **PointCloud 5MB × 30Hz** | 1.27ms = 38ms/sec | ~4% CPU × N consumer |

5MB transport 비용 중 ~97% 가 Python ↔ Rust ZBytes boundary memcpy. Zenoh in-session routing 자체는 28us. 즉 LocalTransport 가 우회하는 진짜 비용은 *boundary memcpy*.

큰 ndarray fanout 만 의미 있는 절감 (~4% × N CPU). 단 framework 두 갈래 (Transport 두 impl + resolver + behavior 일관성) 유지 비용보다 작음. **Zenoh 단일 + derived read model 패턴** (§3.5) 으로 카메라 ~13% CPU 도달 — 추가 7-8% 는 측정 후 진짜 bottleneck 으로 드러나면 그때 박음.

#### 3.4.1 subscribe = payload-only + 인스턴스 정체성 규약 (2026-07-15 확정, 전체 근거 docs/logging.md §10)

**`subscribe(key, cb)` 의 콜백은 payload(bytes) 만 받는다 — 매칭된 concrete key 는 안 준다.** 이건 결함이 아니라 원칙:

- **need-driven projection** — `subscribe` 는 이미 Zenoh Sample 의 timestamp/kind/encoding/attachment 를 다 버린다. "봉투 완전 전달" 은 이 프로젝트의 가치가 아니다 (그러면 저 필드들도 다 노출해야 하는 reductio). 각 메서드는 자기 목적에 필요한 것만 노출.
- **인스턴스 정체성(robot_id/host 등)은 레코드(payload)에 자기완결로 담고**, wire key 는 라우팅 전용 (publisher 가 박음). 파일/DB 에 앉은 레코드가 key 없이도 자기완결 (영속/재생 안전). robot-scoped 는 §2.7 의 `{robot_id}` (payload 필드에서 publish 가 파생), host-scoped 로그는 `log/{host}` (host 는 레코드에도 각인).

**동적 발견은 liveliness 가 이미 key 를 나른다.** `subscribe_liveliness(key, cb)` 콜백은 `(concrete key, alive)` 를 준다 (Mirror 가 이 패턴). 즉 "지금 무엇이 살아있나" 는 별도 key-나르는 채널로 이미 풀려 있어, **데이터 평면 subscribe 가 발견 목적으로 key 를 나를 이유가 원리적으로 없다.**

**인스턴스→데이터 상관 패턴 = enumerate → concrete 구독**: 인스턴스 목록을 레지스트리(robots.yaml)/DB list/liveliness 로 먼저 얻고, **concrete key** 로 구독한다 (wildcard demux 아님). 브라우저가 `/robots` 로 robot 목록 얻어 robot별 concrete stream 구독하는 게 정본. 새 per-instance 텔레메트리(host CPU/mem 등)도 이 패턴을 미러링 (host 레지스트리 → host별 concrete 키).

**미래 B (스키마 무지 wildcard 소비자 — packet recorder / MCAP·Foxglove, §7 defer):** 이게 정말 필요해지면 **`subscribe` 를 바꾸지 말고**, liveliness 동형의 **별도 key-나르는 capture 채널을 additive 로 추가**한다. 드물고(단일 능력) + 구조적으로 다르고 + 고립된 소비자를 위해 정착된 공유 경로를 변형하지 않는다 — 그마저 Zenoh-native 캡처 대안이 먼저. (logging.md §10.6 의 "subscribe 확장" 문구는 이 additive 방침으로 대체 — 2026-07-15 재평가.)

> **일반 규칙**: 드물고+구조적으로 다르고+고립된 소비자가 공통 경로보다 더 요구하면 → 공유 경로 변형 X, **additive 특수 채널** 추가 (transport 의 subscribe vs subscribe_liveliness 가 이미 이 모양).

### 3.5 Derived read model Module — decode dedup 패턴

framework primitive 가 아닌 **Module 패턴**. 큰 payload (카메라 JPEG, depth zstd) 의 decode 가 N consumer × decode 비용으로 누적되는 문제를 푸는 표준 형태. framework 는 모름 — 그저 일반 Module + `@subscriber` + `publish` + `@service`.

stream key 도 §3.0 원칙 정합 — publish / subscribe 양쪽에서 key 직접 박힘. 큰 payload (jpeg bytes / zstd depth) 는 `bytes` field (msgpack native bytes pass-through — §3.4).

**naming — `Event` vs `Stream` 분리**: `event/` prefix = 상태 변화 notification, `stream/` prefix = 고빈도 raw 데이터. nested class 이름도 `Camera.Stream` (Event 아님) — stream 은 event 가 아님.

```python
# modules/camera/contract.py
from enum import StrEnum
from pydantic import BaseModel

class Camera:
    class Service(StrEnum):
        DECODED_SNAPSHOT = "srv/camera/{robot_id}/decoded_snapshot"

    class Stream(StrEnum):
        JPEG          = "stream/camera/{robot_id}/jpeg"
        DEPTH_FRAME   = "stream/camera/{robot_id}/depth_frame"
        DECODED       = "stream/camera/{robot_id}/decoded"

# payload — pure data, key 정보 박지 X
class CameraJpegFrame(BaseModel):
    robot_id: str
    timestamp: float
    jpeg_bytes: bytes

class CameraDecodedFrame(BaseModel):
    robot_id: str
    timestamp: float
    width: int
    height: int
    ndarray_bytes: bytes        # 압축 안 된 BGR raw
```

```
Pi process:
  CameraDriver Module (robot-scoped, self.robot_id)
      ├─ RealSense capture
      ├─ JPEG encode + zstd depth encode
      └─ self.runtime.publish(
             Camera.Stream.JPEG,                                      ← stream key 첫 인자
             CameraJpegFrame(robot_id=..., jpeg_bytes=...),           ← event instance
         )   ← ~600KB × 30Hz
           │
           ▼ Zenoh (Pi → PC, wire = topic substituted, payload = encoded event)
           │
PC process:
  CameraDecoded Module (robot-scoped)              ← derived read model
      @subscriber(Camera.Stream.JPEG)
      def on_jpeg(self, event: CameraJpegFrame):    ← type hint 으로 decode
          ndarray = cv2.imdecode(event.jpeg_bytes, IMREAD_COLOR)   ← decode 1회
          self.runtime.publish(
              Camera.Stream.DECODED,
              CameraDecodedFrame(robot_id=event.robot_id, ...),
          )
           │
           ▼ Zenoh same-session (PC 안)
           │
      ┌────┴────┬────────────┐
      ▼         ▼            ▼
   Detector  Calibration   Scene3D
      각자 @subscriber(Camera.Stream.DECODED) + event: CameraDecodedFrame
```

**핵심** — Decode 가 별도 Module 책임. 각 consumer 가 decode 박지 않음.

측정 ([backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py)):
- JPEG 1280x720 decode = **4.34ms**.
- 각 consumer 가 decode: 4.34ms × 30Hz × N = 130 × N ms/sec (N=3 → **39% CPU**).
- decode dedup 만 (Zenoh 단일): 4.34ms × 30 + ndarray transport × N = (130 + ~21 × N) ms/sec (N=3 → **21% CPU**).
- decode dedup + LocalTransport: 130 ms/sec (N=3 → **13% CPU**, 추가 절감 8%).

→ **decode dedup 이 first-order 절감** (39% → 21%). LocalTransport 의 추가 8% 는 단순성 우선으로 박지 X.

비슷한 패턴 적용 후보:
- **CameraDecoded** — JPEG → ndarray.
- **DepthDecoded** — zstd depth → uint16 ndarray + intrinsic.
- **TcpState** — joint → FK → TCP pose (sag 보정 포함) — 기존 backend 의 `motion_node._on_motor_state_publish_tcp` 가 한 일.
- **JointRad** — raw int → rad (joint_offset 적용) — 기존 `JointStateCache` 가 한 일.

framework 가 모름 — Module 의 한 유형 힌트.

### 3.6 Runtime lifecycle — instantiate → register → start

framework primitive 가 아닌 **Runtime contract**. `Runtime.start()` 의 부팅 순서:

```
① 모든 Module instantiate
       → constructor 호출 (DI: Repository / ObjectStore / robot_id 등 주입)
       → 모든 Module 의 객체 self 만들어짐
       
② 모든 Module 의 @service / @subscriber 등록
       → ZenohTransport 에 queryable / subscriber declare
       → 이 시점에 service 들이 cluster 안 visible
       
③ 모든 Module 의 start() 호출
       → Mirror snapshot fetch / background thread 시작 / hardware init 등
       → Mirror 가 다른 Module 의 service 호출하므로 ② 이후 박힘
       
④ Heartbeat / background workers
```

**왜 이 순서**:
- ③ 의 Mirror snapshot 이 다른 Module 의 `@service` 호출. ② 가 아직 안 됐으면 service register 안 된 상태 → snapshot fail (§3.3.1 의 fallback 으로 떨어짐. 단 항상 fallback 으로 떨어지는 건 design 의도 X).
- ② 와 ③ 분리 = framework 의 진짜 contract. instantiate + register 가 *모든 Module 동시* 끝난 후 start.

**③ 중간 실패 = 이미 start 된 Module rollback** — `start()` 가 예외(또는 SystemExit
등 BaseException) 로 중단되면, 그 전까지 성공한 Module 을 역순 `stop()` + endpoint
undeclare 후 re-raise. 방치하면 앞 Module 의 background thread / uvicorn task 가 좀비로
남아 프로세스 종료 자체를 막는다 (2026-07-07 사고: 유령 backend 가 :8000 점유 →
BridgeModule.start 실패 → rollback 없던 시절엔 pytest 프로세스 hang, [[project-verify-hang-stale-backend]]).
그래서 **BridgeModule.start 는 uvicorn 에 넘기기 전에 소켓을 직접 pre-bind** 한다 —
uvicorn 이 bind 실패 시 `sys.exit(1)` (SystemExit 로 이벤트 루프째 붕괴, rollback 스킵)
하는 걸 평범한 `RuntimeError` 로 바꿔 위 rollback 경로에 태우기 위함. 손으로 bind 하는
코드를 "불필요" 로 보고 되돌리지 말 것.

같은 process 의 Module 간 호출은 ZenohSession same-session in-routing 통과. 다른 process Owner 와는 Zenoh discovery / partition tolerance (§3.3.1 의 empty + fallback 그대로).

### 3.7 ModuleRuntime — Module 의 통신 surface

Module 이 framework 에 publish / call 요청하는 surface. Protocol 박고 constructor 로 주입.

```python
# framework/runtime/api.py
class ModuleRuntime(Protocol):
    """Module 이 Framework 에 요청하는 통신 surface."""

    def publish(self, key: str, event: BaseModel) -> None:
        """event publish. key 첫 인자 (explicit, typed StrEnum), event instance."""
        ...

    async def call(
        self,
        key: str,                              # contract key (StrEnum value)
        req: BaseModel,
        res_cls: type[TRes],                   # explicit — return type narrow + decode 시 사용
        *,
        robot_id: str | None = None,           # robot-scoped service 만 박힘
        timeout: float = 5.0,
    ) -> TRes:
        """service 호출. key + req + res_cls 세 인자 모두 explicit. method reference 박지 X."""
        ...
```

Module 측:

```python
from modules.motion.contract import Motion, MoveCompleted
from modules.calibration.contract import Calibration, CalibrationBundle, SnapshotRequest
from modules.motor.contract import Motor, SetTorqueRequest, SetTorqueResponse

class MotionModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        robot_id: str,
        repo: CalibrationRepository,
    ):
        self.runtime = runtime
        self.robot_id = robot_id
        self._repo = repo

    @service(Motion.Service.MOVE_L)
    def move_l(self, req: MoveLRequest) -> MoveLResponse:
        # key + event 두 인자 모두 typed
        self.runtime.publish(
            Motion.Event.COMPLETED,
            MoveCompleted(robot_id=self.robot_id, ...),
        )
        ...

    async def some_caller(self):
        # 다른 service 호출 — key + req + res_cls (모두 explicit)
        bundle = await self.runtime.call(
            Calibration.Service.SNAPSHOT_BUNDLE,                            # service key
            SnapshotRequest(robot_id=self.robot_id),                        # req
            CalibrationBundle,                                              # res_cls
        )
        # robot-scoped target — robot_id 명시
        await self.runtime.call(
            Motor.Service.SET_TORQUE,
            SetTorqueRequest(enabled=True),
            SetTorqueResponse,
            robot_id=self.robot_id,
        )
```

Runtime 측 — 인스턴스화 시점에 transport 어휘로 adapter 박아 inject:

```python
class _TransportRuntime:                # ModuleRuntime Protocol 만족
    def __init__(self, transport: Transport):
        self._transport = transport

    def publish(self, key: str, event: BaseModel) -> None:
        topic = str(key)                                # StrEnum value → str
        if "{robot_id}" in topic:
            # source = event payload 의 robot_id field (Module scope 무관 — uniform)
            assert hasattr(event, "robot_id"), (
                f"key {topic!r} 에 {{robot_id}} placeholder 박혀있지만 "
                f"event {type(event).__name__} payload 에 robot_id field 없음"
            )
            topic = topic.format(robot_id=event.robot_id)
        self._transport.publish(topic, encode_event(event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):
        key_str = str(key)                               # StrEnum value → str
        if "{robot_id}" in key_str:
            assert robot_id is not None, (
                f"service {key_str} 가 robot-scoped — call 시 robot_id= 인자 명시 필요"
            )
            key_str = key_str.format(robot_id=robot_id)
        payload_bytes = await self._transport.call(key_str, encode(req), timeout)
        return decode(res_cls, payload_bytes)            # res_cls 명시 — spec lookup 없음

# Runtime 부팅:
runtime_api = _TransportRuntime(transport)
instance = MotionModule(runtime=runtime_api, robot_id=rid, repo=repo)
```

**placeholder substitution source — 네 경로**:

| 위치 | source | 시점 |
|---|---|---|
| service queryable register | Module instance 의 `self.robot_id` (robot-scoped Module 만) | Runtime register 시 |
| event publish | event payload 의 `robot_id` field | `runtime.publish(event)` 시 |
| service call (caller side) | caller 의 `robot_id=` kwarg | `runtime.call(key, req, res_cls, robot_id=...)` 시 |
| event subscribe (robot-scoped event) | placeholder → Zenoh wildcard (transport detail) | `@subscriber` register 시 |

**event subscribe 의 wildcard 는 framework contract X** — transport (Zenoh) 의 single-chunk `*` 활용일 뿐, framework primitive 어휘로 노출 X. `@subscriber("*")` 같은 implicit pattern 금지. robot-scoped event subscribe 시 framework 가 placeholder 를 transport wildcard 로 substitute, 사용자 코드에 wildcard 어휘 등장 X.

→ robot-agnostic Module 이 robot-scoped event publish 도 자연 동작 (event payload 의 robot_id field 활용). subscriber 는 wildcard 후 payload `event.robot_id` 로 self-filter (Mirror 도 동일).

**왜 base class / setattr / ctx 박지 않나**:

- **base class** (`class MotionModule(Module)`) — backend/ `BaseNode` 부풀음 경험 (15+ method 누적: publish / log / heartbeat / lifecycle / placeholder expand …) 반복. 얇게 박아도 `_transport` 채우려면 setattr magic 또는 `super().__init__` 강제 → §10.6 의 "lifecycle 강제 X" 위반.
- **setattr inject** (`instance.publish = transport.publish`) — pyright 가 `self.publish` 못 보고 IDE 자동완성 X. §3.4 의 "4 surface 밖 통신 박지 X" 가 IDE 에 보이지 않으면 흔들림.
- **ctx (`RuntimeContext`)** — "context" 가 너무 광범위 (HTTP request context / Go context 와 충돌). 한 문장 정의 fail.
- **composition (`ModuleRuntime` Protocol)** — 명시 deps + Protocol type-safe + naming convention (`X 가 사용하는 Y` — `CalibrationRepository` / `JointStateCache` 와 정합).

**discipline — ModuleRuntime 에 박힐 surface 기준** (hard rule X, PR review 가이드):

| 후보 | ModuleRuntime | constructor 별도 parameter |
|---|---|---|
| publish (event broadcast) | ✅ | |
| call (RPC) | ✅ | |
| logger / metrics / clock | | ✅ |
| repository / object_store | | ✅ |
| Mirror (cross-module read) | | Mirror[T] descriptor (§3.3) |

기준 = **"Module 간 통신 surface 인가 vs 별도 framework concern 인가"**. 후자 = constructor 별도 parameter default. ModuleRuntime 에 박을 경우 PR description 에 정당화 명시.

평가 기준일 뿐 hard list 아님 — 새 후보 들어올 때마다 위 기준으로 판단.

## 4. Owner / Reader 비대칭 — code 형태

> ⚠️ **아래 §4.1–§4.2 의 Calibration↔Motion `Mirror[CalibrationBundle]` 코드는 Mirror 메커니즘 illustration (stand-in) 이다 (2026-07-02).** 실제 calibration 은 boot-time configuration 이라 Mirror 를 쓰지 않고 Motion 은 boot-time `snapshot_bundle` query 로 읽는다 ([calibration.md §6](calibration.md), anchor #2). Owner/Reader **비대칭 원칙 자체는 유효** — 다만 Reader 의 실제 접근이 Mirror 가 아니라 boot-query 인 경우 (calibration) 와 per-request service call 인 경우 (§4.4 Detector) 가 현 도메인의 실제 형태.

### 4.1 Owner side — Calibration Module

```python
# modules/calibration/contract.py — 외부 Public Surface
from enum import StrEnum
from datetime import datetime
from pydantic import BaseModel

class Calibration:
    class Service(StrEnum):
        ACTIVATE         = "srv/calibration/activate"
        SNAPSHOT_BUNDLE  = "srv/calibration/snapshot_bundle"

    class Event(StrEnum):
        ACTIVATED  = "event/calibration/activated"
        COMMITTED  = "event/calibration/committed"

# event payload
class CalibrationActivated(BaseModel):
    """active bundle 변경 (시스템 effective)."""
    robot_id: str
    bundle_id: int

class CalibrationCommitted(BaseModel):
    """새 bundle 저장 완료 (capture / BA → DB insert)."""
    robot_id: str
    bundle_id: int

# service request / response
class ActivateRequest(BaseModel):
    robot_id: str
    result_id: int

class ActivateResponse(BaseModel):
    ok: bool

class SnapshotRequest(BaseModel):
    robot_id: str

# Mirror value (snapshot service 의 response = Mirror 의 cache type)
class CalibrationBundle(BaseModel):
    """한 BA / 한 commit 의 atomic 단위. consumer 의 Mirror[T] type."""
    robot_id: str
    bundle_id: int
    joint_offsets: list[float]
    link_offsets:  list[LinkOffset]
    sag_offsets:   list[float]
    hand_eye:      Transform4x4
    intrinsic:     CameraIntrinsic
    commit_time:   datetime


# modules/calibration/models.py — SQLAlchemy ORM (internal)
class CalibrationResult(Base):
    __tablename__ = "calibration_results"
    id: Mapped[int] = mapped_column(primary_key=True)
    run_id: Mapped[int]
    transform: Mapped[bytes]       # 4x4 matrix serialize
    sigma_rot: Mapped[float]
    sigma_t: Mapped[float]
    is_active: Mapped[bool] = mapped_column(default=False)


# modules/calibration/repository.py — internal
class CalibrationRepository:
    def get_active_bundle(self, robot_id: str) -> CalibrationBundle | None: ...
    def save_result(self, robot_id: str, result: CalibrationResult) -> None: ...
    def activate(self, robot_id: str, result_id: int) -> None: ...    # atomic toggle


# modules/calibration/module.py — robot-agnostic, host 당 1 인스턴스
from .contract import (
    Calibration, CalibrationActivated, CalibrationCommitted,
    ActivateRequest, ActivateResponse, SnapshotRequest, CalibrationBundle,
)

@publishes(
    (Calibration.Event.ACTIVATED, CalibrationActivated),
    (Calibration.Event.COMMITTED, CalibrationCommitted),
)
class CalibrationModule:
    def __init__(self, runtime: ModuleRuntime, repo: CalibrationRepository):
        self.runtime = runtime
        self._repo = repo

    @service(Calibration.Service.ACTIVATE)
    def activate(self, req: ActivateRequest) -> ActivateResponse:
        self._repo.activate(req.robot_id, req.result_id)    # atomic toggle, transaction
        bundle = self._repo.get_active_bundle(req.robot_id)
        self.runtime.publish(
            Calibration.Event.ACTIVATED,
            CalibrationActivated(robot_id=req.robot_id, bundle_id=bundle.bundle_id),
        )
        return ActivateResponse(ok=True)

    @service(Calibration.Service.SNAPSHOT_BUNDLE)
    def snapshot_bundle(self, req: SnapshotRequest) -> CalibrationBundle:
        bundle = self._repo.get_active_bundle(req.robot_id)
        if bundle is None:
            raise NotFound(f"active calibration bundle 없음 (robot={req.robot_id})")
        return bundle
```

특징:
- **robot-agnostic** Module — host 당 1 인스턴스. 매 service request 안 `robot_id` field 로 dispatch.
- Repository 가 `robot_id` parameter 받음 — DB 의 `robot_id` column 으로 자연 multi-tenant.
- 도메인 event 의 `robot_id` field — Reader 측 Mirror 가 자기 robot 의 event 만 필터.
- `repo.save()` / `publish(...)` 직접. framework 가 mutation tracking 으로 자동화 X.
- `snapshot_bundle` 이 *명시적* service. Reader 가 부팅 시 호출할 endpoint.
- domain logic (active toggle 의 atomic 보장) Module 안.

### 4.2 Reader side — Motion Module

```python
# modules/motion/module.py
# robot-scoped — per-robot 인스턴스 (yaml 의 robots: [...] 박힘)
from modules.motion.contract import Motion, MoveLRequest, MoveLResponse
from modules.calibration.contract import (
    Calibration, CalibrationActivated, CalibrationBundle, SnapshotRequest,
)
from modules.motor.contract import Motor, MotorCmdJoint

class MotionModule:
    calibration: Mirror[CalibrationBundle] = Mirror(
        snapshot_service=Calibration.Service.SNAPSHOT_BUNDLE,
        snapshot_req=lambda self: SnapshotRequest(robot_id=self.robot_id),
        change_topic=Calibration.Event.ACTIVATED,
        value_cls=CalibrationBundle,                                    # cache T + snapshot res_cls
        change_event_cls=CalibrationActivated,                          # event class (decode)
        # framework 가 wire:
        #   snapshot 호출 = runtime.call(snapshot_service, snapshot_req(self), value_cls)
        #   event 필터링 = robot_id == self.robot_id (Mirror 가 payload.robot_id 검사)
    )

    def __init__(self, runtime: ModuleRuntime, robot_id: str):
        self.runtime = runtime
        self.robot_id = robot_id
        self._kinematics: Kinematics | None = None     # rebuild on link_offset change

    def start(self):
        # Mirror 가 ready 되면 첫 kinematics build
        if self.calibration.is_ready:
            self._kinematics = self._build_kinematics(self.calibration.value)

    @subscriber(Calibration.Event.ACTIVATED)             # Mirror 와 같은 event key
    def on_calibration_change(self, event: CalibrationActivated):
        # link_offset 이 PyBullet URDF 에 박혀있어 재로드 필요 — consumer 책임
        # joint / sag / hand_eye 는 매 access fresh 라 rebuild 불필요
        self._kinematics = self._build_kinematics(self.calibration.value)

    @service(Motion.Service.MOVE_L)
    def move_l(self, req: MoveLRequest) -> MoveLResponse:
        if not self.calibration.is_ready:
            raise NotReady("calibration 아직 동기화 안 됨")
        cal = self.calibration.value           # 매 호출 fresh
        target_in_base = cal.hand_eye @ req.target_in_camera
        joints = self._kinematics.ik(target_in_base)
        # cal.joint_offsets / cal.sag_offsets 도 kinematics 내부 매 호출 fresh access
        self.runtime.publish(
            Motor.Stream.CMD_JOINT,                            # stream key (100Hz)
            MotorCmdJoint(robot_id=self.robot_id, joints=joints),
        )
        return MoveLResponse(ok=True)
```

특징:
- `self.calibration.value` 매 호출 fresh — sub-field (`cal.hand_eye`, `cal.joint_offsets`) 는 access 시점에 골라 씀.
- `CalibrationActivated` event 받으면 framework 자동 cache 갱신. 단 **PyBullet 재로드 같은 architectural side-effect 는 consumer 가 같은 event 박아 자기 처리**.
- `Mirror` mapping 한 번 박으면 lifecycle 전체 흡수.

### 4.3 비대칭 표

| 자리 | 누가 박나 |
|---|---|
| Owner 의 `repo.save()` | 개발자 (business intent) |
| Owner 의 `publish(Event)` | 개발자 (domain 의미) |
| Owner 의 `snapshot_*` service | 개발자 (service 한 줄, repo.get_active 호출만) |
| Reader 의 부팅 시 snapshot 호출 | framework |
| Reader 의 event subscribe | framework |
| Reader 의 cache management | framework |
| Reader 의 `self.calibration.value` access surface | framework |
| Reader 의 architectural side-effect (PyBullet 재로드 등) 처리 | 개발자 (consumer 가 같은 event 박아 자기 처리) |

### 4.4 robot-agnostic Reader — Detector Module

```python
# robot-agnostic — host 당 1 인스턴스. YOLO model robot 무관.
# 매 detect 호출 시 req.robot_id 로 dispatch. Mirror 박지 않음 — service call 로.
from modules.detector.contract import Detector, DetectRequest, DetectResponse
from modules.camera.contract import Camera, CameraDecodedFrame
from modules.camera.contract import SnapshotRequest as CameraSnapshotRequest
from modules.calibration.contract import Calibration, CalibrationBundle, SnapshotRequest

class DetectorModule:
    def __init__(self, runtime: ModuleRuntime):
        self.runtime = runtime
        self._yolo = YOLO(...)    # model load 1 회

    @service(Detector.Service.DETECT)
    async def detect(self, req: DetectRequest) -> DetectResponse:
        # robot 별 frame / calibration = 매 호출 service call (Mirror 안 박음)
        # key + req + res_cls + robot_id 모두 explicit
        frame = (await self.runtime.call(
            Camera.Service.DECODED_SNAPSHOT,
            CameraSnapshotRequest(),
            CameraDecodedFrame,
            robot_id=req.robot_id,
        )).to_ndarray()
        bundle = await self.runtime.call(
            Calibration.Service.SNAPSHOT_BUNDLE,
            SnapshotRequest(robot_id=req.robot_id),
            CalibrationBundle,
        )
        boxes = self._yolo(frame)
        # 카메라 → base 변환 (calibration_apply_flow §4)
        objects_in_base = self._project(boxes, bundle.hand_eye, bundle.intrinsic, req.tcp_pose)
        return DetectResponse(objects=objects_in_base)
```

특징:
- **robot-agnostic** — YOLO model robot 무관 (같은 가중치), 매 detect 호출 시 robot_id 로 dispatch.
- Mirror 박지 않음 — `detect` 호출 빈도 낮음 (5Hz / 사용자 trigger). 매 호출 service call OK.
- 고빈도 detect 필요 (예: realtime visual servo) 가 생기면 그때 Mirror 또는 robot-scoped sub-module 고려.

## 5. 폴더 구조

```
backend/
│
├── framework/                    # 변하지 않는 시스템 기반
│   │
│   ├── contract/                 # Service / Event / Mirror 데코 + spec
│   │   ├── service.py            # @service(key) factory + ServiceSpec
│   │   ├── subscriber.py         # @subscriber(key) factory + SubscriberSpec
│   │   ├── publisher.py          # @publishes((key, event_cls) pairs) + encode/decode_event (msgpack)
│   │   ├── mirror.py             # Mirror[T] descriptor + MirrorSpec (5 인자)
│   │   └── envelope.py           # ServiceRequest/ServiceResponse Pydantic generic
│   │
│   ├── runtime/                  # Module lifecycle + DI 주입
│   │   ├── api.py                # ModuleRuntime Protocol — Module 의 통신 surface (§3.7)
│   │   ├── app.py                # Runtime: yaml → Module instantiate → start
│   │   ├── lifecycle.py          # Lifecycle Protocol (start / stop)
│   │   └── discovery.py          # Module instance 의 @service / @subscriber / Mirror scan
│   │
│   ├── transport/                # Transport Protocol (Zenoh 단일)
│   │   └── protocol.py           # Transport(Protocol)
│   │
│   ├── persistence/              # Repository Protocol (DB 모름)
│   │   └── protocol.py           # Repository(Protocol)
│   │
│   └── storage/                  # ObjectStore Protocol (S3/MinIO/fs 모름)
│       └── protocol.py           # ObjectStore(Protocol)
│
├── infra/                        # framework Protocol 의 실 구현 (외부 dep 가짐)
│   │
│   ├── transport/
│   │   └── zenoh.py              # ZenohTransport — Zenoh session wrap
│   │
│   ├── database/
│   │   ├── sqlite.py             # SQLAlchemy + sqlite (dev / mock)
│   │   └── postgres.py           # SQLAlchemy + psycopg (운영 NAS)
│   │
│   └── object_store/
│       ├── filesystem.py         # local fs (dev / mock)
│       └── minio.py              # boto3 (S3 compat, 운영 NAS)
│
├── modules/                      # 도메인 기능 — entity 추가 시 여기만 큼
│   │
│   ├── calibration/              # business domain (영속성 owner)
│   │   ├── contract.py           # Public Surface — Service/Event nested StrEnum + Pydantic payload
│   │   ├── models.py             # SQLAlchemy ORM class (internal)
│   │   ├── repository.py         # CalibrationRepository (internal)
│   │   ├── service.py            # business logic (BA / IRLS / observability) (internal)
│   │   └── module.py             # @publishes + @service + @subscriber entry
│   │
│   ├── scan/                     # business domain
│   │   ├── contract.py
│   │   ├── models.py
│   │   ├── repository.py
│   │   ├── artifact.py           # ObjectStore 사용 (scans blob)
│   │   └── module.py
│   │
│   ├── reconstruction/           # business domain (Reader of scan)
│   │   ├── contract.py
│   │   ├── models.py
│   │   ├── pipeline.py           # ICP + PoseGraph + TSDF
│   │   ├── artifact.py
│   │   └── module.py
│   │
│   ├── task/                     # business domain (orchestrator)
│   │   ├── contract.py
│   │   ├── models.py
│   │   ├── repository.py
│   │   ├── dsl/                  # Step / Slot / Recipe — 기존 step_dsl 옮겨심음
│   │   └── module.py
│   │
│   ├── motion/                   # robot-scoped (per-robot kinematics state)
│   │   ├── contract.py           # {robot_id} placeholder 박힌 nested StrEnum
│   │   ├── kinematics.py         # PyBullet + sag corrected
│   │   ├── trajectory.py         # Ruckig
│   │   ├── jog.py                # SE(3) 적분
│   │   └── module.py             # MotionModule(robot_id) + boot-time snapshot_bundle query
│   │
│   ├── motor/                    # robot-scoped (Dynamixel device handle)
│   │   ├── contract.py
│   │   ├── driver/
│   │   │   ├── dynamixel.py
│   │   │   └── feetech.py
│   │   └── module.py             # MotorModule(robot_id)
│   │
│   ├── camera/                   # robot-scoped (RealSense device + per-robot frame)
│   │   ├── contract.py
│   │   ├── driver/
│   │   │   ├── realsense.py
│   │   │   └── mock.py
│   │   ├── module.py             # CameraDriver(robot_id) — raw JPEG / zstd depth
│   │   ├── decoded.py            # CameraDecoded(robot_id) — JPEG → ndarray (derived)
│   │   └── depth_decoded.py      # DepthDecoded(robot_id) — zstd depth → uint16
│   │
│   ├── detector/                 # robot-agnostic (YOLO model robot 무관)
│   │   ├── contract.py
│   │   ├── yolo.py
│   │   └── module.py             # DetectorModule — 매 detect 호출에 req.robot_id
│   │
│   ├── scene3d/                  # robot-agnostic (RGBD primitive service)
│   │   ├── contract.py
│   │   └── module.py
│   │
│   └── gamepad/                  # robot-agnostic (UI input)
│       ├── contract.py
│       └── module.py
│
├── deployments/                  # 어떤 process 에 어떤 Module 띄울지
│   ├── pc.yaml                   # 예시 ↓
│   ├── pi_motor.yaml
│   ├── pi_camera.yaml
│   ├── dev.yaml                  # PC 한 process 에 다 띄움 (Zenoh same-session)
│   └── mock.yaml                 # hardware mock 으로 swap
│
├── apps/
│   └── main.py                   # 한 entry. uv run python apps/main.py --host pc
│
└── tests/
    ├── framework/                # framework 단위 test
    └── modules/                  # Module integration test (Zenoh in-process peer)
```

**module 안 파일 책임 분리** (도메인 boundary):

| 파일 | 역할 | 외부 import 가능? |
|---|---|---|
| `contract.py` | Public Surface — Service/Event/Stream nested StrEnum + Pydantic event/req/res/bundle | ✅ |
| `module.py` | framework entry — `@publishes` / `@service` / `@subscriber` / `Mirror` 박힌 class | ❌ (framework Runtime 만 instantiate) |
| `models.py` | SQLAlchemy ORM | ❌ (Repository 안에서만) |
| `repository.py` | Repository class (framework Protocol 만족) | ❌ (module.py 가 DI 받음) |
| `service.py` | business logic (BA / IRLS 등) | ❌ (module.py 가 호출) |
| `artifact.py` | ObjectStore 사용 (scan blob / mesh 등) | ❌ |
| `driver/` (motor / camera) | 하드웨어 driver | ❌ |
| `alembic/` | migration | ❌ |

**contract.py 의 진화 path** — 첫 박을 때 단일 파일, 비대해지면 (1000 줄+) `contract/` 패키지로:
```
modules/calibration/contract/
  __init__.py     # re-export (외부 import path 자체 안 바뀜)
  keys.py
  events.py
  services.py
```
외부 module 의 `from modules.calibration.contract import ...` 자체 동일 — 내부만 refactor.

## 6. 데이터 흐름

### 6.1 Calibration activate (Owner side)

```
사용자 UI
   │
   ▼
runtime.call(
    Calibration.Service.ACTIVATE,                         # service key
    ActivateRequest(robot_id="omx_f_0", result_id=10),    # req
    ActivateResponse,                                     # res_cls
)
   │  ↑ 세 인자 모두 explicit — service 가리키는 방법 = enum 한 패턴
   ▼
CalibrationModule.activate:
   repo.get(10)
   result.activate()
   repo.save(result)
   runtime.publish(
       Calibration.Event.ACTIVATED,                              ← event key 첫 인자
       CalibrationActivated(robot_id="omx_f_0", bundle_id=...),  ← event instance 두 번째
   )
   ▼
ZenohTransport:
   ├─ 같은 process subscriber → Zenoh same-session in-routing
   └─ 다른 process subscriber → Zenoh between-session (network)
```

### 6.2 Motion read calibration (Reader side)

> ⚠️ **SUPERSEDED (2026-07-02) — 아래 Mirror 흐름은 stand-in illustration.** 실제 calibration = boot-time config → Motion 의 실제 흐름은: `start()` 에서 `runtime.call(Calibration.Service.SNAPSHOT_BUNDLE, ...)` **1회** → kinematics build → 끝. subscribe / event refetch / 런타임 cache 갱신 **없음**. 아래 "런타임: Calibration 측 activate → refetch" 부분은 일어나지 않는다 (activate = "재시작 필요" 알림, [calibration.md §5/§9](calibration.md)). Mirror 흐름의 메커니즘 예시로만 유지.

```
부팅 시점:
   MotionModule.start()
        │
        ▼
   framework discovery 가 Mirror[CalibrationBundle] 발견
        │  Mirror(snapshot_service=..., snapshot_req=..., change_topic=..., value_cls=..., change_event_cls=...)
        ▼
   runtime.call(
       Mirror.snapshot_service,                    # Calibration.Service.SNAPSHOT_BUNDLE
       Mirror.snapshot_req(self),                  # SnapshotRequest(robot_id=self.robot_id)
       Mirror.value_cls,                           # CalibrationBundle
   )
        │  ↑ Mirror 가 key + req factory + res_cls 모두 explicit 박음 (lookup 없음)
        ▼
   결과 local cache 저장
        │
        ▼
   subscribe(Mirror.change_topic)  ─ Calibration.Event.ACTIVATED (Mirror config)
        ← payload.robot_id 로 filter (Mirror invariant: self.robot_id 만 박음)
        ← decode 는 change_event_cls (CalibrationActivated)


런타임:
   MotionModule.move_l(...)
        │
        ▼
   self.calibration.value  ← fresh cache read (network 0)

   ─────

   Calibration 측 activate 발생
        │
        ▼
   runtime.publish(Calibration.Event.ACTIVATED, CalibrationActivated(...))
        │
        ▼
   Reader subscriber callback → cache refetch (snapshot_bundle 재호출, §3.3.5)
```

### 6.3 Scan capture → Reconstruction (cross-module 영속성)

```
TaskModule.scan_task 실행:
   │
   ├─ for each pose:
   │     MotionModule.move_j(...)
   │     ScanModule.capture()   ─ camera frame + zstd depth + ObjectStore put
   │
   └─ ReconstructionModule.build(session_id)
         │
         ├─ ScanModule.list_scans(session_id)  ─ scan metadata
         ├─ ScanModule.get_blob(scan_id)        ─ ObjectStore get
         ├─ ICP + PoseGraph + TSDF
         └─ ObjectStore put (mesh.ply)
              + publish(ReconstructionBuilt(...))
```

각 Module 이 자기 DB + ObjectStore 영역 owner. cross-module call 은 standard `@service`.

### 6.4 Camera frame — decode dedup 흐름

```
Pi process:
  CameraDriver Module (robot-scoped, self.robot_id="omx_f_0")
       ├─ RealSense capture (BGR ndarray + uint16 depth)
       ├─ cv2.imencode JPEG / zstd compress depth
       └─ runtime.publish(
              Camera.Stream.JPEG,                                      ← stream key 첫 인자
              CameraJpegFrame(robot_id=self.robot_id, jpeg_bytes=...), ← event instance
          )
            │
            ▼ Zenoh (Pi → PC, network)
            │
PC process — 한 process / 한 Zenoh session:
  CameraDecoded Module
       @subscriber(Camera.Stream.JPEG)
       on_jpeg(self, event: CameraJpegFrame):       ← type hint 으로 decode
           ndarray = cv2.imdecode(event.jpeg_bytes, ...)         ← decode 1회 (4.34ms × 30Hz)
           self.runtime.publish(
               Camera.Stream.DECODED,
               CameraDecodedFrame(robot_id=event.robot_id, ndarray_bytes=...),
           )
            │
            ▼ Zenoh same-session (PC 안)
            │
       ┌────┴──────┬───────────┬─────────────────┐
       ▼           ▼           ▼                 ▼
   Detector   Calibration   Scene3D     Bridge (raw JPEG forward)
                                          ← Bridge 는 @subscriber(Camera.Stream.JPEG)
                                            (decode 안 함, jpeg_bytes 그대로 WS)
```

Bridge 는 WebSocket 에 raw JPEG bytes 그대로 forward — decode 0. `Camera.Stream.JPEG` 직접 subscribe (CameraDecoded 안 거침).

decoded ndarray 가 필요한 consumer (Detector, Calibration, Scene3D) 는 `Camera.Stream.DECODED` subscribe.

## 7. Module 구조

### 7.1 Module = plain class

base class 강제 X, `@module` 데코 X. framework 가 `@service` / `@subscriber` / `Mirror` 박힌 메소드/속성만 inspect.

```python
# robot-agnostic
from .contract import Calibration, ActivateRequest, ActivateResponse, SnapshotRequest, CalibrationBundle

class CalibrationModule:
    # 생성자 — Runtime 이 DI injection (ModuleRuntime + Repository + ObjectStore 등)
    def __init__(self, runtime: ModuleRuntime, repo: CalibrationRepository):
        self.runtime = runtime
        self._repo = repo

    # lifecycle — Lifecycle Protocol (선택, 안 박아도 됨)
    def start(self) -> None: ...
    def stop(self) -> None: ...

    # contract — framework 가 발견. @service 의 인자 = nested StrEnum value.
    @service(Calibration.Service.ACTIVATE)
    def activate(self, req: ActivateRequest) -> ActivateResponse: ...

    @service(Calibration.Service.SNAPSHOT_BUNDLE)
    def snapshot_bundle(self, req: SnapshotRequest) -> CalibrationBundle: ...


# robot-scoped — yaml `robots: [...]` 박힘. constructor 의 robot_id 가 계약 검증.
from .contract import Motion

class MotionModule:
    def __init__(self, runtime: ModuleRuntime, robot_id: str):
        self.runtime = runtime
        self.robot_id = robot_id

    @service(Motion.Service.MOVE_L)                      # = "srv/motion/{robot_id}/move_l"
    def move_l(self, req): ...                           # Module register 시 {robot_id} 자동 substitute
```

scope 결정 = §2.7 참조 (yaml primary, constructor 계약 검증).

### 7.2 Module 안 책임 분리

(폴더 구조 §5 의 표와 동일 — 한 번 더 정리):

| 파일 | 책임 | 외부 import? |
|---|---|---|
| `contract.py` | Public Surface — nested StrEnum (Service / Event / Stream) + Pydantic (event / req / res / bundle) | ✅ |
| `module.py` | framework entry — `@publishes` / `@service` / `@subscriber` / `Mirror` 박힌 class | ❌ |
| `models.py` | SQLAlchemy ORM (Aggregate root + child relationship) | ❌ |
| `repository.py` | Repository (framework Repository Protocol 만족) | ❌ |
| `service.py` | business logic (BA / IRLS / orchestration — module.py 가 호출) | ❌ |
| `artifact.py` | ObjectStore 사용 (scan blob / mesh 등) | ❌ |

DDD 폴더 모양 (`domain/entities.py`, `domain/value_objects.py`) 박지 않음. Aggregate boundary 의 사고만 가져옴 — 클래스 관계 (SQLAlchemy `relationship` + cascade) 로 표현.

### 7.3 Aggregate root 예 — CalibrationRun

```python
class CalibrationRun(Base):
    __tablename__ = "calibration_runs"
    id: Mapped[int] = mapped_column(primary_key=True)
    status: Mapped[str]
    started_at: Mapped[datetime]

    captures: Mapped[list["Capture"]] = relationship(cascade="all, delete-orphan")
    results: Mapped[list["CalibrationResult"]] = relationship(cascade="all, delete-orphan")

    def finalize(self, ba_output) -> None:
        self.status = "ready_for_analysis"
        self.results.append(CalibrationResult.from_ba(ba_output))
```

Aggregate boundary = transaction boundary. `finalize()` 호출 = run row update + result INSERT 가 한 transaction.

### 7.4 Derived read model Module — 코드 형태

큰 payload decode 비용이 N consumer 마다 누적되는 자리에 박는 패턴 (§3.5).

```python
# modules/camera/contract.py — Public Surface
from enum import StrEnum
from pydantic import BaseModel
import numpy as np

class Camera:
    class Service(StrEnum):
        DECODED_SNAPSHOT = "srv/camera/{robot_id}/decoded_snapshot"

    class Stream(StrEnum):
        JPEG     = "stream/camera/{robot_id}/jpeg"
        DECODED  = "stream/camera/{robot_id}/decoded"

class CameraJpegFrame(BaseModel):
    robot_id: str
    timestamp: float
    jpeg_bytes: bytes

class CameraDecodedFrame(BaseModel):
    robot_id: str
    timestamp: float
    width: int
    height: int
    ndarray_bytes: bytes        # 압축 안 된 BGR raw

    def to_ndarray(self) -> np.ndarray:
        return np.frombuffer(self.ndarray_bytes, dtype=np.uint8).reshape(
            self.height, self.width, 3
        )

class SnapshotRequest(BaseModel):
    pass  # robot-scoped service — robot_id 는 caller 가 인자로 명시


# modules/camera/decoded.py — derived read model Module
from .contract import Camera, CameraJpegFrame, CameraDecodedFrame, SnapshotRequest

@publishes(
    (Camera.Stream.DECODED, CameraDecodedFrame),
)
class CameraDecoded:
    def __init__(self, runtime: ModuleRuntime, robot_id: str):
        self.runtime = runtime
        self.robot_id = robot_id
        self._latest: CameraDecodedFrame | None = None

    @subscriber(Camera.Stream.JPEG)
    def on_jpeg(self, event: CameraJpegFrame) -> None:
        arr = cv2.imdecode(np.frombuffer(event.jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            return
        frame = CameraDecodedFrame(
            robot_id=event.robot_id,
            timestamp=event.timestamp,
            width=arr.shape[1],
            height=arr.shape[0],
            ndarray_bytes=arr.tobytes(),
        )
        self._latest = frame
        self.runtime.publish(Camera.Stream.DECODED, frame)

    @service(Camera.Service.DECODED_SNAPSHOT)
    def snapshot(self, req: SnapshotRequest) -> CameraDecodedFrame:
        if self._latest is None:
            raise NotReady("아직 첫 jpeg 안 옴")
        return self._latest
```

특징:
- Decode 1 회, 결과 publish 로 fanout.
- `@service(...) snapshot` 박아두면 consumer 가 `Mirror[CameraDecodedFrame]` 으로 받음 — late-join + reactive.
- framework primitive 아님 — 그저 일반 Module + `@subscriber` + `publish` + `@service`. 개발자 책임.

consumer 측 (robot-scoped Reader 예 — robot-agnostic Detector 는 §4.4 처럼 매 호출 service call 이 더 자연):
```python
from modules.camera.contract import Camera, CameraDecodedFrame, SnapshotRequest
from modules.detector.contract import Detector

class DetectorModule:
    camera: Mirror[CameraDecodedFrame] = Mirror(
        snapshot_service=Camera.Service.DECODED_SNAPSHOT,
        snapshot_req=lambda self: SnapshotRequest(),
        change_topic=Camera.Stream.DECODED,
        value_cls=CameraDecodedFrame,                              # cache T
        change_event_cls=CameraDecodedFrame,                       # event class (decode)
    )

    @service(Detector.Service.DETECT)
    def detect(self, req):
        frame = self.camera.value                            # fresh CameraDecodedFrame
        arr = frame.to_ndarray()                             # 이미 decoded
        return self._yolo(arr)
```

## 8. DIP — Framework Protocol vs Infra impl

### 8.1 Repository Protocol

```python
# framework/persistence/protocol.py
class Repository(Protocol[T]):
    def get(self, id: int) -> T | None: ...
    def save(self, entity: T) -> None: ...
    def delete(self, id: int) -> None: ...
```

Module 의 Repository 가 이 Protocol 만족 + entity-specific method 추가:

```python
# modules/calibration/repository.py
class CalibrationRepository:
    def __init__(self, session_factory: Callable[[], Session]):
        self._session_factory = session_factory

    def get(self, result_id: int) -> CalibrationResult: ...
    def save(self, result: CalibrationResult) -> None: ...
    def get_active(self) -> CalibrationResult | None: ...    # entity-specific
    def list_by_kind(self, kind: str) -> list[CalibrationResult]: ...
```

framework 가 Repository class 자체 만들지 않음. Module 이 자기 ORM 알고 짜는 게 정직. framework Protocol 은 *type bound* 만.

### 8.2 ObjectStore Protocol

```python
# framework/storage/protocol.py
class ObjectStore(Protocol):
    def put(self, key: str, data: bytes) -> None: ...
    def get(self, key: str) -> bytes: ...
    def delete(self, key: str) -> None: ...
    def list(self, prefix: str) -> list[str]: ...
```

infra impl:
- `infra/object_store/filesystem.py` — local dev / mock
- `infra/object_store/minio.py` — production (boto3)

### 8.3 Transport Protocol

```python
# framework/transport/protocol.py
class Transport(Protocol):
    async def call(self, key: str, payload: bytes, timeout: float) -> bytes: ...
    def publish(self, key: str, payload: bytes) -> None: ...
    def register_service(self, key: str, handler: Callable[[bytes], bytes]) -> Handle: ...
    def subscribe(self, key: str, callback: Callable[[bytes], None]) -> Handle: ...
```

impl:
- `infra/transport/zenoh.py` — `ZenohTransport`. Zenoh session wrap. 유일.

LocalTransport 박지 않음 (§3.4 / §10.8 — 측정 결과 기반 결정). test 는 Zenoh in-process peer 사용.

### 8.4 DI 주입

`apps/main.py` 가 deployment yaml 파싱 + infra 인스턴스 생성 + Runtime 에 주입:

```python
# apps/main.py
def main(host: str):
    cfg = load_yaml(f"deployments/{host}.yaml")

    transport = make_transport(cfg.transport)        # Zenoh (single impl)
    session_factory = make_session(cfg.database)     # sqlite / postgres
    object_store = make_object_store(cfg.storage)    # fs / minio

    runtime = Runtime(transport=transport)           # Runtime 이 ModuleRuntime adapter 박음
    for mod_cfg in cfg.modules:
        mod_cls = MODULE_REGISTRY[mod_cfg.name]
        if mod_cfg.robots:
            # robot-scoped — per-robot 인스턴스
            for rid in mod_cfg.robots:
                runtime.add_module(
                    mod_cls,
                    robot_id=rid,
                    session_factory=session_factory,
                    object_store=object_store,
                )
        else:
            runtime.add_module(
                mod_cls,
                session_factory=session_factory,
                object_store=object_store,
            )

    runtime.start()
```

Runtime 내부 `add_module` 은 `inspect.signature(cls.__init__)` 로 constructor parameter list 추출 후 매칭 inject (`runtime: ModuleRuntime` / `robot_id: str` / `repo: CalibrationRepository` / `object_store: ObjectStore` 등). Module 은 자기 dep 를 constructor 로 받음. **FastAPI Depends 식 lazy DI container 박지 않음** — manual constructor injection 으로 충분.

## 9. Storage Module 폐기

기존 [storage_layer.md](storage_layer.md) 의 Storage Module 은 본 spec 에서 사라짐. 그 3 motivation 이 다음으로 흡수:

### 9.1 Centralization (분산 동기화)

기존: 모든 entity 가 한 Storage Module 의 service 통해 영속화. Cross-module read 도 Storage Module 거침.

새 spec: 각 도메인 Module 이 자기 영속성 owner. Cross-module read = `Mirror[T]` primitive. Storage Module 가운데 끼는 wire 사라짐.

### 9.2 Migration owner — 루트 단일 Alembic (2026-07-02 정정)

> **초안의 "Module N 개 = Alembic N 개" 폐기.** 소유권 ≠ 마이그레이션 권위 ([calibration.md §8](calibration.md)):
> - **테이블/ORM/Repository 소유 = 모듈별** (Storage *Module* RPC 중개자 폐기는 그대로).
> - **마이그레이션 = 루트 하나** (`backend/alembic/`, 공유 `infra/database/base.py::Base`). 같은 프로세스 + 공유 DB = Database-per-**Service** 아님. per-module Alembic 은 version_table 충돌 / cross-module FK 순서 / 전체 초기화 복잡도만 들여옴.

기존: Storage Module 부팅 시 Alembic `upgrade head` 한 번.

새 spec: 루트 `backend/alembic/env.py` 가 모든 DB 모듈 ORM 을 import → 공유 `Base.metadata` 단일 history. runtime `upgrade head` 는 apps boot(또는 DB owner 모듈)가 프로그래매틱 실행. Pi 는 alembic 실행/import 안 함 (PC 전용 도구 — role 격리 유지). **구현·검증됨** (`tests/modules/test_alembic.py`).

### 9.3 DB dependency 격리

기존: Pi 가 Storage Module service 호출, SQLAlchemy import 0.

새 spec: Pi 의 Module 들 (motor / motion / camera) 은 *Reader 만*, 자기 DB 안 가짐. PC 의 Calibration Module 이 owner, Pi 의 Motion 은 boot 시 `snapshot_bundle` query (PC 의 Calibration service 호출) 로 받음. Pi 에 SQLAlchemy / Postgres driver import 0 유지 — DB 접근은 owner(PC) 만, Reader 는 wire 로 받으니 dependency 격리는 boot-query 든 Mirror 든 동일하게 성립.

→ Storage Module 사라지고도 3 motivation 다 만족.

## 10. 하지 않는 것

### 10.1 React-style reactive state framework

`@state` 데코 / mutation tracking / partial state diff / reactive dependency graph 박지 않음. Owner 의 `repo.save()` + `publish(Event)` 가 명시적. DB update ≠ domain event — 의미는 Owner 만 결정.

### 10.2 DI container (FastAPI Depends 식)

call-time lazy resolution 안 박음. HTTP request lifecycle 에 묶인 패턴이라 우리 process-scoped service 에는 정당화 약함. Manual constructor injection + lazy singleton (Repository / ObjectStore 등) 으로 충분.

### 10.3 DDD tactical 폴더 (entities / value_objects / domain layer)

DDD 의 *사고* (Aggregate boundary / 소유 / 변경 동시성) 만 가져옴. 폴더 모양 (`domain/entities.py`, `domain/value_objects.py`) 박지 않음. Aggregate 는 SQLAlchemy `relationship` + cascade 로 자연 표현.

### 10.4 Generic Repository ORM framework

framework 가 SQLAlchemy class 자동 generate / migration auto-apply / query builder 박지 않음. 그저 Repository Protocol 만 정의. Module 이 자기 ORM 직접 짬.

### 10.5 "기술 갈아끼우기 자유도" 명분

"미래 Zenoh → ROS2", "미래 Postgres → MongoDB" 같은 자유도 motivation 으로 Protocol 박지 않음. 진짜 motivation = test mock + import boundary 두 개만.

### 10.6 `@module` 데코 / 클래스 hierarchy 강제

Module = plain Python class. `@module(...)` 데코 박지 않음 (deployment 결정은 yaml 의 책임, 코드 안 host 박지 X). Lifecycle 도 Protocol — base class 상속 강제 X.

### 10.7 한 entry point 여러 개

`apps/robot_runtime.py` + `backend_runtime.py` 식 분리 박지 않음. `apps/main.py` 한 entry + `--host` 인자 + deployment yaml. 기존 backend `main.py` 의 host 자동 감지 + yaml 로딩 패턴 그대로.

### 10.8 LocalTransport / process-local fast-path

같은 process 안 Module 간 호출이 Zenoh 안 거치고 `dict[key] → callback` direct dispatch 박는 자리 = **박지 않음** (§3.4). 측정 ([backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py)) 결과:

- 작은 message: Zenoh same-session ~3us — 가치 0.
- 큰 ndarray (5MB) fanout: ~4% × N CPU 절감 — 단 framework 두 갈래 유지 비용 (Transport 두 impl + resolver + behavior 일관성 risk) 보다 작음.

decode dedup 패턴 (§3.5) 으로 카메라 자리 39% → 21% CPU. LocalTransport 추가는 21% → 13% (8% 더), 단 측정 후 진짜 bottleneck 으로 드러나면 그때 추가. 지금부터 박지 않음.

### 10.9 Runtime resolver / provider locality 결정

§10.8 의 LocalTransport 박지 않음 결정의 자연 귀결. transport 한 갈래 (Zenoh) 라 *어디로 보낼지* 선택할 자리 자체 없음. Runtime 의 책임은 lifecycle + DI + Zenoh queryable/subscriber wire 만.

## 11. 달성 단계

순차. 각 step 끝 = 검증 가능한 산출물.

### Step 1 — Transport abstraction (Zenoh 단일)

`framework/transport/protocol.py` + `infra/transport/zenoh.py`.

검증:
- `ZenohTransport.publish(key, b"...") → 같은 session 안 subscriber callback 발동` (same-process, in-session routing).
- `ZenohTransport.publish(...) → 다른 process subscriber callback 발동` (cross-process, host_mock subprocess).

같은 process 안 routing 도 Zenoh same-session 통과 — 측정 결과 ([backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py)) 작은 message ~3us, 5MB ~1.27ms.

**✅ 완료** — [backend/framework/transport/protocol.py](../backend/framework/transport/protocol.py) + [backend/infra/transport/zenoh.py](../backend/infra/transport/zenoh.py). **7 test PASS** — same-session pub/sub + service call + handler exception → `RemoteError` + timeout → `TimeoutError` + callback exception swallow + cross-process pub/sub (subprocess).

### Step 2 — Contract layer

`framework/contract/{service,subscriber,publisher,envelope}.py`. Pydantic generic envelope + `@service` / `@subscriber` / `@publishes` 데코 + spec 수집.

검증:
- `@service` 박은 메소드를 framework 가 inspect 해서 ServiceSpec 추출.
- ZenohTransport 위에 service register + same-session call round-trip.

**✅ 완료** — [backend/framework/contract/](../backend/framework/contract/). **19 test PASS** — service / subscriber spec 추출 + invalid type hint fail-fast + `@publishes(*pairs)` class 데코 + envelope round-trip + E2E ZenohTransport wire + handler exception E2E + event publish/subscribe E2E.

### Step 3 — Runtime + Module discovery

`framework/runtime/{api,app,lifecycle,discovery}.py`. Module 인스턴스 → spec 수집 → transport 바인딩 → lifecycle.

산출물:
- `api.py` — `ModuleRuntime` Protocol (§3.7). `publish(key, event)` + `call(key, req, res_cls, *, robot_id=, timeout=)`.
- `app.py` — `Runtime` (add_module + start + stop) + `_TransportRuntime` adapter. `{robot_id}` placeholder substitute 4 경로 (register / publish / call / subscribe wildcard).
- `lifecycle.py` — `Lifecycle` Protocol (`start` / `stop`, 선택). sync / async 둘 다.
- `discovery.py` — `discover_services` / `discover_subscribers` helper.

부팅 순서 = **instantiate → register → start** (§3.6).

**✅ 완료** — [backend/framework/runtime/](../backend/framework/runtime/). **12 test PASS**:
- 빈 Module runtime start → stop
- 두 Module + service call round-trip (`runtime.call(Module.Service.X, req, ResCls)`)
- publish → `@subscriber` callback 도달
- Module A start() 가 Module B service 호출 (phase 2 register → phase 3 start 순서 검증)
- robot-scoped service register / call / event publish substitute
- robot_id 누락 fail-fast
- add_module missing dep fail-fast
- sync / async start/stop

### Step 4 — Persistence + Storage Protocol + Infra

`framework/persistence/protocol.py` + `framework/storage/protocol.py` + `infra/database/{sqlite,postgres}.py` + `infra/object_store/{filesystem,minio}.py`.

검증:
- SQLite session 생성 → ORM class INSERT/SELECT round-trip.
- FilesystemObjectStore put/get round-trip.

**✅ 완료** — Repository `Protocol[T]` (sync `get/save/delete`) + ObjectStore `Protocol` (runtime_checkable) + `open_sqlite() / open_postgres() -> (Engine, sessionmaker)` + FilesystemObjectStore (atomic `.tmp + os.replace`, path escape 차단) + MinioObjectStore (boto3, optional dep). **17 test PASS** (4 persistence + 13 storage).

### Step 5 — `Mirror[T]` primitive

`framework/contract/mirror.py`. 5 인자 모두 explicit (snapshot_service, snapshot_req factory, change_topic, value_cls, change_event_cls).

검증:
- Owner Module 이 snapshot service + event publish.
- Reader Module 이 `Mirror[T]` 선언 → 부팅 시 cache fill + event 받으면 cache update.
- Same-process (Zenoh same-session) + cross-process (Zenoh between sessions) 두 case PASS.

**✅ 완료** — Mirror descriptor + MirrorState (per-instance state via `__set_name__` + `__get__`) + NotReady + discover_mirrors + Runtime 통합 (`_register_mirror_subscriber` phase 2 + `_initialize_mirrors` phase 3a + `_refetch_mirror`). **10 test PASS** (same-process snapshot / Owner activate event refetch / Owner-not-up non-blocking / robot-scoped snapshot + event filter / cross-process subprocess + NotReady + per-instance state).

### Step 6 — 첫 Module 박아서 검증 (Calibration)

`modules/calibration/`. ORM + Repository + Module + Alembic.

검증:
- `CalibrationModule.activate(result_id)` round-trip (Zenoh same-session).
- 두 result row, activate 시 한쪽만 is_active=True 자연.
- `CalibrationActivated` event publish 확인.

### Step 7 — Reader 박아서 검증 (Motion)

> ⚠️ **2026-07-02 정정**: 실제 Motion Reader 는 Mirror 아니라 **boot-time `snapshot_bundle` query** (calibration = boot-time config, anchor #2). 아래 "Mirror + activate → 갱신" 검증은 stand-in. 실제 검증 = 부팅 시 1회 조회 → kinematics build (§4 banner + [calibration.md §9](calibration.md)). Mirror primitive 자체 (Step 5) 는 별도로 이미 테스트됨 — domain consumer 만 없음.

`modules/motion/`. boot-time `snapshot_bundle` query + kinematics + IK.

검증:
- 부팅 시 MotionModule 이 `snapshot_bundle` 1회 조회 → kinematics build.
- offline commit → 재시작 → 새 bundle 로 fresh build (런타임 갱신 없음).
- Same-process (PC 한 process) + cross-process (PC + 모터 Pi sim) 두 case PASS.

### Step 7.5 — Derived read model 검증 (CameraDriver + CameraDecoded)

`modules/camera/module.py` (CameraDriver) + `modules/camera/decoded.py` (CameraDecoded).

검증:
- CameraDriver mock impl 이 JPEG bytes publish (실 hardware 없이 합성 frame).
- CameraDecoded 가 `/camera/jpeg` subscribe + `cv2.imdecode` + `CameraFrame` publish.
- Consumer Module (테스트용 dummy) 가 `@subscriber(Camera.Stream.DECODED)` **stream 구독**으로 받음 (Mirror 아님 — derived read model = telemetry stream, §3.5 / §3.2).
- Consumer N=3 일 때 decode 가 1 회만 일어남 (각 consumer 별 decode X).
- decode dedup 의 CPU 절감 측정 (consumer 가 직접 decode 박는 case 와 비교).

### Step 8 — 2-3 entity 추가 (Scan / Reconstruction)

`modules/scan/` (append-only blob + metadata) + `modules/reconstruction/` (Reader of scan + ObjectStore put).

검증:
- ScanModule capture 시 ObjectStore blob put + metadata INSERT.
- ReconstructionModule build 시 scan blob get + mesh ObjectStore put.

### Step 9 — backend/ 의 도메인 logic 옮겨심음

각 Module 의 business logic (BA / IRLS / Ruckig / IK / TSDF / step DSL) 을 `modules/<name>/service.py` 또는 그 안 sub-module 로 옮겨심음. framework 부분은 새로 짠 framework 사용.

옮겨심을 자산 (framework_dogfood_plan §14.7):
- 캘 BA / IRLS / Huber / observability / strategy / ChArUco / capture_quality
- Motion command / TrajectoryRunner / Ruckig / Jog 적분 / IK
- Task DSL / Step / Slot / TaskRunner / Recipe / pick_and_place / scan task
- Detector / YOLO / Grounding DINO / search_and_detect
- Scene3D / depth_frame / consensus / pointcloud stream
- Reconstruction / ICP / PoseGraph / TSDF / mesh extract
- Kinematics (PyBullet + SagCorrected + link_offset patch)
- Coordinates (Joint / Link / Sag)
- Gamepad / 8BitDo mapper
- Robot Registry (robots.yaml + RobotConfig + factory)

### Step 10 — backend/ discard

backend 가 backend 의 모든 기능 가지면 backend/ 폐기. 새 코드 = backend/.

## 12. 알려진 risk

### 12.1 `Mirror[T]` 가 진짜 얇은지 검증

snapshot + subscribe + cache 패턴이 우리 use case 전체에 fit 한지는 entity 3-4 박아본 후 검증. 의심 자리:

- **partial update vs full refetch** — 큰 entity (예: scan_sessions 100 row) 의 한 row update 시 event 가 어떻게? `event = {row_id, delta}` 박고 cache merge? 또는 `event 받으면 snapshot 다시 fetch`? 첫 박을 때는 *full refetch* 가 단순. 부족하면 partial 추가.
- **concurrent write** — 두 process 가 동시에 같은 entity 변경하면? 우리 use case 에 진짜 있는지부터 (각 Module 이 owner = single writer 자연).
- **event ordering** — Reader 가 부팅 snapshot 한 후 event 받기 전 window 에 다른 process 가 변경 → 놓침. snapshot 시점에 subscribe 먼저 + buffer 패턴 박아야 (subscribe-before-snapshot).

### 12.2 N Module × N Alembic 운영 복잡도

Module 8-10 개 = Alembic 8-10 개. 부팅 시 각 Module 자기 schema ensure. risk:
- **부팅 시 lock contention** — 같은 NAS Postgres 면 8 Module 이 동시 `upgrade head` → Alembic version table lock 경쟁. 첫 부팅만 issue, 이후엔 noop. 부팅 순서 hint 또는 retry 박으면 OK.
- **Schema 충돌** — 각 Module 이 자기 table prefix (`calibration_*`, `scan_*`) 만 만들면 0. naming convention 준수.

### 12.3 한 사람 framework capacity

framework 짜는 자체 무거움. Protocol + Runtime + Contract + Transport + Mirror 5 layer. mitigation:
- **MVP 부터 시작** — Step 1-5 끝낼 때까지 Module 0 개. framework 검증.
- **`Mirror[T]` 가 가장 위험** — snapshot + subscribe + cache lifecycle 박는 자리. 첫 박을 때 simplest version (full refetch on event) 으로.
- **infra adapter 는 wrapping 만** — Zenoh / SQLAlchemy / Alembic / boto3 기능 자체는 활용, framework 가 wrap 만.
- **Transport 한 갈래** (Zenoh 만, §3.4) — LocalTransport 박지 않아서 resolver / behavior 일관성 risk / 두 path 유지 부담 0. capacity 절약.

### 12.4 backend/ 와 backend/ 병행 risk

framework_dogfood_plan §14.3 규칙 그대로:
- backend/ 의 framework 부분 (BaseNode / 노드 hierarchy) 추가 변경 X.
- backend 자체 *기능 개발 금지*, framework 검증만.
- 실 hardware 1 robot (omx_f_0) 만 붙여보기.
- backend/ 의 코드 reference OK (BA / Ruckig / IRLS / step DSL 등 자산), 재구성.

## 13. 인접 문서

- [backend.md](backend.md) — 결정 history + plan + §13 결정 history (20 항목) + §14 backend reframe + §15 Runtime-centric reframe. 본 문서는 §15 위 정리.
- [dev_reference.md](dev_reference.md) — 검토 phase protocol. 본 문서는 그 산출물의 한 단계.
- [storage_layer.md](storage_layer.md) — 기존 Storage Module 설계. 본 문서에서 폐기 결정. 단 ORM / Repository 자산 (SQLAlchemy 패턴 / Alembic 운영) 재활용.
- [motion.md](motion.md) — Move / Servo / Jog / Task 4 계층. modules/motion/ 안 그대로 옮겨심음.
- [task.md](task.md) — Step / Slot / Recipe DSL. modules/task/dsl/ 안 그대로.
- [multi_robot_architecture.md](multi_robot_architecture.md) — multi-robot platform 설계. 본 framework 위 robot dispatch 패턴 자연 흡수 (Module 안 `robot_id` 인자).
- [backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py) — Transport latency 측정 script. §3.4 (LocalTransport 박지 않음) + §3.5 (derived read model decode dedup) 결정의 evidence. spec 변경 시 재실행.

## 14. 핵심 결정 anchor

새 세션 진입 시 본 표를 진실 source 로. 결정에 의심 들면 spec 위치 다시 읽기.

| # | 결정 | spec | 핵심 근거 |
|---|---|---|---|
| 1 | Zenoh 단일 (LocalTransport X) | §3.4 + §10.8 | [bench_transport.py](../backend/scripts/bench_transport.py) 측정: 작은 message ~3us, 5MB ndarray ~4% × N CPU 절감 — framework 두 갈래 유지 비용 < 절감. decode dedup (§3.5) 으로 39% → 21% 흡수 |
| 2 | ~~boot-query 1회~~ **→ RE-SUPERSEDED (2026-07-07)**: CalibrationBundle = **Mirror[CalibrationBundle]** (Motion 이 첫 consumer). 옛 boot-query (2026-07-02) 는 분산 부팅 순서 종속성 + silent 무보정 운전을 만들어 폐기 — liveliness 기반 Mirror 로 owner 가 언제 뜨든 수렴. "변경은 재부팅" 은 유지 (없음→값만 live 적용, 값→값′ 은 stale 표시). "atomic Bundle 단위" 유지 | §3.3 배너 + anchor #23 |
| 3 | Exception propagation (envelope `{success, message, data}` X) | §3.1 | Python 자연 = exception. caller 매 호출 `if not res.success` boilerplate 회피. transport 가 `RemoteError(type, message)` raise |
| 4 | Database-per-Module (Storage Module 폐기) — **테이블/ORM/Repository 소유만 모듈별**. **마이그레이션은 루트 단일 Alembic** (2026-07-02 정정, §9.2): 같은 프로세스+공유 DB = Database-per-Service 아님 | §2.4 + §9 | centralization motivation 은 Mirror 흡수 (지금 boot-query). DB dep 격리 = Pi 가 DB 모듈/alembic 안 가짐 |
| 5 | Module = plain class (`@module` 데코 / base class 강제 X) | §3 + §7.1 | backend/ BaseNode 부풀음 (15+ method 누적) 경로 차단. framework 는 `@service` / `@subscriber` / `Mirror` 박힌 attribute 만 inspect |
| 6 | ModuleRuntime Protocol + constructor 주입 (base class / setattr / ctx X) | §3.7 + §4 | base class 부풀음 + ctx 의 한 문장 정의 fail. composition 한 Protocol 이 sweet spot |
| 7 | Wire key = explicit + typed at every use site | §3.0 / §3.1 / §3.2 / §3.3 / §3.7 | 세 원칙: ① 사람이 explicit 지정 모든 use site ② raw string X (typed StrEnum) ③ **service 가리키는 방법 = 항상 `Service.X` enum 하나** (method reference X) |
| 8 | `runtime.call(key, req, res_cls, *, robot_id, timeout) -> TRes` (method ref X) | §3.7 + §4 | 원칙 ③ 정합. publish/subscribe 와 동일 패턴. res_cls 명시로 return type narrow |
| 9 | `Mirror(snapshot_service, snapshot_req, change_topic, value_cls, change_event_cls)` (method ref X) — **활성 (2026-07-07, 첫 consumer = Motion)** + `@mirror.on_change` 반응 훅 | §3.3 + §4.2 | 5 인자 모두 key + Pydantic class. `snapshot_req` = factory `Callable[[self], Req]`. cross-module method import 사라짐. on_change 는 값이 실제로 바뀐 전이만 (동일값 refetch 무발화) |
| 10 | Wire encoding = Pydantic + msgpack layered (DIP) | §3.4 | Module 은 Pydantic schema 만 알고, Transport boundary 가 msgpack 처리. `bytes` field native pass-through (JPEG base64 33% overhead 회피) |
| 11 | Topic prefix = `srv/` / `event/` / `stream/` (`horibot/` X) | §3.0 | 세 종류 첫 chunk 분리 — srv=RPC / event=state notification / stream=고빈도 raw. broker 단일 project 라 namespace prefix 불필요 |
| 12 | `stream` ≠ `event` (nested class naming) | §3.5 / §7.4 | `Module.Stream` 으로 명명 (`Camera.Stream`). `Module.Event` 는 진짜 state notification 만 |
| 13 | Wildcard subscribe = transport detail, framework 어휘 X | §3.7 | `@subscriber("*")` 같은 implicit pattern 금지. robot-scoped event 는 framework 가 `{robot_id}` → transport wildcard substitute, 사용자 코드에 등장 X |
| 14 | robot scope = yaml primary (constructor 가 계약 검증) | §2.7 | yaml `robots: [...]` 박힘 = robot-scoped Module. constructor 에 robot_id parameter 있어야 |
| 15 | Derived read model 패턴 (decode dedup) | §3.5 | 큰 payload (JPEG/depth) 의 decode 비용 = N consumer × decode. framework primitive 가 아닌 Module 패턴 — CameraDecoded 가 1회 decode 후 fanout |
| 16 | Runtime 부팅 순서 = instantiate → register → start | §3.6 | Mirror snapshot / Module A start 이 다른 Module service 호출 — Phase 2 (register) 가 Phase 3 (start) 이전 완료 보장 |
| 17 | Mirror invariant — partial state 노출 X | §3.3.2 | event callback thread vs handler thread race 차단. 구현 자유 (lock / atomic / RCU) |
| 18 | Mirror 동기화 = invalidate+refetch only (push update X) | §3.3.5 | event = 변경 알림 / snapshot = 진실. Bundle atomic 보존 + 다른 Module event 가 trigger 인 use case 자연 표현. push update 필요하면 Mirror 안 박고 `@subscriber` 직접 |
| 19 | **robot_id 는 두 개** — 키의 `{robot_id}` = 주소 (transport, framework/Bridge 키 확장) / body 의 robot_id = **req 필드** (service API, 호출자가 넣고 타입 강제). **Bridge 자동주입 / 생성 scope 메타데이터 / stub 전부 폐기** — global 은 "req 에 필드 없음" 으로 구조적 해결 | §2.7.1 / §2.7.3 | 2026-07-03. agnostic vs global 을 런타임에 구분하려는 순간 메타데이터나 "지금 없으니까" 타협이 필요해짐 — 레이어 재구성으로 문제 자체 소거 |
| 20 | **req robot_id 파생 규칙** — 다른 식별자 (run/session/result/waypoint row id) 로 robot 특정 가능하면 req 에 robot_id 안 둠 (DB row 에서 파생) | §2.7.1 | "run A 에 robot B 캡처" 류 불일치 채널 원천 차단 |
| 21 | **`robot_scoped` 판정 = service 키만** — stream/event 는 payload 라우팅/wildcard 라 host-level 도 robot-scoped 키를 다룸 | §2.7.1 | framework 가 `self.robot_id` 를 요구하는 유일한 자리 = service 키 확장 (`app.py::_register_service`) |
| 22 | robot-agnostic 모듈의 per-robot config = resolve 의 lean 투영 주입 (필요만, 모듈이 robots.yaml 재보유 X). acceptance = 새 로봇 추가 시 모듈 코드 0 수정 + so101+omx 눈속임 방지 테스트 | §2.7.2 / §2.7.3 | 기능("한 robot 되니 끝") 아니라 아키텍처가 코드에 드러나야 |
| 23 | **liveliness = transport 기본 제공** (zenoh liveliness token) — Runtime 이 service 등록 시 같은 key 로 token **자동 선언**, **Mirror 가 이를 구독해 자동 refetch 수렴** (현재 유일 소비자). "모듈이 나 떴어요 publish" 손 컨벤션 금지 | infra/transport/zenoh.py + framework/runtime/app.py | 2026-07-07. 부팅 순서 = distribution 문제 = framework 책임 ("같은 코드가 어디 배치되든 그대로 동작"). 커스텀 ready 이벤트와 달리 **연속적 참** (세션 종료 시 자동 gone = 크래시/재시작 감지). probe 검증 4전이 = test_transport.py::test_liveliness_presence_lifecycle. (owner-대기 전용 `runtime.wait_for` 는 만들었다가 소비자 0 → 제거, 필요 시 subscribe_liveliness 위 5줄로 부활) |

### 작업 원칙

- 본 문서 = framework spec SSOT. 박힌 결정 (위 18개) 의심하지 말고 따를 것.
- 기존 backend/ 코드 = 도메인 logic reference 만 (BA / IRLS / Ruckig / ChArUco / Step DSL / Open3D ICP / TSDF / YOLO 등 알고리즘 자산). framework 부분 (BaseNode / `dict[robot_id]` dispatch / Cache singleton 등) 매몰 X.
- Step 1 부터 순차 implementation. 점프 X.
- 박지 말 패턴: 추가 옵션 카탈로그 던지기 / cost-based reflex ("한 줄 fix") / cargo cult (외부 framework 명명 흉내) / flipflop (사용자 push 자동 반대편 점프) / measurement 없는 추정.
- test 짤 때 production code 에 dogfood 넣지 X.

## 15. 구현 진행 → [backend.md](backend.md)

진행 status / 검증 수치 / 다음 작업은 전부 status 문서로 이동 (본 문서 = spec 만,
진행 표기 안 둠). 아래 test 원칙만 spec 으로 유지:

- **단순 통과 X** — test 통과 ≠ 설계 검증. 모든 test 가 "spec 의 어느 invariant 검증" 명시 박혀야 함
- 새 test 박을 때 = docstring 에 spec ref + invariant 명시 (예: `spec §3.3.2 — Mirror partial state 노출 X`)
- 구현 중 spec 충돌 / 새 invariant 발견 시 §14 anchor 표 update 박은 후 진행

## 16. Module catalog (옛 backend_modules.md 통합, 2026-07-03 현행화)

### 16.1 4 layer + Module catalog

46 책임을 한 발 떨어져 보면 **4 layer 의 자연 분리** — 강제 layer architecture 아닌,
책임의 본질이 다른 묶음. framework 는 layer 모름 (duck typing).

```
┌──────────────────────────────────────────────────────────────┐
│ Layer 4 — Boundary    : Bridge, (Gamepad — 미래)             │
│ Layer 3 — Orchestration: (Task — 미래, §17)                  │
│ Layer 2 — Domain      : Motion, Calibration, Detector,       │
│                         Scene3D, Scan, Waypoint              │
│ Layer 1 — Hardware    : MotorDriver, CameraDriver,           │
│           + Derived     CameraDecoded                        │
└──────────────────────────────────────────────────────────────┘
```

| # | Module | Layer | Scope (§2.7) | Host | 한 줄 책임 | 영속성 |
|---|---|---|---|---|---|---|
| 1 | **MotorDriver** | Hardware | robot-scoped | pi_motor | Feetech/Dynamixel raw 통신 (state 20Hz + command + torque) | X |
| 2 | **CameraDriver** | Hardware | robot-scoped | pi_camera | RealSense capture + JPEG + depth zstd | X |
| 3 | **CameraDecoded** | Derived | robot-scoped | pc | JPEG→BGR + zstd→uint16 (decode dedup, 두 stream) | X |
| 4 | **Motion** | Domain | robot-scoped | pi_motor | kinematics(PyBullet) + Move/Jog primitive + TcpState | X |
| 5 | **Calibration** | Domain | robot-agnostic | pc | 5종 산출물 Bundle owner + capture/preview | DB + ObjectStore |
| 6 | **Detector** | Domain | robot-agnostic | pc | `Detect Object` (GDINO adapter 뒤) → base 3D | X |
| 7 | **Scene3D** | Domain | robot-agnostic | pc | RGBD primitive (라이브 cloud + consensus snapshot) | X |
| 8 | **Scan** | Domain | robot-agnostic | pc | scan 세션/캡처 + **TSDF build** (옛 Reconstruction 흡수) | DB + ObjectStore |
| 9 | **Waypoint** | Domain | robot-agnostic | pc | Robot Asset Layer — 티칭 joint 자세(rad) + group | DB |
| 10 | **Bridge** | Boundary | robot-agnostic | pc | WS relay + MJPEG + HTTP (`/robots` `/system` `/contract*`) + `/robot` static | X |
| — | Task / Gamepad | 미래 | robot-agnostic | pc | §17 task-first / 8BitDo jog dispatch | (DB) |

**합치지 / 더 잘라지 않은 근거** (한 Module = 한 정직한 책임 묶음):
- MotorDriver+Motion 분리 — vendor SDK swap 시 kinematics/Ruckig 변경 0.
- CameraDriver+CameraDecoded 분리 — host 횡단 (pyrealsense2 USB vs decode CPU).
- Scene3D+Scan 분리 — primitive vs workflow+persistence (trigger/cost profile 다름).
  단 옛 설계의 Scan/Reconstruction 분리는 v2 에서 **Scan 한 모듈** (build = `@service`
  + to_thread — 별도 모듈이 줄 격리 이득이 없었음).
- TcpState/JointRad 별도 Module X — small payload, Motion 안 fk+publish SSOT.
- LLM/검출 모델 별도 Module X — Detector 안 adapter (§17 "인터페이스 ≠ 구현").

### 16.2 Host 배치 + deployment yaml

| 머신 | Module | 이유 |
|---|---|---|
| **pi_motor** (192.168.0.101) | MotorDriver, Motion | 100Hz 명령 network 안 넘는 강제. IK 도 RTT 0 |
| **pi_camera** (192.168.0.102) | CameraDriver | pyrealsense2 USB 강제 |
| **pc** | 나머지 전부 | decode CPU + 무거운 연산 (Open3D/GDINO) + DB owner + browser |

| yaml | 의미 | driver |
|---|---|---|
| `pc.yaml` / `pi_motor.yaml` / `pi_camera.yaml` | 운영 분산 | real |
| `mock.yaml` | hardware 없이 전 Module 한 process (UX/wire 검증 + contract gen) | mock |

mock 은 별도 Module 아님 — `modules/<domain>/drivers/mock.py` driver subdir swap.
`dev.yaml`(단일 머신 풀스택 real) 안 둠 — 옛 backend 실사용 결과 불필요.

### 16.3 Cross-module 의존 + 데이터 성격 → primitive rule

| Reader | 패턴 | Source |
|---|---|---|
| Motion | **boot-time query** (`snapshot_bundle` 1회 — §9.3) | Calibration |
| Calibration | subscribe 캐시 (decoded frame + motor raw) | CameraDecoded / MotorDriver |
| Detector | call (매 detect) | Calibration / CameraDecoded / Motion |
| Scene3D | subscribe 캐시 (depth/color) + intrinsic query | CameraDecoded / Calibration |
| Scan | call (capture 시 scene3d SNAPSHOT / build 시 calibration bundle) | Scene3D / Calibration |
| Waypoint | subscribe 캐시 (TcpState) | Motion |
| Bridge | subscribe + relay | 모두 |

**framework 차원 decision rule** — 새 module 설계 시 "이 데이터는 어느 칸인가" 만
판단하면 primitive 가 결정된다:

| 데이터 성격 | 예시 | primitive |
|---|---|---|
| **Runtime telemetry** (지속 변화) | TCP pose 20Hz, joint/motor state, frame | **Stream** 또는 **snapshot service**. Mirror ❌ |
| **Boot-time configuration** (부팅 시 확정) | Capabilities | **Query** (boot 1회). 변경은 다음 부팅부터 |
| **Slowly-changing shared state** (가용 시점 불명 / 갱신 알림 필요) | CalibrationBundle (Motion 소비) | **Mirror** (liveliness 수렴 + on_change 반응, §3.3). 2026-07-07 활성 |

stream vs snapshot service 분리 예 — `Motion.Stream.TCP_STATE` (20Hz continuous,
frontend 시각화) vs `Motion.Service.TCP_SNAPSHOT` (Detector/Scan 의 point-in-time 1회).

### 16.4 Module SDK — bounded context

각 Module 이 자기 도메인의 SDK. **driver 공통 abstraction = Module SDK 안**
(`modules/<domain>/drivers/protocol.py`), framework X — 안 그러면 Gripper/Lidar/PLC
추가마다 framework 가 부풀음.

- 3 계층: ① framework (도메인 모름) ② Module SDK (contract + driver Protocol + impl)
  ③ consumer Module (framework 어휘만, driver Protocol 도 모름).
- `drivers/` 박을 자리 = hardware adapter swap 책임 (motor / camera / 미래 gripper).
  logic 자체가 책임인 Module (motion / calibration / detector / ...) 은 drivers/ 없음
  (detector 는 검출 모델 adapter 를 `backend.py` 로 — 같은 원리).
- 새 vendor / 새 도메인 추가 시 framework 변경 0, 다른 Module 변경 0.

### 16.5 Topology / Capability / Config 어휘 (driver self-declare)

> **Topology = "무엇이 존재하는가"** (구조 — consumer 가 구조 자체를 소비할 때만: Motor ✅
> `motors[id,kind]` / Camera ❌). **Capability = "무엇을 할 수 있는가"** (flags +
> supported max metadata). **Config = "현재 설정 값"**. 셋 다 부팅 1회 확정 → snapshot
> service, Mirror X.

- 값의 SSOT = **driver self-declare** (`driver.topology()` / `driver.capabilities()` —
  yaml 에 박으면 duplication/불일치). module 은 boot 1회 read + cache + service relay.
- Motor 의 GRIPPER / POSITION_PID capability 박지 X — Topology derived / baseline.
- **Intrinsic SSOT 분리**: CameraDriver 의 `get_factory_intrinsic` = Calibration seed
  전용 (internal). 모든 consumer 는 **Calibration Bundle 의 intrinsic** 만 봄 — Camera
  public contract 에 `GET_INTRINSICS` 박지 X (한 어휘 두 의미 ambiguity 차단).
- UI 는 capability flag 만 봄 (D405/UR/Basler 모름) — 새 hardware 추가 시 UI 변경 0.

### 16.6 Public contract surface — 두 소비자 + Bridge invariant

`contract.py` = 두 소비자의 SSOT (둘 다 **runtime-served**):
① **frontend TS gen** — bridge `GET /contract.json` EXPORT → `pnpm gen:types` 가
contract.ts 조립 ([frontend.md §2.1](frontend.md)). 노출 =
`apps/contract_export.py::FRONTEND_EXPOSED` opt-in allowlist 한 곳.
② **developer contract graph viewer** — bridge `GET /contract/graph` (unfiltered
declared universe) → frontend `/contract` React Flow ([contract_graph_viewer.md](contract_graph_viewer.md)).

- contract.py 만 generator 의 read 대상 — module.py / drivers/ / orm / repository 는
  internal (§3.0 "다른 module 이 import 박는가" 기준과 동일).
- **Stream payload invariant** — 모든 stream payload 에 `robot_id` + `seq: int` +
  `timestamp_unix: float` (frontend reconnect / lag / out-of-order 검출 기본 어휘).
- **Bridge = runtime relay only** — domain Module logic 박지 X. framework helper
  (`/robots` robot list / `/system` metric / `/contract*` export) 의 read-only relay 는
  OK. domain 데이터는 반드시 해당 Module 의 service 로 (Bridge 가 DB direct read 등
  우회 금지). heartbeat / logging 등 framework infra 는 Runtime 이 자동 흡수.
- `@service(description=, tags=)` metadata 확장은 viewer v2 자리 (현재 key 만).

### 16.7 후속 자리 (미래 조건 명시)

- **multi-camera per robot** — robots.yaml `camera:` 단수 강제 유지. wrist+workspace
  다중 필요 시 `(robot_id, camera_id)` device-scope 확장 (framework anchor 변경).
- **pyproject role-split** — 현재 단일 deps (bring-up 편의). 실 Pi 배포 시 PEP 735
  group (pi-motor / pi-camera / pc) 분리 — pyrealsense2 소스빌드/open3d 무게가
  load-bearing 해지는 시점.
- **Effective capability** (hardware ∧ runtime condition) / high-level composition
  (pick_and_place = depth ∧ cartesian ∧ gripper) — 실요구 시.

## 17. Task-first 운영 원칙 + Task/PnP 설계 (옛 task_dsl_waypoint_port.md 통합)

### 17.1 운영 원칙 (2026-07-03 잠금 — 프로젝트 전역 지배)

**핵심 관찰** — 실제 task(pick-and-place / 병따기 / 수건 접기)의 어려움은 **조합·순서가
아니라 primitive 안**에 있다 (`GraspCap()` 하나가 R&D 전체, 순서는 짧은 고정 스크립트).
n8n 식 워크플로 플랫폼을 지금 만들면 팔레트가 빈 조합기.

1. **산출물 = "실제 task 를 해내는 로봇"** (자란 DSL/플랫폼 아님). 인프라 중력의 균형추.
2. **승격 3-층 분류**:

   | 층 | 예 | 규칙 |
   |---|---|---|
   | **Day-1 primitive** | MoveJ/MoveL/Stop/Gripper/TCP pose/IK·FK/Detect Object | **처음부터 구축** (표준 산업 로봇 공통 제공) |
   | **Domain primitive** | GraspBottle/FoldTowel | **절대 미리 안 만듦** — task 로컬, 필요 실증 시 공용 승격 |
   | **Orchestration** | Loop/Retry/Parallel/비주얼 에디터 | 실 task 가 요구할 때 (선축조 금지) |

   판별 기준 2개 (둘 다 ✅ = 지금, 하나라도 ✗ = 대기): ① industry 가 표준 primitive 로
   출하하나? ② 하드웨어·대상·알고리즘 무관한 의미인가?
3. **인터페이스 ≠ 구현** — Day-1 은 *능력/의미* 만 계약 노출, 구현체는 adapter 뒤
   (예: `Detect Object` = Day-1 계약, Grounding DINO = 구현체 — YOLO/FoundationPose 교체
   가능). DSL/Runtime 은 "Detect Object" 만 앎.
4. **dev 안전장치는 분류와 직교, 지금 짓는다** — async runner + 디버거 (step/pause/
   breakpoint). hardware burn 직접 절감.

**Phase 순서 (task-first)** — "DSL 먼저 다 짓기" 폐기:
① 첫 task 선정 (= **단팔 pick-and-place**, 2026-07-03 확정) → ② 필요 primitive 정의 →
③ task #1 을 거의 DSL 없이 평범한 async 함수(`await runtime.call`) + 디버거만 얹어 구현
→ ④ task #2 에서 반복 보이면 Step/Slot 추출 → ⑤ 실요구 시 DSL 보강 (각 확장 rule of
three 게이트). 비주얼 에디터는 "문만 열어둠" — task 정본을 직렬화 가능한 스펙 (typed
Slot/Step) 으로 유지, primitive 가 쌓여 조합이 변수가 되는 시점에 얹음.

### 17.2 결정 로그 (D1–D11, 2026-07-03 잠금)

| # | 항목 | 결정 |
|---|---|---|
| D1 | 계층 | Motion → **Robot Asset Layer** (Waypoint = 첫 자산) → Consumer (PnP/Scan) |
| D2 | 실행 모델 | **async runner** — step `async def execute(ctx)` + `await runtime.call` (sync 유지 시 run_coroutine_threadsafe 브리지 부활) |
| D3 | 자산 이름 | Waypoint / WaypointGroup / WaypointGroupMember (UR 어휘 — "Pose" 는 TCP 연상) |
| D4 | 저장 | **joint-only, rad** (Motion.TcpState.joints 소비 — raw encoder 모름, 계층 준수) |
| D5 | Group | 3-테이블 + `order` 컬럼 — reorder/add/remove 가 행 단위, 드래그 UI 와 1:1 |
| D6 | 소유권 | Waypoint.robot_id = **instance id** (설치 위치/캘이 instance 별) |
| D7 | 티칭 | jog → 현재 joint 저장 (IK 재계산 X) |
| D8 | DSL 표면 | `MoveJ(waypoint=<ref>)` — resolve 는 runtime, 식별 방식과 DSL 분리 |
| D9 | Calibration pose | 별도 개념 유지 (알고리즘 생성 — 티칭 자산과 출처/생명주기 다름) |
| D10 | detection | **Top-K + 기하 prior 우선**, multi-view 3D 합의는 후속 (§17.5) |
| D11 | 자동 scan | defer (Waypoint Group 순회로 나중 흡수, UI/UX 사용자 결정 후) |

### 17.3 Motion 완료 계약 (잠금 — 흔들리지 말 것)

- **외부 계약**: `await motion.move_j()` / `move_l()` 는 **trajectory 정상 종료(DONE)
  거나 오류로 끝났을 때 반환**. DONE → return / IK 실패·충돌·Ruckig 오류 (FAILED) →
  **exception** (`MotionFailed`, v2 exception propagation) / STOP → cancellation.
  Task 규칙 하나: **await 성공 = 완료, 다음 step.** (구현: traj thread terminal 상태를
  `asyncio.Future` 로 resolve — 내부 방식은 driver 별 자유, "인터페이스 ≠ 구현".)
- **MoveL v1 제약**: ✓ XYZ 직선 / ✗ orientation 보장 안 함 (position-only IK).
  orientation interpolation (SLERP) = MoveL v2 — 실 task 요구 시.

### 17.4 Task DSL 재이주 매핑 (구현 대기 — 옛 backend 자산의 v2 적응)

옛 `backend/` 의 성숙한 Task DSL (typed Slot lego + 디버거 UI) 을 v2 primitive 로 매핑
(기계적 복사 X):

| 옛 | → v2 |
|---|---|
| TaskNode (ApplicationNode) | TaskModule plain class, PC |
| `ctx.call_motion` (sync + traj Event) | `await self.runtime.call(...)` |
| TaskRunner (threading + Event) | async runner (`asyncio.Event` 게이트, `await pause_event.wait()`) |
| Slot / StepResult / Step / StepContext / task_tree | 거의 그대로 (ctx 가 node 대신 runtime 보유) |
| run/preview/stop/resume/step/run_to/toggle_breakpoint | `@service async def` |
| TASK_STATE / TREE / STEP_RESULT | `@publishes` streams |

디버거 게이트 (`_should_pause_before` → pause → 실행 → publish) 동작 보존, ForEach/Try
의 `ctx.run_child` 재진입도 동일. 검증 = 도메인 step 0개 trivial task (Wait + no-op) 로
runner+디버거 e2e 부터. frontend = TaskProgressPanel / PromptPanel / TaskResultLayer 포팅.
공유 값 타입 (Position3/Pose6/Detection) 위치는 빌드 시 결정 (task+detector 공유).

### 17.5 PnP consumer 설계 (구현 대기)

- step 매핑: `MoveJ(waypoint=<ref>)` (D8) / `MoveTCP`→MOVE_L / Gripper+VerifyGrasp /
  GraspPolicy·PlacePolicy (순수 계산 그대로) / GroundedDetect → Detector.DETECT Top-K.
- **detection 개선** (옛 first-match-wins 오검출 — 흰 안경닦이를 흰 큐브로 오인):
  ① **Top-K** (진짜 물체가 2등이면 top-1 은 영원히 누락) ② **기하 prior** (depth 의
  height/base_z 로 예상 범위 밖 reject — confidence 무관 구분) ③ multi-view 3D 합의는
  후속 (후보 누적 구조만 먼저, 스코어링은 실 데이터 보며).
- recipe 재설계: BreakIf 제거 → **Waypoint Group 순회하며 후보 누적** →
  `SelectTarget(candidates, prompt, priors)` 스코어 → 최종 Detection.
- 검증: 구조/계약/mock e2e = 회사, **detection 정확도 = 집 하드웨어만**.


---
---

# 부록 — 통합 원문 (2026-07-11 문서 다이어트)

> 아래 문서들을 본 문서 부록으로 병합 (원문 그대로):
> - `backend.md`
> - `backend.md`
> - `backend.md`


---
---

<!-- ═══════════ [통합 원문] backend.md ═══════════ -->

# Framework — Async-Uniform Call Contract (설계, 2026-07-03)

> ⚠️ **부분 stale (2026-07-11 문서 전수 감사)**: intro 의 "구현 전 단계" 는 stale — **구현 완료** (scan/detector module.py 가 본 문서를 근거로 인용). 설계 내용 자체는 현행.
> 본 감사에서 삭제된 v1 문서 참조가 남아있을 수 있음 — git history 에서 복원 가능.

> backend framework 의 **모듈 간 호출 API 통일** 설계. **구현 전 단계** — 방향 결정
> 완료, 구현은 다음 세션. 본 문서로 논의 이어가기. ([backend.md](backend.md) 의
> 4 primitive 계약 위에 얹는 실행모델 정정.)

## 1. 문제 — "sync냐 async냐"를 모듈 개발자가 의식하게 만든다

모듈 개발자가 다른 모듈의 service 를 호출할 때, **지금 자기가 어느 함수 안에 있느냐**
에 따라 호출 방법이 달라진다:

| 지금 있는 곳 | 호출 방법 |
|---|---|
| `async def start()` / 내가 띄운 async task | `await self.runtime.call(...)` |
| `@service` / `@subscriber` 핸들러 (sync `def`) | `run_coroutine_threadsafe(...).result()` 브리지 |

실제 사례 — [scan/module.py](../backend/modules/scan/module.py) 의 `_call` 헬퍼:
sync `capture()` 핸들러가 scene3d SNAPSHOT 을 부르려고 이벤트 루프 저장 +
`run_coroutine_threadsafe` + `Future.result()` 를 **모듈 코드에** 들고 있다. 이건 scan
도메인이 아니라 **asyncio 실행모델 처리** — framework 가 감춰야 할 것이 모듈로 새어
올라온 것.

**판정 기준**: 모듈 개발자가 "이거 await 해야 하나? call_sync 였나?" 를 고민하며 다른
코드를 뒤지게 만들면, 그 지점에서 framework 는 UX 실패다. 모듈 간 호출은 **문맥과
무관하게 단 하나의 방법**이어야 한다.

## 2. 현재 실행 모델 (실측)

- **transport.call** ([infra/transport/zenoh.py](../backend/infra/transport/zenoh.py)):
  `async def call` → `await asyncio.to_thread(self._call_sync, ...)`. **이미 Zenoh 의
  sync `session.get()` 을 thread 로 감싸 async 로 노출**한다. (transport layer 는 이미
  옳게 흡수 중.)
- **service 핸들러 등록** ([app.py `_register_service`](../backend/framework/runtime/app.py)):
  `handler_bytes(req_bytes)` 가 `bound_method(req)` 를 **동기** 호출, BaseModel 즉시 반환
  기대. Zenoh queryable 콜백(`_on_query`)은 **Zenoh 워커 스레드**에서 sync 로 불린다 →
  핸들러도 sync 전용.
- **subscriber** (`_register_subscriber`): `bound_method(event)` 동기. sync 전용.
- **Mirror** (`_register_mirror_subscriber`): change_topic 콜백(zenoh 스레드)에서
  `asyncio.run_coroutine_threadsafe(self._refetch_mirror(...), loop)`. → **framework 가
  이미 sync콜백→loop 브리지를 내부에 갖고 있다** (§4 구현의 선례).
- **publish** (`_TransportRuntime.publish`): sync. fire-and-forget (응답 안 기다림).
- **start/stop**: Runtime 이 `await` (sync/async/없음 다 허용, [app.py:143-166]).
  현재 start/stop 을 가진 모든 모듈이 async (CameraDecoded 만 없음 — 띄울 게 없어서).

핵심: **Zenoh 는 sync API 만** 준다. transport.call 은 이미 to_thread 로 흡수했고,
**핸들러/subscriber 콜백 경로만 아직 sync 로 노출**돼 있어 그 위 모듈이 브리지를 떠안는다.

## 3. 결정 — 방향 1 (전부 async 중심, framework 가 Zenoh 흡수)

두 후보:

- **방향 1 — 전부 async 통일**: 핸들러도 `async def`, 어디서나 `await runtime.call(...)`.
  무거운 CPU 는 `await asyncio.to_thread(...)`. framework 가 Zenoh sync 콜백을 loop 로
  bridge (Mirror 와 동일 패턴).
- **방향 2 — async 를 완전히 숨김**: 모듈이 보는 `runtime.call(...)` 은 항상 블로킹처럼
  (await 없이). asyncio 는 내부 구현.

**채택 = 방향 1.** 근거:

1. **cost 가시성** — 네트워크 RPC 는 시간이 걸린다. `await runtime.call(...)` 이 코드에
   보이면 읽는 사람이 "여기서 제어가 넘어갈 수 있다"를 즉시 안다. 방향 2 는 그 비용을
   함수 호출 뒤로 숨겨 오해를 부른다.
2. **생태계 정합** — FastAPI / aiohttp / SQLAlchemy async 전부 `await`. Python 개발자의
   기본 멘탈모델.
3. **이미 async 시스템** — Zenoh pub/sub + streaming + 백그라운드 task + RPC 구조. 일부만
   sync 처럼 숨기면 오히려 "왜 이것만 await 가 없지?" 가 된다.

**단, 방향 1 의 전제 = framework 가 async 핸들러를 제대로 지원해야 한다.** 그래야 모듈
개발자는 `snapshot = await runtime.call(...)` 하나만 알면 된다.

## 4. 목표 계약 (developer-facing)

모듈 개발자가 배워야 할 규칙은 **딱 하나**: **다른 모듈을 부르면 `await runtime.call(...)`.**

- **`call` API 는 하나** — `await self.runtime.call(key, req, res_cls, ...)`. (`ModuleRuntime`
  protocol 은 애초에 `call` 단일 — public `call_sync` 는 존재한 적 없음, §8-2 확인.)
  "두 개 중 뭐 쓰지" 선택 자체가 없다.
- **핸들러는 async 지원** — `@service async def capture(...)` / `@subscriber async def
  on_x(...)` 를 framework 가 자연스럽게 지원. (sync 핸들러도 backward-compat 로 계속
  허용 — §7 마이그레이션. 단 다른 서비스를 호출하려면 async 여야 함 = 자연스러운 강제.)
- **publish 는 sync 그대로** — `self.runtime.publish(...)`. 응답을 안 기다리니 문맥 문제가
  없다. 통일 대상은 **응답을 기다리는 `call` 뿐.** (build progress / state 발행 등 전부
  sync 유지.)
- **start/stop async 그대로.**

즉 통일의 정확한 범위 = **"응답을 기다리는 cross-module 호출은 무조건 `await
runtime.call`"** 하나. publish·start 는 이미 문제가 없다.

## 5. framework 가 흡수하는 것 (Zenoh sync → async)

"모듈이 Zenoh 를 잊어버린다" 를 framework 내부에서 실현:

1. **transport.call** — 이미 `to_thread(_call_sync)`. 유지.
2. **service 핸들러 (신규 async 지원)** — Zenoh queryable 콜백은 여전히 sync (zenoh 스레드).
   그 콜백 안에서 핸들러가 coroutine 이면
   `asyncio.run_coroutine_threadsafe(handler(req), loop).result(timeout)` 로 loop 에서
   실행 후 결과 회수 (Mirror 선례와 동일). **브리지가 모듈에서 framework 로 이동** — 개발자
   눈엔 안 보임. sync 핸들러면 기존대로 직접 호출 (`iscoroutine` 분기).
3. **subscriber (신규 async 지원)** — 콜백에서 coroutine 이면 loop 에 schedule
   (fire-and-forget, 결과 대기 X — subscriber 는 반환값 없음).
4. **예외 전파** — async 핸들러의 예외도 기존 `reply_err` 경로(RemoteError)로 그대로.

## 6. 핵심 설계 과제 — 무거운 CPU 가 이벤트 루프를 막지 않게

방향 1 의 유일한 실질 리스크. 지금 sync 핸들러는 **Zenoh 워커 스레드**에서 돌아 30초짜리
TSDF `build` 가 loop 를 안 막는다 (그게 sync 핸들러의 뜻밖의 이점이었음). async 핸들러로
바꾸면 loop 위에서 돌 위험이 생긴다.

해결 = **관례 명문화**: async 핸들러 안의 CPU 무거운 일은 `await asyncio.to_thread(...)`.
framework 가 `run_coroutine_threadsafe(handler, loop)` 로 loop 에 태워도, 핸들러가
`await to_thread(build)` 하면 그 동안 loop 는 자유 (다른 service/stream 정상). zenoh 워커
스레드 하나가 `.result()` 로 블로킹되지만 pool>1 이라 무방 (현재와 동일).

**미결 — framework 가 이걸 강제/지원할 방법:**
- (a) 순수 관례 (문서로만: "무거우면 to_thread")
- (b) `@service(offload=True)` 같은 선언 → framework 가 자동 to_thread
- (c) heavy 전용 실행 정책(worker pool) 을 framework 가 제공
→ §8 논의.

## 7. 마이그레이션 영향 (모듈별)

| 모듈 | 변경 |
|---|---|
| **scan** | `_call` / `self._loop` / asyncio import **삭제**. `capture`/`build` → `async def` + `await self.runtime.call(...)`. `build` 의 Open3D 부분 → `await asyncio.to_thread(build_mesh)` (§6). |
| **scene3d** | start/live_loop 이미 async. 변경 거의 없음. |
| **calibration / motor / motion / camera** | 핸들러가 sync 지만 cross-service `call` 을 안 함 → **당장 안 바꿔도 동작** (sync 핸들러 backward-compat). 통일하려면 점진적으로 async 로. |

→ 결정 필요: **일괄 async 전환 vs 점진**(call 하는 핸들러만 우선). sync 핸들러 허용을
영구로 둘지, deprecate 할지.

## 8. 항목 분류 (2026-07-03 재구성 — 성격별)

옛 §8 은 6항목을 평평한 "미해결" 로 나열했으나, 실제로는 성격이 셋으로 갈린다.
평면 나열이 "다 정해야 구현 시작" 오해를 부른 것 — 실제로는 ①만 전제, 나머지는
구현을 막지 않는다.

### ① 확정된 전제 (더 이상 미해결 아님)

- Zenoh 는 **sync callback** 을 (별도 워커 스레드에서) 호출한다. zenoh-python 이 async
  콜백 API 를 주지 않으므로, **framework 가 `run_coroutine_threadsafe` bridge 를 내부에서
  담당**한다 (§5-2, Mirror 선례와 동일).
- **서비스 구현자는 이 사실을 몰라도 된다** — 이게 설계의 산출물. "전부 async" 는 목표가
  아니라 결과.
- 사용 중인 zenoh-python 버전 소스를 한 번 확인해 둘 수는 있으나 **설계를 막는 관문은
  아니다** (전제로 확정).

### ② 구현하면서 확인할 항목 (정책 아니라 검증)

- **Zenoh worker pool 크기 + long-handler 동작 실측.** 새 구조에서 heavy call 하나는
  스레드 2개를 쓴다:

  ```
  현재:  Zenoh worker └── build() 30s

  신규:  Zenoh worker  └── future.result() 대기
         event loop    └── await asyncio.to_thread(build)
         threadpool    └── build() 30s
  ```

  "pool>1 이라 무방"(§6) 이 성립하려면 워커 pool 이 실제로 >1 이어야 한다. 구현 중 눈으로
  확인해 둘 값 — **구조를 바꿀 리스크는 아님**.

### ③ 추후 정책 (실사용 경험 후 결정)

- **heavy-work 자동 offload** — `to_thread` 관례로 시작. `@service(offload=True)`(§6-b) /
  worker pool(§6-c) 는 실사용에서 반복 필요성이 보일 때 검토.
- **timeout / 취소** — long handler 의 client timeout ↔ loop-side coroutine 취소 경로.
- **sync 핸들러 backward-compat 존치 기간** (§7) + 일괄 vs 점진 전환.
- **`call_sync` 제거 후 API 정리** — 아래 구현 순서 2번에 포함.

### 구현 (2026-07-03 완료 — 180 test PASS, ruff/pyright clean)

1. ✅ **framework 가 sync/async bridge 를 완전히 흡수** — [app.py `_register_service`](../backend/framework/runtime/app.py)
   `handler_bytes` 가 `asyncio.iscoroutine(result)` 면 `run_coroutine_threadsafe(coro,
   self._loop).result()` (timeout 없음 — long build 는 핸들러 안 `to_thread` 로 loop 안
   막고, 워커 스레드만 완료까지 대기 = sync 핸들러와 동일). `_register_subscriber` 도
   coroutine 이면 fire-and-forget schedule + done-callback 으로 예외 로깅. sync 핸들러는
   기존대로 직접 호출 (backward-compat).
2. ✅ **`call_sync` — 애초에 public API 에 없었음.** `ModuleRuntime` protocol
   ([api.py](../backend/framework/runtime/api.py)) 은 처음부터 `call` 단일. zenoh
   transport 내부 `_call_sync` 만 존재하고 그건 이미 `to_thread` 로 흡수된 올바른 자리.
   §4 의 "call_sync 폐기" 는 선제적 표현이었고 **제거할 대상이 없었다** (no-op 확인).
3. ✅ **scan 모듈 async 정리** — [scan/module.py](../backend/modules/scan/module.py) 의
   `_call` / `self._loop` / `Coroutine`·`cast`·`TypeVar`·`BaseModel` import 삭제.
   `capture`/`build` → `async def` + `await self.runtime.call(...)`. `build_mesh` →
   `await asyncio.to_thread(...)`.
4. ✅ CPU 집약(build_mesh)은 `await asyncio.to_thread(...)` (③-heavy 관례). `@service(
   offload=True)` 자동화는 미도입 — 실사용에서 반복 필요성 보이면 그때.

핵심 목표 **"모듈 개발자는 `await runtime.call(...)` 만 알면 된다"** 달성. 이후 정책
(②-pool 측정 / ③-heavy·timeout·sync 존치)은 실사용 경험 위에서.

## 9. 관련 문서

- [backend.md](backend.md) — framework SSOT (4 primitive / Runtime lifecycle / Owner-Reader). 본 문서는 그 위 **실행모델(sync/async) 정정**.
- [backend.md](backend.md) — Runtime/Module/Transport 3 layer reframe.
- [project_scan_pragmatic_slice] — `_call` 브리지가 처음 등장한 자리 (이 논의의 발단).


---
---

<!-- ═══════════ [통합 원문] backend.md ═══════════ -->

# contract gen — 분산 배치·정적 생성 논의 기록 (2026-07-06)

> **결정: 지금은 아무것도 안 바꾼다.** gen:types 는 현행(전 모듈 로드된 mock/dev
> backend 에서 생성) 유지. 본 문서는 그 결정에 도달하기까지 검토·기각된 선택지들과
> 재논의 트리거를 남기는 기록 — 다음 세션이 같은 의심을 처음부터 다시 돌지 않게.

## 1. 출발 질문

분산 배치 — PC1(모듈A, 모듈B, bridge) + PC2(모듈C, 모듈D) — 에서 모듈 D 의 계약을
프론트로 노출할 수 있는가?

**답은 두 축으로 갈린다:**

| 축 | 되나 | 메커니즘 |
|---|---|---|
| **데이터** (서비스 호출/스트림 구독) | ✅ | bridge 는 relay-only, Zenoh 가 D 의 위치를 투명 라우팅. bridge 와 D 가 다른 host 여도 무관 |
| **타입 생성** (gen:types) | 부분 배치에선 ❌ (의도된 가드) | `build_contract_json` 이 running runtime 의 snapshot 에서 payload 를 채움 → bridge host 에 로드 안 된 모듈의 payload 없음 → incomplete-host guard 가 거부 |

핵심 구분: **gen:types 는 build-time 스텝이다.** contract.ts 는 개발 머신에서 생성해
커밋하는 산출물이고, 분산 배포 중에 gen 을 돌릴 일 자체가 없다. 따라서 위 ❌ 는
운영 결함이 아니라 "gen 은 전 모듈 로드된 host 에서" 라는 워크플로 전제.

## 2. 코드로 확정한 사실 (재검증 불필요)

1. **key↔payload 바인딩의 원천 = 핸들러 시그니처.** contract.py 에는 키(StrEnum)와
   타입(BaseModel)이 나란히 있을 뿐 연결 선언이 없다. 연결은
   `@service` 데코레이터가 `get_type_hints(method)` 로 시그니처에서 추출
   ([framework/contract/service.py](../backend/framework/contract/service.py)).
2. **추출된 바인딩은 metadata 로 클래스에 이미 부착된다** (`ServiceSpec` →
   `_service_spec` attr, `@publishes` 는 데코레이터 인자로 payload 명시). 즉
   **module.py 를 import 만 하면** 인스턴스/runtime 없이 전부 읽힌다 —
   `build_snapshot_from_classes` ([framework/runtime/snapshot.py](../backend/framework/runtime/snapshot.py))
   가 그 구현이고 `/contract/graph` 가 실사용 중.
3. **json 과 graph 의 소스가 다른 건 드리프트가 아니라 요구 차이.**
   graph = 분산 배치 *운영 중에도* 전 fleet 을 보여야 하는 런타임 뷰어 → MODULE_REGISTRY
   정적 introspect. json = build-time 산출물 → running mock 으로 충분.
4. **"전체 import 가능" 은 우연이 아니라 아키텍처 전제.** Runtime(프로세스) 이 최소
   단위고 모듈들은 한 인터프리터에 동거한다 (framework_dogfood_plan §15). 같은 host 에
   배치되는 모듈들의 의존성 공존은 요구사항이며, 개발 머신은 "전 모듈을 배치하는
   host"(mock) — pytest(전체 import) / mock 부팅도 같은 전제 위에 있다.
   **모듈별 venv/Docker 는 이 아키텍처와 양립 불가** (그건 폐기된 "Node=최소단위" 회귀).

## 3. 검토된 선택지 사다리 (우선순위 순)

### 3.1 현행 유지 — 채택 (지금)

전 모듈 로드된 mock/dev backend 에서 gen. incomplete-host guard 가 부분 host 실수를
명확한 메시지로 차단. 개발 루프가 어차피 mock/pytest 로 전체 import 를 요구하므로
gen 만의 추가 전제가 없다.

### 3.2 모듈별 fragment 빌드 산출물 + merge — **미래 지정 경로**

각 모듈(의존성이 있는 자기 환경에서)이 `contract.fragment.json` 을 빌드 산출물로
남기고, gen 은 fragment 들을 수집·merge. **Runtime 불필요 + Zenoh 불필요 + 전체
import 불필요** — 셋을 동시에 만족하는 유일한 선택지라 미래 경로 1순위.

- fragment 내용은 클래스 객체가 아니라 **qualified 이름 문자열**
  (`waypoint.TeachRequest`) 이면 충분 — merge 측이 contract.py 전체(import-light,
  어디서든 가능)로 full catalog 를 만들고 이름으로 실 클래스를 해소. name-conflict
  는 기존 `resolve_names` 그대로.
- framework 수정 불필요 — `build_snapshot_from_classes` 가 per-class 리스트로 이미
  동작 (2026-07-06 확인).
- **버전 정합 가드 필수**: fragment 에 git hash (또는 schema version) 를 박고
  불일치 merge 거부. 서로 다른 checkout 의 fragment 를 섞으면 조용히 깨진
  contract.ts 가 나온다.

### 3.3 Zenoh fleet 집계 (runtime 에 fragment 서비스) — 목적이 다른 도구

각 Runtime 이 `contract_fragment` 서비스를 들고, gen 시점에 zenoh 로 전 peer 의
fragment 를 모아 merge. 기술적으로 성립하고 (각 host 는 자기 모듈을 이미 import 중,
fragment = 문자열) zenoh-native 라 우아하지만 — **빌드 도구가 "fleet 이 떠 있음"
이라는 운영 상태에 의존하게 된다.** 이는 애초 문제(타입 생성이 실행 상태에 결합)의
방향만 바꾼 재현.

→ 판정: **"프론트 타입 생성" 용도로는 부적합. "현재 배포된 fleet 의 실 계약 조회"
라는 별개 목적이 생기면 그때 자연스러운 설계.** 두 목적(build-time 타입 생성 vs
runtime fleet 조회)을 한 메커니즘으로 묶지 말 것.

- 부수 논점: "fleet 완전성 검사" 도 목적 따라 다르다 — 프론트가 안 쓰는 모듈이
  꺼져 있다고 gen 이 실패할 이유는 없음. 완전성 기준은 fleet 전체가 아니라
  FRONTEND_EXPOSED 커버리지.

### 3.4 AST 파싱 — 최후 수단 (사실상 안 씀)

import 없이 소스 텍스트에서 추출. 이 repo 는 **전 파일 `from __future__ import
annotations`** → 모든 annotation 이 문자열이라 alias/forward-ref/typing 해석기를
직접 만들어야 함 (`get_type_hints` 가 공짜로 해주는 것 전부). 유지보수 비용이
Python 언어 기능을 따라가는 영구 부채.

### 3.5 contract.py 에 바인딩 승격 (`SERVICE_PAYLOADS` 선언) — 기각

한때 "정적 생성의 유일한 길" 로 검토됐으나, §2-2 사실(데코레이터 metadata 가 이미
존재, import 만으로 읽힘)로 전제가 무너짐. 선언 중복 + 컨벤션 변경(10개 contract +
framework 검증 + spec 문서) 비용만 있고 얻는 게 없다. **payload 정보를 얻는 방법이
하나뿐이라는 잘못된 가정에서 나온 과잉 해결** — 문제는 "선언 위치" 가 아니라
"추출을 어디서 하느냐" 였다.

## 4. 재논의 트리거

**"어떤 모듈의 의존성이 나머지와 한 venv 에 공존 불가" 가 실제로 발생하는 날.**

그날은 gen 만이 아니라 pytest(전체 import)·mock 단일 프로세스 부팅이 같이 깨지므로
대응은 한 묶음이다:

1. 충돌 모듈을 별도 host/deployment 로 분리 (기존 메커니즘 — deployment yaml + role group)
2. 개발 루프: mock 단일 프로세스 → multi-process sim (host_*_sim 방식) 으로 분할
3. gen: §3.2 (fragment 빌드 산출물 + merge + git hash 가드) 채택

그 전까지는 어떤 선제 구현도 하지 않는다 (1-peer 환경에서 분산 메커니즘의 가치가
발휘될 상황 자체가 없음).

## 5. 남긴 미해결 (재논의 시 진입점)

- fragment 스키마 상세 (git hash 외 schema version 병기 여부)
- 완전성 기준 — FRONTEND_EXPOSED 커버리지 vs fleet 전체 (§3.3 부수 논점)
- "현재 fleet 계약 조회" 라는 별개 도구의 실수요 여부 (§3.3 을 그 목적으로 부활할지)
- `/contract.json` 엔드포인트의 장기 위상 — gen 이 §3.2 로 이관되면 소비자가
  사라짐 (제거 vs runtime 디버그 조회용 존치)


---
---

<!-- ═══════════ [통합 원문] backend.md ═══════════ -->

# Node Framework Dogfood Plan

> 본 문서는 [dev_reference.md](dev_reference.md) 의 검토 phase 첫 큰 산출물.
> **2026-06-25 update — §15 Runtime-centric reframe.** §14 까지의 plan 은 Node 가 최소 단위라는 잘못된 전제. 진짜 깨달음 = Runtime (Process) 이 최소 단위, Module = 기능 묶음. backend_v2/ 폴더 삭제, §15 위에 다시 짬.
> 새 세션에서 "framework 진행하자" / "Runtime" / "Module" / "Transport adapter" 톤 던지면 본 문서 진입.
> 결정된 것만 정리. 미정 항목은 §9.

## 1. 배경

검토 phase 진입 직후 사용자가 짚은 본질:

> "레이어 분리는 잘 됐는데 사람 (나 또는 추후 개발자) 이 코드 이해/파악이 너무 어렵다"

원인 분해 (사용자 + Claude + GPT 공동 reframing):

1. **기능 추적 비용** — 한 wire 추가 시 `messages` → `topic_map` → `api_contract` → handler → repo → client → frontend store → component 까지 N 파일 횡단
2. **반복 boilerplate** — schema + topic 등록 + contract 등재 + handler + client wrapper + frontend gen 패턴 매번 복붙. "작은 RPC 프레임워크를 손으로 쓰는 형태"
3. **Storage Node 책임 침범** — 워크플로우 단계 (run finalize / result activate 등) 가 storage service 안에 들어와 있음. CRUD 인프라가 도메인 로직 흡수

## 2. 3대 방향

| # | 방향 | 채택 |
|---|---|---|
| 1 | 미니 framework (boilerplate 제거, FastAPI DX 미러) | ✅ |
| 2 | system-docs 자동 생성 (노드별 service/topic/publish 한눈) | ✅ |
| 3 | Storage = CRUD only, Workflow = 도메인 노드 | ✅ (경계는 case-by-case 합의 후 진입) |

## 3. 설계 원칙 (다음 세션도 유지)

1. **매직 스트링 금지** — `Service` / `Topic` enum SSOT 유지. 데코레이터 인자는 enum *referent*, 새 string SSOT 신규 X
2. **데코레이터 = binding 메타** — 기존 SSOT (enum + Pydantic message) 들을 함수에 묶는 역할. 새 SSOT 만들지 X
3. **목표 = "transport boilerplate 몰라도 domain 코드만 작성"** — *프레임워크 만들기* 가 아닌 DX 개선
4. **`backend/framework/` 폴더** — frontend `framework/` 와 명명 일치 ([frontend/src/framework/index.ts](../frontend/src/framework/index.ts) 검증된 패턴)
5. **두 audience 분리**:
   - *운영 contract* = `PUBLIC_TOPICS / PUBLIC_SERVICES` 필터, frontend `contract.ts` 자동 emit (기존)
   - *dev system-docs* = 전체 노출, frontend `/system-docs` page (신규)
   - 같은 registry, 다른 exposure
6. **DI container 안 도입** (2026-06-24 결정) — FastAPI `Depends` 의 call-time lookup 본질은 HTTP request lifecycle 에 묶인 패턴. 우리는 process-scoped service라 정당화 약함. testability 는 monkey-patch 패턴 (test_gamepad) 이미 정착. lazy singleton + 명시적 `__init__` 호출로 충분 — cargo cult 회피
7. **production code 자리 dogfood 박지 말 것** (2026-06-24 결정) — test 만을 위한 메소드 / attribute 를 production class 에 박는 자체 noise. cross-process verification 안 되는 자리는 production 박을 정당화 없음. test 안 self-contained dummy class + raw string topic + dummy Pydantic 로 framework 자체만 검증
8. **점진 적용 = 검토 위함** (2026-06-24 명확화) — 호환성 보장 X (개발 단계). 작은 commit + 검토 + 한 자리씩 변환. 변경 때문에 다른 노드 깨지면 `host_mock.yaml::application_nodes` 에서 잠시 빼놓고 진행, 끝나면 다 변환 + 다시 활성

## 4. Framework API (현재까지 확정 — 2026-06-24)

### 4.1 `@service` — RPC handler

```python
from framework import service
from core.transport.messages.base import ServiceRequest, ServiceResponse

class StorageNode(ApplicationNode):
    @service(Service.STORAGE_LIST_CALIBRATIONS)
    def list_calibrations(
        self, req: ServiceRequest[ListCalibrationsReq]
    ) -> ServiceResponse[ListCalibrationsRes]:
        ...
```

- `key` = enum referent (`Service.X` 값)
- `req_cls` / `res_cls` = type hint 에서 자동 추출 (FastAPI 패턴)
- Pydantic v2 generic (`ServiceRequest[X]`) 은 `typing.get_args()` 가 빈 tuple 반환 → `__pydantic_generic_metadata__["args"]` fallback 박혀있음 ([framework/service.py](../backend/framework/service.py))

### 4.2 `@subscriber` — Topic subscriber

```python
from framework import subscriber

class FooNode(BaseNode):
    @subscriber(Topic.STORAGE_CALIBRATION_INVALIDATED)
    def on_invalidation(self, msg: CalibrationInvalidated) -> None:
        ...
```

- `key` = enum referent
- `msg_cls` = type hint 직접 (envelope X, service 와 다름)
- `from __future__ import annotations` 환경에서 `get_type_hints(func)` 가 `func.__globals__` 만 보고 local scope 못 봄 → type hint 의 Pydantic class 는 module-level import 필요

### 4.3 `@publishes` — Topic publisher (Phase B — 미구현)

```python
class MotorNode(BaseNode):
    @publishes(Topic.MOTOR_STATE_JOINT)
    def _publish_state(self, state: MotorJointState) -> None:
        self.publish(Topic.MOTOR_STATE_JOINT, state)
```

**mechanism — mark only** (FastStream wrap 패턴 채택 X). 이유:
- 우리 publish 패턴은 worker loop / event callback 안 `self.publish(...)` 호출 — 함수 return 자동 publish 패턴 안 맞음
- mark only 면 함수 본문 자유 + registry 가 *이 함수가 Topic.X 발행한다* 만 인식
- boilerplate 증가 = 데코 한 줄

class-level (`__publishes__ = (Topic.X,)`) 옵션은 *어디서* 정보 손실 — 함수-level mark 채택.

## 5. attach 가능한 객체 — class hierarchy 강제 X (2026-06-24 재결정)

**§5 이전 버전의 BaseComponent 다이어그램 폐기.** 박을 때 잘못된 진단 박힘 — "JointStateCache 가 `__init__` 안 `ZenohSession.declare_subscriber` 직접 호출하면 invisible" — 현재 cache 코드는 그렇게 안 돼있음. cache 는 `node.create_subscriber` 위임, lifecycle 은 노드가 가짐. hypothetical scenario 자체로 lifecycle 계층 (BaseComponent) 끌어온 cargo cult.

**진짜 문제** = `JointStateCache` 가 어떤 topic 듣는지 framework registry 에서 안 보임. 해결 = 메소드에 `@subscriber` 데코 박는 것만. base class 상속 강제 X.

```
framework helper (bind_decorated_subscribers / collect_*_specs_from_instance)
        ↑                ↑                ↑
        |                |                |
    BaseNode         Handler           Cache
    (start() 안 호출)  (node.attach_     (__init__ 안 호출)
                      handler(self))
```

**원칙** — 데코 박은 메소드 = 계약 + 실행 엔트리포인트. 별도 mark (`__subscribes__ = (...)`) X — dual source of truth 위반.

**framework 가 보고 dispatch 하는 객체 카테고리**:

| 카테고리 | 예시 | attach 시점 |
|---|---|---|
| Node (BaseNode 상속) | CalibrationNode / MotionNode | `start()` 안 self bind |
| Handler (composition member) | CalibrationHandlers / ScanWorkflowHandlers | 노드가 `attach_handler(self)` |
| Cache (process singleton) | JointStateCache / FrameCache | `__init__` 안 self bind |

class hierarchy 강제 X — 셋 다 동일 framework helper (`bind_decorated_subscribers(obj)`) 호출만 다름.

**Bridge 는 scan 제외** — infrastructure layer (FastAPI middleware 등가). application contract 가 아닌 plumbing. system-docs viewer 에 안 박힘.

**docs 두 레벨 분리 (2026-06-24, distributed 관점)**:

- **레벨 1 — 객체 contract**: 단일 객체가 *스스로* 박는 정보. subscribes / publishes / services. `@subscriber` / `@publishes` / `@service` 데코로 객체 안에 박힘. PC 어디 떠 있든 무관 = local declaration.
- **레벨 2 — 시스템 topology**: 여러 객체 contract 합쳐서 보임. "DetectorNode publish DETECTION_RESULT" → "Scene3DNode subscribes DETECTION_RESULT" 자리 연결. registry 전체 합치면 자동 생성.

caller 관계 (누가 service 호출하나) 는 docs 목표에서 제외 — *분산 observability* 문제 자체 별개 layer (process-local 정보 X, `@calls` 박을 수 없음). `@publishes` 의 process_name / module 같은 metadata 자체 후속 검토 (Phase B 진입 시).

## 6. 메타 질문 (2026-06-24 학습)

"새 노드 / framework 변환 시 항상 던질 질문":

1. **이 노드는 일반 노드인가, composite host 인가?** — composite host (예: StorageNode + CalibrationHandlers / ScanWorkflowHandlers) 면 sub-handler 패턴 + `attach_handler` 사용. 일반 노드면 `__init_subclass__` scan 만으로 충분.
2. **이 wire 는 cross-process verification 가능한가?** — service 면 mock backend spawn + test peer call → 응답 받음으로 verify. subscriber callback / production attribute 는 backend process 안 → test peer 가 read 못 함. 안 되면 production 에 박지 말고 test 안 self-contained dummy class 로 framework 만 검증.
3. **dogfood 가 test 만을 위한 production code 박는 경우인가?** — production class 에 dogfood 메소드 / attribute 박는 것 자체 noise. 메소드가 production 으로 실제로 사용되거나 (V2 service 같이 cross-process verification 경우) 아니면 박지 말 것.
4. **FastAPI / Spring / FastStream / Faust / ROS 2 의 어떤 패턴을 차용하나?** — 우리 use case 정당화되는 부분만 차용. 겉모양 / 명명만 흉내 X (cargo cult 회피 메모리).
5. **이 데코는 계약(선언)인가, 실행 흐름인가?** — `@service` / `@subscriber` 는 framework 가 *호출* 하는 자리 = 선언적 계약. `@publishes` 는 객체가 *호출* 하지만 클래스 scope 라 AST 로 데코 vs 실제 `self.publish` 일치 검증 가능 → 계약 OK. `@calls` 는 함수 scope + flow + wrapper + 조건부 → 검증 어려움, stale 위험 → 데코 X, runtime call graph 로 풀 것.
6. **이 hypothetical 진단 박을 때 실제 코드 봤나?** — §5 BaseComponent 다이어그램 박을 때 "cache 가 `__init__` 안 declare_subscriber 직접 호출" 이라고 잘못 진단. 실제 코드는 `node.create_subscriber` 위임. 가상 시나리오로 framework 계층 끌어오는 것 자체 cargo cult. 진단 박기 전 코드 grep 필수.
7. **이 정보는 process-local 인가, 시스템 layer 인가?** — distributed 환경에서 PC A 의 객체가 PC B 의 객체에 대해 *스스로* 박을 수 없는 정보는 객체 contract 가 아님 = 분산 observability layer. `@subscriber` / `@publishes` / `@service` = local declaration (객체 스스로 박힘) = contract OK. `@calls` / caller graph = 시스템 topology (누가 나를 호출하는지는 process 너머 정보) = docs 목표에서 제외.

## 7. Phase 순서

| Phase | 작업 | 산출물 | 상태 |
|---|---|---|---|
| 0 | Storage CRUD vs Workflow 경계 *합의* (코드 짚어서, 실제 이동 X) | 본 문서 §10 표 정밀화 | 미진행 — Phase 5 진입 전 자리 |
| 1 | 1 wire dogfood — framework MVP + `STORAGE_LIST_CALIBRATIONS_V2` 변환 + cross-process test | 동작 + 6 dogfood test PASS | ✅ **완료** (2026-06-24) |
| 2 | dogfood 평가 + 메타데이터 SSOT shape 확정 — `@service` / `@subscriber` 데코 + composite host (`attach_handler`) + production 미침범 | §3 / §5 / §6 결정 박힘 | ✅ **완료** (2026-06-24) |
| **A** | **framework 확장 (signature codec / robot_id inject / wildcard expand / dedup) + cache 한 곳 변환 (JointStateCache)** | cache 가 `@subscriber` 박힌 일반 객체로 동작, registry visible | ✅ **완료** (2026-06-24) — framework/topic.py 확장 + framework/binding.py 신규 + base_node refactor + JointStateCache 변환 + 호출자 6곳 정리. dogfood 6 PASS / calibration_e2e 2 PASS / 전체 pytest PASS |
| ~~B~~ | ~~`@publishes` 데코 + AST lint + 노드 publish 한 곳씩 변환~~ | ~~publish 도 contract SSOT~~ | **보류 — backend_v2 reframe (§14)** |
| ~~C~~ | ~~system-docs viewer~~ | ~~"누가 발행 / 누가 듣는지" 시각화~~ | **보류 — backend_v2** |
| ~~3~~ | ~~두 번째 wire (`MOTION_MOVE_L`) dogfood~~ | ~~robot-scoped placeholder + multi-dispatch 검증~~ | **보류 — backend_v2** |
| ~~4~~ | ~~나머지 wire 일괄 마이그레이션~~ | ~~모든 service/topic `@service` / `@subscriber`~~ | **보류 — backend_v2 완성 후 backend/ discard** |
| ~~5~~ | ~~Storage workflow service 들 도메인 노드로 재배치~~ | ~~Storage = 순수 CRUD~~ | **보류 — backend_v2 의 Component 분리 자체가 흡수** |

**Phase A 세부 변경 (framework)**:
1. robot-scoped key (`{robot_id}` placeholder) 감지 → wildcard subscribe (`horibot/*/...`) + sample.key_expr 에서 robot_id 추출
2. callback signature codec 판단 — `msg: Pydantic` → validate, `payload: bytes` → skip
3. callback signature inject — `(self, robot_id, msg)` / `(self, robot_id, payload: bytes)` / `(self, msg)` / `(self, payload)` (robot-scoped 여부 + codec 조합)
4. instance 단위 bound dedup (cache singleton 자리 N 개 노드가 attach 호출해도 한 번만 bind)
5. `bind_decorated_subscribers(obj)` 일반 helper — Node / Handler / Cache 모두 동일 호출 (BaseNode.start / node.attach_handler / cache.__init__ 안에서 각각)

**Phase A 진입 후 JointStateCache 변환 결과**:

```python
class JointStateCache:
    def __init__(self):
        if self._initialized: return
        self._initialized = True
        ...
        bind_decorated_subscribers(self)

    @subscriber(Topic.MOTOR_STATE_JOINT)
    def _on_motor_state(self, robot_id: str, msg: MotorJointState):
        ...
```

호출자 노드의 `cache.subscribe(self, rid)` 패턴 폐기 — cache 가 자기 subscribe 책임.

## 7.5 Reframe — backend_v2 실험실 (2026-06-24, §14 참조)

Phase A 까지 박은 후 사용자가 더 근본 질문 던짐:

1. **노드 자체 진짜 필요한가?** — framework 박힌 후 cache/handler 가 self-contained 객체로 동작. 노드는 *grouping convention* 일 뿐 — process / robot / component 가 진짜 unit. ROS mental model 의 유산.
2. **여러 노드가 여러 번 구독 = 정상** — Zenoh pub/sub 본질. cache 의 motivation 은 *상태 공유* (ROS-think) 아니라 *boilerplate + 변환 wrapper* (JointState) 또는 *expensive transformation memoization* (Frame decode).
3. **운영 X = 리라이트 cost 작음** — "이미 개발했음" 은 cost-based 근거 (메모리 위반). 진짜 합리적이면 처음부터 다시.
4. **Contract First + Binder = 확정. Node 완전 삭제 + Handler/Cache/Worker/Adapter 4분류 = 가설** — 코드로 검증 필요.

결정 = backend_v2/ 실험실 박음. §14 자체 plan.

Phase B / C / 3 / 4 / 5 자체 자체 보류 — backend_v2 결과에 흡수. 단 Phase A 산출물 (framework 확장 + JointStateCache 변환) 자체 자체 backend/ 안에 박혀 있음 — 회귀 0, 운영 (개발 단계) 안 깨짐.

## 8. 완료된 dogfood (Phase 1)

**wire = `STORAGE_LIST_CALIBRATIONS_V2`** (read-only, host-level, 단순)
- [topic_map.py:127](../backend/core/transport/topic_map.py#L127) — V2 enum 추가 (dogfood-only 임시 wire)
- [handlers/calibration.py:144](../backend/nodes/application/storage/handlers/calibration.py#L144) — `@service(STORAGE_LIST_CALIBRATIONS_V2)` 메소드. `_srv_list` 위임 (구현 재사용)
- [handlers/calibration.py:139](../backend/nodes/application/storage/handlers/calibration.py#L139) — `register()` 끝에 `node.attach_handler(self)` 한 줄로 sub-handler 의 `@service` 메소드 자동 발견 + bind

**dogfood test 6 PASS** ([tests/test_framework_service_dogfood.py](../backend/tests/test_framework_service_dogfood.py))
- `test_v2_same_response_as_v1` (same-process round-trip)
- `test_v2_round_trip_via_mock_backend` (L3 — host_mock subprocess + 분리 zenoh peer cross-process)
- `test_v2_spec_on_sub_handler` (composite host spec discovery)
- `test_subscriber_spec_on_general_node` (BaseNode `__init_subclass__` scan)
- `test_subscriber_spec_on_sub_handler` (`collect_subscriber_specs_from_instance`)
- `test_subscriber_callback_round_trip` (in-process publish → callback 발동)

subscriber test 자리 production 미참조 — `_DOGFOOD_TOPIC = "test/framework/dogfood"` raw string + `_DogfoodMsg(BaseModel)` test 안 dummy.

**갈아치움 step (Phase 4 자리)** — `_srv_list` 의 본문을 `list_calibrations_v2` 로 흡수, `register()` 안 `Service.STORAGE_LIST_CALIBRATIONS` 등록 줄 제거, V2 enum 제거 → frontend / 다른 caller 안 건드림 (key 자체 유지).

## 9. 미정 항목 (다음 세션 진입점)

| # | 미정 | 결정 시점 |
|---|---|---|
| 1 | ~~`@service` 가 owner 노드를 어떻게 식별~~ | ✅ Phase 2 — `__init_subclass__` 자동 scan + composite host 자리 `attach_handler` |
| 2 | robot_id scope 처리 — `BaseNode.r()` 위에 얹는지 / 데코레이터가 직접 관리 | Phase 3 (MOTION_MOVE_L) |
| 3 | ApplicationNode `dict[robot_id]` multi-dispatch 와 데코레이터 결합 | Phase 3 |
| 4 | Storage CRUD vs Workflow 경계 case-by-case 표 정밀화 | Phase 0 (Phase 5 진입 전) |
| ~~5~~ | ~~Phase A 진입 — A1 / A2 갈래~~ | ✅ Phase 2 재검토 — BaseComponent 폐기, framework 확장 + cache 변환만 (§5 / §7 정리) |
| 6 | callback signature inject 디테일 — robot-scoped 아닌 경우 robot_id 인자 자체 없애야 (signature 검사) / Pydantic vs bytes codec 판단 fail-fast | Phase A 구현 |
| 7 | AST lint 구현 — `@publishes` 데코 자리 vs 실제 `self.publish(Topic.X)` 일치 검증 | Phase B |

## 10. Storage 경계 case-by-case (Phase 0 입력)

현재 코드 의심 후보 (Phase 0 에서 추적해 확정):

| service | CRUD ✅ / Workflow ⚠️ | 비고 |
|---|---|---|
| `STORAGE_LIST_CALIBRATIONS` | ✅ | 단순 read |
| `STORAGE_COMMIT_CALIBRATION` | ⚠️ | run finalize + result INSERT + activate 묶음 |
| `STORAGE_ACTIVATE_CALIBRATION` | ⚠️ | 같은 (robot, kind) atomic toggle. 트랜잭션 + 도메인 로직 섞임 |
| `STORAGE_NEW_SCAN_SESSION` | ✅ | session row INSERT |
| `STORAGE_DELETE_SCAN_SESSION` | ✅ | CASCADE — 트랜잭션 무결성 |
| `STORAGE_PUT_SCAN` | ✅ | scan_id allocate + blob put + row INSERT |
| `STORAGE_PUT_RECONSTRUCTION` | ✅ | append-only blob + row |

→ 경계 원칙 *잠정*:
- **트랜잭션 (atomic 보장 필요)** → storage 안 OK
- **도메인 결정 (어느 result 가 active / run status transition)** → 도메인 노드로 이동

Phase 0 = 위 표 정밀화 + 원칙 fix.

## 11. 검토 protocol 와의 관계

본 작업은 [dev_reference.md](dev_reference.md) §산출물 의 첫 큰 docs 산출. protocol 제약 그대로 적용:
- 단편 reflex X — FastAPI 그대로 복사 X, 우리 use case 에 맞게 수정
- "cheap fix" / "한 줄이면 끝" 어휘 X
- "개인 학습 프로젝트라서" / "N=2 라서" scope 핑계 X
- 사용자 push 에 입장 뒤집기 X
- md/docs 인용 X — 실제 코드 우선
- 의도 떠넘기지 X
- "자리" placeholder 의미 없이 박지 X (메모리 자리)
- 짜기 전 hand-simulate + edge case 사전 질문 (memory)

## 12. 다음 세션 시작점

새 세션 진입 시:

1. 본 문서 + [dev_reference.md](dev_reference.md) 동시 anchor.
2. **§15 Runtime-centric reframe (2026-06-25)** 가 현재 결정 — Node 가 잘못된 전제, Runtime (Process) 이 최소 단위, Module = 기능 묶음.
3. **backend_v2/ 폴더 폐기 (2026-06-25)** — Phase 1 MVP 산출물 (framework + 7 test PASS) 이 Node 잘못된 전제 위 코드. §15 reframe 위에 다시 짬.
4. §14 = history (잘못된 전제 위 plan).
5. Phase 1 / 2 / A (§14 의 backend/ 변환) — docs 에 완료 기록 있으나 사용자가 이후 discard. main branch 코드에 framework/ 없음 (참고만).
6. 다음 step = Transport abstraction (§15.7).
7. 새 코드 작성 전 §6 메타 질문 7 가지 던질 것.
8. test 짤 때 production code 에 dogfood 넣지 말 것 — self-contained dummy class 로 framework 만 검증.

사용자가 "Runtime" / "Module" / "Transport adapter" / "Contract layer" / "distribution is runtime concern" 톤 던지면 §15 진입. "backend_v2" / "Component 4분류" / "Node 삭제" 톤은 §15 reframe 안내 (§14 history). "BaseComponent" 톤은 §13.8 정정 안내.

## 13. 결정 history (학습 anchor)

본 plan 진행 중 잘못 짚었다가 사용자가 정정한 자리 — 다음 세션 같은 실수 회피:

1. **handler 분리 패턴 자체 못 짚음** (대화 초기) — `StorageNode` 의 `CalibrationHandlers` / `ScanWorkflowHandlers` composition 자리 보자마자 "composite host?" 메타 질문 던졌어야. V2 메소드를 StorageNode 자체에 박은 자체 잘못. 정정 후 sub-handler `attach_handler` 패턴 박힘.
2. **FastAPI DI cargo cult** — Depends + call-time lookup 패턴 우리에게 적합한지 따져봤다가, HTTP request lifecycle 자체에 묶인 패턴 자리 우리 process-scoped service 자리 정당화 약함 발견. monkey-patch + lazy singleton 패턴 유지.
3. **production code 자리 dogfood 박음** — CalibrationNode + CalibrationHandlers 에 dogfood `@subscriber` 메소드 + attribute 박은 자체 잘못. test peer 가 backend process 안 attribute 못 read → test 가 결국 in-process dummy class 자리. production class 박힌 자리 unused 잔재. 정정 후 production 미참조 + test self-contained.
4. **`STORAGE_CALIBRATION_INVALIDATED_V2` enum 추가도 잘못** — dogfood-only wire 인데 production code 자리 V2 enum publish 자체 없음. enum 자리 잔재. 정정 후 raw string topic 으로 갈아치움.
5. **DI / Container 자리 너무 빨리 확장** — FastStream 패턴 차용 자리 사용자가 다시 짚은 자리 = "JointStateCache 같은 cache 가 framework registry visible 되어야 한다" 만. DI container 자리 사용자 의도 X. BaseComponent layer + `@subscriber` 자체 확장 자리만 정당화.
6. **publish 도 framework registry 자리 visible 필요** — 처음 plan 자리 `@service` / `@subscriber` 만. publish 자리 누락. 사용자가 짚은 자리 = "어떤 노드가 publish 한다" docs 자리 안 보임 → `@publishes` 데코 추가 결정.
7. **점진 적용 = 호환성 X, 검토만** (사용자 명확화) — 개발 단계라 한 곳 깔끔 변환. 다른 노드 깨지면 host_mock 에서 잠시 끄고 진행. *기존 패턴 호환 layer* 안 둠.
8. **BaseComponent 추출 — hypothetical scenario 로 박은 cargo cult** (2026-06-24 두 번째 정정) — §5 다이어그램 박을 때 "cache 가 `__init__` 안 `ZenohSession.declare_subscriber` 직접 호출하면 invisible" 진단 박음. 실제 코드는 `node.create_subscriber` 위임 — cache 가 ZenohSession 직접 안 만짐. 가상 시나리오로 lifecycle 계층 끌어옴. 사용자 reframe — "원래 문제는 docs visibility 만, lifecycle 은 따로 문제" — 정답. 진단 박기 전 코드 grep 필수 메모리 박힘 (§6 메타 6).
9. **`__subscribes__ = (...)` dual mark 추천 — dual source of truth 위반** (2026-06-24) — cache 변환 옵션으로 "class-level mark 한 줄" 박았는데 사용자 정정: 실행 로직 (`def subscribe(...)`) 과 계약 (`__subscribes__`) 분리되면 어긋날 수 있음. 정석 framework = 실행 엔트리포인트 = 계약 (FastAPI `@app.get` 등). 따라서 `@subscriber` 데코 박은 메소드 = 계약 + 실행 한 곳.
10. **`@calls` 와 `@publishes` 같은 카테고리로 묶은 잘못** (2026-06-24) — `@calls` 폐기 한 후 같은 논리로 `@publishes` 도 폐기 박았는데 사용자 반박: "그러면 system-docs viewer 가 누가 publish 하는지 어떻게 박나?" runtime instrumentation 은 idle 노드 / 조건부 publish 못 박음. 차이 = 검증 가능성 — `@publishes` 는 클래스 scope 라 AST 로 데코 vs 실제 `self.publish(Topic.X)` 일치 검증 가능, `@calls` 는 함수 scope + flow + wrapper + 조건부 → 검증 어려움. 따라서 `@publishes` 유지 + AST lint.

    **§10.5 distributed 관점 강화 (2026-06-24 후속)** — `@calls` 대안으로 "runtime caller graph 박자" 박았는데 사용자 반박: distributed 환경에서 `TaskNode (PC A) → MotionNode (PC B)` 호출 자리 MotionNode 프로세스는 호출자가 GroundedDetect 인지 PickTask 인지 모름. caller graph 자체 process-local 정보 X = *시스템 topology* 정보 = 분산 observability layer. 객체 contract 와 다른 layer. 따라서 runtime call graph 자체도 폐기 — *caller 관계는 docs 목표에서 제외*. `@service` / `@subscriber` / `@publishes` 는 객체가 스스로 박는 local declaration (PC 어디 떠 있든 무관) = 객체 contract layer. 두 layer 섞지 말 것.
11. **cost-based 추천 (메모리 `feedback_no_cheap_argument` 위반)** (2026-06-24) — cache 변환 옵션 추천 시 "코드 변경 최소" / "framework 변경 zero" 근거로 박음. 사용자 정정: "적용 비용 기준 빼고 설계적으로만 보면 답 바뀜" — 메모리 박혀 있는데도 reflex 적으로 cost 박음. 정석 / 원칙 / 일관성으로 평가할 것.
12. **자체 분석 안 박고 카탈로그 / 옵션 / "어때?" 패턴 반복** (2026-06-24) — 사용자 명시 지적: "너는 너가 생각 안 해? 왜 이렇게 분석 안 해?" 검토 phase 진행 중 거의 매 turn 옵션 나열 + 의견 물음 패턴. push 두려움 + 자체 틀림 회피 = 안전 자세. 검토 phase 의미 약화 — 자체 입장 박은 후 사용자가 평가해야 검토 의미 있음. 단 입장 박을 때 *근거* 박혀 있어야 (cost 같은 메모리 위반 근거 X).
13. **Cache motivation 잘못 진단 (ROS-think reflex)** (2026-06-24) — JointStateCache 박을 때 "중앙 상태 저장소 자체 자체 모든 노드 공유" motivation 박힘. 사용자 reframe: Zenoh pub/sub 에서 *여러 노드가 여러 번 구독 정상*. cache 진짜 motivation = (a) boilerplate 줄임 + (b) 변환 wrapper. *상태 공유* 아님 (cache 도 process-local, distributed 면 PC 별 별개). singleton 자체 = cost saving 만, correctness 아님. FrameCache 는 다름 — *JPEG decode dedup* 진짜 가치 (단 현재 raw bytes 보유라 미실현).
14. **노드 unique 책임 없음 — grouping convention** (2026-06-24) — "노드 왜 필요?" 질문에 처음 답 = deployment / identity / lifecycle / heartbeat / thread 호스트 — 현재 코드 정당화 답. 사용자 reframe: zero-base 면 process / robot / component (Handler/Cache/Worker/Adapter) 가 진짜 unit, 노드는 ROS mental model 유산 = grouping convention. framework 강제 X.
15. **cost-based reflex 재발 — "이미 개발했음"** (2026-06-24) — Component 분리 박은 후 두 번째 답에서 "분산 + 이미 개발된 코드 + 노드 mental model 자연" 박음. 사용자 정정: "노드 패턴 많이 개발함 이런건 빼고 — 진짜 합리적이면 처음부터 다시 짤 거". cost-based 근거 (메모리 위반) 또 박힘. 정석 / 원칙으로 평가.
16. **운영 X = 리라이트 cost 작음** (2026-06-24) — 사용자 명확화: 운영 단계 아님. 사용자 없음. 배포 안 함. 즉 "절대 갈아엎지 마라" 계열 조언 해당 X. framework cost << future maintenance cost 시점. 단 *바로 전체 삭제 X*, backend_v2 실험실 박은 후 판정.

17. **Node = 잘못된 전제** (2026-06-25) — §14 까지의 plan 이 "Node 라는 실행 단위가 있다" 전제 위에 서 있음. 진짜 깨달음 = Runtime (Process) 이 최소 단위, Node 가 Runtime 책임을 자기 이름에 가졌던 abstraction. Module + Runtime 분리가 진짜 reframe. §15 참조.

18. **"Local 호출처럼 보이게" 표현 잘못** (2026-06-25) — GPT 와 토론 중 framework 책임 표현이 "service 호출은 로컬 호출처럼 보이게" 였는데 사용자 정정: 핵심은 *통신 계약 (service / topic) 은 동일, transport (local memory / Zenoh) 만 바뀐다*. local memory path 도 transport 의 한 종류.

19. **`.` vs `/` — key path 형태** (2026-06-25) — 표현이 `storage.commit` 같은 객체 메서드 호출이었는데 사용자 정정: 통신 계약 이름은 path (`/storage/commit`). 이미 v2 framework MVP 가 `/` 사용 — 변경 없음.

20. **backend_v2/ 폴더 폐기** (2026-06-25) — 사용자가 폴더 삭제. 이유 = §14 의 Phase 1 산출물 (framework MVP + 7 test PASS) 이 Node 잘못된 전제 위에 서 있는 코드. §15 reframe 위에 다시 짬.

**메타 학습** — *코드 보자마자 메타 질문 던질 것* + *짜기 전 hand-simulate + verification path 사전 검증* + *FastAPI / 다른 framework 차용 시 우리 use case 정당화 박을 것* (cargo cult 회피) + *진단 박기 전 실제 코드 grep* + *데코 박을 자리 — 계약(framework 호출) vs 실행 흐름(객체 호출) 판단 + 검증 가능성 판단* + *카탈로그 / 옵션만 던지지 말고 자체 입장 + 근거 박을 것* + *cost-based reflex 재발 주의 (메모리 박혀 있어도 두 번 박음)* + *Zenoh pub/sub 본질 = 여러 구독 OK, ROS-think (중앙 상태 model) reflex 차단*.

## 14. backend_v2 — zero-base 실험실 (2026-06-24)

### 14.1 동기

§7.5 reframe — Phase A 까지 박은 후 발견:
1. Contract First + Binder 모델 = 거의 확정 (`@service` / `@subscriber` / `@publishes` → ContractSpec → Binder)
2. BaseNode = 계약 / 실행 / 배포 / lifecycle 결합. 분리 필요.
3. 일반 객체 (BaseNode 비상속) 도 framework 가 attach 가능해야 — 이미 backend/ 에 capability 박힘.
4. Transport (Zenoh) / Framework (Horibot) 분리 필요.

→ 갈아엎을 만한 확신 박힘. 단 *Node 완전 삭제 + 4분류* 는 가설 — 코드로 검증.

### 14.2 plan

운영 단계 아님 (사용자 / 배포 / production 자체 없음). 리라이트 cost 작음.

```
1. backend/ 의 framework 변경 (Phase A 산출물) 그대로 유지 — 회귀 0
2. backend_v2/ 새 폴더 zero-base 실험실 박음
3. backend_v2 framework 자체 박음 (Contract First + Binder + Transport/Framework 분리)
4. 첫 component 1개 박아서 검증
5. 며칠 사용 후 4 질문 판정:
   - 개발 더 빨라졌나?
   - 테스트 쉬워졌나?
   - 문서 생성 쉬워졌나?
   - 새 컴포넌트 머리 덜 아픈가?
6. YES 3-4개 면 → backend/ 의 도메인 logic (캘 BA / motion / task DSL / scan / detector / scene3d / reconstruction / storage 등) 다 backend_v2 의 component 로 옮겨심음
7. backend_v2 가 backend/ 의 모든 기능 가지면 → backend/ discard
```

### 14.3 규칙 (실패한 리라이트 방지)

많은 리라이트 실패 패턴:
```
v1 멈춤 → v2 시작 → 기능 부족 → 계속 추가 → 6개월 후 둘 다 망함
```

차단:
- **규칙 1**: backend/ 자체 계속 개발 가능 (캘 / motion / scan 등). 단 framework 부분 (BaseNode / 노드 hierarchy) 자체 자체 자체 *추가 변경 X* — Phase A 까지가 끝.
- **규칙 2**: backend_v2 자체 자체 *기능 개발 금지*. 오직 framework 검증. 첫 component 자체 자체 framework 가 진짜 동작하는지 검증용.
- **규칙 3**: 실제 hardware 자체 자체 1 robot (omx_f_0) 만 붙여보기. 설계는 종이에선 다 좋아 보임 — 실제 붙여봐야 Worker / Handler 경계 자체 자체 검증.
- **규칙 4**: backend/ 자체 자체 자체 자체 *코드 reference* 박을 수 있음 — 캘 BA / Ruckig / IRLS / ChArUco / step DSL 등은 자산. *재구성* 자체 자체 자체 — 그저 framework 모양 자체 다름.

### 14.4 폴더 구조 (잠정)

```
backend_v2/
  transport/
    session.py          — ZenohSession (process singleton)
  contract/
    subscriber.py       — @subscriber + SubscriberSpec
    service.py          — @service + ServiceSpec
    publishes.py        — @publishes + PublishesSpec
  binding/
    bind.py             — bind_decorated(obj, session, robot_id, ...)
  components/
    joint_state_read.py — process-singleton Cache 자체 (read model)
    dynamixel_adapter.py — hardware Adapter 자체
    motion_worker.py     — running thread Worker 자체
    calibration_handler.py — stateless Handler 자체
  main.py               — orchestrator (host config → component list → instantiate + start)
```

### 14.5 Component 분류 (가설 — 코드로 검증)

| 종류 | 책임 | lifecycle | state | robot scope |
|---|---|---|---|---|
| **Handler** | service handler 묶음 | 없음 (attach 시점만) | stateless | constructor 인자 |
| **Cache (read model)** | state holder | self-bind in `__init__` | process singleton | 모든 robot (wildcard) |
| **Worker** | thread / state machine | start/stop | 자체 보유 | constructor 인자 |
| **Adapter** | hardware driver wrapper | start/stop | hardware resource | constructor 인자 |

검증 포인트 — *실제 component 박을 때 경계 명확한가?* 예: `CalibrationWorker` 가 service 도 받고 state 도 들고 background 작업도 함 — Handler 인가 Worker 인가 애매. 이게 가설 시험.

### 14.6 Architecture detail

**4 Layer 분리**:
```
Application (Task DSL / Recipe / Step)
       ↓
Components (Handler / Cache / Worker / Adapter)
       ↓
Framework (Contract + Binding)
       ↓
Transport (ZenohSession + Pydantic schema + Key registry)
```

**Layer 별 책임**:

| Layer | 책임 | 폴더 |
|---|---|---|
| Transport | wire + serialize | `transport/` |
| Framework | 데코 + binding helper | `contract/` + `binding/` |
| Component | 객체 책임 (4 종) | `components/` |
| Application | 도메인 logic | `tasks/` + `recipes/` |

**Framework contract (1급 계약 3개)**:
- `@service(key)` — RPC handler
- `@subscriber(key)` — topic sub (signature 기반 codec + robot_id inject)
- `@publishes(key)` — mark only (docs + AST lint, 실제 publish 는 helper 호출)
- `bind_decorated(obj, session, robot_id, ...)` — 일반 helper. Node / Handler / Cache / Worker / Adapter 다 동일 호출.

**Identity model**:
- process: `process_id` (host config)
- robot: `robot_id` (constructor 인자)
- component: `cls.__name__` + optional `robot_id`
- BaseNode 같은 class hierarchy identity 없음 — 그저 plain class.

**Process orchestration**:
- `main.py` 가 host config 읽음
- host config = `process_id` + `transport` + `components` list (cls / robots)
- main 이 import → instantiate → start. lifecycle = component 단위.

**Heartbeat**:
- `ProcessHeartbeat` Worker (process-level 하나)
- payload: `process_id` + active component list (cls / robot_id)
- 노드별 heartbeat 폐기. visibility 는 component list 로 충분.

**Publish API**:
- `from framework import publish` — `publish(key, msg)` stateless helper
- BaseModel / bytes / dict 자동 codec 판단
- `@publishes` 데코 = 마킹만 (docs / lint)
- AST lint 가 데코 vs 실제 `publish(Topic.X, ...)` 호출 일치 검증 → stale 차단

**Data flow (subscribe)**:
```
Publisher → Zenoh router → bind_decorated callback
  → key_expr 에서 robot_id 추출 (template parse)
  → payload codec (Pydantic.model_validate_json / bytes)
  → component method 호출 (robot_id inject 여부는 signature 기반)
```

**Data flow (service)**:
```
call_service → Zenoh queryable → bind_decorated handler
  → req: ServiceRequest[X] model_validate_json
  → component method
  → res: ServiceResponse[Y] model_dump_json reply
```

**Data flow (publish)**:
```
component method → framework.publish(key, msg)
  → ZenohSession.put(key, encoded)
```

**핵심 결정**:
- **BaseNode 폐기**. 모든 객체 plain class. framework 가 wire.
- **Component 4분류 = 책임 분리 가설** — 가설 검증 위해 4 종 다 박아봄.
- **`@publishes` = mark only** — 실제 publish 는 `framework.publish()` helper.
- **Identity = process_id + robot_id** 두 축. component 식별 = cls name + optional robot.
- **Heartbeat = process 1개** — active components list 박음. node-level 자체 폐기.

### 14.7 폐기될 backend/ 의 framework 자산

backend_v2 자체 자체 완성 박힌 후 폐기:
- `core/transport/base_node.py` 자체 `BaseNode` / `start()` / `attach_handler` / `r()` 자체
- `core/transport/application_node.py` / `device_node.py` — 2-layer 분류 자체 자체 (`isinstance(cls, DeviceNode)` 자체 자체 main.py 검증)
- `core/transport/node_registry.py` — lazy-import factory
- `core/cache/joint_state_cache.py` (Phase A 변환된 모양 — backend_v2 자체 자체 자체 재구성)
- `core/cache/frame_cache.py` (Phase A 미적용 — backend_v2 자체 자체 자체 재구성)
- `framework/` 폴더 자체 자체 — backend_v2/contract / backend_v2/binding 자체 재구성

옮겨심을 도메인 logic (재배치, 폐기 X):
- 캘 BA / IRLS / Huber / observability / strategy / ChArUco / capture_quality
- Motion command / TrajectoryRunner / Ruckig / Jog 적분 SE(3) / IK
- Task DSL / Step / Slot / TaskRunner / Recipe / 정규 task (pick_and_place / scan)
- Detector / YOLO / Grounding DINO / search_and_detect
- Scene3D / depth_frame / consensus / pointcloud streaming
- Reconstruction / ICP / PoseGraph / TSDF / mesh extract
- Storage 자체 (RDB / ObjectStore Protocol / Alembic migration / 캘 5종 / scan workflow)
- Bridge (WebSocket + MJPEG + binary framing)
- Kinematics (Pybullet + SagCorrected + link_offset patch)
- Coordinates (Joint / Link / Sag)
- Gamepad / 8BitDo mapper
- Robot Registry (robots.yaml + RobotConfig + factory)

### 14.8 검증 후 시나리오

**Case A — backend_v2 좋음 (예상)**:
- 도메인 logic 다 backend_v2 component 로 옮겨심음 (1-2달 자체)
- backend/ discard

**Case B — backend_v2 별로 (백업)**:
- backend_v2 폐기
- backend/ 그대로 + framework 부분 강화 (Phase B/C 자체 자체 backend/ 안 자체 자체)

### 14.9 폐기 alert (2026-06-25)

§14 전체 plan 이 **잘못된 전제** 위에 서 있음 — Node 가 최소 단위 가정. 진짜 깨달음 = **Runtime (Process) 이 최소 단위, Module = 기능 묶음** (§15 참조). backend_v2/ 폴더 (Phase 1 MVP 산출물 + 7 test PASS) 를 사용자가 삭제 (2026-06-25). §15 reframe 위에 다시 짬.

§14.5 의 4분류 가설 (Handler / Cache / Worker / Adapter) 은 §15 에서 *Module 의 유형 힌트* 로 위치 변경 — framework 는 종류 모름 (duck typing).

§14.6 의 Architecture detail (BaseNode 폐기 / Component 4분류 / Heartbeat 1개 등) 은 §15 의 Contract / Runtime / Transport 3 layer 안 흡수되거나 재배치.

## 15. Runtime-centric Reframe (2026-06-25)

### 15.1 잘못된 전제 발견

§14 까지의 plan 의 전제 = "Node 라는 실행 단위가 있고, BaseNode 가 그 실행 단위의 공통 기능을 제공해야 한다". 이 전제 위에서 자연스럽게:

- BaseNode 가 bind 관리
- BaseNode 가 decorator 수집
- BaseNode 가 lifecycle 관리
- Node 가 서비스/토픽 제공자
- Node 를 없애면 싱글톤은? 실행 단위는?

모두 *Node 가 근본 개념* 가정 위에 서 있는 질문.

진짜 물어야 할 질문 = **"이 시스템에서 배포/실행의 최소 단위가 무엇인가?"**

답 = **Process / Runtime / Deployment Unit**. Node 가 *Runtime 의 책임을 자기 이름에 가졌던 잘못된 abstraction*. Module 과 Runtime 이 한 클래스에 묶여 있었음.

### 15.2 새 사고

```
Runtime (Process)
 |
 +-- Module (Service Provider)
 |
 +-- Module (Service Provider)
 |
 +-- Module (Subscriber)
```

- **Module** = 기능 묶음. `@service` / `@subscriber` / `@publishes` 가진 함수 보유. plain class.
- **Runtime** = 실행 컨테이너. lifecycle / transport 연결 / registry / DI / shutdown / thread 관리.

"Node 삭제" 의 진짜 의미 = **기능 제공자 (Module) 와 실행 컨테이너 (Runtime) 분리**.

### 15.3 Framework 핵심 책임

**"같은 코드가 어디 배치되든 그대로 동작하게 만들기"**

개발자가 절대 신경 쓰지 않아야 함:

- 같은 process 냐?
- 다른 process 냐?
- 다른 장비냐?
- Zenoh 쓰냐?

모두 framework 내부 결정. 한 줄 요약: **"distribution is not a code concern, it is a runtime concern"**.

### 15.4 Contract / Runtime / Transport 분리

```
Contract (계약 — @service / @subscriber / @publishes)
    ↓
Runtime (Module 등록 + lifecycle + Transport 선택)
    ↓
Transport (계약을 만족시키는 매체)
    ├── Local memory (같은 process)
    └── Zenoh (다른 process / 다른 장비)
```

핵심:

- **Service / Topic = 통신 추상화 계약**, key 는 path 형태 (`/storage/commit`, `/camera/frame`)
- **Zenoh = 그 계약을 만족시키는 외부 transport**
- **Local memory path 도 transport 의 한 종류** — 같은 process 의 provider 는 direct dispatch (serialize 없음)
- 어느 transport 쓸지 = **Runtime 의 결정** (provider 위치 resolver)

### 15.5 4분류 (Handler / Cache / Worker / Adapter) 의 위치

§14.5 의 4분류 가설은 **Module 의 유형 힌트** 로 위치 이동. framework 자체는 Module 종류 모름 (duck typing) — Lifecycle protocol (`start()` / `stop()`) 만 호출. 4분류는 *사용자 mental model* + *system-docs viewer 의 분류* 도구.

### 15.6 v2 framework MVP 의 현재 상태

| Layer | 상태 |
|---|---|
| Contract (`@service` / `@subscriber` / `@publishes`) | ✅ 구현됨 (backend_v2/ 박혔던 것, 폴더 삭제) |
| Module direct wire 등록 (`bind_decorated`) | ✅ 구현됨 |
| **Transport abstraction** (local vs Zenoh) | ❌ Zenoh hardcoded |
| **Runtime resolver** (provider 위치) | ❌ |
| **Lifecycle protocol** (start/stop) | ❌ |
| **DI / config** | ❌ |

진짜 reframe 핵심 — `bind_decorated` 가 지금 Zenoh 직접 호출. 같은 process call 도 Zenoh 통과 = wire serialize/deserialize. 이게 *distribution is runtime concern* 원칙 위반.

backend_v2/ 폴더 (2026-06-25 사용자 삭제) — 위 ✅ 두 layer 가 §15 reframe 위에 다시 짜짐 (모양은 거의 동일, transport 만 추상화).

### 15.7 다음 step

step 후보:

1. **Transport interface** — `class Transport(Protocol)`:
   - `call(key, req) → res`
   - `publish(key, msg)`
   - `subscribe(key, cb)`
   - `register_service(key, handler)`
2. **ZenohTransport** — 기존 v2 binding 의 동작 wrapping
3. **LocalTransport** — process-local registry (key → handler dict), direct dispatch, serialize 없음
4. **bind_decorated** → Transport 호출 (Zenoh 직접 X)
5. **Runtime** — config 받은 후 Module instantiate + transport 선택 (provider 위치 resolver)
6. test — 같은 7 case 가 ZenohTransport + LocalTransport 둘 다 PASS

진짜 첫 step = Transport abstraction. 그 위에 Runtime / Lifecycle / DI 쌓는다.

### 15.8 motion 영역에서 검증된 마찰점

§14 reframe 이후 backend/ motion_node 의 design decision 8 개를 problem statement 관점에서 추출 (2026-06-25). v2 의 Module + Runtime 분리 모델이 그 마찰을 어떻게 푸는지 검증:

1. **같은 도메인 4 entrypoint (Move/Servo/Jog/Task) × N 비즈니스 함수 = N×4 wrapper boilerplate** — backend/ 의 `_make_jog_j_topic_subscriber` / `_make_jog_j_service_handler` 등 8+ wrapper factory.
   → v2 풀이 = 한 method 에 데코 여러 개 (`@service + @subscriber`). signature 는 비즈니스 데이터만 (envelope X, error 는 raise). framework 가 entrypoint shape 변환. wrapper 8+ → 0 줄.

2. **Callback 순서 보장 X** — MOTOR_STATE_JOINT 를 JointStateCache + `_on_motor_state_publish_tcp` 둘 다 subscribe, 순서 보장 안 됨 → motion_node 가 cache 무시하고 직접 raw parse.
   → v2 풀이 = derived read model Module (`TcpStateRead` — JointState 받음 → FK → MotionTcpState publish). motion command handler 와 분리.

3. **한 클래스에 두 책임** — MotionNode = motion command handler + derived state publisher.
   → v2 풀이 = Module 분리.

4. **Service envelope vs topic raw 두 unwrap** — `req: ServiceRequest[JogJReq]` 와 `req: JogJReq` 두 signature.
   → v2 풀이 = framework 가 envelope 흡수. handler signature 는 둘 다 raw 비즈니스.

5. **JointStateCache.subscribe(node, robot_id) 패턴** — singleton 인데 어느 robot 받을지 모름.
   → v2 풀이 = 이미 해결 (wildcard subscribe + robot_id callback inject).

6. **self.r(template) 매 호출 명시** — service 등록 8 줄 + topic 등록 2 줄 + publish 3 줄 전부 명시.
   → v2 풀이 = framework 자동 (이미 부분 해결).

7. **100Hz publish boilerplate** — `self.publish(self.r(Topic.X), MotorCmd(...))`.
   → v2 풀이 = stateless `publish(Topic.X, msg, robot_id=...)` (이미 해결).

8. **Cross-process calibration apply** — start() 안 storage fetch + 자기 process 객체 mutate.
   → v2 풀이 = Module lifecycle hook `start()` 그대로 OR `CalibrationApplier` Module.

가장 큰 검증 — **MotionNode 가 한 일 = framework + Module 사이에 끼어있던 wrapper layer**. framework 가 두꺼워지고 Module 이 직접 wire 등록하면 Node 자체가 할 일 없음. §14 의 "Node 삭제" 결론을 코드로 재확인 → §15 Runtime-centric reframe 도달.

### 15.9 새 anchor

새 세션 진입 시:

1. 본 §15 anchor
2. §14 = history (잘못된 전제 위 plan — 폐기)
3. §13 결정 history 17~20 — Node 잘못된 전제 / "Local 호출처럼" 표현 잘못 / `.` vs `/` / backend_v2/ 폐기
4. backend_v2/ 폴더 없음 — 새로 시작

사용자가 "Runtime" / "Module" / "Transport adapter" / "Contract layer" / "distribution is runtime concern" 톤 던지면 §15 진입. "backend_v2" / "Component 4분류" / "Node 삭제" 톤은 §15 reframe 안내 (§14 history).

default plan = Case A. Case B 는 만약 4 질문 결과 NO 가 많을 때 backup.
