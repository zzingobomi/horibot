# PnP 시나리오 재설계 — 논의 진행 문서 (2026-07-21 시작)

> **성격: 세션 간 논의 이어가기용 작업 문서** (사용자 지시로 신설). 결정이
> 확정되면 내용을 도메인 문서([task.md](task.md) / [motion.md](motion.md) /
> [perception.md](perception.md))에 § 로 병합하고 본 문서는 정리한다.
>
> **다음 세션 진입법**: 이 문서를 읽으면 2026-07-21 세션을 그대로 이어받는다.
> **§8 = 07-21 밤 홈 테스트 판정 + 2차 구현 (최신)** — §2.4 가방 가설은 **기각**
> 됐고(§8.1), 진범 = 인접 pair 중첩부의 ICP 앵커 부재. §7 은 1차 구현 기록.
> 미착수 잔존은 §5-5(world 스캔 패널 상세)와 §8.4 남은 것.

---

## §1. 배경 — 어디까지 왔나

- **07-19 (일)**: closed-loop PnP 연속 4런 성공 (위치 스왑·재플랜·재시도 생존
  체인 실물 검증). 스윕 통합 + 빌드 백그라운드화로 45.6s. → backend.md handoff.
- **07-20 (월) 밤**: IK·후보생성 대수술 (EAIK 해석적 IK + polish, 절대 yaw
  0..180° 15° 격자 = 가족 52→312, width 물리 게이트) — [motion.md](motion.md) §11.
- **07-21 (화) 새벽**: 수술 후 첫 실물 2런 — **01:29 성공 / 01:31 파지 실패**
  (+ 01:28 첫 시도는 plan 전멸로 시작도 못 함). 이 세션에서 실패 원인 분석.

## §2. 분석 결과 (2026-07-21 세션 — 증거 기반, 재검증 가능)

### §2.1 파지 실패 (01:31 런) — 유력 원인: IK 잔차 큰 한계 자세 채택

trace: `backend/debug/servo_pick/20260721_013146/` (실패) vs `20260721_012921/` (성공).

- **증상**: 서보는 수렴(lateral 1.6~2.8mm, midstop 재앵커 잔차 ≤8.6mm,
  touchup 2.8mm)했는데 close 2회 모두 빈손 (도달 raw 1941 ≈ 완전닫힘 1935,
  부하 ~50). 1차 close 가 큐브를 **조 축 +방향으로 ~18mm 밀며 82°→−49° 회전**
  (재시도 재획득 관측으로 역산) = 조가 스트래들 못 하고 쓸어냄/튕김 —
  07-17 "close 튕김" 클래스.
- **결정적 로그** (`backend/logs/horibot-2026-07-21.log` 673-674행 vs 429-430행):
  - 실패 런 채택 그룹(jaw@171° tilt+30): **IK 위치잔차 max = 7.35mm**
  - 성공 런 채택 그룹(jaw@81° tilt+30): **0.16mm** (46배 차)
- **기전**: polish 가 3mm 목표에 못 붙는 한계 자세(리밋 경계/특이점 근처)를
  10mm 게이트(`IK_POS_ERROR_LIMIT`, pybullet.py)가 통과시킴. 이 잔차는 서보
  카메라 루프가 마지막으로 확인한 **이후의 모든 이동 명령에 재주입**되므로
  (보정 이동도 같은 IK 플래토를 통과) close 직전 blind 구간에서 못 잡는다.
  21mm 둥근 큐브의 편측 여유 수 mm 를 계통 오차 7mm 가 삼킴.
- **방증**: 같은 가족이 구성 플립 경계 — 재시도 중 보정 MoveL 2회 기각
  ("시작 1.0cm 지점 관절 도약 38° — 구성 플립").
- **반증된 가설**: "높은 J6 손목 구성이 문제" — 과거 성공 런에도 J6 0.2~2.9 rad
  전역 분포 (전 런 J5/J6 스캔으로 확인). 자세 클래스가 아니라 **잔차**가 차별자.
- **확정까지 남은 것**: 잔차 큰 가족을 일부러 실행해 miss 재현 (또는 실패 영상).
  판정 신호는 이미 로그에 있음 (resolve_reachable 채택 잔차 라인).

### §2.2 EAIK / URDF 적용 — 알리바이 확인 (무죄)

이 세션에서 실사용 캘 URDF(`robot/so101_6dof/urdf/so101_6dof.so101_6dof_0.calibrated.urdf`)로 직접 검증:

- 축 추출 정확: J1→−z(0.31°) J2→+y(1.58°) J3→+y(1.45°) J4→+y(2.30°)
  J5→−z(1.47°) J6→−x(0.26°). snap 최대 2.30° < 5° 게이트.
  family=6R-THREE_INNER_PARALLEL.
- 왕복 500회 (나쁜 seed=zero, 자가충돌 표본 제외): 실패 0.8%,
  자세 오차 max 0.26°, 위치 오차 mean 2.7 / p95 7.4 / **max 9.9mm** (10mm
  게이트가 상한을 만듦 — §2.1 의 잔차 문제와 동일 뿌리).
- 구조적으로도 EAIK 는 오답을 몰래 실행할 수 없음: 해석해 = seed 일 뿐, 채택은
  캘 모델 FK 재검증 (위치 10mm + 자세 5° + 충돌) 뒤에만.
- 개념 정리: **해석해 = snap 이상화 모델의 정답** (완전성 담당). 실 모델과의
  간극(수 mm)은 언제나 수치 polish 가 메움 — polish 가 어려운 자세에서 3mm 에
  못 붙는 것(§2.1)은 수술 전부터 있던 수치 IK 의 약점 + 동일 게이트.

### §2.3 IK 시간 — 그룹당 6배↑, 격자 6배 확대가 흡수 (벽시계 비슷)

pi_hori1 동일 머신 before/after (`resolve_reachable` 로그):

| 시점 | 상황 | 그룹 | 시간 |
|---|---|---|---|
| 07-20 22:00 (수치) | 전멸 | 52 | 27.35s (0.53s/그룹) |
| 07-21 01:28 (해석) | 전멸 | 312 | 25.85s (**0.083s/그룹**) |
| 07-20 21:58 (수치) | 채택 | 28 | 9.38s |
| 07-21 01:29 (해석) | 채택 | 28 | **1.30s** |

- 스피드업이 지연 단축이 아니라 커버리지(전 footprint 커버)에 재투자된 구조.
- 전멸 사유 히스토그램 관측성은 약속대로 동작 ("위치 통과 280/312, 자세 IK
  실패 280").
- pi 부팅 로그 `IK=해석적(6R-THREE_INNER_PARALLEL)+polish` 확인 (01:26,
  uv sync 완료 상태).

### §2.4 월드 스캔 메시 품질 붕괴 — ~~근본: 검은 가방의 시야 점유~~ **기각 (07-21 밤, §8.1)**

> ⚠ **이 절의 결론은 틀렸다.** 07-21 밤 가방 없는 씬에서도 동일 붕괴 재현 —
> 진범 = **인접 pair 중첩부의 ICP 앵커 부재** (§8.1, A/B 재빌드로 확정).
> 아래 원문은 "그럴듯한 가설이 물증 사진 몇 장으로 '사실상 확정'까지 갔던"
> 기록으로 보존 — 판정 실험(§4-1) 없이 확정 딱지를 붙이지 말 것.

- **회귀 창**: 07-19 23:47 빌드 건강 (인접 pair 전부 fitness 0.5~0.79, 보정
  4~11mm) → 07-20 21:58 부터 scan 4→3 붕괴 (fitness 0.36, **39mm 보정 채택**)
  → 07-21 01:27 발산 판정 → FK 배치 (그 자리 이중 벽/고스트).
- **물증**: 같은 search pose 5·6 의 컬러 사진 3일치 —
  `backend/debug/detect/20260719_232438/0005·0006_det_blue_box_color.png` (가방
  없음, 페그보드·전원공급기·공유기 보임) vs `20260720_215634/`, `20260721_012544/`
  (검은 가방이 시야 절반 점유, 배경 가림). fitness 붕괴 시점 = 가방 프레임
  진입 시점.
- **배제 완료**: search waypoint 6자세 불변(07-17 생성, DB 확인) / 활성 캘
  불변(07-15 이후, DB 확인) / scan·camera 코드 커밋 없음(git) / 조명 아님
  (07-19 밤 빌드 건강) / IK 수술 무관 (스캔 자세 = waypoint joint 값 MoveJ,
  IK 안 탐).
- **기전**: 검은 패브릭(D405 depth 최악 소재) + colored ICP 앵커였던 특징 풍부
  정적 배경(페그보드 구멍 격자·장비) 가림.
- **맥락 (사용자)**: 가방은 **흰 공유기가 "white cube" 프롬프트에 오검출되는
  문제의 물리 땜빵**이었음 — 한 결함의 땜빵이 다른 결함을 만든 케이스.
- **오검출 물증** (detect json): 07-19 `(+0.094,−0.235,z=−0.029)` footprint
  116×100mm score 0.55 (공유기), `(+0.151,+0.106,z=+0.201)` score 0.68 (로봇
  자기 몸통 추정). 07-20 가림 후에도 `(+0.087,−0.314,z=−0.043)` 104×**38mm**
  score 0.33 누출 — 폭 게이트(65mm)를 좁은 변이 통과할 수 있음. **공통점은
  색이 아니라 위치 = 전부 작업 셀 밖.**

## §3. 합의된 설계 방향 (2026-07-21 사용자 합의 — 구현 전)

### §3.1 world 스캔을 task 에서 분리 (전용 패널)

- 근거: world 메시는 task 내 **소비자 0** ([world.py](../backend/modules/tasks/pick_and_place/steps/world.py)
  docstring — "3D 뷰 배경 갱신이지 pick 의 일부가 아니다", best-effort).
  표시용 workcell 자원이 스윕에 우연히 편승해 있었을 뿐.
- 이득: ① capture 1.2~1.6s×6 = 스윕 크리티컬 패스에서 **7~9s 절약** (45.6s 의
  15~20%) ② 검출용 자세가 아닌 **정합 목적 전용 자세** (J1 조밀, settle 넉넉,
  loop closure 설계 — 좋던 날에도 loop 1/3~4/10 발산이던 만성 문제 해소 여지)
  ③ UX: 스캔→확인→재스캔 명시 루프 + staleness 표시 (이번에 사용자가 품질
  붕괴에 손쓸 방법이 없었음).
- 구현 방침: 전용 waypoint 그룹 (search 재사용 금지), 기존 scan 모듈 계약
  재사용, task 쪽 `RunRequest.build_world` + `WorldScan` 편승 클래스 **삭제**
  (반쪽 분리 금지). 케이던스 근거 = 셀 모델링은 셋업 타임 작업 (MoveIt 정적
  planning scene 관례와 동형).

### §3.2 시나리오 재구조 — "찾기는 위치만, 파지 계획은 근접에서"

사용자 제안 = 업계 표준 coarse-to-fine 과 일치 (bin-picking 셀 관례,
eye-in-hand 공통오차 상쇄 방향과도 일치):

1. **찾기 (coarse)**: 스윕 = XY 위치 파악만. 파지 계획 0, 가족 IK 게이트 0.
2. **접근 (pre-grasp 스테이션)**: 물체 위 관측 자세로 이동 ("잡는 자세"가
   아니라 "보는 자세" — 도달성 널널).
3. **근접 계획 + 실행**: mm 급 관측에서 후보 열거 — 면 정렬 yaw 2 × tilt
   사다리 × flip ≈ **12개**. 해석적 IK 로 즉석 판정 (수십 ms — EAIK 의 진짜
   값어치가 여기). 이후 기존 servo 하강/재앵커/close 기계 재사용.

**"가족 폐기"가 아니라 "가족 이사"** — 후보 열거 + IK/충돌 게이트 구조 자체는
표준 (MoveIt Grasps / Dex-Net / GPD 동형). 죽는 것: 스윕 시점 파지 계획,
312 헤지 격자 (±40mm 관측을 못 믿어 생긴 헤지 — search.py docstring 실측),
312×사다리 사전 resolve. 남는 것: 후보 열거(근접, 소수), servo 기계, 해석 IK.

정직한 트레이드오프: ① 워크스페이스 가장자리 도달성 문제는 관측 품질과 무관하게
잔존 (다만 명중률↑ + 전멸 판정 ms) ② "접근 관측 자세 생성 규칙"이라는 새 설계물
필요 — 현 servo rung 0 (standoff 8cm) 이 사실상 그 자리라 큰 발명은 아님.

### §3.3 workcell 경계 (SSOT 1곳 → 소비자 2곳) + 스캔 게이트

- **detector**: base-frame 투영 후 셀 밖 후보 컷 → 공유기(z<0)·로봇몸통(z=0.2)
  오검출 소멸 → **가방 치울 수 있음** → 스캔 정합 앵커(페그보드) 복귀.
- **scan TSDF 통합**: 셀 ROI 컷 → 메시에 침구/바닥/가방 안 들어감.
- **정합(pairwise ICP) 입력은 전체 유지** (중요 — 사용자 질문으로 확정):
  페그보드 등 원거리 특징이 회전 관측성 앵커 + 셀만 남기면 평면 지배 장면의
  in-plane 퇴화로 회귀 (colored ICP 도입 이유가 그 문제였음). build.py 가
  정합/통합 2단계로 이미 분리돼 있어 통합에만 ROI 걸면 됨.
- 잔여 리스크 (정직): 정합은 여전히 배경 "변화"에 노출 — 재발 시 workcell
  fixtures 개념(선언된 정적 구조물 존) 승격. 지금은 과설계로 판단.
- **게이트 강화 후보**: "fitness 낮음(0.36) + 보정 큼(39mm)" 조합이 채택된
  07-20 케이스 — 오정합 채택은 FK 배치보다 나쁨. 기각 조건 강화 가치.

### §3.4 관측 시작 전 가동 조 열기 (스윕·스캔 공통)

- **현상**: 스윕/월드 캡처는 이전 런이 남긴 그리퍼 상태 그대로 시작 —
  `open_gripper` 첫 호출이 pick 접근 단계([pick.py:113](../backend/modules/tasks/pick_and_place/steps/pick.py#L113)).
  직전 런이 close 로 끝났으면 닫힌 조가 시야 하단 중앙을 가린 채 관측한다
  (§2.4 스윕 사진들에서 육안 확인 — 검은 조+파란 부품이 프레임 하단 점유).
- **방침 (사용자 합의)**: 시나리오 시작(스윕 전)과 스캔 세션 시작 시
  **가동 조 open**. 고정 조는 물리적으로 어쩔 수 없음.
- **구현 노트**: `_GRIPPER_SETTLE_S = 4.0s` — naive 하게 넣으면 런당 +4s.
  open 명령을 첫 MoveJ 와 겹치면 (이동 중 settle) 비용 ~0.
- **연관 별개 이슈 (기록만)**: 열어도 고정 조·마운트는 프레임에 잔존 — 그리퍼
  self-점이 스캔 점군·검출에 들어가는 문제는 별도 (z=+0.2 흰 후보 = 로봇 몸통
  오검출과 같은 클래스 — workcell 경계 §3.3 + robot self-filter 가 정석 자리).

## §4. 즉시 실행 가능한 판정 실험 (코드 변경 0)

1. **스캔**: 가방을 07-19 위치로 빼고 스캔 1회 → 로그 `pair 4→3` fitness 가
   0.6 대 복귀하면 §2.4 종결. (안 돌아오면 가설 기각, 다음 용의자.)
2. **파지**: 잔차 큰 가족(171° 류)을 같은 위치에서 강제 실행 → miss 방향
   재현되면 §2.1 확정. resolve_reachable 채택 잔차 로그가 판정 신호.

## §5. 미결 논점 (다음 세션 여기부터)

> **1~4 는 2026-07-21 이어서 구현 완료 → §7.** 5~6(world 스캔/스캔 게이트)만 잔존.

1. ✅ **구체 시나리오 흐름** (§7-3): 상자 관측→놓기계획 먼저, 물건 마지막 관측→
   파지계획→바로 집기. place 는 집기 독립(단순화, §7-4). 재탐색 루프는 미설계(잔존).
2. ✅ **접근 관측 자세 생성** (§7-2): coarse XY 위 look-pose(standoff 8cm),
   reachability 우선(수직 강제 X), servo 앞단.
3. ✅ **후보 스코어링** (§7-1): resolve 채택을 잔차 선호로 (겨우 닿는 자세 회피).
4. ✅ **잔차 게이트** (§7-1): 하드리밋(10mm)은 유지, 그 아래 "GOOD 2mm" 선호
   문턱 신설(채택 선호지 기각 아님).
5. **world 스캔 패널 상세**: 전용 waypoint 그룹 티칭 vs 자동 생성, staleness
   UX, ROI 설정의 SSOT 위치 (robot/ registry 쪽이 자연스러워 보임 — workcell
   4-entity 모델과 정합).
6. **스캔 게이트**: 저 fitness + 대보정 기각 조건 수치화 (07-20/21 로그가
   튜닝 데이터).

## §6. 증거 인덱스 (재검증용)

| 항목 | 위치 |
|---|---|
| 실패/성공 런 trace | `backend/debug/servo_pick/20260721_013146/`, `20260721_012921/` (summary.json + trace.jsonl) |
| 채택 잔차 로그 | `backend/logs/horibot-2026-07-21.log` — `resolve_reachable` 검색 (429/673행 대비) |
| IK before/after | `horibot-2026-07-20.log` 248-253, 478행 vs `horibot-2026-07-21.log` 258, 429-443행 |
| 스캔 fitness 타임라인 | 각 일자 로그에서 `pair \d` / `발산` 검색 (07-19 15:50·23:47 / 07-20 21:58 / 07-21 01:27) |
| 가방 사진 3일치 | `backend/debug/detect/20260719_232438/`, `20260720_215634/`, `20260721_012544/` 의 `0005·0006_det_blue_box_color.png` |
| 오검출 후보 | 같은 폴더 `*_det_white_small_round_cube.json` (z<0 / z=0.2 / footprint 대형) |
| EAIK 왕복 검증 | 세션 내 1회성 스크립트 (수치는 §2.2 에 박제) — 재현: 캘 URDF 로 fk→ik(quat, zero seed)→fk 왕복 500회 |

## §7. 구현 완료 (2026-07-21 이어서 — sim 초록, 실물 검증 대기)

§3 방향 + §5 논점 1~4 구현. **전부 mock/sim 로직만 증명** (fast 384 / full 467
초록, ruff·pyright clean). **커밋 안 함.** 실물 첫 런 데이터로 조율 전제.

### 무엇이 바뀌었나

1. **회귀 수정 (§5-3/4)** — `resolve_reachable` 채택을 "선호순 첫 통과" →
   "잔차 GOOD(2mm) 이하 첫 그룹, 없으면 잔차 최소" (`motion/module.py`
   `_pick_by_residual` + `_RESOLVE_RESIDUAL_GOOD_MM`). §2.1 헛집음(잔차 7.35mm
   자세 채택) 재발 방지. 하드리밋(10mm)은 kin 그대로 = 기각 아닌 **채택 선호**.
2. **접근·관측 (§5-2)** — 신규 `steps/approach.py::approach_observe`: coarse XY
   위 look-pose(standoff 8cm=카메라 14cm 최적대역, tilt 당 대표 1개 reachability
   우선, **수직 강제 X**)로 이동 → 정확 관측 → 그걸로 계획. 실패 시 coarse 폴백
   (경고 로그). **servo 는 안 건드림** (그 앞단만 = "가족 이사").
3. **시나리오 재배열 (§5-1)** — 상자 관측→놓기계획 **먼저**, 물건 관측→파지계획
   **마지막**, 그 뒤 이동 없이(계획=모션0) 바로 집기. 놓기·집기 도달성 둘 다
   집기 전 검증(쥔 채 멈춤 방지).
4. **놓기 단순화 (사용자 지시)** — `geometry._place_candidates`: 물건 폭/높이
   무시, **상자 정중앙 + 고정 여유높이**에 열기. `plan_place` 가 held/lateral
   의존 제거 → 집기와 독립. ⚠ **TODO(빡빡한 상자/큰·긴 물건)**: 폭(중심 보정)·
   높이(드롭 거리)·집힐 때 회전(in-hand 관측) 되살림 — geometry 주석 + git
   history 에 걷어낸 식.
5. **속도 (312→52)** — 가까이 정확 관측 성공 시 관측 yaw 를 믿어 **파지 yaw 격자
   끔** (`grasp_families(yaw_grid=False)` ← `plan_pick(trust_yaw=)` ← approach 의
   `close` 플래그). 전멸 CT 6배↓. 폴백(coarse)이면 격자 유지. 격자는 coarse yaw
   불신 헤지였으니 정확 관측이 그 근거를 없앤 것.

**최종 흐름:** 찾기(coarse) → 상자 관측→놓기계획 → 물건 관측→파지계획(52) →
servo 집기(불변) → 상자 정중앙 놓기.

### 집 테스트 체크리스트 (실물만 풀리는 미지수 — 로그·debug 로 전부 남김)

1. 관측 자세(8cm) 도달·프레이밍 — `approach_observe(...)` 로그(성공/폴백/위치).
2. 좁힌 52개 커버리지 — `가족 %d개 전멸` 시 격자면 잡혔을지 (집 데이터로만 판정).
3. 잔차 낮은 채택이 헛집음 줄이나 (§2.1 가설, 아직 n=2) — `채택 … 잔차` + servo trace.
4. 실제 속도 — resolve 시간 6배↓ 확인.
5. 놓기 — 정중앙+조금 위로 상자에 잘 들어가나 (빡빡하면 폭/높이 TODO 되살림).

### 튜닝 노브 (실물 데이터 후 — 증상만 보고 딴 fix 쌓지 말고 로그로 원인 판정 먼저)

- **yaw 좁힘**: `trust_yaw` 끄기(격자 복귀) / 면정렬 ±15° 만 넓히기 / 좁게-먼저-
  전멸시-격자 폴백. "얼마나 넓힐지" = 놓친 횟수 데이터로.
- **잔차 GOOD**: `_RESOLVE_RESIDUAL_GOOD_MM`(2mm) — 실측 잔차 분포로.
- **관측**: `approach._OBSERVE_STANDOFF_M`(8cm) / `_OBSERVE_FRAMES`(1, 노이즈면↑).
- **놓기**: `geometry._PLACE_DROP_CLEAR_M` / 폭·높이 반영 되살림.

## §8. 07-21 밤 홈 테스트 판정 + 2차 구현 (2026-07-21 밤 세션)

world_scan 패널 커밋(40b1f1d) 후 첫 실물: 스캔 mesh 붕괴(이중벽 유령) + 성장
점군 안 뜸 + 상자 close 관측이 측면뷰. 전부 데이터로 판정 후 수정.

### §8.1 스캔 붕괴 진범 확정 — **scan1 depth 버퍼 오염 (주범)** + 앵커 부재 (부범)

> 판정 사슬 (가설 3개가 순서대로 죽고 4번째가 물증으로 확정):
> 가방(§2.4) → 기각 (가방 없는 씬에서 재현) / depth 품질 → 기각 (유효율 오늘이
> 더 좋음 89~91%) / 코드·캘·URDF·ROI·신규 게이트 → **전부 기각** (A/B 재빌드:
> s22=07-19 데이터는 어느 조합으로도 건강, s29=오늘 데이터는 옛 파이프라인
> 그대로 돌려도 동일 붕괴 — 게이트/ROI 유무가 결과를 안 바꿈).

- **주범 (물증 확정)**: 스캔별 FK-only 분해 — **s29 scan1 만 책상이 z+5.6cm
  부유** (scan2~6 은 −0.01 일치, s22 는 6장 전부 0±5mm). scan1 모터 raw 는
  07-19 와 동일(±2 tick) = 로봇은 pose1 에 있었는데 depth 는 13cm 다른 장면
  (median 222.8 vs 354.5mm) = **캡처가 이동 중 프레임을 썼다**. 기전: scene3d
  snapshot 이 "최근 N 장"(과거 ring buffer)이라, 긴 MoveJ 직후 settle 1.0s <
  버퍼 시간폭이면 이동 중 depth 가 consensus 에 섞여 pose1 FK 로 배치 →
  6cm 이중층 + pair 1→0 정합 전멸(fitness 0.07~0.19). 두 world_scan 런(20:57
  ·22:05)에서 scan1 median 이 동일하게 222.8mm = 체계적.
  **07-19 이 건강했던 진짜 이유**: 옛 pick 편승 캡처는 같은 자세에서
  detect(GDINO 2~4s)가 끝난 뒤라 버퍼가 이미 씻겨 있었다 — 스캔 분리가 그
  우연한 대기를 제거하며 잠복 결함이 드러남.
- **부범 (잔존 실측)**: scan1 을 제쳐도 왼쪽 pair 들이 30~40mm in-plane 슬라이드
  (fitness 0.32~0.38) — J1 팬 중첩부는 수평 평면이 점의 99%라 in-plane 앵커가
  약하다 (07-19 는 상자가 왼쪽 y=+0.15 에 있어 우연히 앵커 역할). 이건 pair
  게이트(§8.3-1)가 FK 배치로 오염 차단 — 품질의 구조적 해법은 §8.4.
- 재검증 데이터: scan blob = `storage/blobs/scans/so101_6dof_0/` s22
  (`session_20260719_144642`, 건강) vs s29 (`session_20260721_130527`, scan1
  오염) — 스캔별 z 분해 스크립트로 즉시 재현 가능.

### §8.2 상자 "거꾸로 관측" — approach look-pose 가 도달성만으로 −60° 채택

- 로그: `groups=13 → index=8` — tilt 사다리 13개 중 수직측 11개 자세 IK 전멸
  (SO-101 수직 한계), 살아남은 극단 tilt −60° 채택 (잔차 2.53mm, GOOD 없음).
- 결과: 카메라가 책상 모서리 밖에서 상자 측면 관측 (detect 0007 사진) — points
  1296→**397**, height 4.1→3.0cm, base_z −0.016→**+0.010 (26mm 이동)**. 이걸
  "close 성공"으로 믿고 place 계획에 사용 = **실패보다 나쁜 침묵 품질 저하**.
- 설계 구멍: "도달 편함 우선"에 시점 품질 기준이 없었다 — yaw 는 관측 무관이
  맞지만 **tilt 는 시점 기하 그 자체**.

### §8.3 2차 구현 (07-21 밤 — fast 391 초록, ruff/pyright/tsc/vitest clean)

1. **scene3d snapshot fresh-after-request (주범 수정)** — "최근 N 장" → **"요청
   이후 도착한 N 장"** (버퍼 항목에 도착 시각 동봉, perf_counter). 이동 중 프레임
   이 구조적으로 못 들어감 — settle 노브와 무관. fresh 가 6s 내 안 모이면 raise
   (침묵 과거 프레임 금지, 카메라 스트림 지연 표면화). 실사고 회귀 테스트 2건
   (stale 만 있을 때 timeout / stale+fresh 혼재 시 fresh 만 사용).
2. **스캔 게이트 (§5-6 완료, 부범 대응)** — `build.pair_gate`: 발산(40mm) +
   저fitness(<0.30) + 약중첩·대보정(<0.45 & >15mm) 기각→FK 배치 (사유가 method
   접미로 로그에). 07-20/21 붕괴 pair 전부 기각·07-19 건강 pair 전부 통과를
   실측값 그대로 테스트로 잠금 (test_scan_module).
3. **look-pose 생성 = 카메라 중심 재설계 (§8.2 의 진짜 수정)** — 파지 가족
   (TCP 중심) 파생을 폐기: "TCP 를 물체 옆에 두고 손목 꺾어 카메라만 보는"
   자세류를 통째로 놓쳐 되는 자세가 있어도 전멸했다 (사용자 토크오프 실증:
   카메라 11.3cm·고각 55° = 최적 뷰인데 옛 후보군 밖). 지금은 **카메라를 물체
   위 반구(고각 90/75/60/45° × 방위 12 × 거리 13→18cm)에 직접 배치**하고
   hand-eye(캘 번들) 역변환으로 TCP 도출 (`_camera_look_poses`). **오프라인
   실검증**: 오늘 상자 위치에서 실제 캘 URDF+IK 로 48후보 중 13 도달 (수직
   90° 잔차 0.2mm 포함 — 옛 생성기는 0). 시점 품질 = 고각 하한 45° +
   `_OBSERVE_POINTS_TRUST_RATIO=0.7` 새니티 (22:07 런 실동작 확인). tilt 상한
   노브(45→30 왕복)는 폐기 — 잘못된 축이었다. ⚠ 고각 하한/비율은 실물 튜닝점.
4. **ScanGrowth 스트림 배선** — scene3d cloud 는 SET_STREAM enable 필수인데
   배선이 없어 성장 점군 0개였던 버그. 스캔 시작 enable / 종료 disable (단
   LivePointCloudPanel 소유 상태면 존중 — scanStore.liveEnabled). vitest 3건.
5. **scan 캡처 관측성** — `debug/scan/<session_id>/NNN_color.jpg` 덤프 (이미
   인코딩된 JPEG 재사용, 비용 ~0). 이번 분석이 detect 덤프 우연 의존이었던
   구멍 봉합.

### §8.5 3차 구현 (07-21 심야, 사용자 커밋 후 — fast 395 초록)

1. **파지 적응 진입 사다리 ("68가족 매장" 감사의 수정)** — §8.2 와 동형 결함이
   파지에도: 사다리 그룹이 전 rung 에 같은 파지 자세를 요구 → SO-101 수직류는
   높은 z 에서 자세 불가라, 01:28 "312 전멸(도달 불가)" 위치에서 실제로는
   **68가족이 파지 가능**했음 (오프라인 실IK 감사. 진입 가능 최고 standoff:
   5cm 23가족 / 3cm 12 / 2cm 16 — 수직 tilt0 포함 / 파지만 17=포기가 정당).
   수정: `plan._ENTRY_LADDERS` — 기본(8/5cm) 전멸 시 5→3→2cm 진입 라운드
   폴백, 채택 사다리는 `ServoPlan.standoffs` → `pick._effective_cfg` 가 실행
   cfg 에 반영 (판정 사다리 == 실행 사다리). **하강 역학 불변** (자세 전환
   하강 안 함 — 검증된 same-orientation 하강에서 진입 높이만 적응). 낮은
   진입 = 보정 창 축소는 approach close 관측(5-12mm)이 메움. ⚠ 실물 미검증:
   낮은 진입 rung 의 관측 품질/보정 여유는 집 데이터로 판정.
2. **World 메시 교체 버그** — world_scan 프루닝 → sqlite rowid 재사용 → recon
   id 가 매 스캔 동일(139) → World 의 id-dedup 이 새 메시를 스킵 + done 트리거
   (id 기반)도 재발화 안 함. 수정: 메시 정체성 (id, created_at), 트리거 =
   stream seq. + **스캔 시작 시 기존 메시 클리어** (성장 점군 UX 가림 방지,
   중도 포기 시 리로드로 복원 — 탈출구 유지).
3. **공유기 오검출 2중 수정 (23:41 실물 — servo 소실 중단)** — 옆 테이블에서
   솟은 공유기가 ① 꼭대기(z −0.03)만 셀 하한(−0.05)에 걸쳐 ROI position 판정
   통과 (base_z −0.25 는 셀 밖) → **detector ROI 컷에 base_z ≥ 셀 바닥 조건
   추가** ② 프레임에 든 공유기가 GDINO score 를 빨아들여 완벽히 보이는 큐브가
   0.31 로 눌림 → 0.45 tick 게이트가 "관측 소실" 오판 → **저score 기하 구제**
   (`min_score_floor=0.30`: [0.30,0.45) 는 top z 가 기대 대비 z_jump_max 이내
   일 때만 채택 — 07-17 열화 앵커(z 16mm 이탈)는 여전히 기각, 잠금 테스트
   실사고 수치로 갱신). 판별 근거 = 덤프 0044 (큐브 선명한데 0.31 / 공유기
   0.59) + 07-17 vs 07-21 두 사건의 유일 차별자 = z 기하.

### §8.4 남은 것 (미구현 — 다음 세션)

- **정합 앵커의 구조적 보장**: 물체 배치 운에 안 기대는 정석 = 셀 고정 지물
  (fixture/텍스처 마커, workcell fixtures 승격) 또는 J1 조밀화(중첩 확대,
  §3.1 전용 포즈 설계와 같은 자리). 스캔 게이트는 오염 방지지 품질 확보가 아님
  — 게이트가 FK 배치로 도망가면 mesh 는 σ_t~7.5mm 급으로만 맞는다.
- world 스캔 패널 상세 (§5-5): staleness UX / ROI SSOT 등.
- scene3d SET_STREAM 다중 소비자 refcount (지금은 last-writer-wins + 소유
  존중 규약 — ScanGrowth 주석).
- §8.2 의 place 계획이 실제 놓기 정확도에 미친 영향은 미측정 (이번 런 데이터
  로는 판정 불가 — 다음 실물 런에서 close/coarse base_z 대조 로그 확인).

### §8.6 코드 리뷰 발견 — 재시도 rung 하드코딩 × 적응 사다리 (**수정 완료 2026-07-22**)

> 수정: 실패 테스트로 IndexError **관측 재현** (예측 지점 pick.py:210 정확 일치)
> → `ServoState(rung=len(cfg.standoffs) - 1)` → 91개 초록. 회귀 잠금 =
> `test_servo_adaptive_ladder_empty_close_retry_survives` (발견 시나리오 그대로).

**증상 (잠복 — sim/실물 미노출):** 적응 진입 사다리(§8.5-1)로 집은 물체를 헛집어
close 재시도에 진입하면 의미 있는 재시도 대신 `IndexError` 로 죽는다.

**근본 = §8.5-1 과 동형 결함 (사다리 길이 하드코딩)의 놓친 자리.** 어제 sweep 은
`_retreat_for_retry` 의 `standoffs[1]` → `standoffs[-1]` 로 사다리 길이 무관하게
고쳤는데([pick.py:803](../backend/modules/tasks/pick_and_place/steps/pick.py#L803)),
두 줄 아래 형제 자리는 놓쳤다:

```python
# pick.py:367-368
await _retreat_for_retry(ctx, robot_id, run, comp, cfg)
state = servo.ServoState(rung=1)   # ← 기본 2단 사다리 가정 (놓친 자리)
```

**재현 경로:**
1. `plan_pick` 기본 사다리 `(0.08, 0.05)` 전멸 → `_ENTRY_LADDERS` 폴백 →
   `ServoPlan.standoffs = (0.05,)` (길이 1).
2. `servo_pick`: `_effective_cfg` → `cfg.standoffs = (0.05,)`, `last_rung = 0`.
3. close 후 EMPTY → `GraspFailed` → 재시도, `state = ServoState(rung=1)`.
4. 다음 tick 관측 채택 시
   [pick.py:210](../backend/modules/tasks/pick_and_place/steps/pick.py#L210)
   `cfg.standoffs[state.rung]` = `cfg.standoffs[1]` → **`IndexError` (tuple index
   out of range)**. `except BaseException` 이 잡아 trace 는 남지만, 실패 사유가
   파지 관련이 아니라 엉뚱한 IndexError 로 기록 = 재시도 메커니즘이 무력화.

**테스트가 못 잡은 이유:** 유일한 재시도 테스트 `test_servo_empty_close_exhausted_raises`
가 기본 2단 사다리 snapshot(`_SO0`/`_SO1`/`_MIDSTOP`)로만 돌아 `standoffs[1]` 이
존재. "적응 1단 사다리 + close EMPTY 재시도" 조합은 커버 없음.

**제안 수정 (`_retreat_for_retry` 의 `[-1]` 과 짝):**
```python
state = servo.ServoState(rung=len(cfg.standoffs) - 1)
```
기본 사다리에서 `rung=1` 은 "마지막(가장 가까운) rung" 이므로 의미 보존. 회귀
테스트는 발견 시나리오 그대로 — 적응 1단 사다리(`plan.standoffs=(0.05,)`) +
close EMPTY 재시도가 IndexError 없이 재관측 tick 을 도는지 잠근다.

---

## §9. ROI 재설계 — SharedConfig owner + 3D 편집 패널 (2026-07-22 확정, 구현 착수)

> 07-22 세션 라인 리뷰(exclude_xy → ROI → workcell 소유권)에서 파생된 확정
> 설계. 논의 사슬: "exclude_xy 불필요해 보임" → ROI 로 대체 가능 판정 →
> ROI 를 UI 에서 직접 편집 → workcell 값의 소유/전파 재설계.

### §9.1 확정 결정 (전부 사용자 합의)

1. **ROI = AABB 유지** (6스칼라 x/y/z min/max, base frame). yaw-OBB 반려 —
   작업대 정렬 전제, 필요해지면 그때 (데이터 모델·편집 UX·containment 재작업
   비용 인지하고 반려).
2. **workcell 소유 = `shared_config` 모듈 신설** (backend.md 불변식 "config
   소유" 규칙이 boundary 정본): instance.yaml 읽기/쓰기 + SNAPSHOT 서비스 +
   CHANGED 이벤트. detector/scan 은 **Mirror 소비** (calibration→motion 선례
   동형) — resolve.py 의 부팅 주입 폐기. Mirror = 핫리로드 그 자체 (Save →
   owner publish → 소비자 자동 수렴, 재시작 0, 분산 무관).
3. **ROI 패널 (frontend)** — 3D 씬에서 시각 확인 + 직접 편집:
   - 렌더 3계층: 얇은 와이어(`<Edges>` — 삼각 대각선 없는 12모서리) + 반투명
     면 (opacity 0.10~0.15, **depthWrite off** — 포인트클라우드 안 가림,
     DoubleSide) + 상태 램프 (hover 면만 0.25~0.35 / drag 강조).
   - 편집 2모드: **면 핸들 6개 = resize** (한 면 = 한 bound, 박스는 6개
     독립 plane mesh 로 구성 — 면별 하이라이트와 1:1) + **중앙 기즈모·
     Shift드래그 = translate** (min/max 동시 이동, drei TransformControls
     translate 모드 재사용 — scale 모드는 피벗 대칭 결합이라 부적합).
   - 숫자 입력 동기 (정밀값 타이핑, 양방향).
   - **명시 Save** — 편집은 draft 로컬 상태, Save 시 shared_config 서비스 →
     instance.yaml 실제 반영. Cancel/Revert 탈출구.
   - 로봇 base 오버레이/검증 가드 **없음** — 편집하는 그 씬에 로봇 URDF 가
     이미 그려짐 (Scene object) = 가시성 공짜.
4. **스캔 메시 ROI 크롭 제거** — world 메시 목적 = "세상이 이렇구나" 전체
   조망. 크롭은 검출과 무관한 시각 취향이었음 (build.py 크롭과 detector 컷은
   같은 값의 별개 적용점 — 코드 확인). world scan 은 시각화 전용 task 라
   파지 로직 영향 0.
5. **exclude_xy(로봇 베이스 13cm 원기둥 컷) 제거** — ROI 를 "로봇이 안
   들어오게" 조절하는 규율로 대체 (패널이 그 규율을 가시화). ROI 컷이
   detector 상류라 approach_observe 낭비 이동(§9.2)도 함께 소멸. **패널 완성
   후 제거** (순서 의무 — 패널 없이 지우면 비가시 수동 규율로 회귀).

### §9.2 리뷰 발견 (07-22 라인 리뷰 — exclude_xy 제거로 함께 해소)

`exclude_xy` 는 plan_pick 에만 있고 **approach_observe 에는 없다** — 베이스
오검출이 최고 score 면 관측 타깃으로 선정돼 팔이 베이스로 이동 (07-21 밤 실물
관찰과 일치: "계획 세우러 베이스 근처로 갔다가 파지는 큐브"). 부작용: ①낭비
왕복 ②close 관측을 엉뚱한 대상에 소진 — 반환 후보에서 베이스가 빠지면 진짜
타깃은 coarse 데이터로 계획 + trust_yaw 가 잘못 켜짐. ROI 가 detector 에서
상류 컷하면 이 후보 자체가 안 생겨 클래스째 소멸.

### §9.3 구현 슬라이스 (순서)

1. backend.md 불변식 "config 소유" 규칙 (완료 — 이 § 와 동시).
2. §8.6 rung 하드코딩 fix (실패 테스트 재현 먼저 — 별건).
3. `shared_config` 모듈 + contract + yaml R/W + 테스트.
4. detector/scan Mirror 전환 + resolve.py 주입 제거.
5. scan build ROI 크롭 제거.
6. ROI 패널 (contract gen → 렌더 → 편집 → Save).
7. exclude_xy 제거 (plan.py 컷 / module.py 주입 / resolve.py bases / 테스트).
8. 검증 게이트: ruff/pyright/fast pytest + gen:types regen invariant + vitest.

⚠ 실물 미지수 (sim 으로 못 닫는 것): ROI 실측값 조정은 집에서 패널로 직접,
Mirror 분산 수렴은 mock/sim 검증 + 집 분산 부팅에서 최종 확인.
