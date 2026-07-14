# Object-centric 파지 재설계 — 구현 핸드오프 (배경·문제·리서치·계획 완결본)

> **이 문서 하나로 재설계를 이해하고 구현에 착수할 수 있게 자체 완결로 씀.** 다른 세션에
> 이 문서만 주고 "구현해" 하면 되도록: 배경(§1) → 목표(§2) → 현재 코드 문제(§3, 시뮬 재현
> 근거) → 필드 리서치(§4, 출처) → 해결 원리(§5) → 로드맵(§6) → 검증 상태(§7) → 채택 판단(§8),
> 그리고 **부록**으로 "이 결론에 이른 사고 흐름 + assistant 가 갇혔던 오류 + 사용자가 깬 방식"(§9).
>
> **status: §10 확정 설계 구현 완료 (2026-07-14 — §10.5 keep/replace/add 전부
> 반영 + 프로덕션 파이프라인 sim 24/24, backend.md "2026-07-14 (3)" 참조).
> 남은 관문 = 실물 (§10.6).**
> ⚠️ **§1~§9 는 "박스 + 단일 사선 뷰" 로 좁혔던 옛 방향 — §10 이 그걸 뒤집는다.**
> **구현 착수 전 반드시 §10 부터 읽을 것.** 목표는 처음부터 **일반 형상**(박스 전용 아님),
> 그래서 **멀티뷰가 필수**이고 파지는 **관측 점군 표면의 antipodal** 로 뽑는다. §1~§9 의
> reachable-orientation·resolve 게이트·object-centric·phantom 진단은 여전히 유효(§10 이 계승),
> 단 "단일 뷰면 충분 / 윗면 footprint / 고정 궤도 멀티뷰 / height 게이트" 는 §10 에서 폐기.
>
> §10 = 이 세션의 sim 전수 검증(물리 렌더 부분점군 + 실 캘 kinematics + 노이즈/마스크/충돌/
> clutter, adversarial) 결과 + **확정 설계** + **현재 코드 상태(keep/replace/add)** + 재현 방법.
> 관련: [backend.md](backend.md) #1/#2, 메모리 [[project_object_centric_grasp_redesign]].
>
> ⚠️ **구현 규율(§9 계승, 이 세션에서 값비싸게 재확인): 해피케이스로 "됐다" 하지 말 것,
> confirmation-bias 검증(되는 케이스만 봄) 금지 — 깨지는 케이스·노이즈·충돌을 먼저 때려라.
> 하드웨어 없어도 sim 으로 가능한 건 전부 소진하고 실물엔 진짜 hardware-only 만 남겨라.**

---

## 1. 배경 — "버그 2개 고치기"로 시작했다

집에서 실 SO-101 + D405 로 pick_and_place 를 처음 돌린 세션([backend.md](backend.md) 2026-07-13
밤/3)에서 나온 열린 문제 둘로 시작:

- **#1 detection height/base_z 부정확** — 같은 큐브가 뷰마다 height 0.5↔1.5cm, base_z −0.23m 짜리
  phantom 후보가 낌.
- **#2 "IK 도달성 오판"** — 큐브 (0.275,0.208) 에서 44 grasp 후보 전부 IK 불가. 근데 사용자가
  토크오프로 팔을 그 자리에 실제로 갖다 댐 (관절값은 URDF limit 안).

처음엔 "이 둘을 고치자"였는데, 파고들수록 **둘이 별개 버그가 아니라 하나의 숨은 전제("물체는
책상 위")에서 갈라진 증상**이었고 → 파지 접근 전체의 재설계로 갔다. (그 사고 흐름은 §9.)

## 2. 목표 (스코프 잠금)

**한 변 ~2cm 물체를 집어서 어딘가 둔다.** 그게 전부. 물체가 어디 있는지(책상 / 사람 손 / 다른
로봇에 들려 공중) **가정하지 않는다.** 정밀도 기준 = "아무도 안 잡은 2cm 큐브를 집을 수 있는
수준"(= 지금 책상에서 되는 그 수준, **회귀 금지**, 새 정밀도 달성 아님). 누가 들고 있으면 물체는
그만큼 커지므로 2cm-free 가 정밀도 상한.

## 3. 현재 코드의 문제점 (진단 — 시뮬 재현 + 필드 대조)

### 3.1 뿌리 하나: "물체는 책상 위" 전제 → 세 증상 (object-centric 이면 셋 동시 소멸)

| 증상 | 코드 | 못박힌 가정 |
| --- | --- | --- |
| height/base_z 부정확·phantom(−0.23m) | `floor_z_and_height`([detector/projection.py](../backend/modules/detector/projection.py)) = bbox ring 25th pct 로 바닥 재추정 후 `top−floor` | 바닥이 있다 |
| grasp IK 전멸 | `plan_grasp`([tasks/pick_and_place/geometry.py](../backend/modules/tasks/pick_and_place/geometry.py)) = `_TOPDOWN` 가족(수직±tilt 40°)만 생성 | 위에서 접근 |
| lift 이상 | `pre` 가 `grasp` 바로 위(월드 z) + `lift`=MoveL 수직 상승([steps.py](../backend/modules/tasks/pick_and_place/steps.py)) | 위로 들어올림 |

`floor_z_and_height` 는 fragile: ring 이 depth 이상치(책상 모서리 너머/edge artifact) 하나만 물어도
25th pct 가 무너져 floor_z −0.23m → height 19cm phantom. 뷰 간 height 0.5↔1.5cm 도 top/floor 를
소표본 percentile 로 국소 재추정하는 두 노이즈의 차라 합쳐진 것. **더 근본: floor 뺄셈은 "단일 뷰로
안 보이는 바닥을 추측하는 꼼수"** — 책상 없으면 무너지고(공중/손), 추측이라 부정확.

### 3.2 #2 정정 — "IK 오판"이 아니라 top-down 강제 (시뮬 재현으로 확정)

실물 실패 케이스를 **캘 적용 kinematics(link_offset+sag+joint_offset)로 pybullet 재현**:

- **충실성**: 캘 적용 FK(시연관절 [-1.07,2.26,-1.82,0.81,0.61,0.64]) = (0.241,0.178,−0.034) ≈ 실물
  리포트 TCP (0.241,0.179,−0.033), **sub-mm 일치** → 시뮬이 실물 kinematics 를 정확히 재현.
- **솔버 정상**: 시연 pose 는 FK→IK 왕복 성공. IK 버그 아님.
- **위치는 되나 자세가 안 됨**: 큐브 위치에서 position-only IK 는 전 z 성공, **top-down 자세는 전 z 실패.**
- **정량**: 그 위치의 자연 도달 자세는 top-down 에서 **~141° 떨어짐**, top-down±tilt 0~90° 전부 불가.
- **reachable 자세는 존재**: **tilt 30~60° / base 쪽 접근**이면 도달 OK (스윕 56 중 7). 이 작은 팔은
  먼 리치에서 **손목을 수직으로 못 세울 뿐**, 비스듬히는 잡는다.

→ **IK/솔버는 멀쩡. `plan_grasp` 의 top-down 강제가 유일한 범인.** [backend.md](backend.md) #2 의
"도달 가능한 pose 를 불가로 오판" 프레임은 폐기 (top-down pose 는 진짜 불가, 시연은 다른 자세로 간 것).

> 재현 방법(다음 세션이 다시 돌릴 수 있게): `open_sqlite("horibot.db")` → `CalibrationRepository.
> get_active_bundle("so101_6dof_0")` → `build_calibrated_kinematics(URDF, robot, arm, bundle,
> PybulletKinematics)` → `kin.initialize()`. 그 kin 으로 `plan_grasp` 후보/자세 스윕에 `kin.ik` 를
> 돌리면 하드웨어 0 으로 재현된다. (robot/ 은 repo 루트, backend cwd 의 부모.)

### 3.3 `resolve_reachable` 냄새 (벤치로 확인 — 단, naive 개선은 회귀)

[motion/module.py](../backend/modules/motion/module.py) `resolve_reachable` 의 `for budget in
(10,40,None)` staircase 는 겉으론 냄새(매직넘버 + "선호순서 양보" 자백 주석 + `kin.ik` 이 성공해도
early-exit 없이 closest-solution 까지 다 도는 미스매치의 반창고). **그러나 벤치가 naive 수정을 반증**:

| 케이스 | 현재(deepening) | 린(단일패스 early-exit, b=200) |
| --- | --- | --- |
| 실패 많은 스캔 | 419 IK / 208ms | **4275 IK / 2116ms (10× 악화)** |
| reachable 앞정렬 | 100 IK / 57ms | 14 IK / 7ms |

early-exit 은 **성공에만** 이득 (실패 pose 는 예산 끝까지 돎). deepening 의 "싼 예산 먼저"가 실패를
싸게 만드는 게 실제로 옳다. **진짜 문제 = staircase 자체가 아니라 "cheap→expensive 게이팅" 원리가
매직넘버로 굳고 미래(충돌 게이트)를 못 담는 형태.**

### 3.4 MoveL — 끝점만 보장, 경로/자세 미보장

`descend`/`lift` 는 MoveL(Cartesian 직선)인데 **끝점 IK 만 확인**된다. 직선 상 중간점 실현성, 접근 중
자세 고정 여부는 코드에 보장이 없다 (§4 에서 이게 표준 함정임 확인).

## 4. 리서치 — 필드는 어떻게 푸나 (표준 대조)

우리 방향이 표준과 일치함을 확인하고, **우리가 안 넣은 표준 장치**도 드러남:

- **pick 모션 구조** = OMPL(관절공간)로 pre-grasp → Cartesian LIN 으로 직선 접근 → Cartesian 으로
  후퇴. 우리 세그먼트 계획과 동일. ([MoveIt pick&place](https://moveit.github.io/moveit_task_constructor/tutorials/pick-and-place.html))
- **grasp 을 reachability 로 필터**가 명시 표준 (MoveIt Grasps). 확장 = **reachability map / inverse
  reachability map**: FK 를 미리 대량 계산해 도달 가능 SE(3)+manipulability 를 저장 → 요청마다 즉시
  스크린 (라이브 IK 스윕 대체 = 빠른 resolve). ([MoveIt Grasps](https://moveit.picknik.ai/main/doc/examples/moveit_grasps/moveit_grasps_tutorial.html), [reachability-aware grasp](https://arxiv.org/pdf/1910.06404), [IRM](https://www.researchgate.net/publication/273131725_Stance_Selection_for_Humanoid_Grasping_Tasks_by_Inverse_Reachability_Maps))
- **Cartesian(MoveL) 실현성** = "각 점 IK 가 풀려도 직선 이동은 보장 안 됨"이 필드 명제.
  `compute_cartesian_path` 는 **달성 fraction(0~1)** 반환 → fraction≈1.0 요구가 표준. low fraction
  원인 = 특이점 근접/joint-jump/orientation 불가, **jump_threshold** 로 급플립 차단. IK 솔버 품질도
  갈림(KDL 특이점 실패 → **TRAC-IK** 로 해결 흔함) — 우리 pybullet DLS 도 후보. ([cartesian fraction](https://groups.google.com/g/moveit-users/c/wsxMwps2V4w), [singularity/cartesian](https://docs.picknik.ai/how_to/robotics_applications/cartesian_path_following/))
- **object-centric + 핸드오버** = 이미 풀린 연구 영역: Contact-GraspNet(점군 6-DoF grasp, 관측 점에
  rooting), SDF 로 도달 불가 grasp 빠른 스크린, 양 로봇 reachability SDF 로 핸드오버 교환 자세 선택. ([Contact-GraspNet](https://arxiv.org/abs/2103.14127), [reachability-aware handover](https://link.springer.com/article/10.1007/s10514-025-10201-y))

## 5. 해결 원리 (설계 방향)

1. **object-centric 측정** — 물체 3D 는 물체 자기 점군(mask→depth→base, `base_points_from_mask`)에서만.
   `height=top−floor` 폐기. floor 는 detection 에서 제거, **planner 충돌 평면(cm 오차 OK, 옵션)** 으로만 강등.
2. **멀티뷰로 occlusion 극복** — 단일 뷰는 물리적으로 전체 3D 불가(가려진 면은 depth 데이터 자체가
   없음 — 산업 카메라도 동일). 타깃 여러 각도 관측 → base frame 점군 융합(이미 정렬 → 쌓기).
   **타깃 중심 멀티뷰 = 자동 뷰 계산**과 한 덩어리 (미리 티칭한 타깃-중심 자세는 존재 불가;
   `search` waypoint 는 1단계 광역 탐색용이라 별개 — 이걸 멀티뷰로 재활용 X).
3. **reachable-orientation 파지** — top-down 강제 폐기. 조 축 수평(옆면 파지 성립) 유지 + **접근
   자세는 그 위치에서 도달 가능한 것 중 선택**. antipodal(마주 보는 두 면). reachability map 으로
   빠르게 스크린 → 살아남은 소수만 전체 IK 검증.
4. **grasp 프레임 상대 동작** — 접근/후퇴는 **접근축** 기준(월드 +z 아님), 긴 이동은 **관절공간 home
   경유**(pick↔place 도달성 분리). 세그먼트:
   `home→pre: MoveJ(pose)` / `pre→grasp: MoveL(접근축, 자세 고정)` / `grasp→retract: MoveL` /
   `retract→home→place-pre: MoveJ` / `place 삽입·후퇴: MoveL`.
   **MoveL 은 fraction 검사 + jump_threshold, 짧게** (경계 팔이라 중간점 깨질 수 있음).
5. **resolve = cheap→expensive 필터** — 후보를 likelihood 순으로, **싼 게이트부터**(도달성 스크린 →
   (미래)충돌 → 비싼 전체 IK/경로) 통과시켜 첫 합격 채택. deepening 의 "싼 것 먼저" 정신 계승, 매직
   staircase 는 이 원리로 대체. **충돌 게이트가 나중에 한 칸으로 끼워질 자리 예약.** `resolve` 는
   index 말고 **IK 해(joint)를 반환**(실행부 재계산 제거). seed = 현재 관절.
6. **정밀도 = closed-loop** — 2cm 는 캘(σ_t~7.5mm) 대비 빡빡 → 절대 open-loop 금지. 물체 점군 상대
   파지 + eye-in-hand 공통오차 상쇄 + 접근하며 재관측. 멀티뷰가 여러 번 보므로 자연 흡수.
7. **충돌은 지금 최소** — so101 단독/omx 정지로 멀티로봇 동적 충돌 defer. 바닥은 (a)손에 들어 회피
   or (b)평면 하나. cross-robot 공유 충돌 월드(base_pose+URDF+live joint)는 재료는 있으나 dynamic
   coordination 이 커서 뒤로 (step 4 설계).

## 6. 구현 로드맵 (phase — 의존 순서, 각 phase 독립 검증)

```
1. grasp 자세를 reachable-orientation 으로 (top-down 강제 제거)          [✅ 2026-07-14 구현]
     + resolve 를 cheap→expensive 필터로(충돌 자리 예약, 해 반환) + 바닥·self 충돌 plane 최소
     → 이걸로 #2("IK 전멸")가 풀린다. (IK 솔버는 안 건드림 — 정상 확인됨)
2. so101 단독(omx 정지) 물체 바닥 — 자동 멀티뷰 관측 + object-centric detect/grasp
     ★ 불변식: 처음부터 floor-free(물체 점군)로. 검증만 책상에서.  → #1 해소   [✅ 2026-07-14 구현]
3. 손으로 들어(공중) 검증 — object-centric 이 진짜 바닥 무관인지 (+ 손↔물체 세그 분리)
4. cross-robot 충돌 (설계부터 — 공유 충돌 월드; reachability SDF 스크린 검토)
5. 핸드오버 (상대 능동 건넴 + reachability-aware 교환 자세 + coordination)
```
각 단계가 집에서 독립 검증되는 increment (1:큐브 도달성 / 2:책상 파지 / 3:공중 파지 …).
1·2 는 시뮬/유닛 레벨 검증까지 완료 (backend.md "2026-07-14 (2)"), **실물(책상 파지) 검증이
남은 관문** — 실물 통과 전 "됐다" 아님 (구현 규율).

## 7. 검증 상태 (정직하게 — 뭘 확인했고 뭘 안 했나)

- **시뮬 검증됨**: top-down 근본원인(§3.2) / reachable 자세 존재 / resolve 벤치(§3.3).
- ~~미검증 — 지금 시뮬로 가능~~ → **전부 소화 (2026-07-14 구현 세션, 구현 전 선검증)**:
  1. ~~MoveL 직선 중간점 IK~~ → 도달 후보(tilt 30~60)의 pre→grasp **전 구간** 샘플 IK 통과.
     resolve `linear` 게이트로 계획 시점에 상시 검증 (fraction 반환 대신 통과/기각 — 부분
     경로를 쓸 일이 없어서).
  2. ~~자세 고정 vs slerp~~ → 코드 확인: slerp (목표 자세로 s-동기 보간, backend.md #3 그대로).
     PnP 진입/후퇴는 시작≈목표라 사실상 자세 고정.
  3. ~~resolve 전멸 → 깔끔 실패~~ → workspace 밖 케이스 0.13s 사유 있는 전멸 + task 예외
     경로 회귀 테스트 (`test_scenario_ik_exhausted_raises` 등).
  4. ~~pre·grasp·경로 셋 다~~ → resolve 가 끝점 2 + 직선 경로 + 바닥 충돌까지 게이트.
  5. ~~특이점 근처 MoveL 튐~~ → 도달 경로에서 인접 샘플 joint jump ≤5°/cm (플립 없음).
     jump_threshold 등가 게이트(20°/샘플)를 MoveL 사전검증 + resolve 경로 게이트에 도입.
- **실물-only (집) — 여전히 남음**: raw depth 품질(D405 최소거리~7cm/각도 — 자동 뷰 반경
  0.16m 실측 포함) / depth↔color 정렬 전제 / antipodal 이 부분 점군에 견디나 / 실제 파지 성공.

## 8. 채택 판단 필요 (우리 규모에 — cargo-cult 금지)

§4 표준을 **전부 넣는 게 아니라** 우리 use case(SO-101, study, 단일 팔)에서 정당화되는지 따진다:

- **reachability map precompute** vs 라이브 IK 스윕 — 후보 적으면 라이브도 빠를 수 있음. 값어치 판단.
- **TRAC-IK 도입** vs pybullet DLS 유지 — 특이점 실측 문제 나오면.
- **Contact-GraspNet(학습)** vs 기하 antipodal — study 는 기하부터, 필요 시 학습 승격.
- **compute_cartesian_path 등가(fraction/jump_threshold)** 를 우리 MoveL 에 도입 — 거의 필수급.

---

## 9. 부록 — 이 결론에 이른 사고 흐름 (재발 방지용 서사)

기술 결론만 보면 "왜 이렇게 정했나"의 맥락이 날아간다. 다음 세션이 같은 사고 오류를 반복하지
않도록, 이 재설계가 나온 과정을 박아둔다.

### 9.1 assistant 가 갇혀 있던 사고방식

**핵심 고착 = "pick-and-place 니까 물체는 책상 위에 있다."** 의식 못 한 이 전제가 파생 가정을
낳았고, assistant 는 그 가정들 안에서만 "고치려" 했다:

1. **floor 를 없앨 생각 대신 "잘 계산"하려 함** — height/phantom 이 floor 추정에서 나오는데
   "ring→RANSAC 평면"으로만 감. floor 라는 것 자체가 책상 전제임을 못 봄. (band-aid 반사.)
2. **그리퍼는 당연히 top-down 이라 가정** / **집으면 월드 +z 로 든다 가정.**
3. **해피케이스만 보고 "됐다"로 미끄러짐** — 끝점 도달성 하나 확인하고 "실물 테스트 하시죠" 하려 함.
4. **자기 머릿속에서만 추론, 필드 리서치 안 함** — 표준 해법(MoveIt/reachability map/Contact-GraspNet)을
   안 찾고 코드에서 재발명하려 함.
5. **테스트 전 "최적화" 제안** — resolve 를 "단일패스 early-exit"로 바꾸자 했는데, 벤치해보니 실패
   케이스에서 10배 악화(회귀). armchair 최적화가 그 자체로 smell.

공통 뿌리: **눈앞 코드에 갇혀, 전제를 의심하지 않고 증상만 만졌다.**

### 9.2 사용자가 그 틀을 깬 흐름 (질문이 가정을 한 겹씩 벗김)

| 사용자 질문/지적 | 벗겨진 가정 | 도달점 |
| --- | --- | --- |
| "물체 크기 찾는데 **바닥을 꼭 찾아야 해?**" | floor 필요성 | 가로 크기/파지는 물체 점군만으로 됨 |
| "로봇이 **무조건 책상에 놓여야만** 집어? 핸드오버는?" | 책상 전제 자체 | 지지면 무관 = object-centric |
| "그리퍼는 **무조건 top-down**?" | top-down 강제 | reachable 자세 중 선택 |
| "집고 **무조건 +z 로** 들어? 살짝 들고 home 갔다 place 도" | 월드 +z lift | grasp-frame 동작 + home 경유 |
| "**픽앤플레이스에만 꽂혀서** 그런 거 아냐?" | 뿌리 명명 | 세 증상 = 한 전제 |
| "코드 수정 전에 **스크립트로 테스트부터**" | armchair 확신 | 시뮬 재현 → 내 최적화가 회귀임을 발견 |
| "**코드 냄새** 생각해 / **넓게** 봐" | 근시안 | cheap→expensive 필터, MoveJ/L/C/P 고려 |
| "네 생각에 갇히지 말고 **리서치도**" | 재발명 | 필드 표준 대조(§4) |
| "해피케이스만 보고 **잘됐습니다 하려 하지?**" | 조기 종결 | 미검증 non-happy 목록 명시(§7) |

**결정적 전환점**: (a) **핸드오버 시나리오** — "공중에 들려 있으면?"이 책상 전제를 물리적으로
불가능하게 해 object-centric 을 강제. (b) **"테스트부터"** — 시뮬 재현이 잘못된 "IK 버그" 진단과
잘못된 "early-exit 최적화"를 둘 다 죽임.

### 9.3 다음 세션이 새길 메타 교훈

1. **증상이 아니라 전제를 의심하라.** "이 값이 틀렸다" 앞에 "이 값을 애초에 왜 구하나"를 물어라.
2. **band-aid 금지.** threshold/필터/staircase 튜닝으로 덮지 말고 근본. ([[think-root-cause-not-bandaid]])
3. **코드 짜기 전에 시뮬로 재현·검증.** armchair 진단/최적화는 자주 틀린다 (이 세션에서 2번 틀림).
4. **자기 머릿속에 갇히지 말고 필드를 찾아봐라** (그러나 cargo-cult 도 X — use case 정당화 후 채택).
5. **해피케이스로 "됐다" 하지 마라.** 실패 케이스/경로 실현성/충돌까지 봐야 완료.
6. **작은 팔의 한계를 존중하라.** SO-101 은 먼 리치에서 손목을 수직으로 못 세운다 — 큰 팔 기준
   가정(top-down)이 이 팔에선 전멸이 된다.

---

## 10. 대전환 — 일반 형상 + 멀티뷰 필수 (2026-07-14 밤, sim 전수 검증)

> **§1~§9 는 은연중 "작은 박스 + 단일 사선 뷰" 로 문제를 좁혔던 옛 방향이다. 이 §10 이
> 그걸 데이터로 뒤집는다.** 구현 세션은 §10 을 정본으로 삼고, §1~§9 는 맥락(왜 여기 왔나)
> 으로만 읽어라. 아래는 이 세션(구현 없이 sim 검증만)이 확정한 것 전부다.

### 10.1 무엇이 바뀌었나 — scope 대전환

앞 세션이 로드맵 1·2 를 구현해 놓고(아래 §10.5 현재 코드), 이 세션에서 "단일 사선 뷰면
충분하다" 를 밀다가 사용자가 반례를 연달아 던져 근본 전제가 틀렸음이 드러났다:

- **목표는 처음부터 일반 형상이다** (박스 전용 아님). assistant 가 실험 편의로 "작은 큐브"
  로 멋대로 좁혔던 것 — 아무도 박스만 집는다고 한 적 없음. 초기 물체가 극단적이진 않겠지만
  (한 변 ≤ 20cm, 그리퍼가 무는 물체라 실제론 훨씬 작음), **파이프라인은 처음부터 일반 형상
  을 상정해 지어야** 나중에 이상한 물체에서 갈아엎지 않는다.
- **"단일 뷰면 충분" 은 prismatic(박스/세운 원기둥) 전용 지름길이었다** — 윗면 윤곽으로
  가려진 먼 면을 추측하는 것. 구·원뿔·병처럼 높이 따라 단면이 변하면 무너진다(§10.3-A).
- 그래서 §5 의 object-centric·멀티뷰가 옳았고, assistant 가 "멀티뷰 걷어내자" 던 게 틀렸다.
  **걷어낼 건 멀티뷰 개념이 아니라 경직된 구현**(고정 궤도 pose 생성 / 윗면 footprint 파지
  / height 하드게이트)뿐.

### 10.2 검증 방법론 (재현용 — 다음 세션이 다시 돌릴 수 있게)

전부 **물리적으로 정직한 sim** — 하드웨어 0:

- **부분 점군**: PyBullet 카메라(`p.getCameraImage`, TINY renderer)로 물체를 렌더 →
  seg mask 로 물체 픽셀만, depth buffer 를 `inv(P@V)` 로 base frame 역투영. **가려진 면
  (바닥/뒷면)엔 점이 없다** = 실 D405 가 볼 법한 것. 카메라 pose 는 실 팔로 닫음:
  reachable 관측 자세 IK → FK → hand_eye 로 카메라 pose 환산.
- **실 kinematics**: `open_sqlite("backend/horibot.db")` → `CalibrationRepository.
  get_active_bundle("so101_6dof_0")` → `build_calibrated_kinematics(URDF, robot, arm,
  bundle, PybulletKinematics)` → `kin.initialize()`. link_offset+sag+joint_offset 적용,
  FK 가 실물 TCP sub-mm 일치(§3.2 anchor).
- **hand_eye**: `bundle.hand_eye.result_data.R_cam2gripper / t_cam2gripper` (cam→ee).
  카메라 pose→TCP: `R_be = R_bc·R_ceᵀ`, `t_be = cam_pos − R_be·t_ce` (projection.py 규약
  과 동일 — aim_cos≈1.0 로 규약 방향 검증됨).
- **antipodal 탐색**: open3d 로 점군 voxel(3mm) 다운샘플 + 법선 추정(반경 1cm, centroid
  바깥 향하게 orient). 각 점 p_i 에서 접근선 d=−n_i 를 따라 폭[4~35mm]·측방[≤5mm] 안에
  반대 접촉점 p_j 찾고, n_j·d > cos(25°)(anti-parallel) 면 유효 쌍.
- **adversarial 원칙**: 되는 케이스가 아니라 **깨지는 케이스**(구/원뿔/L자 concave) + 노이즈
  (등방 Gaussian σ) + 마스크 bleed(책상 점 섞기) + 아래-outlier(flying-pixel) + clutter 로
  때렸다. 이게 §9 의 confirmation-bias 교훈을 실천한 것.

### 10.3 검증 결과 전부 (숫자)

**A. 단일 뷰 충분성은 prismatic 한정 (shape_generality)** — 사선 45° 단일 뷰, 측정
footprint vs 파지높이 실제 단면:
- box 오차 0mm / 세운 원기둥 +1mm → **OK** (prismatic: 윗면=파지 단면)
- **구 −5mm 과소 → 헛집음** (비-prismatic: 윗면 band 가 적도 단면보다 좁음). 원뿔·병은 더 심함.

**B. 표면 antipodal 은 단일 뷰로 전멸, 멀티뷰로 생성 (verify_antipodal_multiview)** —
가정 없이 관측 표면에서 antipodal 쌍을 찾으면:
- **단일 뷰: box 포함 전 형상 0쌍** (마주보는 두 면 중 먼 쪽이 항상 가림).
- 3뷰 융합: box 196 / 세운원기둥 339 / 눕힌원기둥 154 / 구 259 / L자 405 쌍, 전부 반대편 뭄.
- → **일반 형상 파지엔 멀티뷰 필수.** (박스 단일뷰가 "됐던" 건 §10.3-A 의 prismatic 꼼수)

**C. 닿는 뷰만으로도 커버리지 충분 (verify_reachable_multiview)** — 임의 지정 아닌 실 IK
로 닿는 뷰만 모아 융합. base 쪽 쏠림 우려는 기우: eye-in-hand 라 **방위 180~300°** 확보 →
반대편까지 관측 → antipodal 191~400쌍(3위치 × box/눕힌원기둥/구 전부).

**D. end-to-end 파지 실행 가능 (verify_grasp_execution + comprehensive_verify)** — 접촉쌍
→ SO-101 단일조 TCP pose(Phase-1 상수: TCP→고정조 7.9mm, 조 여유 5mm, 접근 60mm) →
접근 tilt 스윕 → pre+grasp IK + 바닥충돌 + **그리퍼↔물체 충돌**(그리퍼 열림 자세 로봇↔물체
침투 >3mm 검사, kin 실 URDF 씬 재사용):
- **workspace 12위치(x 0.20~0.28, y −0.05~0.22) × 4형상(box/눕힌원기둥/구/L자 concave)
  = 48/48 실행 가능** (tilt 0~30°).

**E. 노이즈·마스크 견고 (verify_grasp_exec_noise)** — σ1mm(D405 실제 ~0.5mm보다 거침) +
책상 bleed 10% + 아래-outlier 2% 주입, Monte-Carlo → 실행 가능 파지 **6/6 케이스 5/5**.

**F. phantom 진짜 원인 + 수정 (outlier_phantom)** — 실물 #1 버그(base_z −0.23m,
height 19cm)를 재현: 현 `object_metrics_from_points` 의 **2-percentile bottom** 이 아래-
outlier 3~5% 에 끌려 base_z −17~−22cm / height 15~20cm (실물 로그 일치). **수정 =
z-gap 군집**: top 에서 아래로 5mm 빈 틈 만나기 전까지가 물체 몸통 → outlier 10% + 노이즈 +
bleed 다 견딤(base_z −4.4cm 안정). footprint(윗면 band)는 outlier 무관하게 항상 견고.

**G. 정지 기준 (verify_motion_stopping)** — 뷰 하나씩 누적하며 실행 가능 antipodal 이 서면
멈춤 → **2~4뷰면 파지 성립**. 고정 7뷰는 과함 → adaptive 정지.

**H. 관측 이동 경로 (verify_motion_stopping)** — naive joint-보간으로 궤도 뷰 사이를 이으면
**floor/obj 충돌 잦음**(retract-via-SEED 로도 대부분 안 풀림). 원인 둘: (1) 관측 view config
자체가 floor/obj 충돌인데 `reachable_cam` 이 IK+self 만 보고 통과시킴(floor·object 미검사),
(2) 큰 관절 스윙 중 그리퍼가 물체 관통. → **관측 자세는 floor+object 충돌 스크리닝 필수 +
뷰 간 이동은 충돌 인지(retract/planning), naive MoveJ 금지.** (숫자는 체크에 caveat 있으나
정성 결론은 확고 — 궤도 이동을 공짜로 가정 말 것.)

**I. clutter (verify_clutter)** — 타깃 주변 이웃 물체(가림 + 접근 충돌):
- 이웃 한쪽/양옆 3.5cm → **파지 O** (타깃 점군 유지, 이웃 충돌 회피 tilt 존재).
- 빽빽 3면 2.8cm → **파지 X** — antipodal 쌍은 124개 나오지만 그리퍼가 이웃 없이 못 들어감
  → **충돌 게이트가 올바로 거부**(맹목 파지 안 함 = fail-safe).

### 10.4 확정 설계 (구현 대상)

**멀티뷰-우선 + 표면 antipodal + reachable-orientation + 충돌 게이트, 일반 형상.**

1. **관측 = adaptive 멀티뷰** — 타깃 검출 후, 반경/고도/방위를 **탐색축**으로(고정 구면 아님)
   도달 가능한 관측 자세를 찾아 base frame 점군 누적. **관측 자세는 IK+self 뿐 아니라
   floor+object 충돌까지 스크리닝**. 뷰 하나 추가할 때마다 파지 성립 검사, **서면 멈춤(2~4뷰)**.
   못 서면 antipodal 이 비어 있는 방향으로 다음 뷰. 상한 넘으면 사유 있는 실패.
2. **파지 선택 = 관측 점군 표면 antipodal** (가정한 footprint 아님) — open3d 법선 + antipodal
   샘플. SO-101 은 조 축 수평 옆파지라 **조 축 수평인 쌍**으로 필터. §10.2 파라미터.
3. **파지 자세 = reachable-orientation** (§5.3 계승) — 접촉쌍 → 단일조 TCP(Phase-1 상수) →
   접근 tilt 스윕 → pre+grasp IK + 바닥충돌 + **그리퍼↔물체/이웃 충돌** 게이트 → 첫 통과 채택.
   못 찾으면(빽빽 clutter 등) **"안전 파지 불가" 명시 실패**(맹목 금지).
4. **뷰 간 이동 = 충돌 인지** — naive MoveJ 금지. retract 경유 or 경로 충돌 검사.
5. **object 기하(bottom/height) = z-gap 군집** (2-percentile 폐기 — §10.3-F). footprint 는
   윗면 band 유지.
6. **height 하드게이트 폐기** — 충분성 판정은 "실행 가능 antipodal 이 섰나"(§10.3-G)지
   height 아님.

### 10.5 현재 코드 상태 — keep / replace / add (구현 세션 필독)

> ✅ **2026-07-14 구현 완료** — 아래 REPLACE/ADD 전부 코드 반영 (상세·검증 수치는
> backend.md "2026-07-14 (3)"). 구현 중 발견 1건: antipodal 폭 하한은 프로토타입
> 4mm 가 아니라 **8mm** — 4mm 는 노이즈 σ1mm 의 4σ 거리라 단일 뷰 edge 에서
> 가짜 쌍(w=4mm)이 생겼다 (verify_production_pipeline 재검증으로 확인·수정).

이 세션 앞부분에서 이미 코드에 들어간 것(테스트 green, 커밋 안 함)과 §10.4 대비:

**KEEP (검증됨, 유지):**
- `modules/tasks/pick_and_place/geometry.py` reachable-orientation tilt 가족(top-down 폐기,
  접근축 pre) — §10.3-D 가 이 위에서 성립. 단 `plan_grasp` 가 **footprint(prismatic) 기반**
  이라 파지 선택부는 §10.4-2(표면 antipodal)로 교체 필요.
- `modules/motion/module.py` `resolve_reachable` cheap→expensive 게이트(floor_z / linear /
  solutions 반환) + `Kinematics.floor_collision` + MoveL `_linear_path_blocker`(jump) — 유지.
- `modules/tasks/pick_and_place/steps.py` grasp-frame 동작(advance/withdraw 접근축) + home
  waypoint 경유 — 유지.
- `DETECTOR.FUSE_ORIENTED` 멀티뷰 점군 융합 개념 — 유지(단 그 위 파지는 antipodal 로).
- detector object-centric(floor ring 삭제) — 유지.

**REPLACE (틀린 방향, 교체):**
- `geometry.py target_view_poses` (고정 반경/고도 6점 궤도) → **adaptive 도달성 기반 뷰
  탐색 + floor/object 충돌 스크리닝** (§10.4-1). 이게 이 세션 최대 삽질 지점.
- `steps.py observe_target` 궤도 순회 + `require_plausible_height` height 게이트 →
  **adaptive 정지(파지 서면 멈춤)** (§10.4-1,6).
- `plan_grasp` 윗면 footprint 파지 → **표면 antipodal 파지** (§10.4-2). prismatic 전용.
- detector `object_metrics_from_points` 의 2-percentile bottom → **z-gap 군집** (§10.3-F).

**ADD (신규):**
- 표면 antipodal 파지 선택기(open3d 법선 + 샘플, 조 축 수평 필터) — §10.2 파라미터가 출발점.
- 접촉쌍 → 단일조 TCP pose 변환 + tilt 스윕 + 그리퍼-물체/이웃 충돌 게이트 (§10.3-D 코드가
  프로토타입).
- 뷰 간 충돌 인지 이동 (§10.4-4).
- "안전 파지 불가" 실패 경로 (§10.3-I).

**backend.md 진행 status "2026-07-14 (2)" 부는 옛 방향(단일뷰/궤도)을 "구현 완료"로 적어둠
— §10 기준으로 그 부도 갱신 필요.**

### 10.6 하드웨어에서만 판명 (sim 불가 — 실물 세션 몫)

- **파지 물리 안정성**: 마찰 / force-closure 여유 / 무게중심 / 슬립. 기하 antipodal ≠ 안정 파지.
- **실 depth 재질 실패**: 반사·투명·어두운 표면 dropout (Gaussian+outlier 는 모델링, 재질
  의존은 불가).
- **실 SAM/GDINO 마스크 품질**: 실 텍스처에서 타깃 세그멘테이션 정확도(bleed/누락은 모델링).

### 10.7 검증 스크립트 (repo 이관 완료 — 재현 가능)

**[backend/scripts/grasp_verify/](../backend/scripts/grasp_verify/)** 에 영속 이관됨
(README 가 스크립트↔§10.3 A~I 매핑 + 실행법). 경로는 `__file__` 기준이라 집/회사 무관.

```powershell
cd backend
uv run --no-sync python scripts/grasp_verify/comprehensive_verify.py  # 예: end-to-end 48/48
```

A `shape_generality` / B `verify_antipodal_multiview` / C `verify_reachable_multiview` /
D `verify_grasp_execution`·`comprehensive_verify` / E `verify_grasp_exec_noise` /
F `outlier_phantom`·`mask_bleed` / G,H `verify_motion_stopping` / I `verify_clutter`.
검증/프로토타입 코드(프로덕션 아님) — antipodal 선택기·접촉쌍→TCP 변환은 여기서 출발.

**구현 후 추가: `verify_production_pipeline.py`** — 프로토타입이 아니라 **실제
모듈 코드**(detector z-gap → view_directions/view_pose_groups → antipodal →
plan_grasp → motion 게이트 프리미티브)를 실 캘 kinematics + 렌더 부분 점군으로
end-to-end: 3위치 × 4형상 × 클린/노이즈 = 24/24 파지 성립, 전 케이스 2뷰 정지.
