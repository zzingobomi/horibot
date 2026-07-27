# omx handover 실물 테스트 핸드오프 (2026-07-25)

> ⚠️⚠️ **2026-07-27 재전환: 큐브 → 8×2×2cm 파란 각봉 + B/down 수직 제시** —
> **§T.7 이 최신 상태다 (여기부터 읽어라).** §T.5 의 "펜 아키텍처 부활(수평
> 제시)" 계획은 probe 로 **뒤집혔다** (수평 봉은 so101 공중 수취 도달 0).
>
> ⚠️ **2026-07-26 물체 전환: 펜 → 2cm 큐브** (사용자 지시 — 동그란 펜은 omx
> 평행조로 안정 파지 어려움). 아래 본문의 "펜" 서술은 대부분 **큐브로 대체**됐다:
> omx 가 큐브 중심을 top-down 으로 집어 제시 → so101 이 omx 가 문 면이 **아닌
> 직교 면**을 받는다 (regrasp). 코드 전면 반영 완료(sim green, 실물 미검증) —
> `pen.py` 삭제 → `frames.py`+`cube.py`, steps/module 큐브화. **여전히 유효한 §**:
> 3-1 그리퍼 과부하 / 3-2 J5 / 3-4 포트 / §2 table_z·torque·닫힌조 오검출 /
> §4 배포 / §8 프로세스 원칙. **바뀐 것**: §3-6·§1 펜 파지 자세 → 큐브 중심 파지,
> 근접 여유 = 큐브라 구조적 tight(best~8mm, margin 5mm, 실물 재특성화 필수).
> 핵심 지시 유지: **`stop_before_receive` 로 omx 집기+제시를 먼저 눈으로 확인**한
> 뒤 수취를 켠다. 정본 = [project_omx_handover_cube_pivot 메모리] + cube.py docstring.
>
> **다른 세션이 그대로 이어받는 문서.** 이 세션(2026-07-25 새벽)은 omx handover 를
> `stop_before_receive=true` 로 **omx 집기+제시만** 반복하며 실물 디버깅했다. 아래는
> 되는 것 / 안 되는 것 / **AI 실수 기록** / 배포 상태 / 다음 착수 순서.
> 설계 배경 정본 = [omx_handover_prep.md](omx_handover_prep.md), task 아키텍처 = [task.md](task.md).

---

## §T. 2026-07-26 저녁 — 큐브 공중 수취 실물 디버깅 (다음 세션 필독, 오늘의 실수)

> 이 날 omx 집기+제시는 실물 OK 가 됐고, **so101 공중 수취가 막혔다.** 아래는
> **왜 막혔나 + 오늘 저지른 실수들** — 같은 함정 반복 금지.

### T.1 오늘 구현된 것 (PC + pi_hori3, 실물 반영)
- 펜→큐브 (frames.py 객체무관 / cube.py 큐브기하 / pen.py 삭제).
- 제시 높이: 책상근처(옛) → 악수높이 → 최종 **so101-도달 robust 지점 직접 겨냥**
  (frames.rendezvous_candidates `prefer_point`, steps `_RENDEZVOUS_PREFER_XY`).
- 수취 = **위/아래 면 수직 조축 파지** (`_RECV_FAMILY` 일반 grasp 샘플러).
- **safe torque enable** (dynamixel `set_torque`: 토크 켜기 전 Goal←Present) —
  시작 "딱"(슬램) 제거. 파지 Z = floor 게이트 + 골무 오프셋(URDF 미모델 손끝 연장).
  관측 저각(elev 30/25)+so101측 방위 — J2 떨림 제거. 검출 score 0.35→0.25.
  omx motion.yaml 감속. 벽(뒤) 게이트 `_behind_wall`.

### T.2 되는 것 (실물)
- omx 관측→DETECT_PLANAR→집기(바닥 안 긁게 ~14-20mm)→제시, verify_grasp ×2 통과.
- 시작 슬램 없음(safe-enable). J2 떨림 없음(관측 저각).

### T.3 **왜 막혔나 — 근본 실수 2개 (핵심)**

**M1. omx 카메라가 URDF/충돌 모델에 아예 없다 — 최악의 실수.**
[robot/omx_f/urdf/omx_f.urdf](../robot/omx_f/urdf/omx_f.urdf) 링크 = world/tcp/link0~7
**뿐, 카메라 없음.** so101 은 D405 홀더가 URDF 에 있는데 **omx 카메라만 통째로 빠짐**
(비대칭). → CrossRobotChecker 충돌검사·occlusion probe·"여유 15mm"·"관측 비가림"
**전부 카메라를 안 본 허수.** 실물에선 so101 이 omx 카메라에 부딪히거나 카메라가
큐브를 가리는데 sim 은 모른다. **사용자가 사진 보고 잡아냄** ("그리퍼 열면 omx
카메라 부딪히겠고"). → **다음 세션 최우선 = omx 카메라를 충돌/occlusion 모델에
추가.** 위치는 안다 (omx hand_eye `T_tcp_cam`), 근사 box 라도 넣어야 — 안 넣으면
이후 모든 sim probe 가 계속 거짓말한다.

**M2. so101 공중 도달이 razor-thin — 어디서든 192 자세 중 3~5개뿐.**
(scratchpad so101_robust_reach_probe). 랑데부를 omx/전방x/반경으로 고르면 실 큐브
(omx 그립으로 계획점에서 **~3cm 어긋남**)에서 도달 **0**이 된다(실측: 제시점 3개
→ 검출큐브 0개). → 랑데부는 so101 도달 robust 지점을 직접 겨냥해야(M2 대응으로
prefer_point 도입). **그런데 이게 M3 와 정면 충돌:**

**M3. so101 도달 ↔ 검출(가림)이 y축에서 정반대로 당긴다 — 공중 직접 수취의 핵심 벽.**
- so101 도달 robust = 큐브를 **omx 쪽(y≈0.26)** 으로.
- 검출 비가림 = 큐브를 **omx 반대(y≈0.18)** 로.
- 실측 2점: y0.18 랑데부 → 검출 후보 **1개**(score 0.327), **y0.27 랑데부 → 후보 0개**
  (완전 가림, `debug/handover/20260726_233827`). 즉 도달 살리려 omx 쪽 당겼더니
  검출이 죽었다. **이 텐션 + 미모델 카메라(M1)** 가 지금 벽의 정체.

### T.4 부수 실수 (다 고침 — 원인만 박제)
- **파지 바닥 훑음**: 물리 손끝이 TCP(link5 x 9.19cm)보다 **~11mm 아래** + **골무**
  (URDF 미모델) → TCP=1cm 면 손끝이 바닥. floor 게이트를 `table+여유+골무오프셋`
  으로, 파지 Z ~14-20mm (probe omx_pick_z_probe: 14mm 부터 손끝 클리어).
- **시작 "딱"**: 토크 enable 시 Goal Position≠Present(sag) → profile=0 슬램. → safe-enable.
- **J2 떨림**: 관측 자세가 너무 높고 뻗음(elev 40/55) → 어깨 부하. → 저각(30/25).
- **검출 실패(1차)**: 진짜 검출인데(116점, 위치 정확) GDINO score 0.327<문턱 0.35.
  → 0.25 (위치/z/반경/점군 게이트가 오검출 방어).

### T.5 다음 세션 착수 순서 — **길쭉한 블록으로 전환이 1순위** (2026-07-26 사용자 안)

**핵심 전환: 2cm 큐브 → 짧은축 2cm × 긴축 5~10cm(권장 8~10cm) 각봉(블록), 파란색
3D 프린트.** 이게 M1/M2/M3 를 **한 번에** 푼다 (사실상 펜 아키텍처 부활 + 펜의
치명약점[둥글어 미끄러짐] 을 각단면으로 해결):
- **M3(도달↔가림 텐션) 소멸**: omx 가 **한쪽 끝**을 물면 so101 은 **반대쪽 끝(5~10cm
  떨어짐)** 을 문다. 노출 끝은 omx 그리퍼·카메라에서 멀어 so101 이 **보이고(비가림)
  + 닿는다(도달)** — 두 그리퍼가 같은 2cm 점에 모이던 게 원인이었다.
- **M1(카메라 충돌) 완화**: so101 이 먼 끝을 잡아 omx 카메라와 이격 (그래도 카메라는
  모델에 넣을 것).
- **검출**: 파란색(흰 그리퍼·배경과 대비) + 큰 물체 → GDINO score↑, 점군↑ (오늘 score
  0.327 마진부족·후보0 둘 다 해소). 프롬프트 "blue block/bar". 흰 그리퍼 오검출도 회피.
- **파지 안정**: 2cm **각**단면 = omx 평행조가 확실히 문다 (둥근 펜 폐기 이유 해결).

**구현 = 펜 아키텍처 부활** (git history 에 `pen.py` + 펜-era steps 있음):
- 살릴 것(펜-era): 먼 끝 frac 파지 / 노출 세그먼트 / 제시 시 노출 끝을 so101 로 /
  so101 이 노출 끝 재검출·파지 (plan_pen_grasp/PenGrasp/_present_orientations 접선족).
- **오늘 만든 객체무관 인프라·수정은 유지**(재사용): frames.py(변환/랑데부),
  **safe torque enable**(dynamixel), **motion.yaml 감속**, **파지 Z floor+골무 오프셋**
  (블록 pick 도 필요), wall 게이트, 관측 저각(가림 완화), trace. 큐브 전용(cube.py
  중심파지/`_RECV_FAMILY` 수직조축/prefer_point robust)만 펜식으로 교체.
- 블록 치수로 펜 게이트 조정 (footprint 긴변=블록 길이, 짧은변 2cm).

**그 다음 (블록으로도 남으면)**: ① omx 카메라 충돌/occlusion 모델에 추가(M1 — hand_eye
오프셋 + 근사 box), ② 카메라 넣고 재-probe.
⚠ **스탠드 중계는 사용자가 거부**(양팔 취지) — 공중 직접 수취 유지.

### T.6 프로세스 교훈 (나 자신 — 반복 금지)
- **충돌/시야 probe 전에 그 형상이 URDF 에 있는지부터 확인하라.** omx 카메라가
  URDF 에 없는 걸 안 보고 clearance/occlusion 숫자를 "정답"처럼 사용자에게 넘겼다
  → 사용자가 사진으로 잡아냄. sim 결과는 **모델에 든 것만** 본다 (so101 D405 는
  있고 omx 카메라는 없는 비대칭을 놓침).
- 관측/파지가 얽힌 기하는 offline probe 로 좁히는 게 유효했다(높이·저각·robust
  지점 다 probe 로 잡음) — **단 모델이 실물과 같을 때만.** 카메라 누락이 그 전제를
  깼다.

---

## §T.7. 2026-07-27 — 봉(8×2×2) 전환 구현 완료 + **B/down 수직 제시 채택** (§T.5 계획 뒤집힘)

> 사용자가 8×2×2cm 파란 각봉을 3D 프린트하기로 확정 → 큐브 코드를 봉으로 전면
> 전환 + frontend /tasks/handover 페이지 신설. **sim/probe 로 증명된 상태로 집
> 실물 테스트 대기.** 실행 절차 = §T.7.4.

### T.7.1 §T.5 의 "펜 아키텍처 부활(수평 제시)" 은 probe 가 기각했다

M1 수정(omx 카메라를 URDF 근사 box 로 추가 — `robot/omx_f/urdf/omx_f.urdf`
`camera` link, origin=hand_eye T_tcp←cam) **후에** 결합 probe
([backend/scripts/handover_block_probe.py](../backend/scripts/handover_block_probe.py) —
omx 제시 도달 × so101 수취 도달([pre,grasp]) × cross 충돌(margin 8mm) × 관측
rayTest 비가림, production IK/checker/캘 DB 그대로) 를 돌린 결과:

| 제시 자세족 | omx 도달 | so101 수취 | 충돌 클리어 | 관측 비가림 |
| --- | --- | --- | --- | --- |
| A 수평(접선 +t) | 171 | **0** | 0 | 0 |
| A 수평(접선 −t) | 167 | 5 | **0** | 0 |
| **B/down (수직, 노출 끝 ↓)** | 62 | 38 | **23** | **23** |
| B/up (수직, 노출 끝 ↑) | 53 | 7 | 5 | 5 |

- **수평 봉이 죽는 이유** = 큐브 시대 실측과 같은 뿌리: so101 공중 수취 도달해는
  전부 **수직 조축** — 수평 봉은 조축 수평을 강제해서 so101 이 못 받는다.
- **B/down**: omx 가 봉을 수직으로 세워 **아래로 늘어뜨려** 제시 → so101 이
  아래 노출부를 수평 접근·수직 조축으로 받는다. 보너스: ① 매달림 = 중력 모멘트
  0 (조 안 회전력 없음) ② 수직 봉 = 방위 대칭 → "so101 로 조준"/yaw 180° 모호성
  소멸 ③ omx 그리퍼+카메라가 E 위 ~4.5cm 라 저각 시선과 구조적 비겹침.
- robust 밴드: omx TCP **xy (0.18~0.24, 0.16) 이 z 3단(0.28/0.30/0.32) 전부
  통과** (큐브 3~5/192 와 자릿수 다름). 수직 봉은 omx TCP 와 so101 파지점 E 가
  같은 xy — 두 팔이 동시에 편한 지점이 존재하게 된 것 (M2/M3 소멸의 실체).
- ⚠ probe 방법 교훈: 자세 후보는 omx 5DOF **도달 다양체 위에서 구성**해야
  (임의 방위에 IK 를 요구하면 5DOF 에선 measure-zero 라 전멸 — 1차 probe 실수).

### T.7.2 구현 (전부 sim green — backend 456+16, frontend 188)

- **backend**: `cube.py` 삭제 → [block.py](../backend/modules/tasks/handover/block.py)
  (양 끝 파지 후보/노출 오프셋/짧음 명시 실패). steps.py: 검출 게이트(긴 변
  5~12cm + 종횡비 ≥2), 끝 파지(tool z ∥ u — 양 끝 동등 후보 × Z 사다리), 제시
  = B/down 단일 자세(`_present_quat_down`, prefer (0.21,0.16) z 0.30/0.32/0.28),
  E = TCP − (0,0,tcp_to_e≈4.5cm) = 재검출 겨냥점, 수취 = 수직 조축 spin 사다리
  (`_recv_orients`, base→E 방위 진입 선호). 골무 오프셋을 floor 게이트에 실제
  배선 (그전엔 주석만 있는 죽은 노브 — +2mm 보수).
- **frontend**: FRONTEND_EXPOSED 에 handover 12키 노출 + `/tasks/handover`
  페이지 (Sidebar Tasks 링크) — HandoverPanel(pick/place/**stop_before_receive
  토글, 기본 ON**) + HandoverProgressPanel(TaskProgressPanel 코어를 task 키
  묶음으로 파라미터화 — PnP 와 공용) + 검출 오버레이 카메라.
- **관측성**: knob_snapshot 이 봉 노브 전체를 trace summary 에 각인, 각 계획
  step 이 봉 기하(ends/length/exposed/chosen_dz/chosen_u/alpha/tcp_to_e)를
  trace 에 기록 — 첫 실물 런이 깨져도 trace.jsonl 만으로 원인분석 가능.

### T.7.3 실물 미지수 (sim 이 못 준 것 — 첫 런에서 데이터로 확인)

1. **픽(봉 수평)→제시(봉 수직) 재배향 스윙 중 봉 끝 책상 스침** — move_j 관절
   보간 경로는 봉을 모델링하지 않는다. stop_before_receive 런에서 눈 확인,
   긁으면 `_PRESENT_Z_WORLD` 상향/경유 자세 추가 (steps.py docstring 가정 ⑤).
2. 파란 봉의 실 GDINO score (게이트 0.45/0.25 는 큐브 실측 기반 — trace 로 확인).
3. omx 조가 8cm 봉을 실제로 유지하나 (계산상 모멘트 ~0.01Nm 로 여유).
4. **M1(omx 카메라 충돌/가림 모델) 미반영** — 2026-07-27 URDF 에 넣었다가 되돌림
   (공유 URDF 오염: visual 박스가 전 3D 뷰에 뜸 + omx self-collision·IK 영향).
   제대로 된 자리 = CrossRobotChecker 가 로드 후 wrist 에 collision-only box 를
   붙이기 (URDF 불침범). **미구현 — 현재 probe/충돌 게이트는 카메라를 안 본다.**
   B/down 은 봉 이격(~4.5cm)으로 카메라와 구조적 여유가 있어 첫 런은 진행 가능
   하나, so101 이 omx 카메라에 닿는지는 **눈으로 확인** 필수 (M1 사고 재발 주의).
5. §3 의 기존 미지수 유지: held 임계(characterize 미실행) / motors.yaml
   current_position+500 배포 상태 / J5 리밋 여유.

### T.7.4 집 실행 절차

1. `git pull` → **PC 재시작 필수** (handover 봉 코드는 PC 에서 돎 — §4). omx
   URDF 는 안 바뀌었다(카메라 추가는 되돌림, T.7.3-4) — pi_hori3 는 dynamixel
   assert 사소 변경뿐이라 재부팅은 선택. motors.yaml current_position+500 이
   아직 미배포면 이번에 같이 재부팅(§4). frontend 는 재빌드/새로고침.
2. 파란 봉(8×2×2cm)을 omx 도달영역 중심 근처(관측 look=(0.21,0) 부근)에 눕혀 둠.
3. frontend `/tasks/handover` → **stop_before_receive ON** 으로 [실행] — omx
   집기+수직 제시를 눈으로 확인 (봉 끝 책상 스침/파지 유지/제시 자세).
   실패 시 `debug/handover/<ts>/trace.jsonl` + summary.json 분석.
4. 통과하면 토글 OFF → so101 수취까지. place 를 채우면 적치까지.
5. (선택) 위치를 옮겨가며 재현성 확인. 필요 시
   `uv run --no-sync python scripts/handover_block_probe.py` 로 재특성화.

---

## 0. 어떻게 이어받나 (30초)

1. **§4 배포 상태 먼저 읽어라** — PC 와 pi_hori3 가 다른 코드로 떠 있(었)다. 이게 혼란의 절반.
2. **§8 (AI 실수/프로세스) 읽어라** — 같은 함정 반복 금지. 핵심: **데이터(trace/log/probe/이미지) 먼저 읽고 이론은 그 다음.**
3. **§7 착수 순서**대로. 실물 시작 전 **모터 하드웨어 에러 상태부터 데이터로 확정** (추측 금지).

---

## 1. 한눈에 (되는 것 / 안 되는 것)

**되는 것 (실물 검증):**
- omx 관측 자세 이동 + mono z=0 검출 (펜 깨끗이 잡힘, score ~0.76) — **단, 그리퍼가 열려 시야 밖일 때만** (§3-5).
- 파지점 계획 (Z 사다리로 바닥 클리어하는 z=8mm 채택) → omx 가 실제로 집으러 내려감.
- **handover task end-to-end `success` 도달 (03:48 런)** — omx 집기→제시까지. **단 그때 그리퍼가 position(풀토크) 모드라 과부하 위험 상태**였음 (§3-1).

**안 되는 것 / 미해결:**
- 그리퍼가 **오래 홀딩 시 과부하**(red LED, 모터 셧다운). position 풀토크 탓. → current_position+goal current 로 고치는 중, **미배포·미검증** (§3-1).
- **motor 5(J5) 도 과부하로 뻗음** — 추정: top-down 파지가 도달 한계 끝이라 J5 를 리밋으로 몲 (§3-2).
- **파지 자세가 부자연스러움** — omx 가 집으러 갈 때 "뒤로 젖힌" 듯 어색한 자세 (§3-6, §3-2 와 한 뿌리).
- 파지 판정(held) 임계가 아직 Feetech 값 → **거짓 "held=true"** 가능 (§3-3).
- 전원 사이클 후 `/dev/ttyACM0` 재인식 실패 (운영 이슈, §3-4).

---

## 2. 실물에서 검증된 사실 (재확인 불필요)

- **omx IK = 해석적** (pi_hori3 부팅 로그 `IK=해석적(ZYYYX-5R(closed-form))+polish`). numeric 아님. (EAIK 도 이번에 hori3 설치됨.) → "flaky/느림"을 numeric 탓으로 돌리지 말 것.
- **Dynamixel 은 전원 on 시 torque OFF 기본.** 그래서 시나리오 시작에 `enable_torque` 스텝 추가함 (so101 Feetech 는 기본 on 이라 무해). 헤드리스 task 는 자기 참여 robot torque 를 스스로 켜야 함.
- **omx top-down 도달은 책상면 얇은 밴드뿐.** 실측(probe): 파지점에서 z=6mm→바닥충돌, **z=8mm→도달**, z≥10mm→미탐. 즉 파지 Z 는 "펜 지름 중심(pen_d/2)"이 아니라 **손끝이 책상 닿는 높이(≈8mm)** = 조가 펜을 감쌈. (pen_d/2 로 잡으면 손가락이 바닥 관통 → 바닥충돌 기각.)
- **eye-in-hand 시야에 닫힌 흰 그리퍼가 있으면 GDINO 가 그걸 "pen"으로 오검출**(356mm 가짜 후보). 그리퍼를 먼저 열어 시야에서 빼야 실제 펜이 잡힌다. (03:07 실패=닫힘, 02:31/03:48 성공=열림 이미지 대조로 확정.)

---

## 3. 열린 문제 (우선순위)

### 3-1. 그리퍼 무는 힘 / 과부하 — **최우선**
- **증상**: 그리퍼가 오래 물면 과부하 에러(red LED) → 토크 셧다운 → 그 뒤 open/close 명령에 **위치 안 변하고 load=0** (probe 로 확정: `pos=2050 고정, load=0`).
- **원인**: 내가 그리퍼를 `position`(풀토크)로 바꿨더니, 스톨 홀딩 시 전류한계까지 밀어 과부하. (원래 config 는 `current_position` 이었는데 **드라이버가 goal current 를 안 걸어 死문서**였음 → 힘없이 안 열림 → 내가 position 으로 도망 → 과부하. 삽질.)
- **수정(코드엔 반영, pi_hori3 미배포)**: 그리퍼 = `current_position` + **Goal Current 500** (힘 상한). 드라이버가 open 마다 operating mode(11) + goal current(102) 를 실제로 씀 (`_apply_operating_mode`).
- **미검증**: 03:48 성공은 **아직 position 모드**(load=-924 가 증거 — 500 상한이면 ~500 이어야 함). current_position+500 은 pi_hori3 재부팅해야 적용.
- **다음**: pi_hori3 배포+재부팅 → 부팅 로그 `operating mode 적용: {6: ('current_position', 500)}` 확인 → 재실행. **500 은 추정값**: 펜 못 들면(약함) `robot/omx_f/motors.yaml` 그리퍼 `current:` 올리기(~791 아래), red LED 또 뜨면 내리기.

### 3-2. motor 5 (J5) 과부하
- **증상**: J5(손목 roll) 도 "죽음"(red LED 추정).
- **가설(미확정)**: top-down 파지가 도달 한계 끝(z=8mm 딱 하나)이라 arm 이 그 자세를 버티며 J5 를 리밋 근처로 몲 → 과부하. 반복 시도 + 전원 사이클이 누적 스트레스.
- **다음**: ① J5 가 진짜 과부하 에러인지 **하드웨어 에러 레지스터**로 데이터 확정(추측 금지). ② 채택된 파지 자세의 **J5 리밋 여유**를 정량 확인 — 여유 없으면 파지 자세 재설계(도달 여유 큰 쪽 선호 / 관측 look 점 조정). ③ current_position 힘 상한이 arm 모터에도 필요한지 검토(현재 arm 은 mode 미선언=건드리지 않음).

### 3-3. 파지 판정(held) 임계 — omx 값 아님
- verify_grasp 이 `thr=1840`(close+5%) + load 로 판정하는데 이는 **so101 Feetech 기준**. omx(XL330 Present Current)는 스케일/부호 다름 → **거짓 "held=true"** 남 (실제 못 잡아도 성공 보고).
- **도구 준비됨**: `scripts/gripper_characterize.py --robot omx_f_0` — 빈손 vs 펜물림 gap·전류를 `debug/gripper_characterize/omx_f_0_*.json` 로 저장(내가 저장 기능 추가함). **아직 안 돌림.**
- **다음**: characterize 돌려 파일 나오면 그 값으로 omx held 임계 배선. 그 전엔 **task status 말고 눈으로** 파지 판단.

### 3-4. `/dev/ttyACM0` 재인식 (운영)
- 전원 사이클 후 U2D2 가 사라지거나 다른 번호로 잡힘. 코드 문제 아님.
- **처방**: hori3 에서 `ls /dev/ttyACM*` → 없으면 omx 전원/USB 재연결, `ttyACM1` 등이면 U2D2 재삽입(보통 ACM0 복귀) 또는 `robot/instances/omx_f_0/instance.yaml` port 를 그 번호로. USB 준비된 뒤 backend 기동.

### 3-6. 부자연스러운 파지 자세 (뒤로 젖힘) — §3-2 와 한 뿌리
- **증상**: omx 가 집으러 갈 때 자세가 어색함 ("뒤로?" 젖힌 듯, 자연스럽지 않음, 2026-07-25 사용자 관찰).
- **가설(미확정)**: ① top-down 파지가 도달 한계 끝이라 IK 가 **contorted branch**(팔꿈치 뒤/손목 꼬임)로만 해를 냄. ② 해석적 IK branch 열거 후 채택이 **자연스러운/현재 자세에 가까운 해를 선호하지 않음** — 여러 branch 중 아무거나. ③ 관측 자세(높은 nadir)→파지(낮은 책상) move_j 대재구성이 이상한 경유.
- **다음**: ① 채택 branch 를 **nominal 자세/현재 config 에 가장 가까운 것으로 선호**(seed 기반 선택 — resolve/폴리시 확인). ② 근본은 §3-2 와 동일(파지가 여유 없는 경계) → 펜을 omx **편한 도달 영역**(가까이·중심)에 두거나 관측 look 점 조정으로 여유 확보. ③ move_j 대신 경유 자세를 두는 것도 검토(단 top-down 은 책상면만 도달 주의).
- ⚠ 자세가 리밋 근처면 J5 과부하(§3-2)와 직결 — **둘을 같이** 본다.

### 3-5. (부수) 검출 강건성 / 펜 배치
- 닫힌 그리퍼 오검출(§2) → 그리퍼 먼저 open. 펜은 omx 도달영역 중심(관측 look=(0.21,0) 근처)에 둬야 시야 중앙. 구석에 두면 실패.

---

## 4. 배포 상태 (⚠ 혼란의 근원 — 꼭 확인)

분산: **PC = handover/detector/task**, **pi_hori3 = omx motor+motion+camera**, pi_hori1/2 = so101.

| 변경 | 도는 곳 | 배포됨? |
| --- | --- | --- |
| handover 슬림 pick + Z 사다리 + stop_before_receive | **PC** | ✅ (03:48 런 = pid 11592, 반영됨) |
| 그리퍼 operating mode 명시 적용 (driver) | **pi_hori3** | 🟡 position 모드로만 부팅됨(03:30~) |
| 그리퍼 current_position + goal current 500 | **pi_hori3** | ❌ **미배포** (config 는 바뀜, hori3 재부팅 안 함) |

→ **다음 세션: pi_hori3 에 최신 코드+`robot/omx_f/motors.yaml` 동기화 후 재부팅**해야 current_position+500 이 실제로 돈다. (04:02 재부팅 시도는 ttyACM0 에러로 실패.)

---

## 5. 이번 세션 코드 변경 (파일별)

- [config/deployments/pc.yaml](../backend/config/deployments/pc.yaml) — handover 모듈 주석 해제(활성화).
- [modules/tasks/handover/steps.py](../backend/modules/tasks/handover/steps.py) — 슬림 pick:
  - `plan_omx_pick_pen` = 파지점만 resolve + **Z 사다리(0.007~0.012, 첫 도달 채택)**. top-down pre/lift 폐기.
  - `omx_pick_pen` = 관측→파지 해로 **move_j 스윙인** → close → 판정. refine(look-then-move) 제거.
  - `enable_torque` 스텝 신설. pen_d clamp 상한 12mm(실측 10mm 펜). 死노브 정리(_OMX_PRE_ABOVE_M/_OMX_LIFT_M/_REFINE_MATCH_RADIUS_M).
- [modules/tasks/handover/module.py](../backend/modules/tasks/handover/module.py) — 시작에 `enable_torque`(so101/omx), `stop_before_receive` 파라미터+제시 후 조기 return.
- [modules/tasks/handover/contract.py](../backend/modules/tasks/handover/contract.py) — `RunRequest.stop_before_receive`.
- [modules/motor/layout.py](../backend/modules/motor/layout.py) — `MotorSpec.mode` / `MotorSpec.goal_current`.
- [apps/config.py](../backend/apps/config.py) — motors.yaml `mode`/`current` 파싱.
- [modules/motor/drivers/dynamixel.py](../backend/modules/motor/drivers/dynamixel.py) — `ADDR_OPERATING_MODE`(11)/`ADDR_GOAL_CURRENT`(102), `_apply_operating_mode`(선언 모터만 mode+goal current 적용, torque off 후 EEPROM write).
- [robot/omx_f/motors.yaml](../robot/omx_f/motors.yaml) — 그리퍼 `mode: current_position` + `current: 500`.
- [scripts/gripper_characterize.py](../backend/scripts/gripper_characterize.py) — 결과 JSON 저장 추가.
- [tests/modules/test_handover.py](../backend/tests/modules/test_handover.py) — 슬림 pick/enable_torque/stop_before_receive 반영 (13개 green).

테스트/lint/type 전부 green 상태로 커밋 가능. sim 회귀는 통과, **실물 검증은 §1 대로 부분적**.

---

## 6. 핵심 데이터/증거 (재현·판단 기준)

- **03:48 성공 trace** (`debug/handover/20260725_034828/`): `chosen_z=0.008`, `group_failures=["바닥충돌"(7mm),""(8mm),미탐...]`, `verify_grasp load=-924 held=true`, `status=success`. **load=-924 = 아직 position 풀토크.**
- **바닥 충돌 Z 스윕**(probe): 6mm=바닥충돌 / 8mm=도달 / 10mm↑=미탐.
- **그리퍼 안 움직임 probe**: torque on + open/close 명령에도 `pos=2050 고정, load=0` → 모터 셧다운(과부하 에러) 상태의 지문.
- **검출 이미지 대조**: `debug/detect/20260725_023108`(그리퍼 열림=펜 정중앙 검출 OK) vs `20260725_030711`(그리퍼 닫힘=흰 조를 pen 으로 오검출).

실행: `/dev` 콘솔 `{ "pick_object":"pen", "stop_before_receive":true }` 또는
`uv run --no-sync python scripts/run_task.py srv/handover/run --deploy pc --param pick_object=pen --param stop_before_receive=true`.

---

## 7. 다음 세션 착수 순서

1. **하드웨어 정상화**: hori3 `ls /dev/ttyACM*` → 포트 살리고, **motor 5/그리퍼 과부하 에러 클리어**(전원 사이클) + **에러 레지스터로 진짜 과부하인지 데이터 확정**.
2. **pi_hori3 배포+재부팅** → 부팅 로그 `operating mode 적용: {6:('current_position',500)}` 확인.
3. **테스트 B 재실행** → 관측: 그리퍼 **열림?** / trace `load_raw` **~500?**(상한 걸림) / **red LED 안 뜸?** / 펜 실제로 들림?(눈).
4. **goal current 튜닝**: 약하면 `motors.yaml` `current:` ↑(~791 아래), 과부하면 ↓.
5. **gripper_characterize 돌려** held 임계 omx 값으로 배선 (거짓 held 제거).
6. **J5 과부하 / 부자연 자세**: 파지 자세 도달 여유 정량화 → 여유 없으면 자세/관측점 재설계 + **IK 채택 branch 를 자연스러운(현재 config 근접) 해로 선호**. (§3-2, §3-6 — 한 뿌리)
7. (그 다음) so101 수취 재개: 랑데부 점을 **책상 중앙**으로(현재 so101 좌측 끝 → 극단 스윙+"딱"), 공중 재검출(thin+가림) 검증. `stop_before_receive` 를 false 로.

---

## 8. AI(나) 실수 기록 + 프로세스 원칙 (다음 세션·나 자신 반복 금지)

이 세션의 최대 실패 = **데이터 안 읽고 추측 → 사용자에게 헛 테스트("똥개훈련") 반복**. 사용자가 여러 번 지적함. 구체 실수:

1. **추측 선행**: 도달성/파지 문제를 trace 읽기 전에 이론으로 단정. flaky IK 를 numeric 탓으로 오진(실제 analytic). "그리퍼 안 열림"을 mode 탓으로 단정했으나 probe 해보니 **모터 셧다운(load=0)** 이 진짜.
2. **모드 왔다갔다**: current_position → position(풀토크) → current_position. position 이 **과부하(사용자 red LED)** 를 유발. 증거 보고서야 정정.
3. **pen_d 과보정**: 지름 12mm clamp → 파지 Z 6mm → **바닥충돌**. Z 사다리로 재수정.
4. **축 오해**: "짧은축 중간"을 긴축 frac(0.5)로 착각, 되돌림.
5. **하드웨어 상태 미확인**: 모터 에러/포트를 코드 탓으로 돌림.

**원칙 (박제):**
- **데이터 먼저**: trace.jsonl / summary.json / 분산 로그 / resolve probe / detector 이미지(`debug/detect/`) 를 **읽고 나서** 가설. 실물 디버깅에서 이론은 데이터 뒤.
- **하드웨어 증상은 하드웨어부터**: red LED=Dynamixel 에러(과부하 등), load=0=토크 셧다운. 에러 레지스터/포트/전원 확인이 코드 의심보다 먼저.
- **설계/config 를 증거 없이 thrash 금지.** 한 번에 하나, 데이터로 검증하고 다음.
- **추측으로 사용자에게 실물 테스트 시키지 말 것.** resolve probe(로봇 안 움직임)·로그·이미지로 내가 먼저 좁힌다.
- **완료 보고 시 "실물로 증명된 것 vs 남은 미지수" 정직 분리** (CLAUDE.md 작업 방식 원칙).

---

*(정본 아님 — 실물 검증 진행 스냅샷. 안정화되면 omx_handover_prep.md §9 / task.md §4 로 흡수.)*
