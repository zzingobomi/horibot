# 집기(Grasping) 연구 — 설계 · 근본원인 · 해결 (SSOT)

> **파지 관련 SSOT.** 2026-07-15 통합: 옛 `grasp_redesign_journey.md`(설계) +
> `grasp_failure_analysis_2026-07-14.md`(실물 실패 현장 로그) + `grasp_rootcause_2026-07-15.md`
> (근본원인)를 여기 하나로 합치고 과정 detail 은 쳐냈다. 삭제분 원문은 git history 로 복원 가능.
> 히스토리는 §8 에 압축.

---

## 0. 현재 status (한눈에)

- **설계**: object-centric 멀티뷰 표면 antipodal 파지 — **구현 완료, sim 전수검증 통과**(§1).
- **실물**: 2026-07-14 실 SO-101+D405 로 2.5cm 흰 큐브 파지 전패("허공을 집음").
- **근본원인 (2026-07-15 규명)**: **파지 코드는 무죄 [확정]**, 실패는 **D405 의 저텍스처 depth
  편향 [유력]** — 흰 큐브 점군이 통째로 ~16mm 위로 떠서 파지가 큐브 위 허공에서 닫힘(§2).
- **다음**: 큐브 텍스처 A/B 로 편향 확정 → 센서 대응(§4) → 잔여 정밀도는 closed-loop(§5).

---

## 1. 파지 아키텍처 (현재 설계 — 구현됨)

### 1.1 원리

**한 변 ~2cm 물체를 집어서 어딘가 둔다. 물체가 어디 있는지(책상/사람 손/공중) 가정하지
않는다.** (일반 형상 · 지지면 무관 — 핸드오버/공중 파지까지 커버가 설계 목표.)

- **object-centric**: 물체 3D 는 물체 자기 점군(mask→depth→base)에서만. 주변 바닥(floor) 추정
  폐기(책상 없으면 무너지고, 추측이라 부정확).
- **멀티뷰 필수**: 단일 뷰는 가려진 면에 depth 자체가 없다. 여러 각도 관측을 base frame 에서 융합.
- **표면 antipodal**: 가정한 윗면 footprint 가 아니라 **관측된 표면에서 마주 보는 두 접촉점**을 찾음.
- **reachable-orientation**: top-down 강제 폐기. 조 축 수평(옆파지) 유지 + 접근 자세는 그 위치에서
  도달 가능한 것 중 선택(tilt 스윕). SO-101 은 먼 리치서 손목을 수직으로 못 세운다.
- **충돌 게이트**: 바닥 + 그리퍼↔물체/이웃 점군 충돌. 못 물면 "안전 파지 불가" 명시 실패(맹목 금지).

### 1.2 데이터 흐름 — 파지 z 가 어디서 오나 (끝까지 추적)

```
detect            search waypoint 자세 전부 돌며 후보 누적 (선택 안 함)
select score      → coarse 타깃 (위치만 씀)
observe (adaptive) coarse 주변 close 뷰를 spread-first 로 누적, 매 뷰마다:
  FUSE_ORIENTED     관측 점군 정합 병합(뷰 중심차 평행이동 + 평균 앵커) → 융합 점군
  object_metrics    top_z=z 98pct / base_z=z-gap 군집 바닥 / position=(윗면 band xy, top_z)
  antipodal         open3d 법선 → 접근선 수평인 마주보는 접촉점 쌍 {mid, jaw_axis, width}
  plan_grasp        tilt×pair×flip 가족. grasp = pair.mid + 횡오프셋(단일 가동 조 보정)
  RESOLVE_REACHABLE IK + 바닥충돌 + 그리퍼↔물체/이웃 충돌 + 직선경로 게이트 → 첫 통과 채택
  → 서면 멈춤 (2~4뷰)
execute           home → pre(MoveJ) → open → advance(MoveL→grasp) → close → withdraw → home
```

**실행 파지 z = `c.grasp.z` = `pair.mid.z`(+수평 오프셋)** = 관측 표면 접촉점의 중점 z.
검출 `position.z`(=top_z)/base_z 는 파지 z 로 안 쓰임(뷰 타깃팅·floor 게이트·놓기용).
**파지 z 정확성은 융합 점군의 절대 z 정확성에 100% 종속** — 이게 §2 의 급소.

구현: [detector/](../backend/modules/detector/) (projection/geometry/module) +
[tasks/pick_and_place/](../backend/modules/tasks/pick_and_place/) (antipodal/geometry/steps).

### 1.3 왜 이 설계인가 (sim 전수검증 근거)

- **단일 뷰 "충분"은 prismatic(박스/세운 원기둥) 전용 꼼수** — 윗면 윤곽으로 가린 면을 추측.
  구 −5mm 과소 → 헛집음. 표면 antipodal 은 단일 뷰로 전 형상 0쌍, **3뷰 융합 시 196~405쌍**.
- **닿는 뷰만으로 커버리지 충분**: eye-in-hand 라 방위 180~300° 확보 → 반대편까지 관측.
- **end-to-end**: workspace 12위치 × 4형상(box/눕힌원기둥/구/L자 concave) **48/48 실행 가능**;
  노이즈(σ1mm)+bleed+outlier Monte-Carlo 6/6; 빽빽 clutter 는 충돌 게이트가 올바로 거부(fail-safe).
- **정지**: 뷰 하나씩 누적, 실행 가능 antipodal 서면 멈춤 → 2~4뷰. (고정 7뷰는 과함.)

---

## 2. 근본원인 — 2cm 큐브를 왜 못 집었나 (2026-07-15)

### 2.1 파지 코드는 무죄 [확정] — A/B/C 결정적 테스트

같은 **프로덕션 함수**(object_metrics/antipodal/plan_grasp)에 세 입력:

| 케이스 | 입력 | 점군 z | 실행 파지 z | 참 큐브(0~25mm) 안 |
| --- | --- | --- | --- | --- |
| A | 어제 실 융합 점군(편향됨) | 17~45mm | 23~38mm | **30%** (허공) |
| B | A 를 바닥이 z=0 되게 하강(**편향만 제거**) | 0~29mm | 7~21mm | **100%** |
| C | 완전 합성 클린 큐브(편향 0) | −1~26mm | 2~17mm | **100%** |

→ **동일 코드가 깨끗한 데이터엔 파지를 큐브 안에 100% 정확히 잡는다.** 실패는 오직 입력의
절대 z 편향 때문. object_metrics/antipodal/plan_grasp/실행에 **로직 버그 없음.**

### 2.2 depth 편향 [유력]

**흰색 무텍스처 큐브가 D405(=IR 프로젝터 없는 passive stereo)에서 표면이 카메라 쪽으로
~16mm near-bias** 되어 재구성 → 점군 통째로 위로(top 40~48mm, 바닥 16~23mm; 참 0~25mm) →
파지 mid 23~38mm 로 큐브 상단(25mm)보다 위 → 조가 허공에서 닫힘.

### 2.3 증거 (전부 하드웨어 0, 어제 debug 데이터로 재현)

1. **blue box 대조**: 같은 세션·캘·프레임에서 blue box 바닥 z≈0~6mm 정상, 큐브만 뜸 → 전역
   캘/FK 오차 아님(그럼 blue box 도 떠야).
2. **pose 독립**: 큐브를 6개 다른 자세로 봐도 top 43/45/48/45/43/38mm 로 일정(std~3mm) →
   자세(FK)가 아니라 물체 표면에 묶인 오차.
3. **grazing 뷰**(0008): 큐브 앞면을 통째로 보는데도 여전히 16~45mm → "부분관측"이 아니라
   "보이는 것도 위로 시프트".
4. **리서치**: D405 는 D415 와 달리 IR 프로젝터 미탑재 → 무텍스처 흰 표면은 passive stereo 최악.

### 2.4 가설 배제 (요약)

| 후보 | 판정 |
| --- | --- |
| 전역 캘 z offset | 반증 (blue box 정상) |
| per-pose FK/백래시 | 반증 (6자세 top 일정) |
| 좌표변환/depth_scale 버그 | 반증 (blue box+합성 클린 정상) |
| 계산 로직 버그 | 반증 (§2.1 B·C 100%) |
| 융합 스미어 | 부차 (파지 XY ±2mm·w 정확) |
| 부분관측 | 반증 (grazing 앞면도 뜸) |
| **depth 센서 near-bias (무텍스처)** | **지지 — 모든 증거 부합 (근본)** |
| **open-loop 이 편향에 무방비** | **지지 (2차 설계)** |

### 2.5 왜 sim 은 다 통과(48/48)했는데 실물은 전패했나

sim 부분점군은 참 표면 주변 **등방 Gaussian+outlier** 만 — depth 무편향. 실 D405 의 **재질
의존 계통 near-bias 는 모델링 안 됨**. 기하 파이프라인은 sim·실물 모두에서 옳게 동작 — 틀린 건
입력 depth 뿐. **그래서 어제 밤 기하 튜닝(융합 정합/antipodal 폭/놓기 자세)은 전부 downstream
이라 무효였다. 기하 파라미터를 더 만지지 말 것.**

---

## 3. 계측 — raw depth dump (2026-07-15 추가)

이전 dump 은 base 변환·다운샘플된 PLY 만 남겨 "센서 편향 vs 캘/프레임 오차"를 원본으로 못
되짚었다. → [detector/module.py](../backend/modules/detector/module.py) `_dump_debug_raw` 추가:
검출마다 `_color.png`(clean) + `_depth.png`(**16-bit aligned depth**) + `_mask_c{i}.png` +
`.json`(intrinsic + depth_scale + hand_eye cam→ee + TCP ee→base + 후보 기하). **왕복 검증 통과**
(저장물로 base 점군 재현 오차 0µm). → 실물 캡처에서 **raw depth 로 표면까지 실측 거리 vs
기대 거리를 mm 로 재서 편향을 직접 확정** 가능(파지 성패와 무관하게 센서를 고발).

---

## 4. 해결 방향

> 순서 원칙(2026-07-15): **원인부터 못박고(band-aid 로 가리지 않기) → (A) 센서 실제 해결 →
> 그래도 필요하면 (B) 보정.** closed-loop 를 먼저 넣으면 "센서가 틀린 걸 겨우 보정한 건지
> 원래 괜찮았는지" 못 가림 = confounding.

### 4.1 두 통 — 둘 다 지지면 무관 (object-anywhere 목표 일관)

- **(A) 센서를 실제로 고친다** — 물체가 테이블이든 손이든 무관. §4.2/§4.3.
- **(B) 센서 오차를 안고 성공률을 올리는 보정** — closed-loop(§5). 재관측이라 지지면 무관.

### 4.2 지금 내 상황에서 가능한 것

- **큐브 텍스처(진단+즉효)**: 면 전체에 촘촘한 랜덤 점/무늬(마커·스프레이). passive stereo 에
  매칭할 특징 공급. **A/B 로 근본원인 확정도 겸함**(§7).
- **조명/노출(HDR)**: 밝은 표면 saturation 억제. raw depth 가 "저텍스처"인지 "밝기"인지
  갈라주면 맞는 걸 고른다.
- **closed-loop**: §5 (센서 확인 뒤).

### 4.3 산업현장 참고 — 지금 내 장비론 불가, 미래용

> 아래는 실제 현장 표준. **현재 나는 장비가 없어 못 함** — 왜 도움되는지만 기록.

- **외부 IR/구조광 패턴 프로젝터**: 텍스처 없는 표면에 인공 패턴을 투사 → passive stereo 가
  매칭할 특징을 강제 공급. 무텍스처 문제의 정공법(진짜 D405 무텍스처 실패의 직접 해결).
- **active-stereo 센서(D415/D435 등, IR 프로젝터 내장)**: D405 와 달리 자체 IR 패턴을 쏨 →
  무텍스처 표면에서 훨씬 안정. 센서 교체로 근본 해결(단 D405 의 근접·정밀 이점은 포기).
- **텍스처 스프레이(예: 검사용 dry developer)**: 물체에 미세 무광 분말 → 일시 텍스처. 산업
  3D 스캔에서 광택·투명·백색 표면에 상용.
- **재질/조명 통제**: 무광 도장, 확산 조명 — dropout·saturation 감소.

### 4.4 ✗ 기각: support-plane 앵커

파지 높이를 테이블 평면 기준으로 앵커하는 건 "물체는 지지면 위" 전제를 되살려 **object-anywhere
목표(핸드오버/공중)에 위배**. 좁은 케이스용 crutch. **평면은 순수 충돌-안전 바닥으로만** 남기고
(그리퍼가 안전 z 아래로 안 가게) 파지 기하 계산엔 안 쓴다.

---

## 5. 정밀도 현실 & closed-loop 의 자리

2cm 물체는 캘 σ_t~7.5mm floor([calibration.md](calibration.md)) 대비 빡빡 → **절대 open-loop
로는 구조적으로 아슬아슬.** 센서를 고쳐 편향을 없애도, 잔여 오차(캘·백래시·조명)를 이기려면
결국 **closed-loop** 이 필요하다. 단:

- **appearance 기반이어야 함** — 편향된 depth 를 3D 재투영해 open-loop 로 가면 편향이 안 상쇄.
  이미지 특징(위치/크기)으로 접근하며 서보해야 depth 편향에 강건.
- **센서 확인 뒤에** — closed-loop 는 D405 를 고치는 기술이 아니라 성공률을 올리는 제어. 텍스처를
  줬을 때 closed-loop 없이도 성공해야 한다(그게 먼저). 산업도 센서를 최대한 정확히 한 뒤 성공률을
  더 끌어올리려 closed-loop 를 쓴다.

---

## 6. 검증 자산 (재현 — 하드웨어 0)

**[backend/scripts/grasp_verify/](../backend/scripts/grasp_verify/)** — 실 debug 데이터 진단 3종:
- `code_vs_data.py`(A/B/C 결정적 테스트, §2.1) / `reproduce_grasp.py`(실 PLY 파지 재현) /
  `analyze_ply.py`(세션별 z 분포, blue box vs 큐브 대조).

**옛 sim 전수검증군(11개)은 2026-07-15 제거** — 기하는 검증했으나(§1.3 근거) 실 병목(depth
재질 편향)은 sim 모델 불가라 **다 초록인데 실물은 전패**(false confidence). §1.3 의 수치(48/48
등)는 그 결과의 박제이고 스크립트 원문은 git history 로 복원 가능. **교훈**: PyBullet 부분점군
sim 은 참 표면 주변 Gaussian+outlier 만 — **재질 depth 편향(반사·저텍스처·투명 dropout)은
못 담는다.** 그게 §2 가 물린 지점(하드웨어에서만 판명). 이후 파지 검증은 **실 debug 데이터 우선**,
sim 은 필요 시 "기하 회귀"만 최소로.

---

## 7. 액션 플랜 — 오늘 밤(집) → 판정 → 통과 후(구현)

### 7.1 오늘 밤 집에서 (사용자)

1. **큐브에 면 전체 촘촘한 랜덤 텍스처** — 마커 점 무늬 or 검사용 dry 스프레이(몇 줄 X/Y/Z 보다
   면 전체 랜덤이 유효).
2. **텍스처 전/후를 같은 자세에서** 파지 run — 통제 A/B(자세 고정해야 FK 오차 안 섞임). 새 dump 이
   raw depth+pose+mask+json 을 자동 저장하므로 **코드 변경 없이 평소대로 실행**하면 됨.
3. (선택) 큐브 실치수·광택 여부 + 테이블 z 한 번 측정.

관찰 포인트: **point cloud / OBB / base_z·top_z / 파지 성공을 함께** 볼 것 — 성공 여부만 보지
말고 "센서 데이터가 실제로 정상으로 돌아오는가"를 봐야 원인이 증명됨.

### 7.2 판정 (내일, raw depth 로 정량)

- **base_z→~0 / top→~25mm 로 내려오고 파지 성공** → **depth 저텍스처 편향 [확정]** → §7.3.
- **안 내려옴** → 텍스처 가설 반증 → 밝기 saturation / 최소거리(~7cm) / 캘 재검토(raw depth 로
  어느 쪽인지 판정). 이 경우 §7.3 구현 대신 센서 진단으로 되돌아감.
- (선택 분리: ①촘촘한 점 ②균일 무광 어두운 코팅 → 텍스처 vs 밝기 어느 쪽 원인인지 갈림.)

### 7.3 통과 후 — Claude 가 구현할 것 (우선순위)

> 전제: 텍스처로 편향이 사라짐 = **센서가 정상 데이터를 줄 수 있음**이 증명된 뒤.

1. **raw depth 편향 측정 스크립트** — 새 `.json`+`_depth.png` 로 표면까지 실측 vs 기대 거리(mm).
   `grasp_verify/` 로 이관(재현 자산화). *(형식 확정 위해 실데이터 나온 뒤 작성 — 먼저.)*
2. **closed-loop eye-in-hand (appearance 기반) — 메인 구현 대상.** (사용자 추측대로 closed-loop 맞음.)
   - **왜 이게 다음인가**: 텍스처는 진단이자 큐브 한정 임시책 — 일반 물체엔 매번 못 그림. 센서가
     정상 범위일 때도 남는 오차(캘 σ_t~7.5mm·백래시·조명)를 **재관측으로 잡는 표준 제어**(§5).
     지지면 무관이라 object-anywhere 목표와도 일관.
   - **설계 골자**: 접근 자세에서 재관측 → **이미지 특징(위치/크기) 서보**(편향 depth 3D 재투영
     open-loop 금지) → 수렴 판정 → 발산/미검출 시 안전 정지. tasks/core 훅·motion 프리미티브 재사용.
   - 착수 전 §5 규율 확인 + 이 문서에 설계 § 추가부터(구현은 그 뒤).

**요약 한 줄**: 오늘 밤 텍스처 A/B 로 원인 확정 → (정상경로) raw depth 측정 자산화 → **closed-loop
구현**. (텍스처로도 안 되면 센서 진단으로 분기.)

---

## 8. 히스토리 (압축 — 어떻게 여기 왔나)

- **시작**: "버그 2개"(뷰마다 height 0.5↔1.5cm·base_z −0.23m phantom / grasp IK 전멸)로 출발 →
  파고드니 별개 버그가 아니라 **숨은 전제 하나("물체는 책상 위")에서 갈라진 증상** → 파지 전체
  재설계로. IK 는 정상이었고 범인은 `plan_grasp` 의 **top-down 강제**(작은 팔이 먼 리치서 수직
  못 세움)였음(캘 kinematics pybullet 재현으로 확정).
- **전제 벗기기**(사용자 질문이 한 겹씩): 바닥 꼭 찾아야 하나 / 무조건 책상 위 / 무조건 top-down /
  무조건 +z lift / 픽앤플레이스에만 꽂힌 것 → **object-centric · reachable-orientation · grasp-frame
  동작**.
- **대전환**: "단일 사선 뷰면 충분(박스)"을 밀다 구/원뿔 반례로 뒤집힘 → **일반 형상 + 멀티뷰
  필수 + 표면 antipodal**(sim 전수검증). phantom 원인 = 2-percentile bottom → **z-gap 군집**으로 수정.
- **2026-07-14 밤 실물**: 여러 downstream 튜닝(융합 정합/antipodal 폭 8mm/놓기 자세·yaw)에도 큐브
  파지 전패. (PC 2회 하드다운 = 유령 중복 backend + 저가 PSU 스파이크 — 파워 교체 권장.)
- **2026-07-15**: debug 실데이터로 **근본원인 = D405 저텍스처 depth 편향, 파지 코드는 무죄** 규명.
  support-plane 은 object-anywhere 위배로 기각. raw depth 계측 추가.

**메타 교훈(재발 방지)**: ① 증상이 아니라 **전제**를 의심하라. ② **band-aid 금지**(threshold/필터
튜닝으로 덮지 말고 근본). ③ 코드 짜기 전 **sim 으로 재현·검증**(armchair 진단/최적화는 자주 틀림).
④ **해피케이스로 "됐다" 금지**. ⑤ **작은 팔의 한계 존중**. ⑥ **downstream 만지기 전 상류(입력
데이터) 먼저 의심** — 어제 밤 전패의 직접 교훈.
