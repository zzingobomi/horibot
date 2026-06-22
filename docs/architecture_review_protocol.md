# 아키텍처 검토 프로토콜

기능 구현 phase 끝, **horibot 전체 코드 아키텍처 점검 phase**. 다른 세션 / 다른 PC 에서도 이 프로토콜대로 진행 — 매번 사용자가 의도 다시 설명 X.

## 사용자 진행 방식

사용자가 코드 한 줄씩 읽으며 던지는 질문:

- "이거 뭐지?"
- "이거 왜 이렇게 짰어?"
- "이거 설계상 맞아?"
- "이거 [원칙/패턴] 깨진 거 아냐?"

매번 "지금 우리가 뭐 하는 중인지" 다시 설명 안 함. 본 protocol 자리로 답.

## 검토 framework — 각 의심 자리에 4 step

### 1. 의심 자리 식별

사용자가 던지면 받고 *단편 reflex X*. "DIP 깨졌네 고치자" / "SOLID 위반" / "표준 패턴 어김" 같은 학술 어휘만 박고 끝내는 자리 금지.

### 2. *왜 이렇게 됐나* 짚기 — 의도 vs 임시 판별

**추측 / 일반론 / 기억 자리 답 X**. 코드 / git / docs 자리 짚어서:

- **코드 docstring / 주석** — "왜 이렇게 짰는지" 명시되어 있나
- **`git log --follow` / `git log --diff-filter=A --follow`** — 도입 커밋 / 커밋 메시지 어휘 / 변천사. 의도였으면 메시지에 흔적 남음, 임시였으면 잡스러운 자리에 끼어 들어옴
- **docs/** 안의 design 자리 — 해당 패턴 자리가 `multi_robot_architecture.md` / `testing_strategy.md` / 등에 명시되어 있나
- **일관성 패턴 자체** — 같은 패턴이 *여러 자리* 에 있으면 design choice 흔적, *1-2 자리* 만이면 임시 / 복붙 가능성

판별 결과:
- (a) **의도된 design choice** — 명시적으로 그렇게 짰음, 이유 있음
- (b) **그냥 빨리 만드느라 / 별 생각 없이 흘러간 자리** — 복붙 / 시간 압박 / 패턴 인지 못 함
- (c) **결과적으로는 OK 인데 의도였는지 흘러간 건지 코드만으론 판별 불가** — 사용자에게 기억 자리 직접 확인

### 3. 분기

- **(a) 의도였음** → 정당화 *명문화* (docs / 주석 / 메모리) + **다른 자리도 같은 원칙에 맞게 정렬** (통일)
- **(b) 임시였음** → 정석 패턴으로 *수정* + 통일
- **(c) 판별 불가** → 사용자 확인 받고 (a)/(b) 분기

### 4. 통일성 갖춰 나감

검토 phase 끝 자리에는:
- *원칙 별로 일관된* 코드
- *원칙 자체* 가 docs / CLAUDE.md / 메모리 자리 박혀 있음 → 다음 세션에서 흔들리지 않음
- 새 코드 추가 자리 자연스럽게 같은 원칙 따름

## 제약

### 1. "지금 잘 돌아가는 이유" 깎으면 기능 망가짐

통일성만 추구하다 *전체 동작 모델 자리 손상* 위험. 검토 = *왜 잘 돌아가는지* 까지 이해한 후 통일. "단편적으로만 보면 안 됨" 이 핵심.

예: `FrameCache()` 가 노드 안에서 직접 호출되는 자리. DIP 위반 어휘로 무조건 고치자 식 reflex X — *왜 topic-based + e2e 검증 자리 모델 자리에선 실제 비용 안 나타나는지* 까지 이해한 후 분기.

### 2. 검토 ≠ 즉시 구현

사용자가 명시적 "구현해" 전엔 **논의만**. 코드 한 줄 수정 X, 새 파일 생성 X, 리팩토링 점프 X. 메모리 [feedback_discuss_before_implement.md](../C:/Users/Oscar/.claude/projects/d--study-horibot/memory/feedback_discuss_before_implement.md) 적용.

### 3. 단편 reflex 금지

- "DIP 위반" / "SOLID 어김" / "표준 패턴" 어휘만 박고 *왜 그렇게 됐는지* 안 짚으면 검토 X
- "한 줄이면 고침" / "cheap fix" 같은 변경 크기 근거 X (메모리 [feedback_no_cheap_argument.md](../C:/Users/Oscar/.claude/projects/d--study-horibot/memory/feedback_no_cheap_argument.md))
- "개인 학습 프로젝트라서" / "N=2 라서" scope 핑계 X (메모리 [feedback_no_scope_excuse.md](../C:/Users/Oscar/.claude/projects/d--study-horibot/memory/feedback_no_scope_excuse.md))

### 4. 코드 답변은 코드 읽고

"이거 어떻게 동작하나" 류 답 *전* Read / Grep / git log 필수. 추측 / 기억 / 일반론으로 시작 X (메모리 [feedback_read_code_before_claiming.md](../C:/Users/Oscar/.claude/projects/d--study-horibot/memory/feedback_read_code_before_claiming.md)).

### 5. md/docs 인용 X — 실제 코드 우선

검토 / 의도 파악 자리 docs 인용으로 답하지 말 것. 코드 자체 (클래스 구현, lock 패턴, 가드 메시지, 멱등 처리, 변수 명명) 가 1차 source.

- docs/md 는 *내가 또는 사용자가 나중에 정리한 자기 의도* 일 가능성 — 코드 ground truth 와 일치 안 할 수 있음
- "docs 에 X 명시되어 있음" 식 권위 인용 X — 사용자가 본 거 또는 직접 적은 거
- 의도 추론 자리: 코드 흔적 짚기
  - 두 클래스 lock 패턴 동일성 → 의식적 통일 흔적
  - 가드 메시지 어휘 유사성 → 같은 손에서 짠 흔적
  - 멱등 / 가드 / 주석 → 의식적 design 여부
- 예외: 외부 표준 / 라이브러리 spec 인용 자리는 OK (Zenoh / Alembic 등)

### 6. 의도를 사용자한테 떠넘기지 말 것

이 코드는 *내가 짠 것*. 사용자는 기능 명세만 줬고 구현 결정은 내가 했음. 그래서 "이게 의도였어?" / "너만 알아" 식 떠넘기기 X.

- 코드 흔적 (위 §5) 으로 *내가 짚어내야* 함
- 사용자가 명시적으로 *기억 자리 직접 확인 부탁* 할 때만 묻기
- 단정 못 하면 "코드 흔적상 의식적인 것 같다 / 자연스럽게 흘러간 것 같다" 식 *내 추론* 으로 답하지, 사용자한테 답을 미루지 말 것

### 7. 사용자 push 에 입장 뒤집기 X

검토 자리 핵심 — 사용자가 challenge 톤 자리 던져도 *새 정보 없으면* 원래 입장 근거 다시 댐. 단 *새 정보 / 정정* 자리는 받아들임 (메모리 [feedback_no_flipflop_under_pressure.md](../C:/Users/Oscar/.claude/projects/d--study-horibot/memory/feedback_no_flipflop_under_pressure.md)).

## 산출물 자리

검토 진행하면서 *합의된 원칙* 자리는 *그때그때* (검토 끝에 한 번에 X) 박을 것. 위치 후보:

| 범위 | 자리 |
|---|---|
| 코드만 짚어도 명확한 단일 패턴 자리 | **주석** (해당 클래스 / 함수 docstring) |
| 한 줄 협업 원칙 자리 | **메모리** (`feedback_*` / `project_*`) |
| 여러 자리 영향 / 다른 노드 정렬 필요 | **`docs/<주제>.md`** (신규) |
| 프로젝트 전반 design decision | **CLAUDE.md "프로젝트 design decision" 섹션** |

사용자 OK 받고 박을 것 — 검토 *대화* 자리만 진행하다 산출물 자리 0 이면 다음 세션 자리 흔들림.

## 진행 방식 — 매 의심 자리 자리에서

1. 사용자 던짐 → 받기
2. (필요하면) Read / Grep / git log 자리로 짚기
3. *왜 이렇게 됐나* 분석 → 의도 / 임시 / 판별 불가 분기
4. 사용자와 **분기 결정** 자리 (정당화 명문화 / 수정 / 보존)
5. 산출물 자리 합의 후 *그때 박기* (코드 수정은 명시적 "구현해" 후)
6. 다음 의심 자리 자리로

## 시작 자리

매 세션 시작 자리에 본 문서 자동 로드 (CLAUDE.md anchor 자리). 사용자가 "검토 모드" / "음 이거 뭐지?" 어휘 자리 던지면 본 protocol 진입.
