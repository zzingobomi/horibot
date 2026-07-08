# robot_ownership_model.md

frontend workspace에서 **robot이 누구의 소유인가** 를 정하는 아키텍처 규칙.
"패널이 robot을 소유한다" 를 SSOT로 하는 불변식(invariant) 문서 — 구현 방법이
아니라 **무엇이 참이어야 하는가** 만 기술한다. 구현체(어떤 hook/직렬화/컴포넌트를
쓸지)는 이 불변식을 만족하는 한 자유롭게 진화할 수 있고, 갈아엎어도 이 문서는
그대로 살아 있어야 한다.

> 진입 톤: "패널이 robot 어떻게 아나" / "route 밖에서 robot-scoped 패널" /
> "robot 셀렉터" / "어느 페이지서든 아무 패널" / "ambient robot" 나오면 본 문서.
> UI 기능(패널 추가/삭제 헤더)은 [workspace_autohide_header.md](workspace_autohide_header.md).

---

## 1. 왜 Route 기반 모델이 한계인가

현재 robot id는 **route에서 ambient로 주입** 된다 — robot-scoped 패널은
`/robots/:id` 라우트 param을 읽어 대상 robot을 안다 (route 밖이면 명시적 throw,
ambient default 없음). 이건 "한 페이지 = 한 robot" 전제 위에선 깔끔했다.

그런데 workspace의 목표가 **"어느 페이지에서든 사용자가 원하는 패널을 띄운다"** 로
올라가면 이 전제가 깨진다:

1. **route가 robot을 안 주는 페이지** (task 페이지, world 등)에 robot-scoped 패널을
   띄우면 대상 robot을 해석할 근거가 없다.
2. `/robots/:id` 안에서도 **모든 패널이 route robot에 강제로 묶여**, 서로 다른
   robot을 나란히 볼 수 없다.

근본 원인: **robot을 navigation 위치(route)에 묶은 것.** 패널이 어느 robot의
데이터를 보여줄지는 navigation 관심사가 아니다. 소유권을 잘못 둔 것이다.

---

## 2. 설계 목표

- 어느 페이지에서든 어떤 패널이든 띄울 수 있다.
- 한 workspace 안에서 서로 다른 robot을 대상으로 하는 패널이 공존할 수 있다.
- robot 바인딩이 **환경 상태(route, 현재 robot 목록, robot 개수)에 흔들리지
  않는다** — 한 번 정해진 패널의 대상은 환경이 바뀌어도 그대로다.

---

## 3. Ownership (소유권)

- **robot은 패널이 소유한다.** 각 패널은 `robot = <id>` 또는 `robot = None` 을
  자기 상태로 가진다.
- **Workspace는 robot을 소유하지 않는다.** Workspace의 책임은 패널 배치·레이아웃·
  저장뿐이다. "현재 robot" 이라는 개념이 Workspace에는 존재하지 않는다.
- **Route는 robot을 소유하지 않는다.** Route는 패널 **생성 시점의 초기값** 만
  제공하고, 그 이후로는 패널의 robot에 관여하지 않는다.

---

## 4. Binding Rules (바인딩)

핵심 불변식:

> **패널이 어느 robot의 데이터를 보여줄지는 오직 그 패널의 `robot` 상태에서
> 결정된다. 환경(route, 현재 robot 목록, robot 개수)은 바인딩에 영향을 주지
> 않는다.**

파생:

- 패널은 **생성 이후 "현재 robot 목록" 을 바인딩 목적으로 다시 참조하지 않는다.**
  자기 `robot` 상태만 본다. robot 목록이 늘거나 줄어도 패널의 대상은 불변.
- **생성 시 초기값 결정** (환경을 읽는 유일한 순간, 그것도 "지금 읽어 고정" 이지
  "매번 읽음" 이 아니다):
  - robot 후보 **0개** → `robot = None`
  - robot 후보 **1개** → 그 robot을 패널 상태에 **기록** (자동 바인딩이 아니라
    route가 초기값을 넣는 것과 동일한 "생성 시 초기값" — 기록 이후엔 환경 무관)
  - robot 후보 **2개 이상** → `robot = None`
- 후보 robot이 있으면 route robot을 우선 초기값으로 쓸 수 있으나, 이는 초기값
  선택 규칙일 뿐이며 바인딩 규칙이 아니다.

---

## 5. Workspace Rules

- **Add = 항상 Spawn.** "추가" 의 의미는 언제나 "새 패널 생성" 이다. 기존 패널을
  앞으로 끌어오는 toggle이 아니다. 이는 robot과 무관한 **Workspace의 공리** —
  robot이 전혀 없는 패널(로그/콘솔 등)도 "추가" 는 곧 "생성" 이어야 자연스럽다.
  robot 소유 모델은 이 공리 위에 얹히는 속성일 뿐이다.
- **레이아웃 저장 시 각 패널의 `robot` 도 함께 저장** 된다. 다시 열면 패널 배치와
  대상 robot이 그대로 복원된다. (환경을 다시 읽어 재계산하지 않는다 — §4 불변식.)

---

## 6. UI Rules

UI는 패널의 `robot` 상태의 **함수** 다 (상태를 바꾸는 원인이 아니라 결과):

- `robot = None` → 패널은 **"Select Robot"** 빈 상태를 보여준다. 임의의 default
  robot 데이터를 조용히 보여주지 않는다.
- `robot = <id>` → 해당 robot의 데이터 뷰.
- **robot 셀렉터는 패널 헤더에 둔다** (본문 아님). robot은 그 패널의 설정이므로
  헤더가 맞고, 여러 패널을 나란히 놨을 때 각 패널이 어느 robot인지 헤더에서 한눈에
  식별된다.
- **robot이 하나뿐이면 셀렉터를 숨긴다** (선택지 하나짜리 picker는 노이즈).

불변식과 UI의 경계 (혼동 방지):

- §4의 "환경을 읽지 않는다" 는 **바인딩에 거는 규칙** 이다.
- **셀렉터(picker)가 옵션을 채우려고 robot 목록을 읽는 것, "N=1이면 숨김" 이
  robot 개수를 읽는 것은 위반이 아니다** — 이는 picker가 제 일을 하는 것이고
  cosmetic affordance일 뿐, 패널의 바인딩(`robot` 상태)을 바꾸지 않는다.

---

## 7. Exception — Task Panel

task 실행에 종속된 패널(프롬프트/진행상황 등)은 robot을 **task 바인딩** 에서
얻는다 — 사용자가 패널마다 고르는 것이 아니다. 한 task의 프롬프트와 진행상황이
같은 robot을 봐야 하기 때문 (패널별로 갈라지면 안 됨). 따라서 **§3~§6의 소유권/
셀렉터 규칙은 task 패널에 적용되지 않는다.** task 패널의 바인딩 단위는 "패널" 이
아니라 "task" 다.

---

## 8. Open Questions

- **Workspace 레이아웃의 저장 범위 — route별 저장 vs 전역 저장.** 현재는 route별
  분리 저장(robot route마다 자기 배치를 기억). 이건 robot 소유 모델과 **직교** —
  두 방식 다 "패널이 robot을 소유" 와 충돌하지 않으므로 여기서 결정하지 않는다.
  Workspace scope를 별도로 설계할 때 결정한다. **그때까지는 현행(route별) 유지.**
