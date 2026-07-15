# Frontend 인수인계 문서

이 문서 하나만 읽으면 frontend 가 **어떤 구조로 되어 있고, 코드를 어디서부터 봐야 하는지** 알 수 있게 쓰였습니다. 처음 이 프로젝트를 맡는 사람을 위한 글입니다. backend 를 먼저 보고 오면([docs/backend-onboarding.md](backend-onboarding.md)) 훨씬 빨리 붙습니다 — frontend 는 결국 backend 계약의 거울이기 때문입니다.

> 이 문서는 "지금 어떻게 생겼나"만 다룹니다. 개별 설계 결정의 배경·과거에 무엇을 왜 기각했는지는 [docs/frontend.md](frontend.md) 와 git log 에 있습니다.

---

## 1. 한 문단 요약

Frontend 는 **로봇 여러 대를 제어하는 3D 웹 워크스페이스**입니다. React 로 만든 단일 페이지 앱(SPA)이고, 화면 가운데에 로봇/카메라/포인트클라우드가 있는 **3D 씬**이 있으며, 그 위에 **떠다니는 패널**(모션 제어, 카메라 뷰, 캘리브레이션 등)을 자유롭게 배치합니다. 백엔드와는 **WebSocket 하나**로 연결돼서, 서비스를 호출하고 실시간 스트림(20Hz 관절 상태, 포인트클라우드 등)을 받습니다.

핵심 설계 한 줄: **"백엔드 계약(contract)이 프론트의 단일 진실 원천이다."** 백엔드가 노출하는 서비스/스트림/이벤트가 TypeScript 타입으로 자동 생성되고, 프론트는 그 타입을 얇은 hook 으로 감싸 쓸 뿐입니다. 백엔드에 새 기능이 생기면 프론트는 대개 **hook 한 줄**로 붙습니다.

기술 스택: React 19 + [react-three-fiber](https://r3f.docs.pmnd.rs/)(three.js 를 React 로) + [dockview](https://dockview.dev/)(플로팅 패널) + [zustand](https://zustand.docs.pmnd.rs/)(상태) + react-router + Tailwind/shadcn(스타일). 번들러는 Vite.

---

## 2. 먼저 잡아야 할 멘탈 모델

frontend 를 이해하는 데 필요한 개념은 세 가지입니다.

### ① Bridge — 백엔드로 가는 유일한 통로

[src/api/bridge.ts](../frontend/src/api/bridge.ts) 의 `BridgeClient` 하나가 백엔드와의 WebSocket 연결을 전담합니다. 앱 전체에서 단 하나의 인스턴스(`bridge`)를 공유합니다. subscribe(스트림 구독)/publish(발행)/callService(RPC) 세 가지 동작을 제공하고, 연결이 끊기면 자동 재연결합니다.

### ② Framework hooks — 계약을 React 로 옮기는 6개 hook

[src/framework/](../frontend/src/framework/) 는 Bridge 의 저수준 동작을 **React hook** 으로 감싼 얇은 층입니다. 컴포넌트는 Bridge 를 직접 만지지 않고 이 hook 들만 씁니다. 6개뿐입니다(§4).

### ③ Panel & Scene — 화면을 이루는 두 종류

화면은 **떠다니는 패널**(dockview)과 **3D 씬**(react-three-fiber)으로 구성됩니다. 이 둘의 소유권 규칙이 프론트에서 가장 헷갈리는 부분이라 §6 에서 따로 다룹니다.

이 세 가지가 층층이 쌓입니다: **컴포넌트 → framework hooks → Bridge → (WebSocket) → 백엔드.**

---

## 3. Bridge — 백엔드와의 연결

[bridge.ts](../frontend/src/api/bridge.ts) 가 하는 일:

- **브라우저→백엔드**: JSON 텍스트 제어 메시지 (`subscribe`/`unsubscribe`/`publish`/`service`)
- **백엔드→브라우저**: 바이너리 프레임 `[u8 ver][u8 type][u16 key_len][key][payload(msgpack)]`. `decodeFrame` 이 파싱하고, type 별로 스트림 콜백/서비스 resolver 로 분기합니다.

알아둘 두 가지 견고성 장치:

1. **robot-scoped 키 확장** ([bridge.ts:22](../frontend/src/api/bridge.ts#L22) `expandTopicKey`): `srv/motor/{robot_id}/...` 같은 키에서 `{robot_id}` 를 실제 로봇 id 로 치환합니다. **robotId 가 없으면 확장하지 않고 그대로 둡니다(fail-soft)** — 기본 로봇으로 추측 라우팅하지 않습니다. 잘못 확장하면 엉뚱한 로봇으로 명령이 새기 때문입니다.

2. **재연결 창 버퍼링**: WebSocket 이 아직 열리기 전에 낸 서비스 RPC 는 버려지지 않고 버퍼됐다가 연결되면 flush 됩니다. 반면 `publish`(예: 50Hz jog)는 버퍼 안 합니다 — 재연결 후 stale 명령을 재생하면 오히려 위험하기 때문입니다.

### 에러 모양 복원

백엔드 wire 에는 성공/실패 봉투가 없습니다(예외가 그냥 건너옵니다). Bridge 는 이걸 **클라이언트에서** `{ success, message, data }` 모양으로 복원해 hook 에 넘깁니다 ([bridge.ts:185](../frontend/src/api/bridge.ts#L185) 근처). 즉 이 `{success,...}` 모양은 wire 규약이 아니라 프론트 편의를 위한 shim 입니다.

---

## 4. Framework hooks — 6개

[src/framework/index.ts](../frontend/src/framework/index.ts) 가 전부 export 합니다. 백엔드의 4가지 통신 원시(service/stream/event/mirror)에 대응합니다.

| hook | 언제 쓰나 | 대응 |
| --- | --- | --- |
| `useService` | RPC 호출 + 응답 자동 캐시 | `srv/...` |
| `useTopic` | 토픽 최신값 하나 구독 | 일반 topic/event |
| `useStream` | 스트림 구독 + seq/lag 불변식(순서·유실 감지) | `stream/...` |
| `useMirror` | 스냅샷 + 변경 시 재요청(늦게 뜬 소유자에도 수렴) | Mirror |
| `useCapability` | 부팅 시 한 번 capability 스냅샷 | capability 서비스 |
| `useResource` | HTTP fetch + 캐시 + 폴링 | bridge 의 REST endpoint (예: `/robots`) |

**핵심 감각**: 백엔드에 뭔가 새로 생겼을 때 프론트에서 할 일이 hook 한 줄로 끝난다는 것. 새 스트림 → `useStream(Topic.X)`, 새 서비스 → `useService(Key.X)`, 새 로봇 → `robots.yaml` 한 줄에 프론트 코드 0. (index.ts 상단 주석에 이 표가 있습니다.)

### 서비스 응답 캐시 정체성 = (key, robotId)

이건 실제 사고가 났던 규칙이라 짚습니다. robot-agnostic 서비스는 wire 라우팅 키가 로봇과 무관하게 같습니다. 그 키로만 캐시하면 **robotA 의 응답이 robotB 뷰에 샙니다.** 그래서 캐시 키는 항상 `bridge.serviceCacheKey(key, robotId)` — `key#robotId` — 로 로봇별로 분리합니다 ([bridge.ts:113](../frontend/src/api/bridge.ts#L113)). 라우팅 키(어디로 보내나)와 캐시 키(어디에 저장하나)가 **다른 개념**이라는 걸 기억하세요.

---

## 5. 요청 하나가 흐르는 길 (end-to-end)

Motion 패널에서 "이 관절 움직여"를 눌렀을 때:

```
MotionPanel 컴포넌트
  │  useService(Motion.MOVE_J).call({...}, { robotId })
  ▼
framework/service.ts
  │  bridge.callService(key, data, { robotId })
  ▼
BridgeClient
  │  키 확장(robotId 치환) → JSON 텍스트로 WS 전송, 응답 대기
  ▼───────────── WebSocket ─────────────▶ 백엔드 Bridge module → Zenoh → Motion module
  ◀───────────── binary frame ──────────
  │  decodeFrame → msgpack decode → {success, message, data} 복원
  │  serviceCacheKey(key, robotId) 로 zustand store 에 캐시
  ▼
useService 가 리렌더 → 패널이 결과/에러 표시
```

스트림도 같은 통로입니다 — `useStream` 이 `bridge.subscribe` 로 토픽을 구독하면, 백엔드가 그 토픽으로 발행할 때마다 바이너리 프레임이 도착해 콜백이 불리고, hook 이 최신값으로 리렌더합니다.

---

## 6. Panel 과 Scene — 소유권이 핵심

프론트에서 가장 헷갈리고, 실제 버그가 가장 많이 났던 영역입니다.

### 패널 = dockview + 레지스트리

떠다니는 패널은 [dockview](https://dockview.dev/) 가 관리하고, 어떤 패널이 있는지는 [components/panels/registry.ts](../frontend/src/components/panels/registry.ts) 가 정합니다. 여기서 **`PANEL_CATALOG` 가 각 패널의 title/크기/필요 capability/scenePart 의 SSOT** 입니다. 각 페이지(mode)의 `PANELS` 는 "이 페이지의 기본 세트"를 배치 선언만 할 뿐입니다.

새 패널 추가 = 컴포넌트 만들고 → `PANEL_COMPONENTS` 한 줄 → `PANEL_CATALOG` 한 줄. (registry.ts 상단 주석 참고)

### 3D 씬 — "월드 객체" vs "scenePart"

3D 씬에 뭔가를 그릴 때 판별 질문 하나로 결정됩니다: **"패널을 닫으면 이게 사라져야 하나?"**

- **아니오 → Scene object (월드가 소유)**: 로봇, 카메라, 스캔 메시처럼 세계에 상시 존재하는 것. 자기가 자기 데이터·대상 로봇을 정해서 그립니다. [scene/Container.tsx](../frontend/src/components/scene/Container.tsx) 는 "특정 로봇의 스트림" 개념 자체가 없고, 각 씬 객체가 알아서 그립니다.
- **예 → scenePart (패널 수명 오버레이)**: 캘리브레이션 보드 프리뷰, 웨이포인트 ghost 처럼 그 패널이 살아있는 동안만 보이는 것. `PANEL_CATALOG` 의 `scenePart` 항목에 R3F 컴포넌트를 한 줄 선언하면 `ScenePartHost` 가 살아있는 패널 인스턴스마다 마운트합니다. `Scene.tsx` 는 안 건드립니다.

예를 들어 카메라 frustum(시야 원뿔)은 카메라가 월드 소유라서 **Scene object 인 [Cameras.tsx](../frontend/src/components/scene/objects/Cameras.tsx) 가 그리고**, 패널은 `cameraStore.showFrustum` 토글만 합니다.

### 로봇 소유권 — 패널이 로봇을 소유한다

"이 화면이 어느 로봇을 보고 있나"는 **패널이 소유**합니다. `ROBOT_OWNED_PANELS` ([registry.ts:116](../frontend/src/components/panels/registry.ts#L116))에 든 패널만 로봇 셀렉터 탭과 "Select Robot" 빈 상태를 갖습니다. 태스크 패널은 로봇을 *고르지* 않고 태스크 바인딩 계약에서 얻으므로 제외됩니다.

여기서 나온 규칙 하나 — **per-robot 상태는 반드시 `Record<robotId, ...>`**. 전역 boolean 으로 두면 두 번째 로봇에서 오발사합니다(frustum 토글, live 토글 등에서 실제로 두 번 사고 났습니다).

---

## 7. 라우트 & 페이지

[App.tsx](../frontend/src/App.tsx):

| 경로 | 페이지 |
| --- | --- |
| `/` | Dashboard (로봇 무관 착지점 — 기본 로봇으로 리다이렉트 안 함) |
| `/robots/:id/move` | 모션 제어 |
| `/robots/:id/calibrate` | 캘리브레이션 |
| `/robots/:id/scan` | 3D 스캔 |
| `/robots/:id/assets` | 웨이포인트 등 자산 |
| `/tasks/pick_and_place` | Pick&Place 전용 페이지 (로봇 하위가 아닌 최상위 — 태스크별 전용) |
| `/contract` | 계약 그래프 뷰어 (개발 도구, lazy-load) |

`/robots/:id` 는 공유 레이아웃([RobotsLayout.tsx](../frontend/src/pages/RobotsLayout.tsx))이고 mode 컴포넌트가 Outlet 에 들어갑니다. **mode 를 바꿔도 3D 씬(react-three-fiber Canvas)은 unmount 되지 않습니다** — 리마운트 비용이 크기 때문입니다.

로봇 목록은 [useRobots](../frontend/src/hooks/useRobots.ts) 가 백엔드 `/robots` 에서 가져옵니다. **"기본 로봇" 개념은 없습니다** — 로봇은 항상 라우트나 태스크 바인딩에서 명시적으로 옵니다.

---

## 8. 타입은 손으로 안 쓴다 — 계약에서 생성

프론트가 쓰는 서비스/스트림 타입은 [src/api/generated/contract.ts](../frontend/src/api/generated/contract.ts) 에 **자동 생성**됩니다. 절대 손으로 편집하지 마세요.

흐름: 백엔드 `GET /contract.json` → `pnpm gen:types` → `contract.ts`. 백엔드가 떠 있는 상태에서 실행합니다.

**불변식**: [src/api/__fixtures__/contract.json](../frontend/src/api/__fixtures__/contract.json) 이 `contract.ts` 와 한 쌍입니다. 계약이 바뀌면 **둘 다** 재생성해야 하고, vitest 가 둘이 byte-identical 인지 검증합니다. 이게 어긋나면 테스트가 빨개집니다.

---

## 9. 로컬에서 띄우기

```powershell
cd frontend
pnpm install
pnpm dev          # vite :5173 (백엔드 bridge 가 CORS 허용)
```

`pnpm dev` 전에 백엔드가 `:8000` 에 떠 있어야 합니다(대개 `--host mock`). 검사/테스트:

```powershell
pnpm build        # tsc -b && vite build (타입 체크 포함)
pnpm lint         # ESLint
pnpm vitest run   # 유닛 테스트
pnpm test:e2e     # Playwright (mock 백엔드 :8000 + pnpm dev :5173 선행 필요)
pnpm gen:types    # 떠 있는 백엔드 계약 → contract.ts 재생성
```

스타일은 ESLint + Prettier, 저장 시 자동 포맷(`editor.formatOnSave`). import 는 `@/` alias 가 [src/](../frontend/src/) 를 가리킵니다.

---

## 10. 코드 짤 때 지킬 것 (요약)

- **Bridge 를 컴포넌트에서 직접 만지지 말 것** — framework hook 을 쓰세요. hook 이 캐시/구독 해제/리렌더를 다 처리합니다.
- **서비스 캐시 정체성은 `(key, robotId)`** — `bridge.serviceCacheKey` 를 통해서만. wire 키로 직접 캐시하면 cross-robot 오염.
- **per-robot 상태는 `Record<robotId, ...>`** — 전역 bool 금지.
- **로봇 id 를 상수/기본값으로 박지 말 것** — 라우트/셀렉터/태스크 바인딩에서 파생.
- **생성된 `contract.ts` 를 손대지 말 것** — 계약 바뀌면 `contract.ts` + `contract.json` 둘 다 재생성.
- **침묵 fallback 금지** — 실패를 기본값으로 덮으면 조용한 오동작. 실패는 사유 + 다음 행동을 사용자에게 보여야 합니다.

---

## 11. 더 깊이 볼 곳

| 궁금한 것 | 문서 |
| --- | --- |
| hooks/패널/씬 소유권/robot ownership 상세 | [docs/frontend.md](frontend.md) |
| 백엔드 구조 (프론트가 거울로 삼는 계약) | [docs/backend-onboarding.md](backend-onboarding.md) |
| 캘리브레이션 UI 흐름 | [docs/calibration.md](calibration.md) |
| 모션(Jog/MoveJ/MoveL) | [docs/motion.md](motion.md) |
| Task 페이지 아키텍처 | [docs/task.md](task.md) |
