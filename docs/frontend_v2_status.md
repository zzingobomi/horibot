# frontend_v2 구현 status + 다음 세션 handoff

> 새 세션이 **바로 이어서 구현**할 수 있게 박은 status. SSOT spec = [frontend_v2.md](frontend_v2.md), backend SSOT = [backend_v2.md](backend_v2.md) + [backend_v2_modules.md](backend_v2_modules.md) + [backend_v2_status.md](backend_v2_status.md).
>
> 본 문서 = "지금 어디까지 됐고 다음 뭐 할지 + 박힌 검증 + latent bug".

## 현재 상태 (2026-07-01)

frontend_v2 = backend_v2 정합 frontend 신규. **Step F1-F5 다 완료. 31 vitest PASS + 1 Playwright PASS + 1 WebdriverIO PASS, ruff/eslint clean.**

| 영역 | 상태 |
|---|---|
| Step F1: scaffold (vite + tsconfig + tailwind + main/App + index.html) | ✅ build PASS, 233KB JS (옛 frontend 1.9MB 의 12%) |
| Step F1.5: Vitest scaffold | ✅ 1 PASS |
| Step F2a: framework carry over (bridge + store + topic + service + resource + bootstrap + index) | ✅ build PASS |
| Step F2b: L2 tests (bridge 8 + service 2 + topic 1 + resource 2) | ✅ 13 PASS |
| Step F3a: useStream + useCapability + L2 tests (5 + 2) | ✅ 7 PASS |
| Step F3b: useMirror + L2 tests (5) | ✅ 5 PASS |
| Step F4a: lib (utils + clsx/tailwind-merge dep) | ✅ |
| Step F4b: 3D scene (AxisFrame + RobotModel + RobotLayer + Scene + Container) | ✅ build PASS |
| Step F4c: Jog panels (JogJ + JogTcp) + RobotStatePanel + L2 tests (JogJ 3 + RobotStatePanel 2) | ✅ 5 PASS |
| Step F5: MovePage + route + useFrameworkBootstrap | ✅ |
| Step F5 L4 e2e: Playwright (WS + URDF + RAW_STATE 도착) | ✅ 1 PASS |
| Step F5 L4 e2e: WebdriverIO (J1+ 800ms hold → 50Hz publish + raw 변화) | ✅ 1 PASS |

**검증** (cwd 반드시 `frontend_v2/`):
```powershell
cd frontend_v2
pnpm install
pnpm lint        # eslint
pnpm build       # tsc -b && vite build
pnpm test        # vitest run — 31 PASS
```

**e2e (mock backend + dev server 사전 띄움 필수)**:
```powershell
# Terminal 1 — mock backend
cd backend_v2
uv run --no-sync python -m apps.main --host mock

# Terminal 2 — frontend dev
cd frontend_v2
pnpm dev

# Terminal 3 — e2e
cd frontend_v2
pnpm test:e2e        # Playwright (test 1 PASS, test 2 skip)
pnpm test:e2e:wdio   # WebdriverIO (1 PASS — 진짜 jog hold)
```

## 아키텍처 불변식 (절대 어기지 말 것)

- **frontend_v2.md §14 anchor 13개 + memory 박힌 cleanup filler 규칙**:
  - "자체 자체 / 박은 자리 / 자리 자체 / 자체 자리" 의미 없는 filler 박지 말 것 ([[feedback-no-jache-jari-filler]] 정합). 답 박기 전 grep self-check 의무.
- **레이어링**: frontend_v2/ 는 frontend/ (옛 backend 호환) 영향 0. 옛 frontend 그대로 두고 frontend_v2/ 독립.
- **contract SSOT**: backend_v2 의 `modules/*/contract.py` → `pnpm gen:types` → `src/api/generated/contract.ts` 자동 emit. 손작업 동기화 0.
- **framework 어휘 1:1**:
  - `useService` — RPC + auto cache
  - `useTopic` / `onTopic` — generic latest cache
  - `useStream` — seq monotonic + timestamp_unix lag invariant (신규)
  - `useMirror` — snapshot + invalidate+refetch (신규)
  - `useCapability` — boot 1회 snapshot (신규)
  - `useResource` — HTTP fetch + cache + poll
- **useCapability + useMirror 의 `useBridgeConnected` dep 필수** — WS 미연결 시 callService drop → timeout. connected=true 박힌 후 fetch.
- **JogJ button 의 `setPointerCapture` 필수** — Chromium 이 button class 변경 시 auto pointercancel → pointerup promote 차단. 실 hardware 박을 때 빠른 손가락 / 누른 채 드래그 시나리오 동일 fix.
- **L4 e2e = WebdriverIO**, Playwright 는 button hold 시나리오 안 됨 (W3C Actions vs CDP Mouse 차이). Playwright 는 단순 wait 시나리오 (WS 연결 / URDF / state 도착) 만.
- 테스트는 통과용 X — 실 동작/invariant 검증 ([[feedback-meaningful-tests]]). 모든 test 의 docstring 에 `spec frontend_v2.md §X — invariant Y` 명시.

## 박힌 파일 인벤토리

**framework + transport**:
- `src/api/bridge.ts` — backend_v2 wire (msgpack + binary frame parser + service shim + setDefaultRobotId)
- `src/api/generated/contract.ts` — gen_contract.py emit (36 model + 4 enum + 12 topic + 11 service)
- `src/types/bridge.ts` — WsOp / FrameType / FRAME_VERSION
- `src/framework/{store,bootstrap,service,topic,resource,stream,mirror,capability,index}.ts` — 9 file
- `src/hooks/useRobots.ts`
- `src/constants/index.ts` — DEFAULT_ROBOT_ID = "so101_6dof_0"
- `src/lib/utils.ts` — cn helper

**UI**:
- `src/components/scene/{AxisFrame,RobotModel,RobotLayer,Scene,Container}.tsx` — 5 file
- `src/components/jog/{JogJ,JogTcp}.tsx` — 2 file (setPointerCapture 박힘)
- `src/components/motor/RobotStatePanel.tsx`
- `src/pages/MovePage.tsx` — 3-column CSS grid
- `src/App.tsx` + `src/main.tsx` + `src/index.css`

**Test (L1-L4)**:
- L2 unit: `src/{__tests__/scaffold,api/bridge,framework/{service,topic,resource,stream,capability,mirror},components/jog/JogJ,components/motor/RobotStatePanel}.test.{ts,tsx}` — 11 file, 31 PASS
- L4 Playwright: `e2e/jog.spec.ts` — 1 PASS + 1 skip (test 2 = chromium pointer hold 안 됨)
- L4 WebdriverIO: `e2e_wdio/jog.test.ts` — 1 PASS (진짜 800ms hold)

**Config**:
- `package.json` — react 19 + vite 8 + zustand + R3F + urdf-loader + msgpack + Playwright + WebdriverIO
- `tsconfig.{json,app,node,wdio}.json`
- `vite.config.ts` — alias + tailwind + vitest config
- `vitest.setup.ts` — RTL + jest-dom
- `playwright.config.ts` — hasTouch:true
- `wdio.conf.ts` — chromedriver + W3C Actions
- `eslint.config.js`

**Backend 변경** (frontend_v2 운영 위해 박음):
- `backend_v2/scripts/gen_contract.py` — Pydantic + StrEnum → TS emit (588 line)
- `backend_v2/config/deployments/mock.yaml` — `bridge` module 추가

## 다음 진입점

### 1. cleanup (작은 자리 — 사용자 컴터 박은 후 박을 자리)
- **JogJ.tsx 의 debug console.log 정리** — `[JogJ] tick`, `[JogJ] stopJog`, `[JogJ] deadman fire` log 박혀있음. e2e debug 흔적. production cleanup 필요.
- **JogTcp.tsx 의 setPointerCapture 검증** — JogJ 와 같은 fix 박았지만 L2 test 박지 X. Step F4c 의 L2 test 추가 가능.
- **Playwright e2e test 2 결정** — skip 한 자리 cleanup 또는 그대로 (WebdriverIO 가 진짜 검증).

### 2. backend bridge ws panic (latent)
**Windows websockets legacy assertion** — frontend 의 reload / close 시 backend `modules/bridge/ws.py` 의 `_drain_helper` `AssertionError` panic. backend log:
```
File "websockets/legacy/protocol.py", line 308, in _drain_helper
  assert waiter is None or waiter.cancelled()
AssertionError
```
- 사용자가 실 hardware 박을 때 browser refresh / navigate 시 backend bridge stuck.
- fix 자리 = backend_v2/modules/bridge/ws.py — websockets legacy → 새 API migrate 또는 connection cleanup 정직 처리.
- 우선순위 = mid (frontend e2e 영향 X, 단 사용자 실 사용 시 reload 후 backend restart 필요).

### 3. Step E+ (backend Calibration / Detector / Scene3D / Scan / Reconstruction / Task / Gamepad)
- backend_v2_status.md §"다음 = C2" 박힘 — backend Step E+ 박힌 후 frontend 의 `useMirror(CalibrationBundle)` + `useCapability(camera)` + `Scene3DLayer` carry over (옛 frontend 의 calibration / scene3D / task UI 자산).
- frontend_v2.md §10 "옛 frontend 에서 carry over 자산 인벤토리" 박힌 자리.

## hardware 검증 (집에서)

- 사용자 목표 = *집에서 frontend_v2 로 실 SO-101 jog* — 검증 시나리오:
  1. 집에서 `cd backend_v2; uv run python -m apps.main --host pc` (또는 pi_motor + pi_camera 분산)
  2. `cd frontend_v2; pnpm dev`
  3. 브라우저 → `http://localhost:5174/robots/so101_6dof_0/move`
  4. J1+ button hold → 실 SO-101 motor 회전 확인
  5. frontend_v2 의 `Motion.Stream.JOG_J` publish → backend pi_motor 가 받음 → motor cmd → 실 Feetech 동작

- 잠재 issue (실 hardware 박을 때):
  - **Feetech driver 실 통신 미검증** (backend_v2_status.md latent — feetech driver 의 register map / sync / signed / clamp 가 실 motor 와 통신 안 해봄)
  - **motors.yaml pid/profile 미적용** (EEPROM default 사용)
  - **backend bridge ws panic** — browser reload 시 backend restart 필요 (위 latent)

## 작업 원칙

- 본 문서 = 현재 진행 status SSOT. spec 결정 자리 = frontend_v2.md §14 anchor 13개.
- 박힌 결정 (anchor) 의심하지 말 것.
- 답 박기 전 grep `자체 자체|박은 자리|자리 자체|자체 자리` self-check 의무 — [[feedback-no-jache-jari-filler]] 정합.
- 테스트 박을 자리 docstring 에 `spec frontend_v2.md §X — invariant Y` 명시 ([[feedback-meaningful-tests]]).
- 실 hardware 자리 deterministic test (backend pytest + L2 vitest + L4 WebdriverIO) 다 통과 후 사용자에게 hardware 검증 넘김 ([[feedback-pre-hardware-test-thoroughness]]).
