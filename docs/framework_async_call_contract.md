# Framework — Async-Uniform Call Contract (설계, 2026-07-03)

> backend_v2 framework 의 **모듈 간 호출 API 통일** 설계. **구현 전 단계** — 방향 결정
> 완료, 구현은 다음 세션. 본 문서로 논의 이어가기. ([backend_v2.md](backend_v2.md) 의
> 4 primitive 계약 위에 얹는 실행모델 정정.)

## 1. 문제 — "sync냐 async냐"를 모듈 개발자가 의식하게 만든다

모듈 개발자가 다른 모듈의 service 를 호출할 때, **지금 자기가 어느 함수 안에 있느냐**
에 따라 호출 방법이 달라진다:

| 지금 있는 곳 | 호출 방법 |
|---|---|
| `async def start()` / 내가 띄운 async task | `await self.runtime.call(...)` |
| `@service` / `@subscriber` 핸들러 (sync `def`) | `run_coroutine_threadsafe(...).result()` 브리지 |

실제 사례 — [scan/module.py](../backend_v2/modules/scan/module.py) 의 `_call` 헬퍼:
sync `capture()` 핸들러가 scene3d SNAPSHOT 을 부르려고 이벤트 루프 저장 +
`run_coroutine_threadsafe` + `Future.result()` 를 **모듈 코드에** 들고 있다. 이건 scan
도메인이 아니라 **asyncio 실행모델 처리** — framework 가 감춰야 할 것이 모듈로 새어
올라온 것.

**판정 기준**: 모듈 개발자가 "이거 await 해야 하나? call_sync 였나?" 를 고민하며 다른
코드를 뒤지게 만들면, 그 지점에서 framework 는 UX 실패다. 모듈 간 호출은 **문맥과
무관하게 단 하나의 방법**이어야 한다.

## 2. 현재 실행 모델 (실측)

- **transport.call** ([infra/transport/zenoh.py](../backend_v2/infra/transport/zenoh.py)):
  `async def call` → `await asyncio.to_thread(self._call_sync, ...)`. **이미 Zenoh 의
  sync `session.get()` 을 thread 로 감싸 async 로 노출**한다. (transport layer 는 이미
  옳게 흡수 중.)
- **service 핸들러 등록** ([app.py `_register_service`](../backend_v2/framework/runtime/app.py)):
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

1. ✅ **framework 가 sync/async bridge 를 완전히 흡수** — [app.py `_register_service`](../backend_v2/framework/runtime/app.py)
   `handler_bytes` 가 `asyncio.iscoroutine(result)` 면 `run_coroutine_threadsafe(coro,
   self._loop).result()` (timeout 없음 — long build 는 핸들러 안 `to_thread` 로 loop 안
   막고, 워커 스레드만 완료까지 대기 = sync 핸들러와 동일). `_register_subscriber` 도
   coroutine 이면 fire-and-forget schedule + done-callback 으로 예외 로깅. sync 핸들러는
   기존대로 직접 호출 (backward-compat).
2. ✅ **`call_sync` — 애초에 public API 에 없었음.** `ModuleRuntime` protocol
   ([api.py](../backend_v2/framework/runtime/api.py)) 은 처음부터 `call` 단일. zenoh
   transport 내부 `_call_sync` 만 존재하고 그건 이미 `to_thread` 로 흡수된 올바른 자리.
   §4 의 "call_sync 폐기" 는 선제적 표현이었고 **제거할 대상이 없었다** (no-op 확인).
3. ✅ **scan 모듈 async 정리** — [scan/module.py](../backend_v2/modules/scan/module.py) 의
   `_call` / `self._loop` / `Coroutine`·`cast`·`TypeVar`·`BaseModel` import 삭제.
   `capture`/`build` → `async def` + `await self.runtime.call(...)`. `build_mesh` →
   `await asyncio.to_thread(...)`.
4. ✅ CPU 집약(build_mesh)은 `await asyncio.to_thread(...)` (③-heavy 관례). `@service(
   offload=True)` 자동화는 미도입 — 실사용에서 반복 필요성 보이면 그때.

핵심 목표 **"모듈 개발자는 `await runtime.call(...)` 만 알면 된다"** 달성. 이후 정책
(②-pool 측정 / ③-heavy·timeout·sync 존치)은 실사용 경험 위에서.

## 9. 관련 문서

- [backend_v2.md](backend_v2.md) — framework SSOT (4 primitive / Runtime lifecycle / Owner-Reader). 본 문서는 그 위 **실행모델(sync/async) 정정**.
- [framework_dogfood_plan.md](framework_dogfood_plan.md) — Runtime/Module/Transport 3 layer reframe.
- [project_scan_pragmatic_slice] — `_call` 브리지가 처음 등장한 자리 (이 논의의 발단).
