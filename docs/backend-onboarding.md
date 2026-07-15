# Backend 인수인계 문서

이 문서 하나만 읽으면 backend 가 **어떤 구조로 되어 있고, 코드를 어디서부터 봐야 하는지** 알 수 있게 쓰였습니다. 처음 이 프로젝트를 맡는 사람을 위한 글입니다.

> 이 문서는 "지금 시스템이 어떻게 생겼나"만 다룹니다. 개별 설계 결정의 배경·트레이드오프, 과거에 무엇을 왜 기각했는지, 지금 진행 중인 작업이 어디까지 왔는지는 다른 곳(각 도메인 `docs/*.md`, git log)에 있습니다.

---

## 1. 한 문단 요약

Horibot 은 **로봇 팔 여러 대를 제어하는 스택**입니다. 현재 로봇 2대(SO-101 6DOF, OMX_F)를 다룹니다. Backend 는 로봇 하드웨어(모터·카메라)를 구동하고, 3D 인식·모션 계획·캘리브레이션·Pick&Place 같은 작업을 수행하며, 그 결과를 브라우저 프론트엔드에 실시간으로 흘려보냅니다.

핵심 설계 한 줄: **"기능을 잘게 나눈 Module 들이, 자기가 어느 기계에서 돌든 똑같은 코드로 동작한다."** 로봇이 PC 한 대에 다 붙어 있든, 여러 대의 라즈베리파이에 분산돼 있든, 코드는 그대로고 **배치 설정(yaml)만 다릅니다.**

---

## 2. 먼저 잡아야 할 멘탈 모델

backend 를 이해하는 데 필요한 개념은 딱 두 가지입니다.

### Module — 기능 한 덩어리

`backend/modules/` 아래 폴더 하나가 Module 하나입니다. `motor`, `camera`, `motion`, `calibration` 처럼요. Module 은 **평범한 Python 클래스**이고, "나는 이런 요청을 처리하고, 이런 데이터를 발행한다"를 데코레이터로 선언합니다. Module 끼리는 서로를 직접 import 하지 않습니다 — 오직 **메시지(wire)** 로만 대화합니다.

### Framework — Module 들을 실어 나르는 3층

`backend/framework/` 는 Module 이 서로 통신하게 해주는 인프라입니다. Module 의 도메인 로직(로봇을 어떻게 움직일지)은 하나도 모르고, 오직 "메시지를 주고받게" 하는 일만 합니다. 3층으로 나뉩니다:

| 층 | 하는 일 | 코드 |
| --- | --- | --- |
| **Contract** | 각 Module 이 주고받는 메시지의 스키마(형태)를 정의 | `modules/<name>/contract.py` |
| **Runtime** | Module 인스턴스를 띄우고, 데코레이터를 스캔해 메시지를 배선 | [framework/runtime/app.py](../backend/framework/runtime/app.py) |
| **Transport** | 실제로 bytes 를 네트워크로 실어 나름 (Zenoh) | [framework/transport/](../backend/framework/transport/) |

이 그림만 잡으면 나머지는 다 이 위에 얹힙니다.

---

## 3. Contract — 메시지의 형태

각 Module 은 `contract.py` 에서 자기가 노출하는 메시지를 정의합니다. 이게 그 Module 통신의 **단일 진실 원천(SSOT)** 입니다. 메시지는 세 종류뿐입니다:

- **Service** (`srv/...`) — 요청/응답. "이거 해주고 결과 돌려줘" (예: 모터 토크 켜기, 현재 관절 상태 읽기)
- **Stream** (`stream/...`) — 지속 발행. "계속 흘려보낼게, 최신값만 보면 돼" (예: 20Hz 모터 상태)
- **Event** (`event/...`) — 이산 이벤트. "이런 일이 일어났어" (예: 토크가 켜졌다)

실제 예시 — [modules/motor/contract.py](../backend/modules/motor/contract.py):

```python
class Motor:
    class Service(StrEnum):
        SET_TORQUE = "srv/motor/{robot_id}/set_torque"
        READ_STATE = "srv/motor/{robot_id}/read_state"

    class Stream(StrEnum):
        RAW_STATE = "stream/motor/{robot_id}/raw_state"  # 20Hz 관절 상태

    class Event(StrEnum):
        TORQUE_CHANGED = "event/motor/{robot_id}/torque_changed"
```

메시지의 **key(주소)** 규칙은 일정합니다:

```
srv|stream|event / <module> [/{robot_id}] / <name>
```

그리고 각 메시지가 실어 나르는 **payload 의 형태**는 pydantic 모델로 같은 파일에 정의합니다. 확정된 계약은 [StrictModel](../backend/framework/contract/model.py)(정의 안 한 필드가 오면 에러 — 오타·스키마 드리프트를 부팅 시점에 잡음)을 상속하고, 아직 형태가 굳지 않은 탐색 단계 계약은 `DraftModel` 을 씁니다.

### `{robot_id}` — robot-scoped vs host-level

key 에 `{robot_id}` 가 들어가는 Module 은 **로봇마다 인스턴스가 따로** 있습니다. 어느 로봇에 대한 명령인지 주소에 박혀 있어야 하기 때문입니다. 이런 Module 은 딱 4개입니다:

- `motor`, `camera`, `camera_decoded`, `motion`

나머지 Module 은 **host 당 하나**이고 로봇을 가리지 않습니다(robot-agnostic). 대상 로봇이 필요하면 요청 payload 의 `robot_id` 필드에서 받습니다. (자세한 규칙은 [docs/backend.md](backend.md) §2.7)

---

## 4. Runtime — Module 을 띄우고 배선한다

부팅 진입점은 [apps/main.py](../backend/apps/main.py) 입니다. 흐름은 이렇습니다:

1. `--host <이름>` 인자로 **배치 설정 yaml** 을 읽는다 (다음 절).
2. yaml 이 나열한 Module 이름들을 [apps/registry.py](../backend/apps/registry.py) 로 **그때그때 import** 한다 (lazy import — 라즈베리파이가 PC 전용 무거운 라이브러리를 안 끌고 오게 하려는 것).
3. 각 Module 을 인스턴스로 만들어 `Runtime` 에 등록한다.
4. `Runtime.start()` 가 각 인스턴스를 스캔해서, `@service`/`@subscriber` 로 표시된 메서드와 Mirror 를 Transport 에 배선한다.

Module 을 만들 때 **생성자 인자를 이름으로 주입**합니다 ([app.py:116](../backend/framework/runtime/app.py#L116) `add_module`). 생성자가 `runtime` 을 받으면 통신용 런타임 핸들을, `robot_id` 를 받으면 이 인스턴스가 담당할 로봇 id 를, DB 세션이 필요하면 세션 팩토리를 — 이름을 맞춰 꽂아줍니다.

한 가지 알아둘 것: **서비스 핸들러는 동기/비동기 둘 다 됩니다.** Zenoh 콜백은 워커 스레드에서 오는데, 핸들러가 `async` 면 Runtime 이 이벤트 루프로 넘겨 실행합니다 ([app.py:255](../backend/framework/runtime/app.py#L255) 근처). 그래서 Module 코드 어디서나 `await runtime.call(...)` 하나로 다른 Module 을 부를 수 있습니다.

### Mirror — 늦게 뜬 소유자에도 자동 수렴

어떤 Module 은 다른 Module 이 소유한 상태를 계속 최신으로 들고 있어야 합니다(예: motion 이 calibration 결과를 참조). 이를 위한 게 **Mirror** 입니다. 소비자는 (1) 시작 시 스냅샷을 한 번 당겨오고, (2) 변경 이벤트가 오면 다시 당겨오고, (3) 소유자가 늦게 떠도 liveliness 신호로 다시 당겨옵니다. 그래서 부팅 순서에 상관없이 결국 수렴합니다.

---

## 5. Transport — Zenoh

실제 bytes 운반은 [Zenoh](../backend/framework/transport/) 가 합니다. 특징:

- **bytes 만** 다룹니다. 직렬화(msgpack)는 Runtime 책임이고 Transport 는 관여 안 합니다.
- 같은 LAN 의 Zenoh peer 들이 **서로 자동으로 발견**합니다. 그래서 분산 배치에서 "누가 어디 있는지" 배선 설정이 따로 필요 없습니다.

### 에러는 어떻게 전파되나

성공/실패를 감싸는 응답 봉투가 **없습니다.** 서비스 핸들러가 예외를 던지면 그게 `RemoteError(type, message)` 형태로 wire 를 건너 호출자에게 그대로 도착합니다. 즉 "실패는 raise, 정상 결과는 반환" 이 규칙입니다.

---

## 6. 요청 하나가 흐르는 길 (end-to-end)

브라우저에서 "이 로봇 토크 켜"를 눌렀을 때 벌어지는 일 전체:

```
브라우저
  │  JSON 텍스트: {op:"service", key:"srv/motor/so101_6dof_0/set_torque", data:{enabled:true}}
  ▼
Bridge (modules/bridge) ── FastAPI WebSocket
  │  msgpack 봉투로 감싸 Zenoh 로 call
  ▼
Transport (Zenoh) ── key 로 라우팅
  ▼
Runtime ── payload 디코드 → 해당 Module 의 @service 메서드 호출
  ▼
MotorDriverModule.set_torque(req)  ── 실제 모터 드라이버 조작
  │  결과 반환 (또는 예외 raise)
  ▼
Runtime ── 응답 인코딩
  ▼
Transport → Bridge → 브라우저 (binary frame)
```

**Bridge** ([modules/bridge/](../backend/modules/bridge/))가 브라우저와 backend 사이 유일한 관문입니다. 브라우저→Bridge 는 JSON 텍스트, Bridge→브라우저 는 바이너리 프레임입니다:

```
[u8 ver=1][u8 type][u16 BE key_len][key utf8][payload(msgpack)]
  type=1 topic 데이터 / type=2 service 응답 / type=3 service 에러
```

느린 브라우저를 위해 채널별 송신 큐 정책이 다릅니다 — `stream/*` 는 최신값만(1개), `event/*` 는 순서 보존 FIFO(128개). 근거는 [ws.py](../backend/modules/bridge/ws.py) 상단 주석에 있습니다.

---

## 7. Module 목록 (14개)

[apps/registry.py](../backend/apps/registry.py) 가 전체 목록의 SSOT 입니다.

| Module | 역할 | robot-scoped? |
| --- | --- | --- |
| `motor` | 모터 드라이버(Dynamixel/Feetech/mock) + 20Hz 상태 발행 | ✅ |
| `camera` | 카메라(RealSense/USB웹캠/mock) — color JPEG + depth | ✅ |
| `camera_decoded` | 인코딩된 카메라 프레임을 디코드 (파생 읽기 모델) | ✅ |
| `motion` | MoveJ/MoveL/Jog + IK(역기구학) + TCP 상태 | ✅ |
| `calibration` | 캘리브레이션 산출물 소유 (캡처 세션 + DB) | |
| `scene3d` | RGBD → 실시간 포인트클라우드 | |
| `scan` | 스캔 세션 + ICP/TSDF 로 3D 재구성 | |
| `waypoint` | 웨이포인트/그룹 CRUD + 티칭 | |
| `detector` | 프롬프트 → 3D 물체 후보 (GroundingDINO/SAM2/mock) | |
| `llm` | 자연어 → 구조화된 pick/place 명령 (Qwen/mock) | |
| `pick_and_place` | Pick&Place 작업 (task Module 표준 레퍼런스) | |
| `bridge` | FastAPI — 브라우저 WS 중계 + `/contract.json` + MJPEG | |
| `logcollector` | 분산 로그 수집 (중앙 host 에서 파일로) | |
| `host_monitor` | 이 host 의 CPU/메모리 발행 (대시보드용) | |

각 Module 폴더 안에서 봐야 할 파일:
- `contract.py` — 이 Module 이 뭘 주고받는지 (여기부터 읽으세요)
- `module.py` — 실제 구현 (`@service`/`@subscriber` 메서드)
- `drivers/` — 하드웨어 종류별 구현 + `mock` (하드웨어 없이 개발용)

---

## 8. 배치 설정 — 같은 코드, 다른 yaml

"어느 기계가 어떤 Module 을 돌리나"는 전부 [config/deployments/](../backend/config/deployments/) 의 yaml 이 정합니다. 다섯 개가 있습니다:

| yaml | 의미 |
| --- | --- |
| `mock` | PC 한 대, 하드웨어 없이 전부 mock 드라이버 — 개발/테스트용 |
| `pc` | PC 한 대에 실제 하드웨어 붙임 |
| `pi_hori1` | 라즈베리파이 1 — SO-101 모터+모션 |
| `pi_hori2` | 라즈베리파이 2 — SO-101 D405 카메라 |
| `pi_hori3` | 라즈베리파이 3 — OMX 모터+모션+카메라 |

yaml 하나를 보면 구조가 다 보입니다 — [mock.yaml](../backend/config/deployments/mock.yaml):

```yaml
driver_mode: mock              # 실제 HW 대신 mock 드라이버
rdb_uri: "sqlite:///:memory:"  # 이 host 가 쓸 DB
modules:
  - name: motor
    robots: [so101_6dof_0, omx_f_0]   # robot-scoped → 로봇마다 인스턴스
  - name: calibration                  # robots 없음 → host 당 1개
  - name: bridge
  ...
```

**분산 배치의 핵심**: `pi_hori1` 은 modules 목록에 `motor`/`motion` 만, `pi_hori2` 는 `camera` 만 넣습니다. 세 파이가 같은 LAN 에서 Zenoh 로 자동 발견해 하나의 시스템처럼 동작합니다. 코드는 하나도 안 바뀝니다 — **yaml 만 다릅니다.**

---

## 9. 로컬에서 띄우기

가장 빠른 길 — 하드웨어 없이 mock 으로:

```powershell
cd backend
uv sync                                            # 최초 1회 (의존성 설치)
uv run --no-sync python -m apps.main --host mock   # bridge 가 :8000 에 뜸
```

뜨면 `GET /dev` (개발 콘솔)에서 프론트 없이 서비스를 직접 두드려볼 수 있고, `GET /contract.json` 에서 노출된 전체 계약을 볼 수 있습니다.

검사/테스트:

```powershell
uv run ruff check .                        # 린트
uv run pyright                             # 타입 체크
uv run --no-sync pytest -m "not sim" -q    # 빠른 테스트 (수 초)
uv run --no-sync pytest -q                 # 전체 (~90s, 시뮬레이션 포함)
```

> ⚠️ 실행/검증용으로 띄운 backend 는 **그 세션 안에서 반드시 종료**하세요. 유령 프로세스가 `:8000` 을 잡고 있으면 이후 테스트가 조용히 멈춥니다.

---

## 10. 규약 (코드 짤 때 지킬 것)

- **린트/타입**: ruff (line-length 88, py311) + pyright. 둘 다 0 이어야 함.
- **wire 스키마 이름은 verb-first** (Google AIP 스타일): `StartRunRequest`, `ListRunsResponse`.
- **저장하는 timestamp 는 UTC-aware datetime** (`datetime.now(UTC)`). float epoch 은 wire 봉투/경과시간 측정 같은 경계에서만.
- **저장소 구현체**는 [infra/](../backend/infra/) 아래 — DB(`database/`: sqlite/postgres), 오브젝트 스토어(`object_store/`: filesystem/minio), 로깅(`logging/`), ML 로더(`ml/`).
- `contract.py` 를 바꾸면 프론트엔드 타입도 재생성해야 합니다 (`pnpm gen:types` — [docs/frontend.md](frontend.md)).

---

## 11. 더 깊이 볼 곳

이 문서는 지도입니다. 특정 도메인을 파고들 때:

| 궁금한 것 | 문서 |
| --- | --- |
| framework 상세 스펙, Module 카탈로그 | [docs/backend.md](backend.md) |
| 프론트엔드 구조 | [docs/frontend.md](frontend.md) |
| 하드웨어(로봇/카메라/파이 토폴로지) | [docs/hardware.md](hardware.md) |
| 캘리브레이션 | [docs/calibration.md](calibration.md) |
| 모션(MoveJ/MoveL/Jog/IK) | [docs/motion.md](motion.md) |
| 인식(검출/스캔/포인트클라우드) | [docs/perception.md](perception.md) |
| Task 아키텍처 (Pick&Place 등) | [docs/task.md](task.md) |
| DB 스키마, 검증 방법론 | [docs/dev_reference.md](dev_reference.md) |
