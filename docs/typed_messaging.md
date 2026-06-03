# Typed Messaging — 결정 기록

토픽/서비스 페이로드를 `dict` → **pydantic 모델** 로 옮긴 작업의 결정 기록.
구현 완료 (2026-06-03). 본 문서는 *왜 이렇게 짰는지* 후속 작업자용 reference.

## 왜 했나

기존 모든 토픽/서비스 payload 가 `dict`. 문제:

- pyright 가 필드 typo / 누락 못 잡음
- 분산 노드에서 *다른 머신이 보낸 dict* 모양 어긋나도 런타임 깊은 자리에서 KeyError
- frontend 가 dict 모양 추측해서 보내는데 schema 가 단일 출처로 없음

pydantic 도입 후:
- 잘못된 모양은 `ValidationError` 로 즉시 잡힘
- 모델 정의 자체가 schema 의 single source
- frontend 가 backend 모델을 그대로 codegen 으로 받음

## 결정 사항

논의 거치며 정한 것들 — 흔들면 안 됨.

### 1. 페이로드만 typed, 토픽 키는 Literal 그대로

`Topic.MOTOR_STATE_JOINT = "omx/motor/state/joint"` 같은 string 상수 *그대로 둠*.
`TopicKey[T]` 같은 typed key 안 만듦.

**이유:** 토픽 키는 frontend / bridge / `_ALWAYS_SUBSCRIBE` / `topics.ts` 등 여러
자리에서 raw string 으로 쓰임. 키 표현 바꾸면 그 자리들 다 손봐야 함.

**잃는 것:** `self.publish(Topic.X, ModelY(...))` 같은 *키↔페이로드 짝 실수* 를
pyright 가 못 잡음 → 런타임 subscriber 의 ValidationError 로 잡힘. 분산에서
빠르게 발견 가능 → 허용.

### 2. 서비스 응답은 `{success, message, data}` envelope 유지

```python
class ServiceResponse(BaseModel, Generic[T]):
    success: bool
    message: str = ""
    data: T

class ServiceRequest(BaseModel, Generic[T]):
    timestamp: float
    data: T
```

bridge / `BridgeClient.callService` 가 envelope 가정해서 짜여있음. flatten 하면
그 공통 처리 깨짐. timestamp 는 분산 latency 측정 / 디버깅용으로 유지.

### 3. subscribe 시 모델 클래스 명시 (ROS 스타일)

```python
self.create_subscriber(
    Topic.MOTOR_STATE_JOINT,
    MotorJointState,                       # ← payload 타입 인자로
    self._on_state,
)
```

ROS2 가 동일 패턴. 콜백 어노테이션에서 타입 inspect 하는 hack 보다 명시적.

### 4. 공통 값 클래스 (Position3, Pose6, Detection, Quaternion) 도 pydantic

`core/values.py` 에 pydantic BaseModel. task 외 (detector, motion, calibration)
에서도 쓰니까 core 자리 맞음.

**그대로 두는 거:** `Slot[T]`, `StepResult` 는 *데이터 값 아니라 참조 / 메타* —
`modules/task/schema.py` 에 dataclass 그대로.

### 5. MotionCommand 내부 dict API 안 건드림

서비스 핸들러 경계만 typed. `MotionCommand.validate(req)` / `.execute(req_urdf, ...)`
는 dict 받음 — 핸들러가 `{"data": req.data.model_dump()}` 로 envelope wrap 해서
넘김. cmd 내부 안 건드림 (scope 밖).

## 폴더 구조

```
backend/
├── core/
│   └── transport/
│       ├── messages/
│       │   ├── base.py        # ServiceRequest[T] / ServiceResponse[T] / EmptyData
│       │   ├── motor.py       # MotorJointState, MotorCmd, MotorEnableReq, ...
│       │   ├── camera.py
│       │   ├── motion.py
│       │   ├── task.py
│       │   ├── detector.py
│       │   ├── calibration.py
│       │   ├── pointcloud.py
│       │   └── system.py
│       ├── base_node.py       # publish/subscribe/service 시그니처 typed
│       └── topic_map.py       # string 상수 (그대로)
├── api_contract.py            # frontend 공개 surface (SSOT)
└── modules/task/schema.py     # Slot[T], StepResult (값 클래스 빼고)
```

## Typed 면제 자리 (의도적 free-form)

```
modules/calibration:
- CALIB_HANDEYE_COMPUTE     — 응답 ~25 동적 필드 (BA mode 분기)
- CALIB_HANDEYE_THRESHOLDS  — thresholds.as_dict() free-form
- CALIB_HANDEYE_PREVIEW     — CHESSBOARD 검출 메타 (optional 분기)

modules/task:
- TASK_RUN / TASK_PREVIEW   — factory 동적 인자 (task name + extras)
- TASK_STATUS               — state.to_dict() free-form
- TASK_TREE / TASK_STATE / TASK_STEP_RESULT — Step 재귀 union + typed value class union
```

면제 카테고리 + 이유:

| 카테고리 | 예 | 이유 |
|---|---|---|
| 동적 빌드 | `CALIB_HANDEYE_COMPUTE` | 알고리즘 분기로 필드 가/감. typed 시 알고리즘 iteration 마찰 ↑ |
| 자유 직렬화 | `TASK_STATUS`, `CALIB_HANDEYE_THRESHOLDS` | `.to_dict()` / `.as_dict()` 자유 dict. typed 시 *수동 sync* 자리 추가, 가치 작음 |
| 재귀 union | `TASK_TREE`, `TASK_STEP_RESULT` | Step DSL 재귀 + typed value class union. discriminator + recursive ref codegen 복잡, frontend 가 어차피 type 별 dispatch |
| factory 동적 인자 | `TASK_RUN` | task 별 인자 모양 다름. typed 시 새 task = 별도 schema 자리 추가 |

원리: schema 가 *코드 변경 마찰* 보다 *런타임 안전성* 이익이 명백한 자리만
typed. OpenAPI / gRPC / GraphQL 도 *동적 dict* 는 `additionalProperties:{}` /
`google.protobuf.Struct` / `JSONObject` 같은 escape hatch 제공 — 같은 자리에서
같은 결정.

## Binary 트랙 (typed 면제 + 별도 라우팅)

- `CAMERA_STREAM_RAW` (JPEG) — bridge MJPEG `/camera/stream` HTTP 라우트
- `CAMERA_DEPTH_FRAME` (header+JPEG+zstd) — pointcloud_node 만 구독, frontend 미사용
- `POINTCLOUD_STREAM` (xyz+rgb binary) — bridge binary WS frame 으로 직송

이 자리들은 `base_node.publish` 안 거치고 zenoh session 직접 호출. typed 면제.

## Frontend Contract — `api_contract.py`

frontend 공개 surface 의 SSOT. 산업 표준 (tRPC `appRouter` / Connect-RPC
`.proto` / ts-rest `initContract`) 의 "하나의 명시적 contract object + opt-in"
원리를 우리 Zenoh-over-WS transport 위에 재구현.

흐름:

```
backend/api_contract.py (SSOT)
  ├─ PUBLIC_TOPICS         (frontend 구독)
  ├─ PUBLIC_BINARY_TOPICS  (raw bytes 트랙)
  └─ PUBLIC_SERVICES       (frontend 호출)
       │
       ▼ bridge custom_openapi() → /openapi.json 에 x-contract 인라인
       ▼
frontend pnpm gen:types
  ├─ openapi-typescript → src/api/generated/types.ts (모델 정의)
  └─ scripts/gen-contract.mjs → src/api/generated/contract.ts
       (Topic / ServiceKey 상수 + TopicPayloadMap / ServiceMap 타입)
```

새 frontend-facing service 추가:

```python
# backend/api_contract.py
Service.MOTOR_X: (MotorXReq, MotorXRes),   # ← 1줄
```

→ backend 재시작 → `pnpm gen:types` → 프론트 자동완성.

새 *internal* service 추가:
- contract 안 건드림 → 자동으로 internal (frontend 키 자체가 contract.ts 에 없음)

산업 표준 A/B/C 비교 + 우리 transport (Zenoh) 위에 재구현 이유는 git history
(이 문서의 이전 revision) + 산업 표준 코드 사례 — tRPC appRouter / Buf .proto
/ ts-rest initContract 다 같은 원리 (단일 명시적 contract + opt-in).

### Internal 자리 (의도적 미등재)

```
Topics:   Topic.CAMERA_DEPTH_FRAME    — pointcloud_node 만 구독 (binary)
Services: Service.MOTOR_GRIPPER           — task / gamepad 만 호출
          Service.MOTOR_SET_PROFILE_ALL   — motion_node 만 호출
          Service.CAMERA_SET_DEPTH_STREAM — detector / pointcloud 만 호출
          Service.DETECT_SERVICE          — 내부 click-to-detect (frontend 미사용)
          Service.SYSTEM_NODE_STATUS      — 미구현
```

## 진행 상태 (완료)

- [x] motor / camera / motion / detector / pointcloud / calibration / task
      슬라이스 — 면제 자리 제외 모두 typed (2026-06-03)
- [x] core/ 폴더 재구성 (transport / coords / cache / robot / top)
- [x] Frontend contract 패턴 (`api_contract.py` + x-contract OpenAPI extension +
      `gen-contract.mjs` codegen + `BridgeClient` generic 화) (2026-06-03)
