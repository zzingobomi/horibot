# Typed Messaging Migration

토픽/서비스 페이로드를 `dict` → **pydantic 모델** 로 옮기는 작업의 plan + 결정 기록. 이 문서를 보고 작업 이어가면 됨.

## 왜 하는가

현재 모든 토픽/서비스 payload 가 `dict` 로 흐름:

```python
self.publish(Topic.MOTOR_STATE_JOINT, {"timestamp": ..., "joints": [...]})

def handler(req: dict) -> dict:
    enable = req.get("data", {}).get("enable", True)
    return {"success": True, "message": "ok", "data": {...}}
```

문제:
- pyright 가 필드 typo / 누락 못 잡음 (`req["enabled"]` 인지 `req["enable"]` 인지 코드 봐야 앎)
- 분산 노드에서 *다른 머신이 보낸 dict* 모양이 어긋나도 런타임 깊은 자리에서 KeyError / AttributeError 로만 터짐
- frontend 가 dict 모양 추측해서 보내는데 schema 가 단일 출처로 없음

pydantic 도입해서:
- 잘못된 모양은 `ValidationError` 로 즉시 잡음
- 모델 정의 자체가 schema 의 단일 출처 (필드 명/타입/필수 여부)
- IDE autocomplete (`req.data.enable` 가 점 자동완성)

## 결정 사항 (확정)

논의 거치며 정한 것들 — 작업 중 흔들리지 말 것.

### 1. 페이로드만 typed, 토픽 키는 Literal 그대로

`Topic.MOTOR_STATE_JOINT = "omx/motor/state/joint"` 같은 string 상수 *그대로 둠*. `TopicKey[T]` 같은 typed key 안 만듦.

**이유:** 토픽 키는 frontend / bridge / `_ALWAYS_SUBSCRIBE` / `topics.ts` 등 여러 자리에서 raw string 으로 쓰임. 키 표현 바꾸면 그 자리들 다 손봐야 함. 페이로드만 바꾸면 이 자리들 *건드릴 필요 없음*.

**잃는 것:** `self.publish(Topic.MOTOR_STATE_JOINT, MotorCmd(...))` 같은 *키↔페이로드 짝 실수* 를 pyright 가 못 잡음. 런타임 subscriber 의 ValidationError 로 잡힘. 분산에서 빠르게 발견 가능 → 허용.

### 2. 서비스 응답은 `{success, message, data}` envelope 유지

```python
# core/service_envelope.py
class ServiceResponse(BaseModel, Generic[T]):
    success: bool
    message: str = ""
    data: T

class ServiceRequest(BaseModel, Generic[T]):
    timestamp: float
    data: T
```

bridge / `BridgeClient.callService` 가 envelope 가정해서 짜여있음 (`success` 체크 후 `data` 꺼냄). flatten 하면 그 공통 처리 깨짐. timestamp 는 분산 latency 측정 / 디버깅용으로 유지.

### 3. subscribe 시 모델 클래스 명시 (ROS 스타일)

```python
self.create_subscriber(
    Topic.MOTOR_STATE_JOINT,
    MotorJointState,                       # ← payload 타입 인자로
    self._on_state,
)
```

ROS2 가 동일 패턴 (`self.create_subscription(MotorState, 'topic', cb, 10)`). 콜백 어노테이션에서 타입 inspect 하는 hack 보다 명시적.

### 4. 공통 값 클래스 (Position3, Pose6, Detection, Quaternion) 도 pydantic

지금 `modules/task/schema.py` 에 dataclass 로 있는 거 → `core/values.py` 로 옮기고 pydantic BaseModel 로 변환. task 외 (detector, motion, calibration) 에서도 쓰니까 core 자리 맞음.

**그대로 두는 거:** `Slot[T]`, `StepResult` 는 *데이터 값 아니라 참조 / 메타* — `modules/task/schema.py` 에 dataclass 그대로.

## 폴더 구조

```
backend/
├── core/
│   ├── values.py              # Position3, Pose6, Quaternion, Detection (공통 값)
│   ├── service_envelope.py    # ServiceRequest[T] / ServiceResponse[T] 제네릭
│   ├── topic_map.py           # 지금 그대로 — string 상수
│   ├── base_node.py           # publish/subscribe/service 시그니처 typed
│   └── messages/              # 도메인별 payload schema
│       ├── __init__.py
│       ├── motor.py           # MotorJointState, MotorCmd, MotorEnableData...
│       ├── camera.py
│       ├── motion.py
│       ├── task.py
│       ├── detector.py
│       ├── calibration.py
│       ├── pointcloud.py
│       └── system.py
└── modules/task/schema.py     # Slot[T], StepResult, SlotOr (값 클래스만 빼고 유지)
```

### Frontend (`topics.ts`) 는 안 건드림

frontend 도 typed 로 가져가려면 별도 작업 (Python schema → JSON Schema → TS 코드젠). 이번 스코프 *밖*. frontend 는 그대로 dict 사용, bridge 가 JSON bytes 그대로 passthrough.

## 마이그레이션 방식 — 노드별 vertical slice

처음엔 "전체 한 번에 쫙" 으로 가려 했는데 실측 보니 무리:
- 토픽 12 + 서비스 ~30 = ~70 schema 클래스
- 일부 payload 동적 dict 구성 (`diag["recommendations"] = ...`) — 정확 모델링 어려움
- motion_node 의 cartesian handler 가 mid-handler 에서 req dict 변형 → 내부 API 가 dict 기반

**한 노드씩, 검증 통과 후 다음** 으로 전환:

1. **모터 노드 먼저** (가장 단순) — base_node API + motor schemas + 모터 노드 migrate
2. pyright 통과 → 실제 모터 Pi 에서 `python main.py` 돌려서 동작 확인
3. 동작 OK → 다음 노드 (카메라 → 모션 → ...) 동일 패턴
4. 동작 NG → 모터 노드 디자인 수정 (이 단계서 발견하면 손볼 곳 1 곳)

"하나하나" 가 아니라 *디자인 검증 단계*. 비유: 새 build system 도입할 때 작은 모듈로 빌드 테스트 후 전체 확장.

## 노드 마이그레이션 순서

복잡도 / 종속성 고려:

| # | 노드 | 복잡도 | 비고 |
|---|---|---|---|
| 1 | motor | 낮음 | 가장 단순. base_node API 검증 시작점 |
| 2 | camera | 낮음 | publish 중심, 서비스 1 개. raw_subscriber 무시 (depth_frame 은 binary) |
| 3 | gamepad | 낮음 | 단순 |
| 4 | system / heartbeat | 낮음 | base_node 자체에 묻혀있음 |
| 5 | motion | **높음** | MoveJ/L/C/P 각각 다른 req. cartesian handler 의 mid-handler dict mutation 처리 필요 |
| 6 | detector | 중 | YOLO + Grounding DINO 각각. response 모양 다름 |
| 7 | task | 중-높 | TASK_TREE 재귀 구조, state.to_dict 직렬화 |
| 8 | calibration | **높음** | 서비스 9 개, handeye/compute 응답 ~20 필드 |
| 9 | pointcloud | 중 | 세션/scan/mesh 서비스들 |

## 각 노드에서 할 일 (체크리스트 템플릿)

각 노드마다:

- [ ] `core/messages/<domain>.py` 에 그 노드의 토픽 / 서비스 payload 모델 정의
  - publish 하는 토픽: 모델 1 개씩
  - 서비스: request data 모델 1 개 + response data 모델 1 개씩
- [ ] `nodes/<domain>_node.py` 수정:
  - `self.publish(Topic.X, {dict})` → `self.publish(Topic.X, ModelX(...))`
  - `def handler(req: dict) -> dict` → `def handler(req: ServiceRequest[XData]) -> ServiceResponse[YData]`
  - 핸들러 내부 `req.get("data", {}).get("foo")` → `req.data.foo`
  - return dict → return `ServiceResponse(success=..., data=YData(...))`
- [ ] 모터 노드 처음에는 `base_node.py` 의 publish/subscribe/service signature 도 같이 수정
- [ ] `uv run pyright` 0 error
- [ ] 실제 노드 실행해서 동작 확인

## 진행 상태

- [x] 결정 사항 합의 (위 4 개)
- [x] 폴더 구조 합의
- [x] 노드 5 개 코드 survey (motor / camera / motion / detector / task)
- [x] **모터 노드 vertical slice 완료** (2026-06-03)
  - `core/values.py` (Position3/Quaternion/Pose6/Detection pydantic BaseModel 로 promote)
  - `core/messages/base.py` 에 `ServiceRequest[T]` / `ServiceResponse[T]` / `EmptyData` 통합
    (기존 multi_robot_architecture.md §7.6 의 `base.py` 가 이미 envelope 보유 — `core/service_envelope.py` 별도 파일 안 만들고 합침. plan 의 "core/service_envelope.py" 항목은 폐기)
  - `core/messages/motor.py`, `core/messages/system.py` 새 schema
  - `core/base_node.py` 에 typed `publish` / `create_subscriber(topic, ModelCls, cb)` / `create_service(key, ReqCls, ResCls, h)` / `call_service(key, ReqModel, ResCls)` 오버로드 추가 — legacy dict API 도 살아있어 다른 노드 영향 X
  - `nodes/motor_node.py` 완전 typed (state/cmd publish/subscribe + 6 service handlers)
  - 모터 service 호출자도 같이 typed: `motion_node._set_arm_profile`, `gamepad_node` 의 `MOTOR_ENABLE` / `MOTOR_GRIPPER`, `task/steps.py::Gripper`
  - `modules/task/schema.py` 는 `core.values` 재export 로 호환 유지
  - pyright 41 → 0 (다른 pre-existing 에러도 같이 정리 — pyrealsense2 Any rebind, gamepad None narrowing, MoveTcpFn signature, opencv `# type: ignore` 등)
- [x] **카메라 노드 vertical slice 완료** (2026-06-03)
  - `core/messages/camera.py` 새 schema — `CameraStatus` (state publish) / `CameraSetDepthStreamReq` / `CameraSetDepthStreamRes`
  - `nodes/camera_node.py` typed — `_publish_status` → `CameraStatus`, `_srv_set_depth_stream` → `ServiceRequest[CameraSetDepthStreamReq] → ServiceResponse[CameraSetDepthStreamRes]`
  - `core/frame_cache.py` — `_latest_status_by_robot: dict[str, CameraStatus]`, `.width`/`.height` attribute 접근 (기존 `.get("width")` 대신), `node.create_subscriber(CAMERA_STATE_STATUS, CameraStatus, ...)` typed
  - service caller 도 같이 typed: `detector_node` 의 `_handle_grounded_detect` 안 `CAMERA_SET_DEPTH_STREAM` 호출, `pointcloud_node` 의 `_srv_configure` 안 동일 호출 — req `CameraSetDepthStreamReq` / res `CameraSetDepthStreamRes` 명시
  - `CAMERA_STREAM_RAW` (JPEG bytes) / `CAMERA_DEPTH_FRAME` (header+JPEG+zstd) 은 binary raw 트랙 — 본 슬라이스 scope 밖 (typed_messaging.md §미해결 #1, #2)
  - pyright 0
- [x] **camera 슬라이스 직전: 폴더 구조 / 파일 네이밍 검토 + 정합화** (2026-06-03)
  - **계기**: 작업 중 `core/realsense_capture.py` 가 leftover (May 9 ca6cc04 의 raw SDK wrap, Jun 2 4365171 architecture 리팩토링에서 `modules/camera/adapters/realsense.py` 가 위에 추가됐는데 원본은 core 에 남음) — `pyrealsense2` import 가 `core/` 에 있다는 시점에 이상하다 알아챘어야. 옮긴 후 발견.
  - **검토 결과 / 적용**:
    - `core/` — `realsense_capture.py` leftover 는 Jun 2 4365171 시점에 이미 `modules/camera/adapters/` 로 이전됨. 잔존 hardware/도메인 specific 파일 없음. **회색 한 개**: `core/gripper_setup.py` — task-specific (self-play 시절 잔재). task 슬라이스 (#7) 시 `modules/task/` 로 이전. **추가 발견**: `core/` 18+ 파일 평면 = 분류감 부족. transport / calibdata / cache / robot / top 으로 그룹화 후보. typed_messaging 슬라이스 끝나고 `core/messages/` 안정된 후 별도 turn 으로.
    - `modules/<domain>/` — 네이밍 정합화:
      - `realsense_capture.py::RealsenseCapture` (raw) → `realsense_driver.py::RealsenseDriver`
      - `realsense.py::RealSenseCapture` (impl) → `realsense_capture.py::RealsenseCapture`
      - Protocol `CameraCapture` 이름 유지 (도메인 어휘). 결과 (Protocol `CameraCapture` ← impl `RealsenseCapture` ← raw `RealsenseDriver`) 가 motor (Protocol `MotorBackend` ← impl `DynamixelBackend` ← raw `DynamixelDriver`) 와 동형
    - `modules/dynamixel/` 빈 폴더 (Jun 2 리팩토 leftover) 삭제
    - kinematics / detector / motion / task / pointcloud / calibration — 도메인 안 일관성 OK. detector 의 `BaseDetector` ABC → Protocol 전환은 detector 슬라이스 (#6) 때 함께

## 작업 재개 시 첫 번째 prompt 추천

> typed_messaging.md 보고 폴더 구조 / 네이밍 검토부터 해줘. 그 다음 다음 노드 슬라이스.

## 미해결 / 결정 필요한 자잘한 거

작업하면서 부딪힐 거:

1. **`base_node.publish` 가 일부 자리에서 raw bytes 받음** — camera_node 의 `session.put(Topic.CAMERA_STREAM_RAW, jpeg_bytes)`. 이건 `BaseNode.publish` 안 쓰고 zenoh session 직접 호출. 그대로 둠 (raw 트랙).
2. **`create_raw_subscriber`** — depth_frame, camera_stream_raw 같은 binary 전용. 시그니처 안 바꿈 (bytes 그대로).
3. **`call_service` 의 응답 파싱** — 서비스 client (다른 노드가 service 호출) 도 응답 모델 명시해야 함. 시그니처: `call_service(Service.X, ReqData(...), ResData) -> ServiceResponse[ResData]`. 모터 노드 작업 시 motion_node 도 motor.set_profile_all 호출하므로 이 자리도 동시에 손봐야.
4. **MotionCommand 내부 dict API** — `cmd.validate(req)` / `cmd.execute(req_urdf, ...)` 가 dict 받음. 서비스 핸들러 경계만 typed 하고 cmd 호출 시 `req.data.model_dump()` 로 다시 dict 변환해 넘김. cmd 내부 안 건드림 (scope 밖).
5. **service handler `req` 인자 unused 케이스** — `_handle_stop(_req)` 같은 자리. 모델은 정의 (`StopRequestData = EmptyData` 패턴) — generic envelope 가 data 필수라 빈 모델이라도 필요.
