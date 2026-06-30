# backend_v2 구현 status + 다음 세션 handoff

> 새 세션이 **바로 이어서 구현**할 수 있게 박은 status. SSOT spec = [backend_v2.md](backend_v2.md), 모듈 catalog = [backend_v2_modules.md](backend_v2_modules.md). 본 문서 = "지금 어디까지 됐고 다음 뭐 할지 + C2 분석 완료본".
>
> **다음 작업 = C2 (frontend 적응) → 집에서 frontend 로 실 SO-101 jog** ([[project-v2-home-robot-goal]]).

## 현재 상태 (2026-06-30)

backend_v2 = framework 재작성. **Motion D3(jog)까지 완료. 123 test PASS, ruff + pyright clean.**

| 영역 | 상태 |
|---|---|
| framework (contract/runtime/transport/persistence/storage) | ✅ |
| infra (zenoh / sqlite·postgres / fs·minio) | ✅ |
| modules/motor (mock + 실 feetech) | ✅ (실 feetech 하드웨어 미검증) |
| modules/camera (mock + 실 realsense_d405) + camera_decoded | ✅ (실 realsense 하드웨어 미검증) |
| modules/bridge (HTTP /robots·/system + WS relay + MJPEG + /robot static mount) | ✅ |
| apps (lazy registry / resolve / main / config) | ✅ |
| robot_v2/ (v2 소유 robot config — robots.yaml lean + so101_6dof/{motors,motion,urdf} + instances) | ✅ |
| Motion D1 kinematics(PyBullet, dof=6) / D2a motor command wire / D2b MoveJ+TCP state / D3 Jog(JogJ/JogTcp) | ✅ |

**검증** (cwd 반드시 `backend_v2/`):
```bash
cd backend_v2
uv run --no-sync pytest -q          # 123 passed
uv run --no-sync ruff check .
uv run --no-sync pyright
uv run --no-sync python -m apps.main --host mock   # 실 boot (motor/motion/camera/camera_decoded)
```

## 아키텍처 불변식 (절대 어기지 말 것 — 포팅 시 [[feedback-port-keep-v2-arch]])

- **레이어링**: `modules/` 는 `apps/` import 금지. 다른 모듈 contract import 는 OK.
- **role 격리 (lazy registry)**: `apps/registry.py` = name→"path:Class" string, importlib lazy. `apps/resolve.py` = name dispatch + branch 안 lazy import. → host 가 자기 deployment 모듈만 import (pi_camera 가 pybullet/fastapi 안 끌어옴). **eager import 금지** (test_boot 에 subprocess 격리 테스트 있음).
- **raw↔rad = Motion 책임** (§4). MotorDriver 는 순수 raw. Motion 이 `Motor.Stream.RAW_STATE` 받아 rad, 명령은 rad→raw 후 `Motor.Stream.COMMAND` publish.
- **contract.py 컨벤션**: nested `Service`/`Stream`/`Event` StrEnum (`srv/`/`stream/`/`event/` path key). stream/event payload 에 `robot_id` + `seq` + `timestamp_unix` (§8.5). **Stream key 는 채널 정의 모듈 contract 에** — output 이면 발행, input(jog/command)이면 구독 (예: `Motion.Stream.JOG_J` 는 frontend 발행, Motion 구독).
- **Bridge = relay only** — `RawTransport`(close/register_service 없음), raw bytes msgpack forward, domain logic 0.
- **robot_v2 robots.yaml = lean** (calib 파라미터 pose_recommend_strategy/wrist_roll/sag 제외 — Step E). vendor(`motor_backend`) ≠ `driver_mode`(deployment real/mock).
- Motion = **pi_motor** (100Hz 명령 network 안 넘게). dof = arm only (gripper 제외, tcp ancestor chain).
- **안전 수치 임의 금지**: limit=motors.yaml(실측), 속도=motion.yaml. jog 도 거기에 clamp + IK reject. 새 값 필요하면 사용자에게 꺼내 보여줄 것, 추측 X.
- 테스트는 통과용 X — 실제 동작/정확성/invariant ([[feedback-meaningful-tests]]). 예: MoveJ/jog 는 e2e(mock motor 가 target 도달), role 격리 subprocess.

## 다음 = C2 (frontend 적응) — 분석 완료, 바로 구현 가능

**목표**: 기존 frontend(React/TS/pnpm, `frontend/`)를 backend_v2 wire 에 맞춰 → 집에서 frontend 로 실 로봇 jog. (gamepad 는 frontend jog 안심 후 — 후순위, **순서는 사용자 결정**.)

frontend 결합은 `frontend/src/api/bridge.ts` (transport) + `generated/contract.ts` (키/타입, 옛 backend gen 산물) 에 집중. UI(10k LOC)는 wire-agnostic.

### C2b-1 — transport (bridge.ts), self-contained, 키/타입과 독립
backend_v2 wire (C1b 에서 박음):
- browser→bridge: **JSON 텍스트** `{op, ...}` (op = subscribe/unsubscribe/publish/service). ⚠️ 현재 frontend 는 `{type, ...}` → **`type`→`op` rename** (bridge.ts `_send` + types/bridge.ts).
- bridge→browser: **binary 프레임** `[u8 ver=1][u8 type][u16 BE key_len][key utf8][payload]`. type 1=topic_data(payload=msgpack), 2=service_response(key=request_id, payload=msgpack `{timestamp,data}`), 3=service_error(payload=msgpack `{type,message}`).
- 변경: `@msgpack/msgpack` dep 추가. incoming binary → type 1: msgpack-decode → topicListeners(객체) / 또는 binaryTopicListeners(raw). type 2: resolve service. type 3: error. **옛 JSON-text incoming 경로 제거**(backend 는 JSON 안 보냄).
- **service shim**: backend 는 exception 모델. bridge.ts 에서 `{success,message,data}` shape 로 매핑(type2→success:true+data / type3→success:false+message) → `framework/{service,store}.ts` + UI **무변경**.

### C2c — generated contract 재생성 (덩어리)
frontend 의 `Topic`/`ServiceKey`/`ServiceMap`/`TopicPayloadMap` = `@/api/generated/contract` (옛 backend `api_contract.py` → `pnpm gen:types`(openapi x-contract) 산물, 옛 키 `horibot/...`). backend_v2 는 모듈별 `modules/*/contract.py` 가 SSOT → **contract.py introspect 하는 새 generator** 필요 (= §8 gen:types/contract viewer 의 frontend 절반). frontend 타입 backbone 전체 교체. (최소 jog 만이면 키 hand-write 도 가능하나 정석은 generator.)

### C2d — jog rewire + 3D
- JogTcp/JogJ 패널([frontend/src/components/panels/motion/Jog*.tsx](frontend/src/components/panels/motion/)) → 새 키 `stream/motion/{robot_id}/jog_tcp`·`jog_j` publish + **payload 에 `robot_id` 포함** (Motion wildcard 구독 후 payload.robot_id self-filter — 현재 frontend 는 key 만 확장, payload 에 robot_id 없음).
- 3D joint-state ← `Motor.Stream.RAW_STATE`(raw) 또는 `Motion.Stream.TCP_STATE`. URDF/mesh ← `GET /robot/...` (Bridge static mount C2a 완료).
- `constants/index.ts`: `DEFAULT_ROBOT_ID` `omx_f_0`→`so101_6dof_0`.

**검증**: `cd frontend; pnpm lint; pnpm build` + 브라우저↔**mock backend**(`python -m apps.main --host mock`) jog (회사 가능, 하드웨어 불요) → 실 로봇 jog (집).

## C2 이후
- **D2c** — cartesian MoveL/C/P (TrajectoryRunner.run_cartesian 이미 port 됨, Motion 에 서비스만 노출 + **6DOF orientation IK 보강**: 현재 `_solve_ik` 가 position-only).
- **Step E+** — Calibration(DB+ObjectStore, calib 파라미터 여기서 자리) / Detector / Scene3D / Scan / Reconstruction / Task / Gamepad. catalog §11 build order.

## 하드웨어 미검증 (집에서) + 리뷰(2026-06-30) latent
- **실 driver 검증 0**: `feetech.py`(register map/sync/signed/clamp), `realsense_d405.py`(pipeline/align). faithful port, 통신 안 해봄.
- **gap (집에서 보고 wire 판단)**: motors.yaml `pid`/`profile` 가 실 모터에 미적용 (feetech driver 가 EEPROM default 사용, `release/restore_profile` no-op). 모션 느리거나 진동 시 wire (EEPROM write-once 주의).
- **latent (해당 step)**: color+depth stream 페어링(독립 seq → Scene3D Step G 때 공유 frame seq) / Mirror refetch coalescing·stale 순서(D4) / pc.yaml 에 bridge 미배치(C2) / discover_services instance getattr(property 평가) / publish on async-loop(대형 payload watch) / Minio 예외·list semantics(Phase 3).
