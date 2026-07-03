# backend_v2 — 진행 status + 다음 세션 handoff

> 새 세션이 **바로 이어서 작업**할 수 있게 박은 status. 아키텍처 SSOT =
> [backend_v2.md](backend_v2.md) (framework §1–§14 + Module catalog §16 + Task-first §17).
> 본 문서 = "지금 어디까지 됐고 다음 뭐 할지" 만 — 설계 결정은 여기 안 둠.

## 현재 상태 (2026-07-03)

**framework + 10 Module 전부 가동 + robot-agnostic 스코프 리팩터 완료.**

검증 (전부 실행 확인, 2026-07-03):

| 층 | 결과 |
|---|---|
| backend pytest | **212 PASS** (모듈별 so101 6DOF + omx 5DOF multi-robot 눈속임 방지 포함) |
| ruff / pyright | 0 / 0 |
| frontend vitest / lint / tsc | **47 PASS** / 0 / 새 에러 0 (pre-existing jest-dom 2건만 — [[project-frontend-v2-build-prexisting-fail]]) |
| **Playwright e2e (headed)** | **14/14** — jog 50Hz full wire / calibrate `CALIB_SIM_BOARD=1` capture over-wire / scan 세션+캡처 / waypoint 티칭+group / contract-graph 9노드 |
| mock 실부팅 | 전 Module host-level/scoped 정상 add+start, 에러 0 |

**집 하드웨어 검증 (2026-07-02)**: frontend_v2 → backend_v2 wire → 실 SO-101 **TCP jog**
동작 확인 (C2 transport + JogTcp→IK→feetech + 토크 enable).

| 영역 | 상태 |
|---|---|
| framework (contract/runtime/transport/persistence/storage/Mirror) | ✅ (Mirror 는 consumer 0 — deferred, spec §3.3) |
| infra (zenoh / sqlite·postgres / fs·minio) + 루트 alembic | ✅ |
| motor (mock + 실 feetech) / camera (mock + realsense_d405) / camera_decoded | ✅ (실 feetech TCP jog 검증됨. realsense·PID/profile 미검증 — 아래) |
| motion — D1 kinematics(dof=6) / D2 MoveJ+TCP state / D3 Jog / **MoveL v1 + await-complete 완료 계약** | ✅ (spec §17.3) |
| calibration — persistence/capture/preview/factory-seed + offline 분석 흐름 | ✅ (capture 는 sim-image — 실 D405 미검증) |
| detector — `Detect Object` (mock backend, 투영 수학 단위검증) | ✅ (GDINO 실 모델 = 슬라이스 3, 집) |
| scene3d / scan (TSDF build 포함) / waypoint | ✅ |
| bridge (WS relay + MJPEG + HTTP + /contract.json + /contract/graph) + frontend contract gen | ✅ |
| **robot-agnostic 스코프 리팩터** (detector·calibration·scan·scene3d·waypoint → host당 1) | ✅ (2026-07-03 — 규칙은 spec §2.7, 아래 히스토리) |
| Task / Gamepad Module | 미착수 (task-first — spec §17) |

**검증 명령** (cwd 반드시 `backend_v2/`):
```bash
cd backend_v2
uv run --no-sync pytest -q                          # 212 passed
uv run --no-sync ruff check . && uv run --no-sync pyright
uv run --no-sync python -m apps.main --host mock    # 실 boot (:8000)
# frontend: cd frontend_v2 && pnpm vitest run && pnpm lint
# e2e: mock backend(CALIB_SIM_BOARD=1) + pnpm dev(:5174) 띄우고 pnpm test:e2e (headed)
```

## 아키텍처 불변식 (절대 어기지 말 것 — 포팅 시 [[feedback-port-keep-v2-arch]])

- **레이어링**: `modules/` 는 `apps/` import 금지. 다른 모듈 contract import 는 OK.
- **role 격리 (lazy registry)**: `apps/registry.py` = name→"path:Class" string lazy import,
  `apps/resolve.py` = branch 안 lazy import. eager import 금지 (test_boot subprocess 검증).
- **scope + robot_id 라우팅 = spec §2.7** — robot-scoped 4 (motor/camera/camera_decoded/
  motion) 외 전부 robot-agnostic. robot_id 는 키(주소) 또는 req 필드(파생 규칙) —
  Bridge 자동주입 금지. 새 모듈/서비스 추가 시 §2.7.1 3갈래부터.
- **raw↔rad = Motion 책임**. MotorDriver 는 순수 raw.
- **contract.py 컨벤션**: nested `Service`/`Stream`/`Event` StrEnum. stream/event payload
  에 `robot_id`+`seq`+`timestamp_unix` (spec §16.6). Stream key 는 채널 정의 모듈 contract 에.
- **Bridge = relay only** (spec §16.6) — domain logic 0.
- Motion = pi_motor 배치 (100Hz 명령 network 안 넘게). dof = arm only.
- **안전 수치 임의 금지**: limit=motors.yaml(실측), 속도=motion.yaml. 새 값 필요하면
  사용자에게 꺼내 보여줄 것, 추측 X.
- 테스트는 통과용 X — 실제 동작/invariant + spec ref docstring ([[feedback-meaningful-tests]], spec §15).

## 다음 작업 후보

1. **detector 슬라이스 3 — 실 GDINO backend** (`Detect Object` 의 구현체). 현재
   `apps/resolve.py::_detector_backend` 가 mock 만 배선 (real = NotImplementedError —
   그 메시지가 진입점).
   **2026-07-03 착수 — pyproject 까지만 완료 (uncommitted)**: `pc` 그룹에
   `transformers>=4.45,<5` + `accelerate` + `pillow` + `torch==2.11.0`
   (cu130 uv.sources/index, 옛 backend 동일 판). `uv sync` 아직 안 돌림.
   **transformers 상한 판단 (사용자 지적)**: 옛 backend 의 `<4.57` 핀 근거
   (meta tensor + `.to(device)` 깨짐) 는 **Qwen LLM 로드에서 관측**된 것 —
   GDINO 단독으론 분리 검증된 적 없고 v2 는 LLM 없음. 옛 결과를 근거로 인용하면
   거짓 권위 → `<5` 만 남김 (v5 의 `AutoModelForZeroShotObjectDetection` 제거는
   API 존재 문제라 확실). **smoke 때 최신 4.x 로 preload 검증 — 깨지면 그때
   실측 근거로 핀**. 남은 구현 순서:
   1. `modules/detector/gdino.py` 신규 — 옛 `backend/modules/detector/grounded_detector.py`
      포팅 (계약 = `detect(img, prompt) -> (bbox, score) | None`). **별도 파일** =
      torch/transformers import 를 mock 배치에서 격리 (motor/camera drivers 패턴 동형).
      load lock + transformers module-top import 유지.
   2. `backend.py` Protocol 에 `preload()` 추가 (Mock no-op).
   3. `module.py` — `start()` background preload (`asyncio.to_thread`) + `detect()` 의
      backend 호출도 `to_thread` (blocking 추론 → async 계약).
   4. `resolve.py::_detector_backend` real branch 배선 + `pc.yaml` 에 detector 추가.
   5. 테스트 (preload 배선 + real resolve 회귀) → `uv sync` → pytest/ruff/pyright →
      실 모델 로드 smoke (최신 4.x 에서 preload OK 확인 — 위 상한 판단의 검증 자리.
      깨지면 에러 실측 후 상한 핀 + 주석에 v2 측정 결과 기록).
   preload race 판단: 옛 race 는 LLM+GDINO **두** `from_pretrained` 동시 실행 전제 —
   v2 는 transformers 모델이 GDINO 하나라 전제가 구조적으로 없음. reproduction script
   ([llm_preload_race_debug.md](llm_preload_race_debug.md)) 는 두 번째 transformers
   소비자(LLM 포팅) 등장 시점의 프로토콜. 지금은 load lock + 단일 preload 경로로 보장.
   모델 로드/배선/mock 대비 회귀는 회사 가능, **검출 정확도는 집 하드웨어**.
   frontend 노출 필요 시 `FRONTEND_EXPOSED` 에 `Detector.Service.DETECT` 추가 + regen.
2. **PnP task (task-first — spec §17)** — ② 필요 primitive 정의 → task #1 을 async 함수 +
   디버거로. Day-1 primitive 중 MoveL·Detect Object(계약) 완료, 남은 것: Gripper 서비스 /
   VerifyGrasp / async runner+디버거 (spec §17.4) / detection Top-K+기하 prior (§17.5 —
   detector 슬라이스 3 과 자연 병행).
3. **Motion boot consumer** — Motion.start() 가 `snapshot_bundle` 읽어 kinematics build
   (link_offset patched URDF + joint/sag). calibration bundle wire 는 살아있음 — 미배선.
4. **offline BA 이월** — `calibrate_offline.py`(1722 LOC 5-stage BA) + `fk_chain.py` v2
   재배선 → 실 horibot.db run 으로 σ regression. capture→finalize 는 완성, BA 만 남음.
   (σ 0.818 재현 불가는 port 버그 아님 — 미기록 drop set, [[project-offline-ba-port-faithful]].)
5. **집 하드웨어 검증** — 아래 미검증 목록.

## 하드웨어 미검증 (집에서)

- `realsense_d405.py` (pipeline/align) — 아직 실 통신 안 해봄. 실 D405 intrinsic /
  ChArUco 캘 정확도 / scan TSDF 실물.
- feetech PID/profile write — motors.yaml `pid`/`profile` 가 실 모터 미적용 (driver 가
  EEPROM default 사용). 모션 느리거나 진동 시 wire (EEPROM write-once 주의).
- joint jog / cartesian MoveL 실물 / detector GDINO 실 모델 + preload race (reproduction
  script 먼저 — [[llm-preload-race]]).

## follow-up (blocking 아님)

- frontend framework store 의 agnostic 서비스 캐시가 robot 간 공유 (마지막 응답 wins) —
  패널이 robot 변경 시 refetch 라 기능 문제 아님. robot 별 캐시 분리는 실사용 시점에.
- omx `enabled: false` 라 mock fleet 투영 제외 — multi-robot **실부팅** 검증은 두 robot
  enabled 배포가 생기는 시점 (unit 층은 눈속임 방지 테스트가 커버).
- Playwright e2e CI 화 시 `CALIB_SIM_BOARD=1` backend 기동을 webServer 에 포함 (sim-board
  capture 테스트 skip 방지).
- latent (해당 step 진입 시): color+depth stream 페어링 (독립 seq) / Mirror refetch
  coalescing (consumer 등장 시) / Minio 예외·list semantics (Phase 3).

## 히스토리 (요지 — 상세는 git log)

- **2026-07-03 robot-agnostic 리팩터**: detector 구현 중 드리프트 발견 (설계 =
  robot-agnostic 인데 calibration 발 robot-scoped 가 복사 전파, 근거 없는 드리프트) →
  사용자 결정 "설계대로 되돌린다" → §2.7 라우팅 규칙 확정 (Bridge 자동주입 폐기 과정
  포함 — spec §2.7.3 폐기안) → calibration (최난도, 패턴 증명) → scan/scene3d/waypoint
  적용 → 전 층 검증 (위 표). mock 초기 자세 버그 fix (`MotorSpec.initial_raw` clamp —
  joint3 영점이 limit 밖) + so101 home/rest waypoint DB 삽입 동반.
- **2026-07-03 task-first 재정의**: "DSL 먼저" 폐기 → spec §17. 첫 task = 단팔 PnP.
  waypoint 모듈 (Robot Asset Layer 첫 자산) backend+frontend 완료.
- **2026-07-02 Calibration Step E 풀스택** + C2 (frontend 적응) TCP jog 실물 검증.
  CalibrationBundle = boot-time config 재분류 → Mirror consumer 0 (deferred).
- **2026-07-01 contract gen 파이프라인** (`/contract.json` EXPORT + gen-contract.mjs) +
  contract graph viewer (`/contract/graph` + React Flow).
- 상세 캘 도메인 결정 = [calibration_module_boundary.md](calibration_module_boundary.md),
  frontend = [frontend_v2.md](frontend_v2.md), framework 결정 history =
  [framework_dogfood_plan.md](framework_dogfood_plan.md).
