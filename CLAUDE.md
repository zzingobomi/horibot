# CLAUDE.md

이 파일은 Claude Code(claude.ai/code)가 이 저장소에서 작업할 때 참고하는 가이드입니다.

## 프로젝트 개요

Horibot — **multi-robot 팔 제어 스택**. 현재 robot 2대 ([robot/robots.yaml](robot/robots.yaml) SSOT):

| robot | 사양 | capability |
| --- | --- | --- |
| `so101_6dof_0` | SO-101 6DOF, Feetech STS3215, RealSense D405 (wrist mount) | move, calibrate, gamepad, rgbd |
| `omx_f_0` | OMX_F (OpenMANIPULATOR-X 커스텀 변형), Dynamixel, USB 웹캠 | move, calibrate |

backend = **module 기반 framework** (Zenoh + msgpack — "같은 코드가 어디 배치되든 그대로 동작", 분산은 deployment yaml 만 다름). frontend = React + react-three-fiber 3D 워크스페이스 + dockview 플로팅 패널.

**아키텍처 SSOT = [docs/backend.md](docs/backend.md) (진행 status/세션 handoff 포함 — 문서 상단) + [docs/frontend.md](docs/frontend.md).** 본 파일은 명령어 / 구조 지도 / 규약 / 문서 인덱스만 — 아키텍처 상세를 여기 중복하지 않는다 (중복 서술이 코드 진화를 못 따라가 통째로 썩었던 옛 CLAUDE.md 의 교훈 — 2026-07-11 전수 감사로 재작성).

## 자주 쓰는 명령어

### Backend (Python 3.11, uv 관리, [backend/](backend/)에서 실행)

```powershell
cd backend
uv sync                                            # PC 개발 환경
uv run --no-sync python -m apps.main --host mock   # 단일 머신, HW 없이 (bridge :8000)
uv run --no-sync python -m apps.main --host pc     # 분산 PC 측 (실 HW)
uv run ruff check .
uv run pyright
uv run --no-sync pytest -m "not sim" -q            # fast loop (수 초)
uv run --no-sync pytest -q                         # full (~90s, sim marker 포함)
```

Pi 배포 (각 deployment yaml 상단 주석이 실행법 SSOT):

```bash
# pi_hori1 = so101 motor+motion / pi_hori2 = so101 D405 camera / pi_hori3 = omx motor+motion+camera
uv sync --no-default-groups --group pi-hori1
uv run --no-sync python -m apps.main --host pi_hori1
# pi_hori2 는 pyrealsense2 소스빌드 wheel 별도 설치 (docs/hardware.md)
```

배포 구성 = [backend/config/deployments/](backend/config/deployments/) — `mock` / `pc` / `pi_hori1` / `pi_hori2` / `pi_hori3`. 각 yaml 이 그 host 의 modules 목록 + `driver_mode`(real/mock) + rdb/object URI 를 선언.

> ⚠️ **검증/실행용으로 띄운 backend 는 그 세션 안에서 반드시 kill** — 유령 `apps.main` 이 :8000 을 점유하면 full-boot pytest 가 조용히 hang ([docs/backend.md](docs/backend.md) 사고 기록).

### Frontend (pnpm, [frontend/](frontend/)에서 실행)

```powershell
cd frontend
pnpm install
pnpm dev          # vite :5173 (bridge 가 CORS 허용)
pnpm build        # tsc -b && vite build
pnpm lint
pnpm vitest run   # unit
pnpm test:e2e     # Playwright headed (mock backend :8000 + pnpm dev :5173 선행 필요)
pnpm gen:types    # 떠 있는 backend /contract.json → src/api/generated/contract.ts
```

## 아키텍처 지도 (개요 — 상세·결정 근거는 docs/backend.md, docs/frontend.md)

### backend — framework 3층 + Module

- **Contract**: 각 모듈의 `modules/<name>/contract.py` 가 그 모듈 wire 의 SSOT — pydantic `StrictModel` + `@service` / `@subscriber` / `@publishes`. 키 = `srv|stream|event/<module>[/{robot_id}]/<name>`.
- **Runtime**: [apps/main.py](backend/apps/main.py) 가 deployment yaml 의 modules 를 [apps/registry.py](backend/apps/registry.py) (lazy import — Pi 가 PC 전용 dep 안 끌고 옴) 로 로드 → 데코레이터 자동 discovery → msgspec 인코딩 + 디스패치. **Mirror** (snapshot + invalidate + liveliness — owner 가 늦게 떠도 자동 수렴) 지원.
- **Transport**: [infra/transport/zenoh.py](backend/infra/transport/zenoh.py) — bytes 전용 (직렬화는 runtime 책임). 같은 LAN Zenoh peer multicast 자동 발견.
- **에러 전파**: 응답 봉투 없음 — backend 예외가 `RemoteError(type, message)` 로 wire 를 건너고, bridge 가 WS error frame 으로 릴레이. (frontend `bridge.ts` shim 이 `{success, message, data}` 모양을 **클라이언트에서** 복원 — wire 규약이 아님.)
- **robot 스코프 규칙 (backend.md §2.7)**: robot-scoped 는 `motor` / `camera` / `camera_decoded` / `motion` 4개뿐 (`{robot_id}` 키 placeholder). 나머지는 host 당 1 인스턴스 (robot-agnostic) — 대상 robot 은 req 필드 `robot_id` 또는 진행 자원 id(run_id 등)에서 파생. Bridge 자동주입 금지.

### Module 12개 ([backend/modules/](backend/modules/))

| module | 역할 |
| --- | --- |
| motor | Dynamixel/Feetech/mock driver + 20Hz raw state |
| camera | RealSense/UVC/mock — color JPEG + zstd depth |
| camera_decoded | JPEG/depth 디코드 dedup (derived read model) |
| motion | MoveJ/MoveL/Jog + PyBullet IK + TCP state (calibration Mirror 첫 consumer) |
| calibration | 5종 캘 산출물 owner — capture 세션 + DB + factory intrinsic seed |
| scene3d | RGBD primitive — 라이브 pointcloud + N-frame consensus snapshot |
| scan | scan 세션/캡처 + ICP/TSDF build + 영속 (Open3D) |
| waypoint | waypoint/group CRUD + teach |
| detector | prompt → base-frame 3D 후보 (GDINO/SAM2/mock driver) |
| llm | 자연어 → pick/place 구조화 (Qwen/mock) |
| tasks/pick_and_place | Pick&Place **task 모듈** (표준형 레퍼런스 — 검출→도달성 선별→파지→적치). 감독은 [modules/tasks/core/](backend/modules/tasks/core/) 부품 상자 (TaskRunner=wire 무지 감독기/TaskContext/@step — 모듈 아닌 라이브러리, [docs/task.md](docs/task.md)) |
| bridge | FastAPI — WS 릴레이 + `/contract.json` + `/robots` + `/dev` 콘솔 + MJPEG |

task 터미널 실행 (frontend 없이): `uv run --no-sync python scripts/run_task.py srv/pick_and_place/run --param "pick_object=white cube"` (트리거 키 직접 — mock in-process 부팅, :8000 미점유).

### bridge wire (browser ↔ backend)

browser→bridge = JSON text (`subscribe`/`unsubscribe`/`publish`/`service`). bridge→browser = **binary frame** `[u8 ver=1][u8 type][u16 BE key_len][key utf8][msgpack payload]` — type 1=topic / 2=service 응답 / 3=service 에러. 클라이언트별 송신 큐: `stream/*` latest-wins(1), `event/*` FIFO(128).

### contract 생성 (frontend 타입)

노출 SSOT = [apps/contract_export.py](backend/apps/contract_export.py) `FRONTEND_EXPOSED` 한 곳 → bridge `GET /contract.json` → `pnpm gen:types` → [src/api/generated/contract.ts](frontend/src/api/generated/contract.ts). **regen invariant**: [src/api/__fixtures__/contract.json](frontend/src/api/__fixtures__/contract.json) 이 contract.ts 와 쌍 — 계약 변경 시 둘 다 재생성 (vitest 가 byte-identical 검증). 분산 배치에서의 gen 재논의 금지 — [docs/backend.md](docs/backend.md) 에 기각 근거 확정.

### frontend

- **framework hooks 6종** ([src/framework/](frontend/src/framework/)): `useService` / `useTopic` / `useStream`(seq/lag invariant) / `useMirror`(snapshot+invalidate) / `useCapability` / `useResource`. ([docs/frontend.md](docs/frontend.md) §3)
- **서비스 응답 캐시 키 = `bridge.serviceCacheKey(key, robotId)`** — wire 라우팅 키(expand)와 분리된 전역 규약. robot-agnostic 서비스를 wire 키로 캐시하면 robotA 응답이 robotB 뷰에 새는 실사고 (2026-07-11).
- **패널** = dockview + [components/panels/registry.ts](frontend/src/components/panels/registry.ts). **PANEL_CATALOG 가 title/size/scenePart 의 SSOT** — mode 파일 PANELS 는 배치 선언(`{id, component}`)만, override 는 예외 자리만 명시.
- **robot ownership**: robot 은 패널이 소유 ([docs/frontend.md](docs/frontend.md)). **per-robot 상태(frustum/live 토글 등)는 `Record<robotId, ...>`** — 전역 bool 은 두 번째 robot 에서 오발사 (실사고 2건).
- **씬 소유권**: Scene object(세계 — Robot/Camera/ScanMesh, 자기가 자기를 그림) vs scenePart(패널 수명 오버레이) — [docs/frontend.md](docs/frontend.md). 판별 질문 = "패널 닫으면 사라져야 하나?"
- **라우트**: `/` Dashboard, `/robots/:id/{move|calibrate|scan|assets}`, `/tasks/pick_and_place` (task 별 전용 페이지 — robot 바인딩/표시 문구는 페이지 소유 상수), `/contract` (계약 그래프 뷰어).

### robot/ (registry — type/instance 분리)

[robot/robots.yaml](robot/robots.yaml) = lean registry (identity / capability / vendor backend). type 폴더 `robot/<type>/` (motors/motion/physical yaml + URDF), instance 폴더 `robot/instances/<id>/` (port 등 개체 설정). **모든 robot type 의 URDF 는 `tcp` 이름 link 필수** — backend ([modules/motion/fk_chain.py](backend/modules/motion/fk_chain.py) 계열) 가 `"tcp"` 하드코드, 없으면 부팅 fail-fast.

### calibration 흐름 (요약 — 상세는 docs/calibration.md)

capture-only 세션: `start_run(kind)` (robot 당 활성 세션 1개 — stale in_progress 자동 정리) → `capture` 반복 (PnP gate + preview 신호등) → `finalize` (intrinsic = 즉시 계산·활성 / hand_eye = ready_for_analysis) / `abort` (중도 포기 탈출구). offline 분석 = [backend/scripts/calibrate_offline.py](backend/scripts/calibrate_offline.py) (5-stage BA + LOOCV, `--commit` 으로 DB activate — backend 종료 후 실행). 영속 = DB `calibration_runs/captures/results` ([backend/horibot.db](backend/horibot.db), **git tracked** — 머신 간 캘 동기화), 롤백 = 과거 result `activate`. D405 robot 은 부팅 시 factory intrinsic 자동 seed, USB 웹캠(omx)은 UI 수동 내부캘.

캘 정확도 도달치: OMX σ_rot 0.65° / σ_t 7.94mm ([docs/calibration.md](docs/calibration.md)), SO-101+D405 effective σ_R 0.801° / σ_t 7.53mm = algorithmic floor ([docs/calibration.md](docs/calibration.md) — **다음 캘 trauma 시 최우선 anchor, 동일 옵션 재검토 금지**).

## 문서 인덱스 ([docs/](docs/) — 8편)

> 2026-07-11 전수 감사 + 다이어트: 61편 → 8편. 도메인별 통합본 — 각 문서 안에 옛 문서들이 원문 그대로 부(part)로 병합돼 있음 (앵커 무손실, 통합 목록은 각 문서 상단 배너). 삭제분은 git history 복원 가능. 신규 문서는 만들지 말고 **해당 도메인 문서에 § 추가**가 기본.

| 문서 | 내용 (통합된 옛 문서) |
| --- | --- |
| [backend.md](docs/backend.md) | **backend 아키텍처 SSOT + 진행 status/세션 handoff (문서 상단 — 세션 handoff 는 항상 이 부를 갱신)** — framework spec §1-14 + Module catalog §16 + Task-first §17. 부록: 모듈 간 호출 규약(async_call_contract) / contract gen 분산 기각 근거 / framework 도입 history(dogfood_plan §15 reframe) / pick·grasp handoff(열린 버그). **새 세션 진입점 — 박힌 결정 의심하지 말고 §14 anchor 확인.** |
| [frontend.md](docs/frontend.md) | **frontend SSOT** — hooks/패널/contract gen. 부록: 씬 시각화 소유권(scene_contribution — scenePart/기각 목록) / robot 소유권 불변식 / 워크스페이스 autohide 헤더. "scenePart" / "robot 셀렉터" / "패널에서 3D" 톤 전부 여기. |
| [hardware.md](docs/hardware.md) | **HW + 운영** — 머신 토폴로지(pi_hori1/2/3 + IP)/실행 명령/OMX 모터·전원/SO-101 6DOF 개조 기록(STS3250·기어비·Feetech provisioning)/카메라/작업대. 부록: pyrealsense2 Pi 소스빌드 가이드. |
| [calibration.md](docs/calibration.md) | **캘 전부** — 모듈 boundary spec(코드 역참조 정본) + 캡처 절차/보드 spec + **σ floor 진단(캘 trauma 최우선 앵커 — cv2_seed/MCMC/StageE/Kalib 전부 reject 확정)** + 확장 BA 도달기 + 정확도 짜내기 전략. |
| [task.md](docs/task.md) | **task 아키텍처 정본 (2026-07-12 확정)** — task=모듈 + tasks/core 부품, 새 task 체크리스트 §3 (+ 대체된 2026-07-08 안 / 폐기 DSL reference 원문). "task" / "시나리오" / "TaskRunner" 톤이면 여기. |
| [motion.md](docs/motion.md) | Move/Servo/Jog/Task 4계층 + 산업 매핑 + jog drift 진단 박제 + URDF visual↔FK mismatch (open). |
| [perception.md](docs/perception.md) | GDINO+SAM2 선택 근거 + multi-way ICP/TSDF 결정·파라미터(구현=modules/scan/build.py) + LLM preload race 진단. |
| [dev_reference.md](docs/dev_reference.md) | DB 스키마 + 4계층 검증 방법론(testing_strategy) + "이거 왜 이렇게 짰어?" 검토 protocol + 아이디어 버킷. |

## 규약

- 대화 응답은 한국어 존댓말. 로그 메시지 / 주석 / docstring 은 한국어 자유 — 주변 코드 스타일 유지.
- Backend: **ruff** (line-length 88, target py311) + **pyright**. Frontend: ESLint + Prettier (`editor.formatOnSave`).
- 프론트 import `@/` alias = [frontend/src/](frontend/src/).
- pytest marker `sim` = Runtime/PyBullet/URDF boot 필요 test — fast loop 은 `pytest -m "not sim"`.
- wire schema 이름은 **verb-first** (Google AIP 스타일 — `StartRunRequest`, `ListRunsResponse`), 정의 자리는 각 `modules/<module>/contract.py`.
- Persisted timestamp = **UTC-aware datetime** (ORM `DateTime(timezone=True)`, wire ISO 8601, 생성 `datetime.now(UTC)`). float epoch 는 wire envelope/elapsed 측정 등 boundary 자리만.
- gitignore: 모델 가중치(`*.pt`/`*.pth`), `.venv/`, `node_modules/`, `frontend/dist/`, blob 산출물(`backend/storage/`), per-instance 런타임 산출물. 단 [backend/horibot.db](backend/horibot.db) 는 **git tracked** (캘/waypoint 데이터의 머신 간 동기화 수단).

### 프로젝트 design decision (다른 PC / 새 세션이 알아야 할 critical context)

- **대원칙: 모든 작업은 UI / UX / DX 를 고려해야 완성** — 계약 구현 + 테스트 초록은 시작점일 뿐 (테스트는 "내가 짠 계약이 내가 짠 대로 도는지"만 봄, 계약 자체의 구멍은 못 잡음). 완료 보고 전 세 렌즈의 실행 체크: ① **UX — 모든 상태에서 나갈 수 있고, 실패해도 복구 가능한가**: 세션/run/task 류 상태머신은 중도 포기(abort) 경로 필수. 워크스루 = 시작→진행→정상종료→중도포기→**실패** — 각 단계마다 "여기서 실패하면?" (service 에러/timeout/WS 재연결/backend 재시작 중 세션 포함). 실패는 상태를 corrupt 하지 않고 재시도·탈출 가능한 상태로 남아야 (예: intrinsic finalize 실패 = 세션 유지 → 더 캡처 후 재시도). capability 가 다른 robot(omx=웹캠 vs so101=D405)으로도 굴려볼 것. ② **UI — 상태가 보이고 구분되는가**: 실패는 **사유 + 다음 행동**이 사용자에게 표시 ("실패"만 찍으면 반쪽). **침묵 fallback 금지** — 실패를 기본값으로 덮으면 조용한 오동작 (hand_eye snapshot 미도달 → identity fallback 사고 전례, useMirror "침묵 금지" 주석). 같은 화면의 다른 인스턴스(탭 title 등)와 구분. ③ **DX — 다음 개발자가 안전하게 확장하는가**: 토글/store/캐시 신설 시 "robot(인스턴스)별이어야 하나" 질문 — 전역에 두면 두 번째 인스턴스에서 오발사. 기본값은 SSOT 한 곳, override 는 예외 선언. 결함 발견 시 그 일반형(클래스)을 정의하고 같은 클래스를 codebase 전체 sweep, 회귀 테스트는 발견 시나리오 그대로. (기원: 2026-07-11 캘 패널 — intrinsic 0장 세션 갇힘(탈출구 부재) / omx [시야]가 so101 frustum 표시(전역 토글) — 둘 다 vitest 초록이었음)
- **Task 아키텍처 확정 (2026-07-12 골격 + 2026-07-13 전면 개정)** — **TaskRunner = wire 무지 범용 감독기** (실행/취소/pause/step 게이트/예외→실패 — runtime·키·robot·계약 무지, 변화는 생성자 콜백 on_state/on_trace 로 통지, 안 달면 headless). **모듈이 전부 소유**: 계약(트리거+노출 결정한 조작판+진행 스트림 — 전부 손 선언), 배선, 진행 발행(콜백에 자기 발행 메서드), 트리거(서비스/@subscriber/내부 호출 — start 는 아무나 호출), 시나리오. **등록 의식 0**: registry/@task/TaskMetadata/GET /tasks 전부 없음 — task 정보 채널은 계약뿐, robot 바인딩/표시 문구는 frontend 전용 페이지 상수. **호출 표면 = ctx.call 하나** (RobotHandle 없음): robot-scoped 키는 `robot_id=` (참여 명부 검증 — 선언 밖 robot 명령 즉시 에러 = STOP 커버리지 보장), agnostic 은 req 필드 (§2.7, agnostic 에 robot_id= 주면 fail-fast). step = 저자가 `@step(title="집기")` 로 지정한 함수 (중첩 = flat trace+depth, step_once = step-into; **병렬 gather = all-stop 의미론 지원** — 회귀 테스트 잠금). 실패: 기술적 실패는 **서비스가 raise** (accepted in-band 폐기), 부정 결과(검출 0/전멸 -1)는 데이터 — 판정은 step. timeout = contract `declare_service_timeouts`. step 간 데이터 = 시나리오 변수/반환값 (ctx.data 류 blackboard 금지). 새 task = pick_and_place 복제 — [docs/task.md](docs/task.md) §3. **STOP 은 안전 의무** — 모듈 stop→cancel + on_abort 참여 robot 전원 Motion.STOP.
- **self-play 는 폐기됨** — 본 방향은 pick_and_place + deterministic IK + 캘/자세 정확도 직접 강화. self-play 점프 제안 / 신규 기능 추가 금지.
- **Study task 에선 industry standard 도구/플로우 우선** — RL/실험/시뮬레이션 도구 추천 시 인프라 재사용 ROI 보다 산업 표준 (MuJoCo / Isaac / PPO / Stable-Baselines3 등) 우선. "표준 단계 다 밟아보기" 자체가 study output.
- **URDF TCP link 컨벤션** — 모든 robot type 의 URDF 는 `tcp` 이름 link 필수 (UR `tool0` 등가). 새 robot type 통합 시 wrist link 끝에 fixed joint child 로 추가 — 없으면 부팅 시 fail-fast.
- **frontend 서비스 캐시 정체성 = (key, robotId)** (`bridge.serviceCacheKey`) + **per-robot 상태는 `Record<robotId>`** — robot ownership 위반(전역 공유)이 실사고 3건의 공통 클래스 (2026-07-11).
- **Persisted timestamp = UTC-aware datetime** (규약 참조 — 세부는 [docs/dev_reference.md](docs/dev_reference.md)).
