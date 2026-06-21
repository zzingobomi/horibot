# Testing Strategy — 실 hardware 가기 전에 최대한 버그 잡기

> 사용자 expectation (2026-06-21): "다른 기능 추가할 때도 테스트 빡세게 돌려서
> 내가 실 hardware 검증 전에 버그는 많이 잡아내고 싶다."

본 문서 = *기능 추가 / 코드 변경 후 어떤 검증을 자동으로 돌려야 사용자가 실
hardware 들고 검증할 때 burn cycle 이 안 나오는지* 의 SSOT. 변경 자체가 작아도
**전체 stack** (backend lint + unit + cross-process + bridge WS leg) 다 통과해야
사용자에게 "검증해보세요" 라고 넘김.

## 1. 4 계층 검증 (실 hardware 직전 까지 자동)

| Layer | 도구 | 자동 가능? | 본 PR 의 예시 (MOTION_STATE_TCP fix) |
|---|---|---|---|
| L1 Lint / Type | `uv run ruff check .` / `uv run pyright` / `pnpm lint` / `tsc -b` | ✅ 항상 | clean |
| L2 Unit | `uv run pytest tests/` | ✅ 항상 | 103/103 + 새 회귀 차단 2/2 |
| L3 단일 process e2e | host_mock 부팅 + WS 클라이언트 verify 스크립트 | ✅ ~30s | bridge WS 499 msgs / 5s |
| L4 Cross-process | host_pc_sim + host_pi_motor_sim 두 프로세스 + 외부 Zenoh peer verify | ✅ ~30s | 98 msgs cross-process / 5s |
| L5 실 hardware | 사용자 토크오프 + 작업대 검증 | ❌ 사용자 | ← 본 단계 *전* 까지 다 잡기 |

**L1–L4 다 통과한 후에만** "사용자 실 hardware 검증해주세요" 라고 넘긴다. 한 layer
라도 떨어지면 그 layer 안에서 root cause 잡고 다음 layer 진행.

## 2. Layer 별 *언제* 실행

### L1 Lint / Type — *내 변경 파일 한정*

```powershell
# Backend (내가 건드린 파일만 — 다른 곳 stale lint error 노이즈 제거)
uv run ruff check <touched files>
uv run pyright <touched files>

# Frontend
cd frontend; pnpm lint; npx tsc -b
```

전체 ruff/pyright 가 깨끗하면 다행이지만, 내 변경과 무관한 기존 에러가 있을 수
있음. 내가 건드린 파일만 강조해서 reporting.

### L2 Unit — *항상 전체*

```powershell
cd backend; uv run pytest tests/ -v
```

새 기능 / 변경마다 **회귀 차단 합성 테스트** 한 개 이상 추가 — [§3 합성 데이터
회귀 차단 패턴](#3-합성-데이터-회귀-차단-테스트-패턴) 참조.

### L3 단일 process e2e — *wire (topic/service) 추가 / 변경 시*

`host_mock` 으로 bridge 까지 단일 process 부팅 → 외부 WS 클라이언트가 새 wire 를
받는지/보낼 수 있는지 검증. [§4 verify 스크립트 템플릿](#4-verify-스크립트-템플릿)
참조.

### L4 Cross-process — *wire 추가 / Coordinates / 캐시 / cross-process state 자리*

`host_pc_sim` + `host_pi_motor_sim` 두 프로세스 + 외부 Zenoh peer 가 메시지 받는지.

cross-process 자리 안 잡히는 버그 종류 ([[feedback_mock_doesnt_verify_distributed]]
memory):
- process-local factory 객체에 mutation 하는 자리 (한 process 만 알고 다른 process 모름)
- 캐시 invalidation (한 쪽 push, 다른 쪽 pull)
- Singleton 이 process 마다 별개라 sync 안 됨
- Topic key expression placeholder expand 누락

이런 자리는 L3 단일 process 자리에서 안 보임. **L4 필수**.

### L5 실 hardware — 사용자

L1-L4 다 통과한 후. 사용자에게 *어떤 자세* / *어떤 토글* / *어떤 결과 기대* 명시
해서 burn cycle 최소화.

## 3. 합성 데이터 회귀 차단 테스트 패턴

본 PR 의 [test_tcp_state_publish.py](../backend/tests/test_tcp_state_publish.py)
가 reference. 패턴:

1. **bug 의 mechanism 을 코드로 재현** — 합성 데이터 만들기 (예: 합성 horizontal
   plane + 합성 robot pose). 실 hardware 안 필요.
2. **fix 전 자리 (naive)** 와 **fix 후 자리 (corrected)** 의 *측정 가능한 metric*
   을 둘 다 계산 (예: tilt angle).
3. **두 metric 의 차이가 의미있는 값** 임을 assert (`naive > 0.5°`, `corrected < 0.001°`).
4. 누군가 fix 를 회귀시키면 (naive 자리로 돌리면) 테스트가 자동 깨짐.

```python
def test_<bug_mechanism>_blocked():
    # 1. 합성 데이터
    angles = synth_pose()
    # 2. fix 전 / 후 양쪽 계산
    metric_naive     = compute_with_naive_chain(angles)
    metric_corrected = compute_with_corrected_chain(angles)
    # 3. assert 의미 있는 차이
    assert metric_corrected < threshold_good
    assert metric_naive     > threshold_bad
```

bug 가 실 캘 / 실 motor state 의존이면, 본 PR 처럼 SQLite 의 active 캘 row 를
fixture 로 사용하고 row 없으면 `pytest.skip` — CI fresh checkout 에선 자동 skip.

## 4. Verify 스크립트 템플릿

### L3 host_mock + WS

```python
# /tmp/verify_<feature>_ws.py
import asyncio, json, os, subprocess, sys, time
from pathlib import Path
import websockets

BACKEND = Path("D:/Study/horibot/backend")
WS_URL = "ws://localhost:8000/ws"
TOPIC_KEY = "horibot/<robot_id>/<your_topic>"

p = subprocess.Popen(
    ["uv", "run", "python", "main.py", "--host", "mock"],
    cwd=str(BACKEND),
    env={**os.environ, "PYTHONIOENCODING": "utf-8"},
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
)
try:
    time.sleep(10)  # boot
    async def run():
        async with websockets.connect(WS_URL) as ws:
            await ws.send(json.dumps({"type": "subscribe", "topic": TOPIC_KEY}))
            received = []
            deadline = time.time() + 5.0
            while time.time() < deadline:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                    d = json.loads(msg)
                    if d.get("type") == "topic_data" and d.get("topic") == TOPIC_KEY:
                        received.append(d["data"])
                except asyncio.TimeoutError:
                    break
            return received
    received = asyncio.run(run())
    assert len(received) > 0, f"FAIL — no messages on {TOPIC_KEY}"
    # schema 검증 추가
    print(f"PASS — {len(received)} msgs / 5s")
finally:
    p.terminate()
```

실행:
```powershell
$env:PYTHONIOENCODING="utf-8"
uv --project backend run --with websockets python verify_<feature>_ws.py
```

### L4 host_pc_sim + host_pi_motor_sim cross-process

```python
# /tmp/verify_<feature>_distributed.py
# 동일 패턴, 차이점:
# - 두 process subprocess.Popen (pc_sim + pi_motor_sim)
# - WS 대신 직접 Zenoh peer 로 subscribe (bridge 우회, 진짜 cross-process)
import zenoh
cfg = zenoh.Config(); cfg.insert_json5("mode", '"peer"')
session = zenoh.open(cfg)
received = []
sub = session.declare_subscriber(TOPIC_KEY, lambda s: received.append(json.loads(bytes(s.payload).decode())))
time.sleep(5.0)
sub.undeclare(); session.close()
```

## 5. 흔한 verify 스크립트 자체 함정

본 PR 검증 자리 직접 마주친 자리들:

| 함정 | 증상 | 해결 |
|---|---|---|
| Windows cp949 + 한글 / em-dash | `UnicodeEncodeError: 'cp949' codec ...` | `env["PYTHONIOENCODING"]="utf-8"` + verify 스크립트는 ASCII-only 권장 |
| WS message field `action` vs `type` | `received 0 msgs` (조용히) | bridge `_handle_message` 의 `msg_type` 자리 확인 — 컨벤션은 `"type"` |
| pyrealsense2 import 자리 분산 Pi 에서 import fail | `--no-install-package pyrealsense2` 후 별도 .whl install | [pyrealsense2-build-guide.md](pyrealsense2-build-guide.md) |
| zenoh peer discovery hang | 무한 대기 | localhost multicast loopback 안 되는 환경은 `connect: ["tcp/127.0.0.1:7447"]` 명시 |

## 5.1 ⚠️ 분산 transport 의 부수효과 — **pytest / sim 도 실 robot 으로 broadcast 됨**

**Zenoh multicast scout 의 default 동작**: 같은 LAN 의 같은 process 자체 자리 *peer 자동 발견* + topic broadcast. 즉:

- `pytest tests/test_motion_e2e.py` 의 `MOTION_MOVE_J` / `MOTOR_CMD_JOINT` publish → **같은 LAN 에 떠있는 실 robot pi backend 가 receive → 실 motor 움직임**
- `host_mock` / `host_pc_sim` / `host_pi_motor_sim` 도 동일 — mock_motor 의 publish 와 실 motor_node 의 publish 가 같은 topic 자리 충돌

**예방 (테스트 시작 전 mandatory)**:

1. 사용자에게 사전 확인 — "실 robot backend 떠있나요?" 떠있으면:
   - 테스트 전 robot pi backend stop (Ctrl-C)
   - 또는 **robot torque-off** 자리 motor 명령이 와도 안 움직이게 (encoder 만 동작)
2. 또는 zenoh `multicast_scouting: false` + 명시적 `connect: []` host config 자리 격리 (단 hosts 끼리 자동 발견 X — 별도 설정 부담)
3. 가장 안전 자리: **테스트 머신 자리 robot pi 와 다른 subnet** 또는 **VLAN 격리**

**자동 검증 스크립트 / pytest 자리도 동일 위험.** 사용자가 robot 띄워있는 상태에선 *어떤 자동 motion publish 도 위험*. 사용자 측 확인 없이 motion-related test 자리 돌리면 안 됨.

[[feedback-distributed-broadcast-affects-real-robot]] memory anchor.

## 6. 새 wire 추가 시 체크리스트

새 topic / service 추가 PR 자리:

- [ ] `backend/core/transport/topic_map.py` 에 키 추가 (`{robot_id}` placeholder convention)
- [ ] `backend/core/transport/messages/<domain>.py` 에 payload schema (`StrictModel`)
- [ ] `backend/api_contract.py` 의 `PUBLIC_TOPICS` / `PUBLIC_SERVICES` 등재
  (등재 안 하면 frontend 자동 못 받음)
- [ ] Backend publisher / subscriber wire 자리
- [ ] **합성 회귀 차단 test** — bug mechanism 재현
- [ ] **L3 host_mock + WS verify 스크립트** — 외부 클라이언트가 받는지
- [ ] **L4 cross-process verify 스크립트** — multi-process 에서 흐르는지
- [ ] `pnpm gen:types` — `contract.ts` + `types.ts` 자동 갱신
- [ ] Frontend consumer 자리 (`useTopic` / `useService` / `callService`) 갱신
- [ ] `pnpm build` 성공
- [ ] CLAUDE.md / docs 자리 anchor 갱신 (다음 세션 인지)

## 7. 사용자 → 다음 세션 전달 사항

본 strategy 가 적용 안 되는 자리 (= 사용자가 실 hardware 검증 단계로 직접 가야
의미 있는 자리):

- **실 motor backlash / sag 의 *물리적* 효과** — 합성으로는 회로 검증만, 실값은 hardware
- **실 D405 의 depth noise / 광량 의존 dropout** — 합성 depth_frame 으로 안 잡힘
- **실 케이블 / 전원 / USB 대역폭 경합** — host_pc_sim 자리 아님
- **UX 의 "이상함"** — 사선이 4°냐 30°냐는 합성으로 정량 측정 가능, 그러나 *왜
  이상한지* 의 perception 은 사용자만

이런 자리는 사용자에게 *명확한 절차 + 기대 결과* 제공:
> 예: "토크오프 + 책상 면 보는 자세로 PC 토글 → PC 면이 책상 수평이면 fix 완료.
>  4° 이상 사선이면 hand_eye observability 가설로 진행."

## 관련 문서

- [slice_abc_verify.md](slice_abc_verify.md) — Phase 2 dev 서버 검증 순차 가이드
- [scan_pipeline_readiness.md](scan_pipeline_readiness.md) — 본 패턴의 다른 적용 예시
- CLAUDE.md `feedback_mock_doesnt_verify_distributed` memory anchor
