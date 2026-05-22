# Logging 설계

> 분산(PC + 모터 Pi + 카메라 Pi) 환경에서 사후 버그 디버깅을 위한 파일 로깅 설계 문서.
> "2시간 전 그 실행에서 뭐가 났더라"를 답할 수 있는 최소 시스템.

## 목적

**사후 버그 디버깅 (에러 추적)**에 한정. 명시적으로 범위 밖:

- 동작 분석(trajectory/캡처 등 도메인 이벤트 구조화 로깅) — 필요해지는 순간 별도 `EventLogger`로 분리.
- 장기 운영 모니터링 — 집 프로젝트라 해당 없음.

이 범위 한정이 중요한 이유: 한 시스템이 위 세 가지를 다 하려고 하면 포맷/레벨/보관 정책이 다 따로 놀게 됨. 디버깅만 집중하면 단순해짐.

## 왜 파일이 필요한가 — 콘솔만으론 부족한 케이스

평소 디버깅은 콘솔(`uv run python main.py` 출력)에서 라이브로 끝남. 파일이 실제 값을 하는 좁은 케이스:

1. **3대 동시 동작** — PC 콘솔 보는 동안 모터 Pi/카메라 Pi 에러는 못 봄. ssh 콘솔도 한 번에 하나만 봄.
2. **밤새 돌려놓은 게 죽었을 때** — 터미널 스크롤백 한계 넘으면 traceback 증발.
3. **"2시간 전 그 실행 다시 보고 싶다"** — 콘솔 스크롤백에서 잃어버리는 경우.
4. **터미널 자체가 닫혔을 때** — VS Code 재시작 등.

위 4개가 실재할 때만 파일 로깅이 값을 함. 본 프로젝트는 실제로 3대 돌리는 분산 토폴로지라 해당.

## 핵심 결정사항

| 항목 | 결정 | 근거 |
|------|------|------|
| 위치 | 머신별 로컬 `backend/logs/omx.log` | 네트워크 끊겨도 살아남는 ground truth |
| 회전 | `TimedRotatingFileHandler`, 자정 회전, 14일 보존 | "어제 그날" 찾기 쉬움 + 디스크 예측 가능 |
| 콘솔 레벨 | INFO+ | 평소 콘솔 깨끗하게 |
| 파일 레벨 | DEBUG+ | 스크롤백 대용이므로 단서 풍부하게 |
| 중앙 집계 | **안 함** | 단순성. 필요해지면 bridge에 5줄로 확장 가능 |
| 포맷 | 노드/머신 이름 박힌 텍스트 | grep으로 출처 추적 |
| 도메인 이벤트 로그 | **별도 시스템으로 미루기** | 디버깅 로그와 분석 로그를 섞지 않음 |

## 대안 비교

### 회전 정책 — 왜 날짜 기반?

| 방식 | 장점 | 단점 |
|------|------|------|
| 크기 기반 (`RotatingFileHandler`) | 디스크 예측 가능 | "어제 그 파일" 찾기 어려움 (인덱스만 있음) |
| **날짜 기반 (`TimedRotatingFileHandler`)** | "어제 = `omx.log.2026-05-22`" 직관적 | 활동량 따라 일별 크기 들쭉날쭉 |
| 실행 단위 파일 | 한 세션 = 한 파일, 가장 명확 | 본인이 가끔 청소 필요, 핸들러 직접 구성 |

집 프로젝트라 디스크 예측보다 **"어제 그 시점 찾기"** 편의가 우선 → 날짜 기반.

### 중앙 집계 — 왜 지금은 안 함?

`omx/system/log` Zenoh 토픽 + `BaseNode.log()`로 이미 모든 노드 로그가 한 토픽으로 흐름. bridge에 raw_subscriber 하나 붙여서 PC 디스크에 합쳐 쓰는 건 코드 5줄 수준.

그럼에도 처음엔 안 만드는 이유:

- 디버깅의 ground truth는 **로컬 파일이어야 함** — Zenoh가 끊긴 순간의 로그도 잡혀야 하니까. 중앙 파일은 항상 "best effort 사본".
- 두 곳을 동시에 신뢰하기 시작하면 어느 게 정답인지 헷갈리는 비용이 생김.
- 실제 답답한 순간(예: "3대 시간순으로 인과 추적해야겠다")이 와서 깔면 그때 명확한 동기를 갖고 만들 수 있음.

확장 시점이 오면 [backend/bridge/zenoh_bridge.py](../backend/bridge/zenoh_bridge.py)에 `SYSTEM_LOG` raw_subscriber → 별도 핸들러로 `backend/logs/omx-central.log` 쓰는 패턴.

### 레벨 분리 — 왜 콘솔과 파일 다르게?

같은 레벨로 통일하면 둘 중 하나가 항상 부적절:

- 둘 다 INFO+: 디버깅 시 단서 부족, DEBUG 켜려면 코드 만지거나 환경변수 설정 필요.
- 둘 다 DEBUG+: 콘솔이 평소에 노이즈로 가득 참.

콘솔(라이브 관찰)과 파일(사후 분석)이 다른 목적을 가지므로 다른 레벨 → 평소 콘솔은 깨끗, 버그 났을 때 파일은 풍부.

## 구현 메모

### 코드 들어갈 위치

1. **`backend/core/logging_setup.py`** (신규) — `setup_logging(host_name)` 함수 하나 export.
   - root logger에 두 핸들러 부착:
     - `StreamHandler` (stderr, INFO+)
     - `TimedRotatingFileHandler` (`backend/logs/omx.log`, `when='midnight'`, `backupCount=14`, DEBUG+)
   - 포맷: `%(asctime)s [%(levelname)s] %(name)s [<host>] %(message)s`
   - `backend/logs/` 디렉토리 없으면 생성.

2. **`backend/main.py`** — host config 로드 직후 한 줄: `setup_logging(host_name)`.

3. **`.gitignore`** — `backend/logs/` 추가.

### BaseNode와의 관계

`BaseNode.log()`는 `omx/system/log` Zenoh 토픽으로 publish하는 별도 경로. 본 변경과 무관 — 그대로 둠. 파일 로깅은 모듈에서 `logger = logging.getLogger(__name__)`로 찍는 모든 호출을 자동 캡처.

만약 `BaseNode.log()`로 찍은 것도 파일에 남기고 싶다면, `BaseNode.log()` 안에서 `logging.getLogger("omx.node").log(level, msg)`를 같이 호출하면 됨 (현재는 미적용 — 필요성 보고 결정).

### 로그 라인 예시

```
2026-05-22 14:30:12,345 [INFO] omx.nodes.motion_node [pc] MoveJ 시작: target=[0, -0.5, 0.5, 0, 0]
2026-05-22 14:30:12,567 [DEBUG] omx.modules.kinematics.solver [pc] IK 수렴 8 iter
2026-05-22 14:30:13,890 [ERROR] omx.nodes.motor_node [pi_motor] Dynamixel ID=3 응답 없음
```

`[pc]` / `[pi_motor]` / `[pi_camera]`로 출처 머신 명시 → 나중에 3대 파일 모아 봐도 헷갈리지 않음.

## 미포함 / 추후 검토

| 항목 | 언제 다시 볼까 |
|------|---------------|
| 중앙 집계 파일 (`omx-central.log`) | 3대 시간순 인과 추적이 실제로 답답해질 때 |
| 도메인 이벤트 로그 (JSONL) | "왜 이 캡처는 TSDF 품질이 나쁘지" 같은 사후 분석이 필요해질 때 |
| 로그 검색 UI (프론트엔드) | `SYSTEM_LOG` 토픽은 이미 bridge에서 BOUNDED_FIFO(128)로 라이브 표시 중 — 파일 기반 검색이 필요해지면 추가 |
| structlog/loguru 전환 | 표준 `logging`으로 모자라다 싶을 때 |
