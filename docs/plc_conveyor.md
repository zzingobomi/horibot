# PLC + 컨베이어 셀 (구축 가이드)

> 책상 위에 **컨베이어 → 포토센서 → PLC → Python → 로봇** 산업 자동화 셀을 재현해
> **PLC 연동 소프트웨어(Modbus TCP 핸드셰이크)**를 study 하기 위한 사이드 프로젝트.
> 목적 = PLC 래더 작성이 아니라 **PLC와 연동하는 상위 소프트웨어 + 실물 셀 경험**.
> (본 문서 = 구매/배선/구성 SSOT. 진행 상태·결정 근거는 memory `project_plc_conveyor_cell`.)

최종 업데이트: 2026-07-19

---

## 1. 전체 그림 — 신호가 지나가는 한 줄

```
물체 → [센서] → [DI보드] → [라즈베리파이] → [릴레이] → [컨베이어]
         눈      통역(입력)    뇌(OpenPLC)     손(스위치)    다리
                                   ↕  Modbus TCP
                            [Python 로봇 플랫폼] → 로봇 집기
```

- **센서** = "물체 왔다"를 24V 전기신호로 알림
- **DI보드** = 센서 24V 신호를 파이가 읽는 3.3V로 낮춤 (파이에 24V 직결 = 파손)
- **라즈베리파이 + OpenPLC** = PLC 두뇌. 센서 읽고 컨베이어 제어 + Modbus 서버
- **릴레이** = 파이의 약한 신호로 컨베이어 12V 전원을 on/off (전기 배달부)
- **24V SMPS** = 센서 전원(밥). 벽 220V → 24V
- **Python** = Modbus TCP 마스터. PLC와 "물체도착/집기완료" 핸드셰이크

---

## 2. 구매 현황

### 2.1 산 것 ✅ (2026-07-19, 합계 ~8만원)

| 품목 | 사양 | 비고 |
| --- | --- | --- |
| **미니 컨베이어** | 알루미늄 프레임 + PVC 녹색 연속 벨트, 폭 8cm × 전체 60cm(유효 안착 ~48cm), 12V DC, 단방향, 노브 속도조절 | **전원 = 12V 배럴잭(분리형)** → 무납땜 릴레이 삽입점 |
| **포토센서** | Autonics **BM200-DDT** — 확산반사형 200mm, 12–24V, **NPN 오픈컬렉터**, Light ON, 3선(갈/파/검), 브래킷 동봉 | 빨간 표시LED로 PLC 없이 12V만으로 동작확인 가능 |

### 2.2 나중에 살 것 (Phase 1, 실물 붙일 때) — 대략 가격대

> 전자부품은 **작고 싼 모듈들**. 파이 빼면 다 합쳐 2~3만원 수준. **책상 위 아님** — 선반/옆에.

| 품목 | 역할 | 대략 가격대 |
| --- | --- | --- |
| **라즈베리파이** (전용, 로봇 Pi와 별개) | PLC 두뇌 (OpenPLC) | 있으면 0 / 신규 ~7만원 |
| **24V SMPS** (24V 2~3A) | 센서 전원 | ~1~1.5만원 |
| **DI 옵토 보드** (24V→3.3V 절연) | 파이가 센서 읽기 | ~5천~1만원 |
| **릴레이 모듈** (1채널, 3.3/5V 구동) | 컨베이어 on/off | ~3천~1만원 |
| **듀폰 점퍼선** (암–암) | 파이↔보드 배선 | ~3천원 |
| **배럴 나사 어댑터** (수+암) | 12V 라인에 릴레이 삽입 | ~2~3천원 |
| **멀티탭** | 콘센트 3구 (SMPS/12V어댑터/파이) | ~1만원 |

**통합 대안 (더 깔끔):** DI보드 + 릴레이를 따로 사는 대신 **라즈베리파이 산업용 I/O HAT**(24V 절연 입력 + 릴레이 출력 통합) 하나로 → 살 것 2개 줄고 배선 단순. 가격 ~3~10만원(제품별). "실제 제어반"스러움을 원하면 이쪽.

**선택(산업 리얼리티):** 페룰 압착 키트 ~2만원 + DIN 레일/단자대 ~1~2만원. 납땜 대신 이게 진짜 공장 방식.

---

## 3. 배선 / 연결 (전부 무납땜)

> 연결은 3가지뿐: **나사 단자**(피복 벗겨 꽂고 조임) · **듀폰 점퍼**(핀에 꽂음) · **배럴 나사 어댑터**. 인두 없음.

### 3.1 센서 (BM200-DDT, 3선) — NPN

| 선 색 | 연결 |
| --- | --- |
| **갈색** | +24V (SMPS +V) |
| **파랑** | 0V (SMPS GND) |
| **검정** | 출력 신호 → DI보드 입력 채널 |

> ⚠️ **NPN 궁합**: NPN은 활성 시 신호선을 0V로 당김(sinking). DI보드/입력의 **커먼을 +24V**로 두는 NPN 배선이어야 함. DI보드가 **NPN 지원(또는 NPN/PNP 겸용)**인지 구매 시 확인 — 많은 유럽식 입력이 PNP 기본이라 이걸 놓치면 신호가 안 잡힘.

### 3.2 DI 보드 (24V→3.3V 옵토)

- **입력측**: 센서 검정선 + 24V/0V 기준
- **출력측(로직)**: VCC ← 파이 3.3V, GND ← 파이 GND, OUT → 파이 GPIO 입력핀 (듀폰)

### 3.3 라즈베리파이 (OpenPLC)

- GPIO 입력핀 ← DI보드 OUT
- GPIO 출력핀 → 릴레이 IN
- 3.3/5V + GND → 릴레이 VCC/GND

### 3.4 릴레이 모듈

- **제어측**: IN ← 파이 GPIO 출력, VCC ← 파이 5V, GND ← 파이 GND
- **부하측(스위치)**: 컨베이어 **12V 배럴잭 분리 지점**의 +선을 **COM–NO에 직렬 삽입**
  - 배럴 나사 어댑터(수+암)를 12V 라인에 끼우고, +12V를 릴레이 COM→NO로 경유
  - 릴레이 열림 = 컨베이어 정지, 닫힘 = 가동. **원선 안 자르고, 납땜 없음.**

```
12V 어댑터 ──[배럴]──┬── COM [릴레이] NO ──[배럴]── 컨베이어
                      │              ▲ IN
                 (GND 통과)      파이 GPIO (듀폰)
```

> **I/O HAT을 쓰면** 3.2~3.4가 HAT 단자 하나로 합쳐짐 (센서→HAT 입력단자, 컨베이어 12V→HAT 릴레이단자).

---

## 4. 전원 구성

| 전압 | 무엇 | 공급원 |
| --- | --- | --- |
| **24V** | 센서 + DI보드 입력측 | 24V SMPS (220V→24V) |
| **12V** | 컨베이어 모터 | 기존 12V 어댑터 (배럴잭) |
| **5V** | 라즈베리파이 | USB-C 충전기 |

- **벽 콘센트에 꽂히는 것 = 3개** (SMPS / 12V어댑터 / 파이충전기) → **멀티탭 하나**로 해결.
- **간단화**: BM200-DDT가 12–24V라, 센서를 12V로 돌리면 **24V SMPS 생략** 가능(12V 하나로 통일). 단 산업 표준은 24V — 리얼리티 원하면 24V 유지.
- **공통 GND**: 신호 기준을 위해 24V측·파이·릴레이의 GND는 서로 연결(공통 0V). (옵토 절연 DI보드는 입력측만 24V GND, 출력측은 파이 GND — 보드가 절연해줌.)

---

## 5. 소프트웨어 구성 (Phase 0 — 지금, 하드웨어 0)

컨베이어 배송 기다리는 동안 **부품 없이** PLC 공부 시작. PLC의 80%가 여기.

### 5.1 설치

1. **OpenPLC Editor** — 래더/ST 작성 (IEC 61131-3)
2. **OpenPLC Runtime** — 프로그램 실행 + Modbus TCP 서버(:502). PC(Windows/Linux)에서 실행 (Phase 1엔 라즈베리파이로 이전)

> ⚠️ **OpenPLC v4 현실 (v3와 다름 — 2026-07-23 실측, 다음 세션이 v3 감각으로 헤매지 말 것):**
> - **Editor 내장 "Start Simulator"** = 래더를 **AVR Arduino Mega(atmega2560) 펌웨어로 컴파일해 에뮬레이션** + 디버거로 변수 force/monitor. **Modbus RTU(시리얼)만, TCP :502 안 엶** — `Test-NetConnection 127.0.0.1 -Port 502` → **False 실측**. ⇒ Simulator = **래더 로직 검증 전용**(변수 force로 렁 동작 확인).
> - **standalone Runtime v4** = **headless(웹 UI 없음** — v3의 :8080 없음. REST API :8443 + WebSocket `/api/debug`). 플러그인으로 **Modbus slave/master + S7comm + OPC UA + EtherCAT** 탑재 = **멀티 프로토콜 대역 + 진짜 TCP :502**. Editor가 `Device→Configuration→Connect(127.0.0.1)→Transfer(↓)`로 프로그램 올려야 `libplc_*.so` 생성됨. 안 올리면 `No libplc_*.so / State transition to RUNNING failed`.
> - **결론: 래더 검증은 Simulator, Python/모듈 실측은 standalone Runtime(:502).** 현재 3렁(§5.2 로직) Simulator 검증 완료(SET 래치·리셋·재가동 확인).

### 5.2 최소 래더 (센서→컨베이어 + 핸드셰이크)

로직:
```
센서 입력(%IX0) = 1  →  컨베이어 출력(%QX0) = 0 (정지) + 코일 object_arrived = 1
Python이 pick_done = 1 씀  →  %QX0 = 1 (재가동) + 플래그 리셋
```

### 5.3 핸드셰이크 레지스터 맵 (Python ↔ PLC 계약 = SSOT)

> 정확한 Modbus 주소 매핑은 OpenPLC 슬레이브 설정에서 확인. 개념 맵:

| 이름 | 종류 | 방향 | 의미 |
| --- | --- | --- | --- |
| `sensor` | 입력 %IX0 | 센서→PLC | 물체 감지 (Phase 0엔 가상) |
| `conveyor_run` | 코일 %QX0 | PLC→릴레이 | 컨베이어 가동/정지 |
| `object_arrived` | 코일 | PLC→Python | 물체 도착, 집어라 |
| `pick_done` | 코일 | Python→PLC | 집기 완료, 재가동해라 |
| `heartbeat_*` | 레지스터 | 양방향 | 워치독 (양측 생존 확인) |

### 5.4 Python 측 (마스터)

- `pymodbus`로 Runtime(:502)에 접속 → 위 코일/레지스터 읽고·쓰기
- **5단계 루프 증명**: ①센서ON → ②PLC 정지+`object_arrived` → ③Python 읽고 집기(mock) → ④`pick_done` 쓰기 → ⑤PLC 재가동
- **워치독 추가**: 주기적 heartbeat 교환, 상대 끊기면 안전측(정지)

**Phase 0 완료 기준**: Python ↔ OpenPLC 핸드셰이크 루프가 돌고, 물체도착/집기완료가 Modbus로 오간다.

---

## 6. 셋업 순서 (체크리스트)

- [ ] **Phase 0** — OpenPLC 설치 → 래더 → Runtime(:502) → Python pymodbus 핸드셰이크 루프 + 워치독
- [ ] 컨베이어 도착 → 12V 물려 동작 확인 / 센서 12V 물려 **빨간 LED** 동작 확인 (PLC 전)
- [ ] **Phase 1** — 부품 구매(§2.2) → 배선(§3) → 전원(§4) → OpenPLC를 라즈베리파이로 이전
- [ ] 센서 마운트 (픽업지점 옆 프레임, 3D 프린트 클램프, Light ON, 감도볼륨 튜닝)
- [ ] 릴레이 12V 라인 삽입 → PLC가 컨베이어 start/stop
- [ ] 컨베이어 프레임 **클램프 고정** (60cm 좌우 걸침 → 모터 끝 전도 방지)
- [ ] 실물 5단계 루프 + 로봇(so101) 집기 연동

---

## 7. 잠근 결정 (다시 논의 X)

| 항목 | 결정 | 이유 |
| --- | --- | --- |
| 방향 | **단방향** | 전후방=수동 스위치라 PLC가 못 씀. 진짜 PLC 역방향은 H브릿지 몫 |
| 벨트 | **PVC 연속** | 비전 픽 = 평평·무광·연속면. 플라스틱 모듈러/목재 STEAM 키트 = 휨·요철=입력오염 |
| PLC 두뇌 | **라즈베리파이 + OpenPLC** | 아두이노 X — Modbus TCP(네트워크) 때문. ESP32 fiddly |
| 배선 | **무납땜** | 나사단자+듀폰. 산업식은 납땜 아닌 페룰+DIN 단자대 |
| 센서 | **확산반사·Light ON·NPN** | DI보드 NPN 호환 필수 |
| 배치 (책상 55×34) | **컨베이어 뒤 · 로봇 가운데 · place 앞 모서리 밖** | 로봇 base 회전으로 뒤=집기/앞=놓기, place는 depth 안 먹음 |

---

## 8. horibot 연결점 (미래)

Python 로봇 플랫폼이 **Modbus TCP 마스터**로 OpenPLC 폴링 → backend에 **`plc` 모듈**이 붙어 핸드셰이크 → **pick_and_place task** 트리거. (배포 진입점 = `apps.main --host pc`.) 상세 설계 = §9.

---

## 9. `plc` 모듈 설계 spec (구현 대기 — 2026-07-23 논의 확정)

> **이 §는 다른 세션의 구현 진입점.** 아래 결정은 §10에서 잠갔으니 재논의 말고 그대로 구현. 결정 근거는 Kepware(채널/디바이스/태그 + 드라이버=프로토콜→OPC UA 번역) · OPC-UA(Node = 타입 있는 Value(Variant) + StatusCode + Subscription) · Ignition(태그 DB + 드라이버별 주소 문자열 문법)을 조사해 정렬한 것.

### 9.0 목표 / 범위

- **여러 PLC 드라이버를 꽂는 framework.** OpenPLC(Modbus TCP) 전용이 아니라 장기적으로 Modbus / Siemens S7 / Allen-Bradley / OPC UA / Mock 을 같은 Protocol로 교체 가능하게.
- 진짜 study 목적 = **실제 산업용 PLC들과 통신하는 소프트웨어 모듈** (모형/OpenPLC는 이 모듈을 개발·검증할 **대역**).
- **지금 만들 드라이버 = `modbus_tcp` + `mock` 둘뿐.** 이 둘로 Protocol 모양을 검증하고, 두 번째 실드라이버(S7)가 올 때 Protocol을 보정. **s7/ab/opcua 파일 미리 만들지 X (죽은 scaffolding 금지).**
- `motor`/`camera` 모듈의 driver Protocol 패턴([modules/motor/drivers/protocol.py](../backend/modules/motor/drivers/protocol.py)) 그대로 재사용.

### 9.1 레이어 + 책임 경계

```text
Task  ──►  PLC Module  ──►  PlcBackend (Protocol)  ──►  Modbus / S7 / AB / OPC UA / Mock
```

| 계층 | 소유 | 프로토콜 |
| --- | --- | --- |
| **Task** | 공정 로직만 (`object_arrived면 pick`, `pick_done 쓰기`). **주소·프로토콜 모름** | 무관 |
| **Module** | 태그 DB(name→PointSpec), **스캔 정책(뭘/얼마나 자주)**, cache, Mirror 유지, 변경→stream, 재연결 정책, 연결상태→liveliness | 무관 |
| **Driver** | wire(연결/재연결 기계장치), **주소 파싱**, **요청 병합(coalescing)**, 인코딩/디코딩, **품질 판정** | 고유 |

> 핵심 문장: **Module은 "무엇을" 폴링할지 정하고, Driver는 "어떻게" 효율적으로 가져올지 정한다.** (SCADA클라 :: OPC서버+태그DB :: 디바이스드라이버 산업 매핑.)

### 9.2 데이터 모델

```python
DType = Literal["bool", "int16", "uint16", "int32", "float32_be", "float32_le_swap"]  # 필요시 확장

class Quality(Enum):        # 초기엔 3개만, 필요시 확장
    GOOD = "good"
    STALE = "stale"
    BAD = "bad"

@dataclass
class TagValue:             # 값 자체가 아니라 값+품질+시각 (OPC-UA Variant+StatusCode, Ignition quality)
    value: bool | int | float | str    # Generic[T] 쓰지 말 것 — 값은 config dtype에서 런타임 디코드라 정적 T가 안 흐름
    quality: Quality
    ts: datetime                       # 드라이버가 읽은 시각(read time). SourceTS/ServerTS 구분은 defer

@dataclass(frozen=True)
class PointSpec:
    address: str               # 드라이버별 문법 문자열 — "coil:1" / "DB1.DBX0.1" / "Motor.Run" / "ns=2;i=5"
    dtype: DType | None = None # Modbus/S7=필수(무타입 워드라 디코드 위해), AB/OPC-UA=생략(장비가 타입 자기기술)
```

### 9.3 Driver Protocol (`PlcBackend`)

```python
class PlcBackend(Protocol):    # motor/drivers/protocol.py 와 동형. Module SDK internal — TS gen/외부 import 대상 X
    def open(self) -> None: ...
    def close(self) -> None: ...
    def is_connected(self) -> bool: ...                                  # 연결상태 = per-tag quality와 별개 관측값
    def validate(self, points: list[PointSpec]) -> None: ...             # config 로드 시 주소 파싱/검증 → 오타 fail-fast
    def read(self, points: list[PointSpec]) -> dict[str, TagValue]: ...  # 배치가 primitive (key = 태그 이름)
    def write(self, point: PointSpec, value: bool | int | float | str) -> None: ...  # 단건. 실패 시 raise
```

> `read_coil()`/`write_coil()` 처럼 **Modbus 용어를 Protocol에 노출 금지** — 지멘스엔 coil 없음, Modbus가 계약에 새어들어감.

### 9.4 태그 바인딩 = 인스턴스 config (Task/Driver 아님)

의미 이름 → PointSpec 매핑을 **instance yaml**에 둔다. Task는 이름만, 드라이버가 주소를 해석:

```yaml
# robot/instances 대응 자리 or plc 인스턴스 config
tags:
  object_arrived: { address: "coil:1", dtype: bool }   # OpenPLC %QX0.1
  pick_done:      { address: "coil:2", dtype: bool }    # %QX0.2
  conveyor_run:   { address: "coil:0", dtype: bool }    # %QX0.0
  # sensor(%IX0.0)는 Discrete Input = 마스터 읽기전용. Phase 0엔 Editor에서 force
# Siemens로 바뀌면 이름 그대로, address만: { address: "DB1.DBX0.1", dtype: bool } → Task/contract 무변경
```

### 9.5 Module 상세 (framework 정합)

- **host-scoped 1 인스턴스** (robot-scoped 아님 — backend.md §2.7. robot-scoped 4개는 motor/camera/camera_decoded/motion 뿐). 대상 PLC는 req 필드/설정에서 파생.
- **`contract.py`** = wire SSOT. 노출: 태그 현재값(Mirror), 변경 이벤트(stream), write 서비스, 핸드셰이크(`object_arrived` 이벤트/`pick_done` write), 연결상태. StrictModel + `@service`/`@subscriber`/`@publishes`, verb-first 스키마명.
- **Mirror로 태그 상태 노출** — 폴링(Modbus)이냐 구독(OPC-UA)이냐를 드라이버 뒤로 숨기는 seam. 프론트는 기존 `useMirror`(snapshot+invalidate+**liveliness**)/`useStream` 그대로. **연결 끊김 = liveliness**로 표면화 (태그값과 무관하게 "PLC 끊김" 표시 — 실패는 사유+다음행동).
- **스캔 루프**: Module이 태그 집합을 매 주기 `driver.read(points)` → cache와 diff → 변경분 stream 발행. `driver.read`가 async 계약이면 blocking 소켓은 `asyncio.to_thread` 분리.
- **`driver_mode`(real/mock)** = deployment yaml 규약 그대로. mock 드라이버 = 인메모리 태그 딕셔너리(라이브 PLC 없이 pytest).
- 구현 시 **backend.md §16 모듈 카탈로그에 등록** + deployment yaml(pc/mock)에 modules 추가.
- **§8 트리거**: `object_arrived` True 감지 → pick_and_place task start 호출. STOP은 안전 의무(task 아키텍처 규약).

### 9.6 프로토콜별 라이브러리 + 타입 자기기술 여부

| 프로토콜 | Python 라이브러리 | 주소 예 | 타입 |
| --- | --- | --- | --- |
| Modbus TCP | `pymodbus` | `coil:1`, `hr:40001` | **무타입** → dtype 필수 (+ 멀티워드 엔디안) |
| Siemens S7 | `python-snap7` | `DB1.DBX0.1` | **무타입**(바이트) → dtype 필수 |
| Allen-Bradley | `pycomm3` | `Motor.Run` | **자기기술**(컨트롤러가 타입 반환) → dtype 무시 |
| OPC UA | `asyncua` | `ns=2;i=5` | **자기기술**(Node Variant) → dtype 무시. 구독(push) 지원 |

### 9.7 검증 대역 (현재 상태)

- **대역 = standalone OpenPLC Runtime v4** (`:502`, 로그에 `MODBUS_SLAVE` 확인). 멀티 프로토콜(S7/OPC-UA도)이라 드라이버 로드맵 전체를 이 한 대로 연습 가능. (Editor Simulator는 §5.1 노트대로 TCP 안 엶 → 래더 검증 전용.)
- 래더 = `plc/test_plc` 3렁, **Simulator 검증 완료**: 렁1 `sensor ∧ ¬pick_done → SET object_arrived` / 렁2 `pick_done → RESET object_arrived` / 렁3 `¬object_arrived → conveyor_run`. SET 래치 유지 확인(센서 OFF 후에도 object_arrived 유지).

---

## 10. `plc` 모듈 — 잠근 설계 결정 (다시 논의 X)

| 항목 | 결정 | 이유 / 기각안 |
| --- | --- | --- |
| 주소 표현 | **`address: str`** (드라이버가 파싱) | 드라이버별 Address 객체를 Protocol에 두면 `read()` 시그니처가 달라져 **치환 불가 → 교체성 목표 붕괴**. Kepware/Ignition도 주소=드라이버별 문자열. 검증은 `validate()` 부팅 훅으로 fail-fast |
| read 형태 | **배치 `read(points)`** 가 primitive | 연속 레지스터 병합(coalescing)은 **프로토콜 지식=드라이버 몫**. 단건+Module 루프는 왕복 폭증 or 프로토콜 누수. 단건은 `read([one])` |
| write 형태 | **단건 `write(point,value)` + 실패 raise** | 쓰기=부작용 명령, 원자성. 배치 write는 필요 시 확장(YAGNI). raise = 하우스 규약(RemoteError 전파) |
| 값 모델 | **`TagValue(value,quality,ts)` 초기 도입** | 품질은 산업통신 본질 — 실물 첫날 연결끊김 만남. quality 없이 "PLC 죽음"과 "값 false" 구분 불가 = **침묵 fallback 금지 위반** |
| Generic | **`TagValue`에 `Generic[T]` 쓰지 X** | value는 config dtype에서 런타임 디코드라 정적 T가 안 흐름 → 이득 없는 장식(cargo-cult) |
| dtype | **`DType \| None` (Optional)** | Modbus/S7=무타입이라 필수, AB/OPC-UA=장비가 타입 자기기술이라 무시. 필수로 박으면 후자에서 어색 |
| Modbus 멀티워드 | **엔디안을 dtype 문자열에 인코딩** (`float32_be` 등) | Modbus 워드순서 표준 없음(big/little/word-swap) = float footgun. 실물서 뒤집힌 값 디버깅보다 지금 박는 게 쌈 |
| 연결상태 | **`is_connected()` 독립 관측 → Mirror liveliness** | per-tag quality(BAD)와 다른 층위. "PLC 자체 끊김"을 태그값 무관하게 표면화 |
| Module 바깥 계약 | **변경알림(Mirror/stream)** | 폴링(Modbus) vs 구독(OPC-UA)을 드라이버 뒤로 숨기는 seam. horibot Mirror/Stream에 직접 매핑 |
| 태그↔주소 바인딩 | **Module의 instance config** | Task가 주소 알면 brittle-integration 안티패턴. Kepware/Ignition = 서버 태그 DB. Task는 의미 이름만 |
| 드라이버 범위 | **`modbus_tcp` + `mock` 만 구현** | s7/ab/opcua는 파일도 만들지 X. 2번째 실드라이버가 Protocol을 보정 (task-first "자란다" 원칙) |

## 11. 남은 세부 (구현 중/후 결정 — 실물 데이터로 튜닝)

- **quality 판정 규칙** (언제 STALE vs BAD): 스캔 타임아웃/연속 실패 임계값 = **실물 첫 런 데이터로 튜닝** (추측 금지, 하드웨어 전 sim 소진 원칙).
- **스캔 주기 config**: 단일 주기로 시작. fast/slow 스캔 클래스는 필요해질 때.
- **timestamp**: read 시각 하나로 시작. OPC-UA SourceTS vs ServerTS 구분은 OPC-UA 드라이버 붙일 때.
- **배치 write / `validate()` 세부 문법**: 드라이버 구현하며 확정.
- **워치독/heartbeat**(§5.4): 태그로 모델링(양방향 register 태그) — Module 스캔 루프에 얹음.

---

## 12. 집 재현 런북 — 설치부터 핸드셰이크 검증까지 (순서대로)

> 2026-07-23 세션에서 한 스텝 그대로. **다른 PC(집)에서 처음부터** 재현하는 절차. `git pull` 후 이 순서대로.

### 12.0 git으로 딸려오는 것 vs 집에서 새로 해야 하는 것

| 딸려옴 (repo) | 집에서 새로 |
| --- | --- |
| 래더 프로그램 `plc/test_plc/` (3렁) | OpenPLC **Editor + Runtime** 설치 |
| Modbus 서버 config `.../servers/test_plc.json` (`enabled:true`, :502) | Runtime **첫 계정** 생성(머신 로컬) |
| 진단 스크립트 `plc/probe.py` | Editor→Runtime **연결** + **Build & Upload**(libplc는 그 머신에서 빌드) |
| | Arduino AVR 툴체인(첫 빌드 때 Editor가 자동 설치) |

### 12.1 설치

1. **OpenPLC Editor v4** 다운로드·설치
2. **OpenPLC Runtime v4** (standalone, headless) 다운로드·설치
   - ⚠️ v4 Runtime은 **웹 UI 없음** (v3의 :8080 아님). REST API :8443 + 디버그 WebSocket. 브라우저로 열지 말 것.

### 12.2 프로젝트 열기

3. Editor에서 `plc/test_plc` 프로젝트 열기 (repo에 있음 — 래더 3렁 + 서버 config 그대로 로드됨)
4. (선택) 래더 검증만 빠르게: 왼쪽 툴바 **Start Simulator** → 디버그에서 `sensor` force → object_arrived/conveyor 반응 확인.
   - ⚠️ **Simulator는 AVR 에뮬 = Modbus TCP(:502) 안 엶** (§5.1). 래더 눈으로 보는 용도. Python 붙이려면 아래 Runtime 필요.

### 12.3 Runtime 띄우고 연결

5. **standalone Runtime 실행** (콘솔에 플러그인 로그 뜸)
6. Editor 왼쪽 트리 **`Device → Configuration`**:
   - Device = `OpenPLC Runtime v4`, IP Address = **`127.0.0.1`**
   - 처음이면 **Create First User** 팝업 → username/password 생성 (이 계정은 **머신 로컬**, Editor↔Runtime 제어(:8443) 로그인용. Modbus랑 무관)
   - **Connect** → `● Connected | PLC: EMPTY` 뜨면 성공 (EMPTY = 아직 프로그램 안 올라감)

### 12.4 서버 확인 + 업로드

7. **서버 config 확인** (repo에서 딸려옴): 트리 `Device → Servers → test_plc` → `Enable Server` ON / Port 502 / 0.0.0.0 인지 확인. (꺼져 있으면 켜기.)
   - Buffer Mapping: 기본값 그대로 (%QX→Coils, %IX→Discrete Inputs).
8. 왼쪽 툴바 **`Build and Upload`** (Clean 아님) → 컴파일+업로드 → Runtime이 `libplc` 빌드(10~30초) → **RUNNING** (`PLC: EMPTY`가 프로그램명으로 바뀜)
   - ⚠️ 이 스텝 빼먹으면 Runtime 로그에 `No libplc_*.so / State transition to RUNNING failed`.

### 12.5 검증 (하드웨어 0)

9. **:502 살아있나**:
   ```powershell
   Test-NetConnection -ComputerName 127.0.0.1 -Port 502   # TcpTestSucceeded : True
   ```
10. **래더 + 핸드셰이크 실측** (두 채널 동시 사용):
    - **Editor 디버그(:8443)** 로 `sensor` **force = TRUE** (물체 도착 흉내. %IX라 마스터가 못 씀 → 여기서 주입)
    - **Python 마스터(:502)** 로 코일 읽기:
      ```powershell
      uv run --with pymodbus --no-project python plc/probe.py          # 코일 상태 read
      uv run --with pymodbus --no-project python plc/probe.py --pick   # 집기응답(pick_done 펄스)
      ```
    - 기대: sensor force True → `object_arrived=True, conveyor_run=False` → `--pick` → `object_arrived=False, conveyor_run=True` → sensor force 해제 → idle.
    - coil 0/1/2 = conveyor_run/object_arrived/pick_done.

### 12.6 트러블슈팅 (이번 세션에 실제로 물린 것들)

| 증상 | 원인 / 해결 |
| --- | --- |
| `No libplc / RUNNING failed` | 프로그램 미업로드 → §12.4 Build and Upload |
| `Test-NetConnection :502` False | Simulator만 켬(TCP 안 엶) → standalone Runtime에 업로드 / 서버 Enable / RUNNING 확인 |
| sensor를 Python으로 못 켬 | `%IX`=Discrete Input=마스터 읽기전용. 하드웨어 없을 땐 **디버그 force**로 주입 (실물은 포토센서가 자동) |
| write 직후 read 했는데 상태 그대로 | **PLC 스캔(20ms) 전에 읽음.** write 후 >20ms 기다렸다 read (probe.py `--pick`은 반영됨). 폴링 루프는 자연히 무관 |
| sensor가 계속 True | 디버그 force는 sticky — 직접 내려야. 실물은 하드웨어가 몰아줌 |
