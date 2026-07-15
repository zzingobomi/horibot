# 분산 로깅 설계 + 구현 계획 (SSOT — 다음 세션 구현 핸드오프)

> **상태: 설계 확정 · 구현 미착수.** 다음 세션이 **이 문서대로** 구현한다. 여기 적힌 결정은
> 2026-07-15 긴 논의로 확정된 것 — **임의로 뒤집지 말 것.** 구현 착수 전 §5(코드 확인 항목)부터.
> 배경 리서치·출처는 §9.
>
> ⚠️ **2026-07-15 개정**: §2 의 초기 결정("host 는 wire key 에만, payload 는 순수 텍스트,
> collector 가 키에서 host 추출")은 **폐기**됐다. 최종 = **host 는 발행 시점에 로그 레코드
> 자체에 담고, collector 는 저장만 한다. wire key 는 라우팅 전용.** 이 개정의 전체 근거·트레이드오프·
> 왜 A/B 를 오래 못 정했는지는 **§10 (결정 기록)** 에 있다 — 프레임워크 레벨 결정이라 반드시 §10 부터.

---

## 0. 목표와 범위 (딱 이것만)

- **문제**: 콘솔 로그가 프로세스 종료 시 유실. **집은 항상 실 분산환경**(PC + 다중 Pi)이라 로그가
  여러 host 에 흩어짐.
- **목표**: 여러 host 의 로그를 **중앙 한 파일로 영속.** 그 이상(구조화 스키마/DB/MCAP/상관 id)은
  지금 범위 아님(§7).
- **범위**: **로깅만.** CPU/메모리 등 메트릭은 **별도**(§7 — 데이터 흐름이 반대라 같은 컴포넌트로
  안 묶음).
- **sink**: **지금은 파일 하나.** DB 는 프로젝트 막바지에 **어댑터 한 곳만** 바꿔 붙임(§4).

---

## 1. 확정 아키텍처

```
[모든 host: PC, pi_hori1, pi_hori2, ...]
  Runtime 부팅 시 logging 핸들러 1개 부착 (= 런타임 인프라, 모듈 아님)
    → 콘솔 출력 그대로 + 같은 줄을 Zenoh 키  log/{host}  로 publish
                              │
                              ▼  (Zenoh transport, 이미 있음)
[중앙: PC 만]
  LogCollector  (계약 없는 모듈, pc.yaml 에만 등록 — bridge 와 같은 부류)
    → log/**  구독 → 한 파일에 append (일단위 rotation)
        logs/horibot-YYYY-MM-DD.log  (rotation 산출물; 당일 활성 파일 = logs/horibot.log)
```

**결정과 이유 (다음 세션이 의심하지 말 것):**

1. **발행(핸들러) = 런타임 인프라, 모든 host.** `logging.configure()` 같은 전역 설정이라 모듈이
   아니다. Runtime 부팅에서 콘솔 핸들러 옆에 "Zenoh 발행 핸들러"를 하나 더 붙인다. 그 host 의
   **모든 모듈 로그**가 자동으로 실린다.
2. **수집(LogCollector) = 계약 없는 모듈, 중앙 host 에만.** 이유 = **정확히 한 host 에서만 떠야**
   하는데 Runtime 은 모든 host 에서 뜬다. 런타임 인프라로 두면 **Pi 마다 collector 가 떠서** 파일을
   제각기 쓴다 → 이를 막으려면 "중앙 host 일 때만"이라는 **host 게이팅을 런타임에 새로 발명**해야 함.
   그런데 "특정 host 에서만 실행"은 **deployment yaml + 모듈**이 이미 해준다: **분산 = pc.yaml 에만
   등록**, **단일 머신 = mock.yaml (그 머신이 곧 중앙)**. Pi yaml 에는 절대 안 넣는다.
   **선례 = `bridge`**: robot 아님·계약 거의 없음·순수 인프라인데도 모듈로 두고 배포로 배치.
   LogCollector 는 bridge 와 동류다.
3. **로그 ↔ 메트릭 분리** — §7.

---

## 2. 와이어 / 데이터 형식 (단순하게 — 결정)

- **키**: `log/{host}` (host = 배포/`--host` 이름: `pc`/`pi_hori1`/…). 구독은 `log/**`.
  키는 **라우팅 전용** — 미래 host 별 선택 구독을 위한 파생 투영(robot-scoped 키가 robot_id 를
  키·payload 양쪽에 두는 것과 동형). **collector 는 이 키를 파싱하지 않는다** (근거 §10).
- **페이로드**: **포맷된 텍스트 로그 줄 1개를 UTF-8 bytes 로.** JSON 스키마/구조화 레코드 **안 만든다**
  (지금 목표는 "콘솔 로그 안 잃기"지 분석용 구조화가 아님). **줄은 발행 시점에 자기완결** —
  `host` 를 포함해 `run_id`/`pid`/`thread`/시각·레벨·로거명이 모두 줄 안에 있어야 한다. 즉 파일에서
  한 줄만 떼어 읽어도 어느 host 에서 왔는지 알 수 있다. 발행자(runtime `--host`)가 자기 host 를
  권위 있게 알므로 **발행 핸들러가 각인**한다. (§10.3–10.4 — host 를 키에서 뽑던 초기 안 폐기.)
- **collector 파일 줄**: 수신한 줄을 **그대로(verbatim) 기록.** host 재추출·접두 조립 안 함
  (레코드가 이미 자기완결이므로). collector 는 "받아서 append" 만 하는 dumb 소비자.

---

## 3. 파일 / 로테이션

- 경로: **`logs/`** (backend CWD 기준, 자체 gitignore). **debug/ 아티팩트와 분리** — 라이프사이클이
  다르다: debug/ = 자주 비우는 scratch, logs/ = 14일 rolling 보존 기록. 섞으면 debug scratch 를
  비우다 로그 히스토리까지 날아간다 (2026-07-15 결정, 초기 `debug/logs` 안 폐기).
- **일단위 rotation**: `logging.handlers.TimedRotatingFileHandler(when="midnight", backupCount=14)`.
  당일 활성 파일 = **`logs/horibot.log`**, 자정 rotation 시 전날 파일이 **`logs/horibot-YYYY-MM-DD.log`**
  로 이름 변경(`namer` 로 date suffix 를 파일명에 넣음 — 기본 `.log.DATE` 대신). per-run 파일 아님.
- 한 파일에 모든 host 병합. 줄 순서는 각 줄의 시각으로 정렬 가능(§8 NTP 전제).

---

## 3-B. debug/ 아티팩트 vs 로그 — 무엇을 어디에 (다음 세션 필독)

로깅이 생기면 지금 detector 가 `debug/` 에 쏟는 것 중 일부(텍스트)는 로그로 옮겨 **중복을 없앤다.**
원칙: **바이너리·대용량·재투영에 이미지와 함께 있어야 하는 것 = 아티팩트(`debug/`)** / **사람이
읽는 서사·결정·타이밍·스칼라 요약 = 로그.** (리서치 §9 의 "시계열 bag vs 결정 로그" 분리와 동형.)

| 지금 파일 (`debug/detect/{세션}/`) | 성격 | 결정 |
| --- | --- | --- |
| `_det_*.png`(overlay) · `_color.png` · `_depth.png`(16bit) · `_mask_c*.png` | 바이너리 이미지 | **debug/ 유지** (로그로 못 남김) |
| `_det_*_c*.ply` · `_fuse_*_c*.ply` | 점군 | **debug/ 유지** |
| `.json`(intrinsic/hand_eye/TCP pose/depth_scale + 후보 기하) | 구조화 사이드카 — **depth·mask 를 3D 로 되짚는 입력**, 이미지 옆에 있어야 의미 | **debug/ 유지** (아티팩트 번들 일부) |
| `_det_*.txt`(score/base_z/height/pos/footprint/points) | 스칼라 텍스트 | **드롭** — 그 값은 (a) `.json` 에 구조화 + (b) 로그에 서사로 남음. 세 번째 사본 불필요 |

**로그로 (이미 `logger.info`, 중앙 파일에 수렴)**: detect/resolve **elapsed**, 채택 파지(pair/tilt/w),
base_z/top/footprint **요약**, 실패 사유, fuse 결과. → **타임라인·결정 = 로그 / 재투영·시각검증
입력 = debug/ 아티팩트.**

**분산 관점 (중요)**: 아티팩트는 **생산 host 로컬 `debug/`** 에 남고(현재 detector=PC), **로그만
중앙으로 수렴.** 무거운 바이너리를 host 간에 안 옮기고 가벼운 로그만 모으므로 이 구분은 분산에서
자연스럽다.

**구현 시 할 일**: [detector/module.py](../backend/modules/detector/module.py) `_dump_debug_image`
에서 **`.txt` 쓰기 제거**(로깅 도입과 함께). PNG/PLY/JSON 덤프는 유지. (덤프 자체 on/off 게이팅은
별개 관심사 — 지금 draft 라 항상 on.)

---

## 4. DB-later (지금 안 만듦, 자리만)

- 나중에 DB 로도 남기려면 **LogCollector 의 "쓰는 대상" 한 곳만** 파일→DB(또는 둘 다)로 바꾼다.
  host 핸들러·발행·전송·구독은 **불변.** 지금은 그 이상 sink 추상화(인터페이스/어댑터 클래스)
  **만들지 않는다** — collector 안에 write 함수 하나면 충분, 추상화는 실제 DB 붙일 때.
- storage Phase 3(Postgres/MinIO, [[project_storage_phase3_minio]])와 결합 검토는 그때.

---

## 5. 구현 착수 전 코드 확인 항목 (다음 세션이 **먼저** 볼 것)

아래는 아직 미탐색 — 추측 금지, 코드로 확인 후 구현:

1. **Runtime 부팅에서 transport(Zenoh) 세션이 서는 시점** — 발행 핸들러는 세션이 준비된 뒤
   붙어야 함. `apps/main.py` `run()` + `framework/runtime/` 확인. (세션 서기 전 로그는 콘솔만 —
   §6 엣지케이스.)
2. **모듈이 raw 키를 구독하는 방법** — `ModuleRuntime` 이 `subscribe`/transport 핸들을 노출하나,
   아니면 `@subscriber`(계약 전용)만 있나. `log/**` 는 계약 스트림이 아니라 raw 구독 필요 →
   ModuleRuntime API 확인([framework/runtime/api.py]). 없으면 transport 직접 접근 경로 확인.
3. **모듈 등록 방식 + pc.yaml 형식** — [apps/registry.py](../backend/apps/registry.py) lazy import
   등록 + [config/deployments/pc.yaml](../backend/config/deployments/pc.yaml) 의 modules 목록에
   collector 추가하는 형태.
4. **host 식별자 소스** — `--host` 인자(배포 이름)를 그대로 `{host}` 로 쓰면 되는지 확인
   ([apps/main.py](../backend/apps/main.py) `args.host`).
5. **현재 로깅 설정 위치** — `apps/main.py` `logging.basicConfig(...)` (이미 시각 포맷 있음). 발행
   핸들러를 여기(또는 runtime 부팅)에 어떻게 얹을지.
6. transport 표면은 확인됨: [infra/transport/zenoh.py](../backend/infra/transport/zenoh.py) 에
   `publish(key, bytes)` / `subscribe(key, cb(payload))` 존재 → 발행·수집 배선은 이걸로 충분.
   **`subscribe` 콜백은 payload(bytes) 만 받고 매칭된 키는 안 넘긴다 — 이건 의도된 채로 유지한다**
   (§10 결정). 그래서 collector 는 키에서 host 를 못 얻고, 못 얻어도 된다: host 는 레코드 안에
   이미 있다(§2 개정). LogCollector 는 boundary 모듈이라 `transport: RawTransport` 를 생성자
   인자로 받아 `transport.subscribe("log/**", on_line)` 로 raw 구독한다 — **bridge 와 동일한
   주입 경로**([framework/runtime/app.py](../backend/framework/runtime/app.py) `add_module` 가
   파라미터 이름 `transport` 를 raw transport 로 주입). `ModuleRuntime`(publish/call) 에는 raw
   subscribe 가 없으므로 이 경로가 정답(§5-2 열린 질문 해소).

---

## 6. 구현 단계 (확인 후)

**L1-a 발행 핸들러 (모든 host)**
- `logging.Handler` 서브클래스: `emit()` 에서 **자기완결 줄로 포맷**(host·run_id·pid·thread·시각·
  레벨·로거명·메시지 포함 — §2 개정) → `transport.publish(f"log/{host}", line.encode())`.
  host 는 발행자가 아는 `--host` 값을 각인, 키 `log/{host}` 도 같은 값에서 만든다(중복 아님 —
  키=라우팅, 줄 안 host=레코드 데이터, 서로 다른 계층 §10.4). root logger 에 콘솔 핸들러와
  **함께** 부착 (runtime 부팅, transport 준비 후).
- **필수 가드**:
  - `emit()` 은 **절대 예외 전파 안 함**(발행 실패해도 앱 무사) — try/except swallow.
  - **피드백 루프 차단**: 발행 경로(transport)가 INFO+ 로그를 내면 무한재귀. reentrancy 가드
    (emit 중이면 skip) + transport 로거(`infra.transport.zenoh`)는 발행 핸들러에서 제외.
- (Zenoh 내부 Rust 로그는 stderr 직행이라 python logging 에 안 탐 — 재귀 걱정 없음.)

**L1-b LogCollector 모듈 (PC 만)**
- 계약 없음. `start()`: `subscribe("log/**", on_line)`. `on_line`: 받은 줄을 **그대로** append
  (host 추출·접두 조립 안 함 — 레코드가 자기완결, §2 개정). 직접 file write 또는
  `TimedRotatingFileHandler` 로 `logs/horibot.log` (rotation 시 `logs/horibot-YYYY-MM-DD.log`).
  `stop()`: 핸들러/구독 닫기.
- **자기 로그 루프 주의**: collector 가 파일 쓰며 INFO 로그를 내면 그게 다시 발행→수신→기록.
  파일 쓰기 경로에서 로깅 최소화 + L1-a reentrancy 가드로 흡수.
- registry 등록 + **중앙 host yaml 에만** modules 추가 (분산=pc / 단일=mock, Pi 는 제외).

**엣지 (다음 세션이 놓치지 말 것)**
- **early-boot**: transport 서기 전 로그는 콘솔만(발행 X) — 허용. 필요하면 작은 버퍼로 나중 flush(선택).
- **collector 미가동**(mock/collector 없는 배포): host 는 `log/{host}` 로 계속 발행하지만 구독자
  없으면 그냥 드롭 — 무해.
- **PC 자기 로그**: PC 도 모듈 돌리므로 `log/pc` 발행 → 같은 PC 의 collector 가 수신 → 파일에 포함(정상).

**검증**
- mock/단일: 로그가 파일에 쌓이나.
- **실 분산(집)**: 모든 host 로그가 **한 파일에 host 태그로** 모이고 시각순 정렬되나(§8 NTP 후).

---

## 7. 지금 범위 밖 (안 함 — 이유 + 미래 진입점)

- **메트릭(CPU/메모리/온도/디스크/GPU/health/uptime …)**: **별도 `host_monitor` 모듈**(각 host 가
  자기 상태 주기 publish → 대시보드가 bridge 통해 구독). **로그와 안 묶는 이유 = 데이터 흐름이 반대.**
  로그는 여러 host → 중앙 collector → 파일로 **수렴**, 메트릭은 각 host → 대시보드로 **분산 소비**.
  성장 방향도 다름(로그=영속/분석, 메트릭=라이브/이력). 저장이 필요해지면 그때 붙임.
- **per-host 로컬 fallback 파일**: 지금 안 함(한 파일 원칙·단순). 수집기/네트워크 장애 복원력이
  필요해지면 추가.
- **구조화 레코드/JSON 스키마/run_id 상관관계/MCAP/OTel**: 파지 A/B 같은 정밀 분석이 텍스트 로그로
  부족해질 때 도입. 그때 `data` payload·`run_id` 전파·MCAP(Foxglove) 검토 — §9 리서치 참고.
- **Zenoh storage-manager 로 자동 영속**: 매력적이나 zenohd **라우터 도입** 필요(우린 peer 모드) +
  KV-최신값 의미라 지금 아님. DB/MinIO 단계에서 재검토.

---

## 8. 집 NTP 시계 동기 (사용자 1회 셋업 — 코드 아님, 전제조건)

병합 로그의 줄 순서·host 간 시각 비교가 맞으려면 PC·Pi 시계가 동기돼야 함. **집 LAN 은 인터넷
됨** → 각 머신이 인터넷 NTP 에 동기하면 끝(로컬 NTP 서버 불필요).

**각 Pi (Raspberry Pi OS / systemd):**
```bash
sudo timedatectl set-ntp true
timedatectl status   # "System clock synchronized: yes" / "NTP service: active" 확인
```
(systemd-timesyncd 내장. 오프라인 대비 강건성 원하면 `sudo apt install chrony` 로 교체 가능하나
인터넷 되면 불필요.)

**Windows PC:**
- 설정 → 시간 및 언어 → 날짜 및 시간 → **"자동으로 시간 설정" 켜기** + **"지금 동기화"**.
- 또는 관리자 cmd: `w32tm /resync` (Windows Time 서비스 자동시작: `sc config w32time start=auto`
  → `net start w32time`). 서버 예: `time.windows.com` 또는 `pool.ntp.org`.

**검증**: PC 와 각 Pi 에서 현재 시각 비교(수 ms 이내여야). LAN NTP 정확도는 sub-ms~수 ms 라 로그
정렬엔 충분. → 이후 병합 로그가 시각순으로 신뢰 가능, debug 덤프 `timestamp_unix` 도 host 간 정확.

---

## 9. 배경 리서치 (압축) + 출처

- **Zenoh 자체**: 앱 로그 수집 기능 없음(내부 `RUST_LOG`→stderr, observability 는 로드맵). storage-
  manager 로 발행 데이터 영속 가능(filesystem/rocksdb/influxdb/**s3=MinIO**) — **zenohd 라우터 필요**
  + KV 의미(§7).
- **로봇 표준(ROS2 rmw_zenoh)**: 로그를 5계층 분리 — **시계열(rosbag2/MCAP)** / **구조화 이벤트**
  / diagnostics / traces / safety. 우리 `debug/detect/` 덤프가 "시계열 bag" 역할. MCAP =
  자기기술·인덱스·직렬화무관·Python·Foxglove (미래 구조화 시 후보).
- **업계(OTel)**: 구조화 JSON + **trace/correlation id 전파** + 중앙 aggregator 스트리밍이 정석
  (미래 구조화 도입 시 지침).
- **우리 결론**: 지금은 "텍스트 로그를 Zenoh 로 중앙 한 파일에 수렴"이면 충분. 구조화·상관·DB 는
  필요해질 때 위 표준을 따른다.

출처: [Zenoh storage-manager](https://zenoh.io/docs/manual/plugin-storage-manager/) ·
[Zenoh S3/MinIO](https://zenoh.io/blog/2023-07-17-s3-backend/) ·
[Zenoh observability 로드맵](https://github.com/eclipse-zenoh/roadmap/discussions/92) ·
[ROS2 구조화 로그·rosbag 5계층](https://thomasthelliez.com/blog/structure-ros-2-logs-rosbags-ai-assisted-robot-debugging/) ·
[MCAP ROS2](https://mcap.dev/guides/getting-started/ros-2) · [rmw_zenoh](https://github.com/ros2/rmw_zenoh) ·
[OTel Logs](https://opentelemetry.io/docs/specs/otel/logs/) ·
[structured logging](https://uptrace.dev/glossary/structured-logging)

---

## 10. 결정 기록 — `RawTransport.subscribe` 가 소비자에게 key 를 줘야 하는가 (2026-07-15 긴 토의)

> 이건 로깅 기능 하나의 결정이 아니라 **transport 계층 API 설계**라 별도 기록으로 박제한다.
> Claude 와 GPT 가 이 문제로 A↔B 를 여러 번 뒤집었고, 그 **뒤집은 과정 자체**가 프레임워크 설계
> 시 피해야 할 실패 패턴이라 §10.2 에 남긴다. 미래 세션은 §10.6 tripwire 만 확인하면 된다.

### 10.1 왜 이 논쟁이 시작됐나

§2 초기 안 = "host 는 wire key(`log/{host}`)에, payload 는 순수 텍스트, collector 가 키에서 host
추출". 구현 확인(§5) 중 사실이 드러남: 우리 `RawTransport.subscribe(key, cb)` 콜백은 **payload(bytes)
만** 받고 **매칭된 concrete key 를 안 넘긴다** ([infra/transport/zenoh.py](../backend/infra/transport/zenoh.py)
`_on_sample` 이 `sample.key_expr` 을 버림). 즉 collector 가 `log/**` 를 wildcard 구독해도 "이 줄이 어느
host 에서 왔나"를 키에서 알 수 없다 → §2 가 현재 transport 표면과 모순. 여기서 "subscribe 를 key 전달로
고쳐야 하나?"가 튀어나왔고, **이게 로깅 문제가 아니라 transport API 설계 문제**임이 드러나며 논쟁이 됨.

- **A** — `subscribe` 는 지금대로 payload 만. host 는 로그 **레코드 자체**에 발행 시점에 담는다.
  collector 는 그대로 저장. transport 안 건드림.
- **B** — `subscribe` 를 `(key, payload)` 전달로 확장(= `subscribe_liveliness` 와 동형). collector 가
  키에서 host 를 얻는다. 최하위 transport API 를 "메시지 봉투 전체 전달"로 완성.

### 10.2 왜 결정을 못 내리고 A↔B 를 왔다갔다 했나 (실패 기록 — 다음 세션 경계)

입장이 여러 번 뒤집혔다. 원인은 **논거의 옳고 그름이 아니라 판단 방법(process)의 결함**:

1. **구현을 당위의 근거로 씀 (descriptive↔normative 혼동).** "지금 코드가 subscribe 에서 key 를
   버리니 그게 규약"이라고 함 — 설계 단계에선 순환 논증. *현재 구현이 설계 구멍일 수 있는데* 그걸
   정답의 증거로 쓴 것.
2. **논거를 고립시켜 평가.** "transport 최하위는 lossy 금지"(B), "로깅 표준은 host 를 레코드에"(A)
   같은 단일 원칙을 **전체 시스템에 대조하지 않고** 하나씩 받아 그때그때 결론.
3. **서로 다른 두 결정을 하나로 뭉갬.** (가) transport API 가 무엇을 추상화하나 (나) 로그 레코드는
   어디에 정체성을 담나 — **다른 계층**인데 한 덩어리로 다뤄, 한쪽 근거로 다른 쪽을 뒤집음.
4. **결론 먼저, 코드 확인 나중.** transport 표면 전체 / 계약 계층(@subscriber 파라미터 강제, publish 의
   key 파생) / 기존 소비자 선례를 다 훑기 전에 결론부터 냄. 순서가 거꾸로.

**교훈 — 프레임워크 레벨 결정 절차:** ① 문제 일반화(로그만인가 프레임워크 전체인가) → ② 그 계층의
모든 관련 구현·선례 확인 → ③ 설계원칙(SSOT / 책임분리 / 확장성 / 일관성)에 대조 → ④ 그다음 결론
고정, **새 사실(코드/제약) 없으면 안 뒤집음** (수사적 압박으로 안 바꿈).

### 10.3 결정을 가른 진짜 축

길게 돌아 도달한 핵심: subscribe 가 key 를 줘야 하는지는 **"우리 시스템은 무엇을 기록/재생하는가"**
하나로 환원된다.

- topic 은 우리 계약에서 `f(event_type, payload.robot_id)` 의 **순수 파생 함수**다: wire_key 템플릿은
  이벤트 타입마다 고정, 런타임 변수 `{robot_id}` 는 payload 에 있고 publish 가 그걸로 키를 만든다
  ([framework/runtime/app.py](../backend/framework/runtime/app.py) `_TransportRuntime.publish` —
  robot_id 없으면 raise). → typed event 를 다시 publish 하면 Framework 가 topic 을 **재생성**. 재생에
  topic 저장 불필요.
- 단 우리 wire payload 는 **타입을 자기기술하지 않는다** (`encode_event` 은 필드 dict 만;
  `decode_event` 은 `event_cls` 를 밖에서 받고, 그 `event_cls` 는 topic→계약 매핑으로 복원). 그래서:
  - **type-specific(도메인) recorder** = 무슨 타입 기록하는지 앎 → topic 불필요 → **A**.
  - **generic/schema-agnostic recorder**(packet capture 류) = 타입 모름 → topic 이 타입 판별자로
    필요 → **B**.

즉 결정 축 = **"Packet 을 기록하는 시스템인가, Domain Event 를 기록하는 시스템인가."**

우리 프로젝트의 replay(Task / Calibration / Debug)는 전부 **도메인 레벨**이고 대부분 이미 DB·아티팩트에
자기기술 레코드로 존재한다. transport packet 을 기록/재생하는 소비자는 로드맵에 **없다** (generic
기록 = MCAP 은 §7 에서 defer, 오더라도 MCAP 은 스키마 자기기술 포맷이라 우리 topic 에 안 기대고
Zenoh-native capture 가 담당할 수 있어 RawTransport 관심사 아님). → **우리는 Domain Event 를 기록하는
시스템 → A.**

### 10.4 최종 결정

- **Transport API = A (변경 없음).** `RawTransport.subscribe` 는 payload 만 전달. transport 코드
  손대지 않음.
- **로그 레코드 = 발행 시 자기완결.** 발행 핸들러가 `host / run_id / pid / thread / ts / level /
  logger / message` 를 담은 줄을 만든다. collector 는 **저장만** (키 파싱·host 재추출 안 함). wire key
  `log/{host}` = **라우팅 전용**.
- 이 둘은 **다른 계층의 원칙**이라 모순 아님: transport = need-driven projection / 도메인 레코드 =
  자기완결. (앞 토의에서 이 둘을 한 결정으로 착각한 게 flip-flop 의 주원인 — §10.2-3.)

### 10.5 트레이드오프 (무엇을 얻고 무엇을 포기했나)

**A 채택으로 얻은 것**
- 정체성 SSOT 가 payload/레코드 **한 곳** — 키 템플릿 스킴과 디커플. 키 포맷 바뀌어도 소비자 안 깨짐.
- 영속/재생 안전: 파일·DB 에 앉은 레코드가 키 없이도 자기완결.
- 기존 framework(계약 typed-event / mirror / robot-scoping §2.7 / contract-gen)와 **100% 정합** —
  새 철학 도입 0.
- RawTransport 의 실제 원칙("소비자가 필요한 것만 노출")과 일관: subscribe 는 이미 Zenoh `Sample` 의
  `timestamp`/`kind`/`encoding`/`attachment` 등을 **다 버리는 need-driven projection** 이다. "lossy
  금지(B)"를 진지하게 밀면 그 필드들도 다 노출해야 하는 **reductio** 에 걸린다.
- **국소 가역**: 판단이 틀려도 `subscribe` 시그니처 ~6곳만 고치면 되고, 모듈 코드로 전파 안 됨
  (모듈은 `@subscriber` 벽 뒤 — 파라미터가 typed event 하나로 강제되어 애초에 키를 못 봄).

**A 로 포기한 것 (= B 였다면 얻었을 것)**
- 최하위 transport API 의 "봉투 완전 전달" 순수성.
- 디코드 없이 키만으로 라우팅/필터하는 generic 소비자 지원(observability / packet sniffer /
  transport-level recorder). — **지금 로드맵에 그 소비자가 없어 speculative surface 로 판단.**
- `publish`(key 필요) / `subscribe_liveliness`(key 있음) / `subscribe`(key 없음) 의 표면 비대칭이
  남음. (단 payloadful vs payloadless 로 설명되는 비대칭이라 비일관은 아님 — liveliness 는 payload 가
  없어 키가 유일 운반체.)

**정직한 신뢰도:** A ~65 / B ~35. B 의 순수성 논거는 진짜라 100% 아님 — 그래서 §10.6 tripwire 를 남긴다.

### 10.6 B 로 뒤집는 tripwire (미래 세션 재검토 조건)

로드맵에 **확정된 generic / schema-agnostic wildcard 소비자**가 올라올 때:
- transport packet recorder / sniffer, 또는
- §7 MCAP 을 Zenoh-native 가 아니라 **우리 transport 위에** 얹기로 결정,

→ 그때 `subscribe` 를 `(key, payload)` 로 확장(= `subscribe_liveliness` 동형). 변경은 국소적
([framework/runtime/app.py](../backend/framework/runtime/app.py) `_register_subscriber`·mirror,
[modules/bridge/ws.py](../backend/modules/bridge/ws.py), [modules/bridge/mjpeg.py](../backend/modules/bridge/mjpeg.py),
[scripts/run_task.py](../backend/scripts/run_task.py) + 테스트 mock transport 3곳)이고 모듈 무영향.
**그 소비자가 실제로 손에 잡히기 전에는 미리 B 로 가지 말 것** — 없는 소비자를 위한 speculative
surface 는 SSOT 조기 파열.
