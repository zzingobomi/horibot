# contract gen — 분산 배치·정적 생성 논의 기록 (2026-07-06)

> **결정: 지금은 아무것도 안 바꾼다.** gen:types 는 현행(전 모듈 로드된 mock/dev
> backend 에서 생성) 유지. 본 문서는 그 결정에 도달하기까지 검토·기각된 선택지들과
> 재논의 트리거를 남기는 기록 — 다음 세션이 같은 의심을 처음부터 다시 돌지 않게.

## 1. 출발 질문

분산 배치 — PC1(모듈A, 모듈B, bridge) + PC2(모듈C, 모듈D) — 에서 모듈 D 의 계약을
프론트로 노출할 수 있는가?

**답은 두 축으로 갈린다:**

| 축 | 되나 | 메커니즘 |
|---|---|---|
| **데이터** (서비스 호출/스트림 구독) | ✅ | bridge 는 relay-only, Zenoh 가 D 의 위치를 투명 라우팅. bridge 와 D 가 다른 host 여도 무관 |
| **타입 생성** (gen:types) | 부분 배치에선 ❌ (의도된 가드) | `build_contract_json` 이 running runtime 의 snapshot 에서 payload 를 채움 → bridge host 에 로드 안 된 모듈의 payload 없음 → incomplete-host guard 가 거부 |

핵심 구분: **gen:types 는 build-time 스텝이다.** contract.ts 는 개발 머신에서 생성해
커밋하는 산출물이고, 분산 배포 중에 gen 을 돌릴 일 자체가 없다. 따라서 위 ❌ 는
운영 결함이 아니라 "gen 은 전 모듈 로드된 host 에서" 라는 워크플로 전제.

## 2. 코드로 확정한 사실 (재검증 불필요)

1. **key↔payload 바인딩의 원천 = 핸들러 시그니처.** contract.py 에는 키(StrEnum)와
   타입(BaseModel)이 나란히 있을 뿐 연결 선언이 없다. 연결은
   `@service` 데코레이터가 `get_type_hints(method)` 로 시그니처에서 추출
   ([framework/contract/service.py](../backend_v2/framework/contract/service.py)).
2. **추출된 바인딩은 metadata 로 클래스에 이미 부착된다** (`ServiceSpec` →
   `_service_spec` attr, `@publishes` 는 데코레이터 인자로 payload 명시). 즉
   **module.py 를 import 만 하면** 인스턴스/runtime 없이 전부 읽힌다 —
   `build_snapshot_from_classes` ([framework/runtime/snapshot.py](../backend_v2/framework/runtime/snapshot.py))
   가 그 구현이고 `/contract/graph` 가 실사용 중.
3. **json 과 graph 의 소스가 다른 건 드리프트가 아니라 요구 차이.**
   graph = 분산 배치 *운영 중에도* 전 fleet 을 보여야 하는 런타임 뷰어 → MODULE_REGISTRY
   정적 introspect. json = build-time 산출물 → running mock 으로 충분.
4. **"전체 import 가능" 은 우연이 아니라 아키텍처 전제.** Runtime(프로세스) 이 최소
   단위고 모듈들은 한 인터프리터에 동거한다 (framework_dogfood_plan §15). 같은 host 에
   배치되는 모듈들의 의존성 공존은 요구사항이며, 개발 머신은 "전 모듈을 배치하는
   host"(mock) — pytest(전체 import) / mock 부팅도 같은 전제 위에 있다.
   **모듈별 venv/Docker 는 이 아키텍처와 양립 불가** (그건 폐기된 "Node=최소단위" 회귀).

## 3. 검토된 선택지 사다리 (우선순위 순)

### 3.1 현행 유지 — 채택 (지금)

전 모듈 로드된 mock/dev backend 에서 gen. incomplete-host guard 가 부분 host 실수를
명확한 메시지로 차단. 개발 루프가 어차피 mock/pytest 로 전체 import 를 요구하므로
gen 만의 추가 전제가 없다.

### 3.2 모듈별 fragment 빌드 산출물 + merge — **미래 지정 경로**

각 모듈(의존성이 있는 자기 환경에서)이 `contract.fragment.json` 을 빌드 산출물로
남기고, gen 은 fragment 들을 수집·merge. **Runtime 불필요 + Zenoh 불필요 + 전체
import 불필요** — 셋을 동시에 만족하는 유일한 선택지라 미래 경로 1순위.

- fragment 내용은 클래스 객체가 아니라 **qualified 이름 문자열**
  (`waypoint.TeachRequest`) 이면 충분 — merge 측이 contract.py 전체(import-light,
  어디서든 가능)로 full catalog 를 만들고 이름으로 실 클래스를 해소. name-conflict
  는 기존 `resolve_names` 그대로.
- framework 수정 불필요 — `build_snapshot_from_classes` 가 per-class 리스트로 이미
  동작 (2026-07-06 확인).
- **버전 정합 가드 필수**: fragment 에 git hash (또는 schema version) 를 박고
  불일치 merge 거부. 서로 다른 checkout 의 fragment 를 섞으면 조용히 깨진
  contract.ts 가 나온다.

### 3.3 Zenoh fleet 집계 (runtime 에 fragment 서비스) — 목적이 다른 도구

각 Runtime 이 `contract_fragment` 서비스를 들고, gen 시점에 zenoh 로 전 peer 의
fragment 를 모아 merge. 기술적으로 성립하고 (각 host 는 자기 모듈을 이미 import 중,
fragment = 문자열) zenoh-native 라 우아하지만 — **빌드 도구가 "fleet 이 떠 있음"
이라는 운영 상태에 의존하게 된다.** 이는 애초 문제(타입 생성이 실행 상태에 결합)의
방향만 바꾼 재현.

→ 판정: **"프론트 타입 생성" 용도로는 부적합. "현재 배포된 fleet 의 실 계약 조회"
라는 별개 목적이 생기면 그때 자연스러운 설계.** 두 목적(build-time 타입 생성 vs
runtime fleet 조회)을 한 메커니즘으로 묶지 말 것.

- 부수 논점: "fleet 완전성 검사" 도 목적 따라 다르다 — 프론트가 안 쓰는 모듈이
  꺼져 있다고 gen 이 실패할 이유는 없음. 완전성 기준은 fleet 전체가 아니라
  FRONTEND_EXPOSED 커버리지.

### 3.4 AST 파싱 — 최후 수단 (사실상 안 씀)

import 없이 소스 텍스트에서 추출. 이 repo 는 **전 파일 `from __future__ import
annotations`** → 모든 annotation 이 문자열이라 alias/forward-ref/typing 해석기를
직접 만들어야 함 (`get_type_hints` 가 공짜로 해주는 것 전부). 유지보수 비용이
Python 언어 기능을 따라가는 영구 부채.

### 3.5 contract.py 에 바인딩 승격 (`SERVICE_PAYLOADS` 선언) — 기각

한때 "정적 생성의 유일한 길" 로 검토됐으나, §2-2 사실(데코레이터 metadata 가 이미
존재, import 만으로 읽힘)로 전제가 무너짐. 선언 중복 + 컨벤션 변경(10개 contract +
framework 검증 + spec 문서) 비용만 있고 얻는 게 없다. **payload 정보를 얻는 방법이
하나뿐이라는 잘못된 가정에서 나온 과잉 해결** — 문제는 "선언 위치" 가 아니라
"추출을 어디서 하느냐" 였다.

## 4. 재논의 트리거

**"어떤 모듈의 의존성이 나머지와 한 venv 에 공존 불가" 가 실제로 발생하는 날.**

그날은 gen 만이 아니라 pytest(전체 import)·mock 단일 프로세스 부팅이 같이 깨지므로
대응은 한 묶음이다:

1. 충돌 모듈을 별도 host/deployment 로 분리 (기존 메커니즘 — deployment yaml + role group)
2. 개발 루프: mock 단일 프로세스 → multi-process sim (host_*_sim 방식) 으로 분할
3. gen: §3.2 (fragment 빌드 산출물 + merge + git hash 가드) 채택

그 전까지는 어떤 선제 구현도 하지 않는다 (1-peer 환경에서 분산 메커니즘의 가치가
발휘될 상황 자체가 없음).

## 5. 남긴 미해결 (재논의 시 진입점)

- fragment 스키마 상세 (git hash 외 schema version 병기 여부)
- 완전성 기준 — FRONTEND_EXPOSED 커버리지 vs fleet 전체 (§3.3 부수 논점)
- "현재 fleet 계약 조회" 라는 별개 도구의 실수요 여부 (§3.3 을 그 목적으로 부활할지)
- `/contract.json` 엔드포인트의 장기 위상 — gen 이 §3.2 로 이관되면 소비자가
  사라짐 (제거 vs runtime 디버그 조회용 존치)
