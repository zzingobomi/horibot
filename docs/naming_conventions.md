# Naming Conventions

본 codebase 의 **wire schema (Zenoh service Req/Res 메시지) class 이름** + **key expression helper 함수 이름** 컨벤션. 산업 표준 (Google AIP / Kubernetes API / Zenoh 공식 어휘) 정렬.

**적용 scope** — `backend/core/transport/messages/*.py` 의 Pydantic Req/Res class + `backend/core/transport/topic_map.py` 의 helper 함수.

---

## 1. Wire schema class 이름 — **verb-first + sub-domain prefix**

### 1.1 어순: verb-first

method 의 이름 = **동사 + 목적어** 어순. 산업 합의:

- [AIP-131 Get](https://google.aip.dev/131) — "The RPC's name **must** begin with the word `Get`."
- [AIP-134 Update](https://google.aip.dev/134) — "The RPC's name **must** begin with the word `Update`."
- [AIP-136 Custom methods](https://google.aip.dev/136) — "The name of the method **should** be a verb followed by a noun."
- [Kubernetes API concepts](https://kubernetes.io/docs/reference/using-api/api-concepts/) — "verbs before nouns" (list/get/watch/create/update/patch/delete)
- AWS SDK — `PutObject`, `GetObject`, `DeleteBucket`

```python
# ❌ Before                  # ✅ After
StorageActivateReq           ActivateCalibrationReq
StoragePutScanReq            PutScanReq
StorageGetBlobReq            GetBlobReq
```

### 1.2 Sub-domain prefix

한 messages 파일에 여러 sub-domain 이 섞이는 자리 (예: `messages/storage.py` 가 calibration / scan / blob / reconstruction 4 도메인 다 다룸), sub-domain 어휘를 **verb 뒤 noun 자리에** 명시:

```python
# ❌ Before                  # ✅ After
StorageGetActiveReq          GetActiveCalibrationReq
StorageNewCalRunReq          CreateCalibrationRunReq         # AIP-133 Create 표준 어휘
StorageDeleteCalRunReq       DeleteCalibrationRunReq
StorageFinalizeCalRunReq     FinalizeCalibrationRunReq
StorageAppendCaptureReq      AppendCalibrationCaptureReq
```

표준 동사 어휘 — `Get` / `List` / `Create` / `Update` / `Delete` / `Put` / `Append` / `Activate` / `Commit` / `Finalize`.

### 1.3 Suffix

- Request → `*Req`
- Response → `*Res`
- 표준 Get/Update 자리도 별도 `*Res` 유지 — AIP 는 resource 자체 반환 권장이지만, 본 codebase 는 `{success, message, data}` envelope 패턴 위해 Res 항상 명시.

### 1.4 예외 — 이미 sub-domain prefix 가 본질인 자리

[messages/calibration.py](../backend/core/transport/messages/calibration.py) 의 `Handeye*` / `Intrinsic*` 류는 **calibration sub-domain 내부 5종 (handeye / intrinsic / joint / link / sag) disambiguator** 자리. verb-first 와 별개 컨벤션 (sub-domain prefix → action).

```python
HandeyeStartRes              # Handeye sub-domain + Start action
IntrinsicSaveRes             # Intrinsic sub-domain + Save action
BeginRefinementReq           # 이미 verb-first
```

→ **그대로 유지**. 신규 코드는 verb-first.

### 1.5 컨벤션이 적용되지 않는 자리 — Event / Value type

본 verb-first 규칙은 **RPC Request/Response 메시지** 한정 ([AIP-136](https://google.aip.dev/136)). 아래 카테고리는 별도 컨벤션 — `Storage*` 류 일괄 rename 시 *건드리지 말 것*:

- **Event payload** (topic message — pub/sub event) — **과거 분사 패턴** (`X.invalidated` / `X.created` / `X.updated`). [AWS EventBridge](https://docs.aws.amazon.com/eventbridge/latest/userguide/eb-events.html), Stripe event types 표준.
  - 예: `CalibrationInvalidated` — `STORAGE_CALIBRATION_INVALIDATED` topic payload. 그대로.
- **Value type / DTO** (Req/Res 안에서 wrapping 되는 helper class) — **명사** (resource type). AIP `Book`, `User` 식.
  - 예: `CalibrationRunSummary` — `ListCalibrationRunsRes.runs` 안의 item. 그대로.

판별 기준: 본 class 가 `Service.X` 의 직접 Req/Res 인가? Yes → verb-first. No (event / nested) → 별도 컨벤션.

---

## 2. Key expression 어휘 — **`key_for`**

[Zenoh 공식 어휘](https://zenoh.io/docs/manual/abstractions/):

- **Key Expression (KE)** — pub/sub 와 queryable (RPC) 모두 공유하는 string identifier. Zenoh 자체 어휘에 "topic" 없음 (ROS 어휘 잔재).
- **Key** — 개별 identifier.

본 codebase 의 `Topic` / `Service` class 는 내부 namespacing 일 뿐 — wire 자리는 같은 key expression. 둘 다 `{robot_id}` placeholder 치환 = 같은 함수.

```python
# ❌ Before
topic_for(Service.CALIB_HANDEYE_START, rid)
topic_for(Topic.CALIB_HANDEYE_PREVIEW, rid)

# ✅ After
key_for(Service.CALIB_HANDEYE_START, rid)
key_for(Topic.CALIB_HANDEYE_PREVIEW, rid)
```

**실 사용 분포** (rename 시점) — `topic_for(Service.X, ...)` 22회 vs `topic_for(Topic.X, ...)` 13회. **Service 자리에 더 많이 쓰임 → `topic_for` 는 misnomer**.

---

## 3. Migration plan + status

### Phase 1 — storage.py 전체 + `key_for` (완료)

- [x] `topic_for` → `key_for` rename ([topic_map.py](../backend/core/transport/topic_map.py) 정의 + 호출 site 35곳)
- [x] [messages/storage.py](../backend/core/transport/messages/storage.py) calibration sub-domain (21 class) — `Storage*` → `*Calibration*` / `*CalibrationRun*` / `*CalibrationCapture*`
- [x] [messages/storage.py](../backend/core/transport/messages/storage.py) scan workflow sub-domain (17 class) — `Storage*` → `*Scan*` / `*ScanSession*` / `*Reconstruction*` / `*Blob*`

### Phase 2 — 다른 노드 messages (예정)

각 노드 별 review — verb-first 가 이미 만족하는 자리는 noop, 어긋난 자리만 rename.

- [ ] [messages/motor.py](../backend/core/transport/messages/motor.py)
- [ ] [messages/motion.py](../backend/core/transport/messages/motion.py)
- [ ] [messages/detector.py](../backend/core/transport/messages/detector.py)
- [ ] [messages/scene3d.py](../backend/core/transport/messages/scene3d.py)
- [ ] [messages/camera.py](../backend/core/transport/messages/camera.py)
- [ ] [messages/reconstruction.py](../backend/core/transport/messages/reconstruction.py)
- [ ] [messages/calibration.py](../backend/core/transport/messages/calibration.py) `BeginRefinement*` 외 verb-first 적용 여부 검토 (§1.4 sub-domain prefix 예외 유지 결정)

---

## 4. 변경 적용 절차

1. **messages 파일**: class 정의 rename (Read → Edit)
2. **import site** 일괄 grep + 수정
   ```powershell
   grep -rn "StorageActivateReq" backend --include="*.py"
   ```
3. **frontend generated types** — `pnpm gen:types` 재생성 (backend `api_contract.py::custom_openapi` 가 자동 emit, frontend `generated/contract.ts` 자동 갱신)
4. **pyright** + **ruff check** + **테스트** 로 누락 검출
5. 단일 mechanical commit — `refactor: ...` 류 메시지

---

## 5. References

- [AIP-131: Standard methods: Get](https://google.aip.dev/131)
- [AIP-134: Standard methods: Update](https://google.aip.dev/134)
- [AIP-136: Custom methods](https://google.aip.dev/136)
- [AIP-191: File and directory structure](https://google.aip.dev/191)
- [Kubernetes API concepts](https://kubernetes.io/docs/reference/using-api/api-concepts/)
- [Zenoh Abstractions](https://zenoh.io/docs/manual/abstractions/)
- [PEP 8 — Style Guide for Python Code](https://peps.python.org/pep-0008/)
