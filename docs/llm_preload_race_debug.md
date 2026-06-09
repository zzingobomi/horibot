# LLM Preload Meta-Tensor Race — 진단 및 fix plan

> **Status**: 가설 검증 전. fix 적용 전. 본 문서는 *진단 박제* 가 목적 — 코드 깔짝
> 거리며 "이번엔 됐다" 식 fix 반복 (10회+) 을 멈추기 위한 anchor.

---

## 1. 증상

LLM (`Qwen2.5-1.5B-Instruct`) preload 시 다음 에러가 *간헐적으로* 발생:

```
NotImplementedError: Cannot copy out of meta tensor; no data!
Please use torch.nn.Module.to_empty() instead of torch.nn.Module.to()
when moving module from meta to a different device.
```

발생 위치: [backend/modules/llm/prompt_parser.py:80](../backend/modules/llm/prompt_parser.py#L80) `_ensure_loaded` 안의 `.to(_device)` 호출.

**핵심 특성: intermittent**.
- 같은 코드, 같은 환경인데 어떤 날엔 발생, 어떤 날엔 안 함
- "fix 후 한두 번 안 나오면 fixed 라 판단" → 며칠 뒤 재발 → 다시 깔짝 fix → ...
- 지금까지 **10회+ 반복**

LLM 로드 실패 후에도 Grounding DINO 는 1-2초 뒤 정상 로드됨. 즉 시스템 자체는 동작하나 task prompt parser 가 죽어 prompt 가 그대로 pick 으로 fallback ([prompt_parser.py:106](../backend/modules/llm/prompt_parser.py#L106)).

---

## 2. 지금까지의 fix 시도 (모두 부분 fix)

| # | 시도 | 잡은 layer | 왜 부족했나 |
|---|---|---|---|
| 1 | transformers 함수 안 lazy import → **module-top import** | transformers `_LazyModule.__getattr__` race (두 스레드가 동시에 `from transformers import X` 시 "cannot import name" 발생) | import race 만 잡음. weight load race 는 그대로 |
| 2 | `from_pretrained(..., low_cpu_mem_usage=False)` | meta-init path 회피 (내 모델만) | accelerate 글로벌 state 가 *상대 스레드* 의 영향 받음 — 내 모델은 False 라도 상대가 meta-init 중이면 leak |

각 fix 가 race window 를 줄이긴 했지만 **원인 자체** 를 안 잡음 → 며칠 후 재발. 패턴 자체가 진단 미완 + 검증 단계 부재.

---

## 3. 현재 가설 — 두 preload thread race

### 증거

마지막 로그 (2026-06-09):
```
04:45:55,022 [ERROR] LLM 모델 로드 실패: Cannot copy out of meta tensor
04:45:56,586 [INFO]  Grounding DINO 로드 완료
```

→ **1.5초 차이로 두 모델이 같은 시간 window 에서 로드 시도** 중.

코드상 두 백그라운드 preload 스레드가 노드 start 시 동시 출발:

- [task_node.py:116-120](../backend/nodes/application/task_node.py#L116-L120) — `prompt-parser-preload` thread → `prompt_parser.preload()` → `AutoModelForCausalLM.from_pretrained(...).to(device)`
- [detector_node.py:122-127](../backend/nodes/application/detector_node.py#L122-L127) — `grounded-preload` thread → `GroundedDetector.preload()` → `AutoModelForZeroShotObjectDetection.from_pretrained(...).to(device)`

### 메커니즘

transformers / accelerate 의 weight 초기화 흐름 (`init_empty_weights()` context manager, `_init_weights`, `dispatch_model`) 이 **thread-safe 하지 않음**. 두 스레드가 동시에 `from_pretrained` 진입하면:

1. 스레드 A 가 `init_empty_weights()` 컨텍스트에 enter (글로벌 상태 변경)
2. 스레드 B 가 `from_pretrained` 진입 — A 의 컨텍스트가 활성 상태로 보임
3. B 가 `low_cpu_mem_usage=False` 라도 A 의 context 가 활성이라 일부 weight 가 meta 인 채로 leak
4. B 의 `.to(device)` 단계에서 meta tensor 만나 NotImplementedError

intermittent 한 이유: A 가 context exit 하기 *전에* B 가 진입하느냐, *후에* 진입하느냐 의 단순 타이밍. CPU 부하 / disk cache 상태 / network 다운로드 속도에 따라 매번 달라짐 → **race condition 의 fingerprint**.

### 가설이 틀릴 가능성

- 단일 모델 (LLM 만) preload 만 돌려도 재발한다면 — 두 스레드 race 아님. transformers 자체 버그 / 환경 문제일 수 있음
- transformers / accelerate 버전 업그레이드 후 사라진다면 — 우연이 아니라 라이브러리 측 fix
- GPU 메모리 부족으로 인한 다른 path 일 수도 (현재 RTX 3060)

→ **검증 없이는 가설 확정 불가**. 다음 섹션이 핵심.

---

## 4. 가설 검증이 fix 보다 먼저

지금까지의 fix 가 모두 실패한 진짜 이유: **"오늘 안 나옴 = fix 됨" 으로 판단**. race condition 은 운으로 안 나올 뿐이라 이 기준이 무의미.

검증 방법:

### 4.1 reproduction script

위치: `backend/scripts/repro_meta_tensor_race.py` (新)

핵심 골격:
```python
# 두 from_pretrained 를 동시 호출. 노드 / Zenoh / bridge 다 빼고
# 순수하게 race 만 노출.
import threading, traceback, sys
from transformers import (
    AutoModelForCausalLM, AutoModelForZeroShotObjectDetection,
)

def load_llm():
    AutoModelForCausalLM.from_pretrained("Qwen/Qwen2.5-1.5B-Instruct").to("cuda")

def load_dino():
    AutoModelForZeroShotObjectDetection.from_pretrained(
        "IDEA-Research/grounding-dino-base"
    ).to("cuda")

errors = []
def one_trial():
    t1 = threading.Thread(target=lambda: _capture(load_llm, errors))
    t2 = threading.Thread(target=lambda: _capture(load_dino, errors))
    t1.start(); t2.start(); t1.join(); t2.join()

# N=100 회 반복, 실패 카운트
```

### 4.2 가설 검증 절차

1. **fix 전 N=100 회 실행** — race 가설 맞으면 ≥1 회 같은 NotImplementedError 재현
2. **재현 안 되면 (0/100)** → race 가설 폐기. 다른 가설로 재출발 (transformers 버전 / GPU 메모리 / disk cache 등)
3. **재현 되면 (예: 23/100)** → race 가 root cause 확정. fix 단계 진입
4. **fix 적용 → 같은 script N=100 회 재실행** → 0/100 입증
5. **0/100 안 되면 fix 미완** — 가설은 맞지만 fix 가 race 의 일부만 잡았다는 신호. 다음 fix 옵션으로

이러면 "다음에 또 나오면 어쩌나" 의 답이 명확함 — 같은 script 가 즉시 재현. "운으로 안 나옴" 과 "구조적으로 안 나옴" 의 구분 가능.

---

## 5. fix 옵션 (4.2 통과 후 적용)

### Option A: 글로벌 lock

`backend/modules/common/transformers_load_lock.py` (新):
```python
import threading
LOAD_LOCK = threading.Lock()
```

prompt_parser / grounded_detector 둘 다 `with LOAD_LOCK:` 안에서 `from_pretrained` 호출.

- 변경 4-6 줄
- *단점*: 새 transformers 사용처가 추가될 때 lock 빠뜨릴 수 있음 → 같은 버그 재발 가능

### Option B (선호): 단일 preload coordinator

[main.py](../backend/main.py) 에 백그라운드 thread 하나만 띄워 두 preload 를 **순차** 호출. [task_node.py:116](../backend/nodes/application/task_node.py#L116) / [detector_node.py:122](../backend/nodes/application/detector_node.py#L122) 의 `_preload_*` thread 삭제.

```python
# main.py 노드들 start() 후
threading.Thread(target=_preload_all, daemon=True, name="model-preload").start()

def _preload_all():
    # 순차 — 동시 실행 가능성 자체가 코드 구조상 없음
    if "detector" in active_nodes:
        detector_node._grounded.preload()
    if "task" in active_nodes:
        prompt_parser.preload()
```

- 두 `from_pretrained` 가 **동시 실행될 가능성 자체가 코드 구조상 존재 안 함**
- lock 누락 가능성 없음
- 각 노드의 `_ensure_loaded` 는 그대로 유지 — 사용자가 preload 안 기다리고 첫 호출 박았을 때 fallback path 로 살아 있음
- *단점*: main.py 가 노드 내부 모듈 (`prompt_parser`, `_grounded`) 직접 호출 — 의존성 방향 살짝 거꾸로. 다만 main.py 는 어차피 노드 lazy import 코디네이터라 큰 위반은 아님

### 선택 기준

- race 재현이 두 모델 동시 호출 케이스만 잡는다면 — **Option B**
- 단일 모델만으로도 재현된다면 — race 가설 자체가 부분적. lock 도 부족. 다른 root cause 탐색

---

## 6. 회귀 방지

- reproduction script 를 `backend/scripts/` 에 영구 보관 (gitignore X)
- 재발 신호 (LLM 모델 로드 실패 로그) 보이면 → 같은 script 가 즉시 재현 도구로 동작 → 또 깔짝 fix 안 함
- transformers / torch / accelerate 버전 업그레이드 시에도 회귀 체크 가능 — `uv sync` 후 script 1회 실행을 권장 절차에 포함

---

## 7. 진행 체크리스트

- [ ] reproduction script 작성 (`backend/scripts/repro_meta_tensor_race.py`)
- [ ] 현재 코드로 script N=100 실행 → race 재현률 측정
- [ ] 재현률 0 이면 → 본 문서의 §3 가설 폐기, §3 의 "가설이 틀릴 가능성" 항목들로 재출발
- [ ] 재현률 ≥1 이면 → Option B 적용
- [ ] Option B 후 N=100 재실행 → 0/100 확인
- [ ] 0/100 안 되면 → 본 문서 §5 보강 (lock + coordinator 병행 등)
- [ ] script 를 repo 에 영구 보관 + README 한 줄
- [ ] 본 문서 §3 / §5 에 *실제 측정 결과* 추가 후 docs/roadmap.md 에 closed 처리

---

## 8. 이 문서의 의도

10 회 깔짝 fix 의 패턴은:
1. 에러 발생 → 가설 즉흥 (보통 직전 fix 의 인접 layer)
2. 코드 한두 줄 수정 → 재시작 → 안 나오면 fixed 판단
3. 며칠 뒤 같은 에러 재발 → 1 로 복귀

본 문서는 이 cycle 을 *재현 script 가 통계적으로 fix 입증할 때까지* 멈추는 게 목적.
"수정했는데 지금은 괜찮다가 다음에 또 나오면 어떻게 할래" — 답: 같은 script 가
즉시 재현. 가설 다시. fix 다시. 검증 다시. 운에 안 맡김.
