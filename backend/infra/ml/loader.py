"""transformers 모델 로드 직렬화 — process-wide lock.

여러 module 어댑터가 백그라운드 preload thread 에서 동시에 `from_pretrained` 를
부르면 transformers/accelerate 의 weight init (`init_empty_weights` / dispatch) 이
thread-safe 하지 않아 meta-tensor race 가 난다 ("Cannot copy out of meta tensor";
[docs/llm_preload_race_debug.md] — v1 에서 10회+ 땜빵 반복한 trauma).

현재 소비자는 detector 의 GDINO 하나뿐이라 경합이 없지만, NL PnP 로 Qwen LLM
(prompt_parser) 이 v2 에 재등장하면 두 번째 transformers 소비자가 된다. 그때
재-땜빵하지 않도록 **모든 transformers 로드를 이 공유 lock 으로 직렬화** — 두
어댑터가 같은 lock 을 잡으면 동시 from_pretrained 자체가 불가능해져 race 가
구조적으로 사라진다 (버전 핀이 아니라 로드 방식이 fix).

사용:
    from infra.ml.loader import transformers_load_lock

    with transformers_load_lock:
        model = AutoModel...from_pretrained(...)

주의: 이건 로드 *직렬화* seam 이다. 두 번째 소비자(Qwen) 실제 포팅 시점의 완전한
검증은 reproduction-script-first (docs/llm_preload_race_debug.md 프로토콜) 로 한다.
"""

from __future__ import annotations

import threading

# process-wide — module import 는 프로세스당 1회라 단일 인스턴스가 보장된다.
transformers_load_lock = threading.Lock()
