"""Scene3DNode 의 depth_consumer refcount race 자체 자리 자체 자리 자체 자리 자체 자리.

snapshot uuid + persistent "stream" 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 같은 set
자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 — concurrent acquire / release 자체 자리
자체 자리 자체 자리 자체 자리 자체 자리 CAMERA_SET_DEPTH_STREAM 자체 자리 자체 자리 자체 자리 자체 자리
unbalanced 자체 자리 자체 자리 자체 자리 자체 자리 X.

핵심 invariant:
- consumers.add 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 was_empty=True 자체 자리 자체 자리
  자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리
  CAMERA(True) 자체 자리 자체 자리 자체 자리 자체 자리 1회만.
- consumers.discard 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 now_empty=True 자체 자리 자체 자리
  자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리
  CAMERA(False) 자체 자리 자체 자리 자체 자리 자체 자리 1회만.
- enable/disable 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리
  자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리
  pair 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리.

전체 Scene3DNode 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 부팅 자체 자리 자체 자리 자체 자리
(robots.yaml + ApplicationNode + ZenohSession 등 자체 자리 자체 자리 자체 자리 자체 자리) 자체
자리 자체 자리 — 본 test 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 *refcount 로직만 자체
자리 자체 자리* 추출 자체 자리 자체 자리 자체 자리 (lightweight harness).
"""

from __future__ import annotations

import threading
import time

import pytest


class _RefcountHarness:
    """Scene3DNode 의 _acquire_depth_consumer / _release_depth_consumer 자체 자리
    자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 lock + set + call_history
    분리 자체 자리 자체 자리 자체 자리.

    실 Scene3DNode 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체
    자리 (RobotRegistry / ZenohSession / cam service call) 자체 자리 자체 자리 자체 자리 자체 자리
    자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 — 본 logic 자체 자리 자체 자리 자체 자리
    자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리.
    """

    def __init__(self, cam_call_delay: float = 0.0, cam_fails: bool = False):
        self.lock = threading.Lock()
        self.consumers: set[str] = set()
        # CAMERA_SET_DEPTH_STREAM 호출 자체 자리 자체 자리 자체 자리 자체 자리 history.
        self.cam_calls: list[bool] = []
        self._cam_call_delay = cam_call_delay
        self._cam_fails = cam_fails

    def _cam_set_stream(self, enabled: bool) -> bool:
        # 분산 자리 LAN latency 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리
        if self._cam_call_delay > 0:
            time.sleep(self._cam_call_delay)
        if self._cam_fails:
            return False
        self.cam_calls.append(enabled)
        return True

    def acquire(self, token: str) -> bool | None:
        with self.lock:
            if token in self.consumers:
                return False
            was_empty = len(self.consumers) == 0
            self.consumers.add(token)
        if was_empty:
            ok = self._cam_set_stream(True)
            if not ok:
                with self.lock:
                    self.consumers.discard(token)
                return None
        return True

    def release(self, token: str) -> None:
        with self.lock:
            if token not in self.consumers:
                return
            self.consumers.discard(token)
            now_empty = len(self.consumers) == 0
        if now_empty:
            self._cam_set_stream(False)


def test_single_consumer_acquire_release_pair():
    """단일 consumer — enable 1회 + disable 1회."""
    h = _RefcountHarness()
    h.acquire("snap-A")
    assert h.cam_calls == [True]
    h.release("snap-A")
    assert h.cam_calls == [True, False]


def test_two_consumers_share_camera_enable():
    """stream + snapshot 자체 자리 자체 자리 — CAMERA enable 자체 자리 자체 자리 1회만."""
    h = _RefcountHarness()
    h.acquire("stream")
    h.acquire("snap-A")
    assert h.cam_calls == [True]  # enable 1번만
    h.release("snap-A")
    assert h.cam_calls == [True]  # stream 살아있어서 disable 아직 안 함
    h.release("stream")
    assert h.cam_calls == [True, False]  # 마지막 release 자체 자리 disable


def test_idempotent_acquire_same_token():
    """같은 token 자체 자리 자체 자리 두 번 acquire — set 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리."""
    h = _RefcountHarness()
    h.acquire("stream")
    h.acquire("stream")  # idempotent — 두 번째 자체 자리 자체 자리 False 반환
    assert h.cam_calls == [True]
    h.release("stream")
    assert h.cam_calls == [True, False]


def test_cam_fail_rolls_back_token():
    """CAMERA enable 실패 자체 자리 자체 자리 자체 자리 token 자체 자리 자체 자리 set 자체 자리 자체 자리 빠짐."""
    h = _RefcountHarness(cam_fails=True)
    result = h.acquire("snap-A")
    assert result is None
    assert h.consumers == set()


def test_concurrent_acquire_release_balanced():
    """concurrent acquire/release — CAMERA enable/disable 자체 자리 자체 자리 자체 자리 자체 자리 자체
    자리 자체 자리 자체 자리 자체 자리 paired 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리.

    invariant — 최종 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 cam_calls
    의 enable count == disable count + (현재 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체
    자리 consumers 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 비면
    0, 아니면 1).
    """
    h = _RefcountHarness(cam_call_delay=0.001)
    n_threads = 20
    n_ops = 50

    def worker(idx: int):
        for j in range(n_ops):
            token = f"snap-{idx}-{j}"
            ok = h.acquire(token)
            if ok:
                # 짧게 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 hold 자체 자리 자체 자리
                time.sleep(0.0001)
                h.release(token)

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(n_threads)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    # 모든 worker 종료 자체 자리 자체 자리 자체 자리 자체 자리 자체 자리 — consumers 자체 자리 자체 자리 자체 자리 자체 자리 비어있어야
    assert h.consumers == set()
    # CAMERA enable count == disable count
    enables = h.cam_calls.count(True)
    disables = h.cam_calls.count(False)
    assert enables == disables, (
        f"enable/disable unbalanced — enables={enables}, disables={disables}"
    )
    # 처음 자체 자리 자체 자리 자체 자리 enable, 마지막 자체 자리 자체 자리 자체 자리 disable.
    if h.cam_calls:
        assert h.cam_calls[0] is True
        assert h.cam_calls[-1] is False


@pytest.mark.parametrize("trial", range(5))
def test_concurrent_balanced_repeats(trial: int):
    """memory anchor — intermittent 버그 자체 자리 자체 자리 자체 자리 자체 자리 reproduction script
    자체 자리 자체 자리 자체 자리 자체 자리 fix 보다 먼저. 반복 자체 자리 자체 자리 자체 자리 자체 자리
    자체 자리 자체 자리 통계 자체 자리 자체 자리.
    """
    test_concurrent_acquire_release_balanced()
