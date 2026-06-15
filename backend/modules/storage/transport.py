"""Generic Zenoh transport for storage services — entity 어휘 0.

storage_node 가 노출하는 service / topic 을 호출하는 client-side 의 typed
envelope. 본 파일 자체는 어떤 entity (캘 / scan / TSDF / task_run) 도 모름.

entity-별 client 가 본 transport 위에 올라감 — `modules/calibration/storage_client.py`
의 `CalibrationStorageClient` 등. 미래 ScanStorageClient / TsdfStorageClient
등도 같은 transport 위.

docs/storage_layer.md §2 — Zenoh service gateway. 본 transport 는 그 client side.
"""

from __future__ import annotations

import logging
import time
from typing import Callable, TypeVar

import zenoh
from pydantic import BaseModel

from core.transport.messages.base import ServiceRequest, ServiceResponse
from core.transport.zenoh_session import ZenohSession

logger = logging.getLogger(__name__)

T = TypeVar("T", bound=BaseModel)
M = TypeVar("M", bound=BaseModel)


class StorageUnavailable(Exception):
    """storage_node 응답 timeout / error. caller 가 spill fallback 으로 우회."""


class StorageTransport:
    """Typed Zenoh service call + topic subscribe.

    BaseNode 의존 X — 그냥 ZenohSession 만 있으면 동작. 싱글톤 / dataclass /
    노드 어디서든 사용. 같은 process 든 분산 모드 든 동일 코드.
    """

    def __init__(self, session: zenoh.Session | None = None, timeout: float = 5.0):
        self._session = session or ZenohSession.get()
        self._timeout = timeout

    def call(self, key: str, req: BaseModel, res_cls: type[T]) -> T:
        """Typed service call. timeout / err reply / data=None 시 StorageUnavailable."""
        envelope = ServiceRequest(timestamp=time.time(), data=req)
        payload = envelope.model_dump_json(by_alias=True).encode()
        res_envelope_cls = ServiceResponse[res_cls]
        try:
            replies = self._session.get(key, payload=payload, timeout=self._timeout)
            for reply in replies:
                if reply.ok is not None:
                    env = res_envelope_cls.model_validate_json(
                        reply.ok.payload.to_bytes()
                    )
                    if not env.success:
                        raise StorageUnavailable(
                            f"{key} success=False: {env.message}"
                        )
                    if env.data is None:
                        raise StorageUnavailable(f"{key} data=None")
                    return env.data
                err = reply.err
                msg = (
                    err.payload.to_string()
                    if err is not None and err.payload is not None
                    else "err reply"
                )
                raise StorageUnavailable(f"{key} err: {msg}")
            raise StorageUnavailable(f"{key} 응답 없음 (timeout={self._timeout}s)")
        except StorageUnavailable:
            raise
        except Exception as e:
            raise StorageUnavailable(f"{key} 호출 오류: {e}") from e

    def subscribe_topic(
        self,
        topic: str,
        model_cls: type[M],
        callback: Callable[[M], None],
    ) -> zenoh.Subscriber:
        """Typed topic subscribe — caller 가 undeclare 책임 (보통 노드 lifecycle).

        validation 실패 시 callback 호출 X (drift 즉시 발견 가능 — log only).
        """

        def _handler(sample: zenoh.Sample) -> None:
            try:
                msg = model_cls.model_validate_json(sample.payload.to_bytes())
                callback(msg)
            except Exception as e:
                logger.error("topic %s 처리 오류: %s", topic, e)

        return self._session.declare_subscriber(topic, _handler)
