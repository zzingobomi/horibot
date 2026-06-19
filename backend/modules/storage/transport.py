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
    """storage_node 응답 timeout / error 발생 시 raise."""


class StorageTransport:
    """
    BaseNode 의존 X — Storage를 호출하는 주체가 Node만 있는 게 아니라 CalibrationCache 같은 일반 객체도 있기 때문에
    """

    def __init__(self, session: zenoh.Session | None = None, timeout: float = 5.0):
        self._session = session or ZenohSession.get()
        self._timeout = timeout

    def call(self, key: str, req: BaseModel, res_cls: type[T]) -> T:
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
                        raise StorageUnavailable(f"{key} success=False: {env.message}")
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
        def _handler(sample: zenoh.Sample) -> None:
            try:
                msg = model_cls.model_validate_json(sample.payload.to_bytes())
                callback(msg)
            except Exception as e:
                logger.error("topic %s 처리 오류: %s", topic, e)

        return self._session.declare_subscriber(topic, _handler)
