from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Callable

import zenoh

from framework.transport.protocol import RemoteError

logger = logging.getLogger(__name__)


class ZenohTransport:
    def __init__(self, config: dict[str, Any] | None = None):
        self._session: zenoh.Session = zenoh.open(self._build_config(config))
        logger.info("ZenohTransport open (cfg=%s)", config or "default")

    # ─── Transport surface ────────────────────────────────────

    async def call(self, key: str, payload: bytes, timeout: float = 5.0) -> bytes:
        return await asyncio.to_thread(self._call_sync, key, payload, timeout)

    def publish(self, key: str, payload: bytes) -> None:
        self._session.put(key, payload)

    def register_service(
        self, key: str, handler: Callable[[bytes], bytes]
    ) -> zenoh.Queryable:
        def _on_query(query: zenoh.Query) -> None:
            req_bytes = query.payload.to_bytes() if query.payload is not None else b""
            try:
                res_bytes = handler(req_bytes)
            except Exception as e:
                err = {"type": type(e).__name__, "message": str(e)}
                try:
                    query.reply_err(json.dumps(err).encode())
                except Exception as reply_err:
                    logger.error("reply_err 실패 (%s): %s", key, reply_err)
                logger.debug(
                    "service handler exception (%s): %s: %s",
                    key,
                    type(e).__name__,
                    e,
                )
                return

            try:
                query.reply(key, res_bytes)
            except Exception as reply_err:
                logger.error("reply 실패 (%s): %s", key, reply_err)

        return self._session.declare_queryable(key, _on_query)

    def subscribe(
        self, key: str, callback: Callable[[bytes], None]
    ) -> zenoh.Subscriber:
        def _on_sample(sample: zenoh.Sample) -> None:
            try:
                callback(sample.payload.to_bytes())
            except Exception as e:
                logger.error(
                    "subscriber callback exception (%s): %s: %s",
                    key,
                    type(e).__name__,
                    e,
                )

        return self._session.declare_subscriber(key, _on_sample)

    def declare_liveliness(self, key: str) -> "zenoh.LivelinessToken":
        return self._session.liveliness().declare_token(key)

    def subscribe_liveliness(
        self, key_expr: str, callback: Callable[[str, bool], None]
    ) -> "zenoh.Subscriber":
        def _on_sample(sample: zenoh.Sample) -> None:
            alive = sample.kind == zenoh.SampleKind.PUT
            try:
                callback(str(sample.key_expr), alive)
            except Exception as e:
                logger.error(
                    "liveliness callback exception (%s): %s: %s",
                    key_expr,
                    type(e).__name__,
                    e,
                )

        return self._session.liveliness().declare_subscriber(
            key_expr, _on_sample, history=True
        )

    def close(self) -> None:
        self._session.close()
        logger.info("ZenohTransport closed")

    # ─── Internal ─────────────────────────────────────────────

    def _call_sync(self, key: str, payload: bytes, timeout: float) -> bytes:
        replies = self._session.get(key, payload=payload, timeout=timeout)
        for reply in replies:
            if reply.ok is not None:
                return reply.ok.payload.to_bytes()
            err = reply.err
            if err is not None and err.payload is not None:
                err_bytes = err.payload.to_bytes()
                # zenoh 자체 timeout 신호 = 평문 b"Timeout" (우리 reply_err 는 항상
                # JSON) — RemoteError("Unknown", "Timeout") 로 둔갑하지 않게 정규화.
                if err_bytes == b"Timeout":
                    raise TimeoutError(f"service {key} 응답 없음 (timeout={timeout}s)")
                raise self._decode_err(err_bytes)
            raise RemoteError(type_name="Unknown", message="empty err reply")
        raise TimeoutError(f"service {key} 응답 없음 (timeout={timeout}s)")

    @staticmethod
    def _decode_err(err_bytes: bytes) -> RemoteError:
        try:
            info = json.loads(err_bytes)
            return RemoteError(
                type_name=info.get("type", "Unknown"),
                message=info.get("message", ""),
            )
        except (json.JSONDecodeError, UnicodeDecodeError):
            return RemoteError(
                type_name="Unknown",
                message=err_bytes.decode("utf-8", errors="replace"),
            )

    @staticmethod
    def _build_config(cfg: dict[str, Any] | None) -> zenoh.Config:
        z_cfg = zenoh.Config()
        if not cfg:
            return z_cfg
        mode = cfg.get("mode")
        if mode:
            z_cfg.insert_json5("mode", json.dumps(mode))
        scouting = cfg.get("scouting") or {}
        multicast = scouting.get("multicast") or {}
        if "enabled" in multicast:
            z_cfg.insert_json5(
                "scouting/multicast/enabled",
                json.dumps(bool(multicast["enabled"])),
            )
        connect = cfg.get("connect") or []
        if connect:
            z_cfg.insert_json5("connect/endpoints", json.dumps(list(connect)))
        listen = cfg.get("listen") or []
        if listen:
            z_cfg.insert_json5("listen/endpoints", json.dumps(list(listen)))
        return z_cfg
