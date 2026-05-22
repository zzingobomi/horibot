import logging
import threading
import time
import json
from typing import Callable

import zenoh

from core.zenoh_session import ZenohSession
from core.topic_map import Topic

logger = logging.getLogger(__name__)


class BaseNode:
    def __init__(self, node_name: str):
        self.node_name = node_name
        self.session: zenoh.Session = ZenohSession.get()
        self._subscribers: list[zenoh.Subscriber] = []
        self._queryables: list[zenoh.Queryable] = []
        self._running = False
        self._heartbeat_thread: threading.Thread | None = None

    # ─── Subscriber ──────────────────────────────────────────

    def create_subscriber(self, topic: str, callback: Callable[[dict], None]) -> None:
        def _handler(sample: zenoh.Sample) -> None:
            try:
                data = json.loads(sample.payload.to_bytes())
                callback(data)
            except Exception as e:
                logger.error(f"[{self.node_name}] subscriber 처리 오류 ({topic}): {e}")

        sub = self.session.declare_subscriber(topic, _handler)
        self._subscribers.append(sub)
        logger.debug(f"[{self.node_name}] subscriber 등록: {topic}")

    def create_raw_subscriber(
        self, topic: str, callback: Callable[[bytes], None]
    ) -> None:
        def _handler(sample: zenoh.Sample) -> None:
            try:
                callback(sample.payload.to_bytes())
            except Exception as e:
                logger.error(f"[{self.node_name}] raw subscriber 오류 ({topic}): {e}")

        sub = self.session.declare_subscriber(topic, _handler)
        self._subscribers.append(sub)
        logger.debug(f"[{self.node_name}] raw subscriber 등록: {topic}")

    # ─── Service (Queryable / 서버) ──────────────────────────

    def create_service(self, key: str, handler: Callable[[dict], dict]) -> None:
        def _handler(query: zenoh.Query) -> None:
            try:
                payload = query.payload
                req = json.loads(payload.to_bytes()) if payload else {}
                res = handler(req)
                reply_payload = json.dumps(res).encode()
                query.reply(key, reply_payload)
            except Exception as e:
                logger.error(f"[{self.node_name}] service 처리 오류 ({key}): {e}")
                err = {"success": False, "message": str(e), "data": {}}
                query.reply(key, json.dumps(err).encode())

        queryable = self.session.declare_queryable(key, _handler)
        self._queryables.append(queryable)
        logger.debug(f"[{self.node_name}] service 등록: {key}")

    # ─── Service Client (Get / 클라이언트) ───────────────────

    def call_service(
        self,
        key: str,
        data: dict,
        timeout: float = 5.0,
    ) -> dict:
        """
        Zenoh Get으로 서비스를 호출하고 응답을 반환.

        Args:
            key:     서비스 키 (Service.* 상수)
            data:    요청 데이터 dict
            timeout: 응답 대기 시간 (초, 기본 5.0)

        Returns:
            {"success": bool, "message": str, "data": dict}
            타임아웃 또는 오류 시 success=False 반환.
        """
        payload = json.dumps(
            {
                "timestamp": time.time(),
                "data": data,
            }
        ).encode()

        try:
            replies = self.session.get(key, payload=payload, timeout=timeout)
            for reply in replies:
                # reply.ok 가 None이면 Zenoh 측에서 err reply를 보낸 것
                if reply.ok is not None:
                    return json.loads(reply.ok.payload.to_bytes())
                err = reply.err
                msg = (
                    err.payload.to_string()
                    if err is not None and err.payload is not None
                    else "서비스 err reply"
                )
                logger.warning(
                    f"[{self.node_name}] service err reply: {key} — {msg}"
                )
                return {"success": False, "message": msg, "data": {}}
            # 응답 없음 (빈 iterator) — queryable이 등록 안 됐거나 타임아웃
            logger.warning(f"[{self.node_name}] service 응답 없음: {key}")
            return {"success": False, "message": "응답 없음", "data": {}}
        except Exception as e:
            logger.error(f"[{self.node_name}] call_service 오류 ({key}): {e}")
            return {"success": False, "message": str(e), "data": {}}

    # ─── Lifecycle ───────────────────────────────────────────

    def start(self) -> None:
        self._running = True
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name=f"{self.node_name}-heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()
        logger.info(f"[{self.node_name}] 시작됨")

    def stop(self) -> None:
        self._running = False
        for sub in self._subscribers:
            sub.undeclare()
        for q in self._queryables:
            q.undeclare()
        self._subscribers.clear()
        self._queryables.clear()
        logger.info(f"[{self.node_name}] 종료됨")

    def spin(self) -> None:
        """노드를 블로킹으로 실행. 스레드에서 호출할 것."""
        self.start()
        try:
            while self._running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    # ─── Publisher ───────────────────────────────────────────

    def publish(self, topic: str, data: dict) -> None:
        payload = json.dumps(data).encode()
        self.session.put(topic, payload)

    # ─── Heartbeat ───────────────────────────────────────────

    def _heartbeat_loop(self) -> None:
        while self._running:
            self.publish(
                Topic.SYSTEM_HEARTBEAT,
                {
                    "node": self.node_name,
                    "timestamp": time.time(),
                    "status": "ok",
                },
            )
            time.sleep(1.0)

    # ─── Log ─────────────────────────────────────────────────

    def log(self, level: str, msg: str) -> None:
        self.publish(
            Topic.SYSTEM_LOG,
            {
                "node": self.node_name,
                "timestamp": time.time(),
                "level": level,
                "message": msg,
            },
        )
        getattr(logger, level, logger.info)(f"[{self.node_name}] {msg}")
