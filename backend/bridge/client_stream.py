import asyncio
import logging

from fastapi import WebSocket

from core.transport.topic_map import Topic

logger = logging.getLogger(__name__)


class StreamPolicy:
    LATEST_WINS = "latest"  # 단일 슬롯, 새 값이 옛 값을 덮어씀
    BOUNDED_FIFO = "fifo"  # N개까지 보존, full이면 oldest 드롭


_DEFAULT_POLICY: tuple[str, int] = (StreamPolicy.LATEST_WINS, 1)
_TOPIC_POLICIES: dict[str, tuple[str, int]] = {
    # 로그는 누락되면 디버깅이 어려워지므로 FIFO로 일정량 보존
    Topic.SYSTEM_LOG: (StreamPolicy.BOUNDED_FIFO, 128),
}


def _policy_for(topic: str) -> tuple[str, int]:
    return _TOPIC_POLICIES.get(topic, _DEFAULT_POLICY)


class ClientStream:
    def __init__(
        self, ws: WebSocket, topic: str, send_lock: asyncio.Lock
    ) -> None:
        self.ws = ws
        self.topic = topic
        self.send_lock = send_lock
        self.policy, maxsize = _policy_for(topic)
        self.queue: asyncio.Queue = asyncio.Queue(maxsize=maxsize)
        self.task: asyncio.Task = asyncio.create_task(self._run())

    def put(self, payload: bytes | str) -> None:
        try:
            self.queue.put_nowait(payload)
        except asyncio.QueueFull:
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            try:
                self.queue.put_nowait(payload)
            except asyncio.QueueFull:
                pass  # 이론상 도달 불가 (단일 스레드)

    # 여러 topic stream task가 동일 WebSocket에 동시에 send하지 않도록
    # 실제 ws.send_* 호출은 send_lock으로 직렬화한다.
    # pcd stream 과 motor stream이 동시에 send 할때 assertion error 발생했었음
    async def _run(self) -> None:
        while True:
            try:
                payload = await self.queue.get()
                async with self.send_lock:
                    if isinstance(payload, bytes):
                        await self.ws.send_bytes(payload)
                    else:
                        await self.ws.send_text(payload)
            except asyncio.CancelledError:
                return
            except Exception as e:
                logger.warning(
                    f"ClientStream({self.topic}) send 실패, 계속 진행: {e!r}"
                )

    def close(self) -> None:
        self.task.cancel()


class ConnectionManager:
    def __init__(self):
        self._streams: dict[WebSocket, dict[str, ClientStream]] = {}
        self._send_locks: dict[WebSocket, asyncio.Lock] = {}

    def subscribe(self, ws: WebSocket, topic: str) -> None:
        client = self._streams.setdefault(ws, {})
        lock = self._send_locks.setdefault(ws, asyncio.Lock())
        if topic not in client:
            client[topic] = ClientStream(ws, topic, lock)

    def unsubscribe(self, ws: WebSocket, topic: str) -> None:
        client = self._streams.get(ws)
        if not client:
            return
        stream = client.pop(topic, None)
        if stream is not None:
            stream.close()

    def remove_client(self, ws: WebSocket) -> None:
        client = self._streams.pop(ws, None)
        self._send_locks.pop(ws, None)
        if not client:
            return
        for stream in client.values():
            stream.close()

    def fanout(self, topic: str, payload: bytes | str) -> None:
        """이벤트 루프 스레드에서 호출. 토픽 구독자들의 큐에 push."""
        for client in self._streams.values():
            stream = client.get(topic)
            if stream is not None:
                stream.put(payload)
