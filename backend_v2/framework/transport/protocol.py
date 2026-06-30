"""프레임워크의 통신 인터페이스.

모든 Module은 Transport만 의존하며,
실제 통신 방식(Zenoh 등)은 구현체가 담당한다.

Transport는 bytes 전송만 책임진다.
직렬화/역직렬화는 상위 레이어의 책임이다.
"""

from __future__ import annotations

from typing import Callable, Protocol, runtime_checkable


@runtime_checkable
class Handle(Protocol):
    """등록된 리소스를 해제할 수 있는 핸들."""

    def undeclare(self) -> None: ...


class RemoteError(Exception):
    """원격 서비스에서 발생한 예외.

    예외 객체 자체는 전송할 수 없으므로
    예외 타입 이름과 메시지만 전달한다.
    """

    def __init__(self, type_name: str, message: str):
        self.type_name = type_name
        self.message = message
        super().__init__(f"{type_name}: {message}")


@runtime_checkable
class RawTransport(Protocol):
    """Boundary Module (Bridge) 전용 — raw bytes relay 최소 권한.

    Transport 의 부분집합. `close()` (공유 세션을 닫으면 프로세스 전체 transport
    사망) 와 `register_service()` (Bridge 는 service 제공자 X, 호출자 only) 를
    의도적으로 뺀다. Bridge 가 실제 쓰는 call / publish / subscribe 만 노출.
    같은 Transport 구현이 구조적으로 이를 만족 — 주입은 동일 객체, 타입만 좁힘.
    """

    async def call(self, key: str, payload: bytes, timeout: float = 5.0) -> bytes:
        ...

    def publish(self, key: str, payload: bytes) -> None:
        ...

    def subscribe(self, key: str, callback: Callable[[bytes], None]) -> Handle:
        ...


@runtime_checkable
class Transport(Protocol):
    """모든 Transport 구현이 제공해야 하는 인터페이스."""

    async def call(self, key: str, payload: bytes, timeout: float = 5.0) -> bytes:
        """원격 서비스 호출.

        Args:
            key: 서비스 키
            payload: 요청 데이터
            timeout: 응답 대기 시간(초)

        Returns:
            서비스 응답 데이터

        Raises:
            RemoteError: 원격 서비스에서 예외 발생
            TimeoutError: 응답 시간 초과
        """
        ...

    def publish(self, key: str, payload: bytes) -> None:
        """토픽에 메시지 발행."""
        ...

    def register_service(self, key: str, handler: Callable[[bytes], bytes]) -> Handle:
        """서비스 핸들러 등록.

        등록된 서비스는 call()을 통해 호출될 수 있다.

        Returns:
            등록 해제에 사용할 Handle
        """
        ...

    def subscribe(self, key: str, callback: Callable[[bytes], None]) -> Handle:
        """토픽 구독.

        메시지가 수신되면 callback이 호출된다.

        Returns:
            구독 해제에 사용할 Handle
        """
        ...

    def close(self) -> None:
        """Transport를 종료하고 모든 리소스를 정리한다."""
        ...
