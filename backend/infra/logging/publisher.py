"""분산 로그 발행 핸들러 — 런타임 인프라 (모듈 아님, 모든 host).

Runtime 부팅 시 root logger 에 부착한다. 콘솔 핸들러와 **함께** 붙어서, 그 host 의
모든 모듈 로그를 콘솔에 찍는 것과 동시에 같은 줄을 Zenoh 키 `log/{host}` 로 발행한다.
중앙 PC 의 LogCollector 가 `log/**` 를 구독해 한 파일에 모은다 (docs/logging.md).

**설계 (docs/logging.md §2/§10)**: 로그 줄은 발행 시점에 **자기완결** — host 를 포함해
ts/level/logger/pid/thread 가 전부 줄 안에 있다. 파일에서 한 줄만 떼어 읽어도 어느 host
에서 왔는지 안다. wire 키 `log/{host}` 는 라우팅 전용 (collector 는 파싱 안 함).
host 는 발행자(runtime `--host`)가 권위 있게 알므로 여기서 각인한다.
"""

from __future__ import annotations

import logging
import threading

from framework.transport.protocol import RawTransport

# host 를 각인한 자기완결 포맷. 콘솔(basicConfig)과 달리 [{host}] + pid/thread 추가.
# host 는 핸들러 인스턴스마다 고정이라 포맷 문자열에 바로 박는다.
_LOG_FORMAT = (
    "%(asctime)s.%(msecs)03d [{host}] [%(levelname)s] %(name)s "
    "(pid=%(process)d %(threadName)s): %(message)s"
)
_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

# 발행 경로 자신이 내는 로그는 제외 — 안 그러면 발행→로그→발행 무한재귀.
# (reentrancy 가드로도 막지만, transport 로거는 애초에 발행 대상에서 뺀다.)
_EXCLUDED_LOGGER_PREFIX = "infra.transport.zenoh"


class ZenohLogPublisher(logging.Handler):
    """포맷된 로그 줄을 Zenoh `log/{host}` 로 발행하는 logging 핸들러.

    가드 (docs/logging.md §6 L1-a):
    - emit() 은 절대 예외 전파 안 함 — 발행 실패해도 앱 무사.
    - reentrancy 가드 — 발행 경로가 다시 로그를 내도 무한재귀 안 함.
    - transport 로거 제외 — 재귀 원인 차단.
    """

    def __init__(self, transport: RawTransport, host: str) -> None:
        super().__init__()
        self._transport = transport
        self._key = f"log/{host}"
        self._local = threading.local()
        self.setFormatter(
            logging.Formatter(_LOG_FORMAT.format(host=host), datefmt=_DATE_FORMAT)
        )
        self.addFilter(_exclude_transport_logger)

    def emit(self, record: logging.LogRecord) -> None:
        # reentrancy 가드 — 발행 중 발생한 로그는 무시 (같은 스레드 재진입).
        if getattr(self._local, "in_emit", False):
            return
        self._local.in_emit = True
        try:
            line = self.format(record)
            self._transport.publish(self._key, line.encode("utf-8"))
        except Exception:
            # 발행 실패는 앱에 전파하지 않는다 (로깅이 앱을 죽이면 안 됨).
            pass
        finally:
            self._local.in_emit = False


def _exclude_transport_logger(record: logging.LogRecord) -> bool:
    return not record.name.startswith(_EXCLUDED_LOGGER_PREFIX)


def attach_log_publisher(transport: RawTransport, host: str) -> ZenohLogPublisher:
    """root logger 에 발행 핸들러를 부착하고 그 핸들러를 반환한다.

    transport 세션이 준비된 뒤(ZenohTransport 생성 후) 호출해야 한다.
    반환값은 종료 시 detach 에 넘긴다.
    """
    handler = ZenohLogPublisher(transport, host)
    logging.getLogger().addHandler(handler)
    return handler


def detach_log_publisher(handler: ZenohLogPublisher) -> None:
    """발행 핸들러를 root logger 에서 제거한다 (transport.close 전에 호출)."""
    logging.getLogger().removeHandler(handler)
    handler.close()
