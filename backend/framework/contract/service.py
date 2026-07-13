from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Any, Callable, Mapping, get_type_hints

from pydantic import BaseModel


@dataclass(frozen=True)
class ServiceSpec:
    method_name: str
    wire_key: str
    req_cls: type[BaseModel]
    res_cls: type[BaseModel]
    handler: Callable[..., Any]


_SERVICE_ATTR = "_service_spec"

# ─── 서비스 기본 timeout (호출측) ─────────────────────────────────────
#
# timeout 은 서비스의 성질 (MoveL 은 60s 급, snapshot 은 5s 급) — 호출부마다
# 아는 게 아니라 그 서비스의 contract.py 가 선언한다. runtime.call 이
# timeout 미지정 호출에서 이 registry 를 참조 (선언 없으면 DEFAULT).
# 키 = wire 키 template ({robot_id} 미확장) — contract 의 StrEnum 값 그대로.

DEFAULT_SERVICE_TIMEOUT_S = 5.0

_SERVICE_TIMEOUTS: dict[str, float] = {}


def declare_service_timeouts(mapping: Mapping[str, float]) -> None:
    """contract.py 파일 하단에서 호출 — 자기 모듈 서비스의 기본 timeout 선언.

    같은 키 재선언은 프로그래밍 오류 (fail-fast — 두 곳이 서로 다른 값을
    주장하면 어느 쪽이 이겼는지 침묵으로 갈리는 사고 차단).
    """
    for key, seconds in mapping.items():
        key_str = str(key)
        if key_str in _SERVICE_TIMEOUTS and _SERVICE_TIMEOUTS[key_str] != float(seconds):
            raise ValueError(
                f"service timeout 중복 선언: {key_str} "
                f"({_SERVICE_TIMEOUTS[key_str]}s vs {seconds}s)"
            )
        _SERVICE_TIMEOUTS[key_str] = float(seconds)


def resolve_service_timeout(key: str, timeout: float | None) -> float:
    """명시 timeout > contract 선언 > DEFAULT — runtime.call 이 사용."""
    if timeout is not None:
        return timeout
    return _SERVICE_TIMEOUTS.get(str(key), DEFAULT_SERVICE_TIMEOUT_S)


def service(wire_key: str) -> Callable[[Callable[..., Any]], Callable[..., Any]]:

    key_str = str(wire_key)

    def decorator(method: Callable[..., Any]) -> Callable[..., Any]:
        hints = get_type_hints(method)
        sig = inspect.signature(method)
        params = [p for p in sig.parameters.values() if p.name != "self"]

        if len(params) != 1:
            raise TypeError(
                f"@service '{method.__name__}': self + req 한 parameter 필요 "
                f"(got {len(params)} parameter)"
            )

        req_name = params[0].name
        req_cls = hints.get(req_name)
        res_cls = hints.get("return")

        if (
            req_cls is None
            or not isinstance(req_cls, type)
            or not issubclass(req_cls, BaseModel)
        ):
            raise TypeError(
                f"@service '{method.__name__}': req parameter '{req_name}' 의 "
                f"type hint 가 BaseModel subclass 여야 함 (got {req_cls})"
            )
        if (
            res_cls is None
            or not isinstance(res_cls, type)
            or not issubclass(res_cls, BaseModel)
        ):
            raise TypeError(
                f"@service '{method.__name__}': return type hint 가 "
                f"BaseModel subclass 여야 함 (got {res_cls})"
            )

        spec = ServiceSpec(
            method_name=method.__name__,
            wire_key=key_str,
            req_cls=req_cls,
            res_cls=res_cls,
            handler=method,
        )
        setattr(method, _SERVICE_ATTR, spec)
        return method

    return decorator


def is_service(method: Any) -> bool:
    return hasattr(method, _SERVICE_ATTR)


def get_service_spec(method: Any) -> ServiceSpec | None:
    return getattr(method, _SERVICE_ATTR, None)
