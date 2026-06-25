"""@service 데코 — service handler 박힌 method 의 contract spec 추출.

사용 (spec §3.1):
    class CalibrationModule:
        @service
        def activate(self, req: ActivateRequest) -> ActivateResponse:
            ...

framework 가 type hint 에서 자동:
- req parameter 의 annotation → req_cls
- return annotation → res_cls

method 자체에 `_service_spec` attribute 박힘. Runtime (Step 3) 이 Module instance 의
메소드 inspect → spec 발견 → transport.register_service wiring.
"""

from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Any, Callable, get_type_hints

from pydantic import BaseModel


@dataclass(frozen=True)
class ServiceSpec:
    """@service 데코가 박힌 method 의 contract spec."""

    method_name: str
    req_cls: type[BaseModel]
    res_cls: type[BaseModel]
    handler: Callable[..., Any]


_SERVICE_ATTR = "_service_spec"


def service(method: Callable[..., Any]) -> Callable[..., Any]:
    """method 박힌 type hint 에서 req_cls / res_cls 자동 추출.

    제약:
    - method 시그니처 = `(self, req: ReqCls) -> ResCls`
    - ReqCls / ResCls 둘 다 Pydantic BaseModel subclass
    """
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
        req_cls=req_cls,
        res_cls=res_cls,
        handler=method,
    )
    setattr(method, _SERVICE_ATTR, spec)
    return method


def is_service(method: Any) -> bool:
    return hasattr(method, _SERVICE_ATTR)


def get_service_spec(method: Any) -> ServiceSpec | None:
    return getattr(method, _SERVICE_ATTR, None)
