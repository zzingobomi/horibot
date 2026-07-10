from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Any, Callable, get_type_hints

from pydantic import BaseModel


@dataclass(frozen=True)
class ServiceSpec:
    method_name: str
    wire_key: str
    req_cls: type[BaseModel]
    res_cls: type[BaseModel]
    handler: Callable[..., Any]


_SERVICE_ATTR = "_service_spec"


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
