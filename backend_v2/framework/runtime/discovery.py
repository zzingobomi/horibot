"""Module instance scan — @service / @subscriber 박힌 method 발견.

inspect 자세 박혀있는 instance 자세 method 자세 walk + spec 박힌 자세 추출.
spec 자세 = ServiceSpec / SubscriberSpec (framework/contract/ 박힘).
"""

from __future__ import annotations

from typing import Any

from framework.contract.service import ServiceSpec, get_service_spec
from framework.contract.subscriber import SubscriberSpec, get_subscriber_spec


def discover_services(module: Any) -> list[tuple[Any, ServiceSpec]]:
    """Module instance 의 @service 박힌 method 자세 + bound method 자세 추출.

    return = (bound_method, spec) pair 자세 list. bound_method 자세 호출 자세 self 자동 박힘.
    """
    result: list[tuple[Any, ServiceSpec]] = []
    for attr_name in dir(module):
        if attr_name.startswith("_"):
            continue
        try:
            attr = getattr(module, attr_name)
        except AttributeError:
            continue
        if not callable(attr):
            continue
        spec = get_service_spec(attr)
        if spec is not None:
            result.append((attr, spec))
    return result


def discover_subscribers(module: Any) -> list[tuple[Any, SubscriberSpec]]:
    """Module instance 의 @subscriber 박힌 method 자세 + bound method 자세 추출."""
    result: list[tuple[Any, SubscriberSpec]] = []
    for attr_name in dir(module):
        if attr_name.startswith("_"):
            continue
        try:
            attr = getattr(module, attr_name)
        except AttributeError:
            continue
        if not callable(attr):
            continue
        spec = get_subscriber_spec(attr)
        if spec is not None:
            result.append((attr, spec))
    return result
