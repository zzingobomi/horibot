from __future__ import annotations

from typing import Any

from framework.contract.service import ServiceSpec, get_service_spec
from framework.contract.subscriber import SubscriberSpec, get_subscriber_spec


def discover_services(module: Any) -> list[tuple[Any, ServiceSpec]]:
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
