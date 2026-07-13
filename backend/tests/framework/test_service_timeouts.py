"""서비스 기본 timeout 선언/해석 — contract 가 timeout 의 SSOT.

의미 (뒤집으면 회귀): 명시 timeout 무시 / 선언 없는 키가 기본값 아님 /
같은 키 상충 재선언 침묵 허용 (두 contract 가 다른 값 주장 → 조용한 오동작).
"""

from __future__ import annotations

import pytest

from framework.contract.service import (
    DEFAULT_SERVICE_TIMEOUT_S,
    declare_service_timeouts,
    resolve_service_timeout,
)


def test_resolution_order_explicit_declared_default():
    declare_service_timeouts({"srv/test_timeouts/slow": 60.0})

    # 명시 > 선언
    assert resolve_service_timeout("srv/test_timeouts/slow", 3.0) == 3.0
    # 선언
    assert resolve_service_timeout("srv/test_timeouts/slow", None) == 60.0
    # 미선언 → 기본
    assert (
        resolve_service_timeout("srv/test_timeouts/unknown", None)
        == DEFAULT_SERVICE_TIMEOUT_S
    )


def test_conflicting_redeclaration_fails_fast():
    declare_service_timeouts({"srv/test_timeouts/dup": 10.0})
    declare_service_timeouts({"srv/test_timeouts/dup": 10.0})  # 동일 값 = 허용
    with pytest.raises(ValueError, match="중복 선언"):
        declare_service_timeouts({"srv/test_timeouts/dup": 20.0})


def test_robot_scoped_template_key_is_the_registry_key():
    # 등록은 template — runtime.call 이 robot_id 확장 **전** 키로 조회하는 계약
    declare_service_timeouts({"srv/test_timeouts/{robot_id}/move": 45.0})
    assert resolve_service_timeout("srv/test_timeouts/{robot_id}/move", None) == 45.0
