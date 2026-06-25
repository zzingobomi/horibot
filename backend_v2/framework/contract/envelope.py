"""서비스 요청/응답을 감싸는 공통 모델.

Framework는 서비스 호출 시 요청과 응답에
timestamp 정보를 함께 포함한다.

사용자는 Request/Response 모델만 작성하면 되며,
ServiceRequest / ServiceResponse 생성과 해제는
Framework가 내부에서 처리한다.
"""

from __future__ import annotations

from typing import Generic, TypeVar

from pydantic import BaseModel

T = TypeVar("T", bound=BaseModel)


class ServiceRequest(BaseModel, Generic[T]):
    timestamp: float
    data: T


class ServiceResponse(BaseModel, Generic[T]):
    timestamp: float
    data: T
