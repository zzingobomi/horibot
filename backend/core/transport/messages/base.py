from __future__ import annotations

from typing import Generic, TypeVar

from pydantic import BaseModel, ConfigDict

T = TypeVar("T", bound=BaseModel)


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class EmptyData(StrictModel):
    """payload 가 빈 service request / response 용."""


class ServiceRequest(StrictModel, Generic[T]):
    """모든 service request 의 envelope."""

    timestamp: float
    data: T


class ServiceResponse(StrictModel, Generic[T]):
    """모든 service response 의 envelope."""

    success: bool
    message: str = ""
    data: T | None = None

    def unwrap(self) -> T:
        if not self.success or self.data is None:
            raise RuntimeError(f"service failed: {self.message}")
        return self.data
