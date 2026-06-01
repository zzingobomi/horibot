"""공통 base — 모든 typed payload 가 상속하는 schema.

multi_robot_architecture.md §7.6 / §4 참조.
"""

from __future__ import annotations

from typing import Generic, TypeVar

from pydantic import BaseModel, ConfigDict, Field

T = TypeVar("T")


class BaseRobotMessage(BaseModel):
    """robot-scoped 토픽 페이로드의 공통 base.

    robot_id 가 모든 페이로드에 강제 — multi-robot routing 시 일관성.
    timestamp 는 발신 시각 (epoch seconds, UTC).
    """

    model_config = ConfigDict(
        # Zenoh JSON 통신에 frozen 까지는 필요 X — 그러나 extra="forbid" 로
        # schema 외 필드 들어오면 에러 (drift 방지).
        extra="forbid",
    )

    robot_id: str = Field(..., description="robot instance id (robots.yaml 의 key)")
    timestamp: float = Field(..., description="발신 시각, epoch seconds (UTC)")


class ServiceResponse(BaseModel, Generic[T]):
    """모든 service response 의 envelope.

    기존 free-form dict `{success, message, data}` 의 typed 버전.
    `data` 가 generic T — 각 service 가 자기 response data type 지정.

    Example:
        class MoveResultData(BaseModel):
            duration_s: float

        def move_l(...) -> ServiceResponse[MoveResultData]:
            ...
    """

    model_config = ConfigDict(extra="forbid")

    success: bool
    message: str = ""
    data: T | None = None
