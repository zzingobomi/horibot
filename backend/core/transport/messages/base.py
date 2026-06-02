"""공통 base — 모든 typed payload 가 상속하는 schema + 서비스 envelope.

multi_robot_architecture.md §7.6 / §4 참조 (BaseRobotMessage).
typed_messaging.md 결정 사항 #2 참조 (ServiceRequest / ServiceResponse envelope).
"""

from __future__ import annotations

from typing import Generic, TypeVar

from pydantic import BaseModel, ConfigDict, Field

T = TypeVar("T", bound=BaseModel)


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


class EmptyData(BaseModel):
    """payload 가 빈 service request / response 용. unused `_req` 자리에도 쓰임.

    generic envelope 의 data 필드가 required 라서 빈 모델이라도 인스턴스가 필요.
    """

    model_config = ConfigDict(extra="forbid")


class ServiceRequest(BaseModel, Generic[T]):
    """Zenoh get payload 의 typed wrapper. data 는 도메인별 ReqData 모델.

    `base_node.call_service` 가 caller 측에서 timestamp 채워 발신.
    handler 는 `req.data.<field>` 로 접근.
    """

    model_config = ConfigDict(extra="forbid")

    timestamp: float
    data: T


class ServiceResponse(BaseModel, Generic[T]):
    """모든 service response 의 envelope.

    기존 free-form dict `{success, message, data}` 의 typed 버전.
    `data` 가 generic T — 각 service 가 자기 response data type 지정.
    error 시 `success=False`, `data=None`. caller 는 success 확인 후 data 접근.

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

    def unwrap(self) -> T:
        """성공 시 data 반환, 실패 시 RuntimeError. 검증 단순화용 단축."""
        if not self.success or self.data is None:
            raise RuntimeError(f"service failed: {self.message}")
        return self.data
