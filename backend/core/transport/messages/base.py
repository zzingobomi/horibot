"""공통 base — 모든 typed payload 가 상속하는 schema + 서비스 envelope.

multi_robot_architecture.md §7.6 / §4 참조 (BaseRobotMessage).
typed_messaging.md 결정 사항 #2 참조 (ServiceRequest / ServiceResponse envelope).

`StrictModel` — 모든 message 모델의 기본 base. `extra="forbid"` 한 자리에서
강제하고 자식들은 상속만 — 자식이 자기 `model_config` 로 override 가능 (pydantic v2
의 model_config 는 merge 됨; 예: `populate_by_name=True` 만 추가 시 `extra="forbid"`
유지). 자유 schema 가 필요한 자리만 `extra="allow"` override.
"""

from __future__ import annotations

from typing import Generic, TypeVar

from pydantic import BaseModel, ConfigDict, Field

T = TypeVar("T", bound=BaseModel)


class StrictModel(BaseModel):
    """모든 typed message / envelope 의 base. extra="forbid" 강제."""

    model_config = ConfigDict(extra="forbid")


class BaseRobotMessage(StrictModel):
    """robot-scoped 토픽 페이로드의 공통 base.

    robot_id 가 모든 페이로드에 강제 — multi-robot routing 시 일관성.
    timestamp 는 발신 시각 (epoch seconds, UTC).
    """

    robot_id: str = Field(..., description="robot instance id (robots.yaml 의 key)")
    timestamp: float = Field(..., description="발신 시각, epoch seconds (UTC)")


class EmptyData(StrictModel):
    """payload 가 빈 service request / response 용. unused `_req` 자리에도 쓰임.

    generic envelope 의 data 필드가 required 라서 빈 모델이라도 인스턴스가 필요.
    """


class ServiceRequest(StrictModel, Generic[T]):
    """Zenoh get payload 의 typed wrapper. data 는 도메인별 ReqData 모델.

    `base_node.call_service` 가 caller 측에서 timestamp 채워 발신.
    handler 는 `req.data.<field>` 로 접근.
    """

    timestamp: float
    data: T


class ServiceResponse(StrictModel, Generic[T]):
    """모든 service response 의 envelope.

    기존 free-form dict `{success, message, data}` 의 typed 버전.
    `data` 가 generic T — 각 service 가 자기 response data type 지정.
    error 시 `success=False`, `data=None`. caller 는 success 확인 후 data 접근.

    Example:
        class MoveResultData(StrictModel):
            duration_s: float

        def move_l(...) -> ServiceResponse[MoveResultData]:
            ...
    """

    success: bool
    message: str = ""
    data: T | None = None

    def unwrap(self) -> T:
        """성공 시 data 반환, 실패 시 RuntimeError. 검증 단순화용 단축."""
        if not self.success or self.data is None:
            raise RuntimeError(f"service failed: {self.message}")
        return self.data
