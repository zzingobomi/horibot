"""Task DSL schema — Slot[T] typed reference + StepResult + 공유 값 타입.

옛 backend/modules/task/schema.py + core/values.py 를 v2 로 재구성 (기계적 복사 X).
Step 간 데이터 전달은 string key 가 아니라 **Slot[T]** typed reference:

    pick = GroundedDetect(prompt="cube")     # → pick.out: Slot[Detection]
    grasp = GraspPolicy(target=pick.out)     # 입력 Slot[Detection] → 출력 Slot[Position3]
    MoveTCP(target=grasp.out, offset=Position3(x=0, y=0, z=0.06))

값 타입 (Position3/Pose6/Quaternion) = task DSL geometry, 여기 소유 (§17.4 "위치 =
빌드 시 결정"). Detection 은 detector 소유 (modules.detector.contract) — task 가
import (검출은 detector 도메인, task 는 소비자). frozen pydantic — wire 직렬화 겸용.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar, Union

from pydantic import BaseModel, ConfigDict


# ─── 공유 값 타입 (base frame geometry) ──────────────────────────────


class Position3(BaseModel):
    model_config = ConfigDict(frozen=True)

    x: float
    y: float
    z: float

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    @classmethod
    def from_iter(cls, it: tuple[float, float, float] | list[float]) -> "Position3":
        x, y, z = it
        return cls(x=float(x), y=float(y), z=float(z))

    def __add__(
        self, other: "Position3 | tuple[float, float, float]"
    ) -> "Position3":
        if isinstance(other, Position3):
            return Position3(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)
        ox, oy, oz = other
        return Position3(x=self.x + ox, y=self.y + oy, z=self.z + oz)


class Quaternion(BaseModel):
    model_config = ConfigDict(frozen=True)

    x: float
    y: float
    z: float
    w: float

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.w]


class Pose6(BaseModel):
    """6DoF pose — position + (optional) orientation (MoveL v1 은 position-only)."""

    model_config = ConfigDict(frozen=True)

    position: Position3
    orientation: Quaternion | None = None


# ─── Slot[T] — 다른 step 출력에 대한 typed reference ─────────────────

# Slot 은 read-only reference (frozen) 라 covariant 안전 — Slot[Position3] 가
# SlotOr[Position3 | Detection] 자리에 들어감.
T_co = TypeVar("T_co", covariant=True)
T = TypeVar("T")


@dataclass(frozen=True)
class Slot(Generic[T_co]):
    """plan-time = step_id 만. run-time = runner 의 results[step_id] lookup.

    사용자 코드는 변수처럼 Slot 을 들고 다니며 다음 step 인자에 박는다. runner 가
    resolve 시 결과 dict lookup — 임의 string key ("pick_pos") 안 나옴.
    """

    step_id: str

    def __repr__(self) -> str:
        return f"Slot[{self.step_id}]"


# 값 또는 Slot — 모든 step 입력 필드의 공통 type. literal 값으로도, 이전 step
# 출력(Slot)으로도 받음. SlotOr[A | B] 자리에 Slot[A] 는 Slot 의 covariance 로 OK.
SlotOr = Union[T, Slot[T]]


# ─── StepResult — step_result 토픽으로 흐르는 직렬화 형태 ─────────────


@dataclass
class StepResult:
    """Step 실행 결과 — runner 가 results dict 저장 + step_result 토픽 payload.

    type_name = frontend TaskResultLayer 가 렌더러 dispatch 에 쓰는 식별자
    ("Detection" → sphere, "Position3" → marker, "None" → 도달 마커).
    value = pydantic BaseModel (model_dump) / dict·list plain / None.
    """

    step_id: str
    type_name: str  # "Position3" / "Detection" / "list" / "None" ...
    value: object | None

    def to_dict(self) -> dict:
        return {
            "step_id": self.step_id,
            "type": self.type_name,
            "value": _serialize(self.value),
        }


def _serialize(value: object) -> object:
    """BaseModel / list / dict 재귀 직렬화 — list[Detection] 등 step 결과도 wire 안전.

    frontend TaskResultLayer 가 dispatch 하는 JSON payload. Detection 후보 리스트를
    반환하는 step (SearchWaypointGroup) 이 있어 재귀 필요.
    """
    if isinstance(value, BaseModel):
        return value.model_dump()
    if isinstance(value, (list, tuple)):
        return [_serialize(v) for v in value]
    if isinstance(value, dict):
        return {k: _serialize(v) for k, v in value.items()}
    return value
