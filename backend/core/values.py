"""공통 값 클래스 — pydantic BaseModel 로 정의된 typed value 타입.

modules/task/schema.py 에 dataclass 로 있던 Position3 / Quaternion / Pose6 /
Detection 을 core 로 promote. task 외에 detector / motion / calibration 메시지
schema 에서도 쓰이므로 core 자리가 맞음.

frozen=True 로 hashable + immutable — Slot[T] covariance 와 동일하게 plan-time
참조용 값 / 메시지 payload 양쪽에서 동일하게 흐를 수 있게.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict


class Position3(BaseModel):
    """베이스 프레임 xyz [m]. EE / 객체 / 검출 결과 위치의 공통 표현."""

    model_config = ConfigDict(frozen=True)

    x: float
    y: float
    z: float

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    @classmethod
    def from_iter(cls, it) -> Position3:
        """list/tuple/ndarray 등 길이 3 iterable → Position3."""
        x, y, z = it
        return cls(x=float(x), y=float(y), z=float(z))

    def __add__(self, other: Position3 | tuple[float, float, float]) -> Position3:
        if isinstance(other, Position3):
            return Position3(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)
        ox, oy, oz = other
        return Position3(x=self.x + ox, y=self.y + oy, z=self.z + oz)


class Quaternion(BaseModel):
    """단위 quaternion. solver.ik / solver.fk 의 orientation 인자와 호환."""

    model_config = ConfigDict(frozen=True)

    x: float
    y: float
    z: float
    w: float

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.w]


class Pose6(BaseModel):
    """6DoF pose — position + (optional) orientation.

    `orientation=None` 은 motion API 의 "orientation 자유" 모드와 호환 — 5DOF
    OMX_F 에서 top-down 강제 안 하고 IK 가 manipulability 최적 자세 고르게 함.
    """

    model_config = ConfigDict(frozen=True)

    position: Position3
    orientation: Quaternion | None = None


class Detection(BaseModel):
    """Grounding DINO / YOLO 검출 결과.

    `position` 은 객체 윗면 중심 (base frame). GraspPolicy 가 base_z/height 로
    grasp_z 를 derive 하므로 셋 다 같이 다님 — context dict 의 `_meta` suffix
    꼼수 추방의 핵심 자산.
    """

    model_config = ConfigDict(frozen=True)

    position: Position3
    height: float = 0.0
    base_z: float = 0.0
    confidence: float = 0.0
    prompt: str = ""
