from __future__ import annotations

from pydantic import BaseModel, ConfigDict


class Position3(BaseModel):
    model_config = ConfigDict(frozen=True)

    x: float
    y: float
    z: float

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    @classmethod
    def from_iter(cls, it) -> Position3:
        x, y, z = it
        return cls(x=float(x), y=float(y), z=float(z))

    def __add__(self, other: Position3 | tuple[float, float, float]) -> Position3:
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
    """6DoF pose — position + (optional) orientation."""

    model_config = ConfigDict(frozen=True)

    position: Position3
    orientation: Quaternion | None = None


class Detection(BaseModel):
    """Grounding DINO / YOLO 검출 결과."""

    model_config = ConfigDict(frozen=True)

    position: Position3
    height: float = 0.0
    base_z: float = 0.0
    confidence: float = 0.0
    prompt: str = ""
