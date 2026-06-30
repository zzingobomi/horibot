"""Bridge — public contract surface (frontend 노출).

backend_v2_modules.md §1.1 #12 (Bridge) + §8.6 (Bridge = runtime relay only) +
§9.1 (/robots = RobotConfig list 의 read-only view relay, framework helper).

Bridge 는 domain Module 이 아니라 Boundary Adapter — wire 의미는 해석하지 않고
raw relay 만. 단 HTTP helper endpoint (`/robots` / `/system`) 의 응답 shape 은
frontend 와의 contract 라 여기 박는다 (TS gen 대상, §8).

레이어링: modules/bridge 는 apps 를 import 안 함. apps/resolve 가 내부 config
모델 (apps.config.RobotConfig) → 이 wire 모델 (RobotInfo) 변환 책임.
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class BasePoseInfo(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0


class RobotInfo(BaseModel):
    """`/robots` 가 노출하는 robot 1개 — world frame 배치 + capability."""

    id: str
    type: str
    base_pose: BasePoseInfo = Field(default_factory=BasePoseInfo)
    capabilities: list[str] = Field(default_factory=list)


class RobotsResponse(BaseModel):
    robots: list[RobotInfo]
    default: str | None = None  # N=1 편의 — 첫 robot


class SystemMetrics(BaseModel):
    """`/system` — framework metric helper (§9). domain 무관."""

    cpu_percent: float
    mem_percent: float
