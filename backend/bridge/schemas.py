"""bridge REST endpoint 응답 Pydantic schema — frontend 와의 SSOT.

`api_contract.py` 는 Zenoh topic/service payload 의 SSOT 이고, 이 파일은 bridge
가 노출하는 REST endpoint (`/robots`, `/tasks`, `/system` 등) 응답의 Pydantic
모델 — FastAPI 의 `response_model=` 으로 명시하면 OpenAPI `components/schemas`
에 자동 등재됨. frontend `pnpm gen:types` 가 같은 JSON 을 읽어 `types.ts` 로
emit → frontend 는 hand-sync 없이 `components["schemas"]["…"]` 로 import.

새 REST endpoint 추가 절차:
  - 응답 Pydantic 모델 정의 here
  - endpoint 에 `response_model=...` 명시
  - backend 재시작 후 frontend 에서 `pnpm gen:types`
"""

from __future__ import annotations

from pydantic import BaseModel

from core.robot.robot_registry import RobotCapability


class BasePoseSchema(BaseModel):
    """World frame 의 robot base pose (m + deg)."""

    x: float
    y: float
    z: float
    yaw_deg: float


class RobotInfo(BaseModel):
    """robots.yaml 의 entry 1개를 frontend 가 받는 모양 — `RobotConfig` 의 hardware
    path 들 (`type_dir`, `calibration_dir` 등) 은 제외한 frontend-exposed subset.
    `capabilities` 는 sidebar mode sub-item / `/robots/:id/:mode` route 활성화 결정."""

    id: str
    type: str
    enabled: bool
    capabilities: list[RobotCapability]
    base_pose: BasePoseSchema
    urdf_url: str


class RobotsListResponse(BaseModel):
    """`GET /robots` 응답. default 는 enabled robot 이 정확히 1개일 때만 값,
    아니면 null (frontend 가 명시적 robot_id 사용하도록 강제)."""

    robots: list[RobotInfo]
    default: str | None


class SystemMetrics(BaseModel):
    """`GET /system` 응답. psutil + zenoh peer info — Dashboard overview source."""

    cpu_pct: float
    mem_used_mb: float
    mem_total_mb: float
    mem_pct: float
    zenoh_routers: int
    zenoh_peers: int


class TasksResponse(BaseModel):
    """`GET /tasks` 응답 — task_node.TASK_REGISTRY enumeration."""

    tasks: list[str]


class IntrinsicSchema(BaseModel):
    """카메라 intrinsic 응답 형식 — `loader.to_json` 의 intrinsic 키 내용."""

    camera_matrix: list[list[float]]  # 3x3
    dist_coeffs: list[list[float]]  # 1xN
    image_size: list[int] | None = None  # [w, h]


class HandEyeSchema(BaseModel):
    """Hand-Eye 응답 — 카메라 ↔ EE 변환 R/t."""

    R: list[list[float]]  # 3x3
    t: list[list[float]]  # 3x1


class JointOffsetSchema(BaseModel):
    """단일 joint offset 항목."""

    motor_id: int
    offset_rad: float


class CalibrationResults(BaseModel):
    """`GET /calibration/results` 응답. npz 없으면 해당 필드 생략, joint_offsets 는
    항상 포함 (없으면 빈 리스트)."""

    intrinsic: IntrinsicSchema | None = None
    hand_eye: HandEyeSchema | None = None
    joint_offsets: list[JointOffsetSchema] = []
