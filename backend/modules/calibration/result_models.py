"""캘 5종의 domain Pydantic 모델 — *계산 결과* shape.

본 파일은 BA / 캡처 알고리즘이 만들어내는 type. 같은 도메인의 *영속화 row shape*
는 `persistence_models.py` 의 `CalibrationResultRecord` (discriminated union) 가
`result_data` 필드에 본 모델 중 하나를 가짐 — `kind` 값에 따라 어떤 ResultData
인지 type system 차원에서 강제.

docs/storage_layer.md §6 — 캘 5종의 3계층 (Result/Evidence/Artifact) 매핑.

field name 은 기존 npz key 컨벤션을 따름 — 마이그레이션 스크립트가 1:1 매핑.
"""

from __future__ import annotations

from core.transport.messages.base import StrictModel


# ─── intrinsic ────────────────────────────────────────────────


class IntrinsicResultData(StrictModel):
    """카메라 내부 파라미터 (D405 등)."""

    camera_matrix: list[list[float]]  # (3, 3)
    dist_coeffs: list[list[float]]  # (1, N) — OpenCV 컨벤션
    image_size: list[int] | None = None  # [width, height]


# ─── hand_eye ─────────────────────────────────────────────────


class HandEyeResultData(StrictModel):
    """카메라 ↔ EE (gripper) 변환. R_cam2gripper / t_cam2gripper.

    Detector + PointCloudLayer 가 사용 — runtime cache 의 hot path.
    """

    R_cam2gripper: list[list[float]]  # (3, 3)
    t_cam2gripper: list[list[float]]  # (3, 1)
    method: str  # "BA(physical_sag_irls)" / "TSAI" 등


# ─── joint_offset ─────────────────────────────────────────────


class JointOffsetResultData(StrictModel):
    """모터 raw zero 오차 보정. motor_id (int) → offset_rad (float).

    JSON 직렬화 시 dict[int, float] 의 key 는 string 화 — Pydantic 가 양방향 변환.
    """

    offsets: dict[int, float]
    method: str


# ─── link_offset ──────────────────────────────────────────────


class LinkOffsetEntry(StrictModel):
    joint_id: int
    trans_m: list[float]  # (3,) — m
    rot_rad: list[float]  # (3,) — rad rotvec (Rodrigues)


class LinkOffsetResultData(StrictModel):
    """URDF 링크 기하 오차 — 부팅 시 URDF patch 에 반영 (restart 필요)."""

    offsets: list[LinkOffsetEntry]
    method: str


# ─── sag ──────────────────────────────────────────────────────


class SagOffsetResultData(StrictModel):
    """J2/J3 자세 의존 중력 처짐 보정. joint_id (int) → k_rad_per_m (float)."""

    k_rad_per_m: dict[int, float]
    method: str


# ─── union alias ───────────────────────────────────────────────
# persistence_models 의 ResultRecord 들이 kind 별로 본 union 의 한 arm 을 가짐.
# 본 alias 자체는 *kind 분리 없이* 5종 중 하나라는 의미 — discriminator 가 없음.
# 진짜 discriminated union 은 persistence_models.CalibrationResultRecord.

CalibrationResultData = (
    HandEyeResultData
    | IntrinsicResultData
    | JointOffsetResultData
    | LinkOffsetResultData
    | SagOffsetResultData
)
