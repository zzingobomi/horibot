"""캘 5종의 domain Pydantic 모델 — *계산 결과* shape + runtime in-memory 표현.

본 파일은 BA / 캡처 알고리즘이 만들어내는 type. 같은 도메인의 *영속화 row shape*
는 `persistence_models.py` 의 `CalibrationResultRecord` (discriminated union) 가
`result_data` 필드에 본 모델 중 하나를 가짐 — `kind` 값에 따라 어떤 ResultData
인지 type system 차원에서 강제.

docs/storage_layer.md §6 — 캘 5종의 3계층 (Result/Evidence/Artifact) 매핑.

field name 은 기존 npz key 컨벤션을 따름 — 마이그레이션 스크립트가 1:1 매핑.

Runtime 사용 — LinkCoordinates / SagCoordinates 가 본 모델을 in-memory state 로
직접 보유 (storage_layer 도입 전 dataclass 가 옛 layer, storage 후 본 Pydantic
모델이 SSOT — 2026-06-22 정리). helper 메서드 (`get_k` / `get_trans` / 등) 가
runtime 호출 자리 자리 사용.
"""

from __future__ import annotations

import numpy as np

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

    def get(self, motor_id: int) -> float:
        return float(self.offsets.get(motor_id, 0.0))

    def is_empty(self) -> bool:
        return not self.offsets


# ─── link_offset ──────────────────────────────────────────────


class LinkOffsetEntry(StrictModel):
    joint_id: int
    trans_m: list[float]  # (3,) — m
    rot_rad: list[float]  # (3,) — rad rotvec (Rodrigues)


class LinkOffsetResultData(StrictModel):
    """URDF 링크 기하 오차 — 부팅 시 URDF patch 에 반영 (restart 필요)."""

    offsets: list[LinkOffsetEntry]
    method: str

    def get_trans(self, jid: int) -> np.ndarray:
        for e in self.offsets:
            if e.joint_id == jid:
                return np.array(e.trans_m, dtype=np.float64)
        return np.zeros(3, dtype=np.float64)

    def get_rot(self, jid: int) -> np.ndarray:
        for e in self.offsets:
            if e.joint_id == jid:
                return np.array(e.rot_rad, dtype=np.float64)
        return np.zeros(3, dtype=np.float64)

    def is_empty(self) -> bool:
        return not self.offsets

    @classmethod
    def from_dicts(
        cls,
        trans: dict[int, np.ndarray],
        rot: dict[int, np.ndarray],
        method: str,
    ) -> "LinkOffsetResultData":
        """{jid: (3,) ndarray} 두 dict → LinkOffsetResultData. trans / rot 의
        joint_id union 으로 entry list 생성, 한쪽에만 있으면 다른 쪽은 zeros."""
        ids = sorted(set(trans.keys()) | set(rot.keys()))
        entries = [
            LinkOffsetEntry(
                joint_id=jid,
                trans_m=list(trans.get(jid, np.zeros(3))),
                rot_rad=list(rot.get(jid, np.zeros(3))),
            )
            for jid in ids
        ]
        return cls(offsets=entries, method=method)


# ─── sag ──────────────────────────────────────────────────────


class SagOffsetResultData(StrictModel):
    """J2/J3 자세 의존 중력 처짐 보정. joint_id (int) → k_rad_per_m (float)."""

    k_rad_per_m: dict[int, float]
    method: str

    def get_k(self, jid: int) -> float:
        return float(self.k_rad_per_m.get(jid, 0.0))

    def as_array_for_joints(self, joint_ids: list[int]) -> np.ndarray:
        """주어진 joint id 순서로 k 값 배열 반환. 없으면 0."""
        return np.array([self.get_k(j) for j in joint_ids], dtype=np.float64)

    def is_empty(self) -> bool:
        return not self.k_rad_per_m


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
