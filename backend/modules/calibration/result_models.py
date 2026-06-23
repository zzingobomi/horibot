from __future__ import annotations

import numpy as np

from core.transport.messages.base import StrictModel


class IntrinsicResultData(StrictModel):
    camera_matrix: list[list[float]]  # (3, 3)
    dist_coeffs: list[list[float]]  # (1, N)
    image_size: list[int] | None = None


class HandEyeResultData(StrictModel):
    R_cam2gripper: list[list[float]]  # (3, 3)
    t_cam2gripper: list[list[float]]  # (3, 1)
    method: str  # "BA(physical_sag_irls)" / "TSAI" 등


class JointOffsetResultData(StrictModel):
    offsets: dict[int, float]
    method: str

    def get(self, motor_id: int) -> float:
        return float(self.offsets.get(motor_id, 0.0))

    def is_empty(self) -> bool:
        return not self.offsets


class LinkOffsetEntry(StrictModel):
    joint_id: int
    trans_m: list[float]  # (3,) — m
    rot_rad: list[float]  # (3,) — rad rotvec (Rodrigues)


class LinkOffsetResultData(StrictModel):
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
    def from_calibration_result(
        cls,
        trans: dict[int, np.ndarray],
        rot: dict[int, np.ndarray],
        method: str,
    ) -> "LinkOffsetResultData":
        """translation / rotation 결과를 joint 기준으로 하나의 offset 목록으로 변환."""
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


class SagOffsetResultData(StrictModel):
    k_rad_per_m: dict[int, float]
    method: str

    def get_k(self, jid: int) -> float:
        return float(self.k_rad_per_m.get(jid, 0.0))

    def as_array_for_joints(self, joint_ids: list[int]) -> np.ndarray:
        return np.array([self.get_k(j) for j in joint_ids], dtype=np.float64)

    def is_empty(self) -> bool:
        return not self.k_rad_per_m
