"""CalibrationCache — runtime 소비자의 in-memory state + cross-process fetch.

두 가지 진입 path:
1. **PC in-process push** (기존) — calibration_node 가 부팅 시 + COMMIT 후 `set()`.
   PC consumer (DetectorNode / bridge router / task_node / tsdf_builder) 가 `get()` read.
2. **Cross-process self-fill** — `fetch_active(robot_id)` 가 storage 호출해 5종 snapshot
   구성 + intrinsic/hand_eye 자기 cache 에 보관 + signal_ready. 분산 자리의 consumer
   process (motor Pi 의 motion_node) 가 부팅 시 호출.

docs/storage_layer.md §7 — calibration_node 만 storage 알아야 한다는 원칙은 PC
in-process push 자리. 분산 consumer 는 자기 process 에서 storage 호출 = layer 위반
아니라 transport (Zenoh) 통한 SSOT 조회 — Zenoh 가 transport 추상화.

intrinsic + hand_eye 만 본 cache 에 — joint/link/sag offsets 는 Coordinates 싱글톤이
자체 in-memory state. snapshot 은 caller (CalibrationApplier) 가 받아서 Coordinates
3종에 push (cache 는 그 자리 모름).
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field

import numpy as np

from modules.calibration.link_offsets import LinkOffsets
from modules.calibration.loader import (
    CalibrationData,
    HandEyeData,
    IntrinsicData,
)
from modules.calibration.sag_offsets import SagOffsets

logger = logging.getLogger(__name__)


@dataclass
class CalibrationSnapshot:
    """5종 캘 한 묶음 — fetch_active 결과. CalibrationApplier 가 소비.

    cache 는 본 snapshot 자체를 *저장하지 않음* (intrinsic/hand_eye 만 self._by_robot
    에 보관). joint/link/sag 는 caller 가 받아서 Coordinates 싱글톤에 push.
    """
    joint_offsets: dict[int, float] = field(default_factory=dict)
    link_offsets: LinkOffsets = field(default_factory=LinkOffsets)
    sag_offsets: SagOffsets = field(default_factory=SagOffsets)
    intrinsic: IntrinsicData | None = None
    hand_eye: HandEyeData | None = None


class CalibrationCache:
    _instance: "CalibrationCache | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "CalibrationCache":
        if cls._instance is None:
            with cls._new_lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        self._lock = threading.Lock()
        self._by_robot: dict[str, CalibrationData] = {}
        # ready event 는 calibration_node 의 atomic push 완료 신호. consumer 의
        # hot path 가 wait_ready 로 partial state 회피.
        self._ready: dict[str, threading.Event] = {}

    def _event(self, robot_id: str) -> threading.Event:
        with self._lock:
            ev = self._ready.get(robot_id)
            if ev is None:
                ev = threading.Event()
                self._ready[robot_id] = ev
            return ev

    def set(self, robot_id: str, calib: CalibrationData) -> None:
        """calibration_node 가 atomic push 완료 직전 호출 + signal_ready 호출.

        intrinsic / hand_eye 외 다른 source (Coordinates 들) 도 같이 set 되어
        있는 상태에서 호출 — partial publish 금지 (docs/storage_layer.md §7).
        """
        with self._lock:
            self._by_robot[robot_id] = calib

    def signal_ready(self, robot_id: str) -> None:
        """5종 push 완료 후 호출. consumer hot path 의 wait_ready 가 풀림."""
        self._event(robot_id).set()

    def is_ready(self, robot_id: str) -> bool:
        return self._event(robot_id).is_set()

    def wait_ready(self, robot_id: str, timeout: float | None = None) -> bool:
        """consumer hot path — calibration_node 의 atomic push 완료까지 대기.

        timeout=None 이면 무한. False 반환 = timeout — caller 가 service 거부 결정.
        """
        return self._event(robot_id).wait(timeout)

    def get(self, robot_id: str) -> CalibrationData:
        """robot 에 아직 push 안 됐으면 empty CalibrationData — caller 가 is_ready
        체크해서 서비스 거부 / UI 경고.
        """
        with self._lock:
            return self._by_robot.get(robot_id) or CalibrationData()

    # ─── Cross-process self-fill (분산 consumer 부팅 자리) ──────

    def fetch_active(self, robot_id: str) -> CalibrationSnapshot:
        """Storage 에서 5종 active calibration fetch → snapshot 반환.

        intrinsic/hand_eye 는 자기 cache 에 보관 + signal_ready (기존 consumer
        호환). joint/link/sag offsets 는 snapshot 으로만 caller (Applier) 에
        전달 — cache 는 보관 X.

        storage 응답 OK + found=false (첫 부팅 robot) 자리는 해당 kind 만 default
        (empty offsets / identity matrix) — fail 아님. storage unreachable 자리는
        load_active_blocking 가 무한 retry (docs/storage_layer.md §7 의 "Storage
        필수 가정"). caller 는 storage 떠 있는 자리에서만 호출.
        """
        from modules.calibration.storage_client import load_active_blocking

        joint_rec = load_active_blocking(robot_id, "joint_offset")
        link_rec = load_active_blocking(robot_id, "link_offset")
        sag_rec = load_active_blocking(robot_id, "sag")
        intrinsic_rec = load_active_blocking(robot_id, "intrinsic")
        hand_eye_rec = load_active_blocking(robot_id, "hand_eye")

        joint_offsets = (
            dict(joint_rec.result_data.offsets)
            if joint_rec is not None and joint_rec.kind == "joint_offset"
            else {}
        )
        if link_rec is not None and link_rec.kind == "link_offset":
            link_offsets = LinkOffsets(
                trans={
                    e.joint_id: np.array(e.trans_m, dtype=np.float64)
                    for e in link_rec.result_data.offsets
                },
                rot={
                    e.joint_id: np.array(e.rot_rad, dtype=np.float64)
                    for e in link_rec.result_data.offsets
                },
            )
        else:
            link_offsets = LinkOffsets()
        sag_offsets = (
            SagOffsets(k_rad_per_m=dict(sag_rec.result_data.k_rad_per_m))
            if sag_rec is not None and sag_rec.kind == "sag"
            else SagOffsets()
        )
        intrinsic = (
            IntrinsicData(
                camera_matrix=np.array(
                    intrinsic_rec.result_data.camera_matrix, dtype=np.float64
                ),
                dist_coeffs=np.array(
                    intrinsic_rec.result_data.dist_coeffs, dtype=np.float64
                ),
                image_size=(
                    tuple(intrinsic_rec.result_data.image_size)  # type: ignore[arg-type]
                    if intrinsic_rec.result_data.image_size is not None
                    else None
                ),
            )
            if intrinsic_rec is not None and intrinsic_rec.kind == "intrinsic"
            else None
        )
        hand_eye = (
            HandEyeData(
                R=np.array(hand_eye_rec.result_data.R_cam2gripper, dtype=np.float64),
                t=np.array(hand_eye_rec.result_data.t_cam2gripper, dtype=np.float64),
            )
            if hand_eye_rec is not None and hand_eye_rec.kind == "hand_eye"
            else None
        )

        snapshot = CalibrationSnapshot(
            joint_offsets=joint_offsets,
            link_offsets=link_offsets,
            sag_offsets=sag_offsets,
            intrinsic=intrinsic,
            hand_eye=hand_eye,
        )

        # 기존 consumer 호환 — intrinsic/hand_eye 는 cache 에 set + signal_ready.
        self.set(robot_id, CalibrationData(intrinsic=intrinsic, hand_eye=hand_eye))
        self.signal_ready(robot_id)
        logger.info(
            "[%s] fetch_active 완료 — joint=%d link.trans=%d sag=%d intrinsic=%s hand_eye=%s",
            robot_id,
            len(joint_offsets),
            len(link_offsets.trans),
            len(sag_offsets.k_rad_per_m),
            "Y" if intrinsic is not None else "N",
            "Y" if hand_eye is not None else "N",
        )
        return snapshot
