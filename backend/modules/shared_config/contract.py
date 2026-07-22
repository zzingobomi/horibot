"""shared_config wire 계약 — "자연 소유 모듈이 없는 공유 config" 의 owner.

boundary (backend.md 아키텍처 불변식 "config 소유" — 2026-07-22 확정):
여러 모듈이 공유하고 특정 한 모듈의 도메인이 아닌 config **만** 여기 둔다.
첫 멤버 = workcell ROI (detector 셀 밖 후보 컷 + frontend ROI 패널 공유).
**모듈 자기 튜너블은 그 모듈이 소유** — detector score_threshold=Detector /
camera exposure=Camera / motion velocity=Motion. "설정이니까 여기" 로 몰아넣기
금지 (중앙 SettingsService 반사). 항목별 **독립 Mirror** — 한 항목 변경이
무관 소비자에 change fan-out 되지 않게 (거대 단일 Mirror[Everything] 금지).

영속 = robot/instances/<id>/instance.yaml `workcell:` 블록 (SSOT 불변 —
module 이 읽고 쓴다. 손 주석 보존 = ruamel round-trip). 소비 = Mirror
(snapshot + WORKCELL_CHANGED invalidate — Save 즉시 소비자 수렴, 재시작 0).
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import model_validator

from framework.contract.model import StrictModel


class WorkcellRoi(StrictModel):
    """작업 셀 ROI (base frame **AABB**, m) — instance.yaml SSOT 의 wire 투영.

    yaw-OBB 반려 (2026-07-22 — 작업대가 base 축과 정렬 전제, pnp_scenario_rework
    §9.1). **Z 는 바닥 평면이 아니라 볼륨** (핸드오버 공중 물체 포함)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    @model_validator(mode="after")
    def _bounds_ordered(self) -> WorkcellRoi:
        for lo, hi in (("x_min", "x_max"), ("y_min", "y_max"), ("z_min", "z_max")):
            if getattr(self, lo) >= getattr(self, hi):
                raise ValueError(
                    f"workcell ROI {lo}({getattr(self, lo)}) < {hi}"
                    f"({getattr(self, hi)}) 여야 합니다"
                )
        return self


class WorkcellBundle(StrictModel):
    """전 robot workcell snapshot — Mirror value. 미설정 robot 은 dict 에 없음
    (= ROI 컷 미적용, 하위 호환)."""

    robots: dict[str, WorkcellRoi]


class SnapshotWorkcellRequest(StrictModel):
    pass


class SetWorkcellRequest(StrictModel):
    robot_id: str
    roi: WorkcellRoi


class SetWorkcellResponse(StrictModel):
    roi: WorkcellRoi  # 저장·발행 확정값 (검증 통과본)


class WorkcellChanged(StrictModel):
    """Mirror invalidate 이벤트 — 소비자는 이걸 계기로 snapshot 재당김."""

    robot_id: str
    seq: int
    timestamp_unix: float
    roi: WorkcellRoi


class SharedConfig:
    class Service(StrEnum):
        SNAPSHOT_WORKCELL = "srv/shared_config/snapshot_workcell"
        SET_WORKCELL = "srv/shared_config/set_workcell"

    class Event(StrEnum):
        WORKCELL_CHANGED = "event/shared_config/workcell_changed"
