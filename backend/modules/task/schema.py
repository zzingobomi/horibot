"""Task DSL schema — Slot[T] typed reference + StepResult envelope.

Step 간 데이터 전달은 string key 가 아니라 **Slot[T]** typed reference 로 함.
사용자 코드는 변수처럼 step 출력을 들고 다니다가 다음 step 인자에 직접 넘김:

    pick = SearchAndDetect(prompt="cube")        # → pick.out: Slot[Detection]
    grasp = GraspPolicy(target=pick.out)         # 입력: Slot[Detection]
                                                 # 출력: grasp.out: Slot[Pose6]
    MoveTCP(target=grasp.out, offset=(0,0,0.06))

TaskRunner 가 plan ↔ run 사이 `Slot.step_id` 로 results dict lookup. 사용자
코드에 `output_key="pick_pos"` 같은 임의 string 안 나옴.

조립 검증:
    - plan-time: pyright 가 Slot[T] 타입 mismatch 거부
    - run-time: 누락된 step_id / 잘못된 type 은 runner 가 resolve 단계에서 fail

값 클래스 (Position3 / Pose6 / Quaternion / Detection) 는 [core/values.py](
core/values.py) 로 promote — task 외 (detector / motion / calibration) 에서도
공유. 본 모듈은 호환 위해 re-export.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar, Union

from pydantic import BaseModel

# 호환 re-export — 기존 `from modules.task.schema import Position3, ...` 유지.
from core.values import Detection, Position3, Pose6, Quaternion

__all__ = [
    "Detection",
    "Position3",
    "Pose6",
    "Quaternion",
    "Slot",
    "SlotOr",
    "StepResult",
]


# ─── Slot[T] — typed reference to a future step result ──────────────


# Slot 은 read-only reference (frozen=True) 라 covariant 안전.
# 이게 있어야 `Slot[Position3]` 이 `SlotOr[Position3 | Detection]` 자리에 들어감
# (pick_and_place 에서 GraspPolicy 출력 슬롯을 MoveTCP 에 박을 때 필요).
T_co = TypeVar("T_co", covariant=True)
T = TypeVar("T")


@dataclass(frozen=True)
class Slot(Generic[T_co]):
    """다른 step 의 출력에 대한 typed reference.

    Plan-time 표현:
        - `step_id` 만 들고 다님. 그 step 의 출력 type 이 `T`.
        - 사용자 코드는 변수처럼 Slot 을 들고 다니며 다음 step 인자에 박음.

    Run-time 해소:
        - TaskRunner 가 `results: dict[step_id, T]` 유지 → Slot resolve 시 lookup.

    왜 attr_path / transform 안 박는가:
        - v1 은 *단일 typed 값* 만 전달 (Slot[Detection] 자체가 GraspPolicy 입력).
        - sub-attribute (Detection.position 만 필요한 경우) 는 step 내부에서 처리.
        - offset 같은 transform 은 *step 파라미터* 로 남김 (MoveTCP.offset). Slot
          자체를 가공하면 plan-time 표현이 폭증.
    """

    step_id: str

    def __repr__(self) -> str:
        return f"Slot[{self.step_id}]"


# 값 또는 Slot — 모든 step 입력 필드의 공통 type. literal 값으로도 받고
# 이전 step 출력으로도 받음 (e.g. MoveTCP(target=Pose6(...)) vs MoveTCP(target=grasp.out)).
# 여기서는 invariant T 사용 — `SlotOr[A | B]` 자리에 `Slot[A]` 가 들어가는 건
# Slot 의 covariance 가 처리 (위 T_co).
SlotOr = Union[T, Slot[T]]


# ─── Step result envelope — 토픽으로 흐르는 직렬화 형태 ───────────────


@dataclass
class StepResult:
    """Step 실행 결과 — runner 가 results dict 에 저장하는 형태.

    토픽 publish 시 (`omx/task/step_results`) 사용:
        {"step_id": "...", "type": "Detection", "value": {...}}

    type 문자열은 frontend TaskResultLayer 가 렌더러 dispatch 에 사용.
    None value 는 "사이드이펙트만 있고 출력 없는 step" (MoveTCP/Gripper/...).

    value 는 보통 pydantic BaseModel (Position3 / Detection / ...) — `model_dump`
    로 직렬화. dict / list 같은 plain 타입은 그대로 통과.
    """

    step_id: str
    type_name: str  # "Position3" / "Pose6" / "Detection" / "None"
    value: object | None

    def to_dict(self) -> dict:
        if self.value is None:
            payload = None
        elif isinstance(self.value, BaseModel):
            payload = self.value.model_dump()
        else:
            payload = self.value
        return {
            "step_id": self.step_id,
            "type": self.type_name,
            "value": payload,
        }
