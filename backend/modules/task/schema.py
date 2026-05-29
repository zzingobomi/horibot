"""Task DSL schema — typed value classes + Slot[T] reference 시스템.

Step 간 데이터 전달은 string key 가 아니라 **Slot[T]** typed reference 로 함.
사용자 코드는 변수처럼 step 출력을 들고 다니다가 다음 step 인자에 직접 넘김:

    pick = SearchAndDetect(prompt="cube")        # → pick.out: Slot[Detection]
    grasp = GraspPolicy(target=pick.out)         # 입력: Slot[Detection]
                                                 # 출력: grasp.out: Slot[Pose6]
    MoveTCP(target=grasp.out, offset=(0,0,0.06))

TaskRunner 가 plan ↔ run 사이 `Slot.step_id` 로 results dict lookup. 사용자
코드에 `output_key="pick_pos"` 같은 임의 string 안 나옴.

조립 검증 (ideas.md "Step DSL 레고화" 의 lego test #1, #2):
    - plan-time: pyright 가 Slot[T] 타입 mismatch 거부
    - run-time: 누락된 step_id / 잘못된 type 은 runner 가 resolve 단계에서 fail

frontend 시각화 자동화: 모든 typed value 가 dataclass 라 `asdict` 로 토픽 직렬화
가능 → frontend TaskResultLayer 가 type 별 (Detection/Pose6/...) 자동 렌더.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar, Union


# ─── Typed value classes — step 출력 / 사용자 입력으로 흐르는 값 ──────


@dataclass(frozen=True)
class Position3:
    """베이스 프레임 xyz [m]. EE / 객체 / 검출 결과 위치의 공통 표현."""

    x: float
    y: float
    z: float

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    @classmethod
    def from_iter(cls, it) -> Position3:
        """list/tuple/ndarray 등 길이 3 iterable → Position3."""
        x, y, z = it
        return cls(float(x), float(y), float(z))

    def __add__(self, other: Position3 | tuple) -> Position3:
        if isinstance(other, Position3):
            return Position3(self.x + other.x, self.y + other.y, self.z + other.z)
        ox, oy, oz = other
        return Position3(self.x + ox, self.y + oy, self.z + oz)


@dataclass(frozen=True)
class Quaternion:
    """단위 quaternion. solver.ik / solver.fk 의 orientation 인자와 호환."""

    x: float
    y: float
    z: float
    w: float

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.w]


@dataclass(frozen=True)
class Pose6:
    """6DoF pose — position + (optional) orientation.

    `orientation=None` 은 motion API 의 "orientation 자유" 모드와 호환 — 5DOF
    OMX_F 에서 top-down 강제 안 하고 IK 가 manipulability 최적 자세 고르게 함.
    """

    position: Position3
    orientation: Quaternion | None = None


@dataclass(frozen=True)
class Detection:
    """Grounding DINO / YOLO 검출 결과.

    `position` 은 객체 윗면 중심 (base frame). GraspPolicy 가 base_z/height 로
    grasp_z 를 derive 하므로 셋 다 같이 다님 — context dict 의 `_meta` suffix
    꼼수 추방의 핵심 자산.
    """

    position: Position3
    height: float = 0.0
    base_z: float = 0.0
    confidence: float = 0.0
    prompt: str = ""


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
    """

    step_id: str
    type_name: str  # "Position3" / "Pose6" / "Detection" / "None"
    value: object | None  # dataclass instance — to_dict 시 asdict 처리

    def to_dict(self) -> dict:
        from dataclasses import asdict, is_dataclass

        if self.value is None:
            payload = None
        elif is_dataclass(self.value) and not isinstance(self.value, type):
            payload = asdict(self.value)
        else:
            payload = self.value
        return {
            "step_id": self.step_id,
            "type": self.type_name,
            "value": payload,
        }
