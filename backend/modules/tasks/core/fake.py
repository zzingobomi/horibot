"""FakeContext — 시나리오 로직을 하드웨어/wire 없이 검증하는 공식 테스트 표면.

의미 수준 fake: primitive 호출을 (kind, robot_id, label, args) 로 기록하고 스크립트된
값을 돌려준다. TaskContext/RobotHandle 을 **상속 + 동일 시그니처 override** —
실 표면과 어긋나면 pyright 가 잡는다 (드리프트 방지).

    ctx = FakeContext(
        robots=["so101_6dof_0"],
        detect_script={"white cube": [[], [det(score=0.9)]]},   # 호출 차수별 반환
        reachable_script=[2],                                   # select 호출별 index
    )
    await pick_and_place_scenario(ctx, pick_object="white cube")
    assert ctx.kinds() == ["detect_oriented", ...]
    assert "pre_place" not in ctx.labels()                      # place 분기 안 탐

실패 경로: detect_script 소진/미등록 prompt 는 명확한 에러, reachable -1 은 실물과
동일하게 NoReachableGrasp raise, fail_labels 에 label 을 넣으면 그 move/gripper 가
MotionRejected/GripperFailed 를 raise.
"""

from __future__ import annotations

from typing import Any, Literal, TypeVar

from pydantic import BaseModel

from modules.detector.contract import OrientedDetection
from modules.motion.contract import TcpPose

from .context import Quat, RobotHandle, TaskContext, Vec3, dump_value
from .errors import GripperFailed, MotionRejected, NoReachableGrasp
from .spec import TaskRobotSpec

TRes = TypeVar("TRes", bound=BaseModel)


class _NoWireRuntime:
    """FakeContext 는 wire 에 닿으면 안 됨 — 닿는 순간 명확히 터뜨린다."""

    def publish(self, wire_key: str, event: BaseModel) -> None:
        raise AssertionError(f"FakeContext 에서 wire publish 발생: {wire_key}")

    async def call(self, key: str, req: BaseModel, res_cls: Any, **_: Any) -> Any:
        raise AssertionError(
            f"FakeContext 에서 wire 호출 발생: {key} — service_script 에 등록하거나 "
            "primitive 를 쓰세요"
        )


class FakeContext(TaskContext):
    def __init__(
        self,
        *,
        robots: list[str] | None = None,
        specs: dict[str, TaskRobotSpec] | None = None,
        detect_script: dict[str, list[list[OrientedDetection]]] | None = None,
        reachable_script: list[int] | None = None,
        fail_labels: set[str] | None = None,
        service_script: dict[str, list[Any]] | None = None,
    ) -> None:
        super().__init__(_NoWireRuntime(), specs or {})  # type: ignore[arg-type]
        self._allowed = set(robots) if robots else None
        self._detect_script = {k: list(v) for k, v in (detect_script or {}).items()}
        self._reachable_script = list(reachable_script or [])
        self._fail_labels = fail_labels or set()
        self._service_script = {k: list(v) for k, v in (service_script or {}).items()}
        # 기록 — 시나리오 검증용
        self.call_log: list[dict[str, Any]] = []  # {kind, robot_id, label, ...args}
        self.result_log: list[tuple[str, Any]] = []  # ctx.record / 값 primitive
        self.aborted = False
        self._kind_counts: dict[str, int] = {}

    # ─── 검증 helper ─────────────────────────────────────────────────

    def kinds(self) -> list[str]:
        return [c["kind"] for c in self.call_log]

    def labels(self) -> list[str]:
        return [c["label"] for c in self.call_log]

    def calls(self, kind: str) -> list[dict[str, Any]]:
        return [c for c in self.call_log if c["kind"] == kind]

    # ─── TaskContext override ────────────────────────────────────────

    def robot(self, robot_id: str) -> RobotHandle:
        if self._allowed is not None and robot_id not in self._allowed:
            return super().robot(robot_id)  # 실물과 동일한 TaskError 경로
        handle = self._handles.get(robot_id)
        if handle is None:
            handle = FakeRobotHandle(self, robot_id, self._robots.get(robot_id))
            self._handles[robot_id] = handle
        return handle

    async def wait(self, sec: float, *, label: str = "") -> None:
        self._log("wait", "", label, sec=sec)  # sleep 없음 — 테스트 즉시 진행

    def record(self, label: str, value: BaseModel | list | None) -> None:
        _, dumped = dump_value(value)
        self.result_log.append((label, dumped))

    async def call(
        self, key: str, req: BaseModel, res_cls: type[TRes], *, timeout: float = 5.0
    ) -> TRes:
        return self._pop_service(key)

    async def on_abort(self) -> None:
        self.aborted = True

    # ─── internal (FakeRobotHandle 이 사용) ──────────────────────────

    def _log(self, kind: str, robot_id: str, label: str, **args: Any) -> str:
        n = self._kind_counts.get(kind, 0) + 1
        self._kind_counts[kind] = n
        label = label or f"{kind}#{n}"
        self.call_log.append({"kind": kind, "robot_id": robot_id, "label": label, **args})
        return label

    def _check_fail(self, kind: str, label: str) -> None:
        if label in self._fail_labels:
            if kind == "gripper":
                raise GripperFailed(label)
            raise MotionRejected(kind, f"fail_labels 스크립트 ({label})")

    def _pop_detect(self, prompt: str) -> list[OrientedDetection]:
        seq = self._detect_script.get(prompt)
        if not seq:
            raise AssertionError(
                f"detect_script 소진/미등록: prompt='{prompt}' — 테스트 스크립트 확인"
            )
        return seq.pop(0)

    def _pop_reachable(self) -> int:
        idx = self._reachable_script.pop(0) if self._reachable_script else 0
        if idx < 0:
            raise NoReachableGrasp("reachable_script -1")
        return idx

    def _pop_service(self, key: str) -> Any:
        seq = self._service_script.get(str(key))
        if not seq:
            raise AssertionError(f"service_script 미등록: {key}")
        return seq.pop(0)


class FakeRobotHandle(RobotHandle):
    """RobotHandle 과 동일 시그니처 — 기록 + 스크립트 반환 (wire 없음)."""

    def __init__(
        self, ctx: FakeContext, robot_id: str, spec: TaskRobotSpec | None
    ) -> None:
        super().__init__(ctx, robot_id, spec)
        self._fake = ctx

    async def detect_oriented(
        self, prompt: str, *, top_k: int = 5, label: str = ""
    ) -> list[OrientedDetection]:
        used = self._fake._log(
            "detect_oriented", self.robot_id, label, prompt=prompt, top_k=top_k
        )
        cands = self._fake._pop_detect(prompt)
        _, dumped = dump_value(list(cands))
        self._fake.result_log.append((used, dumped))
        return cands

    async def select_reachable(
        self, groups: list[list[TcpPose]], *, label: str = ""
    ) -> int:
        used = self._fake._log(
            "select_reachable", self.robot_id, label, groups=len(groups)
        )
        self._fake._check_fail("select_reachable", used)
        return self._fake._pop_reachable()

    async def move_j_pose(
        self, position: Vec3, quaternion: Quat | None = None, *, label: str = ""
    ) -> None:
        used = self._fake._log(
            "move_j_pose", self.robot_id, label,
            position=position, quaternion=quaternion,
        )
        self._fake._mark_moved(self.robot_id)
        self._fake._check_fail("move_j_pose", used)

    async def move_l(
        self, position: Vec3, quaternion: Quat | None = None, *, label: str = ""
    ) -> None:
        used = self._fake._log(
            "move_l", self.robot_id, label,
            position=position, quaternion=quaternion,
        )
        self._fake._mark_moved(self.robot_id)
        self._fake._check_fail("move_l", used)

    async def gripper(
        self, action: Literal["open", "close"], *, label: str = ""
    ) -> None:
        used = self._fake._log(
            "gripper", self.robot_id, label or f"gripper_{action}", action=action
        )
        self._fake._check_fail("gripper", used)

    async def call(
        self, key: str, req: BaseModel, res_cls: type[TRes], *, timeout: float = 5.0
    ) -> TRes:
        self._fake._log("call", self.robot_id, "", key=str(key))
        return self._fake._pop_service(key)
