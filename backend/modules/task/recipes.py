"""Recipe 함수 — primitive step 의 흔한 조합 단축형.

step **클래스** 가 아니라 **함수**. 함수가 step 인스턴스 (또는 list + Slot)
반환. primitive 박스를 깨끗하게 유지하면서 자주 쓰는 패턴 공유.

- 단일 step 반환 (예: `home()` → `MoveJByName`) — task.steps 에 그대로 박음
- list + Slot 반환 (예: `search_and_detect(prompt)` → `(list[Step], Slot[Detection])`)
  — caller 가 list 를 task.steps 에 spread + Slot 을 다음 step 의 입력에 사용

ideas.md "Step DSL 레고화" 의 "매크로 → recipe 함수" 정신. SearchAndDetect 같은
매크로 step 을 이리로 분해.
"""

from __future__ import annotations

from core.robot_poses import list_pose_names
from modules.task.schema import Detection, Slot
from modules.task.step import Step
from modules.task.steps import (
    BreakIf,
    ForEach,
    GroundedDetect,
    MoveJByName,
    Try,
    Wait,
)


def home(*, label: str = "return_home") -> MoveJByName:
    """home 자세로 복귀 — `MoveJByName("home")` 의 단축형."""
    return MoveJByName(pose_name="home", label=label)


def search_and_detect(
    prompt: str,
    *,
    pose_prefix: str = "search_",
    settle_sec: float = 0.5,
    label: str = "",
) -> tuple[list[Step], Slot[Detection]]:
    """search pose 순회 + GroundedDetect + 첫 성공 시 break.

    BT 의 Selector node 와 동등한 의미를 ForEach + Try + BreakIf 로 분해 표현.
    이전 `SearchAndDetectStep` 매크로의 lego 화 결과.

    Returns:
        (steps, detection_slot) — caller 는 steps 를 task.steps 에 spread, 후속
        step (GraspPolicy 등) 에 detection_slot 을 박음.

    동작:
        - robot_poses.yaml 의 `pose_prefix`* 자세들 순회 (lexical sort)
        - 각 자세에서 MoveJ → settle → Try(GroundedDetect)
          - GroundedDetect 실패 시 Try 가 None 으로 변환 → 다음 자세 continue
          - 성공 시 BreakIf 가 break → loop 종료
        - 마지막 자세까지 다 실패하면 detection_slot 이 None → 후속 step 이
          Detection 기대 시 TypeError (의도적 fail 위치 명시)

    Example:
        steps, pick_slot = search_and_detect("white cube")
        task_steps += steps
        task_steps.append(GraspPolicy(target=pick_slot))
    """
    pose_names = list_pose_names(pose_prefix)
    if not pose_names:
        raise RuntimeError(
            f"search_and_detect: '{pose_prefix}' prefix pose 없음 "
            "(robot_poses.yaml 의 search_* 등록 필요)"
        )

    detect = GroundedDetect(prompt=prompt, label=f"detect:{prompt}")

    loop = ForEach.over(
        pose_names,
        lambda pose: [
            MoveJByName(pose_name=pose, label="move_search_pose"),
            Wait(duration_sec=settle_sec, label="search_settle"),
            Try(child=detect, label="try_detect"),
            BreakIf(condition=detect.out, label="break_on_detect"),
        ],
        label=label or f"search_and_detect:{prompt}",
    )
    return [loop], detect.out
