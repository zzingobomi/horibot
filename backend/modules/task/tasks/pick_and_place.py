"""Pick-and-place — typed Slot DSL 로 조립.

흐름 (이전 dict-key 기반 버전과 *동등 동작*):
  1. open gripper
  2. search_pick: search pose 들 순회하며 pick 객체 detect
  3. (place_object 있으면) search_place: place 객체 detect
  4. grasp_policy: pick 객체 옆면 중간 (base_z + height * 0.5)
  5. pre_grasp → grasp → close+verify → settle → lift → verify
  6. (place_object 있으면) place_policy → pre_place → place → verify → release → settle → retreat
  7. home

[ideas.md](../../../../docs/ideas.md) Step DSL 레고화 entry 의 acceptance test —
이 task 가 새 lego 블록 조합으로 동등 동작.

핵심 차이점 — string key chaining 추방:
    이전: SearchAndDetectStep(output_key="pick_pos"), GraspPolicyStep(input_key="pick_pos", ...)
    지금: pick = SearchAndDetect(...); GraspPolicy(target=pick.out, ...)
"""

from modules.task.schema import Position3
from modules.task.step import Step, Task
from modules.task.steps import (
    Gripper,
    GraspPolicy,
    Home,
    MoveTCP,
    PlacePolicy,
    SearchAndDetect,
    VerifyGrasp,
    Wait,
)


PRE_GRASP_DZ = 0.06   # grasp 위치 위 6cm hover
LIFT_DZ = 0.08        # 파지 후 들어올리는 높이 (grasp 위치 기준)
PLACE_HOVER_DZ = 0.05  # place 위치 위 5cm hover


def create_pick_and_place_task(
    pick_object: str,
    place_object: str | None = None,
) -> Task:
    desc = (
        f"'{pick_object}' 집어서 '{place_object}' 에 두기"
        if place_object
        else f"'{pick_object}' 집기"
    )

    pick = SearchAndDetect(prompt=pick_object, label=f"search_pick:{pick_object}")
    place = (
        SearchAndDetect(prompt=place_object, label=f"search_place:{place_object}")
        if place_object
        else None
    )
    grasp = GraspPolicy(target=pick.out, label="grasp_policy")

    steps: list[Step] = [
        Gripper(action="open", label="open_gripper"),
        pick,
    ]
    if place is not None:
        steps.append(place)

    steps += [
        grasp,
        MoveTCP(
            target=grasp.out,
            offset=Position3(0.0, 0.0, PRE_GRASP_DZ),
            label="pre_grasp",
        ),
        MoveTCP(target=grasp.out, label="grasp"),
        Gripper(action="close", verify_grasp=True, label="close_gripper"),
        Wait(duration_sec=0.5, label="grip_settle"),
        MoveTCP(
            target=grasp.out,
            offset=Position3(0.0, 0.0, LIFT_DZ),
            label="lift",
        ),
        VerifyGrasp(label="verify_after_lift"),
    ]

    if place is not None:
        place_xyz = PlacePolicy(target=place.out, label="place_policy")
        steps += [
            place_xyz,
            MoveTCP(
                target=place_xyz.out,
                offset=Position3(0.0, 0.0, PLACE_HOVER_DZ),
                label="pre_place",
            ),
            MoveTCP(target=place_xyz.out, label="place"),
            VerifyGrasp(label="verify_before_release"),
            Gripper(action="open", label="release"),
            Wait(duration_sec=0.3, label="release_settle"),
            MoveTCP(
                target=place_xyz.out,
                offset=Position3(0.0, 0.0, PLACE_HOVER_DZ),
                label="post_place_retreat",
            ),
        ]

    steps.append(Home(label="return_home"))

    return Task(name="pick_and_place", description=desc, steps=steps)
