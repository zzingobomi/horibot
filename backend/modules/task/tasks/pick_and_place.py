"""Pick-and-place — typed Slot DSL + ForEach/Try/BreakIf 분해 recipe.

흐름 (이전 dict-key 기반 버전과 *동등 동작*):
  1. open gripper
  2. search_pick: search pose 들 순회하며 pick 객체 detect (search_and_detect recipe)
  3. (place_object 있으면) search_place: place 객체 detect
  4. grasp_policy: pick 객체 옆면 중간 (base_z + height * 0.5)
  5. pre_grasp → grasp → close+verify → settle → lift → verify
  6. (place_object 있으면) place_policy → pre_place → place → verify → release → settle → retreat
  7. home (recipe 함수)

[ideas.md](../../../../docs/ideas.md) Step DSL 레고화 의 acceptance test —
이 task 가 lego 블록 (primitive + recipe) 조합으로 동등 동작.

핵심 변경 — string-key chaining 추방 + 매크로 분해:
    이전: SearchAndDetectStep(output_key="pick_pos") + GraspPolicyStep(input_key="pick_pos")
    지금: pick_steps, pick_slot = search_and_detect(pick_object)
          → grasp = GraspPolicy(target=pick_slot)
    search_and_detect 는 recipe — 내부적으로 ForEach + Try + BreakIf + GroundedDetect 조합.
"""

from modules.task.recipes import home, search_and_detect
from modules.task.schema import Position3
from modules.task.step import Step, Task
from modules.task.steps import (
    Gripper,
    GraspPolicy,
    MoveTCP,
    PlacePolicy,
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

    pick_steps, pick_slot = search_and_detect(
        pick_object, label=f"search_pick:{pick_object}"
    )
    grasp = GraspPolicy(target=pick_slot, label="grasp_policy")

    steps: list[Step] = [Gripper(action="open", label="open_gripper")]
    steps += pick_steps

    if place_object:
        place_steps, place_slot = search_and_detect(
            place_object, label=f"search_place:{place_object}"
        )
        steps += place_steps
    else:
        place_slot = None

    steps += [
        grasp,
        MoveTCP(
            target=grasp.out,
            offset=Position3(x=0.0, y=0.0, z=PRE_GRASP_DZ),
            label="pre_grasp",
        ),
        MoveTCP(target=grasp.out, label="grasp"),
        Gripper(action="close", verify_grasp=True, label="close_gripper"),
        Wait(duration_sec=0.5, label="grip_settle"),
        MoveTCP(
            target=grasp.out,
            offset=Position3(x=0.0, y=0.0, z=LIFT_DZ),
            label="lift",
        ),
        VerifyGrasp(label="verify_after_lift"),
    ]

    if place_slot is not None:
        place_xyz = PlacePolicy(target=place_slot, label="place_policy")
        steps += [
            place_xyz,
            MoveTCP(
                target=place_xyz.out,
                offset=Position3(x=0.0, y=0.0, z=PLACE_HOVER_DZ),
                label="pre_place",
            ),
            MoveTCP(target=place_xyz.out, label="place"),
            VerifyGrasp(label="verify_before_release"),
            Gripper(action="open", label="release"),
            Wait(duration_sec=0.3, label="release_settle"),
            MoveTCP(
                target=place_xyz.out,
                offset=Position3(x=0.0, y=0.0, z=PLACE_HOVER_DZ),
                label="post_place_retreat",
            ),
        ]

    steps.append(home())

    return Task(name="pick_and_place", description=desc, steps=steps)
