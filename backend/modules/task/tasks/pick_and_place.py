"""Pick-and-place — Grounding DINO + search pose 순회 + 옆면 그립 정책.

흐름:
  1. search pose 들 순회하며 pick / place 객체 둘 다 detect (SearchAndDetect)
  2. grasp_policy: pick 객체 옆면 중간 (base_z + height * 0.5)
  3. place_policy: place 객체 윗면 + drop_clearance (공중에서 안 떨구게)
  4. pre_grasp → grasp → close(+verify) → lift → place 위치 → release → home

prompt 형식: "X를 집어서 Y에 둬" 같은 한국어 자연어. task_node 가 split.

place_object=None 이면 단순 pick 동작 (집고 그 자리 위로 lift + home, drop 없음).

close step 은 verify_grasp=True — Present_Position 으로 빈손 검증, 빈손이면
task 실패로 끊김.
"""

from modules.task.step_types import (
    GraspPolicyStep,
    GripperStep,
    HomeStep,
    MoveTCPStep,
    PlacePolicyStep,
    SearchAndDetectStep,
    Step,
    Task,
    VerifyGraspStep,
    WaitStep,
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

    steps: list[Step] = [
        GripperStep(action="open", label="open_gripper"),
        SearchAndDetectStep(
            prompt=pick_object,
            output_key="pick_pos",
            label=f"search_pick:{pick_object}",
        ),
    ]
    if place_object:
        steps.append(
            SearchAndDetectStep(
                prompt=place_object,
                output_key="place_pos",
                label=f"search_place:{place_object}",
            )
        )

    steps += [
        GraspPolicyStep(
            input_key="pick_pos",
            output_key="grasp_xyz",
            label="grasp_policy",
        ),
        MoveTCPStep(
            position_key="grasp_xyz",
            offset=(0.0, 0.0, PRE_GRASP_DZ),
            label="pre_grasp",
        ),
        MoveTCPStep(
            position_key="grasp_xyz",
            offset=(0.0, 0.0, 0.0),
            label="grasp",
        ),
        GripperStep(action="close", verify_grasp=True, label="close_gripper"),
        WaitStep(duration_sec=0.5, label="grip_settle"),
        MoveTCPStep(
            position_key="grasp_xyz",
            offset=(0.0, 0.0, LIFT_DZ),
            label="lift",
        ),
        VerifyGraspStep(label="verify_after_lift"),
    ]

    if place_object:
        steps += [
            PlacePolicyStep(
                input_key="place_pos",
                output_key="place_xyz",
                label="place_policy",
            ),
            # place 위 hover (충돌 피해 진입)
            MoveTCPStep(
                position_key="place_xyz",
                offset=(0.0, 0.0, PLACE_HOVER_DZ),
                label="pre_place",
            ),
            MoveTCPStep(
                position_key="place_xyz",
                offset=(0.0, 0.0, 0.0),
                label="place",
            ),
            VerifyGraspStep(label="verify_before_release"),
            GripperStep(action="open", label="release"),
            WaitStep(duration_sec=0.3, label="release_settle"),
            # 떨어진 큐브 위에서 안전 retreat
            MoveTCPStep(
                position_key="place_xyz",
                offset=(0.0, 0.0, PLACE_HOVER_DZ),
                label="post_place_retreat",
            ),
        ]

    steps.append(HomeStep(label="return_home"))

    return Task(name="pick_and_place", description=desc, steps=steps)
