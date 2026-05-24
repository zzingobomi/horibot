"""자연어 prompt 기반 pick-and-place 시나리오.

pick_and_place 와 거의 같지만 detect step 만 GroundedDetectStep 으로 교체.
PLACE 좌표는 호출 측이 task 파라미터로 전달.
"""

from modules.kinematics.solver import Position3
from modules.task.step_types import (
    GripperStep,
    GroundedDetectStep,
    HomeStep,
    MoveTCPStep,
    Task,
    WaitStep,
)


PRE_GRASP_Z = 0.06  # 오브젝트 위 6cm
GRASP_Z = 0.010  # 오브젝트 위 1cm
LIFT_Z = 0.08  # 파지 후 들어올리는 높이


def create_pick_named_object_task(
    prompt: str,
    place_position: Position3,
) -> Task:
    return Task(
        name="pick_named_object",
        description=(
            f"'{prompt}' 찾아서 ({place_position[0]:.3f}, "
            f"{place_position[1]:.3f}, {place_position[2]:.3f})에 내려놓기"
        ),
        steps=[
            GripperStep(action="open", label="open_gripper"),
            GroundedDetectStep(
                prompt=prompt,
                output_key="object_pos",
                label=f"detect:{prompt}",
            ),
            MoveTCPStep(
                position_key="object_pos",
                offset=(0.0, 0.0, PRE_GRASP_Z),
                label="pre_grasp",
            ),
            MoveTCPStep(
                position_key="object_pos",
                offset=(0.0, 0.0, GRASP_Z),
                label="grasp",
            ),
            GripperStep(action="close", label="close_gripper"),
            WaitStep(duration_sec=0.5, label="grip_settle"),
            MoveTCPStep(
                position_key="object_pos",
                offset=(0.0, 0.0, LIFT_Z),
                label="lift",
            ),
            MoveTCPStep(
                position=place_position,
                label="move_to_place",
            ),
            GripperStep(action="open", label="release"),
            WaitStep(duration_sec=0.3, label="release_settle"),
            HomeStep(label="return_home"),
        ],
    )
