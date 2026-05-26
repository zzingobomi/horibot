"""Pick-and-place — Grounding DINO + height 기반 grasp 정책.

흐름:
  1. detect (GroundedDetect): prompt → 객체 윗면 base xyz + height
  2. grasp_policy: height 보고 grasp_z 결정 (얇음: 윗면 누름 / 두꺼움: 옆면 중간)
  3. pre_grasp → grasp → close → lift → place → release → home

grasp_xyz는 (x, y, grasp_z) 형태로 context에 들어가므로, 후속 MoveTCPStep은
position_key="grasp_xyz" + offset=(0, 0, dz) 형태로 깊이만 변주하면 됨.
"""

from modules.kinematics.solver import Position3
from modules.task.step_types import (
    GraspPolicyStep,
    GripperStep,
    GroundedDetectStep,
    HomeStep,
    MoveTCPStep,
    Task,
    WaitStep,
)


PRE_GRASP_DZ = 0.06   # grasp 위치 위 6cm hover
LIFT_DZ = 0.08        # 파지 후 들어올리는 높이 (grasp 위치 기준)


def create_pick_and_place_task(
    prompt: str,
    place_position: Position3,
) -> Task:
    return Task(
        name="pick_and_place",
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
            GraspPolicyStep(
                input_key="object_pos",
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
            GripperStep(action="close", label="close_gripper"),
            WaitStep(duration_sec=0.5, label="grip_settle"),
            MoveTCPStep(
                position_key="grasp_xyz",
                offset=(0.0, 0.0, LIFT_DZ),
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
