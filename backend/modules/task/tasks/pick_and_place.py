"""Pick-and-place — Grounding DINO + 옆면 그립 정책.

흐름:
  1. detect (GroundedDetect): prompt → 객체 윗면 base xyz + height
  2. grasp_policy: 항상 옆면 중간 (base_z + height * 0.5)
  3. pre_grasp → grasp → close(+verify) → lift → place → release → home

grasp_xyz는 (x, y, grasp_z) 형태로 context에 들어가므로, 후속 MoveTCPStep은
position_key="grasp_xyz" + offset=(0, 0, dz) 형태로 깊이만 변주하면 됨.

close step 은 verify_grasp=True — Present_Position 으로 빈손 검증, 빈손이면
task 실패로 끊김 (그 상태로 place 까지 가서 success 라고 뜨는 거 방지).
"""

from modules.kinematics.solver import Position3
from modules.task.step_types import (
    GraspPolicyStep,
    GripperStep,
    GroundedDetectStep,
    HomeStep,
    MoveTCPStep,
    Task,
    VerifyGraspStep,
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
            GripperStep(action="close", verify_grasp=True, label="close_gripper"),
            WaitStep(duration_sec=0.5, label="grip_settle"),
            MoveTCPStep(
                position_key="grasp_xyz",
                offset=(0.0, 0.0, LIFT_DZ),
                label="lift",
            ),
            VerifyGraspStep(label="verify_after_lift"),
            MoveTCPStep(
                position=place_position,
                label="move_to_place",
            ),
            VerifyGraspStep(label="verify_before_release"),
            GripperStep(action="open", label="release"),
            WaitStep(duration_sec=0.3, label="release_settle"),
            HomeStep(label="return_home"),
        ],
    )
