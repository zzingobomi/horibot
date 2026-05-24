"""Self-play pick task factory.

prompt / max_attempts / gripper_setup 받아 `SelfPlayStep` 1개를 가진 Task 반환.
grasp z 는 detector 가 자동 추정한 height 보고 runner 가 정책 결정.
gripper_setup 은 객체별 override (frontend 의 객체 type preset → raw 값).
§ docs/self_play_pick.md 결정 로그 #2 참조.
"""

from pathlib import Path

from core.gripper_setup import GripperSetup
from modules.task.step_types import SelfPlayStep, Task

# Project root 의 robot/logs/self_play 를 가리킴 (cwd 무관). backend/ 에서
# 실행하든 root 에서 실행하든 항상 동일 위치 — bin_offsets.json 세션 누적
# 보장.
DEFAULT_LOG_DIR = str(
    Path(__file__).parents[4] / "robot" / "logs" / "self_play"
)


def create_self_play_pick_task(
    prompt: str,
    max_attempts: int = 100,
    log_dir: str | None = None,
    gripper_setup: GripperSetup | None = None,
) -> Task:
    return Task(
        name="self_play_pick",
        description=f"Self-play pick prompt='{prompt}' max_attempts={max_attempts}",
        steps=[
            SelfPlayStep(
                prompt=prompt,
                max_attempts=max_attempts,
                log_dir=log_dir or DEFAULT_LOG_DIR,
                gripper_setup=gripper_setup,
                label="self_play",
            ),
        ],
    )
