"""ScanTask — multi-pose RGBD scan + reconstruction.

흐름 (scene3d_decoupling.md §9):
  1. NewSession (STORAGE_NEW_SCAN_SESSION)
  2. ForEach(scan_pose):
     - MoveJByName (named pose)
     - CaptureScan (SCENE3D_SNAPSHOT + STORAGE_PUT_SCAN)
  3. BuildReconstruction (RECONSTRUCTION_BUILD → ICP+TSDF+mesh+storage put)

robot 자리: TaskNode 의 default robot 자리 — required_capabilities=["rgbd"] 자리
frontend 의 robot select dropdown 자리 filter (rgbd capability 인 robot 만).

scan_poses 자리: scene3d_decoupling.md §14.4 — robots.yaml 자리 named_poses 자리
또는 별도 scan_config 자리 추후. 본 자리 hardcoded prototype 자리 default.
"""

from modules.task.step import Step, Task
from modules.task.steps import (
    BuildReconstruction,
    CaptureScan,
    ForEach,
    MoveJByName,
    NewSession,
)


DEFAULT_SCAN_POSES = ["home"]  # 본 자리 hardcoded prototype — robot_poses.yaml 자리
                                # named_poses 자리 진입 시 robots.yaml 자체 자리 자리.


def create_scan_task(
    label: str | None = None,
    scan_poses: list[str] | None = None,
) -> Task:
    """scan workflow task — NewSession → ForEach(MoveJ + CaptureScan) → BuildReconstruction.

    label: 사용자 친화적 session label (storage 의 ScanSessionRecord.label 자리).
    scan_poses: named pose list. None 이면 DEFAULT_SCAN_POSES.
    """
    poses = list(scan_poses) if scan_poses else list(DEFAULT_SCAN_POSES)

    session_step = NewSession(session_label=label, label="new_session")
    inner = ForEach.over(
        items=poses,
        body=lambda pose_slot: [
            MoveJByName(pose_name=pose_slot, label="move_to_scan_pose"),
            CaptureScan(session=session_step.out, label="capture_scan"),
        ],
        label="for_each_scan_pose",
    )
    build = BuildReconstruction(
        session=session_step.out, label="build_reconstruction"
    )

    steps: list[Step] = [session_step, inner, build]
    desc = (
        f"scan {len(poses)}-pose + reconstruction "
        f"({label})" if label else f"scan {len(poses)}-pose + reconstruction"
    )
    return Task(name="scan", steps=steps, description=desc)
