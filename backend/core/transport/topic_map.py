"""Zenoh 토픽 / 서비스 키 SSOT.

multi_robot_phase2_frontend.md §1 결정문:
- 프로젝트 prefix: `horibot/`
- robot-scoped 키: `horibot/{robot_id}/<domain>/...` (template string)
- global 키: `horibot/<domain>/...` (task / system)

caller 는 robot-scoped template 을 `.format(robot_id=...)` 으로 expand.
BaseNode.r(template) 헬퍼가 self.robot_id 자동 채움.

api_contract.py 의 `_attr_name_by_value(Topic)` / `to_x_contract()` 는 string
attribute 만 본다 — 함수형 아닌 template string 유지로 codegen 흐름 그대로.
frontend `contract.ts` 는 이 template 들이 그대로 string 으로 emit, 콜사이트
에서 `topicFor(template, robotId)` 헬퍼로 expand.
"""


class Topic:
    # ─── Motor (robot-scoped) ──────────────────────────────
    MOTOR_STATE_JOINT = "horibot/{robot_id}/motor/state/joint"
    MOTOR_CMD_JOINT = "horibot/{robot_id}/motor/cmd/joint"

    # ─── Camera (robot-scoped — wrist 마운트) ──────────────
    CAMERA_STREAM_RAW = "horibot/{robot_id}/camera/stream/raw"
    CAMERA_STATE_STATUS = "horibot/{robot_id}/camera/state/status"
    CAMERA_DEPTH_FRAME = "horibot/{robot_id}/camera/stream/depth_frame"

    # ─── System (global) ───────────────────────────────────
    SYSTEM_HEARTBEAT = "horibot/system/heartbeat"
    SYSTEM_LOG = "horibot/system/log"

    # ─── Motion (robot-scoped) ─────────────────────────────
    MOTION_STATE_TRAJ = "horibot/{robot_id}/motion/state/trajectory"

    # ─── Calibration (robot-scoped) ────────────────────────
    CALIB_HANDEYE_PREVIEW = "horibot/{robot_id}/calib/state/handeye_preview"

    # ─── Task (global) ─────────────────────────────────────
    # task 가 robot 을 포함 — task 자체는 robot-scoped 아님.
    TASK_STATE = "horibot/task/state"
    TASK_TREE = "horibot/task/tree"
    TASK_STEP_RESULT = "horibot/task/step_result"

    # ─── Detector (robot-scoped — 한 robot+camera 쌍에 종속) ──
    DETECTOR_STATE = "horibot/{robot_id}/detector/state"
    PERCEPTION_GROUNDED_STATE = "horibot/{robot_id}/perception/state/grounded"

    # ─── PointCloud (robot-scoped) ─────────────────────────
    POINTCLOUD_STREAM = "horibot/{robot_id}/pointcloud/stream"
    POINTCLOUD_SNAPSHOT = "horibot/{robot_id}/pointcloud/snapshot"
    POINTCLOUD_STATE = "horibot/{robot_id}/pointcloud/state"


class Service:
    # ─── Motor (robot-scoped) ──────────────────────────────
    MOTOR_ENABLE = "horibot/{robot_id}/motor/srv/enable"
    MOTOR_SET_PROFILE = "horibot/{robot_id}/motor/srv/set_profile"
    MOTOR_SET_PROFILE_ALL = "horibot/{robot_id}/motor/srv/set_profile_all"
    MOTOR_REBOOT = "horibot/{robot_id}/motor/srv/reboot"
    MOTOR_GET_CONFIG = "horibot/{robot_id}/motor/srv/get_config"
    MOTOR_GRIPPER = "horibot/{robot_id}/motor/srv/gripper"

    # ─── Camera (robot-scoped) ─────────────────────────────
    CAMERA_SET_DEPTH_STREAM = "horibot/{robot_id}/camera/srv/set_depth_stream"

    # ─── Motion (robot-scoped) ─────────────────────────────
    MOTION_GET_TCP = "horibot/{robot_id}/motion/srv/get_tcp"
    MOTION_MOVE_TCP = "horibot/{robot_id}/motion/srv/move_tcp"
    MOTION_MOVE_J = "horibot/{robot_id}/motion/srv/move_j"
    MOTION_MOVE_L = "horibot/{robot_id}/motion/srv/move_l"
    MOTION_MOVE_C = "horibot/{robot_id}/motion/srv/move_c"
    MOTION_MOVE_P = "horibot/{robot_id}/motion/srv/move_p"
    MOTION_STOP = "horibot/{robot_id}/motion/srv/stop"

    # ─── Calibration (robot-scoped) ────────────────────────
    CALIB_INTRINSIC_START = "horibot/{robot_id}/calib/srv/intrinsic/start"
    CALIB_INTRINSIC_SAVE = "horibot/{robot_id}/calib/srv/intrinsic/save"
    CALIB_HANDEYE_CAPTURE = "horibot/{robot_id}/calib/srv/handeye/capture"
    CALIB_HANDEYE_RESET = "horibot/{robot_id}/calib/srv/handeye/reset"
    CALIB_HANDEYE_COMPUTE = "horibot/{robot_id}/calib/srv/handeye/compute"
    CALIB_HANDEYE_COMMIT = "horibot/{robot_id}/calib/srv/handeye/commit"
    CALIB_HANDEYE_LIST_POSES = "horibot/{robot_id}/calib/srv/handeye/list_poses"
    CALIB_HANDEYE_PREVIEW_ENABLE = "horibot/{robot_id}/calib/srv/handeye/preview_enable"
    CALIB_HANDEYE_THRESHOLDS = "horibot/{robot_id}/calib/srv/handeye/thresholds"
    CALIB_CAPTURE = "horibot/{robot_id}/calib/srv/capture"

    # ─── System (global) ───────────────────────────────────
    SYSTEM_NODE_STATUS = "horibot/system/srv/node_status"

    # ─── Task (global) ─────────────────────────────────────
    TASK_RUN = "horibot/task/srv/run"
    TASK_STOP = "horibot/task/srv/stop"
    TASK_PAUSE = "horibot/task/srv/pause"
    TASK_RESUME = "horibot/task/srv/resume"  # AUTO 모드로 재개 (다음 breakpoint 까지)
    TASK_STATUS = "horibot/task/srv/status"
    # 디버거 컨트롤 — PAUSED 상태에서만 의미 있음.
    TASK_STEP = "horibot/task/srv/step"               # 1 step 만 실행 후 다시 PAUSE
    TASK_RUN_TO = "horibot/task/srv/run_to"           # {step_id} 직전까지 진행
    TASK_TOGGLE_BREAKPOINT = "horibot/task/srv/toggle_breakpoint"  # {step_id} 토글
    # 실행 없이 task tree 만 빌드 — Run 전에 breakpoint 박을 수 있도록 사전 표시.
    # 응답으로 직접 tree 반환 + TASK_TREE 토픽으로도 발행 (다른 클라이언트 동기화).
    TASK_PREVIEW = "horibot/task/srv/preview"

    # ─── Detector (robot-scoped) ───────────────────────────
    DETECT_SERVICE = "horibot/{robot_id}/detector/srv/detect"

    # ─── Perception (robot-scoped, Grounding DINO 기반 open-vocabulary) ──
    # 실제 구현은 별도 세션. 일단 키만 선점.
    PERCEPTION_GROUNDED_DETECT = "horibot/{robot_id}/perception/srv/grounded_detect"

    # ─── PointCloud (robot-scoped) ─────────────────────────
    POINTCLOUD_CONFIGURE = "horibot/{robot_id}/pointcloud/srv/configure"
    # capture
    POINTCLOUD_NEW_SESSION = "horibot/{robot_id}/pointcloud/srv/new_session"
    POINTCLOUD_CAPTURE = "horibot/{robot_id}/pointcloud/srv/capture"
    POINTCLOUD_LIST_SESSIONS = "horibot/{robot_id}/pointcloud/srv/list_sessions"
    POINTCLOUD_LIST_SCANS = "horibot/{robot_id}/pointcloud/srv/list_scans"
    POINTCLOUD_DELETE_SCAN = "horibot/{robot_id}/pointcloud/srv/delete_scan"
    # TSDF
    POINTCLOUD_BUILD_MESH = "horibot/{robot_id}/pointcloud/srv/build_mesh"
    POINTCLOUD_LIST_MESHES = "horibot/{robot_id}/pointcloud/srv/list_meshes"


# ─── Helpers ────────────────────────────────────────────────

_ROBOT_PLACEHOLDER = "{robot_id}"


def is_robot_scoped(template: str) -> bool:
    return _ROBOT_PLACEHOLDER in template


def topic_for(template: str, robot_id: str) -> str:
    """robot-scoped template 을 expand. global template 은 그대로 반환."""
    if _ROBOT_PLACEHOLDER not in template:
        return template
    return template.format(robot_id=robot_id)
