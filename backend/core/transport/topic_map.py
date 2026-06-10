class Topic:
    # ─── Motor ─────────────────────────────────────────────
    MOTOR_STATE_JOINT = "horibot/{robot_id}/motor/state/joint"
    MOTOR_CMD_JOINT = "horibot/{robot_id}/motor/cmd/joint"

    # ─── Camera ────────────────────────────────────────────
    CAMERA_STREAM_RAW = "horibot/{robot_id}/camera/stream/raw"
    CAMERA_STATE_STATUS = "horibot/{robot_id}/camera/state/status"
    CAMERA_DEPTH_FRAME = "horibot/{robot_id}/camera/stream/depth_frame"

    # ─── Motion ────────────────────────────────────────────
    MOTION_STATE_TRAJ = "horibot/{robot_id}/motion/state/trajectory"

    # ─── System ────────────────────────────────────────────
    SYSTEM_HEARTBEAT = "horibot/system/heartbeat"
    SYSTEM_LOG = "horibot/system/log"

    # ─── Calibration ───────────────────────────────────────
    CALIB_HANDEYE_PREVIEW = "horibot/{robot_id}/calib/state/handeye_preview"
    CALIB_HANDEYE_SIGMA = "horibot/{robot_id}/calib/state/handeye_sigma"
    CALIB_HANDEYE_RECOMMENDATIONS = "horibot/{robot_id}/calib/state/handeye_recommendations"
    CALIB_HANDEYE_SATURATE = "horibot/{robot_id}/calib/state/handeye_saturate"

    # ─── Task ──────────────────────────────────────────────
    TASK_STATE = "horibot/task/state"
    TASK_TREE = "horibot/task/tree"
    TASK_STEP_RESULT = "horibot/task/step_result"

    # ─── Detector ──────────────────────────────────────────
    DETECTOR_STATE = "horibot/{robot_id}/detector/state"
    PERCEPTION_GROUNDED_STATE = "horibot/{robot_id}/perception/state/grounded"

    # ─── PointCloud ────────────────────────────────────────
    POINTCLOUD_STREAM = "horibot/{robot_id}/pointcloud/stream"
    POINTCLOUD_SNAPSHOT = "horibot/{robot_id}/pointcloud/snapshot"
    POINTCLOUD_STATE = "horibot/{robot_id}/pointcloud/state"


class Service:
    # ─── Motor ─────────────────────────────────────────────
    MOTOR_ENABLE = "horibot/{robot_id}/motor/srv/enable"
    MOTOR_SET_PROFILE = "horibot/{robot_id}/motor/srv/set_profile"
    MOTOR_SET_PROFILE_ALL = "horibot/{robot_id}/motor/srv/set_profile_all"
    MOTOR_REBOOT = "horibot/{robot_id}/motor/srv/reboot"
    MOTOR_GET_CONFIG = "horibot/{robot_id}/motor/srv/get_config"
    MOTOR_GRIPPER = "horibot/{robot_id}/motor/srv/gripper"

    # ─── Camera ────────────────────────────────────────────
    CAMERA_SET_DEPTH_STREAM = "horibot/{robot_id}/camera/srv/set_depth_stream"

    # ─── Motion ────────────────────────────────────────────
    MOTION_GET_TCP = "horibot/{robot_id}/motion/srv/get_tcp"
    MOTION_MOVE_TCP = "horibot/{robot_id}/motion/srv/move_tcp"
    MOTION_MOVE_J = "horibot/{robot_id}/motion/srv/move_j"
    MOTION_MOVE_L = "horibot/{robot_id}/motion/srv/move_l"
    MOTION_MOVE_C = "horibot/{robot_id}/motion/srv/move_c"
    MOTION_MOVE_P = "horibot/{robot_id}/motion/srv/move_p"
    MOTION_STOP = "horibot/{robot_id}/motion/srv/stop"

    # ─── System ────────────────────────────────────────────
    SYSTEM_NODE_STATUS = "horibot/system/srv/node_status"

    # ─── Calibration ───────────────────────────────────────
    CALIB_INTRINSIC_START = "horibot/{robot_id}/calib/srv/intrinsic/start"
    CALIB_INTRINSIC_SAVE = "horibot/{robot_id}/calib/srv/intrinsic/save"
    CALIB_HANDEYE_CAPTURE = "horibot/{robot_id}/calib/srv/handeye/capture"
    CALIB_HANDEYE_RESET = "horibot/{robot_id}/calib/srv/handeye/reset"
    CALIB_HANDEYE_COMPUTE = "horibot/{robot_id}/calib/srv/handeye/compute"
    CALIB_HANDEYE_COMMIT = "horibot/{robot_id}/calib/srv/handeye/commit"
    CALIB_HANDEYE_LIST_POSES = "horibot/{robot_id}/calib/srv/handeye/list_poses"
    CALIB_HANDEYE_PREVIEW_ENABLE = "horibot/{robot_id}/calib/srv/handeye/preview_enable"
    CALIB_HANDEYE_THRESHOLDS = "horibot/{robot_id}/calib/srv/handeye/thresholds"
    CALIB_HANDEYE_RECOMMENDATION_FAIL = "horibot/{robot_id}/calib/srv/handeye/recommendation_fail"
    CALIB_HANDEYE_MULTI_START = "horibot/{robot_id}/calib/srv/handeye/multi_start"
    CALIB_CAPTURE = "horibot/{robot_id}/calib/srv/capture"
    CALIB_BACKUP_LIST = "horibot/{robot_id}/calib/srv/backup/list"
    CALIB_BACKUP_RESTORE = "horibot/{robot_id}/calib/srv/backup/restore"

    # ─── Task ──────────────────────────────────────────────
    TASK_RUN = "horibot/task/srv/run"
    TASK_STOP = "horibot/task/srv/stop"
    TASK_PAUSE = "horibot/task/srv/pause"
    TASK_RESUME = "horibot/task/srv/resume"
    TASK_STATUS = "horibot/task/srv/status"
    TASK_STEP = "horibot/task/srv/step"
    TASK_RUN_TO = "horibot/task/srv/run_to"
    TASK_TOGGLE_BREAKPOINT = "horibot/task/srv/toggle_breakpoint"
    TASK_PREVIEW = "horibot/task/srv/preview"

    # ─── Detector ──────────────────────────────────────────
    DETECT_SERVICE = "horibot/{robot_id}/detector/srv/detect"

    # ─── Perception ────────────────────────────────────────
    PERCEPTION_GROUNDED_DETECT = "horibot/{robot_id}/perception/srv/grounded_detect"

    # ─── PointCloud ────────────────────────────────────────
    POINTCLOUD_CONFIGURE = "horibot/{robot_id}/pointcloud/srv/configure"
    POINTCLOUD_NEW_SESSION = "horibot/{robot_id}/pointcloud/srv/new_session"
    POINTCLOUD_CAPTURE = "horibot/{robot_id}/pointcloud/srv/capture"
    POINTCLOUD_LIST_SESSIONS = "horibot/{robot_id}/pointcloud/srv/list_sessions"
    POINTCLOUD_LIST_SCANS = "horibot/{robot_id}/pointcloud/srv/list_scans"
    POINTCLOUD_DELETE_SCAN = "horibot/{robot_id}/pointcloud/srv/delete_scan"
    POINTCLOUD_BUILD_MESH = "horibot/{robot_id}/pointcloud/srv/build_mesh"
    POINTCLOUD_LIST_MESHES = "horibot/{robot_id}/pointcloud/srv/list_meshes"


def topic_for(template: str, robot_id: str) -> str:
    if "{robot_id}" not in template:
        return template
    return template.format(robot_id=robot_id)
