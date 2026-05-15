class Topic:
    # ─── Motor ───────────────────────────────────────────────
    MOTOR_STATE_JOINT = "omx/motor/state/joint"
    MOTOR_CMD_JOINT = "omx/motor/cmd/joint"

    # ─── Camera ──────────────────────────────────────────────
    CAMERA_STREAM_RAW = "omx/camera/stream/raw"
    CAMERA_STATE_STATUS = "omx/camera/state/status"
    CAMERA_DEPTH_FRAME = "omx/camera/stream/depth_frame"

    # ─── System ──────────────────────────────────────────────
    SYSTEM_HEARTBEAT = "omx/system/heartbeat"
    SYSTEM_LOG = "omx/system/log"

    # ─── Motion ───────────────────────────────────────────────
    MOTION_STATE_TRAJ = "omx/motion/state/trajectory"

    # ─── Task ────────────────────────────────────────────────
    TASK_STATE = "omx/task/state"

    # ─── Detector ─────────────────────────────────────────────
    DETECTOR_STATE = "omx/detector/state"

    # ─── PointCloud ───────────────────────────────────────────
    POINTCLOUD_STREAM = "omx/pointcloud/stream"
    POINTCLOUD_SNAPSHOT = "omx/pointcloud/snapshot"
    POINTCLOUD_STATE = "omx/pointcloud/state"


class Service:
    # ─── Motor ───────────────────────────────────────────────
    MOTOR_ENABLE = "omx/motor/srv/enable"
    MOTOR_SET_PROFILE = "omx/motor/srv/set_profile"
    MOTOR_SET_PROFILE_ALL = "omx/motor/srv/set_profile_all"
    MOTOR_REBOOT = "omx/motor/srv/reboot"
    MOTOR_GET_CONFIG = "omx/motor/srv/get_config"
    MOTOR_GRIPPER = "omx/motor/srv/gripper"

    # ─── Camera ──────────────────────────────────────────────
    CAMERA_SET_DEPTH_STREAM = "omx/camera/srv/set_depth_stream"
    CAMERA_CAPTURE_DEPTH_FRAMES = "omx/camera/srv/capture_depth_frames"

    # ─── Motion ───────────────────────────────────────────────
    MOTION_GET_TCP = "omx/motion/srv/get_tcp"
    MOTION_MOVE_TCP = "omx/motion/srv/move_tcp"
    MOTION_MOVE_J = "omx/motion/srv/move_j"
    MOTION_MOVE_L = "omx/motion/srv/move_l"
    MOTION_MOVE_C = "omx/motion/srv/move_c"
    MOTION_MOVE_P = "omx/motion/srv/move_p"
    MOTION_STOP = "omx/motion/srv/stop"

    # ─── Calibration ─────────────────────────────────────────
    CALIB_INTRINSIC_START = "omx/calib/srv/intrinsic/start"
    CALIB_INTRINSIC_SAVE = "omx/calib/srv/intrinsic/save"
    CALIB_HANDEYE_START = "omx/calib/srv/handeye/start"
    CALIB_HANDEYE_SAVE = "omx/calib/srv/handeye/save"
    CALIB_CAPTURE = "omx/calib/srv/capture"

    # ─── System ──────────────────────────────────────────────
    SYSTEM_NODE_STATUS = "omx/system/srv/node_status"

    # ─── Task ────────────────────────────────────────────────
    TASK_RUN = "omx/task/srv/run"
    TASK_STOP = "omx/task/srv/stop"
    TASK_PAUSE = "omx/task/srv/pause"
    TASK_RESUME = "omx/task/srv/resume"
    TASK_STATUS = "omx/task/srv/status"

    # ─── Detector ─────────────────────────────────────────────
    DETECT_SERVICE = "omx/detector/srv/detect"

    # ─── PointCloud ───────────────────────────────────────────
    POINTCLOUD_CONFIGURE = "omx/pointcloud/srv/configure"
    POINTCLOUD_CAPTURE = "omx/pointcloud/srv/capture"
    POINTCLOUD_NEW_SESSION = "omx/pointcloud/srv/new_session"
    POINTCLOUD_LIST_SCANS = "omx/pointcloud/srv/list_scans"
    POINTCLOUD_LOAD_SCAN = "omx/pointcloud/srv/load_scan"
    POINTCLOUD_CLEAR_SNAPSHOT = "omx/pointcloud/srv/clear_snapshot"
