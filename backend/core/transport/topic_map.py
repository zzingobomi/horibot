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

    # ─── Calibration ─────────────────────────────────────────
    CALIB_HANDEYE_PREVIEW = "omx/calibration/state/handeye_preview"

    # ─── Task ────────────────────────────────────────────────
    TASK_STATE = "omx/task/state"
    # Task 시작 시 1회 publish — 전체 step 트리 (frontend 가 받아서 시각화).
    # latest-wins 큐로 늦게 붙은 클라이언트도 마지막 tree 를 받음.
    TASK_TREE = "omx/task/tree"
    # 각 step 완료 시 1회 publish — {step_id, type, value(typed dataclass dict)}.
    # frontend TaskResultLayer 가 type 별 (Detection / Pose6 / Position3) 자동 렌더.
    # None 출력 (MoveTCP/Gripper/...) 도 publish — "여기까지 진행됨" 마커 역할.
    TASK_STEP_RESULT = "omx/task/step_result"

    # ─── Detector ─────────────────────────────────────────────
    DETECTOR_STATE = "omx/detector/state"

    # ─── Perception (Grounding DINO grounded_detect 결과 broadcast) ──
    # 서비스 응답과 별개로, 호출 시마다 결과를 토픽으로도 발행 →
    # frontend 카메라 feed/3D 마커가 호출자(PromptPanel, self-play, task) 와
    # 무관하게 일관 시각화.
    PERCEPTION_GROUNDED_STATE = "omx/perception/state/grounded"

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
    CALIB_HANDEYE_CAPTURE = "omx/calib/srv/handeye/capture"
    CALIB_HANDEYE_RESET = "omx/calib/srv/handeye/reset"
    CALIB_HANDEYE_COMPUTE = "omx/calib/srv/handeye/compute"
    CALIB_HANDEYE_COMMIT = "omx/calib/srv/handeye/commit"
    CALIB_HANDEYE_LIST_POSES = "omx/calib/srv/handeye/list_poses"
    CALIB_HANDEYE_PREVIEW_ENABLE = "omx/calib/srv/handeye/preview_enable"
    CALIB_HANDEYE_THRESHOLDS = "omx/calib/srv/handeye/thresholds"
    CALIB_CAPTURE = "omx/calib/srv/capture"

    # ─── System ──────────────────────────────────────────────
    SYSTEM_NODE_STATUS = "omx/system/srv/node_status"

    # ─── Task ────────────────────────────────────────────────
    TASK_RUN = "omx/task/srv/run"
    TASK_STOP = "omx/task/srv/stop"
    TASK_PAUSE = "omx/task/srv/pause"
    TASK_RESUME = "omx/task/srv/resume"  # AUTO 모드로 재개 (다음 breakpoint 까지)
    TASK_STATUS = "omx/task/srv/status"
    # 디버거 컨트롤 — PAUSED 상태에서만 의미 있음.
    TASK_STEP = "omx/task/srv/step"               # 1 step 만 실행 후 다시 PAUSE
    TASK_RUN_TO = "omx/task/srv/run_to"           # {step_id} 직전까지 진행
    TASK_TOGGLE_BREAKPOINT = "omx/task/srv/toggle_breakpoint"  # {step_id} 토글
    # 실행 없이 task tree 만 빌드 — Run 전에 breakpoint 박을 수 있도록 사전 표시.
    # 응답으로 직접 tree 반환 + TASK_TREE 토픽으로도 발행 (다른 클라이언트 동기화).
    TASK_PREVIEW = "omx/task/srv/preview"

    # ─── Detector ─────────────────────────────────────────────
    DETECT_SERVICE = "omx/detector/srv/detect"

    # ─── Perception (Grounding DINO 기반 open-vocabulary detection) ──
    # 실제 구현은 별도 세션. 일단 키만 선점.
    PERCEPTION_GROUNDED_DETECT = "omx/perception/srv/grounded_detect"

    # ─── PointCloud ───────────────────────────────────────────
    POINTCLOUD_CONFIGURE = "omx/pointcloud/srv/configure"
    # capture
    POINTCLOUD_NEW_SESSION = "omx/pointcloud/srv/new_session"
    POINTCLOUD_CAPTURE = "omx/pointcloud/srv/capture"
    POINTCLOUD_LIST_SESSIONS = "omx/pointcloud/srv/list_sessions"
    POINTCLOUD_LIST_SCANS = "omx/pointcloud/srv/list_scans"
    POINTCLOUD_DELETE_SCAN = "omx/pointcloud/srv/delete_scan"
    # TSDF
    POINTCLOUD_BUILD_MESH = "omx/pointcloud/srv/build_mesh"
    POINTCLOUD_LIST_MESHES = "omx/pointcloud/srv/list_meshes"
