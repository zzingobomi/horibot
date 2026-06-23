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
    # corrected EE pose stream (sag + link_offset + joint_offset 적용).
    # MOTION_GET_TCP service 와 같은 값을 streaming 으로 push — frontend 가 자체
    # URDF FK 안 돌리고 backend SSOT 만 신뢰. motor state 와 같은 20Hz.
    MOTION_STATE_TCP = "horibot/{robot_id}/motion/state/tcp"
    # Jog stream (frontend / gamepad 50Hz publish — motion_taxonomy.md §Jog).
    # service 50Hz RTT 회피 위해 topic publish (fire-and-forget). backend
    # JogJCommand / JogTcpCommand 가 ref latch + 적분 + IK + publish_cmd.
    MOTION_JOG_TCP_STREAM = "horibot/{robot_id}/motion/cmd/jog_tcp_stream"
    MOTION_JOG_J_STREAM = "horibot/{robot_id}/motion/cmd/jog_j_stream"

    # ─── System ────────────────────────────────────────────
    SYSTEM_HEARTBEAT = "horibot/system/heartbeat"
    SYSTEM_LOG = "horibot/system/log"

    # ─── Calibration ───────────────────────────────────────
    # capture-only 시나리오 — 실시간 G/Y/R traffic light + ChArUco overlay.
    # online BA / 추천 / observability / sigma / saturate / ba_status 자리 전부 폐기
    # (offline Python 스크립트가 분석).
    CALIB_HANDEYE_PREVIEW = "horibot/{robot_id}/calib/state/handeye_preview"

    # ─── Task ──────────────────────────────────────────────
    TASK_STATE = "horibot/task/state"
    TASK_TREE = "horibot/task/tree"
    TASK_STEP_RESULT = "horibot/task/step_result"

    # ─── Detector ──────────────────────────────────────────
    DETECTOR_STATE = "horibot/{robot_id}/detector/state"
    PERCEPTION_GROUNDED_STATE = "horibot/{robot_id}/perception/state/grounded"

    # ─── Scene3D — RGBD primitive ──────────────────────────
    SCENE3D_STREAM = "horibot/{robot_id}/scene3d/stream"
    SCENE3D_STATE = "horibot/{robot_id}/scene3d/state"

    # ─── Storage (global — robot_id 가 payload 에 포함) ────
    # 캘 INVALIDATED — ACTIVATE 마다 1회. payload=(robot_id, kind). 각 노드의
    # CalibrationCache 가 구독해 refetch 트리거. docs/storage_layer.md §7.
    STORAGE_CALIBRATION_INVALIDATED = "horibot/storage/state/calibration_invalidated"

    # ─── Reconstruction (global) ───────────────────────────
    # build 진행 중 stage / percent / message publish. ScanTask 의
    # BuildReconstruction step 자리 progress bar 자리 사용.
    RECONSTRUCTION_PROGRESS = "horibot/reconstruction/state/progress"


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
    # Trajectory-planned (단발 target → Ruckig jerk-limited profile)
    MOTION_GET_TCP = "horibot/{robot_id}/motion/srv/get_tcp"
    MOTION_MOVE_J = "horibot/{robot_id}/motion/srv/move_j"
    MOTION_MOVE_L = "horibot/{robot_id}/motion/srv/move_l"
    MOTION_MOVE_C = "horibot/{robot_id}/motion/srv/move_c"
    MOTION_MOVE_P = "horibot/{robot_id}/motion/srv/move_p"
    # Servo (외부 controller — RL/Vision — 절대 target → direct IK + publish)
    MOTION_SERVO_TCP = "horibot/{robot_id}/motion/srv/servo_tcp"
    MOTION_SERVO_J = "horibot/{robot_id}/motion/srv/servo_j"
    # Jog (human/manual velocity — frontend/gamepad → backend latch + 적분).
    # service 자리 = 자동화 tool / test 자리 단발 호출, topic stream 자리 = 50Hz 자리.
    MOTION_JOG_TCP = "horibot/{robot_id}/motion/srv/jog_tcp"
    MOTION_JOG_J = "horibot/{robot_id}/motion/srv/jog_j"
    MOTION_STOP = "horibot/{robot_id}/motion/srv/stop"

    # ─── System ────────────────────────────────────────────
    SYSTEM_NODE_STATUS = "horibot/system/srv/node_status"

    # ─── Calibration ───────────────────────────────────────
    # intrinsic — 별개 kind (5종 중 1). flow 변경 없음.
    CALIB_INTRINSIC_START = "horibot/{robot_id}/calib/srv/intrinsic/start"
    CALIB_INTRINSIC_SAVE = "horibot/{robot_id}/calib/srv/intrinsic/save"
    CALIB_INTRINSIC_CAPTURE = "horibot/{robot_id}/calib/srv/intrinsic/capture"
    # hand-eye — capture-only 시나리오. online BA / commit / 추천 / refinement 폐기.
    # offline Python 스크립트가 captures + blobs read → BA → finalize_run + activate.
    CALIB_HANDEYE_START = "horibot/{robot_id}/calib/srv/handeye/start"
    CALIB_HANDEYE_CAPTURE = "horibot/{robot_id}/calib/srv/handeye/capture"
    CALIB_HANDEYE_RESET = "horibot/{robot_id}/calib/srv/handeye/reset"
    CALIB_HANDEYE_UNDO_LAST_CAPTURE = "horibot/{robot_id}/calib/srv/handeye/undo_last_capture"
    CALIB_HANDEYE_FINALIZE = "horibot/{robot_id}/calib/srv/handeye/finalize"
    CALIB_HANDEYE_LIST_POSES = "horibot/{robot_id}/calib/srv/handeye/list_poses"
    CALIB_HANDEYE_PREVIEW_ENABLE = "horibot/{robot_id}/calib/srv/handeye/preview_enable"
    CALIB_HANDEYE_THRESHOLDS = "horibot/{robot_id}/calib/srv/handeye/thresholds"

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

    # ─── Storage (global — payload 에 robot_id 포함) ───────
    # 캘 5종 저장/조회/활성화. docs/storage_layer.md §2 architecture.
    STORAGE_GET_ACTIVE_CALIBRATION = "horibot/storage/srv/calibration/get_active"
    STORAGE_LIST_CALIBRATIONS = "horibot/storage/srv/calibration/list"
    # Run 단위 history. frontend list/ACTIVATE 패널이 사용 (storage_layer.md
    # Stage 4 §6.A — MLflow Model Registry / git history 정합).
    STORAGE_LIST_CALIBRATION_RUNS = "horibot/storage/srv/calibration/list_runs"
    STORAGE_COMMIT_CALIBRATION = "horibot/storage/srv/calibration/commit"
    STORAGE_ACTIVATE_CALIBRATION = "horibot/storage/srv/calibration/activate"

    # ─── Draft run / capture-as-you-go (사용자 [캘 시작] flow) ─────
    # [캘 시작] → NEW_CAL_RUN (in_progress run). [캡처] → APPEND_CAPTURE.
    # [되돌리기] → DELETE_LAST_CAPTURE. [리셋] → DELETE_CAL_RUN.
    # 부팅 시 GET_IN_PROGRESS_RUN 로 복원.
    # [세션 종료] → MARK_CAL_RUN_READY. 이후 BA finalize 는 calibrate_offline.py
    # 가 raw sqlite3 로 직접 commit — wire path 없음.
    STORAGE_NEW_CAL_RUN = "horibot/storage/srv/calibration/new_run"
    STORAGE_APPEND_CAPTURE = "horibot/storage/srv/calibration/append_capture"
    STORAGE_DELETE_LAST_CAPTURE = "horibot/storage/srv/calibration/delete_last_capture"
    STORAGE_GET_IN_PROGRESS_RUN = "horibot/storage/srv/calibration/get_in_progress_run"
    STORAGE_DELETE_CAL_RUN = "horibot/storage/srv/calibration/delete_run"
    STORAGE_MARK_CAL_RUN_READY = "horibot/storage/srv/calibration/mark_run_ready"
    # 임의 run_id 의 captures fetch — offline 스크립트 / 진단 도구 자리.
    STORAGE_LIST_RUN_CAPTURES = "horibot/storage/srv/calibration/list_run_captures"

    # ─── Storage Phase 2 — scan workflow ───────────────────
    # scan_sessions / scans / reconstructions. append-only blob + immutable
    # metadata row. ScanTask + ReconstructionNode 자리 caller.
    STORAGE_NEW_SCAN_SESSION = "horibot/storage/srv/scan/new_session"
    STORAGE_LIST_SCAN_SESSIONS = "horibot/storage/srv/scan/list_sessions"
    STORAGE_DELETE_SCAN_SESSION = "horibot/storage/srv/scan/delete_session"
    STORAGE_PUT_SCAN = "horibot/storage/srv/scan/put"
    STORAGE_LIST_SCANS = "horibot/storage/srv/scan/list"
    STORAGE_DELETE_SCAN = "horibot/storage/srv/scan/delete"
    STORAGE_GET_BLOB = "horibot/storage/srv/blob/get"  # generic (scan / reconstruction)
    STORAGE_PUT_RECONSTRUCTION = "horibot/storage/srv/reconstruction/put"
    STORAGE_LIST_RECONSTRUCTIONS = "horibot/storage/srv/reconstruction/list"
    STORAGE_DELETE_RECONSTRUCTION = "horibot/storage/srv/reconstruction/delete"

    # ─── Reconstruction (global — heavy compute) ───────────
    # ScanTask 의 BuildReconstruction step 자리 caller. session 안 모든 scan
    # fetch + ICP + PoseGraph + TSDF + mesh + storage put 자리.
    RECONSTRUCTION_BUILD = "horibot/reconstruction/srv/build"

    # ─── Scene3D — RGBD primitive (snapshot + stream) ──────
    SCENE3D_SNAPSHOT = "horibot/{robot_id}/scene3d/srv/snapshot"
    SCENE3D_SET_STREAM = "horibot/{robot_id}/scene3d/srv/set_stream"


def key_for(template: str, robot_id: str) -> str:
    if "{robot_id}" not in template:
        return template
    return template.format(robot_id=robot_id)
