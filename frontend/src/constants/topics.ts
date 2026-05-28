export const Topic = {
  // Motor
  MOTOR_STATE_JOINT: "omx/motor/state/joint",
  MOTOR_CMD_JOINT: "omx/motor/cmd/joint",

  // Camera
  CAMERA_STREAM_RAW: "omx/camera/stream/raw",
  CAMERA_STATE_STATUS: "omx/camera/state/status",

  // Motion
  MOTION_STATE_TRAJ: "omx/motion/state/trajectory",

  // Calibration
  CALIB_HANDEYE_PREVIEW: "omx/calibration/state/handeye_preview",

  // System
  SYSTEM_HEARTBEAT: "omx/system/heartbeat",
  SYSTEM_LOG: "omx/system/log",

  // Task
  TASK_STATE: "omx/task/state",
  TASK_TREE: "omx/task/tree",

  // Self-play
  SELF_PLAY_STATE: "omx/self_play/state",

  // Detector
  DETECTOR_STATE: "omx/detector/state",

  // Perception (Grounding DINO grounded_detect 결과 broadcast)
  PERCEPTION_GROUNDED_STATE: "omx/perception/state/grounded",

  // PointCloud
  POINTCLOUD_STREAM: "omx/pointcloud/stream",
  POINTCLOUD_SNAPSHOT: "omx/pointcloud/snapshot",
  POINTCLOUD_STATE: "omx/pointcloud/state",
} as const;

export const ServiceKey = {
  // Motor
  MOTOR_ENABLE: "omx/motor/srv/enable",
  MOTOR_SET_PROFILE: "omx/motor/srv/set_profile",
  MOTOR_REBOOT: "omx/motor/srv/reboot",
  MOTOR_GET_CONFIG: "omx/motor/srv/get_config",

  // Motion
  MOTION_GET_TCP: "omx/motion/srv/get_tcp",
  MOTION_MOVE_TCP: "omx/motion/srv/move_tcp",
  MOTION_MOVE_J: "omx/motion/srv/move_j",
  MOTION_MOVE_L: "omx/motion/srv/move_l",
  MOTION_MOVE_C: "omx/motion/srv/move_c",
  MOTION_MOVE_P: "omx/motion/srv/move_p",
  MOTION_STOP: "omx/motion/srv/stop",

  // Calibration
  CALIB_CAPTURE: "omx/calib/srv/capture",
  CALIB_INTRINSIC_START: "omx/calib/srv/intrinsic/start",
  CALIB_INTRINSIC_SAVE: "omx/calib/srv/intrinsic/save",
  CALIB_HANDEYE_CAPTURE: "omx/calib/srv/handeye/capture",
  CALIB_HANDEYE_RESET: "omx/calib/srv/handeye/reset",
  CALIB_HANDEYE_COMPUTE: "omx/calib/srv/handeye/compute",
  CALIB_HANDEYE_COMMIT: "omx/calib/srv/handeye/commit",
  CALIB_HANDEYE_LIST_POSES: "omx/calib/srv/handeye/list_poses",
  CALIB_HANDEYE_PREVIEW_ENABLE: "omx/calib/srv/handeye/preview_enable",
  CALIB_HANDEYE_THRESHOLDS: "omx/calib/srv/handeye/thresholds",

  // System
  SYSTEM_NODE_STATUS: "omx/system/srv/node_status",

  // Task
  TASK_RUN: "omx/task/srv/run",
  TASK_STOP: "omx/task/srv/stop",
  TASK_PAUSE: "omx/task/srv/pause",
  TASK_RESUME: "omx/task/srv/resume",
  TASK_STATUS: "omx/task/srv/status",
  TASK_STEP: "omx/task/srv/step",
  TASK_RUN_TO: "omx/task/srv/run_to",
  TASK_TOGGLE_BREAKPOINT: "omx/task/srv/toggle_breakpoint",
  TASK_PREVIEW: "omx/task/srv/preview",

  // Perception (Grounding DINO 기반 open-vocabulary detection)
  PERCEPTION_GROUNDED_DETECT: "omx/perception/srv/grounded_detect",

  // PointCloud
  POINTCLOUD_CONFIGURE: "omx/pointcloud/srv/configure",
  // capture
  POINTCLOUD_NEW_SESSION: "omx/pointcloud/srv/new_session",
  POINTCLOUD_CAPTURE: "omx/pointcloud/srv/capture",
  POINTCLOUD_LIST_SESSIONS: "omx/pointcloud/srv/list_sessions",
  POINTCLOUD_LIST_SCANS: "omx/pointcloud/srv/list_scans",
  POINTCLOUD_DELETE_SCAN: "omx/pointcloud/srv/delete_scan",
  // TSDF
  POINTCLOUD_BUILD_MESH: "omx/pointcloud/srv/build_mesh",
  POINTCLOUD_LIST_MESHES: "omx/pointcloud/srv/list_meshes",
} as const;
