"""프론트엔드 ↔ 백엔드 공개 API contract

여기 등재된 토픽 / 서비스만 프론트엔드에 공개.
미등재 = internal (백엔드 노드간 호출만, 프론트는 호출 불가).

흐름:
1. bridge 가 `PUBLIC_TOPICS` 읽어 `_ALWAYS_SUBSCRIBE` 자동 — 프론트로 mirror.
2. bridge `custom_openapi()` 가 `PUBLIC_TOPICS` / `PUBLIC_SERVICES` 를 OpenAPI
   `x-contract` vendor extension 으로 `/openapi.json` 에 인라인.
3. frontend `pnpm gen:types` 가 같은 JSON 의 `x-contract` 읽어 `contract.ts`
   (Topic / ServiceKey 상수 + TopicPayloadMap / ServiceMap 타입) emit.

새 frontend-facing service 추가:
  PUBLIC_SERVICES[Service.NEW_SERVICE] = (NewReq, NewRes)  ← 1줄 추가
  → backend 재시작, frontend `pnpm gen:types`.

새 internal service 추가:
  본 파일 건드림 X. service 정의만 (topic_map / messages / handler) — 프론트
  자동으로 호출 불가 (key 자체가 contract.ts 에 없음).
"""

from __future__ import annotations

from typing import TypeAlias

from pydantic import BaseModel

from core.transport.messages import (
    calibration as _calibration,
    camera as _camera,
    detector as _detector,
    motion as _motion,
    motor as _motor,
    reconstruction as _reconstruction,
    scene3d as _scene3d,
    storage as _storage,
    system as _system,
    task as _task,
)
from core.transport.messages.base import EmptyData
from core.transport.topic_map import Service, Topic


# ─── 타입 ──────────────────────────────────────────────────────────────

# typed schema 가 있는 자리는 BaseModel subclass.
# None = free-form dict (typed_messaging.md 면제 자리 — 동적 dict).
TopicPayload: TypeAlias = type[BaseModel] | None
ServicePair: TypeAlias = tuple[type[BaseModel] | None, type[BaseModel] | None]


# ─── Public topics ─────────────────────────────────────────────────────
# 프론트엔드가 구독하는 토픽. bridge `_ALWAYS_SUBSCRIBE` 자동 생성 source.

PUBLIC_TOPICS: dict[str, TopicPayload] = {
    # System
    Topic.SYSTEM_HEARTBEAT: _system.Heartbeat,
    Topic.SYSTEM_LOG: _system.LogMessage,
    # Motor
    Topic.MOTOR_STATE_JOINT: _motor.MotorJointState,
    Topic.MOTOR_CMD_JOINT: _motor.MotorCmd,
    # Camera
    Topic.CAMERA_STATE_STATUS: _camera.CameraStatus,
    # Motion
    Topic.MOTION_STATE_TRAJ: _motion.MotionTrajState,
    # Detector / Perception
    Topic.DETECTOR_STATE: _detector.DetectorState,
    Topic.PERCEPTION_GROUNDED_STATE: _detector.GroundedDetectionResult,
    # Scene3D — RGBD primitive sensor 상태
    Topic.SCENE3D_STATE: _scene3d.Scene3DState,
    # ── free-form 면제 자리 (typed_messaging.md §마이그레이션 사유) ──
    # 동적 dict 페이로드. 프론트는 unknown / any 로 받음.
    Topic.TASK_STATE: None,
    Topic.TASK_TREE: None,
    Topic.TASK_STEP_RESULT: None,
    Topic.CALIB_HANDEYE_PREVIEW: None,
    Topic.CALIB_HANDEYE_SIGMA: _calibration.HandeyeSigmaState,
    # 추천 자세 + saturate state — 동적 dict (recommendations list 자체 자리, sigma_history list)
    Topic.CALIB_HANDEYE_RECOMMENDATIONS: None,
    Topic.CALIB_HANDEYE_SATURATE: None,
    Topic.CALIB_HANDEYE_OBSERVABILITY: _calibration.HandeyeObservabilityState,
    Topic.CALIB_HANDEYE_PARAM_OBSERVABILITY: (
        _calibration.HandeyeParamObservabilityState
    ),
    # Storage — ACTIVATE 마다 1회. frontend list 패널이 활성 row 갱신 트리거.
    Topic.STORAGE_CALIBRATION_INVALIDATED: _storage.CalibrationInvalidated,
    # Reconstruction — BuildReconstruction step 자리 progress bar 자리.
    Topic.RECONSTRUCTION_PROGRESS: _reconstruction.ReconstructionProgress,
    # ── Internal (의도적 미등재) ──
    # Topic.CAMERA_DEPTH_FRAME    — pointcloud_node 만 구독 (binary)
}


# ─── Public binary topics ──────────────────────────────────────────────
# Schema 없는 raw bytes. bridge 가 binary WS frame 으로 중계.
# 프론트는 자체 decoder 사용 (frontend/src/api/bridge.ts).

PUBLIC_BINARY_TOPICS: set[str] = {
    Topic.SCENE3D_STREAM,
    # ── CAMERA_STREAM_RAW 는 MJPEG `/camera/stream` HTTP 별도 라우트 ──
}


# ─── Frontend → backend topic publishes ────────────────────────────────
# 방향이 *한 쪽* — frontend 가 publish 만 하는 토픽 (50Hz servo stream 자리).
# - frontend 는 typed publish 가능해야 → contract.ts emit 필요 → PUBLIC 분류
# - bridge `_ALWAYS_SUBSCRIBE` 는 *backend publish 를 frontend mirror* 자리라
#   여기 토픽 자기 자신을 구독하면 echo loop. 분리 set 으로 제외.
# 새 자리 추가 시: 본 dict 1줄. bridge 자동 forward (subscribe 자리 X).

PUBLIC_PUBLISH_TOPICS: dict[str, TopicPayload] = {
    # Jog streams — frontend / gamepad 50Hz publish, backend motion_node 직접 subscribe.
    # docs/motion_taxonomy.md §Jog — LeRobot delta-pose 패턴 (backend latch + 적분).
    Topic.MOTION_JOG_TCP_STREAM: _motion.JogTcpReq,
    Topic.MOTION_JOG_J_STREAM: _motion.JogJReq,
}


# ─── Public services ───────────────────────────────────────────────────
# 프론트엔드가 호출하는 서비스. (req_model, res_model) — None=free-form.

PUBLIC_SERVICES: dict[str, ServicePair] = {
    # ─ Motor (frontend-facing)
    Service.MOTOR_ENABLE: (_motor.MotorEnableReq, _motor.MotorEnableRes),
    Service.MOTOR_REBOOT: (_motor.MotorRebootReq, EmptyData),
    Service.MOTOR_SET_PROFILE: (_motor.MotorSetProfileReq, EmptyData),
    Service.MOTOR_GET_CONFIG: (EmptyData, _motor.MotorGetConfigRes),
    # ─ Motion
    Service.MOTION_GET_TCP: (EmptyData, _motion.MotionTcpPose),
    Service.MOTION_MOVE_J: (_motion.MoveJReq, EmptyData),
    Service.MOTION_MOVE_L: (_motion.MoveLReq, EmptyData),
    Service.MOTION_MOVE_C: (_motion.MoveCReq, EmptyData),
    Service.MOTION_MOVE_P: (_motion.MovePReq, EmptyData),
    Service.MOTION_SERVO_TCP: (_motion.ServoTcpReq, EmptyData),
    Service.MOTION_SERVO_J: (_motion.ServoJReq, EmptyData),
    Service.MOTION_JOG_TCP: (_motion.JogTcpReq, EmptyData),
    Service.MOTION_JOG_J: (_motion.JogJReq, EmptyData),
    Service.MOTION_STOP: (EmptyData, EmptyData),
    # ─ Perception (Grounding DINO)
    Service.PERCEPTION_GROUNDED_DETECT: (
        _detector.GroundedDetectReq,
        _detector.GroundedDetectionResult,
    ),
    # ─ Calibration
    Service.CALIB_INTRINSIC_CAPTURE: (EmptyData, _calibration.IntrinsicCaptureRes),
    Service.CALIB_INTRINSIC_START: (EmptyData, EmptyData),
    Service.CALIB_INTRINSIC_SAVE: (EmptyData, _calibration.IntrinsicSaveRes),
    Service.CALIB_HANDEYE_CAPTURE: (EmptyData, _calibration.HandeyeCaptureRes),
    Service.CALIB_HANDEYE_RESET: (EmptyData, _calibration.HandeyeResetRes),
    Service.CALIB_HANDEYE_COMMIT: (EmptyData, _calibration.HandeyeCommitRes),
    Service.CALIB_HANDEYE_LIST_POSES: (EmptyData, _calibration.HandeyeListPosesRes),
    Service.CALIB_HANDEYE_PREVIEW_ENABLE: (
        _calibration.HandeyePreviewEnableReq,
        _calibration.HandeyePreviewEnableRes,
    ),
    Service.CALIB_HANDEYE_BEGIN_REFINEMENT: (
        _calibration.BeginRefinementReq,
        _calibration.BeginRefinementRes,
    ),
    # Draft run flow — 사용자 [캘 시작] / [되돌리기]. storage_layer.md §13.
    Service.CALIB_HANDEYE_START: (EmptyData, _calibration.HandeyeStartRes),
    Service.CALIB_HANDEYE_UNDO_LAST_CAPTURE: (
        EmptyData,
        _calibration.HandeyeUndoLastCaptureRes,
    ),
    # ─ Storage (Phase 1 — 캘 4 service)
    Service.STORAGE_GET_ACTIVE_CALIBRATION: (
        _storage.StorageGetActiveReq,
        _storage.StorageGetActiveRes,
    ),
    Service.STORAGE_LIST_CALIBRATIONS: (
        _storage.StorageListReq,
        _storage.StorageListRes,
    ),
    Service.STORAGE_LIST_CALIBRATION_RUNS: (
        _storage.StorageListRunsReq,
        _storage.StorageListRunsRes,
    ),
    Service.STORAGE_COMMIT_CALIBRATION: (
        _storage.StorageCommitReq,
        _storage.StorageCommitRes,
    ),
    Service.STORAGE_ACTIVATE_CALIBRATION: (
        _storage.StorageActivateReq,
        _storage.StorageActivateRes,
    ),
    # ─ Storage Phase 2 — scan workflow (frontend TasksPage scan task 자리)
    Service.STORAGE_NEW_SCAN_SESSION: (
        _storage.StorageNewScanSessionReq,
        _storage.StorageNewScanSessionRes,
    ),
    Service.STORAGE_LIST_SCAN_SESSIONS: (
        _storage.StorageListScanSessionsReq,
        _storage.StorageListScanSessionsRes,
    ),
    Service.STORAGE_DELETE_SCAN_SESSION: (
        _storage.StorageDeleteScanSessionReq,
        EmptyData,
    ),
    Service.STORAGE_LIST_SCANS: (
        _storage.StorageListScansReq,
        _storage.StorageListScansRes,
    ),
    Service.STORAGE_DELETE_SCAN: (
        _storage.StorageDeleteScanReq,
        EmptyData,
    ),
    Service.STORAGE_LIST_RECONSTRUCTIONS: (
        _storage.StorageListReconstructionsReq,
        _storage.StorageListReconstructionsRes,
    ),
    Service.STORAGE_DELETE_RECONSTRUCTION: (
        _storage.StorageDeleteReconstructionReq,
        EmptyData,
    ),
    # ── Internal (의도적 미등재 — backend 노드 간 호출만, blob 자리 wire 큰
    # 자리라 frontend 노출 X)
    # Service.STORAGE_PUT_SCAN              — Scene3DNode + Storage 자리
    # Service.STORAGE_PUT_RECONSTRUCTION    — ReconstructionNode 자리
    # Service.STORAGE_GET_BLOB              — ReconstructionNode 자리
    # ─ Task
    Service.TASK_STOP: (EmptyData, EmptyData),
    Service.TASK_PAUSE: (EmptyData, EmptyData),
    Service.TASK_RESUME: (EmptyData, EmptyData),
    Service.TASK_STEP: (EmptyData, EmptyData),
    Service.TASK_RUN_TO: (_task.TaskStepIdReq, EmptyData),
    Service.TASK_TOGGLE_BREAKPOINT: (_task.TaskStepIdReq, EmptyData),
    # ─ Scene3D — RGBD primitive (snapshot + stream)
    Service.SCENE3D_SNAPSHOT: (
        _scene3d.Scene3DSnapshotReq,
        _scene3d.Scene3DSnapshotRes,
    ),
    Service.SCENE3D_SET_STREAM: (
        _scene3d.Scene3DSetStreamReq,
        _scene3d.Scene3DSetStreamRes,
    ),
    # ── free-form 면제 자리 (typed_messaging.md §마이그레이션 사유) ──
    Service.TASK_RUN: (None, None),
    Service.TASK_STATUS: (EmptyData, None),
    Service.TASK_PREVIEW: (None, None),
    Service.CALIB_HANDEYE_COMPUTE: (None, None),
    Service.CALIB_HANDEYE_THRESHOLDS: (EmptyData, None),
    # ── Internal (의도적 미등재) ──
    # Service.MOTOR_GRIPPER          — task / gamepad 만 호출
    # Service.MOTOR_SET_PROFILE_ALL  — motion_node 만 호출
    # Service.CAMERA_SET_DEPTH_STREAM — detector / pointcloud 만 호출
    # Service.DETECT_SERVICE         — 내부 click-to-detect (현재 frontend 미사용)
    # Service.SYSTEM_NODE_STATUS     — 미구현
}


# ─── Codegen helper ────────────────────────────────────────────────────


def all_referenced_models() -> set[type[BaseModel]]:
    """contract 가 참조하는 모든 Pydantic 모델 집합.

    bridge `OpenApiSchemaRegistry` 자동 생성에 사용 — 본 집합의 모델만
    `/openapi.json` `components/schemas` 에 등재.
    """
    models: set[type[BaseModel]] = set()
    for payload in PUBLIC_TOPICS.values():
        if payload is not None:
            models.add(payload)
    for payload in PUBLIC_PUBLISH_TOPICS.values():
        if payload is not None:
            models.add(payload)
    for req, res in PUBLIC_SERVICES.values():
        if req is not None:
            models.add(req)
        if res is not None:
            models.add(res)
    return models


def _attr_name_by_value(cls: type) -> dict[str, str]:
    """Topic / Service 클래스의 attribute name reverse-lookup.

    `Topic.MOTOR_STATE_JOINT = "horibot/{robot_id}/motor/state/joint"` 같은
    형태에서 `{"horibot/{robot_id}/motor/state/joint": "MOTOR_STATE_JOINT"}`
    추출. robot-scoped template 도 그대로 key — frontend 가 expand.
    """
    return {
        v: k
        for k, v in vars(cls).items()
        if isinstance(v, str) and not k.startswith("_")
    }


def to_x_contract() -> dict[str, object]:
    """OpenAPI `x-contract` vendor extension 페이로드 빌드.

    형식:
        {
          "topics": {
            "<topic_key>": {"name": "<ATTR_NAME>",
                            "payload": "<schema_name>" | null},
            ...
          },
          "binary_topics": [{"key": "<topic_key>", "name": "<ATTR_NAME>"}, ...],
          "services": {
            "<service_key>": {"name": "<ATTR_NAME>",
                              "req": "<name>" | null,
                              "res": "<name>" | null},
            ...
          }
        }

    `name` 은 `topic_map.py` 의 attribute name — frontend `contract.ts` 가
    이 이름으로 `Topic.<NAME>` / `ServiceKey.<NAME>` 상수 emit (backend ↔
    frontend 식별자 통일).

    frontend `gen-contract.mjs` 가 본 형태 파싱.
    """
    topic_names = _attr_name_by_value(Topic)
    service_names = _attr_name_by_value(Service)
    # frontend contract.ts 는 PUBLIC_TOPICS (backend→frontend) 와
    # PUBLIC_PUBLISH_TOPICS (frontend→backend) 둘 다 같은 TopicPayloadMap 에
    # 박힘. 다만 _ALWAYS_SUBSCRIBE 는 backend→frontend 자리만.
    all_topics = {**PUBLIC_TOPICS, **PUBLIC_PUBLISH_TOPICS}
    return {
        "topics": {
            key: {
                "name": topic_names[key],
                "payload": payload.__name__ if payload is not None else None,
            }
            for key, payload in all_topics.items()
        },
        "binary_topics": [
            {"key": key, "name": topic_names[key]}
            for key in sorted(PUBLIC_BINARY_TOPICS)
        ],
        "services": {
            key: {
                "name": service_names[key],
                "req": req.__name__ if req is not None else None,
                "res": res.__name__ if res is not None else None,
            }
            for key, (req, res) in PUBLIC_SERVICES.items()
        },
    }
