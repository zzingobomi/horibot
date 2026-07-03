"""DetectorModule DETECT 단위테스트 — fake runtime 으로 camera/calibration/motion
canned 응답 주입. 실 모듈/하드웨어/모델 없이 DETECT 배선 + 투영 통합 검증 (회사).

의미 있는 검증: mock 검출 bbox + 합성 depth/intrinsic/hand_eye/TCP → base 좌표가
손계산과 일치 + 캘 없을 때 not-found.
"""

from __future__ import annotations

from datetime import UTC, datetime

import numpy as np

from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    HandEyeResultData,
    HandEyeResultRecord,
    IntrinsicResultData,
    IntrinsicResultRecord,
)
from modules.camera.contract import (
    Camera,
    CameraDecodedFrame,
    CameraDepthDecodedFrame,
)
from modules.detector.backend import MockDetectorBackend
from modules.detector.contract import DetectRequest
from modules.detector.module import DetectorModule
from modules.motion.contract import Motion, TcpState

_ROBOT = "so101_6dof_0"


class _FakeRuntime:
    """key(str) → canned 응답. await runtime.call 만족 (async)."""

    def __init__(self, responses: dict):
        self._responses = responses

    def publish(self, wire_key, event):  # noqa: ANN001, D102
        pass

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):
        return self._responses[str(key)]


def _bundle() -> CalibrationBundle:
    now = datetime.now(UTC)
    return CalibrationBundle(
        robot_id=_ROBOT,
        intrinsic=IntrinsicResultRecord(
            run_id=1,
            robot_id=_ROBOT,
            created_at=now,
            result_data=IntrinsicResultData(
                camera_matrix=[[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
                dist_coeffs=[[0.0, 0.0, 0.0, 0.0, 0.0]],
            ),
        ),
        hand_eye=HandEyeResultRecord(
            run_id=1,
            robot_id=_ROBOT,
            created_at=now,
            result_data=HandEyeResultData(
                R_cam2gripper=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                t_cam2gripper=[[0.0], [0.0], [0.0]],
                method="test",
            ),
        ),
    )


async def test_detect_returns_base_position():
    w, h = 640, 480
    color = CameraDecodedFrame(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        ndarray_bytes=np.zeros((h, w, 3), np.uint8).tobytes(), width=w, height=h,
    )
    # mock bbox = 중앙 20% → x∈[256,384], y∈[192,288]. 그 영역 depth raw=300.
    depth_arr = np.zeros((h, w), np.uint16)
    depth_arr[192:288, 256:384] = 300
    depth = CameraDepthDecodedFrame(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        depth_bytes=depth_arr.tobytes(), width=w, height=h, depth_scale=0.001,
    )
    tcp = TcpState(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        position=(0.0, 0.0, 0.5), quaternion=(0.0, 0.0, 0.0, 1.0),
        joint_names=["joint1"], joints=[0.0],
    )
    rt = _FakeRuntime({
        str(Calibration.Service.SNAPSHOT_BUNDLE): _bundle(),
        str(Camera.Service.DECODED_SNAPSHOT): color,
        str(Camera.Service.DEPTH_DECODED_SNAPSHOT): depth,
        str(Motion.Service.TCP_SNAPSHOT): tcp,
    })
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())

    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    assert res.found, res.message
    assert res.detection is not None
    # bbox center=(320,240)=(cx,cy) → X=Y=0, Z_cam=0.3; hand_eye=I; base t=[0,0,0.5]
    # → base=[0,0,0.8]
    assert np.allclose(res.detection.position, [0.0, 0.0, 0.8], atol=1e-6), (
        res.detection.position
    )
    assert res.detection.prompt == "cube"
    assert res.detection.score == 0.99


async def test_detect_without_calibration_not_found():
    rt = _FakeRuntime(
        {str(Calibration.Service.SNAPSHOT_BUNDLE): CalibrationBundle(robot_id=_ROBOT)}
    )
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    assert not res.found
    assert "캘" in res.message


async def test_detect_empty_prompt_not_found():
    rt = _FakeRuntime({})
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="  "))
    assert not res.found
