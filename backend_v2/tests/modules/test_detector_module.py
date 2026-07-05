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
from modules.detector.drivers.mock import MockDetectorBackend
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


async def test_detect_returns_topk_base_positions():
    w, h = 640, 480
    color = CameraDecodedFrame(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        ndarray_bytes=np.zeros((h, w, 3), np.uint8).tobytes(), width=w, height=h,
    )
    # mock 후보 2개: 중앙(320,240) 20% → x∈[256,384],y∈[192,288] depth 300 → z 0.3 →
    # base [0,0,0.8]. 우상(448,144) → x∈[384,512],y∈[96,192] depth 500 → z 0.5 →
    # base [0.1067,-0.08,1.0] (base_z 더 높음 = 후보 구분 근거).
    depth_arr = np.zeros((h, w), np.uint16)
    depth_arr[192:288, 256:384] = 300
    depth_arr[96:192, 384:512] = 500
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
    # Top-K: 후보 2개 + score 내림차순 (§17.5 ①)
    assert len(res.candidates) == 2
    assert res.candidates[0].score >= res.candidates[1].score

    best = res.candidates[0]
    # 중앙 bbox center=(320,240)=(cx,cy) → X=Y=0, Z_cam=0.3; hand_eye=I; base t=[0,0,0.5]
    assert np.allclose(best.position, [0.0, 0.0, 0.8], atol=1e-6), best.position
    assert best.prompt == "cube"
    assert best.score == 0.95
    # base_z/height 필드 전파 + 불변식 (ring floor_z/height 수치는 projection unit test).
    assert isinstance(best.base_z, float) and isinstance(best.height, float)
    assert best.height == max(0.0, best.position[2] - best.base_z)

    # 2등 후보 = 우상 (score 낮음) — Top-K 누적 확인
    second = res.candidates[1]
    assert second.score == 0.60
    assert abs(second.position[2] - 1.0) < 1e-6, second.position


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


async def test_start_triggers_backend_preload():
    """start() 가 백그라운드로 backend.preload() 를 호출 — 첫 detect 지연 제거 배선.
    (start 에서 preload 를 빼면 이 assert 가 깨짐 = 의미 있는 회귀.)"""

    class _RecordingBackend:
        def __init__(self) -> None:
            self.preloaded = False

        def detect(
            self, image_bgr: np.ndarray, prompt: str, top_k: int
        ) -> list[tuple[tuple[float, float, float, float], float]]:
            return []

        def preload(self) -> None:
            self.preloaded = True

    backend = _RecordingBackend()
    mod = DetectorModule(runtime=_FakeRuntime({}), backend=backend)
    await mod.start()
    assert mod._preload_task is not None
    await mod._preload_task  # 백그라운드 preload 완료 대기
    assert backend.preloaded
    await mod.stop()
