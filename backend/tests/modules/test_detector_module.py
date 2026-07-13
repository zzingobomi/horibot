"""DetectorModule DETECT 단위테스트 — fake runtime 으로 camera/calibration/motion
canned 응답 주입. 실 모듈/하드웨어/모델 없이 DETECT 배선 + 투영 통합 검증 (회사).

의미 있는 검증: mock 검출 bbox + 합성 depth/intrinsic/hand_eye/TCP → base 좌표가
손계산과 일치 + 캘 없을 때 not-found.
"""

from __future__ import annotations

import math
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
from modules.detector.drivers.protocol import RawDetection
from modules.detector.contract import DetectRequest, OrientedDetection
from modules.detector.module import DetectorModule
from modules.motion.contract import Motion, TcpState

_ROBOT = "so101_6dof_0"


class _FakeRuntime:
    """key(str) → canned 응답. await runtime.call 만족 (async). publish 캡처."""

    def __init__(self, responses: dict):
        self._responses = responses
        self.published: list[tuple[str, object]] = []

    def publish(self, wire_key, event):  # noqa: ANN001, D102
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):
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


def _frame_runtime(w: int = 640, h: int = 480) -> _FakeRuntime:
    """캘/color/depth/tcp canned 응답 rt. detect / detect_oriented 공유.

    mock 후보 2개: 중앙(320,240) 20% → x∈[256,384],y∈[192,288] depth 300 → z 0.3 →
    base [0,0,0.8]. 우상(448,144) → x∈[384,512],y∈[96,192] depth 500 → z 0.5 →
    base [0.1067,-0.08,1.0] (base_z 더 높음 = 후보 구분 근거).
    """
    color = CameraDecodedFrame(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        ndarray_bytes=np.zeros((h, w, 3), np.uint8).tobytes(), width=w, height=h,
    )
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
    return _FakeRuntime({
        str(Calibration.Service.SNAPSHOT_BUNDLE): _bundle(),
        str(Camera.Service.DECODED_SNAPSHOT): color,
        str(Camera.Service.DEPTH_DECODED_SNAPSHOT): depth,
        str(Motion.Service.TCP_SNAPSHOT): tcp,
    })


async def test_detect_returns_topk_base_positions():
    w, h = 640, 480
    rt = _frame_runtime(w, h)
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())

    res = await mod.detect(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    assert res.found, res.message
    # Top-K: 후보 2개 + score 내림차순 (§17.5 ①)
    assert len(res.candidates) == 2
    assert res.candidates[0].score >= res.candidates[1].score

    best = res.candidates[0]
    # 중앙 균일 윗면 → 윗면 픽셀 centroid ≈ (cx,cy), Z_cam=0.3; hand_eye=I; base t=[0,0,0.5]
    # → base ≈ [0,0,0.8]. (centroid 는 픽셀 그리드 중심이라 bbox 중심과 0.5px≈0.25mm 차 —
    #  균일 블록에선 무해. object_top_center_base = 윗면 실측 중심.)
    assert np.allclose(best.position, [0.0, 0.0, 0.8], atol=1e-3), best.position
    assert best.prompt == "cube"
    assert best.score == 0.95
    # base_z/height 필드 전파 + 불변식 (ring floor_z/height 수치는 projection unit test).
    assert isinstance(best.base_z, float) and isinstance(best.height, float)
    assert best.height == max(0.0, best.position[2] - best.base_z)

    # 2등 후보 = 우상 (score 낮음) — Top-K 누적 확인
    second = res.candidates[1]
    assert second.score == 0.60
    assert abs(second.position[2] - 1.0) < 1e-6, second.position

    # bbox_2d 전파 — frontend 카메라 오버레이 소비 (중앙 후보 = x∈[256,384])
    assert best.bbox_2d is not None
    assert best.bbox_2d[0] == 256.0 and best.bbox_2d[2] == 384.0

    # DETECT 마다 DetectionsUpdate publish (v1 DETECTOR_STATE 계승 — 오버레이 wire)
    dets = [e for k, e in rt.published if k.endswith("/detections")]
    assert len(dets) == 1
    upd = dets[0]
    assert upd.robot_id == _ROBOT  # type: ignore[attr-defined]
    assert upd.image_width == w and upd.image_height == h  # type: ignore[attr-defined]
    assert len(upd.candidates) == 2  # type: ignore[attr-defined]


async def test_detect_oriented_returns_obb_candidates():
    # 같은 파이프라인 + mask(=bbox 채운 사각형)→base 점군→minAreaRect OBB.
    rt = _frame_runtime()
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect_oriented(DetectRequest(robot_id=_ROBOT, prompt="cube"))
    assert res.found, res.message
    assert len(res.candidates) == 2  # detect 와 동일 Top-K
    c = res.candidates[0]
    # detect 와 base 위치/score 동일 (공통 _detect_candidates)
    assert np.allclose(c.position, [0.0, 0.0, 0.8], atol=1e-3), c.position
    assert c.score == 0.95
    # OBB 필드 — grasp yaw(rad) + footprint(long≥short, m)
    assert isinstance(c.grasp_yaw, float)
    assert len(c.footprint) == 2 and c.footprint[0] >= c.footprint[1]
    # 중앙 bbox 128×96px @ z0.3, fx600 → footprint ≈ (0.064, 0.048)m, 축정렬 yaw≈0
    assert abs(c.footprint[0] - 0.064) < 3e-3, c.footprint
    assert abs(c.footprint[1] - 0.048) < 3e-3, c.footprint
    assert abs(c.grasp_yaw) < math.radians(5), c.grasp_yaw
    # 오버레이: obb_2d = 코너 4개(base OBB → 픽셀 reproject). 축정렬 mask 라 다시
    # bbox 로 돌아와야 (round-trip) — 중앙 bbox x∈[256,384], y∈[192,288].
    assert c.obb_2d is not None and len(c.obb_2d) == 4, c.obb_2d
    xs = [p[0] for p in c.obb_2d]
    ys = [p[1] for p in c.obb_2d]
    assert abs(min(xs) - 256) < 3 and abs(max(xs) - 384) < 3, c.obb_2d
    assert abs(min(ys) - 192) < 3 and abs(max(ys) - 288) < 3, c.obb_2d
    # mask_contour = SAM mask 윤곽 (mock = 채운 bbox → 사각형 윤곽 폴리곤)
    assert c.mask_contour is not None and len(c.mask_contour) >= 4, c.mask_contour
    # DraftModel = extra allow (shape 미확정 마커 — 굳으면 StrictModel 로 교체)
    assert OrientedDetection.model_config.get("extra") == "allow"

    # 오버레이 스냅샷 publish (DETECTIONS_ORIENTED) — 카메라 패널 소비
    ups = [e for k, e in rt.published if k.endswith("/detections_oriented")]
    assert len(ups) == 1
    assert len(ups[0].candidates) == 2  # type: ignore[attr-defined]


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
        ) -> list[RawDetection]:
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
