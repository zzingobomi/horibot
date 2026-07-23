"""mono 평면 검출 (DETECT_PLANAR + projection.plane_points_from_pixels) 검증.

의미 있는 검증 (오피스 단위테스트 — omx_handover_prep.md §5.3 "순수 numpy·
결정적"):
  ① 평면 역투영 손계산 일치 (nadir 카메라 — 중앙/엣지 픽셀)
  ② **왜곡 round-trip**: base 평면 점 → 왜곡 투영(projectPoints) → 역투영이
     원점 복원 (undistort 생략 회귀 — 생략하면 barrel k1 에서 수 cm 틀어짐)
  ③ ray 가 평면과 안 만나는 픽셀 필터 (카메라가 평면을 등짐 → None)
  ④ detect_planar 서비스 배선 — depth 스냅샷 **없이** OBB/오버레이/발행
  ⑤ 캘 없음 → not found (침묵 진행 금지)
"""

from __future__ import annotations

import math
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import pytest

from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    HandEyeResultData,
    HandEyeResultRecord,
    IntrinsicResultData,
    IntrinsicResultRecord,
)
from modules.camera.contract import Camera, CameraDecodedFrame
from modules.detector import projection
from modules.detector.contract import DetectPlanarRequest
from modules.detector.drivers.mock import MockDetectorBackend
from modules.detector.module import DetectorModule
from modules.motion.contract import Motion, TcpState

_ROBOT = "omx_f_0"
_FX = _FY = 600.0
_CX, _CY = 320.0, 240.0

# 카메라: base (0.2, 0, 0.4) 에서 수직 하향 (optical z = −z_base).
# R_bc 열 = cam x/y/z 의 base 표현: x→+x, y→−y, z→−z (det=+1).
_R_BC = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
_C = np.array([0.2, 0.0, 0.4])
# hand_eye = identity → TCP pose 가 곧 카메라 pose. quat = 회전행렬 _R_BC
# (x 축 180° 회전) = (1, 0, 0, 0).
_TCP_QUAT = (1.0, 0.0, 0.0, 0.0)
_I3 = np.eye(3)
_Z3 = np.zeros(3)


def _rbe() -> np.ndarray:
    return _R_BC  # hand_eye=I → r_be 가 카메라 회전


# ─── ① 손계산 일치 (무왜곡 nadir) ────────────────────────────────────


def test_plane_points_nadir_hand_calc():
    pts = projection.plane_points_from_pixels(
        np.array([_CX, 620.0]), np.array([_CY, 240.0]), 0.0,
        _FX, _FY, _CX, _CY, None, _rbe(), _C, _I3, _Z3,
    )
    assert pts is not None and pts.shape == (2, 3)
    # 중앙 픽셀 → 카메라 바로 아래 (0.2, 0, 0)
    assert pts[0] == pytest.approx([0.2, 0.0, 0.0], abs=1e-12)
    # (620,240): xn=0.5 → base 방향 (0.5, 0, −1), t=0.4 → (0.4, 0, 0)
    assert pts[1] == pytest.approx([0.4, 0.0, 0.0], abs=1e-12)


def test_plane_points_y_axis_flip():
    # (320, 540): yn=0.5 → cam y=0.5 는 base −y (R_bc) → (0.2, −0.2, 0)
    pts = projection.plane_points_from_pixels(
        np.array([320.0]), np.array([540.0]), 0.0,
        _FX, _FY, _CX, _CY, None, _rbe(), _C, _I3, _Z3,
    )
    assert pts is not None
    assert pts[0] == pytest.approx([0.2, -0.2, 0.0], abs=1e-12)


# ─── ② 왜곡 round-trip ───────────────────────────────────────────────


def test_distortion_roundtrip_recovers_plane_points():
    """barrel 왜곡(k1=−0.4)으로 투영한 픽셀을 역투영 — 원점 복원. undistort
    선행을 빼면 엣지에서 cm 급으로 틀어진다 (§5.3 필수 요건 회귀)."""
    dist = np.array([[-0.4, 0.1, 0.0, 0.0, 0.0]])
    truth = np.array([
        [0.2, 0.0, 0.0],
        [0.35, 0.10, 0.0],  # 엣지 쪽 — 왜곡이 크게 무는 자리
        [0.05, -0.12, 0.0],
    ])
    px = projection.project_base_to_pixel_distorted(
        truth, _FX, _FY, _CX, _CY, dist, _rbe(), _C, _I3, _Z3
    )
    rec = projection.plane_points_from_pixels(
        px[:, 0], px[:, 1], 0.0,
        _FX, _FY, _CX, _CY, dist, _rbe(), _C, _I3, _Z3,
    )
    assert rec is not None
    # cv2.undistortPoints 는 반복 역산 — µm 급 잔차 허용 (0.1mm 면 충분)
    assert rec == pytest.approx(truth, abs=1e-4)
    # 대조: undistort 를 생략(순진한 pinhole)하면 엣지 점이 cm 급으로 틀어진다
    naive = projection.plane_points_from_pixels(
        px[:, 0], px[:, 1], 0.0,
        _FX, _FY, _CX, _CY, None, _rbe(), _C, _I3, _Z3,
    )
    assert naive is not None
    edge_err = float(np.linalg.norm(naive[1] - truth[1]))
    assert edge_err > 0.01, f"왜곡 무시 오차가 {edge_err*1000:.1f}mm 뿐이면 회귀 무의미"


# ─── ③ 평면 미교차 필터 ──────────────────────────────────────────────


def test_plane_above_camera_returns_none():
    # 카메라(z=0.4)가 아래를 보는데 평면이 위(z=1.0) — ray 는 뒤로만 만남 → None
    pts = projection.plane_points_from_pixels(
        np.array([_CX]), np.array([_CY]), 1.0,
        _FX, _FY, _CX, _CY, None, _rbe(), _C, _I3, _Z3,
    )
    assert pts is None


# ─── ④⑤ detect_planar 서비스 배선 ────────────────────────────────────


class _FakeRuntime:
    def __init__(self, responses: dict):
        self._responses = responses
        self.published: list[tuple[str, object]] = []

    def publish(self, wire_key, event):  # noqa: ANN001, D102
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):
        return self._responses[str(key)]  # depth 요청 시 KeyError = 계약 위반


@pytest.fixture(autouse=True)
def _dump_to_tmp(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    import modules.detector.module as dmod

    monkeypatch.setattr(dmod, "_DETECT_DUMP_ROOT", tmp_path / "detect")
    monkeypatch.setattr(dmod, "_DEBUG_DIR", tmp_path)


def _bundle() -> CalibrationBundle:
    now = datetime.now(UTC)
    return CalibrationBundle(
        robot_id=_ROBOT,
        intrinsic=IntrinsicResultRecord(
            run_id=1, robot_id=_ROBOT, created_at=now,
            result_data=IntrinsicResultData(
                camera_matrix=[[_FX, 0.0, _CX], [0.0, _FY, _CY], [0.0, 0.0, 1.0]],
                dist_coeffs=[[0.0, 0.0, 0.0, 0.0, 0.0]],
            ),
        ),
        hand_eye=HandEyeResultRecord(
            run_id=1, robot_id=_ROBOT, created_at=now,
            result_data=HandEyeResultData(
                R_cam2gripper=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                t_cam2gripper=[[0.0], [0.0], [0.0]],
                method="test",
            ),
        ),
    )


def _runtime(w: int = 640, h: int = 480) -> _FakeRuntime:
    """color+캘+TCP 만 — depth 스냅샷 응답 없음 (planar 는 부르면 KeyError)."""
    color = CameraDecodedFrame(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        ndarray_bytes=np.zeros((h, w, 3), np.uint8).tobytes(), width=w, height=h,
    )
    tcp = TcpState(
        robot_id=_ROBOT, seq=0, timestamp_unix=0.0,
        position=(0.2, 0.0, 0.4), quaternion=_TCP_QUAT,
        joint_names=["joint1"], joints=[0.0],
    )
    return _FakeRuntime({
        str(Calibration.Service.SNAPSHOT_BUNDLE): _bundle(),
        str(Camera.Service.DECODED_SNAPSHOT): color,
        str(Motion.Service.TCP_SNAPSHOT): tcp,
    })


async def test_detect_planar_returns_plane_obb_without_depth():
    rt = _runtime()
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect_planar(
        DetectPlanarRequest(robot_id=_ROBOT, plane_z=0.0, prompt="pen")
    )
    assert res.found, res.message
    assert len(res.candidates) == 2  # mock Top-K (중앙 + 우상단)
    best = res.candidates[0]
    assert best.score == 0.95
    # 중앙 bbox 128×96px, 높이 0.4 위 카메라 → 발자국 128/600·0.4=0.0853 ×
    # 96/600·0.4=0.064 (base). 중심 = 카메라 바로 아래 (0.2, 0, plane_z)
    assert best.position == pytest.approx((0.2, 0.0, 0.0), abs=2e-3)
    assert best.base_z == 0.0
    assert best.height == 0.0  # mono 정직 표기 — 높이는 모른다
    assert abs(best.footprint[0] - 0.0853) < 3e-3, best.footprint
    assert abs(best.footprint[1] - 0.064) < 3e-3, best.footprint
    assert abs(best.grasp_yaw) < math.radians(5)
    assert best.points  # 평면 점군 동봉 (소비자 폭 측정/디버그)
    assert all(abs(p[2]) < 1e-9 for p in best.points)  # 전부 평면 위
    # 오버레이 — obb_2d 가 bbox 로 round-trip (무왜곡, 중앙 x∈[256,384] y∈[192,288])
    assert best.obb_2d is not None and len(best.obb_2d) == 4
    xs = [p[0] for p in best.obb_2d]
    ys = [p[1] for p in best.obb_2d]
    # 하한은 정확, 상한은 stride 서브샘플(_PLANAR_MAX_PIXELS)로 마지막 몇 px
    # 가 빠질 수 있음 — 5px 허용
    assert abs(min(xs) - 256) < 3 and abs(max(xs) - 384) < 5, best.obb_2d
    assert abs(min(ys) - 192) < 3 and abs(max(ys) - 288) < 5, best.obb_2d
    # DETECTIONS_ORIENTED 스트림 발행 (카메라 패널 오버레이 채널 공유)
    ups = [e for k, e in rt.published if k.endswith("/detections_oriented")]
    assert len(ups) == 1
    await mod.stop()  # 백그라운드 덤프 드레인


async def test_detect_planar_without_calibration_not_found():
    rt = _FakeRuntime({
        str(Calibration.Service.SNAPSHOT_BUNDLE): CalibrationBundle(
            robot_id=_ROBOT
        )
    })
    mod = DetectorModule(runtime=rt, backend=MockDetectorBackend())
    res = await mod.detect_planar(
        DetectPlanarRequest(robot_id=_ROBOT, plane_z=0.0, prompt="pen")
    )
    assert not res.found
    assert "캘" in res.message


async def test_detect_planar_empty_prompt_not_found():
    mod = DetectorModule(runtime=_FakeRuntime({}), backend=MockDetectorBackend())
    res = await mod.detect_planar(
        DetectPlanarRequest(robot_id=_ROBOT, plane_z=0.0, prompt="  ")
    )
    assert not res.found
